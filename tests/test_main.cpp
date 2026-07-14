#include "atc/aisg.hpp"
#include "atc/controller.hpp"
#include "atc/domain.hpp"
#include "atc/hdlc.hpp"
#include "atc/posix_serial_transport.hpp"
#include "atc/simulated_transport.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <exception>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {

using namespace std::chrono_literals;

void require(const bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

atc::hdlc::Bytes bytes(std::initializer_list<std::uint8_t> value) { return value; }

std::vector<atc::ControllerEvent> waitFor(atc::ControllerService& controller,
                                          const atc::OperationId operation,
                                          const std::chrono::milliseconds timeout = 2s) {
    std::vector<atc::ControllerEvent> observed;
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        auto events = controller.drainEvents();
        for (auto& event : events) {
            const auto terminal = event.operation == operation &&
                                  (event.kind == atc::EventKind::operationCompleted ||
                                   event.kind == atc::EventKind::operationCancelled ||
                                   event.kind == atc::EventKind::operationFailed);
            if (terminal) {
                const auto failed = event.kind == atc::EventKind::operationFailed;
                const auto message = event.message;
                observed.push_back(std::move(event));
                require(!failed, "controller operation failed: " + message);
                return observed;
            }
            observed.push_back(std::move(event));
        }
        std::this_thread::sleep_for(2ms);
    }
    throw std::runtime_error("controller operation did not finish in time");
}

void testCrcAndCapturedFrames() {
    const std::string standard = "123456789";
    require(atc::hdlc::crc16X25(std::span(
                reinterpret_cast<const std::uint8_t*>(standard.data()), standard.size())) == 0x906E,
            "CRC-16/X-25 standard check vector failed");

    const auto snrm = atc::hdlc::encode(atc::aisg::makeSnrm(1));
    require(snrm == bytes({0x7E, 0x01, 0x93, 0x8D, 0xB0, 0x7E}),
            "SNRM did not match captured address-1 frame");

    const auto ua = atc::hdlc::decode(bytes({0x7E, 0x02, 0x73, 0xEB, 0x7D, 0x5D, 0x7E}));
    require(ua && ua.frame->address == 2 && ua.frame->control == 0x73,
            "escaped FCS in captured address-2 UA was not decoded");

    const auto getTilt = atc::hdlc::encode(atc::aisg::makeGetTiltRequest(1, 0xBA));
    require(getTilt == bytes({0x7E, 0x01, 0xBA, 0x34, 0x00, 0x00, 0xCE, 0x9C, 0x7E}),
            "GET_TILT did not match captured frame");

    const auto tiltResponse = atc::hdlc::decode(
        bytes({0x7E, 0x01, 0xDA, 0x34, 0x03, 0x00, 0x00, 0x78, 0x00, 0xFD, 0xEB, 0x7E}));
    require(static_cast<bool>(tiltResponse), "captured tilt response failed CRC validation");
    const auto tilt = atc::aisg::parseTilt(*tiltResponse.frame);
    require(tilt && *tilt == 12.0, "captured tilt value was not parsed as 12.0 degrees");

    const atc::hdlc::Frame rejectedMove{
        1, 0x30, {static_cast<std::uint8_t>(atc::aisg::Command::setTilt), 0x01, 0x00, 0x03}};
    const auto rejectedStatus = atc::aisg::parseStatus(rejectedMove, atc::aisg::Command::setTilt);
    require(!rejectedStatus.success && rejectedStatus.code == 0x03,
            "AISG return code was not read after the two-byte data length");

    const auto initial = atc::hdlc::decode(bytes({
        0x7E, 0x01, 0x52, 0x05, 0x34, 0x00, 0x00, 0x0C, 0x52, 0x43, 0x55,
        0x2D, 0x30, 0x38, 0x30, 0x39, 0x2D, 0x44, 0x56, 0x54, 0x11, 0x30,
        0x38, 0x30, 0x39, 0x52, 0x31, 0x2D, 0x48, 0x32, 0x33, 0x31, 0x32,
        0x33, 0x34, 0x35, 0x36, 0x31, 0x0D, 0x52, 0x45, 0x54, 0x5F, 0x4A,
        0x53, 0x58, 0x58, 0x5F, 0x56, 0x35, 0x2E, 0x30, 0x05, 0x34, 0x2E,
        0x34, 0x2E, 0x34, 0xFF, 0xFB, 0x7E}));
    require(static_cast<bool>(initial), "captured initial-data response failed CRC validation");
    const auto parsed = atc::aisg::parseInitialData(*initial.frame);
    require(parsed && parsed->product == "RCU-0809-DVT" && parsed->softwareVersion == "4.4.4",
            "captured initial-data response was parsed incorrectly");
}

void testStuffingAndStreaming() {
    const atc::hdlc::Frame source{7, 0x54, {0x7E, 0x7D, 0x00, 0x7E}};
    const auto encoded = atc::hdlc::encode(source);
    require(std::count(encoded.begin(), encoded.end(), atc::hdlc::escape) >= 3,
            "reserved bytes were not escaped");
    const auto decoded = atc::hdlc::decode(encoded);
    require(decoded && *decoded.frame == source, "stuffed frame did not round trip");

    atc::hdlc::StreamDecoder stream;
    const auto firstHalf = std::span(encoded).first(encoded.size() / 2);
    const auto secondHalf = std::span(encoded).subspan(encoded.size() / 2);
    require(stream.push(firstHalf).empty(), "partial stream produced a premature frame");
    const auto results = stream.push(secondHalf);
    require(results.size() == 1 && results[0] && *results[0].frame == source,
            "chunked stream did not produce original frame");

    auto corrupted = encoded;
    corrupted[2] ^= 0x01;
    const auto invalid = atc::hdlc::decode(corrupted);
    require(!invalid && invalid.error == atc::hdlc::DecodeError::checksumMismatch,
            "corrupt frame was not rejected by CRC");
}

void testDomainValidation() {
    atc::InstallationData valid;
    valid.antennaSerial = "12345678901234567";
    valid.installationDate = "071426";
    valid.installerId = "ABCDE";
    valid.bearingDegrees = 359.9;
    valid.heightMeters = 999.0;
    require(!valid.validationError(), "valid installation data was rejected");
    valid.bearingDegrees = 360.0;
    require(valid.validationError().has_value(), "invalid bearing was accepted");

    const atc::Ret ret{5.0, 0.0, 15.0, true, false};
    require(ret.acceptsTilt(0.0) && ret.acceptsTilt(15.0) && !ret.acceptsTilt(15.1),
            "RET limit validation is incorrect");
}

void testSimulatorLowLevel() {
    atc::SimulatedTransport simulator;
    simulator.open({});
    const auto request = atc::hdlc::encode(atc::aisg::makeSnrm(2));
    simulator.write(request);
    const auto responseBytes = simulator.read(20ms);
    const auto response = atc::hdlc::decode(responseBytes);
    require(response && response.frame->address == 2 && response.frame->control == 0x73,
            "simulator did not respond to SNRM with UA");
    simulator.close();
}

void testPosixSerialFailureIsSafe() {
    atc::PosixSerialTransport serial;
    atc::TransportConfig config;
    config.endpoint = "/dev/atc-device-that-does-not-exist";
    config.baudRate = 9600;
    bool rejected = false;
    try {
        serial.open(config);
    } catch (const atc::TransportError&) {
        rejected = true;
    }
    require(rejected && !serial.isOpen(),
            "failed POSIX serial open did not leave the transport closed");
}

void testControllerEndToEnd() {
    auto transport = std::make_shared<atc::SimulatedTransport>();
    atc::ControllerService controller(transport);

    waitFor(controller, controller.connect());
    require(controller.isConnected(), "controller did not connect to simulator");

    atc::ScanOptions options;
    options.firstAddress = 1;
    options.lastAddress = 4;
    options.responseTimeout = 5ms;
    const auto scanEvents = waitFor(controller, controller.scan(options));
    require(controller.devices().size() == 3, "controller scan did not find three simulated devices");
    require(std::count_if(scanEvents.begin(), scanEvents.end(), [](const auto& event) {
                return event.kind == atc::EventKind::deviceAdded;
            }) == 3,
            "scan did not emit one deviceAdded event per device");

    auto first = controller.device(1);
    require(first && first->ret() && first->ret()->electricalTiltDegrees == 12.0,
            "scan did not populate RET tilt");
    waitFor(controller, controller.moveRet(1, 9.7));
    first = controller.device(1);
    require(first && first->ret() && first->ret()->electricalTiltDegrees == 9.7,
            "RET move did not update controller snapshot");

    auto third = controller.device(3);
    require(third && third->tma() && third->tma()->gainDb == 12.0 &&
                third->tma()->mode == atc::TmaMode::normal,
            "scan did not populate TMA gain and mode");
    waitFor(controller, controller.setTmaGain(3, 13.25));
    waitFor(controller, controller.setTmaMode(3, atc::TmaMode::bypass));
    third = controller.device(3);
    require(third && third->tma() && third->tma()->gainDb == 13.25 &&
                third->tma()->mode == atc::TmaMode::bypass,
            "TMA gain/mode operations did not update controller snapshot");

    waitFor(controller, controller.refreshAlarms(2));
    auto second = controller.device(2);
    require(second && second->alarms.size() == 2 && second->status == atc::DeviceStatus::alarm,
            "controller did not parse simulated alarms");
    waitFor(controller, controller.clearAlarms(2));
    second = controller.device(2);
    require(second && second->alarms.empty() && second->status == atc::DeviceStatus::ready,
            "controller did not clear simulated alarms");

    waitFor(controller, controller.setDeviceField(
                            1, atc::aisg::Field::sectorId, {'N', 'W'}));
    first = controller.device(1);
    require(first && first->installation.sectorId == "NW",
            "setDeviceField did not update local device snapshot");

    waitFor(controller, controller.disconnect());
    require(!controller.isConnected() && controller.devices().empty(),
            "disconnect did not clear controller state");
}

void testControllerCancellation() {
    auto transport = std::make_shared<atc::SimulatedTransport>();
    atc::ControllerService controller(transport);
    waitFor(controller, controller.connect());
    atc::ScanOptions options;
    options.firstAddress = 4;
    options.lastAddress = 32;
    options.responseTimeout = 50ms;
    const auto operation = controller.scan(options);
    controller.cancel(operation);
    const auto events = waitFor(controller, operation);
    require(std::any_of(events.begin(), events.end(), [operation](const auto& event) {
                return event.operation == operation &&
                       event.kind == atc::EventKind::operationCancelled;
            }),
            "cancelled scan did not emit operationCancelled");
    waitFor(controller, controller.disconnect());
}

} // namespace

int main() {
    const std::vector<std::pair<std::string, std::function<void()>>> tests{
        {"CRC and captured frames", testCrcAndCapturedFrames},
        {"stuffing and streaming", testStuffingAndStreaming},
        {"domain validation", testDomainValidation},
        {"simulator low-level", testSimulatorLowLevel},
        {"POSIX serial safe failure", testPosixSerialFailureIsSafe},
        {"controller end-to-end", testControllerEndToEnd},
        {"controller cancellation", testControllerCancellation},
    };

    int failures = 0;
    for (const auto& [name, test] : tests) {
        try {
            test();
            std::cout << "[PASS] " << name << '\n';
        } catch (const std::exception& exception) {
            ++failures;
            std::cerr << "[FAIL] " << name << ": " << exception.what() << '\n';
        }
    }
    std::cout << tests.size() - static_cast<std::size_t>(failures) << '/' << tests.size()
              << " tests passed\n";
    return failures == 0 ? 0 : 1;
}
