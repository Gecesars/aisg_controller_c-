#include "atc/adb.hpp"
#include "atc/aisg.hpp"
#include "atc/aisg3.hpp"
#include "atc/controller.hpp"
#include "atc/domain.hpp"
#include "atc/hdlc.hpp"
#include "atc/posix_serial_transport.hpp"
#include "atc/simulated_transport.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <deque>
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

atc::aisg3::Response aisg3Response(const atc::aisg3::Command command,
                                   atc::hdlc::Bytes data,
                                   const atc::aisg3::ReturnCode code = atc::aisg3::ReturnCode::ok) {
    return {command, 1, code, std::move(data), std::nullopt, std::nullopt};
}

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

class Aisg3AdbTestTransport final : public atc::ITransport {
public:
    void open(const atc::TransportConfig& config) override {
        require(config.baudRate == 9600, "AISG v3 test transport was not opened at 9600 baud");
        open_ = true;
        assigned_ = false;
        negotiated_ = false;
        address_ = 0;
        responses_.clear();
    }

    void close() noexcept override {
        open_ = false;
        responses_.clear();
    }

    [[nodiscard]] bool isOpen() const noexcept override { return open_; }

    void write(const std::span<const std::uint8_t> encoded) override {
        if (!open_) throw atc::TransportError("AISG v3 test transport is closed");
        const auto decoded = atc::hdlc::decode(encoded);
        if (!decoded) throw atc::TransportError("test transport received an invalid HDLC frame");
        const auto& request = *decoded.frame;

        if (request.address == atc::aisg3::allStationsAddress &&
            request.control == atc::aisg3::xidControl) {
            handleXid(request);
            return;
        }
        if (assigned_ && request.address == address_ && request.control == 0x93) {
            queue({address_, 0x73, {}});
            return;
        }
        if (assigned_ && request.address == address_) {
            handleLayer7(request);
        }
    }

    [[nodiscard]] atc::hdlc::Bytes read(const std::chrono::milliseconds timeout,
                                        const std::size_t maximumBytes = 4096) override {
        if (!open_) throw atc::TransportError("AISG v3 test transport is closed");
        if (responses_.empty()) {
            std::this_thread::sleep_for(std::min(timeout, 1ms));
            return {};
        }
        auto result = std::move(responses_.front());
        responses_.pop_front();
        if (result.size() > maximumBytes) result.resize(maximumBytes);
        return result;
    }

    [[nodiscard]] std::string description() const override { return "AISG3 ADB conformance ALD"; }
    [[nodiscard]] unsigned int scans() const noexcept { return scans_; }
    [[nodiscard]] unsigned int assignments() const noexcept { return assignments_; }

private:
    static std::uint16_t readLe16(const atc::hdlc::Bytes& value, const std::size_t offset) {
        if (offset + 2 > value.size()) throw atc::TransportError("truncated Layer-7 request");
        return static_cast<std::uint16_t>(value[offset]) |
               (static_cast<std::uint16_t>(value[offset + 1]) << 8U);
    }

    static void appendLe16(atc::hdlc::Bytes& target, const std::uint16_t value) {
        target.push_back(static_cast<std::uint8_t>(value & 0xFFU));
        target.push_back(static_cast<std::uint8_t>((value >> 8U) & 0xFFU));
    }

    void queue(const atc::hdlc::Frame& response) {
        responses_.push_back(atc::hdlc::encode(response));
    }

    void handleXid(const atc::hdlc::Frame& request) {
        const auto payload = atc::aisg3::parseXid(request.information);
        if (!payload) throw atc::TransportError("malformed XID request");
        const auto assignment = std::find_if(payload->parameters.begin(), payload->parameters.end(),
            [](const auto& parameter) { return parameter.identifier == 2; });
        if (assignment == payload->parameters.end()) {
            ++scans_;
            if (assigned_) return;
            const atc::hdlc::Bytes uid(uniqueId_.begin(), uniqueId_.end());
            queue({atc::aisg3::noStationAddress, atc::aisg3::xidControl,
                   atc::aisg3::encodeXid({{{1, uid},
                                           {4, {static_cast<std::uint8_t>(atc::aisg3::AldType::sald)}},
                                           {6, {'A', 'B'}},
                                           {10, {0, 1}},
                                           {22, {3, 0, 8}},
                                           {27, {static_cast<std::uint8_t>(atc::aisg3::SubunitType::adb)}}}})});
            return;
        }
        if (assignment->value.size() != 1 || assignment->value[0] == 0 ||
            assignment->value[0] == atc::aisg3::allStationsAddress) {
            throw atc::TransportError("invalid assigned address");
        }
        ++assignments_;
        address_ = assignment->value[0];
        assigned_ = true;
        const atc::hdlc::Bytes uid(uniqueId_.begin(), uniqueId_.end());
        queue({address_, atc::aisg3::xidControl,
               atc::aisg3::encodeXid({{{1, uid},
                                       {4, {static_cast<std::uint8_t>(atc::aisg3::AldType::sald)}},
                                       {10, {0, 1}}}})});
    }

    void handleLayer7(const atc::hdlc::Frame& request) {
        if (request.information.size() < 8) throw atc::TransportError("short Layer-7 request");
        const auto command = static_cast<atc::aisg3::Command>(readLe16(request.information, 0));
        const auto sequence = readLe16(request.information, 2);
        const auto subunit = readLe16(request.information, 4);
        const auto dataLength = readLe16(request.information, 6);
        if (request.information.size() != 8U + dataLength) {
            throw atc::TransportError("Layer-7 request length mismatch");
        }

        atc::hdlc::Bytes data;
        switch (command) {
        case atc::aisg3::Command::getInformation:
            data = {7, 'A', 'L', 'D', '-', 'A', 'D', 'B',
                    6, 'C', 'T', 'R', 'L', '0', '1',
                    3, 'H', 'W', '1', 3, 'S', 'W', '1'};
            break;
        case atc::aisg3::Command::getSubunitList:
            data = {1, 0, 1, 0,
                    static_cast<std::uint8_t>(atc::aisg3::SubunitType::adb)};
            break;
        case atc::aisg3::Command::getSubunitTypeStandardVersions:
            data = {0, 0, 0, 1, 3, 1, 7};
            break;
        case atc::aisg3::Command::setSubunitTypeStandardVersion:
            if (request.information.size() != 12 || request.information[8] != 3 ||
                request.information[9] != 3 || request.information[10] != 1 ||
                request.information[11] != 7) {
                throw atc::TransportError("controller negotiated the wrong ADB version");
            }
            negotiated_ = true;
            break;
        case atc::aisg3::Command::adbGetAntennaInfo:
            if (!negotiated_ || subunit != 1) {
                throw atc::TransportError("ADB command sent before negotiation");
            }
            data = {7, 'A', 'N', 'T', '-', '9', '0', '0', 1,
                    6, 'A', 'N', 'T', '0', '0', '1', 1, 2, 1};
            break;
        case atc::aisg3::Command::adbGetAntennaInstallationInfo:
            if (!negotiated_ || subunit != 1) {
                throw atc::TransportError("ADB command sent before negotiation");
            }
            data = {2, 'S', '1', 4, 4, 'T', 'O', 'P', 'O', 4,
                    0x84, 0x03, 4, 0xFB, 0xFF, 4};
            break;
        case atc::aisg3::Command::getAlarmStatus:
            data = {0};
            break;
        case atc::aisg3::Command::clearActiveAlarms:
            break;
        default:
            throw atc::TransportError("unexpected Layer-7 command in conformance scenario");
        }

        atc::hdlc::Bytes information;
        appendLe16(information, static_cast<std::uint16_t>(command));
        appendLe16(information, sequence);
        appendLe16(information, static_cast<std::uint16_t>(atc::aisg3::ReturnCode::ok));
        appendLe16(information, static_cast<std::uint16_t>(data.size()));
        information.insert(information.end(), data.begin(), data.end());
        queue({address_, static_cast<std::uint8_t>(request.control + 0x20U),
               std::move(information)});
    }

    bool open_{};
    bool assigned_{};
    bool negotiated_{};
    std::uint8_t address_{};
    unsigned int scans_{};
    unsigned int assignments_{};
    const std::array<std::uint8_t, 19> uniqueId_{
        'A', 'B', 'A', 'D', 'B', '-', 'T', 'E', 'S', 'T', '-', '0', '0', '0', '0', '0', '0', '0', '1'};
    std::deque<atc::hdlc::Bytes> responses_;
};

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

void testAisg3XidConformance() {
    using namespace atc::aisg3;

    const auto scan = makeDeviceScanCommand();
    require(scan.address == 0xFF && scan.control == 0xBF,
            "AISG v3 scan did not use the all-station XID header");
    require(scan.information == bytes({
                0x81, 0xF0, 0x0B,
                0x01, 0x00,
                0x08, 0x00,
                0x0A, 0x00,
                0x0B, 0x00,
                0x13, 0x01, 0x01}),
            "AISG v3 device-scan payload differs from Base 11.11.2");

    std::array<std::uint8_t, 19> uid{};
    uid[0] = 'A';
    uid[1] = 'B';
    uid[17] = '0';
    uid[18] = '1';
    const atc::hdlc::Bytes uidBytes(uid.begin(), uid.end());
    const auto scanPayload = encodeXid({{{1, uidBytes},
                                         {4, {64}},
                                         {6, {'A', 'B'}},
                                         {10, {0x00, 0x01}},
                                         {22, {3, 0, 8}},
                                         {27, {3}}}});
    const auto parsedScan = parseDeviceScanResponse({0, 0xBF, scanPayload});
    require(parsedScan && parsedScan->uniqueId == uid && parsedScan->portNumber == 1 &&
                parsedScan->baseVersions == std::vector<Version>{{3, 0, 8}} &&
                parsedScan->subunitTypes == std::vector<SubunitType>{SubunitType::adb},
            "AISG v3 device-scan response was not decoded according to Base 11.11.2");

    auto invalidUid = uidBytes;
    invalidUid[4] = 'X';
    invalidUid[5] = 0;
    require(!parseDeviceScanResponse({0, 0xBF, encodeXid({{{1, invalidUid},
                                                           {4, {64}},
                                                           {6, {'A', 'B'}},
                                                           {10, {1}},
                                                           {22, {3, 0, 8}},
                                                           {27, {3}}}})}),
            "AISG v3 parser accepted non-right-aligned UniqueID padding");

    AddressAssignment assignment;
    assignment.address = 1;
    assignment.primaryId = 0x12345678;
    assignment.uniqueId = uid;
    assignment.vendorCode = {'A', 'B'};
    assignment.portNumber = 1;
    const auto assign = makeAddressAssignmentCommand(assignment);
    require(assign.address == 0xFF && assign.control == 0xBF &&
                assign.information.size() == 49 && assign.information[2] == 0x2E,
            "AISG v3 address assignment has an invalid XID group length");
    require(std::equal(assign.information.begin() + 3, assign.information.begin() + 17,
                       bytes({0x02, 0x01, 0x01,
                              0x16, 0x03, 0x03, 0x00, 0x08,
                              0x1A, 0x04, 0x12, 0x34, 0x56, 0x78}).begin()),
            "AISG v3 assignment did not encode version/PrimaryID in XID big-endian order");

    const auto assignmentResponse = encodeXid({{{1, uidBytes}, {4, {64}}, {10, {0, 1}}}});
    const auto parsedAssignment = parseAddressAssignmentResponse({1, 0xBF, assignmentResponse});
    require(parsedAssignment && parsedAssignment->address == 1 &&
                parsedAssignment->uniqueId == uid && parsedAssignment->portNumber == 1,
            "AISG v3 address-assignment response was rejected");

    auto malformed = scan.information;
    malformed[2] = 0x0C;
    require(!parseXid(malformed), "XID parser accepted an inconsistent group length");
}

void testAisg3Layer7Conformance() {
    using namespace atc::aisg3;

    require(primaryIdForNodeName("abc") == 0xA9993E36U,
            "AISG PrimaryID is not the left-most SHA-1 word of the node name");

    const auto getInformation = makeGetInformation(0x1234);
    require(encodeCommand(getInformation) ==
                bytes({0x05, 0x00, 0x34, 0x12, 0x00, 0x00, 0x00, 0x00}),
            "AISG v3 Layer-7 header is not little-endian or is not 8 octets");

    const atc::hdlc::Frame responseFrame{
        1, 0x30,
        {0x05, 0x00, 0x34, 0x12, 0x00, 0x00, 0x08, 0x00,
         0x01, 'P', 0x01, 'S', 0x01, 'H', 0x01, 'W'}};
    const auto response = parseResponse(responseFrame, Command::getInformation, 0x1234);
    const auto information = response ? parseInformation(*response) : std::nullopt;
    require(information && information->productNumber == "P" &&
                information->serialNumber == "S" && information->hardwareVersion == "H" &&
                information->softwareVersion == "W",
            "AISG v3 GetInformation response was not decoded");

    const atc::hdlc::Frame errorFrame{
        1, 0x30,
        {0x05, 0x00, 0x34, 0x12, 0x24, 0x00, 0x02, 0x00, 0x01, 0x06}};
    const auto error = parseResponse(errorFrame, Command::getInformation, 0x1234);
    require(error && !error->success() && error->returnCode == ReturnCode::formatError &&
                error->aldState == 1 && error->connectionState == 6,
            "AISG v3 error response did not preserve state fields");

    const auto subunits = parseSubunitList(aisg3Response(
        Command::getSubunitList, {0x02, 0x00, 0x01, 0x00, 0x03, 0x02, 0x00, 0x01}));
    require(subunits && subunits->size() == 2 && (*subunits)[0].number == 1 &&
                (*subunits)[0].type == SubunitType::adb && (*subunits)[1].number == 2 &&
                (*subunits)[1].type == SubunitType::ret,
            "AISG v3 subunit list layout is incorrect");

    const auto versions = parseSubunitVersions(aisg3Response(
        Command::getSubunitTypeStandardVersions,
        {0x00, 0x00, 0x00, 0x02, 0x03, 0x01, 0x06, 0x03, 0x01, 0x07}));
    require(versions && versions->supported.size() == 2 &&
                versions->supported.back() == supportedAdbVersion,
            "AISG v3 subunit-version response was not decoded");
    require(makeSetSubunitVersion(7, SubunitType::adb, supportedAdbVersion).data ==
                bytes({0x03, 0x03, 0x01, 0x07}),
            "AISG v3 ADB version negotiation command is incorrect");

    const auto alarms = parseAlarmStatus(
        aisg3Response(Command::getAlarmStatus, {0x02, 0x06, 0x00, 0x03, 0x08, 0x00, 0x01}));
    require(alarms && alarms->size() == 2 && (*alarms)[0].code == 6 &&
                (*alarms)[0].severity == 3,
            "AISG v3 AlarmState list was not decoded as uint16/severity tuples");
    require(!parseAlarmStatus(aisg3Response(Command::getAlarmStatus, {1, 6, 0, 0})),
            "AISG v3 active-alarm list accepted Cleared severity");

    bool oversizedRejected = false;
    try {
        (void)encodeCommand({Command::getInformation, 1, 0,
                             atc::hdlc::Bytes(maximumLayer7Data + 1, 0)});
    } catch (const std::invalid_argument&) {
        oversizedRejected = true;
    }
    require(oversizedRejected, "AISG v3 encoder accepted more than 256 data octets");
}

void testAdb317Conformance() {
    using namespace atc::aisg3;
    using namespace atc::aisg3::adb;

    const auto antenna = parseAntennaInfo(aisg3Response(
        Command::adbGetAntennaInfo,
        {0x01, 'M', 0x01, 0x01, 'S', 0x01, 0x02, 0x01}));
    require(antenna && antenna->modelNumber.value == "M" &&
                antenna->serialNumber.value == "S" && antenna->numberOfArrayElements == 2,
            "ADB Get Antenna Info response was not decoded");

    const auto port = parseAntennaPortInfo(aisg3Response(
        Command::adbGetAntennaPortInfo, {0x02, 0x01, 0x00, 0x03, 0x00, 0x01}));
    require(port && port->arrayElements == std::vector<std::uint16_t>({1, 3}) &&
                port->provenance == Provenance::factory,
            "ADB antenna-port response was not decoded");

    const auto array = parseArrayElementInfo(aisg3Response(
        Command::adbGetAntennaArrayElementInfo,
        {0x8A, 0x02, 0x01, 0xB4, 0x00, 0x01, 0x00,
         0x01, 0x03, 0x01, 0x00, 0x01, 0x00, 0x01}));
    require(array && array->azimuthBeamwidthDeciDegrees == 650 &&
                array->gainDeciDbi == 180 && array->frequencies.empty() &&
                array->polarisation == 3,
            "ADB array-element response was not decoded");

    InstallationUpdate update;
    update.sectorId = TextValue{"A", Provenance::manual};
    update.installationNotes = TextValue{"\xC3\xA9", Provenance::manual};
    update.mechanicalAzimuthDeciDegrees = 3599;
    update.mechanicalTiltDeciDegrees = -900;
    const auto setInstallation = makeSetInstallationInfo(9, 1, update);
    require(setInstallation.data == bytes({
                0x0F,
                0x01, 'A', 0x04,
                0x02, 0xC3, 0xA9, 0x04,
                0x0F, 0x0E, 0x04,
                0x7C, 0xFC, 0x04}),
            "ADB installation data does not use mask/order/deci-degree encoding from 11.6.4");

    const auto installation = parseInstallationInfo(aisg3Response(
        Command::adbGetAntennaInstallationInfo,
        {0x01, 'A', 0x04, 0x00, 0x04, 0x0F, 0x0E, 0x04, 0x7C, 0xFC, 0x04}));
    require(installation && installation->sectorId.value == "A" &&
                installation->mechanicalAzimuthDeciDegrees == 3599 &&
                installation->mechanicalTiltDeciDegrees == -900,
            "ADB signed mechanical tilt or azimuth was not decoded correctly");

    require(!parseAntennaInfo(aisg3Response(
                Command::adbGetAntennaInfo, {1, 0xFF, 1, 1, 'S', 1, 1, 1})),
            "ADB parser accepted malformed UTF-8 antenna information");
    require(!parseInstallationInfo(aisg3Response(
                Command::adbGetAntennaInstallationInfo,
                {1, '\n', 4, 0, 4, 0, 0, 4, 0, 0, 4})),
            "ADB parser accepted a non-TextString sector ID");

    const auto setPaths = makeSetRfPathIds(10, 1, 2, {{0x1234, 0xABCD}, Provenance::manual});
    require(setPaths.data == bytes({0x02, 0x00, 0x02, 0x34, 0x12, 0xCD, 0xAB, 0x04}),
            "ADB RF Path ID assignment is not little-endian");
    const auto getPaths = makeGetRfPathIds(11, 1, 0x12345678, 2);
    require(getPaths.data == bytes({0x78, 0x56, 0x34, 0x12, 0x02, 0x00}),
            "ADB Layer-7 PrimaryID must be little-endian");
}

void testAisgFrameLengthLimits() {
    const atc::hdlc::Frame maximum{1, 0x10,
                                   atc::hdlc::Bytes(atc::hdlc::maximumInformationBytes, 0x55)};
    const auto encoded = atc::hdlc::encode(maximum);
    const auto decoded = atc::hdlc::decode(encoded);
    require(decoded && decoded.frame->information.size() == atc::hdlc::maximumInformationBytes,
            "maximum AISG frame length did not round trip");

    bool rejected = false;
    try {
        (void)atc::hdlc::encode({1, 0x10,
                                 atc::hdlc::Bytes(atc::hdlc::maximumInformationBytes + 1, 0x55)});
    } catch (const std::invalid_argument&) {
        rejected = true;
    }
    require(rejected, "HDLC encoder accepted a frame larger than AISG Base 11.5 permits");
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

void testControllerAisg3AdbEndToEnd() {
    auto transport = std::make_shared<Aisg3AdbTestTransport>();
    atc::ControllerService controller(transport);

    atc::TransportConfig config;
    config.endpoint = "test://aisg3-adb";
    config.baudRate = 9600;
    waitFor(controller, controller.connect(config));

    atc::ScanOptions options;
    options.firstAddress = 1;
    options.lastAddress = 2;
    options.responseTimeout = 1ms;
    options.protocol = atc::ProtocolProfile::aisg3;
    options.primaryId = 0x41544331;
    const auto events = waitFor(controller, controller.scan(options), 5s);
    require(transport->scans() >= 3 && transport->assignments() == 1,
            "AISG v3 scan did not perform discovery, identity confirmation and assignment");
    require(std::count_if(events.begin(), events.end(), [](const auto& event) {
                return event.kind == atc::EventKind::deviceAdded;
            }) == 1,
            "AISG v3 scan did not publish exactly one assigned ALD");

    const auto device = controller.device(1);
    require(device && device->protocol == atc::ProtocolProfile::aisg3 &&
                device->kind == atc::DeviceKind::adb && device->vendor == "AB" &&
                device->product == "ALD-ADB" && device->serialNumber == "CTRL01",
            "AISG v3 Base information or ADB subunit identity was not populated");
    require(device->installation.antennaModel == "ANT-900" &&
                device->installation.antennaSerial == "ANT001" &&
                device->installation.sectorId == "S1" &&
                device->installation.location == "TOPO" &&
                device->installation.bearingDegrees == 90.0 &&
                device->installation.mechanicalTiltDegrees == -0.5,
            "ADB 3.1.7 antenna/installation information was not populated");

    waitFor(controller, controller.refreshAlarms(1), 5s);
    const auto refreshed = controller.device(1);
    require(refreshed && refreshed->alarms.empty() &&
                refreshed->status == atc::DeviceStatus::ready,
            "AISG v3 Base alarm refresh did not cover the ALD and ADB subunit");
    waitFor(controller, controller.disconnect());
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
        {"AISG v3 XID conformance", testAisg3XidConformance},
        {"AISG v3 Layer 7 conformance", testAisg3Layer7Conformance},
        {"ADB 3.1.7 conformance", testAdb317Conformance},
        {"AISG frame length limits", testAisgFrameLengthLimits},
        {"domain validation", testDomainValidation},
        {"simulator low-level", testSimulatorLowLevel},
        {"POSIX serial safe failure", testPosixSerialFailureIsSafe},
        {"controller end-to-end", testControllerEndToEnd},
        {"AISG v3 ADB controller end-to-end", testControllerAisg3AdbEndToEnd},
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
