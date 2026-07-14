#include "atc/simulated_transport.hpp"

#include "atc/aisg.hpp"

#include <algorithm>
#include <cmath>
#include <string_view>

namespace atc {
namespace {

Device simulatedRet(const std::uint8_t address,
                    std::string serial,
                    const double tilt,
                    std::string sector) {
    Device device;
    device.address = address;
    device.uid = "TY" + serial;
    device.vendor = "Commscope";
    device.product = "RCU-0809-DVT";
    device.serialNumber = std::move(serial);
    device.hardwareVersion = "RET_JSXX_V5.0";
    device.softwareVersion = "4.4.4";
    device.kind = DeviceKind::ret;
    device.status = DeviceStatus::ready;
    device.installation.antennaModel = "P500169";
    device.installation.antennaSerial = device.serialNumber;
    device.installation.baseStationId = "SITE-DEMO-01";
    device.installation.sectorId = std::move(sector);
    device.installation.installationDate = "071426";
    device.details = Ret{tilt, 0.0, 15.0, true, false};
    return device;
}

Device simulatedTma(const std::uint8_t address) {
    Device device;
    device.address = address;
    device.uid = "TYTMA0000000000001";
    device.vendor = "Commscope";
    device.product = "TMA-DUAL-DEMO";
    device.serialNumber = "TMA-DEMO-0001";
    device.hardwareVersion = "TMA-HW-1";
    device.softwareVersion = "1.0.0";
    device.kind = DeviceKind::tma;
    device.status = DeviceStatus::ready;
    device.installation.baseStationId = "SITE-DEMO-01";
    device.installation.sectorId = "C";
    device.details = Tma{12.0, 0.0, 30.0, TmaMode::normal};
    return device;
}

void appendString(hdlc::Bytes& target, const std::string_view value) {
    const auto length = std::min<std::size_t>(value.size(), 255);
    target.push_back(static_cast<std::uint8_t>(length));
    target.insert(target.end(), value.begin(), value.begin() + static_cast<std::ptrdiff_t>(length));
}

hdlc::Bytes textBytes(const std::string& text) {
    return {text.begin(), text.end()};
}

std::vector<SimulatedDevice> defaults() {
    auto first = simulatedRet(1, "0809R1-H231234561", 12.0, "A");
    auto second = simulatedRet(2, "0809R2-H231234562", 5.0, "B");
    return {{std::move(first), {}}, {std::move(second), {0x0B, 0x0E}},
            {simulatedTma(3), {}}};
}

} // namespace

SimulatedTransport::SimulatedTransport() : SimulatedTransport(defaults()) {}

SimulatedTransport::SimulatedTransport(std::vector<SimulatedDevice> devices)
    : devices_(std::move(devices)) {}

void SimulatedTransport::open(const TransportConfig&) {
    std::scoped_lock lock(mutex_);
    open_ = true;
    received_.clear();
    decoder_.reset();
}

void SimulatedTransport::close() noexcept {
    {
        std::scoped_lock lock(mutex_);
        open_ = false;
        received_.clear();
        decoder_.reset();
    }
    condition_.notify_all();
}

bool SimulatedTransport::isOpen() const noexcept {
    std::scoped_lock lock(mutex_);
    return open_;
}

void SimulatedTransport::write(const std::span<const std::uint8_t> bytes) {
    {
        std::scoped_lock lock(mutex_);
        if (!open_) {
            throw TransportError("simulated transport is closed");
        }
        for (auto& decoded : decoder_.push(bytes)) {
            if (decoded.frame) {
                process(*decoded.frame);
            }
        }
    }
    condition_.notify_all();
}

hdlc::Bytes SimulatedTransport::read(const std::chrono::milliseconds timeout,
                                     const std::size_t maximumBytes) {
    std::unique_lock lock(mutex_);
    condition_.wait_for(lock, timeout, [this] { return !received_.empty() || !open_; });
    const auto count = std::min(maximumBytes, received_.size());
    hdlc::Bytes result;
    result.reserve(count);
    for (std::size_t index = 0; index < count; ++index) {
        result.push_back(received_.front());
        received_.pop_front();
    }
    return result;
}

std::string SimulatedTransport::description() const { return "AISG deterministic simulator"; }

std::vector<SimulatedDevice> SimulatedTransport::snapshot() const {
    std::scoped_lock lock(mutex_);
    return devices_;
}

void SimulatedTransport::process(const hdlc::Frame& request) {
    auto iterator = std::find_if(devices_.begin(), devices_.end(), [&](const auto& candidate) {
        return candidate.device.address == request.address;
    });
    if (iterator == devices_.end()) {
        return;
    }
    auto& simulated = *iterator;

    if (request.control == 0x93 && request.information.empty()) {
        queue({request.address, 0x73, {}});
        return;
    }
    if (request.information.empty()) {
        queue(request);
        return;
    }

    const auto responseControl = aisg::expectedResponseControl(request.control);
    const auto command = static_cast<aisg::Command>(request.information.front());
    hdlc::Bytes information;

    switch (command) {
    case aisg::Command::initialData: {
        information = {static_cast<std::uint8_t>(command), 0x00, 0x00, 0x00};
        appendString(information, simulated.device.product);
        appendString(information, simulated.device.serialNumber);
        appendString(information, simulated.device.hardwareVersion);
        appendString(information, simulated.device.softwareVersion);
        const auto dataLength = information.size() - 3;
        information[1] = static_cast<std::uint8_t>(dataLength & 0xFFU);
        information[2] = static_cast<std::uint8_t>((dataLength >> 8U) & 0xFFU);
        break;
    }
    case aisg::Command::getTilt: {
        const auto* ret = simulated.device.ret();
        if (!ret) {
            information = {static_cast<std::uint8_t>(command), 0x01, 0x00, 0x01};
            break;
        }
        const auto tilt = static_cast<std::uint16_t>(std::lround(ret->electricalTiltDegrees * 10.0));
        information = {static_cast<std::uint8_t>(command), 0x03, 0x00, 0x00,
                       static_cast<std::uint8_t>(tilt & 0xFFU),
                       static_cast<std::uint8_t>((tilt >> 8U) & 0xFFU)};
        break;
    }
    case aisg::Command::setTilt: {
        auto* ret = simulated.device.ret();
        if (!ret || request.information.size() < 6) {
            information = {static_cast<std::uint8_t>(command), 0x01, 0x00, 0x03};
            break;
        }
        const auto deciDegrees = static_cast<std::uint16_t>(request.information[4]) |
                                 (static_cast<std::uint16_t>(request.information[5]) << 8U);
        const auto degrees = static_cast<double>(deciDegrees) / 10.0;
        if (!ret->acceptsTilt(degrees)) {
            information = {static_cast<std::uint8_t>(command), 0x01, 0x00, 0x03};
            break;
        }
        ret->electricalTiltDegrees = degrees;
        ++simulated.device.revision;
        information = {static_cast<std::uint8_t>(command), 0x01, 0x00, 0x00};
        break;
    }
    case aisg::Command::getAlarms:
        information = {static_cast<std::uint8_t>(command),
                       static_cast<std::uint8_t>(simulated.alarmCodes.size() + 1),
                       0x00, 0x00};
        information.insert(information.end(), simulated.alarmCodes.begin(), simulated.alarmCodes.end());
        break;
    case aisg::Command::clearAlarms:
        simulated.alarmCodes.clear();
        simulated.device.alarms.clear();
        information = {static_cast<std::uint8_t>(command), 0x01, 0x00, 0x00};
        break;
    case aisg::Command::selfTest:
    case aisg::Command::calibrate:
    case aisg::Command::subscribeAlarms:
        if (command == aisg::Command::calibrate) {
            if (auto* ret = simulated.device.ret()) {
                ret->calibrated = true;
            }
        }
        information = {static_cast<std::uint8_t>(command), 0x01, 0x00, 0x00};
        break;
    case aisg::Command::setTmaMode: {
        auto* tma = simulated.device.tma();
        if (!tma || request.information.size() < 4) {
            information = {static_cast<std::uint8_t>(command), 0x01, 0x00, 0x03};
            break;
        }
        tma->mode = request.information[3] == 0 ? TmaMode::normal : TmaMode::bypass;
        ++simulated.device.revision;
        information = {static_cast<std::uint8_t>(command), 0x01, 0x00, 0x00};
        break;
    }
    case aisg::Command::getTmaMode: {
        const auto* tma = simulated.device.tma();
        if (!tma) {
            information = {static_cast<std::uint8_t>(command), 0x01, 0x00, 0x03};
            break;
        }
        const auto value = tma->mode == TmaMode::bypass ? std::uint8_t{1} : std::uint8_t{0};
        information = {static_cast<std::uint8_t>(command), 0x02, 0x00, 0x00, value};
        break;
    }
    case aisg::Command::setTmaGain: {
        auto* tma = simulated.device.tma();
        if (!tma || request.information.size() < 4) {
            information = {static_cast<std::uint8_t>(command), 0x01, 0x00, 0x03};
            break;
        }
        const auto gain = static_cast<double>(request.information[3]) / 4.0;
        if (gain < tma->minimumGainDb || gain > tma->maximumGainDb) {
            information = {static_cast<std::uint8_t>(command), 0x01, 0x00, 0x03};
            break;
        }
        tma->gainDb = gain;
        ++simulated.device.revision;
        information = {static_cast<std::uint8_t>(command), 0x01, 0x00, 0x00};
        break;
    }
    case aisg::Command::getTmaGain: {
        const auto* tma = simulated.device.tma();
        if (!tma) {
            information = {static_cast<std::uint8_t>(command), 0x01, 0x00, 0x03};
            break;
        }
        const auto value = static_cast<std::uint8_t>(std::lround(tma->gainDb * 4.0));
        information = {static_cast<std::uint8_t>(command), 0x02, 0x00, 0x00, value};
        break;
    }
    case aisg::Command::getDeviceData: {
        if (request.information.size() < 4) {
            information = {static_cast<std::uint8_t>(command), 0x01, 0x00, 0x03};
            break;
        }
        const auto field = static_cast<aisg::Field>(request.information[3]);
        hdlc::Bytes data;
        switch (field) {
        case aisg::Field::antennaModel:
            data = textBytes(simulated.device.installation.antennaModel); break;
        case aisg::Field::uniqueId:
            data = textBytes(simulated.device.uid); break;
        case aisg::Field::installationDate:
            data = textBytes(simulated.device.installation.installationDate); break;
        case aisg::Field::installerId:
            data = textBytes(simulated.device.installation.installerId); break;
        case aisg::Field::baseStationId:
            data = textBytes(simulated.device.installation.baseStationId); break;
        case aisg::Field::sectorId:
            data = textBytes(simulated.device.installation.sectorId); break;
        case aisg::Field::bearing: {
            const auto value = static_cast<std::uint16_t>(
                std::lround(simulated.device.installation.bearingDegrees * 10.0));
            data = {static_cast<std::uint8_t>(value & 0xFFU),
                    static_cast<std::uint8_t>((value >> 8U) & 0xFFU)};
            break;
        }
        case aisg::Field::mechanicalTilt: {
            const auto value = static_cast<std::uint16_t>(
                std::lround(simulated.device.installation.mechanicalTiltDegrees * 10.0));
            data = {static_cast<std::uint8_t>(value & 0xFFU),
                    static_cast<std::uint8_t>((value >> 8U) & 0xFFU)};
            break;
        }
        case aisg::Field::maximumTilt: {
            const auto* ret = simulated.device.ret();
            const auto value = static_cast<std::uint16_t>(
                std::lround((ret ? ret->maximumTiltDegrees : 0.0) * 10.0));
            data = {static_cast<std::uint8_t>(value & 0xFFU),
                    static_cast<std::uint8_t>((value >> 8U) & 0xFFU)};
            break;
        }
        case aisg::Field::minimumTilt: {
            const auto* ret = simulated.device.ret();
            const auto value = static_cast<std::uint16_t>(
                std::lround((ret ? ret->minimumTiltDegrees : 0.0) * 10.0));
            data = {static_cast<std::uint8_t>(value & 0xFFU),
                    static_cast<std::uint8_t>((value >> 8U) & 0xFFU)};
            break;
        }
        default:
            break;
        }
        information = {static_cast<std::uint8_t>(command),
                       static_cast<std::uint8_t>(data.size() + 1), 0x00, 0x00};
        information.insert(information.end(), data.begin(), data.end());
        break;
    }
    case aisg::Command::setDeviceData: {
        if (request.information.size() < 4) {
            information = {static_cast<std::uint8_t>(command), 0x01, 0x00, 0x03};
            break;
        }
        const auto field = static_cast<aisg::Field>(request.information[3]);
        const std::string value(request.information.begin() + 4, request.information.end());
        switch (field) {
        case aisg::Field::antennaModel: simulated.device.installation.antennaModel = value; break;
        case aisg::Field::installationDate: simulated.device.installation.installationDate = value; break;
        case aisg::Field::installerId: simulated.device.installation.installerId = value; break;
        case aisg::Field::baseStationId: simulated.device.installation.baseStationId = value; break;
        case aisg::Field::sectorId: simulated.device.installation.sectorId = value; break;
        default: break;
        }
        ++simulated.device.revision;
        information = {static_cast<std::uint8_t>(command), 0x01, 0x00, 0x00};
        break;
    }
    default:
        information = {request.information.front(), 0x01, 0x00, 0x01};
        break;
    }

    queue({request.address, responseControl, std::move(information)});
}

void SimulatedTransport::queue(const hdlc::Frame& response) {
    const auto encoded = hdlc::encode(response);
    received_.insert(received_.end(), encoded.begin(), encoded.end());
}

} // namespace atc
