#include "atc/controller.hpp"

#include "atc/adb.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace atc {

namespace {

std::string hex16(const std::uint16_t value) {
    constexpr char digits[] = "0123456789ABCDEF";
    std::string result(4, '0');
    result[0] = digits[(value >> 12U) & 0x0FU];
    result[1] = digits[(value >> 8U) & 0x0FU];
    result[2] = digits[(value >> 4U) & 0x0FU];
    result[3] = digits[value & 0x0FU];
    return result;
}

std::optional<double> parseSignedDeciDegreeData(const hdlc::Frame& response) {
    const auto data = aisg::parseData(response);
    if (!data || !data->status.success || data->value.size() != 2) {
        return std::nullopt;
    }
    const auto raw = static_cast<std::uint16_t>(
        static_cast<std::uint16_t>(data->value[0]) |
        (static_cast<std::uint16_t>(data->value[1]) << 8U));
    const auto signedValue = raw < 0x8000U
                                 ? static_cast<std::int32_t>(raw)
                                 : static_cast<std::int32_t>(raw) - 0x10000;
    return static_cast<double>(signedValue) / 10.0;
}

} // namespace

struct ControllerService::Work {
    OperationId id{};
    std::string name;
    Task task;
    std::shared_ptr<std::atomic_bool> cancelled;
};

ControllerService::ControllerService(std::shared_ptr<ITransport> transport)
    : transport_(std::move(transport)),
      worker_([this](const std::stop_token stopToken) { workerLoop(stopToken); }) {
    if (!transport_) {
        throw std::invalid_argument("ControllerService needs a transport");
    }
}

ControllerService::~ControllerService() {
    worker_.request_stop();
    queueCondition_.notify_all();
    if (worker_.joinable()) {
        worker_.join();
    }
    transport_->close();
}

OperationId ControllerService::connect(TransportConfig config) {
    return enqueue("Connect", [this, config = std::move(config)](
                                  const OperationId operation,
                                  const std::shared_ptr<std::atomic_bool>&) {
        if (transport_->isOpen()) {
            transport_->close();
        }
        transport_->open(config);
        decoder_.reset();
        {
            std::scoped_lock lock(stateMutex_);
            connected_ = true;
            minimumFrameInterval_ =
                std::max(config.minimumFrameInterval, std::chrono::milliseconds::zero());
            lastFrameActivity_.reset();
        }
        emit({EventKind::connectionChanged, operation, std::chrono::system_clock::now(),
              std::nullopt, std::nullopt, -1, "Connected: " + transport_->description(), {}});
    });
}

OperationId ControllerService::disconnect() {
    return enqueue("Disconnect", [this](const OperationId operation,
                                         const std::shared_ptr<std::atomic_bool>&) {
        transport_->close();
        decoder_.reset();
        {
            std::scoped_lock lock(stateMutex_);
            connected_ = false;
            devices_.clear();
            sequences_.clear();
            minimumFrameInterval_ = std::chrono::milliseconds::zero();
            lastFrameActivity_.reset();
        }
        emit({EventKind::devicesCleared, operation, std::chrono::system_clock::now(),
              std::nullopt, std::nullopt, -1, "Device list cleared", {}});
        emit({EventKind::connectionChanged, operation, std::chrono::system_clock::now(),
              std::nullopt, std::nullopt, -1, "Disconnected", {}});
    });
}

OperationId ControllerService::scan(const ScanOptions options) {
    return enqueue("Scan", [this, options](const OperationId operation,
                                            const std::shared_ptr<std::atomic_bool>& cancelled) {
        if (!isConnected()) {
            throw std::runtime_error("cannot scan while disconnected");
        }
        if (options.firstAddress == 0 || options.lastAddress == aisg3::allStationsAddress ||
            options.firstAddress > options.lastAddress) {
            throw std::invalid_argument("invalid AISG scan address range");
        }
        if (options.protocol == ProtocolProfile::aisg3) {
            scanAisg3(operation, options, cancelled);
            return;
        }
        scanAisg2(operation, options, cancelled);
    });
}

void ControllerService::scanAisg2(
    const OperationId operation,
    const ScanOptions& options,
    const std::shared_ptr<std::atomic_bool>& cancelled) {
    using namespace std::chrono_literals;

    {
        std::scoped_lock lock(stateMutex_);
        devices_.clear();
        sequences_.clear();
    }
    emit({EventKind::devicesCleared, operation, std::chrono::system_clock::now(),
          std::nullopt, std::nullopt, 0,
          options.discoverUnaddressed ? "Starting AISG 2.0 XID device scan"
                                      : "Starting AISG 2.0 address scan", {}});

    struct AssignedDevice {
        std::uint8_t address{};
        std::optional<aisg::XidIdentity> identity;
    };
    std::vector<AssignedDevice> assigned;

    struct XidExchange {
        bool activity{};
        bool invalid{};
        std::vector<hdlc::Frame> frames;
    };
    const auto exchangeXid = [&](const hdlc::Frame& request,
                                 const std::chrono::milliseconds timeout) {
        XidExchange result;
        decoder_.reset();
        const auto encoded = hdlc::encode(request);
        if (!transmit(operation, encoded, cancelled)) return result;

        const auto deadline = std::chrono::steady_clock::now() + timeout;
        bool rawActivity = false;
        while (std::chrono::steady_clock::now() < deadline && !cancelled->load()) {
            const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
                deadline - std::chrono::steady_clock::now());
            const auto received = transport_->read(std::min(std::max(remaining, 1ms), 25ms));
            if (received.empty()) {
                if (rawActivity) break;
                continue;
            }
            noteFrameActivity();
            rawActivity = true;
            logFrame(EventKind::rxFrame, operation, received);
            for (auto& decoded : decoder_.push(received)) {
                if (!decoded.frame) {
                    result.invalid = true;
                    continue;
                }
                if (*decoded.frame == request) continue;
                result.activity = true;
                result.frames.push_back(std::move(*decoded.frame));
            }
        }
        result.activity = result.activity || result.invalid || rawActivity;
        decoder_.reset();
        if (!result.frames.empty()) std::this_thread::sleep_for(3ms);
        return result;
    };

    if (options.discoverUnaddressed) {
        if (options.resetBeforeDiscovery) {
            emit({EventKind::log, operation, std::chrono::system_clock::now(),
                  std::nullopt, std::nullopt, -1,
                  "Resetting virtual AISG 2.0 devices to NoAddress before discovery", {}});
            // Reset Device in broadcast intentionally has no response. The
            // configured quiet interval gives every simulated RET enough time
            // to reinitialize before the first Device Scan.
            (void)exchangeXid(aisg::makeResetDeviceRequest(),
                              std::max(options.responseTimeout, 100ms));
        }

        enum class QueryKind { none, found, collision };
        struct QueryResult {
            QueryKind kind{QueryKind::none};
            std::optional<aisg::XidIdentity> identity;
        };
        std::size_t queryCount{};
        const auto query = [&](const hdlc::Bytes& value, const hdlc::Bytes& mask) {
            if (++queryCount > 512) {
                throw std::runtime_error(
                    "AISG 2.0 XID collision resolution exceeded 512 queries");
            }
            const auto exchange = exchangeXid(
                aisg::makeDeviceScanRequest(value, mask),
                std::max(options.responseTimeout, 75ms));
            std::vector<aisg::XidIdentity> identities;
            for (const auto& frame : exchange.frames) {
                if (auto parsed = aisg::parseDeviceScanResponse(frame)) {
                    identities.push_back(std::move(*parsed));
                }
            }
            // Em uma bancada virtual, respostas temporizadas de vários ALDs
            // podem chegar como quadros íntegros. Qualquer identidade completa
            // pode ser atribuída; a próxima varredura encontrará as restantes.
            if (!exchange.invalid && !identities.empty()) {
                return QueryResult{QueryKind::found, std::move(identities.front())};
            }
            return QueryResult{exchange.activity ? QueryKind::collision : QueryKind::none,
                               std::nullopt};
        };

        using Pattern = std::array<std::uint8_t, 19>;
        const auto scanPattern = [](const Pattern& value, const Pattern& mask) {
            return std::pair{hdlc::Bytes(value.begin(), value.end()),
                             hdlc::Bytes(mask.begin(), mask.end())};
        };
        std::function<std::optional<aisg::XidIdentity>(Pattern, Pattern, std::size_t)>
            resolveCollision;
        resolveCollision = [&](Pattern value, Pattern mask, const std::size_t firstBit)
            -> std::optional<aisg::XidIdentity> {
            for (std::size_t bit = firstBit; bit < value.size() * 8; ++bit) {
                const auto octet = bit / 8;
                const auto bitMask = static_cast<std::uint8_t>(1U << (7U - bit % 8U));
                for (const bool one : {false, true}) {
                    auto branchValue = value;
                    auto branchMask = mask;
                    branchMask[octet] |= bitMask;
                    if (one) branchValue[octet] |= bitMask;
                    else branchValue[octet] &= static_cast<std::uint8_t>(~bitMask);
                    const auto [bytes, masks] = scanPattern(branchValue, branchMask);
                    const auto branch = query(bytes, masks);
                    if (branch.kind == QueryKind::found) return branch.identity;
                    if (branch.kind == QueryKind::collision) {
                        if (auto resolved = resolveCollision(
                                branchValue, branchMask, bit + 1)) {
                            return resolved;
                        }
                    }
                }
            }
            return std::nullopt;
        };

        auto nextAddress = options.firstAddress;
        while (!cancelled->load() && nextAddress <= options.lastAddress) {
            queryCount = 0;
            const auto broad = query({}, {});
            if (broad.kind == QueryKind::none) break;
            auto candidate = broad.identity;
            if (broad.kind == QueryKind::collision) {
                candidate = resolveCollision({}, {}, 0);
            }
            if (!candidate) {
                throw std::runtime_error(
                    "AISG 2.0 XID collision could not be resolved");
            }

            const hdlc::Bytes exactMask(candidate->uniqueId.size(), 0xFF);
            const auto confirmation = query(candidate->uniqueId, exactMask);
            if (confirmation.kind != QueryKind::found || !confirmation.identity ||
                *confirmation.identity != *candidate) {
                throw std::runtime_error("AISG 2.0 XID identity confirmation failed");
            }
            candidate = confirmation.identity;

            const auto assignment = exchangeXid(
                aisg::makeAddressAssignmentRequest(*candidate, nextAddress), 250ms);
            std::size_t acknowledgements{};
            for (const auto& frame : assignment.frames) {
                if (aisg::isAddressAssignmentResponse(frame, *candidate, nextAddress)) {
                    ++acknowledgements;
                }
            }
            if (acknowledgements != 1) {
                throw std::runtime_error(
                    "AISG 2.0 address assignment was not uniquely acknowledged");
            }
            assigned.push_back({nextAddress, std::move(candidate)});
            emit({EventKind::scanProgress, operation, std::chrono::system_clock::now(),
                  nextAddress, std::nullopt,
                  static_cast<int>((assigned.size() * 50U) /
                                   (options.lastAddress - options.firstAddress + 1U)),
                  "AISG 2.0 XID identity assigned", {}});
            ++nextAddress;
        }
    } else {
        for (unsigned int address = options.firstAddress;
             address <= options.lastAddress; ++address) {
            assigned.push_back({static_cast<std::uint8_t>(address), std::nullopt});
        }
    }

    const auto responseTimeout = std::max(options.responseTimeout, 50ms);
    for (std::size_t index = 0; index < assigned.size() && !cancelled->load(); ++index) {
        const auto address = assigned[index].address;
        const auto ua = transact(operation, aisg::makeSnrm(address),
                                 responseTimeout, cancelled);
        if (!ua || ua->control != 0x73) {
            if (options.discoverUnaddressed) {
                throw std::runtime_error("AISG 2.0 RET did not answer SNRM with UA");
            }
            continue;
        }
        std::this_thread::sleep_for(3ms);

        if (options.discoverUnaddressed) {
            bool accepted = false;
            for (unsigned int attempt = 0; attempt < 2 && !accepted; ++attempt) {
                const auto negotiation = exchangeXid(
                    aisg::makeReleaseNegotiationRequest(address), responseTimeout);
                accepted = std::ranges::any_of(
                    negotiation.frames, [address](const auto& frame) {
                        return aisg::isReleaseNegotiationResponse(frame, address);
                    });
                if (!accepted && attempt == 0 && !cancelled->load()) {
                    emit({EventKind::log, operation, std::chrono::system_clock::now(),
                          address, std::nullopt, -1,
                          "AISG 2.0 release ACK not collected; retrying once", {}});
                }
            }
            if (!accepted) {
                throw std::runtime_error("AISG 2.0 release negotiation was not acknowledged");
            }
        }

        Device found;
        found.address = address;
        found.status = DeviceStatus::initializing;
        if (assigned[index].identity) {
            const auto& identity = *assigned[index].identity;
            found.uid.assign(identity.uniqueId.begin(), identity.uniqueId.end());
            found.vendor.assign(identity.vendorCode.begin(), identity.vendorCode.end());
        }

        (void)transact(operation,
                       aisg::makeSubscribeAlarmsRequest(address, nextControl(address)),
                       responseTimeout, cancelled);
        const auto initialResponse = transact(
            operation, aisg::makeInitialDataRequest(address, nextControl(address)),
            responseTimeout, cancelled);
        if (initialResponse) {
            if (const auto initial = aisg::parseInitialData(*initialResponse);
                initial && initial->status.success) {
                found.product = initial->product;
                found.serialNumber = initial->serialNumber;
                found.hardwareVersion = initial->hardwareVersion;
                found.softwareVersion = initial->softwareVersion;
                if (found.uid.empty()) found.uid = initial->serialNumber;
            }
        }

        const auto isTma = found.product.find("TMA") != std::string::npos;
        found.kind = isTma ? DeviceKind::tma : DeviceKind::ret;
        if (isTma) {
            found.details = Tma{};
            auto& tma = std::get<Tma>(found.details);
            if (const auto response = transact(
                    operation, aisg::makeGetTmaGainRequest(address, nextControl(address)),
                    responseTimeout, cancelled)) {
                if (const auto gain = aisg::parseTmaGain(*response)) tma.gainDb = *gain;
            }
            if (const auto response = transact(
                    operation, aisg::makeGetTmaModeRequest(address, nextControl(address)),
                    responseTimeout, cancelled)) {
                if (const auto mode = aisg::parseTmaMode(*response)) tma.mode = *mode;
            }
        } else {
            found.details = Ret{};
            auto& ret = std::get<Ret>(found.details);
            if (const auto response = transact(
                    operation, aisg::makeGetTiltRequest(address, nextControl(address)),
                    responseTimeout, cancelled)) {
                if (const auto tilt = aisg::parseTilt(*response)) {
                    ret.electricalTiltDegrees = *tilt;
                }
            }
            const auto readLimit = [&](const aisg::Field field) -> std::optional<double> {
                const auto response = transact(
                    operation, aisg::makeGetDataRequest(address, nextControl(address), field),
                    responseTimeout, cancelled);
                return response ? parseSignedDeciDegreeData(*response) : std::nullopt;
            };
            const auto maximum = readLimit(aisg::Field::maximumTilt);
            const auto minimum = readLimit(aisg::Field::minimumTilt);
            if (maximum && minimum && *minimum <= *maximum) {
                ret.minimumTiltDegrees = *minimum;
                ret.maximumTiltDegrees = *maximum;
            }
        }
        found.status = DeviceStatus::ready;
        storeDevice(std::move(found), operation, true);
        emit({EventKind::scanProgress, operation, std::chrono::system_clock::now(),
              address, std::nullopt,
              static_cast<int>(((index + 1U) * 100U) / assigned.size()),
              "AISG 2.0 RET discovered and initialized", {}});
    }
}

void ControllerService::scanAisg3(
    const OperationId operation,
    const ScanOptions& options,
    const std::shared_ptr<std::atomic_bool>& cancelled) {
    using namespace std::chrono_literals;

    if (options.primaryId == 0) {
        throw std::invalid_argument("AISG v3 PrimaryID must be non-zero");
    }
    {
        std::scoped_lock lock(stateMutex_);
        devices_.clear();
        sequences_.clear();
        aisg3Sequence_ = 0;
    }
    emit({EventKind::devicesCleared, operation, std::chrono::system_clock::now(),
          std::nullopt, std::nullopt, 0, "Starting AISG v3 XID device scan", {}});

    struct XidExchange {
        bool activity{};
        bool invalid{};
        std::vector<hdlc::Frame> frames;
    };
    const auto exchangeXid = [&](const hdlc::Frame& request,
                                 const std::chrono::milliseconds timeout) {
        XidExchange result;
        decoder_.reset();
        const auto encoded = hdlc::encode(request);
        if (!transmit(operation, encoded, cancelled)) return result;

        const auto deadline = std::chrono::steady_clock::now() + timeout;
        bool rawActivity = false;
        bool sawEcho = false;
        while (std::chrono::steady_clock::now() < deadline && !cancelled->load()) {
            const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
                deadline - std::chrono::steady_clock::now());
            const auto slice = std::min(std::max(remaining, 1ms), 25ms);
            const auto received = transport_->read(slice);
            if (received.empty()) {
                if (rawActivity) {
                    break;
                }
                continue;
            }
            noteFrameActivity();
            rawActivity = true;
            logFrame(EventKind::rxFrame, operation, received);
            for (auto& decoded : decoder_.push(received)) {
                if (!decoded.frame) {
                    result.invalid = true;
                    continue;
                }
                if (*decoded.frame == request) {
                    sawEcho = true;
                    continue;
                }
                result.activity = true;
                result.frames.push_back(std::move(*decoded.frame));
            }
        }
        if (rawActivity && !sawEcho && result.frames.empty()) {
            result.activity = true;
        }
        if (result.invalid) {
            result.activity = true;
        }
        decoder_.reset();
        // AISG Base 11.8: after a Final response the primary must leave at
        // least 3 ms before starting its next transmission.
        if (!result.frames.empty()) {
            std::this_thread::sleep_for(3ms);
        }
        return result;
    };

    enum class QueryKind { none, found, collision };
    struct QueryResult {
        QueryKind kind{QueryKind::none};
        std::optional<aisg3::DeviceScanResponse> device;
    };
    std::size_t queryCount{};
    const auto query = [&](const aisg3::DeviceScanPattern& pattern) {
        if (++queryCount > 512) {
            throw std::runtime_error("AISG v3 XID collision resolution exceeded 512 queries");
        }
        const auto request = aisg3::makeDeviceScanCommand(pattern);
        const auto exchange = exchangeXid(request, std::max(options.responseTimeout, 250ms));
        std::vector<aisg3::DeviceScanResponse> devices;
        for (const auto& frame : exchange.frames) {
            if (const auto parsed = aisg3::parseDeviceScanResponse(frame)) {
                devices.push_back(*parsed);
            }
        }
        if (!exchange.invalid && exchange.frames.size() == 1 && devices.size() == 1) {
            return QueryResult{QueryKind::found, std::move(devices.front())};
        }
        return QueryResult{exchange.activity ? QueryKind::collision : QueryKind::none,
                           std::nullopt};
    };

    using PatternBytes = std::array<std::uint8_t, 21>;
    const auto makePattern = [](const PatternBytes& value, const PatternBytes& mask) {
        aisg3::DeviceScanPattern pattern;
        pattern.uniqueId.assign(value.begin(), value.begin() + 19);
        pattern.uniqueIdMask.assign(mask.begin(), mask.begin() + 19);
        pattern.portNumber.assign(value.begin() + 19, value.end());
        pattern.portNumberMask.assign(mask.begin() + 19, mask.end());
        return pattern;
    };

    std::function<std::optional<aisg3::DeviceScanResponse>(PatternBytes, PatternBytes, std::size_t)>
        resolveCollision;
    resolveCollision = [&](PatternBytes value, PatternBytes mask, const std::size_t firstBit)
        -> std::optional<aisg3::DeviceScanResponse> {
        for (std::size_t bit = firstBit; bit < value.size() * 8; ++bit) {
            const auto octet = bit / 8;
            const auto bitMask = static_cast<std::uint8_t>(1U << (7U - (bit % 8U)));

            auto zeroValue = value;
            auto zeroMask = mask;
            zeroMask[octet] |= bitMask;
            zeroValue[octet] &= static_cast<std::uint8_t>(~bitMask);
            const auto zero = query(makePattern(zeroValue, zeroMask));
            if (zero.kind == QueryKind::found) {
                return zero.device;
            }
            if (zero.kind == QueryKind::collision) {
                if (auto resolved = resolveCollision(zeroValue, zeroMask, bit + 1)) {
                    return resolved;
                }
            }

            auto oneValue = value;
            auto oneMask = mask;
            oneMask[octet] |= bitMask;
            oneValue[octet] |= bitMask;
            const auto one = query(makePattern(oneValue, oneMask));
            if (one.kind == QueryKind::found) {
                return one.device;
            }
            if (one.kind == QueryKind::collision) {
                if (auto resolved = resolveCollision(oneValue, oneMask, bit + 1)) {
                    return resolved;
                }
            }
        }
        return std::nullopt;
    };

    auto nextAddress = options.firstAddress;
    while (!cancelled->load() && nextAddress <= options.lastAddress) {
        queryCount = 0;
        const auto broad = query({});
        if (broad.kind == QueryKind::none) {
            break;
        }
        auto candidate = broad.device;
        if (broad.kind == QueryKind::collision) {
            candidate = resolveCollision({}, {}, 0);
        }
        if (!candidate) {
            throw std::runtime_error("AISG v3 XID collision could not be resolved");
        }

        PatternBytes exactValue{};
        PatternBytes exactMask{};
        std::copy(candidate->uniqueId.begin(), candidate->uniqueId.end(), exactValue.begin());
        exactValue[19] = static_cast<std::uint8_t>((candidate->portNumber >> 8U) & 0xFFU);
        exactValue[20] = static_cast<std::uint8_t>(candidate->portNumber & 0xFFU);
        exactMask.fill(0xFF);
        const auto confirmation = query(makePattern(exactValue, exactMask));
        if (confirmation.kind != QueryKind::found || !confirmation.device ||
            confirmation.device->uniqueId != candidate->uniqueId ||
            confirmation.device->portNumber != candidate->portNumber) {
            throw std::runtime_error("AISG v3 XID identity confirmation failed");
        }
        candidate = confirmation.device;

        if (std::find(candidate->baseVersions.begin(), candidate->baseVersions.end(),
                      aisg3::supportedBaseVersion) == candidate->baseVersions.end()) {
            throw std::runtime_error("ALD does not advertise AISG Base version 3.0.8");
        }

        aisg3::AddressAssignment assignment;
        assignment.address = nextAddress;
        assignment.primaryId = options.primaryId;
        assignment.uniqueId = candidate->uniqueId;
        assignment.aldType = candidate->aldType;
        assignment.vendorCode = candidate->vendorCode;
        assignment.portNumber = candidate->portNumber;
        const auto assignmentFrame = aisg3::makeAddressAssignmentCommand(assignment);
        const auto assignmentExchange = exchangeXid(assignmentFrame, 500ms);
        std::optional<aisg3::AddressAssignmentResponse> assignmentResponse;
        for (const auto& frame : assignmentExchange.frames) {
            if (const auto parsed = aisg3::parseAddressAssignmentResponse(frame);
                parsed && parsed->address == nextAddress && parsed->uniqueId == candidate->uniqueId &&
                parsed->portNumber == candidate->portNumber) {
                if (assignmentResponse) {
                    throw std::runtime_error("multiple ALDs accepted one AISG v3 address assignment");
                }
                assignmentResponse = parsed;
            }
        }
        if (!assignmentResponse) {
            throw std::runtime_error("AISG v3 address assignment was not acknowledged");
        }

        const auto ua = transact(operation, aisg::makeSnrm(nextAddress), 250ms, cancelled);
        if (!ua || ua->control != 0x73) {
            throw std::runtime_error("AISG v3 ALD did not establish the HDLC link with UA");
        }
        std::this_thread::sleep_for(3ms);

        Device found;
        found.address = nextAddress;
        found.protocol = ProtocolProfile::aisg3;
        found.status = DeviceStatus::initializing;
        found.uid = aisg3::uniqueIdString(candidate->uniqueId);
        found.vendor.assign(candidate->vendorCode.begin(), candidate->vendorCode.end());

        const auto informationCommand = aisg3::makeGetInformation(nextAisg3Sequence());
        const auto informationResponse = transactAisg3(
            operation, nextAddress, informationCommand, 1200ms, cancelled);
        const auto information = informationResponse
                                     ? aisg3::parseInformation(*informationResponse)
                                     : std::nullopt;
        if (!information) {
            throw std::runtime_error("AISG v3 GetInformation failed or returned malformed data");
        }
        found.product = information->productNumber;
        found.serialNumber = information->serialNumber;
        found.hardwareVersion = information->hardwareVersion;
        found.softwareVersion = information->softwareVersion;

        const auto subunitCommand = aisg3::makeGetSubunitList(nextAisg3Sequence());
        const auto subunitResponse = transactAisg3(
            operation, nextAddress, subunitCommand, 1200ms, cancelled);
        const auto subunits = subunitResponse ? aisg3::parseSubunitList(*subunitResponse)
                                              : std::nullopt;
        if (!subunits) {
            throw std::runtime_error("AISG v3 GetSubunitList failed or returned malformed data");
        }
        std::optional<std::uint16_t> adbSubunit;
        for (const auto& subunit : *subunits) {
            DeviceKind kind = DeviceKind::unknown;
            switch (subunit.type) {
            case aisg3::SubunitType::ret: kind = DeviceKind::ret; break;
            case aisg3::SubunitType::tma: kind = DeviceKind::tma; break;
            case aisg3::SubunitType::adb:
                kind = DeviceKind::adb;
                if (!adbSubunit) {
                    adbSubunit = subunit.number;
                }
                break;
            case aisg3::SubunitType::als: kind = DeviceKind::unknown; break;
            }
            found.subunits.push_back({subunit.number, kind});
        }
        const auto hasKind = [&](const DeviceKind kind) {
            return std::any_of(found.subunits.begin(), found.subunits.end(),
                               [&](const auto& subunit) { return subunit.kind == kind; });
        };
        found.kind = hasKind(DeviceKind::ret) ? DeviceKind::ret
                    : hasKind(DeviceKind::tma) ? DeviceKind::tma
                    : hasKind(DeviceKind::adb) ? DeviceKind::adb
                                               : DeviceKind::unknown;

        if (adbSubunit) {
            const auto versionsCommand = aisg3::makeGetSubunitVersions(
                nextAisg3Sequence(), aisg3::SubunitType::adb);
            const auto versionsResponse = transactAisg3(
                operation, nextAddress, versionsCommand, 1200ms, cancelled);
            const auto versions = versionsResponse
                                      ? aisg3::parseSubunitVersions(*versionsResponse)
                                      : std::nullopt;
            if (!versions ||
                std::find(versions->supported.begin(), versions->supported.end(),
                          aisg3::supportedAdbVersion) == versions->supported.end()) {
                throw std::runtime_error("ADB subunit does not advertise version 3.1.7");
            }
            const auto setVersionCommand = aisg3::makeSetSubunitVersion(
                nextAisg3Sequence(), aisg3::SubunitType::adb, aisg3::supportedAdbVersion);
            const auto setVersionResponse = transactAisg3(
                operation, nextAddress, setVersionCommand, 1200ms, cancelled);
            if (!setVersionResponse || !setVersionResponse->success()) {
                throw std::runtime_error("ADB 3.1.7 version negotiation was rejected");
            }

            const auto antennaCommand = aisg3::adb::makeGetAntennaInfo(
                nextAisg3Sequence(), *adbSubunit);
            const auto antennaResponse = transactAisg3(
                operation, nextAddress, antennaCommand, 1200ms, cancelled);
            const auto antenna = antennaResponse
                                     ? aisg3::adb::parseAntennaInfo(*antennaResponse)
                                     : std::nullopt;
            if (!antenna) {
                throw std::runtime_error("ADB GetAntennaInfo returned malformed data");
            }
            found.installation.antennaModel = antenna->modelNumber.value;
            found.installation.antennaSerial = antenna->serialNumber.value;

            const auto installationCommand = aisg3::adb::makeGetInstallationInfo(
                nextAisg3Sequence(), *adbSubunit);
            const auto installationResponse = transactAisg3(
                operation, nextAddress, installationCommand, 1200ms, cancelled);
            if (installationResponse && installationResponse->success()) {
                if (const auto installation =
                        aisg3::adb::parseInstallationInfo(*installationResponse)) {
                    found.installation.sectorId = installation->sectorId.value;
                    found.installation.location = installation->installationNotes.value;
                    found.installation.bearingDegrees =
                        static_cast<double>(installation->mechanicalAzimuthDeciDegrees) / 10.0;
                    found.installation.mechanicalTiltDegrees =
                        static_cast<double>(installation->mechanicalTiltDeciDegrees) / 10.0;
                }
            }
        }

        found.status = DeviceStatus::ready;
        storeDevice(std::move(found), operation, true);
        const auto completed = static_cast<unsigned int>(nextAddress - options.firstAddress) + 1U;
        const auto total = static_cast<unsigned int>(options.lastAddress - options.firstAddress) + 1U;
        emit({EventKind::scanProgress, operation, std::chrono::system_clock::now(),
              nextAddress, std::nullopt, static_cast<int>((completed * 100U) / total),
              "AISG v3 ALD assigned and negotiated", {}});
        ++nextAddress;
    }

    if (nextAddress > options.lastAddress) {
        emit({EventKind::log, operation, std::chrono::system_clock::now(), std::nullopt,
              std::nullopt, 100, "AISG v3 address pool exhausted", {}});
    }
}

OperationId ControllerService::refresh(const std::uint8_t address) {
    return enqueue("Refresh device", [this, address](
                                              const OperationId operation,
                                              const std::shared_ptr<std::atomic_bool>& cancelled) {
        auto current = requireDevice(address);
        if (current.protocol == ProtocolProfile::aisg3) {
            const auto informationCommand = aisg3::makeGetInformation(nextAisg3Sequence());
            const auto informationResponse = transactAisg3(
                operation, address, informationCommand, std::chrono::milliseconds(1200), cancelled);
            const auto information = informationResponse
                                         ? aisg3::parseInformation(*informationResponse)
                                         : std::nullopt;
            if (!information) {
                throw std::runtime_error("AISG v3 GetInformation failed");
            }
            current.product = information->productNumber;
            current.serialNumber = information->serialNumber;
            current.hardwareVersion = information->hardwareVersion;
            current.softwareVersion = information->softwareVersion;

            const auto adbSubunit = std::find_if(
                current.subunits.begin(), current.subunits.end(),
                [](const auto& subunit) { return subunit.kind == DeviceKind::adb; });
            if (adbSubunit != current.subunits.end()) {
                const auto antennaCommand = aisg3::adb::makeGetAntennaInfo(
                    nextAisg3Sequence(), adbSubunit->number);
                const auto antennaResponse = transactAisg3(
                    operation, address, antennaCommand, std::chrono::milliseconds(1200), cancelled);
                if (antennaResponse) {
                    if (const auto antenna = aisg3::adb::parseAntennaInfo(*antennaResponse)) {
                        current.installation.antennaModel = antenna->modelNumber.value;
                        current.installation.antennaSerial = antenna->serialNumber.value;
                    }
                }
                const auto installationCommand = aisg3::adb::makeGetInstallationInfo(
                    nextAisg3Sequence(), adbSubunit->number);
                const auto installationResponse = transactAisg3(
                    operation, address, installationCommand,
                    std::chrono::milliseconds(1200), cancelled);
                if (installationResponse) {
                    if (const auto installation =
                            aisg3::adb::parseInstallationInfo(*installationResponse)) {
                        current.installation.sectorId = installation->sectorId.value;
                        current.installation.location = installation->installationNotes.value;
                        current.installation.bearingDegrees =
                            static_cast<double>(installation->mechanicalAzimuthDeciDegrees) / 10.0;
                        current.installation.mechanicalTiltDegrees =
                            static_cast<double>(installation->mechanicalTiltDeciDegrees) / 10.0;
                    }
                }
            }
            current.status = current.alarms.empty() ? DeviceStatus::ready : DeviceStatus::alarm;
            storeDevice(std::move(current), operation);
            return;
        }
        const auto timeout = std::chrono::milliseconds(250);
        const auto initialRequest = aisg::makeInitialDataRequest(address, nextControl(address));
        const auto initialResponse = transact(operation, initialRequest, timeout, cancelled);
        if (!initialResponse) {
            throw std::runtime_error("device information request timed out");
        }
        const auto initial = aisg::parseInitialData(*initialResponse);
        if (!initial || !initial->status.success) {
            throw std::runtime_error("device returned invalid initial data");
        }
        current.product = initial->product;
        current.serialNumber = initial->serialNumber;
        current.hardwareVersion = initial->hardwareVersion;
        current.softwareVersion = initial->softwareVersion;

        if (auto* ret = current.ret()) {
            const auto tiltRequest = aisg::makeGetTiltRequest(address, nextControl(address));
            const auto tiltResponse = transact(operation, tiltRequest, timeout, cancelled);
            if (!tiltResponse) {
                throw std::runtime_error("tilt request timed out");
            }
            if (const auto tilt = aisg::parseTilt(*tiltResponse)) {
                ret->electricalTiltDegrees = *tilt;
            }
        }
        current.status = DeviceStatus::ready;
        storeDevice(std::move(current), operation);
    });
}

OperationId ControllerService::moveRet(const std::uint8_t address, const double tiltDegrees) {
    return enqueue("Move RET", [this, address, tiltDegrees](
                                         const OperationId operation,
                                         const std::shared_ptr<std::atomic_bool>& cancelled) {
        auto current = requireDevice(address);
        if (current.protocol == ProtocolProfile::aisg3) {
            throw std::runtime_error(
                "RET AISG v3 write is blocked until the AISG-ST-RET profile is implemented");
        }
        auto* ret = current.ret();
        if (!ret) {
            throw std::invalid_argument("selected device is not a RET");
        }
        if (!ret->acceptsTilt(tiltDegrees)) {
            throw std::invalid_argument("requested tilt is outside RET limits");
        }

        const auto simulatedTravel = std::chrono::milliseconds(
            static_cast<std::int64_t>(std::ceil(
                std::abs(tiltDegrees - ret->electricalTiltDegrees) * 2000.0)));
        const auto request = aisg::makeSetTiltRequest(address, nextControl(address), tiltDegrees);
        const auto response = transact(
            operation, request,
            std::max(std::chrono::milliseconds(3000),
                     simulatedTravel + std::chrono::milliseconds(3000)),
            cancelled);
        if (!response) {
            throw std::runtime_error("RET move timed out");
        }
        const auto status = aisg::parseStatus(*response, aisg::Command::setTilt);
        if (!status.success) {
            throw std::runtime_error("RET rejected move: " + status.message);
        }
        ret->electricalTiltDegrees = std::round(tiltDegrees * 10.0) / 10.0;
        ret->moving = false;
        current.status = current.alarms.empty() ? DeviceStatus::ready : DeviceStatus::alarm;
        storeDevice(std::move(current), operation);
    });
}

OperationId ControllerService::refreshAlarms(const std::uint8_t address) {
    return enqueue("Get alarms", [this, address](
                                          const OperationId operation,
                                          const std::shared_ptr<std::atomic_bool>& cancelled) {
        auto current = requireDevice(address);
        if (current.protocol == ProtocolProfile::aisg3) {
            current.alarms.clear();
            const auto readAlarms = [&](const std::uint16_t subunit) {
                const auto command = aisg3::makeGetAlarmStatus(nextAisg3Sequence(), subunit);
                const auto response = transactAisg3(
                    operation, address, command, std::chrono::milliseconds(1200), cancelled);
                const auto alarms = response ? aisg3::parseAlarmStatus(*response) : std::nullopt;
                if (!alarms) {
                    throw std::runtime_error("AISG v3 GetAlarmStatus failed");
                }
                for (const auto& alarm : *alarms) {
                    const auto severity = alarm.severity >= 3 ? AlarmSeverity::critical
                                           : alarm.severity == 0 ? AlarmSeverity::information
                                                                 : AlarmSeverity::warning;
                    current.alarms.push_back(
                        {alarm.code, severity,
                         "AISG v3 alarm 0x" + hex16(alarm.code),
                         true, std::chrono::system_clock::now()});
                }
            };
            readAlarms(0);
            for (const auto& subunit : current.subunits) {
                readAlarms(subunit.number);
            }
            current.status = current.alarms.empty() ? DeviceStatus::ready : DeviceStatus::alarm;
            storeDevice(std::move(current), operation);
            return;
        }
        const auto request = aisg::makeGetAlarmsRequest(address, nextControl(address));
        const auto response = transact(operation, request, std::chrono::milliseconds(500), cancelled);
        if (!response) {
            throw std::runtime_error("alarm request timed out");
        }
        const auto alarms = aisg::parseAlarms(*response);
        if (!alarms || !alarms->status.success) {
            throw std::runtime_error("device returned invalid alarm data");
        }
        current.alarms = alarms->alarms;
        current.status = current.alarms.empty() ? DeviceStatus::ready : DeviceStatus::alarm;
        storeDevice(std::move(current), operation);
    });
}

OperationId ControllerService::clearAlarms(const std::uint8_t address) {
    return enqueue("Clear alarms", [this, address](
                                            const OperationId operation,
                                            const std::shared_ptr<std::atomic_bool>& cancelled) {
        auto current = requireDevice(address);
        if (current.protocol == ProtocolProfile::aisg3) {
            const auto clear = [&](const std::uint16_t subunit) {
                const auto command = aisg3::makeClearActiveAlarms(nextAisg3Sequence(), subunit);
                const auto response = transactAisg3(
                    operation, address, command, std::chrono::milliseconds(1200), cancelled);
                if (!response || !response->success()) {
                    throw std::runtime_error("AISG v3 ClearActiveAlarms was rejected");
                }
            };
            clear(0);
            for (const auto& subunit : current.subunits) {
                clear(subunit.number);
            }
            current.alarms.clear();
            current.status = DeviceStatus::ready;
            storeDevice(std::move(current), operation);
            return;
        }
        const auto request = aisg::makeClearAlarmsRequest(address, nextControl(address));
        const auto response = transact(operation, request, std::chrono::milliseconds(500), cancelled);
        if (!response) {
            throw std::runtime_error("clear alarms timed out");
        }
        const auto status = aisg::parseStatus(*response, aisg::Command::clearAlarms);
        if (!status.success) {
            throw std::runtime_error("device rejected clear alarms: " + status.message);
        }
        current.alarms.clear();
        current.status = DeviceStatus::ready;
        storeDevice(std::move(current), operation);
    });
}

OperationId ControllerService::runSelfTest(const std::uint8_t address) {
    return enqueue("Self test", [this, address](
                                        const OperationId operation,
                                        const std::shared_ptr<std::atomic_bool>& cancelled) {
        auto current = requireDevice(address);
        if (current.protocol == ProtocolProfile::aisg3) {
            throw std::runtime_error(
                "AISG v3 self-test requires the applicable subunit type standard");
        }
        const auto request = aisg::makeSelfTestRequest(address, nextControl(address));
        const auto response = transact(operation, request, std::chrono::milliseconds(1000), cancelled);
        if (!response) {
            throw std::runtime_error("self test timed out");
        }
        const auto status = aisg::parseStatus(*response, aisg::Command::selfTest);
        if (!status.success) {
            throw std::runtime_error("self test failed: " + status.message);
        }
        current.status = DeviceStatus::ready;
        storeDevice(std::move(current), operation);
    });
}

OperationId ControllerService::calibrate(const std::uint8_t address) {
    return enqueue("Calibrate", [this, address](
                                        const OperationId operation,
                                        const std::shared_ptr<std::atomic_bool>& cancelled) {
        auto current = requireDevice(address);
        if (current.protocol == ProtocolProfile::aisg3) {
            throw std::runtime_error(
                "RET AISG v3 calibration is blocked until AISG-ST-RET is implemented");
        }
        auto* ret = current.ret();
        if (!ret) {
            throw std::invalid_argument("selected device is not a RET");
        }
        const auto request = aisg::makeCalibrateRequest(address, nextControl(address));
        const auto response = transact(operation, request, std::chrono::milliseconds(2000), cancelled);
        if (!response) {
            throw std::runtime_error("calibration timed out");
        }
        const auto status = aisg::parseStatus(*response, aisg::Command::calibrate);
        if (!status.success) {
            throw std::runtime_error("calibration failed: " + status.message);
        }
        ret->calibrated = true;
        current.status = DeviceStatus::ready;
        storeDevice(std::move(current), operation);
    });
}

OperationId ControllerService::setTmaGain(const std::uint8_t address, const double gainDb) {
    return enqueue("Set TMA gain", [this, address, gainDb](
                                          const OperationId operation,
                                          const std::shared_ptr<std::atomic_bool>& cancelled) {
        auto current = requireDevice(address);
        if (current.protocol == ProtocolProfile::aisg3) {
            throw std::runtime_error(
                "TMA AISG v3 write is blocked until AISG-ST-TMA is implemented");
        }
        auto* tma = current.tma();
        if (!tma) {
            throw std::invalid_argument("selected device is not a TMA");
        }
        if (!std::isfinite(gainDb) || gainDb < tma->minimumGainDb || gainDb > tma->maximumGainDb) {
            throw std::invalid_argument("requested gain is outside TMA limits");
        }
        const auto rounded = std::round(gainDb * 4.0) / 4.0;
        const auto request = aisg::makeSetTmaGainRequest(address, nextControl(address), rounded);
        const auto response = transact(operation, request, std::chrono::milliseconds(500), cancelled);
        if (!response) {
            throw std::runtime_error("TMA gain request timed out");
        }
        const auto status = aisg::parseStatus(*response, aisg::Command::setTmaGain);
        if (!status.success) {
            throw std::runtime_error("TMA rejected gain: " + status.message);
        }
        tma->gainDb = rounded;
        current.status = current.alarms.empty() ? DeviceStatus::ready : DeviceStatus::alarm;
        storeDevice(std::move(current), operation);
    });
}

OperationId ControllerService::setTmaMode(const std::uint8_t address, const TmaMode mode) {
    return enqueue("Set TMA mode", [this, address, mode](
                                          const OperationId operation,
                                          const std::shared_ptr<std::atomic_bool>& cancelled) {
        auto current = requireDevice(address);
        if (current.protocol == ProtocolProfile::aisg3) {
            throw std::runtime_error(
                "TMA AISG v3 write is blocked until AISG-ST-TMA is implemented");
        }
        auto* tma = current.tma();
        if (!tma) {
            throw std::invalid_argument("selected device is not a TMA");
        }
        const auto request = aisg::makeSetTmaModeRequest(address, nextControl(address), mode);
        const auto response = transact(operation, request, std::chrono::milliseconds(500), cancelled);
        if (!response) {
            throw std::runtime_error("TMA mode request timed out");
        }
        const auto status = aisg::parseStatus(*response, aisg::Command::setTmaMode);
        if (!status.success) {
            throw std::runtime_error("TMA rejected mode: " + status.message);
        }
        tma->mode = mode;
        current.status = current.alarms.empty() ? DeviceStatus::ready : DeviceStatus::alarm;
        storeDevice(std::move(current), operation);
    });
}

OperationId ControllerService::setDeviceField(const std::uint8_t address,
                                               const aisg::Field field,
                                               hdlc::Bytes value) {
    return enqueue("Set device data", [this, address, field, value = std::move(value)](
                                               const OperationId operation,
                                               const std::shared_ptr<std::atomic_bool>& cancelled) {
        auto current = requireDevice(address);
        if (current.protocol == ProtocolProfile::aisg3) {
            throw std::runtime_error(
                "legacy SetDeviceData is not valid for AISG v3; ADB writes remain blocked in the GUI");
        }
        const auto request = aisg::makeSetDataRequest(address, nextControl(address), field, value);
        const auto response = transact(operation, request, std::chrono::milliseconds(500), cancelled);
        if (!response) {
            throw std::runtime_error("set device data timed out");
        }
        const auto status = aisg::parseStatus(*response, aisg::Command::setDeviceData);
        if (!status.success) {
            throw std::runtime_error("device rejected configuration: " + status.message);
        }
        const std::string textValue(value.begin(), value.end());
        switch (field) {
        case aisg::Field::antennaModel: current.installation.antennaModel = textValue; break;
        case aisg::Field::installationDate: current.installation.installationDate = textValue; break;
        case aisg::Field::installerId: current.installation.installerId = textValue; break;
        case aisg::Field::baseStationId: current.installation.baseStationId = textValue; break;
        case aisg::Field::sectorId: current.installation.sectorId = textValue; break;
        default: break;
        }
        storeDevice(std::move(current), operation);
    });
}

void ControllerService::cancel(const OperationId operation) noexcept {
    std::scoped_lock lock(queueMutex_);
    if (const auto iterator = cancellation_.find(operation); iterator != cancellation_.end()) {
        iterator->second->store(true);
    }
}

bool ControllerService::isConnected() const noexcept {
    std::scoped_lock lock(stateMutex_);
    return connected_;
}

bool ControllerService::isBusy() const noexcept {
    std::scoped_lock stateLock(stateMutex_);
    std::scoped_lock queueLock(queueMutex_);
    return activeWork_ || !workQueue_.empty();
}

std::vector<Device> ControllerService::devices() const {
    std::scoped_lock lock(stateMutex_);
    std::vector<Device> result;
    result.reserve(devices_.size());
    for (const auto& [address, device] : devices_) {
        (void)address;
        result.push_back(device);
    }
    std::sort(result.begin(), result.end(), [](const Device& left, const Device& right) {
        return left.address < right.address;
    });
    return result;
}

std::optional<Device> ControllerService::device(const std::uint8_t address) const {
    std::scoped_lock lock(stateMutex_);
    const auto iterator = devices_.find(address);
    return iterator == devices_.end() ? std::nullopt : std::optional<Device>(iterator->second);
}

std::vector<ControllerEvent> ControllerService::drainEvents() {
    std::scoped_lock lock(eventMutex_);
    auto result = std::move(events_);
    events_.clear();
    return result;
}

OperationId ControllerService::enqueue(std::string name, Task task) {
    const auto id = nextOperation_.fetch_add(1);
    auto cancelled = std::make_shared<std::atomic_bool>(false);
    {
        std::scoped_lock lock(queueMutex_);
        cancellation_[id] = cancelled;
        workQueue_.push_back({id, std::move(name), std::move(task), std::move(cancelled)});
    }
    queueCondition_.notify_all();
    return id;
}

void ControllerService::workerLoop(const std::stop_token stopToken) {
    while (!stopToken.stop_requested()) {
        Work work;
        {
            std::unique_lock lock(queueMutex_);
            if (!queueCondition_.wait(lock, stopToken, [this] { return !workQueue_.empty(); })) {
                break;
            }
            work = std::move(workQueue_.front());
            workQueue_.pop_front();
        }
        {
            std::scoped_lock lock(stateMutex_);
            activeWork_ = true;
        }
        emit({EventKind::operationStarted, work.id, std::chrono::system_clock::now(),
              std::nullopt, std::nullopt, -1, work.name, {}});
        try {
            if (!work.cancelled->load()) {
                work.task(work.id, work.cancelled);
            }
            emit({work.cancelled->load() ? EventKind::operationCancelled
                                         : EventKind::operationCompleted,
                  work.id, std::chrono::system_clock::now(), std::nullopt, std::nullopt,
                  -1, work.cancelled->load() ? work.name + " cancelled" : work.name + " completed", {}});
        } catch (const std::exception& exception) {
            emit({EventKind::operationFailed, work.id, std::chrono::system_clock::now(),
                  std::nullopt, std::nullopt, -1, exception.what(), {}});
        } catch (...) {
            emit({EventKind::operationFailed, work.id, std::chrono::system_clock::now(),
                  std::nullopt, std::nullopt, -1, "unknown controller error", {}});
        }
        {
            std::scoped_lock lock(queueMutex_);
            cancellation_.erase(work.id);
        }
        {
            std::scoped_lock lock(stateMutex_);
            activeWork_ = false;
        }
    }
}

void ControllerService::emit(ControllerEvent event) {
    std::scoped_lock lock(eventMutex_);
    events_.push_back(std::move(event));
}

void ControllerService::logFrame(const EventKind kind,
                                 const OperationId operation,
                                 const hdlc::Bytes& encoded) {
    emit({kind, operation, std::chrono::system_clock::now(), std::nullopt, std::nullopt,
          -1, hdlc::toHex(encoded), encoded});
}

bool ControllerService::transmit(
    const OperationId operation,
    const hdlc::Bytes& encoded,
    const std::shared_ptr<std::atomic_bool>& cancelled) {
    while (!cancelled->load() && lastFrameActivity_) {
        const auto allowedAt = *lastFrameActivity_ + minimumFrameInterval_;
        const auto now = std::chrono::steady_clock::now();
        if (now >= allowedAt) break;
        const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
            allowedAt - now);
        std::this_thread::sleep_for(std::min(remaining + std::chrono::milliseconds(1),
                                             std::chrono::milliseconds(10)));
    }
    if (cancelled->load()) return false;
    logFrame(EventKind::txFrame, operation, encoded);
    transport_->write(encoded);
    noteFrameActivity();
    return true;
}

void ControllerService::noteFrameActivity() noexcept {
    lastFrameActivity_ = std::chrono::steady_clock::now();
}

std::optional<hdlc::Frame> ControllerService::transact(
    const OperationId operation,
    const hdlc::Frame& request,
    const std::chrono::milliseconds timeout,
    const std::shared_ptr<std::atomic_bool>& cancelled) {
    if (!transport_->isOpen()) {
        throw TransportError("transport is closed");
    }
    const auto encoded = hdlc::encode(request);
    if (!transmit(operation, encoded, cancelled)) return std::nullopt;

    // An accepted long-running Layer-7 command may first produce RR. Keep
    // polling at the configured bus pace until its Layer-7 result is ready.
    const auto deadline = std::chrono::steady_clock::now() + timeout +
                          minimumFrameInterval_ + std::chrono::milliseconds(100);
    while (std::chrono::steady_clock::now() < deadline && !cancelled->load()) {
        const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
            deadline - std::chrono::steady_clock::now());
        const auto bytes = transport_->read(std::max(remaining, std::chrono::milliseconds(1)));
        if (bytes.empty()) {
            continue;
        }
        noteFrameActivity();
        logFrame(EventKind::rxFrame, operation, bytes);
        for (auto& result : decoder_.push(bytes)) {
            if (!result.frame) {
                emit({EventKind::log, operation, std::chrono::system_clock::now(),
                      std::nullopt, std::nullopt, -1, "Discarded HDLC frame: " + result.message, {}});
                continue;
            }
            if (aisg::isResponseFor(*result.frame, request)) {
                return std::move(*result.frame);
            }
            const auto& frame = *result.frame;
            const bool receiveReady = frame.address == request.address &&
                                      frame.information.empty() &&
                                      (frame.control & 0x0FU) == 0x01U &&
                                      (frame.control & 0x10U) != 0U;
            const auto acknowledged = static_cast<std::uint8_t>(
                ((((request.control >> 1U) & 7U) + 1U) & 7U));
            if (receiveReady && ((frame.control >> 5U) & 7U) == acknowledged) {
                // The RET accepted the primary I-frame but its asynchronous
                // Layer-7 result is not ready yet. Poll with RR while keeping
                // N(R) at the next secondary I-frame expected by the primary.
                const auto nextPollAt =
                    lastFrameActivity_.value_or(std::chrono::steady_clock::now()) +
                    minimumFrameInterval_;
                if (nextPollAt >= deadline) {
                    continue;
                }
                const auto pollControl = static_cast<std::uint8_t>(
                    (request.control & 0xE0U) | 0x11U);
                const auto poll = hdlc::encode(
                    aisg::makeKeepAlive(request.address, pollControl));
                if (!transmit(operation, poll, cancelled)) return std::nullopt;
                continue;
            }
            emit({EventKind::log, operation, std::chrono::system_clock::now(),
                  result.frame->address, std::nullopt, -1,
                  "Received an unrelated or unsolicited AISG frame", {}});
        }
    }
    return std::nullopt;
}

std::optional<aisg3::Response> ControllerService::transactAisg3(
    const OperationId operation,
    const std::uint8_t address,
    const aisg3::PrimaryCommand& command,
    const std::chrono::milliseconds timeout,
    const std::shared_ptr<std::atomic_bool>& cancelled) {
    if (!transport_->isOpen()) {
        throw TransportError("transport is closed");
    }
    const auto request = aisg3::makeCommandFrame(address, nextControl(address), command);
    const auto encoded = hdlc::encode(request);
    if (!transmit(operation, encoded, cancelled)) return std::nullopt;

    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline && !cancelled->load()) {
        const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
            deadline - std::chrono::steady_clock::now());
        const auto received = transport_->read(std::max(remaining, std::chrono::milliseconds(1)));
        if (received.empty()) {
            continue;
        }
        noteFrameActivity();
        logFrame(EventKind::rxFrame, operation, received);
        for (auto& decoded : decoder_.push(received)) {
            if (!decoded.frame) {
                emit({EventKind::log, operation, std::chrono::system_clock::now(),
                      std::nullopt, std::nullopt, -1,
                      "Discarded AISG v3 HDLC frame: " + decoded.message, {}});
                continue;
            }
            if (decoded.frame->address == request.address &&
                decoded.frame->control == static_cast<std::uint8_t>(request.control + 0x20U)) {
                if (auto response = aisg3::parseResponse(
                        *decoded.frame, command.command, command.sequence)) {
                    // The ALD I-frame carries F=1. Keep the normative minimum
                    // turnaround before any following poll/command.
                    std::this_thread::sleep_for(std::chrono::milliseconds(3));
                    return response;
                }
            }
            emit({EventKind::log, operation, std::chrono::system_clock::now(),
                  decoded.frame->address, std::nullopt, -1,
                  "Received an unrelated, unsolicited or malformed AISG v3 frame", {}});
        }
    }
    return std::nullopt;
}

Device ControllerService::requireDevice(const std::uint8_t address) const {
    if (!isConnected()) {
        throw std::runtime_error("controller is disconnected");
    }
    std::scoped_lock lock(stateMutex_);
    const auto iterator = devices_.find(address);
    if (iterator == devices_.end()) {
        throw std::out_of_range("AISG address is not present in the device list");
    }
    return iterator->second;
}

void ControllerService::storeDevice(Device device,
                                    const OperationId operation,
                                    const bool added) {
    ++device.revision;
    const auto address = device.address;
    {
        std::scoped_lock lock(stateMutex_);
        devices_[address] = device;
    }
    emit({added ? EventKind::deviceAdded : EventKind::deviceUpdated,
          operation, std::chrono::system_clock::now(), address, std::move(device),
          -1, added ? "Device discovered" : "Device updated", {}});
}

std::uint8_t ControllerService::nextControl(const std::uint8_t address) {
    std::scoped_lock lock(stateMutex_);
    return sequences_[address].nextRequest();
}

std::uint16_t ControllerService::nextAisg3Sequence() noexcept {
    aisg3Sequence_ = static_cast<std::uint16_t>(aisg3Sequence_ + 1U);
    return aisg3Sequence_;
}

const char* toString(const EventKind value) noexcept {
    switch (value) {
    case EventKind::operationStarted: return "Operation started";
    case EventKind::operationCompleted: return "Operation completed";
    case EventKind::operationCancelled: return "Operation cancelled";
    case EventKind::operationFailed: return "Operation failed";
    case EventKind::connectionChanged: return "Connection changed";
    case EventKind::scanProgress: return "Scan progress";
    case EventKind::deviceAdded: return "Device added";
    case EventKind::deviceUpdated: return "Device updated";
    case EventKind::devicesCleared: return "Devices cleared";
    case EventKind::txFrame: return "TX";
    case EventKind::rxFrame: return "RX";
    case EventKind::log: return "Log";
    }
    return "Unknown";
}

} // namespace atc
