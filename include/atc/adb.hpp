#pragma once

#include "atc/aisg3.hpp"

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace atc::aisg3::adb {

struct TextValue {
    std::string value;
    Provenance provenance{Provenance::notSet};
};

struct AntennaInfo {
    TextValue modelNumber;
    TextValue serialNumber;
    std::uint8_t numberOfArrayElements{};
    Provenance numberOfArrayElementsProvenance{Provenance::notSet};
};

struct AntennaPortInfo {
    std::vector<std::uint16_t> arrayElements;
    Provenance provenance{Provenance::notSet};
};

enum class LinkDescriptor : std::uint8_t {
    uplink = 1,
    downlink = 2,
    bidirectional = 3,
};

struct FrequencyRange {
    LinkDescriptor link{LinkDescriptor::bidirectional};
    std::uint32_t minimumKhz{};
    std::uint32_t maximumKhz{};
};

struct ArrayElementInfo {
    std::uint16_t azimuthBeamwidthDeciDegrees{};
    Provenance azimuthBeamwidthProvenance{Provenance::notSet};
    std::uint16_t gainDeciDbi{};
    Provenance gainProvenance{Provenance::notSet};
    std::vector<FrequencyRange> frequencies;
    Provenance frequenciesProvenance{Provenance::notSet};
    std::uint8_t polarisation{};
    Provenance polarisationProvenance{Provenance::notSet};
    std::uint8_t observationReference{};
    Provenance observationReferenceProvenance{Provenance::notSet};
    std::uint8_t polarisationSenseReference{};
    Provenance polarisationSenseReferenceProvenance{Provenance::notSet};
};

struct InstallationInfo {
    TextValue sectorId;
    TextValue installationNotes;
    std::uint16_t mechanicalAzimuthDeciDegrees{};
    Provenance mechanicalAzimuthProvenance{Provenance::notSet};
    std::int16_t mechanicalTiltDeciDegrees{};
    Provenance mechanicalTiltProvenance{Provenance::notSet};
};

struct InstallationUpdate {
    std::optional<TextValue> sectorId;
    std::optional<TextValue> installationNotes;
    std::optional<std::uint16_t> mechanicalAzimuthDeciDegrees;
    Provenance mechanicalAzimuthProvenance{Provenance::manual};
    std::optional<std::int16_t> mechanicalTiltDeciDegrees;
    Provenance mechanicalTiltProvenance{Provenance::manual};
};

struct RfPathIds {
    std::vector<std::uint16_t> values;
    Provenance provenance{Provenance::notSet};
};

[[nodiscard]] PrimaryCommand makeGetAntennaInfo(std::uint16_t sequence,
                                                std::uint16_t subunit);
[[nodiscard]] std::optional<AntennaInfo> parseAntennaInfo(const Response& response);

[[nodiscard]] PrimaryCommand makeGetAntennaPortInfo(std::uint16_t sequence,
                                                    std::uint16_t subunit,
                                                    std::uint16_t portNumber);
[[nodiscard]] std::optional<AntennaPortInfo> parseAntennaPortInfo(const Response& response);

[[nodiscard]] PrimaryCommand makeGetArrayElementInfo(std::uint16_t sequence,
                                                     std::uint16_t subunit,
                                                     std::uint16_t arrayElementNumber);
[[nodiscard]] std::optional<ArrayElementInfo> parseArrayElementInfo(const Response& response);

[[nodiscard]] PrimaryCommand makeSetInstallationInfo(std::uint16_t sequence,
                                                     std::uint16_t subunit,
                                                     const InstallationUpdate& update);
[[nodiscard]] PrimaryCommand makeGetInstallationInfo(std::uint16_t sequence,
                                                     std::uint16_t subunit);
[[nodiscard]] std::optional<InstallationInfo> parseInstallationInfo(const Response& response);

[[nodiscard]] PrimaryCommand makeSetRfPathIds(std::uint16_t sequence,
                                              std::uint16_t subunit,
                                              std::uint16_t arrayElementNumber,
                                              const RfPathIds& ids);
[[nodiscard]] PrimaryCommand makeGetRfPathIds(std::uint16_t sequence,
                                              std::uint16_t subunit,
                                              std::uint32_t primaryId,
                                              std::uint16_t arrayElementNumber);
[[nodiscard]] std::optional<RfPathIds> parseRfPathIds(const Response& response);

} // namespace atc::aisg3::adb
