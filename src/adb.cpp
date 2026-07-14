#include "atc/adb.hpp"

#include <stdexcept>

namespace atc::aisg3::adb {
namespace {

void appendLe16(hdlc::Bytes& target, const std::uint16_t value) {
    target.push_back(static_cast<std::uint8_t>(value & 0xFFU));
    target.push_back(static_cast<std::uint8_t>((value >> 8U) & 0xFFU));
}

void appendLe32(hdlc::Bytes& target, const std::uint32_t value) {
    target.push_back(static_cast<std::uint8_t>(value & 0xFFU));
    target.push_back(static_cast<std::uint8_t>((value >> 8U) & 0xFFU));
    target.push_back(static_cast<std::uint8_t>((value >> 16U) & 0xFFU));
    target.push_back(static_cast<std::uint8_t>((value >> 24U) & 0xFFU));
}

std::optional<std::uint16_t> readLe16(const hdlc::Bytes& data, const std::size_t offset) {
    if (offset > data.size() || data.size() - offset < 2) {
        return std::nullopt;
    }
    return static_cast<std::uint16_t>(data[offset]) |
           (static_cast<std::uint16_t>(data[offset + 1]) << 8U);
}

std::optional<std::uint32_t> readLe32(const hdlc::Bytes& data, const std::size_t offset) {
    if (offset > data.size() || data.size() - offset < 4) {
        return std::nullopt;
    }
    return static_cast<std::uint32_t>(data[offset]) |
           (static_cast<std::uint32_t>(data[offset + 1]) << 8U) |
           (static_cast<std::uint32_t>(data[offset + 2]) << 16U) |
           (static_cast<std::uint32_t>(data[offset + 3]) << 24U);
}

bool validProvenance(const std::uint8_t value) {
    return value <= static_cast<std::uint8_t>(Provenance::manual);
}

bool writableProvenance(const Provenance value) {
    return value == Provenance::manual || value == Provenance::automatic;
}

bool validTextString(const std::string& value) {
    for (const auto character : value) {
        const auto octet = static_cast<unsigned char>(character);
        if (octet < 0x20 || octet > 0x7E) {
            return false;
        }
    }
    return true;
}

bool validUtf8(const std::string& value) {
    const auto* data = reinterpret_cast<const unsigned char*>(value.data());
    std::size_t index{};
    while (index < value.size()) {
        const auto first = data[index++];
        if (first <= 0x7F) {
            continue;
        }
        std::size_t continuation{};
        std::uint32_t codepoint{};
        if (first >= 0xC2 && first <= 0xDF) {
            continuation = 1;
            codepoint = first & 0x1FU;
        } else if (first >= 0xE0 && first <= 0xEF) {
            continuation = 2;
            codepoint = first & 0x0FU;
        } else if (first >= 0xF0 && first <= 0xF4) {
            continuation = 3;
            codepoint = first & 0x07U;
        } else {
            return false;
        }
        if (continuation > value.size() - index) {
            return false;
        }
        for (std::size_t count = 0; count < continuation; ++count) {
            const auto next = data[index++];
            if ((next & 0xC0U) != 0x80U) {
                return false;
            }
            codepoint = (codepoint << 6U) | (next & 0x3FU);
        }
        if ((continuation == 2 && codepoint < 0x800U) ||
            (continuation == 3 && codepoint < 0x10000U) ||
            (codepoint >= 0xD800U && codepoint <= 0xDFFFU) || codepoint > 0x10FFFFU) {
            return false;
        }
    }
    return true;
}

std::optional<TextValue> readText(const hdlc::Bytes& data,
                                  std::size_t& offset,
                                  const std::size_t maximum,
                                  const bool textString) {
    if (offset >= data.size()) {
        return std::nullopt;
    }
    const auto length = static_cast<std::size_t>(data[offset++]);
    if (length > maximum || length > data.size() - offset ||
        data.size() - offset - length < 1) {
        return std::nullopt;
    }
    TextValue result;
    result.value.assign(data.begin() + static_cast<std::ptrdiff_t>(offset),
                        data.begin() + static_cast<std::ptrdiff_t>(offset + length));
    offset += length;
    if (!validProvenance(data[offset]) ||
        (textString ? !validTextString(result.value) : !validUtf8(result.value))) {
        return std::nullopt;
    }
    result.provenance = static_cast<Provenance>(data[offset++]);
    return result;
}

void appendText(hdlc::Bytes& target,
                const TextValue& text,
                const std::size_t maximum,
                const bool asciiOnly) {
    if (text.value.size() > maximum || text.value.size() > 255 ||
        !writableProvenance(text.provenance) ||
        (asciiOnly ? !validTextString(text.value) : !validUtf8(text.value))) {
        throw std::invalid_argument("ADB text value or provenance is invalid");
    }
    target.push_back(static_cast<std::uint8_t>(text.value.size()));
    target.insert(target.end(), text.value.begin(), text.value.end());
    target.push_back(static_cast<std::uint8_t>(text.provenance));
}

bool isExpected(const Response& response, const Command command) {
    return response.success() && response.command == command;
}

} // namespace

PrimaryCommand makeGetAntennaInfo(const std::uint16_t sequence,
                                  const std::uint16_t subunit) {
    if (subunit == 0) {
        throw std::invalid_argument("ADB commands require a non-zero subunit");
    }
    return {Command::adbGetAntennaInfo, sequence, subunit, {}};
}

std::optional<AntennaInfo> parseAntennaInfo(const Response& response) {
    if (!isExpected(response, Command::adbGetAntennaInfo)) {
        return std::nullopt;
    }
    std::size_t offset{};
    const auto model = readText(response.data, offset, 32, false);
    const auto serial = readText(response.data, offset, 32, false);
    if (!model || !serial || response.data.size() - offset != 2 ||
        !validProvenance(response.data[offset + 1])) {
        return std::nullopt;
    }
    return AntennaInfo{*model, *serial, response.data[offset],
                       static_cast<Provenance>(response.data[offset + 1])};
}

PrimaryCommand makeGetAntennaPortInfo(const std::uint16_t sequence,
                                      const std::uint16_t subunit,
                                      const std::uint16_t portNumber) {
    if (subunit == 0 || portNumber == 0) {
        throw std::invalid_argument("ADB port-info requires non-zero subunit and port");
    }
    hdlc::Bytes data;
    appendLe16(data, portNumber);
    return {Command::adbGetAntennaPortInfo, sequence, subunit, std::move(data)};
}

std::optional<AntennaPortInfo> parseAntennaPortInfo(const Response& response) {
    if (!isExpected(response, Command::adbGetAntennaPortInfo) || response.data.size() < 2) {
        return std::nullopt;
    }
    const auto count = static_cast<std::size_t>(response.data[0]);
    if (response.data.size() != 2 + count * 2 || !validProvenance(response.data.back())) {
        return std::nullopt;
    }
    AntennaPortInfo result;
    result.arrayElements.reserve(count);
    for (std::size_t index = 0; index < count; ++index) {
        const auto value = readLe16(response.data, 1 + index * 2);
        if (!value || *value == 0) {
            return std::nullopt;
        }
        result.arrayElements.push_back(*value);
    }
    result.provenance = static_cast<Provenance>(response.data.back());
    return result;
}

PrimaryCommand makeGetArrayElementInfo(const std::uint16_t sequence,
                                       const std::uint16_t subunit,
                                       const std::uint16_t arrayElementNumber) {
    if (subunit == 0 || arrayElementNumber == 0) {
        throw std::invalid_argument("ADB array-info requires non-zero subunit and array element");
    }
    hdlc::Bytes data;
    appendLe16(data, arrayElementNumber);
    return {Command::adbGetAntennaArrayElementInfo, sequence, subunit, std::move(data)};
}

std::optional<ArrayElementInfo> parseArrayElementInfo(const Response& response) {
    if (!isExpected(response, Command::adbGetAntennaArrayElementInfo) || response.data.size() < 14) {
        return std::nullopt;
    }
    std::size_t offset{};
    const auto beamwidth = readLe16(response.data, offset);
    offset += 2;
    if (!beamwidth || !validProvenance(response.data[offset])) {
        return std::nullopt;
    }
    const auto beamwidthProvenance = static_cast<Provenance>(response.data[offset++]);
    const auto gain = readLe16(response.data, offset);
    offset += 2;
    if (!gain || !validProvenance(response.data[offset])) {
        return std::nullopt;
    }
    const auto gainProvenance = static_cast<Provenance>(response.data[offset++]);
    const auto count = static_cast<std::size_t>(response.data[offset++]);
    const auto fixedTail = std::size_t{7};
    if (count > (response.data.size() - offset) / 9 ||
        response.data.size() - offset != count * 9 + fixedTail) {
        return std::nullopt;
    }
    ArrayElementInfo result;
    result.azimuthBeamwidthDeciDegrees = *beamwidth;
    result.azimuthBeamwidthProvenance = beamwidthProvenance;
    result.gainDeciDbi = *gain;
    result.gainProvenance = gainProvenance;
    result.frequencies.reserve(count);
    for (std::size_t index = 0; index < count; ++index) {
        const auto link = response.data[offset++];
        const auto minimum = readLe32(response.data, offset);
        offset += 4;
        const auto maximum = readLe32(response.data, offset);
        offset += 4;
        if (link < static_cast<std::uint8_t>(LinkDescriptor::uplink) ||
            link > static_cast<std::uint8_t>(LinkDescriptor::bidirectional) ||
            !minimum || !maximum || *minimum > *maximum) {
            return std::nullopt;
        }
        result.frequencies.push_back(
            {static_cast<LinkDescriptor>(link), *minimum, *maximum});
    }
    if (!validProvenance(response.data[offset])) {
        return std::nullopt;
    }
    result.frequenciesProvenance = static_cast<Provenance>(response.data[offset++]);
    result.polarisation = response.data[offset++];
    if (!validProvenance(response.data[offset])) {
        return std::nullopt;
    }
    result.polarisationProvenance = static_cast<Provenance>(response.data[offset++]);
    result.observationReference = response.data[offset++];
    if (result.observationReference > 2 || !validProvenance(response.data[offset])) {
        return std::nullopt;
    }
    result.observationReferenceProvenance = static_cast<Provenance>(response.data[offset++]);
    result.polarisationSenseReference = response.data[offset++];
    if (result.polarisationSenseReference > 2 || !validProvenance(response.data[offset])) {
        return std::nullopt;
    }
    result.polarisationSenseReferenceProvenance = static_cast<Provenance>(response.data[offset++]);
    return offset == response.data.size() ? std::optional<ArrayElementInfo>(std::move(result))
                                          : std::nullopt;
}

PrimaryCommand makeSetInstallationInfo(const std::uint16_t sequence,
                                       const std::uint16_t subunit,
                                       const InstallationUpdate& update) {
    if (subunit == 0) {
        throw std::invalid_argument("ADB installation command requires a non-zero subunit");
    }
    std::uint8_t mask{};
    hdlc::Bytes data{0};
    if (update.sectorId) {
        mask |= 0x01U;
        appendText(data, *update.sectorId, 32, true);
    }
    if (update.installationNotes) {
        mask |= 0x02U;
        appendText(data, *update.installationNotes, 140, false);
    }
    if (update.mechanicalAzimuthDeciDegrees) {
        if (*update.mechanicalAzimuthDeciDegrees > 3599 ||
            !writableProvenance(update.mechanicalAzimuthProvenance)) {
            throw std::invalid_argument("ADB mechanical azimuth or provenance is invalid");
        }
        mask |= 0x04U;
        appendLe16(data, *update.mechanicalAzimuthDeciDegrees);
        data.push_back(static_cast<std::uint8_t>(update.mechanicalAzimuthProvenance));
    }
    if (update.mechanicalTiltDeciDegrees) {
        if (*update.mechanicalTiltDeciDegrees < -900 || *update.mechanicalTiltDeciDegrees > 900 ||
            !writableProvenance(update.mechanicalTiltProvenance)) {
            throw std::invalid_argument("ADB mechanical tilt or provenance is invalid");
        }
        mask |= 0x08U;
        appendLe16(data, static_cast<std::uint16_t>(*update.mechanicalTiltDeciDegrees));
        data.push_back(static_cast<std::uint8_t>(update.mechanicalTiltProvenance));
    }
    data[0] = mask;
    return {Command::adbSetAntennaInstallationInfo, sequence, subunit, std::move(data)};
}

PrimaryCommand makeGetInstallationInfo(const std::uint16_t sequence,
                                       const std::uint16_t subunit) {
    if (subunit == 0) {
        throw std::invalid_argument("ADB installation command requires a non-zero subunit");
    }
    return {Command::adbGetAntennaInstallationInfo, sequence, subunit, {}};
}

std::optional<InstallationInfo> parseInstallationInfo(const Response& response) {
    if (!isExpected(response, Command::adbGetAntennaInstallationInfo)) {
        return std::nullopt;
    }
    std::size_t offset{};
    const auto sector = readText(response.data, offset, 32, true);
    const auto notes = readText(response.data, offset, 140, false);
    if (!sector || !notes || response.data.size() - offset != 6) {
        return std::nullopt;
    }
    const auto azimuth = readLe16(response.data, offset);
    offset += 2;
    if (!azimuth || *azimuth > 3599 || !validProvenance(response.data[offset])) {
        return std::nullopt;
    }
    const auto azimuthProvenance = static_cast<Provenance>(response.data[offset++]);
    const auto tiltRaw = readLe16(response.data, offset);
    offset += 2;
    const auto tilt = tiltRaw ? static_cast<std::int16_t>(*tiltRaw) : std::int16_t{};
    if (!tiltRaw || tilt < -900 || tilt > 900 || !validProvenance(response.data[offset])) {
        return std::nullopt;
    }
    return InstallationInfo{*sector, *notes, *azimuth, azimuthProvenance, tilt,
                            static_cast<Provenance>(response.data[offset])};
}

PrimaryCommand makeSetRfPathIds(const std::uint16_t sequence,
                                const std::uint16_t subunit,
                                const std::uint16_t arrayElementNumber,
                                const RfPathIds& ids) {
    if (subunit == 0 || arrayElementNumber == 0 || ids.values.size() > 6 ||
        !writableProvenance(ids.provenance)) {
        throw std::invalid_argument("ADB RF path assignment is invalid");
    }
    hdlc::Bytes data;
    appendLe16(data, arrayElementNumber);
    data.push_back(static_cast<std::uint8_t>(ids.values.size()));
    for (const auto value : ids.values) {
        appendLe16(data, value);
    }
    data.push_back(static_cast<std::uint8_t>(ids.provenance));
    return {Command::adbSetRfPathIdToArrayElement, sequence, subunit, std::move(data)};
}

PrimaryCommand makeGetRfPathIds(const std::uint16_t sequence,
                                const std::uint16_t subunit,
                                const std::uint32_t primaryId,
                                const std::uint16_t arrayElementNumber) {
    if (subunit == 0 || primaryId == 0 || arrayElementNumber == 0) {
        throw std::invalid_argument("ADB RF path query is invalid");
    }
    hdlc::Bytes data;
    appendLe32(data, primaryId);
    appendLe16(data, arrayElementNumber);
    return {Command::adbGetRfPathIdOfArrayElement, sequence, subunit, std::move(data)};
}

std::optional<RfPathIds> parseRfPathIds(const Response& response) {
    if (!isExpected(response, Command::adbGetRfPathIdOfArrayElement) || response.data.size() < 2) {
        return std::nullopt;
    }
    const auto count = static_cast<std::size_t>(response.data[0]);
    if (response.data.size() != 2 + count * 2 || !validProvenance(response.data.back())) {
        return std::nullopt;
    }
    RfPathIds result;
    result.values.reserve(count);
    for (std::size_t index = 0; index < count; ++index) {
        const auto value = readLe16(response.data, 1 + index * 2);
        if (!value) {
            return std::nullopt;
        }
        result.values.push_back(*value);
    }
    result.provenance = static_cast<Provenance>(response.data.back());
    return result;
}

} // namespace atc::aisg3::adb
