#include "ret/ret_hdlc.h"
#include "ret/ret_protocol.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CHECK(condition)                                                        \
    do {                                                                        \
        if (!(condition)) {                                                     \
            fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__, __LINE__,  \
                    #condition);                                                \
            exit(EXIT_FAILURE);                                                 \
        }                                                                       \
    } while (0)

typedef struct {
    uint32_t now;
    uint8_t wire[RET_HDLC_MAX_WIRE];
    size_t wire_length;
    unsigned transmit_count;
    unsigned application_reset_count;
    unsigned system_reset_count;
    unsigned save_count;
    bool stored;
    ret_config_t storage;
    bool motor_start_ok;
    ret_motor_result_t motor_result;
    int16_t motor_position;
    ret_motor_operation_t last_motor_operation;
    int16_t last_motor_target;
} mock_t;

typedef struct {
    ret_device_t device;
    mock_t mock;
    uint8_t primary_ns;
    uint8_t secondary_ack;
} fixture_t;

static uint32_t mock_millis(void *opaque) {
    return ((mock_t *)opaque)->now;
}

static void mock_transmit(void *opaque, const uint8_t *data, size_t length) {
    mock_t *mock = opaque;
    CHECK(length <= sizeof(mock->wire));
    memcpy(mock->wire, data, length);
    mock->wire_length = length;
    ++mock->transmit_count;
}

static bool mock_load(void *opaque, ret_config_t *config) {
    mock_t *mock = opaque;
    if (!mock->stored) {
        return false;
    }
    *config = mock->storage;
    return true;
}

static bool mock_save(void *opaque, const ret_config_t *config) {
    mock_t *mock = opaque;
    mock->storage = *config;
    mock->stored = true;
    ++mock->save_count;
    return true;
}

static bool mock_motor_start(void *opaque,
                             ret_motor_operation_t operation,
                             uint8_t antenna_number,
                             int16_t target) {
    mock_t *mock = opaque;
    CHECK(antenna_number == 1u);
    mock->last_motor_operation = operation;
    mock->last_motor_target = target;
    return mock->motor_start_ok;
}

static ret_motor_result_t mock_motor_poll(void *opaque,
                                          uint8_t antenna_number,
                                          int16_t *position) {
    mock_t *mock = opaque;
    CHECK(antenna_number == 1u);
    *position = mock->motor_position;
    return mock->motor_result;
}

static void mock_motor_stop(void *opaque) {
    (void)opaque;
}

static void mock_application_reset(void *opaque) {
    ++((mock_t *)opaque)->application_reset_count;
}

static void mock_system_reset(void *opaque) {
    ++((mock_t *)opaque)->system_reset_count;
}

static ret_platform_t make_mock_platform(mock_t *mock) {
    ret_platform_t platform;
    memset(&platform, 0, sizeof(platform));
    platform.context = mock;
    platform.millis = mock_millis;
    platform.transmit = mock_transmit;
    platform.storage_load = mock_load;
    platform.storage_save = mock_save;
    platform.motor_start = mock_motor_start;
    platform.motor_poll = mock_motor_poll;
    platform.motor_stop = mock_motor_stop;
    platform.application_reset = mock_application_reset;
    platform.system_reset = mock_system_reset;
    return platform;
}

static void send_frame(ret_device_t *device, const ret_hdlc_frame_t *frame) {
    uint8_t wire[RET_HDLC_MAX_WIRE];
    size_t wire_length = 0u;
    CHECK(ret_hdlc_encode(frame, wire, sizeof(wire), &wire_length));
    ret_device_receive(device, wire, wire_length);
}

static ret_hdlc_frame_t take_response(fixture_t *fixture) {
    ret_hdlc_decoder_t decoder;
    ret_hdlc_frame_t result;
    bool found = false;
    size_t index;
    fixture->mock.now += 4u;
    ret_device_poll(&fixture->device);
    CHECK(fixture->mock.wire_length != 0u);
    ret_hdlc_decoder_init(&decoder);
    memset(&result, 0, sizeof(result));
    for (index = 0u; index < fixture->mock.wire_length; ++index) {
        if (ret_hdlc_decoder_push(&decoder, fixture->mock.wire[index],
                                  &result) == RET_HDLC_DECODER_FRAME) {
            found = true;
        }
    }
    CHECK(found);
    fixture->mock.wire_length = 0u;
    return result;
}

static void send_xid_scan(fixture_t *fixture) {
    ret_hdlc_frame_t frame;
    ret_hdlc_frame_t response;
    const uint8_t information[] = {0x81u, 0xF0u, 0x04u,
                                   0x01u, 0x00u, 0x03u, 0x00u};
    memset(&frame, 0, sizeof(frame));
    frame.address = 0xFFu;
    frame.control = 0xBFu;
    frame.information_length = sizeof(information);
    memcpy(frame.information, information, sizeof(information));
    send_frame(&fixture->device, &frame);
    fixture->mock.now +=
        fixture->device.config.unit_id[fixture->device.config.unit_id_length - 1u] %
        7u;
    response = take_response(fixture);
    CHECK(response.address == 0u);
    CHECK(response.control == 0xBFu);
    CHECK(response.information_length >= 10u);
    CHECK(response.information[0] == 0x81u);
    CHECK(response.information[1] == 0xF0u);
}

static void assign_address(fixture_t *fixture, uint8_t address) {
    ret_hdlc_frame_t frame;
    ret_hdlc_frame_t response;
    uint8_t uid[19];
    size_t uid_length = fixture->device.config.unit_id_length + 2u;
    size_t length = 0u;
    memset(&frame, 0, sizeof(frame));
    uid[0] = (uint8_t)fixture->device.config.vendor_code[0];
    uid[1] = (uint8_t)fixture->device.config.vendor_code[1];
    memcpy(&uid[2], fixture->device.config.unit_id,
           fixture->device.config.unit_id_length);
    frame.information[length++] = 0x81u;
    frame.information[length++] = 0xF0u;
    frame.information[length++] = (uint8_t)(uid_length + 5u);
    frame.information[length++] = 0x01u;
    frame.information[length++] = (uint8_t)uid_length;
    memcpy(&frame.information[length], uid, uid_length);
    length += uid_length;
    frame.information[length++] = 0x02u;
    frame.information[length++] = 0x01u;
    frame.information[length++] = address;
    frame.address = 0xFFu;
    frame.control = 0xBFu;
    frame.information_length = (uint16_t)length;
    send_frame(&fixture->device, &frame);
    response = take_response(fixture);
    CHECK(response.address == address);
    CHECK(ret_device_address(&fixture->device) == address);
    CHECK(ret_device_link_state(&fixture->device) ==
          RET_LINK_ADDRESS_ASSIGNED);
}

static void connect_link(fixture_t *fixture) {
    ret_hdlc_frame_t frame;
    ret_hdlc_frame_t response;
    memset(&frame, 0, sizeof(frame));
    frame.address = ret_device_address(&fixture->device);
    frame.control = 0x93u;
    send_frame(&fixture->device, &frame);
    response = take_response(fixture);
    CHECK(response.address == frame.address);
    CHECK(response.control == 0x73u);
    CHECK(ret_device_link_state(&fixture->device) == RET_LINK_CONNECTED);
    fixture->primary_ns = 0u;
    fixture->secondary_ack = 0u;
}

static void init_fixture(fixture_t *fixture, bool calibrated, bool multi) {
    ret_config_t config;
    ret_platform_t platform;
    memset(fixture, 0, sizeof(*fixture));
    fixture->mock.motor_start_ok = true;
    fixture->mock.motor_result = RET_MOTOR_RUNNING;
    fixture->mock.motor_position = 20;
    ret_config_set_defaults(&config);
    config.antennas[0].calibrated = calibrated;
    if (multi) {
        config.device_type = RET_DEVICE_TYPE_MULTI;
        config.antenna_count = 2u;
        config.antennas[1] = config.antennas[0];
        config.antennas[1].serial[16] = '2';
    }
    platform = make_mock_platform(&fixture->mock);
    ret_device_init(&fixture->device, &platform, &config);
}

static void discover_and_connect(fixture_t *fixture) {
    send_xid_scan(fixture);
    assign_address(fixture, 1u);
    connect_link(fixture);
}

static ret_hdlc_frame_t send_l7(fixture_t *fixture,
                                uint8_t procedure,
                                const uint8_t *data,
                                uint16_t data_length) {
    ret_hdlc_frame_t request;
    memset(&request, 0, sizeof(request));
    request.address = ret_device_address(&fixture->device);
    request.control = (uint8_t)(0x10u |
                                ((fixture->primary_ns & 7u) << 1u) |
                                ((fixture->secondary_ack & 7u) << 5u));
    request.information[0] = procedure;
    request.information[1] = (uint8_t)data_length;
    request.information[2] = (uint8_t)(data_length >> 8u);
    if (data_length != 0u) {
        memcpy(&request.information[3], data, data_length);
    }
    request.information_length = (uint16_t)(data_length + 3u);
    send_frame(&fixture->device, &request);
    fixture->primary_ns = (uint8_t)((fixture->primary_ns + 1u) & 7u);
    return take_response(fixture);
}

static ret_hdlc_frame_t send_rr_poll(fixture_t *fixture) {
    ret_hdlc_frame_t request;
    memset(&request, 0, sizeof(request));
    request.address = ret_device_address(&fixture->device);
    request.control = (uint8_t)(0x11u |
                                ((fixture->secondary_ack & 7u) << 5u));
    send_frame(&fixture->device, &request);
    return take_response(fixture);
}

static void check_i_response(fixture_t *fixture,
                             const ret_hdlc_frame_t *response,
                             uint8_t procedure,
                             const uint8_t *expected,
                             uint16_t expected_length) {
    CHECK((response->control & 1u) == 0u);
    CHECK(((response->control >> 1u) & 7u) == fixture->secondary_ack);
    CHECK(response->information_length == expected_length + 3u);
    CHECK(response->information[0] == procedure);
    CHECK(response->information[1] == (uint8_t)expected_length);
    CHECK(response->information[2] == (uint8_t)(expected_length >> 8u));
    if (expected_length != 0u) {
        CHECK(memcmp(&response->information[3], expected, expected_length) == 0);
    }
    fixture->secondary_ack =
        (uint8_t)((fixture->secondary_ack + 1u) & 7u);
}

static void test_hdlc(void) {
    static const uint8_t check_text[] = "123456789";
    ret_hdlc_frame_t input;
    ret_hdlc_frame_t output;
    ret_hdlc_decoder_t decoder;
    uint8_t wire[RET_HDLC_MAX_WIRE];
    size_t wire_length = 0u;
    size_t index;
    bool decoded = false;
    CHECK(ret_hdlc_crc16_x25(check_text, 9u) == 0x906Eu);
    memset(&input, 0, sizeof(input));
    input.address = 0x7Eu;
    input.control = 0x7Du;
    input.information[0] = 0x7Eu;
    input.information[1] = 0x7Du;
    input.information_length = 2u;
    CHECK(ret_hdlc_encode(&input, wire, sizeof(wire), &wire_length));
    CHECK(wire[0] == 0x7Eu);
    CHECK(wire[wire_length - 1u] == 0x7Eu);
    ret_hdlc_decoder_init(&decoder);
    for (index = 0u; index < wire_length; ++index) {
        if (ret_hdlc_decoder_push(&decoder, wire[index], &output) ==
            RET_HDLC_DECODER_FRAME) {
            decoded = true;
        }
    }
    CHECK(decoded);
    CHECK(output.address == input.address);
    CHECK(output.control == input.control);
    CHECK(output.information_length == input.information_length);
    CHECK(memcmp(output.information, input.information,
                 input.information_length) == 0);
}

static void test_discovery_information_and_reset(void) {
    fixture_t fixture;
    ret_hdlc_frame_t response;
    init_fixture(&fixture, false, false);
    discover_and_connect(&fixture);

    response = send_l7(&fixture, 0x05u, NULL, 0u);
    CHECK((response.control & 1u) == 0u);
    CHECK(response.information[0] == 0x05u);
    CHECK(response.information[3] == 0x00u);
    fixture.secondary_ack = 1u;

    response = send_l7(&fixture, 0x03u, NULL, 0u);
    {
        const uint8_t ok[] = {0x00u};
        check_i_response(&fixture, &response, 0x03u, ok, sizeof(ok));
    }
    CHECK(fixture.mock.application_reset_count == 0u);
    response = send_rr_poll(&fixture);
    CHECK((response.control & 0x0Fu) == 0x01u);
    CHECK(fixture.mock.application_reset_count == 1u);
    CHECK(ret_device_link_state(&fixture.device) == RET_LINK_CONNECTED);
}

static void test_scan_uses_vendor_left_and_uid_right(void) {
    fixture_t fixture;
    ret_hdlc_frame_t request;
    ret_hdlc_frame_t response;
    uint8_t last;
    const uint8_t mask[] = {0xFFu, 0xFFu, 0xFFu};
    init_fixture(&fixture, false, false);
    last = fixture.device.config.unit_id[fixture.device.config.unit_id_length - 1u];
    memset(&request, 0, sizeof(request));
    request.address = 0xFFu;
    request.control = 0xBFu;
    request.information[0] = 0x81u;
    request.information[1] = 0xF0u;
    request.information[2] = 10u;
    request.information[3] = 0x01u;
    request.information[4] = 3u;
    request.information[5] = (uint8_t)fixture.device.config.vendor_code[0];
    request.information[6] = (uint8_t)fixture.device.config.vendor_code[1];
    request.information[7] = last;
    request.information[8] = 0x03u;
    request.information[9] = 3u;
    memcpy(&request.information[10], mask, sizeof(mask));
    request.information_length = 13u;
    send_frame(&fixture.device, &request);
    fixture.mock.now += last % 7u;
    response = take_response(&fixture);
    CHECK(response.address == 0u);
    CHECK(response.control == 0xBFu);
}

static void test_rnr_blocks_information_retransmission(void) {
    fixture_t fixture;
    ret_hdlc_frame_t response;
    ret_hdlc_frame_t poll;
    init_fixture(&fixture, false, false);
    discover_and_connect(&fixture);
    response = send_l7(&fixture, 0x05u, NULL, 0u);
    CHECK((response.control & 1u) == 0u);

    memset(&poll, 0, sizeof(poll));
    poll.address = 1u;
    poll.control = 0x15u; /* RNR, P=1, N(R)=0: do not acknowledge. */
    send_frame(&fixture.device, &poll);
    response = take_response(&fixture);
    CHECK((response.control & 0x0Fu) == 0x01u);

    poll.control = 0x11u; /* RR, P=1, still asks for retransmission. */
    send_frame(&fixture.device, &poll);
    response = take_response(&fixture);
    CHECK((response.control & 1u) == 0u);
    CHECK(((response.control >> 1u) & 7u) == 0u);
}

static void test_segmented_configuration(void) {
    fixture_t fixture;
    ret_hdlc_frame_t response;
    uint8_t first[71];
    const uint8_t second[] = {0xA1u, 0xB2u, 0xC3u};
    const uint8_t ok[] = {0x00u};
    size_t index;
    init_fixture(&fixture, false, false);
    discover_and_connect(&fixture);
    for (index = 0u; index < sizeof(first); ++index) {
        first[index] = (uint8_t)index;
    }
    response = send_l7(&fixture, 0x32u, first, sizeof(first));
    check_i_response(&fixture, &response, 0x32u, ok, sizeof(ok));
    CHECK(!fixture.device.config.antennas[0].configured);
    CHECK(fixture.device.config.antennas[0].vendor_configuration_length == 71u);

    response = send_l7(&fixture, 0x32u, second, sizeof(second));
    check_i_response(&fixture, &response, 0x32u, ok, sizeof(ok));
    CHECK(fixture.device.config.antennas[0].configured);
    CHECK(fixture.device.config.antennas[0].vendor_configuration_length == 74u);
    CHECK(memcmp(fixture.device.config.antennas[0].vendor_configuration,
                 first, sizeof(first)) == 0);
    CHECK(memcmp(&fixture.device.config.antennas[0].vendor_configuration[71],
                 second, sizeof(second)) == 0);
}

static void test_tcc_and_alarm_indication(void) {
    fixture_t fixture;
    ret_hdlc_frame_t response;
    const uint8_t target[] = {50u, 0u};
    const uint8_t ok[] = {0x00u};
    const uint8_t alarm[] = {0x11u, 0x01u};
    init_fixture(&fixture, true, false);
    discover_and_connect(&fixture);

    response = send_l7(&fixture, 0x33u, target, sizeof(target));
    CHECK((response.control & 0x0Fu) == 0x01u);
    CHECK(fixture.mock.last_motor_operation == RET_MOTOR_SET_TILT);
    CHECK(fixture.mock.last_motor_target == 50);
    fixture.mock.motor_position = 50;
    fixture.mock.motor_result = RET_MOTOR_OK;
    ret_device_poll(&fixture.device);
    response = send_rr_poll(&fixture);
    check_i_response(&fixture, &response, 0x33u, ok, sizeof(ok));
    CHECK(fixture.device.config.antennas[0].current_tilt_tenths == 50);

    response = send_l7(&fixture, 0x12u, NULL, 0u);
    check_i_response(&fixture, &response, 0x12u, ok, sizeof(ok));
    response = send_rr_poll(&fixture);
    CHECK((response.control & 0x0Fu) == 0x01u);

    ret_device_set_common_alarm(&fixture.device, 0x11u, true);
    response = send_rr_poll(&fixture);
    check_i_response(&fixture, &response, 0x07u, alarm, sizeof(alarm));
}

static void test_xid_negotiation_and_multi_rejection(void) {
    fixture_t fixture;
    ret_hdlc_frame_t request;
    ret_hdlc_frame_t response;
    const uint8_t expected_error[] = {0x0Bu, 0x25u};
    const uint8_t negotiation[] = {
        0x81u, 0x80u, 0x0Fu,
        0x05u, 0x04u, 0x00u, 0x00u, 0x10u, 0x00u,
        0x06u, 0x04u, 0x00u, 0x00u, 0x10u, 0x00u,
        0x07u, 0x01u, 0x07u,
        0x81u, 0xF0u, 0x07u,
        0x14u, 0x01u, 0x03u,
        0x15u, 0x02u, 0x11u, 0x07u};
    init_fixture(&fixture, true, true);
    discover_and_connect(&fixture);

    memset(&request, 0, sizeof(request));
    request.address = 1u;
    request.control = 0xBFu;
    request.information_length = sizeof(negotiation);
    memcpy(request.information, negotiation, sizeof(negotiation));
    send_frame(&fixture.device, &request);
    response = take_response(&fixture);
    CHECK(response.control == 0xBFu);
    CHECK(fixture.device.maximum_tx_information == RET_HDLC_MAX_INFO);
    CHECK(fixture.device.maximum_rx_information == RET_HDLC_MAX_INFO);
    CHECK(memchr(response.information, 0x14u,
                 response.information_length) != NULL);

    response = send_l7(&fixture, 0x0Eu, (const uint8_t[]){0x01u}, 1u);
    check_i_response(&fixture, &response, 0x0Eu, expected_error,
                     sizeof(expected_error));
}

static void test_link_timeout(void) {
    fixture_t fixture;
    init_fixture(&fixture, false, false);
    discover_and_connect(&fixture);
    fixture.mock.now += 180001u;
    ret_device_poll(&fixture.device);
    CHECK(fixture.mock.system_reset_count == 1u);
}

int main(void) {
    test_hdlc();
    test_discovery_information_and_reset();
    test_scan_uses_vendor_left_and_uid_right();
    test_rnr_blocks_information_retransmission();
    test_segmented_configuration();
    test_tcc_and_alarm_indication();
    test_xid_negotiation_and_multi_rejection();
    test_link_timeout();
    puts("RET AISG 2.0 tests: OK");
    return EXIT_SUCCESS;
}
