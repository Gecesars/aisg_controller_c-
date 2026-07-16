#define _DEFAULT_SOURCE
#define _POSIX_C_SOURCE 200809L

#include "ret/ret_protocol.h"

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#define HOST_MAX_DEVICES 8u

typedef struct {
    int serial_fd;
    struct timespec started_at;
    bool io_failed;
    bool stored;
    ret_config_t storage;
    bool motor_running;
    bool motor_timing_initialized;
    uint32_t motor_ready_at_ms;
    ret_motor_operation_t motor_operation;
    int16_t motor_target;
    bool system_reset_requested;
} host_context_t;

static volatile sig_atomic_t stop_requested;

static void request_stop(const int signal_number) {
    (void)signal_number;
    stop_requested = 1;
}

static uint32_t host_millis(void *opaque) {
    host_context_t *context = opaque;
    struct timespec now;
    uint64_t milliseconds;
    (void)clock_gettime(CLOCK_MONOTONIC, &now);
    milliseconds = (uint64_t)(now.tv_sec - context->started_at.tv_sec) * 1000u;
    if (now.tv_nsec >= context->started_at.tv_nsec) {
        milliseconds += (uint64_t)(now.tv_nsec - context->started_at.tv_nsec) /
                        1000000u;
    } else {
        milliseconds -= 1000u;
        milliseconds += (uint64_t)(1000000000L + now.tv_nsec -
                                   context->started_at.tv_nsec) /
                        1000000u;
    }
    return (uint32_t)milliseconds;
}

static void host_transmit(void *opaque, const uint8_t *data,
                          const size_t length) {
    host_context_t *context = opaque;
    size_t offset = 0u;
    while (offset < length) {
        const ssize_t written = write(context->serial_fd, &data[offset],
                                      length - offset);
        if (written > 0) {
            offset += (size_t)written;
            continue;
        }
        if ((written < 0) && (errno == EINTR)) {
            continue;
        }
        if ((written < 0) && ((errno == EAGAIN) || (errno == EWOULDBLOCK))) {
            struct pollfd descriptor = {context->serial_fd, POLLOUT, 0};
            if (poll(&descriptor, 1u, 100) >= 0) {
                continue;
            }
        }
        context->io_failed = true;
        return;
    }
}

static bool host_storage_load(void *opaque, ret_config_t *config) {
    host_context_t *context = opaque;
    if (!context->stored) {
        return false;
    }
    *config = context->storage;
    return true;
}

static bool host_storage_save(void *opaque, const ret_config_t *config) {
    host_context_t *context = opaque;
    context->storage = *config;
    context->stored = true;
    return true;
}

static bool host_motor_start(void *opaque,
                             const ret_motor_operation_t operation,
                             const uint8_t antenna_number,
                             const int16_t target_tilt_tenths) {
    host_context_t *context = opaque;
    if (antenna_number != 1u) {
        return false;
    }
    context->motor_running = true;
    context->motor_timing_initialized = operation != RET_MOTOR_SET_TILT;
    context->motor_operation = operation;
    context->motor_target = target_tilt_tenths;
    context->motor_ready_at_ms = context->motor_timing_initialized
                                     ? host_millis(context) + 250u
                                     : 0u;
    return true;
}

static ret_motor_result_t host_motor_poll(void *opaque,
                                          const uint8_t antenna_number,
                                          int16_t *position_tenths) {
    host_context_t *context = opaque;
    if ((antenna_number != 1u) || !context->motor_running) {
        return RET_MOTOR_IDLE;
    }
    if (!context->motor_timing_initialized) {
        int32_t distance_tenths =
            (int32_t)context->motor_target - (int32_t)*position_tenths;
        if (distance_tenths < 0) {
            distance_tenths = -distance_tenths;
        }
        /* 200 ms per tenth of a degree is exactly two seconds per degree. */
        context->motor_ready_at_ms =
            host_millis(context) + (uint32_t)distance_tenths * 200u;
        context->motor_timing_initialized = true;
    }
    if ((int32_t)(host_millis(context) - context->motor_ready_at_ms) < 0) {
        return RET_MOTOR_RUNNING;
    }
    context->motor_running = false;
    if (context->motor_operation == RET_MOTOR_SET_TILT) {
        *position_tenths = context->motor_target;
    }
    return RET_MOTOR_OK;
}

static void host_motor_stop(void *opaque) {
    host_context_t *context = opaque;
    context->motor_running = false;
    context->motor_timing_initialized = false;
}

static void host_application_reset(void *opaque) {
    (void)opaque;
}

static void host_system_reset(void *opaque) {
    ((host_context_t *)opaque)->system_reset_requested = true;
}

static ret_platform_t make_platform(host_context_t *context) {
    ret_platform_t platform;
    memset(&platform, 0, sizeof(platform));
    platform.context = context;
    platform.millis = host_millis;
    platform.transmit = host_transmit;
    platform.storage_load = host_storage_load;
    platform.storage_save = host_storage_save;
    platform.motor_start = host_motor_start;
    platform.motor_poll = host_motor_poll;
    platform.motor_stop = host_motor_stop;
    platform.application_reset = host_application_reset;
    platform.system_reset = host_system_reset;
    return platform;
}

static int open_serial(const char *path) {
    struct termios attributes;
    const int descriptor = open(path, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (descriptor < 0) {
        return -1;
    }
    if (tcgetattr(descriptor, &attributes) != 0) {
        (void)close(descriptor);
        return -1;
    }
    cfmakeraw(&attributes);
    (void)cfsetispeed(&attributes, B9600);
    (void)cfsetospeed(&attributes, B9600);
    attributes.c_cflag |= CLOCAL | CREAD;
    attributes.c_cflag &= (tcflag_t)~(PARENB | CSTOPB | CSIZE);
    attributes.c_cflag |= CS8;
    if (tcsetattr(descriptor, TCSANOW, &attributes) != 0) {
        (void)close(descriptor);
        return -1;
    }
    (void)tcflush(descriptor, TCIOFLUSH);
    return descriptor;
}

static void set_host_defaults(ret_config_t *config, const size_t device_index) {
    const unsigned int number = (unsigned int)device_index + 1u;
    ret_config_set_defaults(config);
    (void)snprintf(config->product_number, sizeof(config->product_number),
                   "ATC-RET-HOST");
    (void)snprintf((char *)config->unit_id, sizeof(config->unit_id),
                   "RET%013u", number);
    config->unit_id_length = 16u;
    (void)snprintf(config->serial_number, sizeof(config->serial_number),
                   "RET%013u", number);
    (void)snprintf(config->antennas[0].serial,
                   sizeof(config->antennas[0].serial), "ANT%013u", number);
    (void)snprintf(config->hardware_version, sizeof(config->hardware_version),
                   "STM32F405-SIM");
    (void)snprintf(config->software_version, sizeof(config->software_version),
                   "2.0.0-host");
    config->antennas[0].current_tilt_tenths =
        (int16_t)(20 + (int16_t)(device_index * 30u));
    /* Match the 0.0..15.0 degree bench actuator represented by the GUI. */
    config->antennas[0].maximum_tilt_tenths = 150;
    config->antennas[0].calibrated = true;
}

static void initialize_device(ret_device_t *device,
                              const ret_platform_t *platform,
                              const ret_config_t *fallback,
                              const uint8_t address) {
    ret_device_init(device, platform, fallback);
    if (address != 0u) {
        device->address = address;
        device->link_state = RET_LINK_ADDRESS_ASSIGNED;
        device->last_addressed_frame_ms = host_millis(platform->context);
    }
}

static void print_usage(const char *program) {
    fprintf(stderr,
            "Uso: %s --port PORTA [--devices 1..8] [--address 1..254]\n",
            program);
}

int main(int argc, char **argv) {
    const char *port = NULL;
    unsigned long address_value = 0u;
    unsigned long device_count = 2u;
    host_context_t contexts[HOST_MAX_DEVICES];
    ret_config_t fallbacks[HOST_MAX_DEVICES];
    ret_platform_t platforms[HOST_MAX_DEVICES];
    ret_device_t devices[HOST_MAX_DEVICES];
    int serial_fd;
    int index;
    size_t device_index;

    for (index = 1; index < argc; ++index) {
        if ((strcmp(argv[index], "--port") == 0) && (index + 1 < argc)) {
            port = argv[++index];
        } else if ((strcmp(argv[index], "--address") == 0) &&
                   (index + 1 < argc)) {
            char *end = NULL;
            address_value = strtoul(argv[++index], &end, 10);
            if ((end == argv[index]) || (*end != '\0') ||
                (address_value == 0u) || (address_value >= 0xFFu)) {
                print_usage(argv[0]);
                return EXIT_FAILURE;
            }
        } else if ((strcmp(argv[index], "--devices") == 0) &&
                   (index + 1 < argc)) {
            char *end = NULL;
            device_count = strtoul(argv[++index], &end, 10);
            if ((end == argv[index]) || (*end != '\0') ||
                (device_count == 0u) || (device_count > HOST_MAX_DEVICES)) {
                print_usage(argv[0]);
                return EXIT_FAILURE;
            }
        } else if ((strcmp(argv[index], "--help") == 0) ||
                   (strcmp(argv[index], "-h") == 0)) {
            print_usage(argv[0]);
            return EXIT_SUCCESS;
        } else {
            print_usage(argv[0]);
            return EXIT_FAILURE;
        }
    }
    if (port == NULL) {
        print_usage(argv[0]);
        return EXIT_FAILURE;
    }

    if ((address_value != 0u) &&
        (address_value + device_count - 1u >= 0xFFu)) {
        print_usage(argv[0]);
        return EXIT_FAILURE;
    }

    serial_fd = open_serial(port);
    if (serial_fd < 0) {
        fprintf(stderr, "RET host: não foi possível abrir %s: %s\n", port,
                strerror(errno));
        return EXIT_FAILURE;
    }

    memset(contexts, 0, sizeof(contexts));
    for (device_index = 0u; device_index < device_count; ++device_index) {
        contexts[device_index].serial_fd = serial_fd;
        (void)clock_gettime(CLOCK_MONOTONIC,
                            &contexts[device_index].started_at);
        set_host_defaults(&fallbacks[device_index], device_index);
        platforms[device_index] = make_platform(&contexts[device_index]);
        initialize_device(
            &devices[device_index], &platforms[device_index],
            &fallbacks[device_index],
            address_value == 0u
                ? 0u
                : (uint8_t)(address_value + (unsigned long)device_index));
    }
    (void)signal(SIGINT, request_stop);
    (void)signal(SIGTERM, request_stop);
    if (address_value == 0u) {
        fprintf(stderr,
                "RET host: %lu instâncias ret_core ativas em %s, estado NoAddress\n",
                device_count, port);
    } else {
        fprintf(stderr,
                "RET host: %lu instâncias ret_core ativas em %s, endereços 0x%02lX..0x%02lX\n",
                device_count, port, address_value,
                address_value + device_count - 1u);
    }

    while (!stop_requested) {
        bool io_failed = false;
        struct pollfd descriptor = {serial_fd, POLLIN, 0};
        const int ready = poll(&descriptor, 1u, 2);
        if ((ready > 0) && ((descriptor.revents & POLLIN) != 0)) {
            uint8_t received[256];
            const ssize_t count = read(serial_fd, received, sizeof(received));
            if (count > 0) {
                for (device_index = 0u; device_index < device_count;
                     ++device_index) {
                    ret_device_receive(&devices[device_index], received,
                                       (size_t)count);
                }
            } else if ((count < 0) && (errno != EAGAIN) &&
                       (errno != EWOULDBLOCK) && (errno != EINTR)) {
                io_failed = true;
            }
        } else if ((ready < 0) && (errno != EINTR)) {
            io_failed = true;
        }
        for (device_index = 0u; device_index < device_count; ++device_index) {
            ret_device_poll(&devices[device_index]);
            io_failed = io_failed || contexts[device_index].io_failed;
            if (contexts[device_index].system_reset_requested) {
                contexts[device_index].system_reset_requested = false;
                initialize_device(
                    &devices[device_index], &platforms[device_index],
                    &fallbacks[device_index],
                    address_value == 0u
                        ? 0u
                        : (uint8_t)(address_value +
                                    (unsigned long)device_index));
            }
        }
        if (io_failed) {
            (void)close(serial_fd);
            fprintf(stderr, "RET host: falha de E/S na porta serial\n");
            return EXIT_FAILURE;
        }
    }

    (void)close(serial_fd);
    fprintf(stderr, "RET host: encerrado\n");
    return EXIT_SUCCESS;
}
