#include <ctype.h>
#include <dirent.h>
#include <fcntl.h>
#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

#define DEVICE_RECONNECT_DELAY 10
#define SYSTEM_CONFIG_FILE "/etc/display-sensors/config.txt"
#define LOCAL_CONFIG_FILE "config.txt"

const char *DEVICE_ID = "1a86";

int update_interval = 5;
char *cpu_path = NULL;
char *gpu_edge_path = NULL;
char *gpu_mem_path = NULL;
char *nvme_path = NULL;

int serial_fd = -1;

int prev_cpu_temp = 0;
int prev_gpu_temp = 0;
int prev_nvme_temp = 0;

static inline int max(int a, int b) {
    return (a > b) ? a : b;
}

const char *find_config_file() {
    if (access(LOCAL_CONFIG_FILE, R_OK) == 0) {
        return LOCAL_CONFIG_FILE;
    }

    if (access(SYSTEM_CONFIG_FILE, R_OK) == 0) {
        return SYSTEM_CONFIG_FILE;
    }

    return NULL;
}

void read_config_file() {
    const char *config_file = find_config_file();


    if (!config_file) {
        printf("Error: Unable to open the configuration file '%s'\n\n", config_file);
        printf(
            "Make sure it exists under one of the following paths:\n"
            "\t./config.txt\n"
            "\t/etc/display-sensors/config.txt\n"
        );
        exit(1);
    }

    FILE *file = fopen(config_file, "r");
    if (!file) {
        printf("Error: Cannot open the configuration file: %s\n\n", config_file);
        printf("Please make sure a config file has the following content:\n");
        printf(
            "\tcpu_path=/sys/class/hwmon/hwmon4/temp1_input\n"
            "\tgpu_edge_path=/sys/class/hwmon/hwmon2/temp2_input\n"
            "\tgpu_mem_path=/sys/class/hwmon/hwmon2/temp3_input\n"
            "\tnvme_path=/sys/class/hwmon/hwmon1/temp3_input\n"
            "\tupdate_interval=5\n"
        );
        exit(1);
    }

    char line[256];
    while (fgets(line, sizeof(line), file)) {
        char *newline = strchr(line, '\n');
        if (newline) {
            *newline = '\0';
        }

        char *key = strtok(line, "=");
        char *value = strtok(NULL, "=");

        if (key && value) {
            if (strcmp(key, "cpu_path") == 0) {
                cpu_path = strdup(value);
            } else if (strcmp(key, "gpu_edge_path") == 0) {
                gpu_edge_path = strdup(value);
            } else if (strcmp(key, "gpu_mem_path") == 0) {
                gpu_mem_path = strdup(value);
            } else if (strcmp(key, "nvme_path") == 0) {
                nvme_path = strdup(value);
            } else if (strcmp(key, "update_interval") == 0) {
                char *endptr;
                int _update_interval = strtol(value, &endptr, 10);
                if (*endptr != '\0' || _update_interval <= 0) {
                    printf("Error: Invalid 'update_interval' value in the configuration file. It must be a positive integer.\n");
                    fclose(file);
                    exit(1);
                }
                update_interval = _update_interval;
            }
        }
    }

    fclose(file);

    if (!cpu_path || !gpu_edge_path || !gpu_mem_path || !nvme_path) {
        printf("Error: Missing entries in the configuration file.\n\nPlease ensure it has the following format:\n\n");
        printf(
            "cpu_path=/sys/class/hwmon/hwmon4/temp1_input\n"
            "gpu_edge_path=/sys/class/hwmon/hwmon2/temp2_input\n"
            "gpu_mem_path=/sys/class/hwmon/hwmon2/temp3_input\n"
            "nvme_path=/sys/class/hwmon/hwmon1/temp3_input\n"
            "update_interval=5\n"
        );
        exit(1);
    }
}

const char *find_device() {
    DIR *dir;
    struct dirent *entry;
    static char device[300];

    if ((dir = opendir("/dev/serial/by-id")) == NULL) {
        printf("Error opening '/dev/serial/by-id'.\n");
        return NULL;
    }

    while ((entry = readdir(dir)) != NULL) {
        if (strstr(entry->d_name, DEVICE_ID)) {
            snprintf(device, sizeof(device), "/dev/serial/by-id/%s", entry->d_name);
            closedir(dir);
            return device;
        }
    }
    closedir(dir);
    return NULL;
}

void flush_serial_port(int fd) {
    tcflush(fd, TCIOFLUSH);
}

void set_serial_port(int fd) {
    struct termios options;
    tcgetattr(fd, &options);

    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);

    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;

    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_oflag &= ~OPOST;
    options.c_iflag &= ~(IXON | IXOFF | IXANY);

    tcsetattr(fd, TCSANOW, &options);

    // Ensure non-blocking mode is not set
    int flags = fcntl(fd, F_GETFL, 0);
    if (flags & O_NONBLOCK) {
        fcntl(fd, F_SETFL, flags & ~O_NONBLOCK);
    }

    // Flush the serial port to clear any buffered data
    flush_serial_port(fd);
}

int open_serial_port(const char *device) {
    serial_fd = open(device, O_RDWR | O_NOCTTY | O_SYNC);
    if (serial_fd < 0) {
        return 0;
    }
    set_serial_port(serial_fd);
    return 1;
}

void close_serial_port() {
    if (serial_fd >= 0) {
        close(serial_fd);
    }
}

int read_temperature(const char *filepath) {
    FILE *file = fopen(filepath, "r");
    if (!file) {
        printf("Error opening file: %s.\n", filepath);
        return -1;
    }

    char buffer[6];
    fseek(file, 0, SEEK_SET);
    if (!fgets(buffer, sizeof(buffer), file)) {
        printf("Error reading from file: %s.\n", filepath);
        fclose(file);
        return -1;
    }
    fclose(file);
    return (int)round(atof(buffer) / 1000);
}

int verify_serial_written(int written, int should_write) {
    if (written < 0) {
        printf("Error writing to serial port.\n");
        return 0;
    } else if (written < should_write) {
        printf("Partial write occurred.\n");
        return 0;
    }
    return 1;
}

int send_message(int fd, const char *message) {
    size_t written = write(fd, "\x02", 1);
    written += write(fd, message, strlen(message));
    written += write(fd, "\x03", 1);
    return verify_serial_written(written, strlen(message + 1));
}

int send_repeat(int fd) {
    size_t written = write(fd, "\x01", 1);
    return verify_serial_written(written, 1);
}

int send_handshake(int fd) {
    size_t written = write(fd, "\x00", 1);
    return verify_serial_written(written, 1);
}

int send_close(int fd) {
    size_t written = write(fd, "\x04", 1);
    return verify_serial_written(written, 1);
}

int wait_for_ack(int fd) {
    char ack;
    int result;
    fd_set read_fds;
    struct timeval timeout;

    timeout.tv_sec = 3;
    timeout.tv_usec = 0;

    FD_ZERO(&read_fds);
    FD_SET(fd, &read_fds);

    result = select(fd + 1, &read_fds, NULL, NULL, &timeout);

    if (result == -1) {
        printf("Error reading from serial while waiting for ACK.\n");
        return 0;
    } else if (result == 0) {
        printf("Timeout waiting for ACK.\n");
        return 0;
    } else {
        if (FD_ISSET(fd, &read_fds)) {
            result = read(fd, &ack, 1);
            if (result < 0) {
                printf("ACK read error.\n");
                return 0;
            } else if (result == 0) {
                printf("No data received. Expected ACK.\n");
                return 0;
            } else if (ack != 0x06) {
                printf("Invalid value %d\n", ack);
                return 0;
            }
            return 1;
        }
    }
}

void close_and_exit(int exit_code) {
    send_close(serial_fd);
    close_serial_port();
    exit(exit_code);
}

void handle_terminate_signal(int sig) {
    printf("Caught signal %d, cleaning up and exiting...\n", sig);
    close_and_exit(0);
}

int main() {
    signal(SIGINT, handle_terminate_signal);
    signal(SIGTERM, handle_terminate_signal);
    int reconnect = 0;

    read_config_file();

    while (1) {
        const char *device = find_device();
        if (!device) {
            printf("Device not found. Retry in %d seconds...\n", DEVICE_RECONNECT_DELAY);
            sleep(DEVICE_RECONNECT_DELAY);
            continue;
        }
        printf("Found device on %s\n", device);

        if (!open_serial_port(device)) {
            printf("Error opening serial device: %s\n", device);
            sleep(DEVICE_RECONNECT_DELAY);
            continue;
        }
        reconnect = 0;

        while (1) {
            if (reconnect) {
                break;
            }
            int cpu_temp = read_temperature("/sys/class/hwmon/hwmon4/temp1_input");
            int gpu_temp = max(
                read_temperature("/sys/class/hwmon/hwmon2/temp2_input"),
                read_temperature("/sys/class/hwmon/hwmon2/temp3_input")
            );
            int nvme_temp = read_temperature("/sys/class/hwmon/hwmon1/temp3_input");

            if (cpu_temp != prev_cpu_temp || gpu_temp != prev_gpu_temp || nvme_temp != prev_nvme_temp) {
                char message[10];
                snprintf(
                    message, sizeof(message), "%03d%03d%03d",
                    cpu_temp != -1 ? cpu_temp : 0,
                    gpu_temp != -1 ? gpu_temp : 0,
                    nvme_temp != -1 ? nvme_temp : 0
                );

                if (!send_message(serial_fd, message)) {
                    reconnect = 1;
                    close_serial_port();
                    break;
                }

                prev_cpu_temp = cpu_temp;
                prev_gpu_temp = gpu_temp;
                prev_nvme_temp = nvme_temp;
            } else {
                if (!send_repeat(serial_fd)) {
                    reconnect = 1;
                    close_serial_port();
                    break;
                }
            }
            if (!wait_for_ack(serial_fd)) {
                reconnect = 1;
                close_serial_port();
                break;
            }
            sleep(update_interval);
        }
    }

    return 0;
}
