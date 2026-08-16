#ifndef MQTT
#define MQTT

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Send a raw JSON string to the ESP32 over UART0 with a 4-byte
 *        little-endian length header (the partner-robot / mission-control link).
 *        Used for pose / status / object telemetry; send_mqtt() builds on it.
 */
void mqtt_publish_json(const char* json);

/**
 * @brief Format rock data as JSON and send it to the ESP32 over UART0 with a
 *        4-byte little-endian length header (partner-robot MQTT link).
 */
void send_mqtt(int x, int y, const char* color, int size, float temp);

/**
 * @brief Non-blocking-ish poll: if a framed message is waiting on UART0, parse
 *        the partner robot's rock (x, y, size, temp, color) into the outputs.
 *        color_buf must be at least 32 bytes. Leaves outputs untouched when no
 *        frame is available.
 * @return true if a complete frame was parsed into the outputs this call, false
 *         otherwise. Call in a `while (recv_mqtt(...))` loop to drain every queued
 *         frame in one poll (one frame is parsed per call).
 */
bool recv_mqtt(int* xz, int* yz, int* sizez, float* tempz, char* color_buf);

/**
 * @brief Route UART0 to AR0/AR1 and bring it up. NOTE: robot_main sets UART0 up
 *        itself (to control switchbox ordering vs the sensors), so this is
 *        provided for standalone use and is not called there.
 */
void init_mqtt(void);

#endif
