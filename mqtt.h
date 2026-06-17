#ifndef MQTT
#define MQTT

#include <stdint.h>

/**
 * @brief Format rock data as JSON and send it to the ESP32 over UART0 with a
 *        4-byte little-endian length header (partner-robot MQTT link).
 */
void send_mqtt(int x, int y, const char* color, int size, float temp);

/**
 * @brief Enable the MQTT link (call once UART0 is up). Until then every publish
 *        is a no-op, so we never touch an uninitialised UART.
 */
void mqtt_enable(void);

/**
 * @brief Frame + send one raw JSON string over the MQTT link. Used to mirror ALL
 *        the UI telemetry (pose/status/observation/scannable) so the dashboard can
 *        run fully wireless over MQTT. No-op until mqtt_enable().
 */
void mqtt_publish_raw(const char* json);

/**
 * @brief Non-blocking-ish poll: if a framed message is waiting on UART0, parse
 *        the partner robot's rock (x, y, size, temp, color) into the outputs.
 *        color_buf must be at least 32 bytes. Leaves outputs untouched when no
 *        frame is available.
 */
void recv_mqtt(int* xz, int* yz, int* sizez, float* tempz, char* color_buf);

/**
 * @brief Route UART0 to AR0/AR1 and bring it up. NOTE: robot_main sets UART0 up
 *        itself (to control switchbox ordering vs the sensors), so this is
 *        provided for standalone use and is not called there.
 */
void init_mqtt(void);

#endif
