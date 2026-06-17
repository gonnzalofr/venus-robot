#include "mqtt.h"

#include <libpynq.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/*
 *  Partner-robot comms over UART0 (AR0/AR1) -> ESP32 -> MQTT broker.
 *  Wire format: 4-byte little-endian length, then a JSON payload.
 *
 *  Debug output goes to stderr so it never pollutes stdout (which robot_main
 *  uses for the UI telemetry stream).
 */

void init_mqtt(void) {
    switchbox_set_pin(IO_AR1, SWB_UART0_TX);
    switchbox_set_pin(IO_AR0, SWB_UART0_RX);
    uart_init(UART0);
    /* NOTE: do NOT call switchbox_init() here -- it resets the WHOLE switchbox
     * to defaults, wiping the UART pins just set above plus every sensor/stepper
     * routing. robot_main sets UART0 up itself and never calls this, but the
     * stray reset made init_mqtt a landmine for anyone who did. */
}

/* Set true by mqtt_enable() once robot_main has brought UART0 up. Guards every
 * publish so we never poke an uninitialised UART (which can hang). */
static int s_link_up = 0;
void mqtt_enable(void) { s_link_up = 1; }

/* Frame one JSON string (4-byte LE length + payload) onto UART0 -> ESP32 -> MQTT.
 * No-op until the link is enabled. */
void mqtt_publish_raw(const char* json) {
    if (!s_link_up || json == NULL) return;
    uint32_t len = (uint32_t)strlen(json);
    if (len == 0 || len > 1024) return;            /* sanity bound */
    uart_send(UART0, (len >> 0) & 0xFF);
    uart_send(UART0, (len >> 8) & 0xFF);
    uart_send(UART0, (len >> 16) & 0xFF);
    uart_send(UART0, (len >> 24) & 0xFF);
    for (uint32_t i = 0; i < len; i++) uart_send(UART0, (uint8_t)json[i]);
}

void send_mqtt(int x, int y, const char* color, int size, float temp) {
    char json_payload[160];
    snprintf(json_payload, sizeof(json_payload),
             "{\"type\":\"ROCK\",\"x\":\"%i\",\"y\":\"%i\",\"color\":\"%s\","
             "\"size\":%d,\"temp\":%.2f}",
             x, y, color, size, temp);
    mqtt_publish_raw(json_payload);
    fprintf(stderr, "[mqtt] sent ROCK: %s\n", json_payload);
}

/*
 * NON-BLOCKING. Drains only the bytes currently available on UART0 into a
 * persistent buffer, then parses ONE complete length-prefixed frame if a
 * whole one has arrived. Never calls uart_recv() unless uart_has_data() is
 * true, so it can never stall the caller's loop on a partial frame.
 */
void recv_mqtt(int* xz, int* yz, int* sizez, float* tempz, char* color_buf) {
    static uint8_t buf[2048];
    static size_t  buf_len = 0;

    /* Pull in whatever is waiting -- bounded, never blocks. */
    while (uart_has_data(UART0) && buf_len < sizeof(buf)) {
        buf[buf_len++] = uart_recv(UART0);
    }

    if (buf_len < 4) return;                       /* not even a length yet */

    uint32_t len = (uint32_t)buf[0]
                 | ((uint32_t)buf[1] << 8)
                 | ((uint32_t)buf[2] << 16)
                 | ((uint32_t)buf[3] << 24);

    if (len == 0 || len >= sizeof(buf) - 4) {      /* bogus header: resync */
        memmove(buf, buf + 1, --buf_len);
        return;
    }
    if (buf_len < 4 + (size_t)len) return;         /* frame not complete yet */

    char json_buffer[len + 1];
    memcpy(json_buffer, buf + 4, len);
    json_buffer[len] = '\0';

    /* consume this frame from the buffer */
    size_t consumed = 4 + (size_t)len;
    memmove(buf, buf + consumed, buf_len - consumed);
    buf_len -= consumed;

    /* Only ROCK frames matter to the partner robot. UI frames (pose/status/etc.)
     * now share this link too (so the dashboard can run over MQTT) -- ignore them
     * here so they don't get mis-parsed as a rock. */
    if (strstr(json_buffer, "\"type\":\"ROCK\"") == NULL) return;

    int x = 0, y = 0, size = 0;
    float temp = 0.0f;
    char *p;

    if ((p = strstr(json_buffer, "\"x\":\"")))   x = atoi(p + 5);
    if ((p = strstr(json_buffer, "\"y\":\"")))   y = atoi(p + 5);
    if ((p = strstr(json_buffer, "\"size\":")))  size = atoi(p + 7);
    if ((p = strstr(json_buffer, "\"temp\":")))  temp = (float)atof(p + 7);

    if ((p = strstr(json_buffer, "\"color\":\""))) {
        char *start = p + 9;
        char *end = strchr(start, '\"');
        if (end) {
            size_t color_len = (size_t)(end - start);
            if (color_len > 31) color_len = 31;
            strncpy(color_buf, start, color_len);
            color_buf[color_len] = '\0';
        }
    }

    fprintf(stderr, "[mqtt] partner rock: x=%d y=%d color=%s size=%d temp=%.2f\n",
            x, y, color_buf, size, temp);

    *xz = x;
    *yz = y;
    *sizez = size;
    *tempz = temp;
}
