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

void mqtt_publish_json(const char* json) {
    uint32_t len = (uint32_t)strlen(json);

    uart_send(UART0, (len >> 0) & 0xFF);
    uart_send(UART0, (len >> 8) & 0xFF);
    uart_send(UART0, (len >> 16) & 0xFF);
    uart_send(UART0, (len >> 24) & 0xFF);

    for (uint32_t i = 0; i < len; i++) {
        uart_send(UART0, json[i]);
    }

    fprintf(stderr, "[mqtt] sent (%u bytes): %s\n", len, json);
}

void send_mqtt(int x, int y, const char* color, int size, float temp) {
    char json_payload[160];

    snprintf(json_payload, sizeof(json_payload),
             "{\"type\":\"ROCK\",\"x\":\"%i\",\"y\":\"%i\",\"color\":\"%s\","
             "\"size\":%d,\"temp\":%.2f}",
             x, y, color, size, temp);
    mqtt_publish_json(json_payload);
}

/*
 * NON-BLOCKING. Drains the bytes currently waiting on UART0 into a persistent
 * buffer, then extracts ONE complete JSON object if a whole one has arrived.
 *
 * It locates the object by scanning for '{' .. '}' rather than trusting a fixed
 * length header, so it works whether or not the pynqbridge keeps the 4-byte
 * length prefix in front of the payload (any leading framing bytes before '{'
 * are simply skipped). This mirrors how mission_control.py finds the JSON, and
 * is what lets robot 1 reliably ingest robot 2's relayed cube reports.
 *
 * Our messages are flat (no nested braces, no '}' inside string values), so the
 * first '}' closes the object. Returns true once per parsed object; call it in a
 * `while (recv_mqtt(...))` loop to drain every queued frame. color_buf is set
 * only for messages that carry a "color" (i.e. ROCK cube reports); it is cleared
 * otherwise, so the caller can ignore relayed pose/status/object messages.
 */
bool recv_mqtt(int* xz, int* yz, int* sizez, float* tempz, char* color_buf) {
    static uint8_t buf[2048];
    static size_t  buf_len = 0;

    /* Pull in whatever is waiting -- bounded, never blocks. */
    while (uart_has_data(UART0) && buf_len < sizeof(buf)) {
        buf[buf_len++] = uart_recv(UART0);
    }
    if (buf_len == 0) return false;

    /* Find the start of a JSON object (skip any length/framing bytes). */
    size_t s = 0;
    while (s < buf_len && buf[s] != '{') s++;
    if (s >= buf_len) { buf_len = 0; return false; }   /* only noise -> drop it */

    /* Find the end of the (flat) object. */
    size_t e = s + 1;
    while (e < buf_len && buf[e] != '}') e++;
    if (e >= buf_len) {                                /* not fully arrived yet */
        if (s > 0) { memmove(buf, buf + s, buf_len - s); buf_len -= s; }  /* drop junk */
        else if (buf_len == sizeof(buf)) buf_len = 0;  /* overflow w/o close -> reset */
        return false;
    }

    const size_t json_len = e - s + 1;
    char json_buffer[json_len + 1];
    memcpy(json_buffer, buf + s, json_len);
    json_buffer[json_len] = '\0';

    /* Consume everything through this object (incl. any leading framing bytes). */
    const size_t consumed = e + 1;
    memmove(buf, buf + consumed, buf_len - consumed);
    buf_len -= consumed;

    /* Parse -- tolerant of quoted ("x":"15") or bare ("x":15) values. */
    int x = 0, y = 0, size = 0;
    float temp = 0.0f;
    char *p;
    color_buf[0] = '\0';   /* only ROCK carries colour; cleared so non-ROCK is ignored */

    if ((p = strstr(json_buffer, "\"x\":")))    { p += 4; if (*p == '"') p++; x = atoi(p); }
    if ((p = strstr(json_buffer, "\"y\":")))    { p += 4; if (*p == '"') p++; y = atoi(p); }
    if ((p = strstr(json_buffer, "\"size\":"))) { p += 7; if (*p == '"') p++; size = atoi(p); }
    if ((p = strstr(json_buffer, "\"temp\":"))) { p += 7; if (*p == '"') p++; temp = (float)atof(p); }

    /* Treat this as a partner CUBE only if it is an explicit ROCK message. The
     * caller (poll_partner_mqtt) registers a cube only when color_buf is set, so
     * relayed pose/status/object frames (no ROCK, no colour) are ignored -- even
     * if a status "detail" string happened to contain the substring "color". */
    if (strstr(json_buffer, "\"ROCK\"") &&
        (p = strstr(json_buffer, "\"color\":\""))) {
        char *start = p + 9;
        char *end = strchr(start, '\"');
        if (end) {
            size_t color_len = (size_t)(end - start);
            if (color_len > 31) color_len = 31;
            strncpy(color_buf, start, color_len);
            color_buf[color_len] = '\0';
        }
    }

    fprintf(stderr, "[mqtt] rx frame: x=%d y=%d color=%s size=%d temp=%.2f\n",
            x, y, color_buf, size, temp);

    *xz = x;
    *yz = y;
    *sizez = size;
    *tempz = temp;
    return true;
}
