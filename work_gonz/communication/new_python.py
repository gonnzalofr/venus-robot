import paho.mqtt.client as mqtt
import json
import datetime

# --- SETTINGS ---
BROKER = "mqtt.ics.ele.tue.nl"

# Robot A Credentials
USER_A = "robot_66_1"
PASS_A = "cSTY2NJS"
ID_A   = "66"

# Robot B Credentials
USER_B = "robot_6_1"
PASS_B = "f8rWApIb"
ID_B   = "6"

# Topics
A_SEND = f"/pynqbridge/{ID_A}/send"
A_RECV = f"/pynqbridge/{ID_A}/recv"

B_SEND = f"/pynqbridge/{ID_B}/send"
B_RECV = f"/pynqbridge/{ID_B}/recv"

LOG_FILE = "pynq_mission_log.json"

# --- MEMORY ---
last_known_data = {
    ID_A: None,
    ID_B: None
}

# --- SHARED LOGGING FUNCTION ---

def log_and_parse(robot_id, payload):

    try:

        raw_payload = payload.decode(
            "utf-8",
            errors="ignore"
        )

        start_index = raw_payload.find('{')

        if start_index != -1:

            clean_json = raw_payload[start_index:]

            mission_data = json.loads(clean_json)

            log_entry = {

                "timestamp":
                datetime.datetime.now().strftime(
                    "%Y-%m-%d %H:%M:%S"
                ),

                "sender_id": robot_id,

                "data": mission_data
            }

            with open(LOG_FILE, "a") as f:
                f.write(json.dumps(log_entry) + "\n")

            print(
                f"\nLOG: Captured new data from {robot_id}"
            )

            print(f"DATA: {mission_data}")

            return True

    except Exception as e:

        print(f"LOG ERROR: {e}")

    return False

# --- CALLBACKS ---

def on_message_from_A(client, userdata, msg):

    global last_known_data

    try:

        raw_payload = msg.payload.decode(
            "utf-8",
            errors="ignore"
        )

        start_index = raw_payload.find('{')

        if start_index != -1:

            mission_data = json.loads(
                raw_payload[start_index:]
            )

            if mission_data == last_known_data[ID_A]:
                return

            last_known_data[ID_A] = mission_data

    except Exception:
        pass

    log_and_parse(ID_A, msg.payload)

    client_b.publish(B_RECV, msg.payload)

    print(
        f"RELAY: {ID_A} >>> {ID_B}"
    )

def on_message_from_B(client, userdata, msg):

    global last_known_data

    try:

        raw_payload = msg.payload.decode(
            "utf-8",
            errors="ignore"
        )

        start_index = raw_payload.find('{')

        if start_index != -1:

            mission_data = json.loads(
                raw_payload[start_index:]
            )

            if mission_data == last_known_data[ID_B]:
                return

            last_known_data[ID_B] = mission_data

    except Exception:
        pass

    log_and_parse(ID_B, msg.payload)

    client_a.publish(A_RECV, msg.payload)

    print(
        f"RELAY: {ID_B} >>> {ID_A}"
    )

# --- INITIALIZE CLIENTS ---

client_a = mqtt.Client(
    callback_api_version=
    mqtt.CallbackAPIVersion.VERSION1
)

client_a.username_pw_set(
    USER_A,
    PASS_A
)

client_a.on_message = on_message_from_A

client_b = mqtt.Client(
    callback_api_version=
    mqtt.CallbackAPIVersion.VERSION1
)

client_b.username_pw_set(
    USER_B,
    PASS_B
)

client_b.on_message = on_message_from_B

# --- RUN LOOP ---

try:

    print(
        "Starting Bi-Directional Bridge..."
    )

    client_a.connect(BROKER)

    client_b.connect(BROKER)

    client_a.subscribe(A_SEND)

    client_b.subscribe(B_SEND)

    print(
        "Bridge Active. Listening..."
    )

    client_a.loop_start()

    client_b.loop_forever()

except KeyboardInterrupt:

    print("\nShutting down bridge...")

    client_a.loop_stop()

    client_b.loop_stop()

    client_a.disconnect()

    client_b.disconnect()