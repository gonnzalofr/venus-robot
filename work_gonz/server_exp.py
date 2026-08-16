import pygame
import paho.mqtt.client as mqtt
import threading
import sys

# =========================================
# SETTINGS
# =========================================

WIDTH = 1300
HEIGHT = 850
GRID = 10

BROKER_HOST = "mqtt.ics.ele.tue.nl"


# =========================================
# COLORS
# =========================================

WHITE = (240, 240, 240)
BLACK = (0, 0, 0)
GREEN = (0, 180, 0)
RED = (200, 0, 0)
BLUE = (0, 0, 255)
GRAY = (200, 200, 200)
BROWN = (120, 70, 20)
PINK = (255, 105, 180)
SILVER = (192, 192, 192)
YELLOW = (255, 215, 0)
ORANGE = (255, 165, 0)

ROCK_TYPES = {
    "green3": {"color": GREEN, "draw_size": 0.5},
    "green6": {"color": (0, 255, 0), "draw_size": 1},
    "red": {"color": RED, "draw_size": 0.5},
    "blue": {"color": BLUE, "draw_size": 0.5},
    "black": {"color": BLACK, "draw_size": 0.5},
    "white": {"color": SILVER, "draw_size": 0.5},
}

# =========================================
# COLORS
# =========================================
def get_object_size(terrain_type):
    sizes = {
        "rock": (1, 1),
        "hill": (5, 5),
        "cliff": (7, 3),
        "boundary": (10, 1),
        "robotA": (2, 2),
        "robotB": (2, 2)
    }

    return sizes.get(terrain_type, (1, 1))

def make_id(obj_type, x, y):
    return f"{obj_type}_{x//2}_{y//2}"

# =========================================
# MAP STORAGE
# =========================================

# stores terrain like:
# terrain_map[(x, y)] = "rock"

terrain_map = {}
robots = {}
rock_data = {}

# =========================================
# MQTT MESSAGE HANDLER
# =========================================

# FORMAT

def extract_coordinates(message):
    parts = message.split(",")

    x = None
    y = None
    obj_type = None

    for part in parts:
        part = part.strip()

        if part.startswith("x="):
            x = float(part.split("=")[1])

        elif part.startswith("y="):
            y = float(part.split("=")[1])

        elif part.startswith("t="):
            obj_type = part.split("=")[1].strip()

    if x is None or y is None or obj_type is None:
        return None

    return x, y, obj_type

# MQTT MESSAGE HANDLER
def messageHandler(client, userdata, message):
    global terrain_map

    try:
        text = message.payload.decode().strip()
        print(f"\n[Received on {message.topic}]: {text}")
        
        # IGNORE EVENT / DEBUG MESSAGES
        if not text.startswith("POS"):
            return

        result = extract_coordinates(text)

        if result is None:
            print("Invalid POS message format:", text)
            return

        x, y, obj_type = result

        # center scaling
        screen_x = (WIDTH // 2) + x
        screen_y = (HEIGHT // 2) - y

        grid_x = int(screen_x // GRID)
        grid_y = int(screen_y // GRID)

        # ROBOTS
        if obj_type in ["robotA", "robotB"]:
            robots[obj_type] = (grid_x, grid_y)
            return

        # TERRAIN
        obj_id = f"{obj_type}_{grid_x}_{grid_y}"
        terrain_map[obj_id] = (obj_type, grid_x, grid_y)

        # TEMPERATURE TRACKING
        if obj_type in ROCK_TYPES:
            rock_data[(grid_x, grid_y)] = {
                "type": obj_type,
                "temp": None
            }

    except Exception as e:
        print("Error processing incoming message data:", e)
        
# =========================================
# MQTT THREAD
# =========================================
# BROKER CLASS ARCHITECTURE
class Broker:
    def __init__(self, username, password, topicSubList):
        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        self.client.username_pw_set(username, password)
        self.client.on_message = messageHandler

        try:
            if self.client.connect(host=BROKER_HOST) != 0:
                raise RuntimeError(f"Could not connect to {BROKER_HOST}")

            for topic in topicSubList:
                self.client.subscribe(topic)

            # Instantly forks off a background thread for continuous listening
            self.client.loop_start()
            print(f"Connected background worker for {username}")
        except Exception as err:
            print(f"Setup Error for {username}: {err}")

    def send_message(self, topic, text_payload):
        result = self.client.publish(topic, text_payload)
        if result.rc == mqtt.MQTT_ERR_SUCCESS:
            print(f"Sent successfully to {topic}: {text_payload}")
        else:
            print(f"Failed transmission to {topic}")

    def close(self):
        self.client.loop_stop()
        self.client.disconnect()

# PYGAME ENGINE INITIALIZATION
pygame.init()
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Venus Terrain Map - Dual Robot System")
clock = pygame.time.Clock()
font = pygame.font.SysFont("Calibri", 12)

# =========================================
# DRAW LEGEND
# =========================================

def draw_legend():

    # pushed further right
    legend_x = WIDTH - 110
    legend_y = 40

    # smaller width
    # pygame.draw.rect(screen, WHITE, (legend_x - 10, legend_y - 10, 110, 260))
    # pygame.draw.rect(screen, BLACK, (legend_x - 10, legend_y - 10, 110, 260), 2)

    title = font.render("Legend", True, BLACK)
    screen.blit(title, (legend_x, legend_y))

    legend_y += 25

    items = [
        ("Green 3x3", GREEN),
        ("Green 6x6", (0, 255, 0)),
        ("Red 3x3", RED),
        ("Blue 3x3", BLUE),
        ("Black 3x3", BLACK),
        ("White 3x3", SILVER),
        ("Boundary", BLACK),
        ("Hill", BROWN),
        ("Cliff", YELLOW),
        ("Robot A", ORANGE),
        ("Robot B", PINK)
    ]

    for name, color in items:

        # ROCKS = outlined squares
        if "3x3" in name or "6x6" in name:

            pygame.draw.rect(
                screen,
                color,
                (legend_x, legend_y, 12, 12),
                2
            )

        # ROBOTS = circles
        elif "Robot" in name:

            pygame.draw.circle(
                screen,
                color,
                (legend_x + 6, legend_y + 6),
                6
            )

        # BOUNDARY = filled black square
        elif name == "Boundary":

            pygame.draw.rect(
                screen,
                BLACK,
                (legend_x, legend_y, 12, 12)
            )

        # HILLS + CLIFFS = filled squares
        else:

            pygame.draw.rect(
                screen,
                color,
                (legend_x, legend_y, 12, 12)
            )

        text = font.render(name, True, BLACK)

        screen.blit(text, (legend_x + 20, legend_y - 2))

        legend_y += 20
        
        
# =========================================
# DRAW TEMPERATURE LIST
# =========================================
def draw_temperatures():

    temp_x = WIDTH - 110
    temp_y = 320

    title = font.render("Temperatures", True, BLACK)
    screen.blit(title, (temp_x, temp_y))

    temp_y += 25

    for (gx, gy), data in rock_data.items():

        rock_type = data["type"]
        temp = data["temp"]

        if temp is None:
            continue

        rock = ROCK_TYPES[rock_type]
        color = rock["color"]
        size_factor = rock["draw_size"]

        symbol_size = 12 if size_factor == 1 else 6

        # rock symbol
        pygame.draw.rect(
            screen,
            color,
            (
                temp_x,
                temp_y,
                symbol_size,
                symbol_size
            ),
            2
        )

        # temp text
        text = font.render(
            f"{temp:.1f} C",
            True,
            BLACK
        )

        screen.blit(text, (temp_x + 20, temp_y - 2))

        temp_y += 20
# =========================================
# DRAW MAP
# =========================================    
def draw():

    screen.fill(WHITE)

    # =========================================
    # DRAW GRID
    # =========================================

    for x in range(0, WIDTH, GRID):
        pygame.draw.line(screen, GRAY, (x, 0), (x, HEIGHT))

    for y in range(0, HEIGHT, GRID):
        pygame.draw.line(screen, GRAY, (0, y), (WIDTH, y))

    # =========================================
    # DRAW AXES
    # =========================================

    # x-axis (center horizontal)
    pygame.draw.line(
        screen,
        (128, 128, 128),
        (0, HEIGHT // 2),
        (WIDTH, HEIGHT // 2),
        2
    )

    # y-axis (center vertical)
    pygame.draw.line(
        screen,
        (128, 128, 128),
        (WIDTH // 2, 0),
        (WIDTH // 2, HEIGHT),
        2
    )
    
    # =========================================
    # DRAW AXIS NUMBERS
    # =========================================

    # x-axis numbers

    for x in range(0, WIDTH, GRID * 2):

        coord_x = x - (WIDTH // 2)

        number = font.render(str(coord_x), True, BLACK)

        rotated = pygame.transform.rotate(number, 90)

        screen.blit(rotated, (x + 2, 2)) 
                    
    # y-axis number
    for y in range(0, HEIGHT, GRID * 2):

        coord_y = (HEIGHT // 2) - y

        number = font.render(str(coord_y), True, BLACK)

        screen.blit(number, (2, y + 2))

    # =========================================
    # DRAW TERRAIN
    # =========================================

    for obj_id,(obj_type, gx, gy)in terrain_map.items():

        # object size in grid cells
        obj_w, obj_h = get_object_size(obj_type)

        # convert grid coordinate to pixel center
        center_x = gx * GRID
        center_y = gy * GRID

        # convert object size to pixels
        pixel_w = obj_w * GRID
        pixel_h = obj_h * GRID

        # shift so coordinate becomes CENTER
        x = center_x - (pixel_w // 2)
        y = center_y - (pixel_h // 2)

        if obj_type in ROCK_TYPES:

            rock = ROCK_TYPES[obj_type]
            color = rock["color"]
            size_factor = rock["draw_size"]

            pixel_size = int(GRID * size_factor)

            # center smaller rocks
            offset = (GRID - pixel_size) // 2

            pygame.draw.rect(
                screen,
                color,
                (
                    x + offset,
                    y + offset,
                    pixel_size,
                    pixel_size
                ),
                2
            )


        # HILL
        elif obj_type == "hill":

            pygame.draw.rect(
                screen,
                BROWN,
                (x, y, pixel_w, pixel_h)
            )

        # CLIFF
        elif obj_type == "cliff":

            pygame.draw.rect(
                screen,
                YELLOW,
                (x, y, pixel_w, pixel_h)
            )

        # BOUNDARY
        elif obj_type == "boundary":

            pygame.draw.rect(
                screen,
                BLACK,
                (x, y, GRID, GRID)
            )

        # ROBOTS
        # elif terrain_type == "robotA":

           # pygame.draw.rect(
                # screen,
                # ORANGE,
                # (x, y, GRID, GRID)
            
            
        # elif terrain_type == "robotB":

            # pygame.draw.rect(
               # screen,
               # PINK,
               # (x, y, GRID, GRID)
            # )
            
        else:

            pygame.draw.rect(
                screen,
                BROWN,
                (x, y, GRID, GRID)
            )
    
    # DRAW ROBOTS (always on top)
    for robot_id, (x, y) in robots.items():

        color = ORANGE if robot_id == "robotA" else PINK

        pygame.draw.circle(
        screen,
        color,
        (x * GRID, y * GRID),
        GRID
        )
    

    # =========================================
    # TITLE
    # =========================================

    title = font.render("Venus Terrain Map", True, BLACK)

    screen.blit(title, (10, HEIGHT - 30))

    draw_legend()
    draw_temperatures()

    pygame.display.flip()

# =========================================
# START MQTT THREAD
# =========================================

# =========================================
# MAIN LOOP
# =========================================

# INSTANTIATE BOTH ACTIVE CONNECTIONS
broker_75 = Broker("robot_75_1", "REDACTED_SEE_ARCHIVE", ["/pynqbridge/75/send"])
broker_73 = Broker("robot_73_1", "REDACTED_SEE_ARCHIVE", ["/pynqbridge/73/send"])

# TERMINAL USER INPUT IN THE BACKGROUND
def terminal_input_thread():
    """ Runs completely parallel to let you type messages without halting visuals """
    print("\n--- Terminal Prompt Active ---")
    print("Type your message here and press Enter to broadcast to BOTH robots simultaneously.\n")
    try:
        while running:
            user_msg = input("Enter Command: ")
            if user_msg.strip().lower() == 'exit':
                break
            if user_msg.strip():
                broker_75.send_message("/pynqbridge/75/recv", user_msg)
                broker_73.send_message("/pynqbridge/73/recv", user_msg)
    except Exception as e:
        print("Terminal prompt shut down:", e)

# Spin up a small thread dedicated specifically to keeping your terminal active
running = True
threading.Thread(target=terminal_input_thread, daemon=True).start()

# CORE PYGAME EXECUTION LOOP
# Cleaned up your old string rendering syntax error here too!
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # Keeps rendering incoming data variations instantly onto your monitor 
    draw()
    clock.tick(30)

# RESOURCE CLEANUP ON TERMINATION
print("\nShutting down connections...")
running = False
broker_75.close()
broker_73.close()
pygame.quit()
sys.exit()