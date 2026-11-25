from controller import Supervisor
from pathlib import Path
from math import dist

robot = Supervisor()
time_step = int(robot.getBasicTimeStep())

# Flight settings
hover_height = 6
takeoff_duration = 4.0
settle_duration = 5.0
flight_speed = 1.5
landing_duration = 4.0
# Move all dirt objects below the panels
for i in range(1, 29):

    dirt = robot.getFromDef(f"DIRT_{i}")

    if dirt is not None:

        translation = dirt.getField("translation")
        current_position = translation.getSFVec3f()

        translation.setSFVec3f([
            current_position[0],
            current_position[1],
            current_position[2] - 0.4
        ])

        print(f"DIRT_{i} REMOVED")

    else:
        print(f"DIRT_{i} NOT FOUND")
# Solar panel coordinates (X, Y, Z)
panels = [
    (0.41, -0.02, 3.74),
    (0.41, 3.03, 3.74),
    (0.41, 5.93, 3.74),
    (0.41, 8.62, 3.74),
    (0.41, 11.4, 3.74),
    (0.41, 14.5, 3.74),
    (0.41, 17.3, 3.74),

    (-3.97, 0, 3.74),
    (-3.97, 3.11, 3.74),
    (-3.97, 6.02, 3.74),
    (-3.97, 8.72, 3.74),
    (-3.97, 11.3, 3.74),
    (-3.97, 14.2, 3.74),
    (-3.97, 16.9, 3.74),

    (-7, 0, 3.74),
    (-7, 3.11, 3.74),
    (-7, 6.02, 3.74),
    (-7, 8.72, 3.74),
    (-7, 11.3, 3.74),
    (-7, 14.2, 3.74),
    (-7, 16.9, 3.74),

    (-10.5, 0, 3.74),
    (-10.5, 3.11, 3.74),
    (-10.5, 6.02, 3.74),
    (-10.5, 8.72, 3.74),
    (-10.5, 11.3, 3.74),
    (-10.5, 14.2, 3.74),
    (-10.5, 16.9, 3.74)
]

# Image folder
image_folder = Path(
    "/Users/.../solar_panel_watering/panel_images"
)

image_folder.mkdir(parents=True, exist_ok=True)

# Camera
camera = robot.getDevice("camera")
camera.enable(time_step)

# Webots visual animation
drone = robot.getSelf()
position = drone.getField("translation")

# Home position
home = position.getSFVec3f()

flight_altitude = max(
    home[2] + hover_height,
    max(panel[2] for panel in panels) + hover_height
)

home_hover = [
    home[0],
    home[1],
    flight_altitude
]

# Move drone
def set_position(target):
    position.setSFVec3f(list(target))
    drone.resetPhysics()


# Smooth movement between two positions
def interpolate(start, target, progress):
    return [
        start[i] + (target[i] - start[i]) * progress
        for i in range(3)
    ]


# Mission settings
state = "TAKEOFF"
state_start_time = robot.getTime()

panel_index = 0
move_start = home.copy()
move_target = home_hover.copy()
move_duration = takeoff_duration

print("MISSION STARTED")

while robot.step(time_step) != -1:

    current_time = robot.getTime()
    elapsed = current_time - state_start_time

    # 1. TAKEOFF
    if state == "TAKEOFF":

        progress = min(elapsed / takeoff_duration, 1.0)

        set_position(
            interpolate(home, home_hover, progress)
        )

        if progress >= 1.0:

            state = "INITIAL_HOLD"
            state_start_time = current_time

            print("TAKEOFF COMPLETED")

    # 2. INITIAL HOLD
    elif state == "INITIAL_HOLD":

        set_position(home_hover)

        if elapsed >= settle_duration:

            panel = panels[panel_index]

            move_start = position.getSFVec3f()

            move_target = [
                panel[0],
                panel[1],
                flight_altitude
            ]

            move_duration = max(
                dist(move_start, move_target) / flight_speed,
                0.1
            )

            state = "MOVE_TO_PANEL"
            state_start_time = current_time

            print(f"MOVING TO PANEL {panel_index + 1}")

    # 3. MOVE TO PANEL
    elif state == "MOVE_TO_PANEL":

        progress = min(elapsed / move_duration, 1.0)

        set_position(
            interpolate(move_start, move_target, progress)
        )

        if progress >= 1.0:

            state = "PANEL_HOLD"
            state_start_time = current_time

            print(f"PANEL {panel_index + 1} REACHED")

    # 4. HOLD ABOVE PANEL AND TAKE IMAGE
    elif state == "PANEL_HOLD":

        set_position(move_target)

        if elapsed >= settle_duration:

            panel_id = panel_index + 1

            filename = (
                f"panel_{panel_id:02d}_"
                f"height_{hover_height:.2f}m.jpg"
            )

            image_path = image_folder / filename

            result = camera.saveImage(str(image_path), 100)

            if result != 0:

                print(f"IMAGE SAVE FAILED: PANEL {panel_id}")

                state_start_time = current_time
                continue

            print(f"IMAGE SAVED: {image_path}")

            panel_index += 1

            # Move to the next panel
            if panel_index < len(panels):

                panel = panels[panel_index]

                move_start = position.getSFVec3f()

                move_target = [
                    panel[0],
                    panel[1],
                    flight_altitude
                ]

                move_duration = max(
                    dist(move_start, move_target) / flight_speed,
                    0.1
                )

                state = "MOVE_TO_PANEL"
                state_start_time = current_time

                print(f"MOVING TO PANEL {panel_index + 1}")

            # All panels completed
            else:

                print("ALL 28 PANELS COMPLETED")

                move_start = position.getSFVec3f()
                move_target = home_hover.copy()

                move_duration = max(
                    dist(move_start, move_target) / flight_speed,
                    0.1
                )

                state = "RETURN_HOME"
                state_start_time = current_time

                print("RETURNING TO STATION")

    # 5. RETURN TO STATION
    elif state == "RETURN_HOME":

        progress = min(elapsed / move_duration, 1.0)

        set_position(
            interpolate(move_start, move_target, progress)
        )

        if progress >= 1.0:

            state = "LANDING"
            state_start_time = current_time

            print("STATION REACHED")

    # 6. LANDING
    elif state == "LANDING":

        progress = min(elapsed / landing_duration, 1.0)

        set_position(
            interpolate(home_hover, home, progress)
        )

        if progress >= 1.0:

            state = "COMPLETED"

            print("LANDING COMPLETED")
            print("MISSION COMPLETED")

    # 7. MISSION COMPLETED
    elif state == "COMPLETED":

        set_position(home)