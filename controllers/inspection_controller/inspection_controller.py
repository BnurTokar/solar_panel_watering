"""Webots Supervisor controller for a single 28-panel inspection round."""

from math import dist
from controller import Supervisor

robot = Supervisor()
step_ms = int(robot.getBasicTimeStep())
camera = robot.getDevice("camera")
camera.enable(step_ms)

drone = robot.getSelf()
position = drone.getField("translation")
home = list(position.getSFVec3f())

# Each row gives the world X coordinate and the ordered panel Y coordinates.
rows = [
    (0.41, [-0.02, 3.03, 5.93, 8.62, 11.4, 14.5, 17.3]),
    (-3.97, [0, 3.11, 6.02, 8.72, 11.3, 14.2, 16.9]),
    (-7.0, [0, 3.11, 6.02, 8.72, 11.3, 14.2, 16.9]),
    (-10.5, [0, 3.11, 6.02, 8.72, 11.3, 14.2, 16.9]),
]

panel_z = 3.74
clearance = 5.5
flight_speed = 1.5
flight_z = max(home[2], panel_z) + clearance
home_hover = [home[0], home[1], flight_z]


def step_at(point):
    position.setSFVec3f(point)
    drone.resetPhysics()
    if robot.step(step_ms) == -1:
        raise SystemExit


def move_to(target, duration):
    start = list(position.getSFVec3f())
    start_time = robot.getTime()
    while True:
        elapsed = robot.getTime() - start_time
        progress = min(elapsed / duration, 1.0)
        point = [a + (b - a) * progress for a, b in zip(start, target)]
        step_at(point)
        if progress >= 1.0:
            return


def cruise_to(target):
    duration = max(dist(position.getSFVec3f(), target) / flight_speed, 0.1)
    move_to(target, duration)


def hold_at(point, seconds):
    start_time = robot.getTime()
    while robot.getTime() - start_time < seconds:
        step_at(point)


print("Inspection started: taking off.")
move_to(home_hover, 4.0)
hold_at(home_hover, 3.0)

panel_id = 0
for x, y_values in rows:
    for y in y_values:
        panel_id += 1
        name = f"panel_{panel_id:02d}"
        target = [x, y, flight_z]

        print(f"Moving to {name}.")
        cruise_to(target)
        hold_at(target, 2.0)

        frame = camera.getImage()
        while not frame:
            print(f"{name}: waiting for a camera frame.")
            hold_at(target, 1.0)
            frame = camera.getImage()
                        
        print(f"{name}: frame captured.")
        # TODO: Classify frame here and record name if the panel is dirty.

print("All 28 panels inspected. Returning to base.")
cruise_to(home_hover)
move_to(home, 4.0)
print("Landing complete.")
# TODO: Send the recorded dirty panel names to cleaning_drone here.

while True:
    step_at(home)
