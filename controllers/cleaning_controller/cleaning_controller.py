"""Webots Mavic 2 Pro temizlik testi.

1. Kalkış noktasının 2 metre üzerine çık.
2. Hedef konumda kesintisiz 5 saniye kal.
3. Su konisini 5 saniye görünür yap.
4. Suyu kapat ve aynı konumda havada bekle.
"""

from math import cos, hypot, isfinite, sin

from controller import Supervisor


def clamp(value, minimum, maximum):
    return min(max(value, minimum), maximum)


def find_water_transparency(robot):
    """Drone'un bodySlot alanındaki su konisinin transparency alanını bul."""
    drone_node = robot.getSelf()
    if drone_node is None:
        raise RuntimeError(
            "Drone düğümüne erişilemedi. "
            "cleaning_drone üzerinde supervisor alanını TRUE yap."
        )

    body_slot = drone_node.getField("bodySlot")
    if body_slot is None:
        raise RuntimeError(
            "bodySlot alanı bulunamadı. Bu controller'ın "
            "cleaning_drone Mavic2Pro modeline atandığını kontrol et."
        )

    nodes = [body_slot.getMFNode(i) for i in range(body_slot.getCount())]
    cone_shapes = []

    while nodes:
        node = nodes.pop()
        if node is None:
            continue

        if node.getTypeName() == "Shape":
            geometry = node.getField("geometry").getSFNode()
            if geometry is not None and geometry.getTypeName() == "Cone":
                cone_shapes.append(node)
            continue

        children = node.getField("children")
        if children is not None:
            for i in range(children.getCount()):
                nodes.append(children.getMFNode(i))

    if len(cone_shapes) != 1:
        raise RuntimeError(
            f"cleaning_drone bodySlot içinde {len(cone_shapes)} adet Cone bulundu. "
            "Bu test için su konisini bodySlot > Pose > children > Shape "
            "altına yerleştir ve Shape'in geometry alanını Cone yap."
        )

    appearance = cone_shapes[0].getField("appearance").getSFNode()
    if appearance is None or appearance.getTypeName() != "PBRAppearance":
        raise RuntimeError(
            "Su konisi bulundu. Koniyi içeren Shape'in appearance alanına "
            "PBRAppearance ekle."
        )

    return appearance.getField("transparency")


def run_robot(robot):
    time_step = int(robot.getBasicTimeStep())

    # Animasyon ayarları
    takeoff_duration = 4.0
    transit_duration = 8.0
    settle_duration = 5.0
    spray_duration = 5.0
    return_duration = 8.0
    landing_duration = 4.0

    # Drone modeli
    drone = robot.getSelf()
    position = drone.getField("translation")
    rotation = drone.getField("rotation")

    home = position.getSFVec3f()

    # Panel üzerindeki animasyon konumu
    panel_position = [0.41, -0.02, 5.74]
    takeoff_position = [home[0], home[1], 5.74]

    # Su animasyonu
    water_transparency = find_water_transparency(robot)
    water_transparency.setSFFloat(1.0)

    # Kir objesi
    dirt = robot.getFromDef("DIRT_1")

    if dirt is None:
        raise RuntimeError("DIRT_1 bulunamadı.")

    dirt_position = dirt.getField("translation")
    initial_dirt_position = dirt_position.getSFVec3f()

    start_time = robot.getTime()
    cleaned = False
    previous_phase = None

    def interpolate(a, b, progress):
        progress = clamp(progress, 0.0, 1.0)
        return [
            a[i] + (b[i] - a[i]) * progress
            for i in range(3)
        ]

    while robot.step(time_step) != -1:
        elapsed = robot.getTime() - start_time

        if elapsed < takeoff_duration:
            phase = "KALKIS"
            p = interpolate(
                home,
                takeoff_position,
                elapsed / takeoff_duration
            )

        elif elapsed < takeoff_duration + transit_duration:
            phase = "PANELE_GECIS"

            progress = (
                elapsed - takeoff_duration
            ) / transit_duration

            p = interpolate(
                takeoff_position,
                panel_position,
                progress
            )

        else:
            p = panel_position

            spray_start = (
                takeoff_duration
                + transit_duration
                + settle_duration
            )

            spray_end = spray_start + spray_duration

            if elapsed < spray_start:
                phase = "PANELDE_BEKLEME"

            elif elapsed < spray_end:
                phase = "SULAMA"

            elif elapsed < spray_end + return_duration:
                phase = "ISTASYONA_DONUS"
            
                progress = (elapsed - spray_end) / return_duration
                p = interpolate(panel_position, takeoff_position, progress)
            
            elif elapsed < spray_end + return_duration + landing_duration:
                phase = "INIS"
            
                progress = (
                    elapsed - spray_end - return_duration
                ) / landing_duration
            
                p = interpolate(takeoff_position, home, progress)
            
            else:
                phase = "ISTASYONDA"
                p = home

        # Webots görsel konum animasyonu
        position.setSFVec3f(p)
        rotation.setSFRotation([0, 0, 1, 0])
        drone.resetPhysics()

        if phase == "SULAMA":
            water_transparency.setSFFloat(0.35)
        else:
            water_transparency.setSFFloat(1.0)

        if phase == "ISTASYONA_DONUS" and not cleaned:
            x, y, z = initial_dirt_position
            dirt_position.setSFVec3f([x, y, z - 5.0])
            cleaned = True

            print("DIRT_1 temizlendi.")

        if phase != previous_phase:
            print(f"t={elapsed:.2f}s | {phase}")
            previous_phase = phase


if __name__ == "__main__":
    run_robot(Supervisor())