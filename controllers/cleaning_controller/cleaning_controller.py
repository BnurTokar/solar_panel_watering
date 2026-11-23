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
    dt = time_step / 1000.0

    # Görev ayarları
    hover_height = 2.0            # Kalkış konumuna göre hedef yükseklik (m)
    settle_duration = 5.0         # Sulama öncesi hedefte bekleme süresi (s)
    spray_duration = 5.0          # Suyun açık kalacağı süre (s)
    horizontal_tolerance = 0.03   # İzin verilen yatay hata (m)
    height_tolerance = 0.02       # İzin verilen yükseklik hatası (m)

    # Önceki uçuş testinde kullandığın kontrol ayarları
    hover_thrust = 69.15
    roll_gain = 50.0
    pitch_gain = 30.0
    cruise_speed = 0.12
    approach_gain = 0.5
    velocity_gain = 1.2
    integral_gain = 0.08

    # Su başlangıçta kapalı.
    water_transparency = find_water_transparency(robot)
    water_transparency.setSFFloat(1.0)
    print("SU HAZIR: bodySlot içindeki su konisi bulundu. Su kapalı.")

    # Sensörler ve kamera
    imu = robot.getDevice("inertial unit")
    gps = robot.getDevice("gps")
    gyro = robot.getDevice("gyro")
    camera = robot.getDevice("camera")
    compass = robot.getDevice("compass")

    for sensor in (imu, gps, gyro, camera, compass):
        sensor.enable(time_step)

    robot.getDevice("camera pitch").setPosition(0.7)

    # Pervaneler
    front_left = robot.getDevice("front left propeller")
    front_right = robot.getDevice("front right propeller")
    rear_left = robot.getDevice("rear left propeller")
    rear_right = robot.getDevice("rear right propeller")

    for motor in (front_left, front_right, rear_left, rear_right):
        motor.setPosition(float("inf"))
        motor.setVelocity(0.0)

    home = None
    phase = "KALKIS"
    stable_since = None
    spray_end_time = None
    integral_x = 0.0
    integral_y = 0.0
    next_report = 0.0

    while robot.step(time_step) != -1:
        now = robot.getTime()

        # Su zamanlayıcısı uçuş döngüsünü durdurmadan çalışır.
        if phase == "SULAMA" and now >= spray_end_time:
            water_transparency.setSFFloat(1.0)
            phase = "BEKLEME"
            print(f"t={now:.2f}s  SU KAPALI. Drone havada bekliyor.")

        x, y, z = gps.getValues()
        vx, vy, vz = gps.getSpeedVector()
        roll, pitch, yaw = imu.getRollPitchYaw()
        roll_rate, pitch_rate, _ = gyro.getValues()

        readings = (x, y, z, vx, vy, vz, roll, pitch, yaw, roll_rate, pitch_rate)
        if not all(isfinite(value) for value in readings):
            stable_since = None
            continue

        if home is None:
            home = (x, y, z)
            print(
                f"Kalkış noktası: X={x:.3f}, Y={y:.3f}, Z={z:.3f}. "
                f"Hedef: kalkış noktasının {hover_height:.1f} m üzeri."
            )

        # Üç aşamada da yatay hedef, başlangıçtaki GPS konumudur.
        dx = home[0] - x
        dy = home[1] - y
        horizontal_error = hypot(dx, dy)

        height = z - home[2]
        height_error = hover_height - height

        # Yatay kayma varsa kalkış merkezine doğru bir hız iste.
        desired_speed = min(cruise_speed, approach_gain * horizontal_error)

        if horizontal_error > 0.0:
            desired_vx = desired_speed * dx / horizontal_error
            desired_vy = desired_speed * dy / horizontal_error
        else:
            desired_vx = 0.0
            desired_vy = 0.0

        vx_error = desired_vx - vx
        vy_error = desired_vy - vy

        if abs(height_error) <= 0.05:
            integral_x = clamp(integral_x + vx_error * dt, -0.8, 0.8)
            integral_y = clamp(integral_y + vy_error * dt, -0.8, 0.8)

        east = velocity_gain * vx_error + integral_gain * integral_x
        north = velocity_gain * vy_error + integral_gain * integral_y

        # Dünya eksenlerindeki düzeltmeyi drone'un gövde eksenlerine çevir.
        forward = cos(yaw) * east + sin(yaw) * north
        left = -sin(yaw) * east + cos(yaw) * north

        target_pitch = clamp(forward / 9.81, -0.12, 0.12)
        target_roll = clamp(-left / 9.81, -0.12, 0.12)

        roll_input = roll_gain * clamp(roll - target_roll, -1.0, 1.0) + roll_rate
        pitch_input = pitch_gain * clamp(pitch - target_pitch, -1.0, 1.0) + pitch_rate
        vertical_input = clamp(10.0 * height_error - 6.0 * vz, -8.0, 8.0)
        thrust = hover_thrust + vertical_input

        front_left.setVelocity(thrust + pitch_input - roll_input)
        front_right.setVelocity(-(thrust + pitch_input + roll_input))
        rear_left.setVelocity(-(thrust - pitch_input - roll_input))
        rear_right.setVelocity(thrust - pitch_input + roll_input)

        # Hedef bölgeden çıkarsa beş saniyelik bekleme yeniden başlar.
        if phase == "KALKIS":
            inside_target = (
                horizontal_error <= horizontal_tolerance
                and abs(height_error) <= height_tolerance
            )

            if not inside_target:
                stable_since = None
            elif stable_since is None:
                stable_since = now
            elif now - stable_since >= settle_duration:
                water_transparency.setSFFloat(0.35)
                spray_end_time = now + spray_duration
                phase = "SULAMA"
                print(f"t={now:.2f}s  SU AÇIK. Yükseklik={height:.3f} m.")

        if now >= next_report:
            print(
                f"t={now:.1f}s  {phase}  "
                f"yükseklik={height:.3f} m  "
                f"yatay_hata={horizontal_error:.3f} m  "
                f"yatay_hız={hypot(vx, vy):.3f} m/s"
            )
            next_report = now + 1.0


if __name__ == "__main__":
    run_robot(Supervisor())