from machine import Pin, I2C, PWM, reset
from mpu6050 import MPU6050
from ssd1306 import SSD1306_I2C
from time import sleep, ticks_ms
from math import atan2, degrees

# I2C 設定
i2c_mpu = I2C(0, scl=Pin(9), sda=Pin(8))  # MPU-6050 用 (GPIO 9 / GPIO 8)
i2c_oled = I2C(1, scl=Pin(15), sda=Pin(14))  # OLED 用 (GPIO 15 / GPIO 14)

# MPU-6050 の初期化
mpu = MPU6050(i2c_mpu)

# OLED の初期化
oled_width = 128
oled_height = 64
oled = SSD1306_I2C(oled_width, oled_height, i2c_oled)

# ブザーの設定 (GPIO 5)
buzzer = PWM(Pin(5))
buzzer.deinit()  # 最初は音を鳴らさない

# スライドスイッチの設定 (Mute 切替, GPIO 28)
mute_switch = Pin(28, Pin.IN, Pin.PULL_UP)

# ボタンの設定 (GP27 と GP21)
button1 = Pin(27, Pin.IN, Pin.PULL_UP)  # ボタン 1
button2 = Pin(21, Pin.IN, Pin.PULL_UP)  # ボタン 2

# 角度の初期値
angle_x = 0
angle_y = 0
last_time = ticks_ms()

# コンプリメントフィルタの係数
alpha = 0.95

# 起動画面の表示
oled.fill(0)
oled.text("Car-Camp-Leveler", 0, 20)
oled.text("Loading...", 30, 40)
oled.show()
sleep(2)

print("Car-Camp-Leveler 起動中...")

# 初期キャリブレーション
print("キャリブレーション中... 3秒間動かさないでください")
sleep(3)
gyro_offset_x = 0
gyro_offset_y = 0
samples = 100
for _ in range(samples):
    gyro = mpu.get_gyro_data()
    gyro_offset_x += gyro['x']
    gyro_offset_y += gyro['y']
    sleep(0.01)
gyro_offset_x /= samples
gyro_offset_y /= samples
print("キャリブレーション完了")

while True:
    # 経過時間の計算 (秒)
    current_time = ticks_ms()
    dt = (current_time - last_time) / 1000.0
    last_time = current_time

    # 加速度データの取得
    accel = mpu.get_accel_data()

    # 加速度センサーによる角度計算
    accel_angle_x = degrees(atan2(accel['y'], accel['z']))
    accel_angle_y = degrees(atan2(-accel['x'], accel['z']))

    # ジャイロデータの取得
    gyro = mpu.get_gyro_data()

    # キャリブレーションの適用 (ドリフト補正)
    gyro['x'] -= gyro_offset_x
    gyro['y'] -= gyro_offset_y

    # ジャイロセンサーによる角度計算 (角速度 * 経過時間)
    angle_x += gyro['x'] * dt
    angle_y += gyro['y'] * dt

    # コンプリメントフィルタによる角度算出
    angle_x = alpha * (angle_x) + (1 - alpha) * accel_angle_x
    angle_y = alpha * (angle_y) + (1 - alpha) * accel_angle_y

    # マイナス角度を無くす (絶対値を使用)
    display_x = abs(int(angle_x))
    display_y = abs(int(angle_y))

    # OLED 表示クリア
    oled.fill(0)
    oled.text("Car-Camp-Leveler", 0, 0)
    oled.text("X-Angle:", 0, 20)
    oled.text(f"{display_x}deg", 80, 20)
    oled.text("Y-Angle:", 0, 35)
    oled.text(f"{display_y}deg", 80, 35)
    
    # Mute 状態の確認
    is_mute = mute_switch.value()  # HIGH: Mute OFF, LOW: Mute ON

    # Flat 状態の判定
    if display_x < 5 and display_y < 5:
        oled.text("Flat (OK)", 80, 50)
        if not is_mute:
            buzzer.freq(5000)
            buzzer.duty_u16(65535)
        else:
            buzzer.duty_u16(0)
    else:
        oled.text("Tilted", 80, 50)
        buzzer.duty_u16(0)

    # Mute 状態の表示
    if is_mute:
        oled.text("Mute: ON", 0, 50)
    else:
        oled.text("Mute: OFF", 0, 50)

    # 同時押しで再起動 (Mute 状態のみ)
    if is_mute and button1.value() == 0 and button2.value() == 0:
        oled.fill(0)
        oled.text("Restarting...", 20, 30)
        oled.show()
        sleep(1)
        reset()  # 再起動

    oled.show()

    # シリアルモニタにも出力
    print(f"X-Angle: {display_x}deg, Y-Angle: {display_y}deg")
    
    sleep(0.1)
