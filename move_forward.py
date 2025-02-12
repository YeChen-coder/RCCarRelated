import pigpio
import time

# 确保 pigpiod 守护程序正在运行
def ensure_pigpiod_running():
    import subprocess
    try:
        result = subprocess.run(['pgrep', 'pigpiod'], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        if result.returncode != 0:
            print("pigpiod is not running. Starting it now...")
            subprocess.run(['sudo', 'pigpiod'], check=True)
        else:
            print("pigpiod is already running.")
    except Exception as e:
        print(f"Error ensuring pigpiod is running: {e}")

ensure_pigpiod_running()

time.sleep(1)
pi = pigpio.pi()
if not pi.connected:
    raise RuntimeError("Failed to connect to pigpio daemon.")

# PWM 引脚和频率设置
pin2 = 13  # GPIO 引脚号
Freq = 50  # 频率，单位 Hz

# 设置 PWM 范围和频率
pi.set_PWM_range(pin2, 1000)
pi.set_PWM_frequency(pin2, Freq)

# 油门变量
ThrottleMsCentered = 1.35  # 中立位置
ThrottleMsRange = 0.25  # 油门范围

# 小车前进3秒
def move_forward_3_seconds():
    ThrottleMsOut = ThrottleMsCentered + ThrottleMsRange * 1  # 设置油门为前进方向的最大值
    Duty = ThrottleMsOut / (1 / pi.get_PWM_frequency(pin2) * 1000)
    pi.set_PWM_dutycycle(pin2, Duty * 1000)  # 更新油门的 PWM 信号
    time.sleep(3)  # 等待3秒
    pi.set_PWM_dutycycle(pin2, 0)  # 停止小车

move_forward_3_seconds()

# 关闭 PWM
pi.set_PWM_dutycycle(pin2, 0)
pi.stop()