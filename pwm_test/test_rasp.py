from gpiozero import PWMOutputDevice
import time

PIN = 32

# Replace 18 with your PWM-capable pin number
servo = PWMOutputDevice(18, True, 0, 50)  # frequency=50Hz

def set_servo_us(pulse_us):
    # Convert 1000-2000 us to 0-1 duty cycle
    duty = pulse_us / 20000  # period = 20ms
    servo.value = duty

# Sweep example
# for pulse in range(1000, 2001, 50):
#     set_servo_us(pulse)
#     time.sleep(0.1)

for percent in range(0, 20):
    dec = percent / 100.0
    pulse_width = 1000 + int((2000 - 1000) * dec)
    print(f"Setting pulse width to {pulse_width} us ({dec*100:.1f}%)")
    set_servo_us(pulse_width)
    time.sleep(0.1)
