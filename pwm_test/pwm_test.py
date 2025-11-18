PWM_CHIP = 0   # Usually pwmchip0, check /sys/class/pwm/
PWM_CHANNEL = 0

def write_pwm(path, value):
    with open(path, "w") as f:
        f.write(str(value))

# Export the channel
write_pwm(f"/sys/class/pwm/pwmchip{PWM_CHIP}/export", PWM_CHANNEL)

# Set period to 20ms (20000000 ns)
write_pwm(f"/sys/class/pwm/pwmchip{PWM_CHIP}/pwm{PWM_CHANNEL}/period", 20000000)

# Function to set duty cycle in microseconds
def set_pulse_us(pulse_us):
    # pulse_us should be between 1000 and 2000
    write_pwm(f"/sys/class/pwm/pwmchip{PWM_CHIP}/pwm{PWM_CHANNEL}/duty_cycle", pulse_us * 1000)

# Enable PWM
write_pwm(f"/sys/class/pwm/pwmchip{PWM_CHIP}/pwm{PWM_CHANNEL}/enable", 1)

# Example: sweep
import time
# for pulse in range(1000, 2001, 50):
#     set_pulse_us(pulse)
#     time.sleep(0.1)

import time
width = 2000 - 1000
for percent in range(0, 20):
    dec = percent / 100.0
    pulse_width = 1000 + int(width * dec)
    print(f"Setting pulse width to {pulse_width} us ({dec*100:.1f}%)")
    set_pulse_us(pulse_width)
    time.sleep(0.1)

# set to 0%
set_pulse_us(1000)

# Disable PWM
write_pwm(f"/sys/class/pwm/pwmchip{PWM_CHIP}/pwm{PWM_CHANNEL}/enable", 0)