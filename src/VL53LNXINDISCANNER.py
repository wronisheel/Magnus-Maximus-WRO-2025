import time
from smbus2 import SMBus
from VL53L1X import VL53L1X

I2C_BUS = 1
MUX_ADDR = 0x70
VL53L1X_ADDR = 0x29
CHANNEL = 0 # <- Channel you want to read from

bus = SMBus(I2C_BUS)

def select_mux_channel(channel):
    if 0 <= channel <= 7:
        bus.write_byte(MUX_ADDR, 1 << channel)
        time.sleep(0.1)
    else:
        raise ValueError("Channel must be between 0 and 7")

# --- Select channel 1 on PCA9548A ---
select_mux_channel(CHANNEL)

# --- Initialize VL53L1X Sensor ---
tof = VL53L1X(i2c_bus=I2C_BUS, i2c_address=VL53L1X_ADDR)
tof.open()
tof.start_ranging(1)  # Mode 1 = short, 2 = medium, 3 = long

print("🔍 Reading distance from VL53L1X on Channel 1...\n")

try:
    while True:
        distance = tof.get_distance()
        print(f"📏 Distance: {distance} mm")
        time.sleep(0.5)
except KeyboardInterrupt:
    print("\nStopping...")

tof.stop_ranging()
tof.close()
