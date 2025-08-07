import time
import board
import busio
import smbus2
import adafruit_vl53l1x

# === Config ===
TCA_ADDRESS = 0x70
VL53_ADDRESS = 0x29

SENSORS = {
    0: "center",
    1: "right",
    2: "left",
    3: "back",
}

# === Select channel on TCA9548A ===
def select_tca_channel(channel):
    with smbus2.SMBus(1) as tca:
        tca.write_byte(TCA_ADDRESS, 1 << channel)
    time.sleep(0.1)

# === Initialize a VL53L1X sensor on a given channel ===
def init_sensor(channel):
    print(f"\n>>> Selecting TCA Channel {channel} ({SENSORS[channel]})")
    select_tca_channel(channel)

    i2c = board.I2C()
    sensor = adafruit_vl53l1x.VL53L1X(i2c)
    sensor.distance_mode = 2  # Long
    sensor.timing_budget = 100
    sensor.start_ranging()
    return sensor

# === Main Program ===
print("Initializing all sensors via TCA9548A...")
sensor_objects = {}

for ch in SENSORS:
    try:
        sensor_objects[ch] = init_sensor(ch)
        print(f"✅ Sensor '{SENSORS[ch]}' initialized.")
    except Exception as e:
        print(f"❌ Failed to initialize sensor on channel {ch}: {e}")

print("\n=== Reading distances ===")
try:
    while True:
        for ch in SENSORS:
            select_tca_channel(ch)
            sensor = sensor_objects.get(ch)
            if sensor and sensor.data_ready:
                print(f"{SENSORS[ch].capitalize():>6}: {sensor.distance} cm")
                sensor.clear_interrupt()
            time.sleep(0.1)
        print("-" * 30)
        time.sleep(0.5)
except KeyboardInterrupt:
    for sensor in sensor_objects.values():
        sensor.stop_ranging()
    print("Stopped all sensors.")
