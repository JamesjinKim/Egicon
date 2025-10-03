import smbus2
import time

# TCA9548A I2C 멀티플렉서 주소
TCA9548A_ADDRS = [
    0x70, 0x71, 0x72, 0x73, 0x74, 0x75, 0x76, 0x77
]

# 함수: TCA9548A의 특정 채널을 활성화
def tca9548a_select_channel(bus, mux_index, channel):
    try:
        bus.write_byte(TCA9548A_ADDRS[mux_index], 1 << channel)
        time.sleep(0.1)  # 채널 전환 후 대기 시간
    except OSError:
        pass  # 에러 메시지 출력 생략

# BH1750 클래스 정의
class BH1750:
    def __init__(self, bus, address=0x23):
        self.bus = bus
        self.address = address

    def init_sensor(self):
        try:
            self.bus.write_byte_data(self.address, 0x01, 0x10)  # Continuous high-res mode 2
            time.sleep(0.1)  # 센서 초기화 대기 시간
        except OSError:
            pass  # 에러 메시지 출력 생략

    def read_light(self):
        try:
            data = self.bus.read_i2c_block_data(self.address, 0x10, 2)
            return ((data[1] + (256 * data[0])) / 1.2)
        except OSError:
            return None  # 에러 메시지 출력 생략

if __name__ == "__main__":
    bus = smbus2.SMBus(0)

    while True:
        print("----CHECK ALL CH---- ")
        for mux_index in range(8):
            for channel in range(9):
                tca9548a_select_channel(bus, mux_index, channel)
                sensor = BH1750(bus)
                sensor.init_sensor()

                light_level = sensor.read_light()

                if light_level is not None:
                    print(f"Sensor {mux_index * 8 + channel + 1}: {light_level} lux")
