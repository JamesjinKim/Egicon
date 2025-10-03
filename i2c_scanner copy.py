#!/usr/bin/env python3
"""
간소화된 I2C 센서 스캐너 (버스 0,1 전용)
SHT40, BME688, BH1750 센서 감지용
"""

import smbus2
import time
import subprocess

# --- 센서별 I2C 주소 정의 ---
SENSOR_ADDRESSES = {
    "SHT40": [0x44],  # SHT40 기본 I2C 주소
    "BME688": [0x76, 0x77], # BME688 기본 I2C 주소
    "BH1750": [0x23, 0x5C] # BH1750 기본 I2C 주소
}

class Sensor:
    """모든 I2C 센서 클래스의 기본 틀."""
    def __init__(self, bus, address, name="Unknown"):
        self.bus = bus
        self.address = address
        self.name = name
        self.is_connected = False

    def check_connection(self):
        """센서 연결 여부를 확인합니다 (ACK 응답)."""
        try:
            self.bus.read_byte(self.address)
            self.is_connected = True
            return True
        except OSError:
            self.is_connected = False
            return False

    def read_data(self):
        """센서 데이터를 읽는 추상 메서드. 각 센서 클래스에서 구현해야 합니다."""
        raise NotImplementedError("Subclasses must implement read_data method.")

class SHT40(Sensor):
    """SHT40 온습도 센서 클래스 (저수준 I2C 통신 방식 사용)"""
    
    # Commands
    CMD_MEASURE_HIGH_PRECISION = 0xFD
    CMD_SOFT_RESET = 0x94
    
    def __init__(self, bus, address=0x44):
        super().__init__(bus, address, "SHT40")
        self._initialized = False
    
    def calculate_crc(self, data):
        """CRC-8 체크섬 계산"""
        POLYNOMIAL = 0x31
        CRC = 0xFF
        for byte in data:
            CRC ^= byte
            for _ in range(8):
                if CRC & 0x80:
                    CRC = ((CRC << 1) ^ POLYNOMIAL) & 0xFF
                else:
                    CRC = (CRC << 1) & 0xFF
        return CRC
    
    def check_connection(self):
        """SHT40 연결 테스트 (소프트 리셋 명령 전송)"""
        try:
            # 소프트 리셋 명령으로 연결 확인
            write_msg = smbus2.i2c_msg.write(self.address, [self.CMD_SOFT_RESET])
            self.bus.i2c_rdwr(write_msg)
            time.sleep(0.01)
            
            # 측정 명령을 보내고 데이터를 읽어봄
            write_msg = smbus2.i2c_msg.write(self.address, [self.CMD_MEASURE_HIGH_PRECISION])
            self.bus.i2c_rdwr(write_msg)
            time.sleep(0.02)
            
            # 데이터 읽기 시도
            read_msg = smbus2.i2c_msg.read(self.address, 6)
            self.bus.i2c_rdwr(read_msg)
            
            self.is_connected = True
            self._initialized = True
            return True
        except Exception:
            self.is_connected = False
            self._initialized = False
            return False

    def read_data(self):
        """온도와 습도 측정 (저수준 I2C 통신 방식)"""
        if not self.is_connected:
            return None
            
        try:
            # 고정밀 측정 명령 전송
            write_msg = smbus2.i2c_msg.write(self.address, [self.CMD_MEASURE_HIGH_PRECISION])
            self.bus.i2c_rdwr(write_msg)
            time.sleep(0.02)
            
            # 데이터 읽기
            read_msg = smbus2.i2c_msg.read(self.address, 6)
            self.bus.i2c_rdwr(read_msg)
            
            # 읽은 데이터 처리
            data = list(read_msg)
            
            # 온도 및 습도 데이터 분리
            t_data = [data[0], data[1]]
            t_crc = data[2]
            rh_data = [data[3], data[4]]
            rh_crc = data[5]
            
            # CRC 검증
            t_crc_ok = self.calculate_crc(t_data) == t_crc
            rh_crc_ok = self.calculate_crc(rh_data) == rh_crc
            
            # 원시 데이터를 실제 값으로 변환
            t_raw = (t_data[0] << 8) | t_data[1]
            rh_raw = (rh_data[0] << 8) | rh_data[1]
            
            # 데이터시트의 변환 공식 적용
            temperature = -45 + 175 * (t_raw / 65535.0)
            humidity = -6 + 125 * (rh_raw / 65535.0)
            humidity = max(0, min(100, humidity))
            
            return {
                "temperature": f"{temperature:.2f} C", 
                "humidity": f"{humidity:.2f} %RH"
            }
            
        except Exception as e:
            return None

class BME688(Sensor):
    def __init__(self, bus, address=0x76):
        super().__init__(bus, address, "BME688")
        self._initialized = False

    def initialize(self):
        """BME688 초기화 (간단한 버전)"""
        try:
            chip_id = self.bus.read_byte_data(self.address, 0xD0)
            if chip_id != 0x61:
                self._initialized = False
                return
            self._initialized = True
        except OSError:
            self._initialized = False

    def read_data(self):
        if not self.is_connected:
            return None
        if not self._initialized:
            self.initialize()
            if not self._initialized:
                return None

        try:
            chip_id = self.bus.read_byte_data(self.address, 0xD0)
            return {"chip_id_check": f"0x{chip_id:02X}"}
        except OSError:
            self._initialized = False
            return None

class BH1750(Sensor):
    """BH1750 조도 센서 클래스 (저수준 I2C 통신 방식 사용)"""
    
    # BH1750 Commands
    POWER_DOWN = 0x00
    POWER_ON = 0x01
    RESET = 0x07
    CONTINUOUS_HIGH_RES_MODE = 0x10
    CONTINUOUS_HIGH_RES_MODE_2 = 0x11
    ONE_TIME_HIGH_RES_MODE = 0x20
    
    def __init__(self, bus, address=0x23):
        super().__init__(bus, address, "BH1750")
        self._initialized = False

    def check_connection(self):
        """BH1750 연결 테스트"""
        try:
            # 저수준 I2C 통신으로 전원 켜기 명령 전송
            write_msg = smbus2.i2c_msg.write(self.address, [self.POWER_ON])
            self.bus.i2c_rdwr(write_msg)
            time.sleep(0.05)  # 안정화 시간
            
            # 리셋 명령 전송
            write_msg = smbus2.i2c_msg.write(self.address, [self.RESET])
            self.bus.i2c_rdwr(write_msg)
            time.sleep(0.05)  # 리셋 후 대기
            
            self.is_connected = True
            return True
        except Exception:
            self.is_connected = False
            return False

    def initialize(self):
        """저수준 I2C 통신으로 BH1750 초기화"""
        try:
            # 전원 켜기
            write_msg = smbus2.i2c_msg.write(self.address, [self.POWER_ON])
            self.bus.i2c_rdwr(write_msg)
            time.sleep(0.05)
            
            # 측정 모드 설정 (여러 모드 중 하나 시도)
            write_msg = smbus2.i2c_msg.write(self.address, [self.CONTINUOUS_HIGH_RES_MODE])
            self.bus.i2c_rdwr(write_msg)
            time.sleep(0.2)  # 측정 준비 시간 증가
            
            self._initialized = True
            return True
        except Exception:
            self._initialized = False
            return False

    def read_data(self):
        """저수준 I2C 통신으로 조도값 읽기"""
        if not self.is_connected:
            return None
            
        if not self._initialized:
            if not self.initialize():
                return None
            
        try:
            # 저수준 I2C 통신으로 데이터 읽기 (2바이트)
            read_msg = smbus2.i2c_msg.read(self.address, 2)
            self.bus.i2c_rdwr(read_msg)
            
            # 읽은 데이터 처리
            data = list(read_msg)
            
            if len(data) >= 2:
                # 조도값 계산 (데이터시트 참조)
                lux = ((data[0] << 8) | data[1]) / 1.2
                return {"light_level": f"{lux:.2f} lux"}
            else:
                return None
                
        except Exception as e:
            print(f"  BH1750 데이터 읽기 오류: {e}")
            return None

def scan_bus(bus_id):
    """지정된 I2C 버스에서 센서 스캔"""
    print(f"\n------ I2C 버스 {bus_id} 스캔 ------")
    
    try:
        # i2cdetect 결과 확인 (간소화된 출력)
        result = subprocess.run(['i2cdetect', '-y', str(bus_id)], 
                             capture_output=True, text=True, check=True)
        print(f"버스 {bus_id} 주소 맵: ", end="")
        
        # 감지된 주소만 출력
        addresses = []
        for line in result.stdout.strip().split('\n')[1:]:  # 첫 줄(헤더) 건너뛰기
            parts = line.split(':')
            if len(parts) > 1:
                row = parts[0].strip()
                for i, val in enumerate(parts[1].split()):
                    if val != "--":
                        col = format(i, 'x')
                        addresses.append(f"0x{row}{col}")
        
        if addresses:
            print(", ".join(addresses))
        else:
            print("감지된 장치 없음")
        
    except Exception as e:
        print(f"버스 {bus_id} 스캔 오류: {e}")
        return []
    
    # 버스 초기화
    try:
        bus = smbus2.SMBus(bus_id)
    except Exception as e:
        print(f"버스 {bus_id} 초기화 실패: {e}")
        return []
    
    found_sensors = []
    
    # 센서 스캔
    for sensor_type, addresses in SENSOR_ADDRESSES.items():
        for addr in addresses:
            sensor = None
            if sensor_type == "SHT40":
                sensor = SHT40(bus, addr)
            elif sensor_type == "BME688":
                sensor = BME688(bus, addr)
            elif sensor_type == "BH1750":
                sensor = BH1750(bus, addr)

            if sensor and sensor.check_connection():
                print(f"발견: {sensor.name} (0x{addr:02X})", end=" - ")
                
                data = sensor.read_data()
                if data:
                    # 간소화된 데이터 출력
                    data_str = ", ".join(f"{k}: {v}" for k, v in data.items())
                    print(data_str)
                    found_sensors.append(f"버스 {bus_id}: {sensor.name} (0x{addr:02X}) - {data_str}")
                else:
                    print("데이터 읽기 실패")
                    found_sensors.append(f"버스 {bus_id}: {sensor.name} (0x{addr:02X}) - 데이터 읽기 실패")
    
    bus.close()
    return found_sensors

def main():
    print("== I2C 버스 0, 1 센서 스캔 시작 ==")
    all_sensors = []
    
    # 버스 0 스캔
    sensors = scan_bus(0)
    all_sensors.extend(sensors)
    
    # 버스 1 스캔
    sensors = scan_bus(1)
    all_sensors.extend(sensors)
    
    # 결과 요약
    print("\n== 스캔 결과 요약 ==")
    if all_sensors:
        for sensor in all_sensors:
            print(f"- {sensor}")
    else:
        print("감지된 센서가 없습니다.")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n프로그램이 사용자에 의해 중단되었습니다.")
    finally:
        print("프로그램 종료")