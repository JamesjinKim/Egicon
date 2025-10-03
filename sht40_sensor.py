#!/usr/bin/env python3
"""
SHT40 Sensor Reading Program for Raspberry Pi (올바른 I2C 통신 방식)
이 프로그램은 SHT40 센서의 올바른 I2C 통신 프로토콜을 사용합니다.
"""

import time
import smbus2
import logging
import subprocess

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

class SHT40:
    """SHT40 온습도 센서와 통신하는 클래스 (순수 I2C 방식)"""
    
    # I2C addresses
    DEFAULT_I2C_ADDRESS = 0x44  # SHT40 표준 주소
    
    # Commands
    CMD_MEASURE_HIGH_PRECISION = 0xFD  # High precision measurement
    CMD_MEASURE_MEDIUM_PRECISION = 0xF6  # Medium precision measurement
    CMD_MEASURE_LOW_PRECISION = 0xE0  # Low precision measurement
    CMD_READ_SERIAL_NUMBER = 0x89  # Read serial number
    CMD_SOFT_RESET = 0x94  # Soft reset
    
    def __init__(self, i2c_bus=1, address=DEFAULT_I2C_ADDRESS):
        """SHT40 센서 초기화"""
        self.address = address
        self.bus = smbus2.SMBus(i2c_bus)
        logger.info(f"SHT40 초기화 완료 (버스: {i2c_bus}, 주소: 0x{address:02X})")
        
        # 초기화 시 센서 리셋
        self.reset()
        # 센서 안정화를 위한 대기 시간
        time.sleep(0.1)
    
    def reset(self):
        """센서 소프트 리셋"""
        try:
            # I2C 메시지 방식으로 리셋 명령 전송
            write_msg = smbus2.i2c_msg.write(self.address, [self.CMD_SOFT_RESET])
            self.bus.i2c_rdwr(write_msg)
            time.sleep(0.01)  # 리셋 완료 대기
            logger.info("SHT40 센서 리셋 완료")
            return True
        except Exception as e:
            logger.error(f"센서 리셋 실패: {e}")
            return False
    
    def read_serial_number(self):
        """센서 시리얼 번호 읽기"""
        try:
            # 시리얼 번호 읽기 명령 전송
            write_msg = smbus2.i2c_msg.write(self.address, [self.CMD_READ_SERIAL_NUMBER])
            read_msg = smbus2.i2c_msg.read(self.address, 6)
            
            self.bus.i2c_rdwr(write_msg)
            time.sleep(0.01)  # 명령 처리 대기
            self.bus.i2c_rdwr(read_msg)
            
            # 읽은 데이터 처리
            data = list(read_msg)
            
            # 시리얼 번호 추출 (CRC 무시)
            serial_1 = (data[0] << 8) | data[1]
            serial_2 = (data[3] << 8) | data[4]
            
            logger.info(f"센서 시리얼 번호: {serial_1:04X}-{serial_2:04X}")
            return (serial_1, serial_2)
            
        except Exception as e:
            logger.error(f"시리얼 번호 읽기 실패: {e}")
            return None
    
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
    
    def verify_crc(self, data, crc):
        """CRC 검증"""
        return self.calculate_crc(data) == crc
    
    def measure_temperature_humidity(self, precision="high"):
        """
        온도와 습도 측정 (올바른 I2C 통신 방식)
        
        Args:
            precision: 측정 정밀도 ("high", "medium", "low")
        
        Returns:
            tuple: (온도 °C, 상대습도 %RH) 또는 None
        """
        try:
            # 정밀도에 따른 명령 및 대기시간 설정
            if precision == "medium":
                cmd = self.CMD_MEASURE_MEDIUM_PRECISION
                wait_time = 0.01  # 10ms로 증가
            elif precision == "low":
                cmd = self.CMD_MEASURE_LOW_PRECISION
                wait_time = 0.005  # 5ms로 증가
            else:  # high precision (default)
                cmd = self.CMD_MEASURE_HIGH_PRECISION
                wait_time = 0.02  # 20ms로 증가
            
            # 1단계: 측정 명령 전송
            write_msg = smbus2.i2c_msg.write(self.address, [cmd])
            self.bus.i2c_rdwr(write_msg)
            
            # 2단계: 측정 완료까지 대기 (대기 시간 증가)
            time.sleep(wait_time)
            
            # 3단계: 데이터 읽기 (6바이트: T_MSB, T_LSB, T_CRC, RH_MSB, RH_LSB, RH_CRC)
            read_msg = smbus2.i2c_msg.read(self.address, 6)
            self.bus.i2c_rdwr(read_msg)
            
            # 읽은 데이터 처리
            data = list(read_msg)
            
            # 디버깅을 위해 원시 데이터 출력
            logger.debug(f"Raw data: {[hex(x) for x in data]}")
            
            # 온도 및 습도 데이터 분리
            t_data = [data[0], data[1]]
            t_crc = data[2]
            rh_data = [data[3], data[4]]
            rh_crc = data[5]
            
            # CRC 검증 (선택사항)
            t_crc_ok = self.verify_crc(t_data, t_crc)
            rh_crc_ok = self.verify_crc(rh_data, rh_crc)
            
            if not t_crc_ok:
                logger.warning("온도 데이터 CRC 검증 실패")
            if not rh_crc_ok:
                logger.warning("습도 데이터 CRC 검증 실패")
            
            # 원시 데이터를 실제 값으로 변환
            t_raw = (t_data[0] << 8) | t_data[1]
            rh_raw = (rh_data[0] << 8) | rh_data[1]
            
            # 데이터시트의 변환 공식 적용
            temperature = -45 + 175 * (t_raw / 65535.0)
            humidity = -6 + 125 * (rh_raw / 65535.0)
            
            # 습도를 물리적 범위로 제한
            humidity = max(0, min(100, humidity))
            
            return temperature, humidity
            
        except Exception as e:
            logger.error(f"측정 실패: {e}")
            return None
    
    def measure_with_retry(self, precision="high", max_retries=3):
        """재시도 기능이 있는 측정"""
        for attempt in range(max_retries):
            try:
                result = self.measure_temperature_humidity(precision)
                if result is not None:
                    return result
                time.sleep(0.1)
            except Exception as e:
                logger.warning(f"측정 시도 {attempt + 1} 실패: {e}")
                if attempt < max_retries - 1:
                    time.sleep(0.1)
        
        logger.error(f"{max_retries}번 시도 후 측정 실패")
        return None
    
    def close(self):
        """I2C 버스 연결 종료"""
        try:
            self.bus.close()
            logger.info("I2C 버스 연결 종료")
        except:
            pass

def scan_i2c_bus():
    """I2C 버스 0과 1 모두 스캔하고 SHT40 센서를 찾음 (0x44 주소만 검사)"""
    found_sensor = False
    found_bus = None
    found_address = 0x44  # SHT40의 기본 주소는 0x44로 고정
    
    # 버스 0과 1 모두 스캔
    for bus_number in [0, 1]:
        try:
            print(f"\n=== I2C 버스 {bus_number} 스캔 중... ===")
            result = subprocess.run(['i2cdetect', '-y', str(bus_number)], 
                                  capture_output=True, text=True, check=True)
            print(f"I2C 버스 {bus_number} 스캔 결과:")
            print(result.stdout)
            
            # SHT40 기본 주소(0x44)만 확인
            if "44" in result.stdout:
                logger.info(f"SHT40 센서가 주소 0x44에서 발견됨 (버스 {bus_number})")
                found_sensor = True
                found_bus = bus_number
                print(f"버스 {bus_number}에서 SHT40 센서 발견! (주소: 0x44)")
                # 발견된 센서 즉시 반환
                return found_sensor, found_bus, found_address
                
        except Exception as e:
            logger.warning(f"I2C 버스 {bus_number} 스캔 실패: {e}")
    
    if not found_sensor:
        logger.warning("어떤 I2C 버스에서도 SHT40 센서를 찾을 수 없음 (주소 0x44)")
        return False, None, None
    
    return found_sensor, found_bus, found_address

def test_basic_communication(bus_number, address):
    """기본 I2C 통신 테스트"""
    try:
        print(f"\n=== SHT40 센서 (버스 {bus_number}, 주소 0x{address:02X}) 기본 통신 테스트 ===")
        
        bus = smbus2.SMBus(bus_number)
        
        # 테스트 1: 센서 리셋으로 시작
        print("1. 센서 리셋 테스트...")
        try:
            write_msg = smbus2.i2c_msg.write(address, [0x94])  # Soft reset command
            bus.i2c_rdwr(write_msg)
            time.sleep(0.1)  # 리셋 후 충분한 대기 시간
            print("   센서 리셋 성공")
        except Exception as e:
            print(f"   리셋 실패: {e}")
            
        # 테스트 2: I2C 메시지 방식 테스트
        print("2. I2C 메시지 방식 테스트...")
        try:
            # 고정밀 측정 명령 전송
            write_msg = smbus2.i2c_msg.write(address, [0xFD])
            bus.i2c_rdwr(write_msg)
            print("   측정 명령 전송 성공")
            
            time.sleep(0.02)  # 측정 대기 시간 증가 (20ms)
            
            # 데이터 읽기
            read_msg = smbus2.i2c_msg.read(address, 6)
            bus.i2c_rdwr(read_msg)
            
            data = list(read_msg)
            print(f"   읽은 데이터: {[hex(x) for x in data]}")
            
            # 온도/습도 계산
            t_raw = (data[0] << 8) | data[1]
            rh_raw = (data[3] << 8) | data[4]
            
            temperature = -45 + 175 * (t_raw / 65535.0)
            humidity = -6 + 125 * (rh_raw / 65535.0)
            humidity = max(0, min(100, humidity))
            
            print(f"   온도: {temperature:.2f} °C")
            print(f"   습도: {humidity:.2f} %RH")
            
            # CRC 확인
            crc_t = calculate_crc([data[0], data[1]])
            crc_rh = calculate_crc([data[3], data[4]])
            print(f"   온도 CRC 계산: 0x{crc_t:02X}, 수신: 0x{data[2]:02X}, 일치: {crc_t == data[2]}")
            print(f"   습도 CRC 계산: 0x{crc_rh:02X}, 수신: 0x{data[5]:02X}, 일치: {crc_rh == data[5]}")
            
            if crc_t == data[2] and crc_rh == data[5]:
                print("   통신 테스트 성공!")
            else:
                print("   통신 테스트 부분 성공 (CRC 오류)")
            
        except Exception as e:
            print(f"   실패: {e}")
        
        bus.close()
        
    except Exception as e:
        print(f"통신 테스트 실패: {e}")

# 전역 CRC 계산 함수 추가 (클래스 외부에서도 사용 가능)
def calculate_crc(data):
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

def main():
    """메인 함수"""
    print("=" * 50)
    print("SHT40 온습도 센서 읽기 프로그램 (올바른 I2C 방식)")
    print("=" * 50)
    
    # I2C 버스 0과 1 스캔
    found_sensor, found_bus, found_address = scan_i2c_bus()
    
    if not found_sensor:
        print("\n문제 해결 방법:")
        print("1. I2C 활성화: sudo raspi-config > Interface Options > I2C")
        print("2. 하드웨어 연결 확인:")
        print("   - VCC → 3.3V (핀 1 또는 17)")
        print("   - GND → GND (핀 6, 9, 14, 20, 25, 30, 34, 39 중 하나)")
        print("   - SDA → GPIO2 (핀 3)")
        print("   - SCL → GPIO3 (핀 5)")
        return
    
    # 기본 통신 테스트 (발견된 버스와 주소로 테스트)
    test_basic_communication(found_bus, found_address)
    
    try:
        # SHT40 센서 객체 생성 (발견된 버스와 주소 사용)
        sensor = SHT40(i2c_bus=found_bus, address=found_address)
        
        # 센서 리셋
        if not sensor.reset():
            print("센서 리셋 실패")
            return
        
        # 안정화를 위한 대기
        time.sleep(0.1)
        
        # 시리얼 번호 읽기
        serial = sensor.read_serial_number()
        if serial:
            print(f"센서 시리얼 번호: {serial[0]:04X}-{serial[1]:04X}")
        
        # 연속 측정
        print("\n=== 연속 온습도 측정 시작 ===")
        print("종료하려면 Ctrl+C를 누르세요.\n")
        
        measurement_count = 0
        error_count = 0
        
        while True:
            result = sensor.measure_with_retry(precision="high")
            
            if result:
                temperature, humidity = result
                measurement_count += 1
                print(f"[{measurement_count:04d}] 온도: {temperature:.2f}°C, 습도: {humidity:.2f}%RH")
                error_count = 0  # 성공하면 에러 카운트 리셋
            else:
                error_count += 1
                print(f"측정 실패 (연속 {error_count}회)")
                
                # 연속 3회 실패 시 센서 리셋 (5회에서 3회로 변경)
                if error_count >= 3:
                    print("연속 실패로 인한 센서 리셋...")
                    if not sensor.reset():
                        print("센서 리셋 실패. 프로그램 종료.")
                        break
                    time.sleep(0.1)  # 리셋 후 안정화 시간
                    error_count = 0
            
            time.sleep(30)  # 2초 간격으로 측정/ 발열문제로 인해 30초로 변경
            
    except KeyboardInterrupt:
        print("\n프로그램이 사용자에 의해 종료되었습니다.")
    except Exception as e:
        logger.error(f"프로그램 실행 중 오류: {e}")
    finally:
        try:
            sensor.close()
        except:
            pass

if __name__ == "__main__":
    main()