#!/usr/bin/env python3
"""
SHT40 Sensor Reading Program for Raspberry Pi (보정값 적용된 터미널 버전)
GUI 버전에서 터미널 버전으로 변환, 보정값 및 개선된 로직 적용
"""

import time
import smbus2
import logging
import subprocess
from datetime import datetime
from collections import deque

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

class SimpleSHT40:
    """SHT40 온습도 센서 클래스 (개선된 I2C 방식 + 보정값 적용)"""
    
    # I2C addresses
    DEFAULT_I2C_ADDRESS = 0x44  # SHT40 표준 주소
    
    # Commands
    CMD_MEASURE_HIGH_PRECISION = 0xFD  # High precision measurement
    CMD_MEASURE_MEDIUM_PRECISION = 0xF6  # Medium precision measurement
    CMD_MEASURE_LOW_PRECISION = 0xE0  # Low precision measurement
    CMD_READ_SERIAL_NUMBER = 0x89  # Read serial number
    CMD_SOFT_RESET = 0x94  # Soft reset
    
    # 보정값 설정 (Calibration offsets)
    TEMPERATURE_OFFSET = 0.0  # 온도 보정값 (°C) - 현재 측정값이 0도 높게 나와서 0.0 적용
    HUMIDITY_OFFSET = 0.0      # 습도 보정값 (%RH) - 필요시 수정
    
    def __init__(self, bus=1, address=DEFAULT_I2C_ADDRESS):
        self.bus_num = bus
        self.address = address
        self.bus = None
        logger.info(f"SimpleSHT40 초기화 (버스: {bus}, 주소: 0x{address:02X})")
        logger.info(f"보정값 적용: 온도 {self.TEMPERATURE_OFFSET:+.1f}°C, 습도 {self.HUMIDITY_OFFSET:+.1f}%RH")
    
    def connect(self):
        """센서 연결 및 초기화"""
        try:
            self.bus = smbus2.SMBus(self.bus_num)
            
            # 연결 테스트 - 리셋 명령 전송
            write_msg = smbus2.i2c_msg.write(self.address, [self.CMD_SOFT_RESET])
            self.bus.i2c_rdwr(write_msg)
            time.sleep(0.1)  # 리셋 후 충분한 대기 시간
            
            logger.info(f"SHT40 센서 연결 및 리셋 완료 (버스: {self.bus_num}, 주소: 0x{self.address:02X})")
            return True
        except Exception as e:
            if self.bus:
                self.bus.close()
                self.bus = None
            logger.error(f"센서 연결 실패: {e}")
            raise e
    
    def reset(self):
        """센서 소프트 리셋"""
        if not self.bus:
            raise Exception("센서가 연결되지 않음")
            
        try:
            # I2C 메시지 방식으로 리셋 명령 전송
            write_msg = smbus2.i2c_msg.write(self.address, [self.CMD_SOFT_RESET])
            self.bus.i2c_rdwr(write_msg)
            time.sleep(0.1)  # 리셋 완료 대기
            logger.info("SHT40 센서 리셋 완료")
            return True
        except Exception as e:
            logger.error(f"센서 리셋 실패: {e}")
            return False
    
    def read_serial_number(self):
        """센서 시리얼 번호 읽기"""
        if not self.bus:
            raise Exception("센서가 연결되지 않음")
            
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
    
    def read_temperature_humidity(self, precision="high"):
        """온습도값 읽기 (개선된 방식)"""
        if not self.bus:
            raise Exception("센서가 연결되지 않음")
            
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
            
            # 2단계: 측정 완료까지 대기 (증가된 대기 시간)
            time.sleep(wait_time)
            
            # 3단계: 데이터 읽기 (6바이트: T_MSB, T_LSB, T_CRC, RH_MSB, RH_LSB, RH_CRC)
            read_msg = smbus2.i2c_msg.read(self.address, 6)
            self.bus.i2c_rdwr(read_msg)
            
            # 읽은 데이터 처리
            data = list(read_msg)
            
            # 디버깅을 위한 원시 데이터 로깅
            logger.debug(f"Raw data: {[hex(x) for x in data]}")
            
            # 온도 및 습도 데이터 분리
            t_data = [data[0], data[1]]
            t_crc = data[2]
            rh_data = [data[3], data[4]]
            rh_crc = data[5]
            
            # CRC 검증
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
            
            # 보정값 적용 (Calibration correction)
            temperature += self.TEMPERATURE_OFFSET
            humidity += self.HUMIDITY_OFFSET
            
            # 습도를 물리적 범위로 제한
            humidity = max(0, min(100, humidity))
            
            return round(temperature, 2), round(humidity, 2)
            
        except Exception as e:
            logger.error(f"온습도 측정 실패: {e}")
            raise Exception(f"온습도 측정 실패: {e}")
    
    def read_with_retry(self, precision="high", max_retries=3):
        """재시도 기능이 있는 측정"""
        for attempt in range(max_retries):
            try:
                result = self.read_temperature_humidity(precision)
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
        """연결 종료"""
        if self.bus:
            self.bus.close()
            self.bus = None
            logger.info("I2C 버스 연결 종료")

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
        
        sensor = SimpleSHT40(bus=bus_number, address=address)
        
        # 테스트 1: 센서 연결 및 리셋
        print("1. 센서 연결 및 리셋 테스트...")
        try:
            sensor.connect()
            print("   센서 연결 및 리셋 성공")
        except Exception as e:
            print(f"   연결 실패: {e}")
            return False
        
        # 테스트 2: 측정 테스트
        print("2. 온습도 측정 테스트...")
        try:
            result = sensor.read_with_retry(precision="high")
            if result:
                temperature, humidity = result
                print(f"   온도: {temperature:.2f} °C")
                print(f"   습도: {humidity:.2f} %RH")
                print("   통신 테스트 성공!")
                sensor.close()
                return True
            else:
                print("   측정 실패")
                sensor.close()
                return False
                
        except Exception as e:
            print(f"   측정 실패: {e}")
            sensor.close()
            return False
            
    except Exception as e:
        print(f"통신 테스트 실패: {e}")
        return False

def get_comfort_status(temperature, humidity):
    """온습도값에 따른 환경 상태 반환"""
    # 온도 기준 (°C)
    if temperature < 18:
        temp_status = "춥다"
    elif temperature < 20:
        temp_status = "서늘"
    elif temperature < 26:
        temp_status = "적정"
    elif temperature < 28:
        temp_status = "따뜻"
    else:
        temp_status = "덥다"
    
    # 습도 기준 (%RH)
    if humidity < 30:
        humidity_status = "건조"
    elif humidity < 40:
        humidity_status = "약간건조"
    elif humidity < 60:
        humidity_status = "적정"
    elif humidity < 70:
        humidity_status = "약간습함"
    else:
        humidity_status = "습함"
    
    # 종합 판정
    if temp_status == "적정" and humidity_status == "적정":
        return "쾌적"
    elif temp_status in ["서늘", "따뜻"] and humidity_status in ["약간건조", "약간습함"]:
        return "양호"
    elif temp_status in ["춥다", "덥다"] or humidity_status in ["건조", "습함"]:
        return "불쾌"
    else:
        return "보통"

def calculate_stats(temperature_data, humidity_data):
    """통계 계산"""
    if len(temperature_data) == 0:
        return None, None, None, None, None, None
    
    temp_values = list(temperature_data)
    humidity_values = list(humidity_data)
    
    temp_max = max(temp_values)
    temp_min = min(temp_values)
    temp_avg = sum(temp_values) / len(temp_values)
    
    humidity_max = max(humidity_values)
    humidity_min = min(humidity_values)
    humidity_avg = sum(humidity_values) / len(humidity_values)
    
    return temp_max, temp_min, temp_avg, humidity_max, humidity_min, humidity_avg

def main():
    """메인 함수"""
    print("=" * 60)
    print("SHT40 온습도 센서 읽기 프로그램 (보정값 적용된 터미널 버전)")
    print("=" * 60)
    print(f"적용된 보정값: 온도 {SimpleSHT40.TEMPERATURE_OFFSET:+.1f}°C, 습도 {SimpleSHT40.HUMIDITY_OFFSET:+.1f}%RH")
    print("=" * 60)
    
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
    if not test_basic_communication(found_bus, found_address):
        print("기본 통신 테스트 실패. 센서 연결을 확인하세요.")
        return
    
    try:
        # SHT40 센서 객체 생성 (발견된 버스와 주소 사용)
        sensor = SimpleSHT40(bus=found_bus, address=found_address)
        
        # 센서 연결
        sensor.connect()
        
        # 시리얼 번호 읽기
        serial = sensor.read_serial_number()
        if serial:
            print(f"\n센서 시리얼 번호: {serial[0]:04X}-{serial[1]:04X}")
        
        # 데이터 저장용 deque (최대 60개 데이터 포인트 - 통계용)
        max_points = 60
        temperature_data = deque(maxlen=max_points)
        humidity_data = deque(maxlen=max_points)
        
        # 연속 측정
        print("\n=== 연속 온습도 측정 시작 ===")
        print("종료하려면 Ctrl+C를 누르세요.")
        print("-" * 80)
        print(f"{'시간':^12} | {'온도(°C)':^8} | {'습도(%RH)':^9} | {'환경상태':^8} | {'통계정보':^20}")
        print("-" * 80)
        
        measurement_count = 0
        error_count = 0
        
        while True:
            result = sensor.read_with_retry(precision="high")
            
            if result:
                temperature, humidity = result
                measurement_count += 1
                
                # 데이터 저장 (통계용)
                temperature_data.append(temperature)
                humidity_data.append(humidity)
                
                # 환경 상태 계산
                comfort_status = get_comfort_status(temperature, humidity)
                
                # 통계 계산
                temp_max, temp_min, temp_avg, humidity_max, humidity_min, humidity_avg = calculate_stats(temperature_data, humidity_data)
                
                # 현재 시간
                current_time = datetime.now().strftime("%H:%M:%S")
                
                # 통계 정보 텍스트
                if temp_max is not None:
                    stats_info = f"T:{temp_avg:.1f}avg H:{humidity_avg:.1f}avg"
                else:
                    stats_info = "계산중..."
                
                # 측정 결과 출력 (한 줄로)
                print(f"{current_time:^12} | {temperature:^8.2f} | {humidity:^9.2f} | {comfort_status:^8} | {stats_info:^20}")
                
                # 10회마다 상세 통계 출력
                if measurement_count % 10 == 0 and temp_max is not None:
                    print("-" * 80)
                    print(f"[통계 정보 - 최근 {len(temperature_data)}회 측정]")
                    print(f"온도: 최대 {temp_max:.1f}°C, 최소 {temp_min:.1f}°C, 평균 {temp_avg:.1f}°C")
                    print(f"습도: 최대 {humidity_max:.1f}%RH, 최소 {humidity_min:.1f}%RH, 평균 {humidity_avg:.1f}%RH")
                    print("-" * 80)
                
                error_count = 0  # 성공하면 에러 카운트 리셋
            else:
                error_count += 1
                current_time = datetime.now().strftime("%H:%M:%S")
                print(f"{current_time:^12} | {'측정실패':^8} | {'측정실패':^9} | {'오류':^8} | 연속 {error_count}회 실패")
                
                # 연속 3회 실패 시 센서 리셋
                if error_count >= 3:
                    print(f"{datetime.now().strftime('%H:%M:%S')} | 연속 실패로 인한 센서 리셋...")
                    if not sensor.reset():
                        print("센서 리셋 실패. 프로그램 종료.")
                        break
                    time.sleep(0.1)  # 리셋 후 안정화 시간
                    error_count = 0
            
            time.sleep(30)  # 30초 간격으로 측정 (발열 문제 방지)
            
    except KeyboardInterrupt:
        print(f"\n{datetime.now().strftime('%H:%M:%S')} | 프로그램이 사용자에 의해 종료되었습니다.")
        
        # 최종 통계 출력
        if len(temperature_data) > 0:
            temp_max, temp_min, temp_avg, humidity_max, humidity_min, humidity_avg = calculate_stats(temperature_data, humidity_data)
            print("\n=== 최종 통계 ===")
            print(f"총 측정 횟수: {measurement_count}회")
            print(f"온도: 최대 {temp_max:.2f}°C, 최소 {temp_min:.2f}°C, 평균 {temp_avg:.2f}°C")
            print(f"습도: 최대 {humidity_max:.2f}%RH, 최소 {humidity_min:.2f}%RH, 평균 {humidity_avg:.2f}%RH")
        
    except Exception as e:
        logger.error(f"프로그램 실행 중 오류: {e}")
    finally:
        try:
            sensor.close()
        except:
            pass

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n프로그램이 사용자에 의해 중단되었습니다.")
    finally:
        print("프로그램 종료")