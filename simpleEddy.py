#!/usr/bin/env python3
"""
SDP810 차압 센서 연속 모니터링 프로그램
- 채널 0, 1 자동 스캔 및 센서 감지
- 저수준 I2C 통신 방식 (SDP 시리즈 표준)
- 실시간 차압 데이터 연속 출력
- 양수: P1(흡기), 음수: P2(배기) 자동 분류

설정 변경 방법:
- DIRECTION_MODE: "Normal" 또는 "Reverse"로 변경
- READ_INTERVAL: 읽기 간격(초) 조정
"""

import smbus2
import time
import struct
import subprocess

# ========== 사용자 설정 영역 ==========
DIRECTION_MODE = "Normal"    # "Normal" 또는 "Reverse"로 변경하세요
READ_INTERVAL = 1.0          # 읽기 간격(초)
# =====================================

class SDP810Scanner:
    """SDP810 센서 스캔 및 관리 클래스"""
    
    def __init__(self):
        self.sdp810_address = 0x25
        self.detected_sensors = []
    
    def test_sdp810_communication(self, bus_number):
        """SDP810과의 직접 통신 테스트"""
        try:
            bus = smbus2.SMBus(bus_number)
            
            try:
                read_msg = smbus2.i2c_msg.read(self.sdp810_address, 3)
                bus.i2c_rdwr(read_msg)
                data = list(read_msg)
                
                if len(data) == 3:
                    crc = self._calculate_crc8(data[:2])
                    is_valid = crc == data[2]
                    
                    raw_pressure = struct.unpack('>h', bytes(data[:2]))[0]
                    pressure = raw_pressure / 60.0
                    
                    bus.close()
                    return True, pressure, is_valid
                
                bus.close()
                return True, 0.0, False
                
            except Exception:
                bus.close()
                return False, 0.0, False
            
        except Exception:
            return False, 0.0, False
    
    def _calculate_crc8(self, data):
        """CRC-8 계산"""
        crc = 0xFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x80:
                    crc = (crc << 1) ^ 0x31
                else:
                    crc = crc << 1
                crc &= 0xFF
        return crc
    
    def scan_all_buses(self):
        """모든 I2C 버스에서 SDP810 검색"""
        print("=== SDP810 센서 스캔 시작 ===")
        self.detected_sensors = []
        
        for bus_num in [0, 1]:
            print(f"버스 {bus_num} 스캔 중...", end=" ")
            
            success, pressure, crc_ok = self.test_sdp810_communication(bus_num)
            
            if success:
                status = "✓" if crc_ok else "⚠"
                print(f"SDP810 발견! {pressure:.2f} Pa {status}")
                self.detected_sensors.append({
                    'bus': bus_num,
                    'address': self.sdp810_address,
                    'pressure': pressure,
                    'crc_ok': crc_ok
                })
            else:
                print("센서 없음")
        
        if self.detected_sensors:
            print(f"\n총 {len(self.detected_sensors)}개의 SDP810 센서가 발견되었습니다.")
        else:
            print("\nSDP810 센서를 찾을 수 없습니다.")
            print("디버그 정보:")
            
            for bus_num in [0, 1]:
                try:
                    result = subprocess.run(['i2cdetect', '-y', str(bus_num)], 
                                         capture_output=True, text=True, check=True)
                    print(f"버스 {bus_num} i2cdetect 결과:")
                    print(result.stdout)
                except Exception as e:
                    print(f"버스 {bus_num} i2cdetect 실패: {e}")
        
        return self.detected_sensors

class SDP810Monitor:
    """SDP810 차압 센서 모니터링 클래스"""
    
    def __init__(self, bus_number, slave_address=0x25, direction_mode="Normal"):
        self.bus_number = bus_number
        self.bus = smbus2.SMBus(bus_number)
        self.slave_address = slave_address
        self.packet_size = 3
        self.direction_mode = direction_mode
        
        print(f"SDP810 모니터 시작 - 버스: {bus_number}, 주소: 0x{slave_address:02X}, 모드: {direction_mode}")
    
    def calculate_crc8(self, data):
        """CRC-8 계산"""
        crc = 0xFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x80:
                    crc = (crc << 1) ^ 0x31
                else:
                    crc = crc << 1
                crc &= 0xFF
        return crc
    
    def read_pressure(self):
        """압력 데이터 읽기 (저수준 I2C)"""
        try:
            read_msg = smbus2.i2c_msg.read(self.slave_address, self.packet_size)
            self.bus.i2c_rdwr(read_msg)
            raw_data = list(read_msg)
            
            if len(raw_data) != self.packet_size:
                return None, None, f"데이터 길이 오류"
            
            pressure_msb = raw_data[0]
            pressure_lsb = raw_data[1]
            received_crc = raw_data[2]
            
            calculated_crc = self.calculate_crc8([pressure_msb, pressure_lsb])
            crc_ok = calculated_crc == received_crc
            
            raw_pressure = struct.unpack('>h', bytes([pressure_msb, pressure_lsb]))[0]
            pressure_pa = raw_pressure / 60.0
            
            # 방향 모드에 따른 부호 조정
            if self.direction_mode == "Reverse":
                pressure_pa = -pressure_pa
            
            return pressure_pa, crc_ok, "OK"
            
        except Exception as e:
            return None, None, f"읽기 오류: {e}"
    
    def get_port_label(self, pressure):
        """압력값에 따른 포트 레이블 반환"""
        if pressure is None:
            return "Error", ""
        
        if pressure >= 0:
            return "P1(흡기)", f"+{pressure:.1f}"
        else:
            return "P2(배기)", f"{pressure:.1f}"
    
    def start_monitoring(self, interval=1.0):
        """연속 모니터링 시작"""
        print(f"\n=== SDP810 차압 센서 연속 모니터링 시작 ===")
        print(f"버스: {self.bus_number} | 주소: 0x{self.slave_address:02X} | 모드: {self.direction_mode} | 간격: {interval}초")
        print("Ctrl+C로 중단")
        print("-" * 60)
        print("   시간     | 포트 및 압력값              | 상태")
        print("-" * 60)
        
        read_count = 0
        success_count = 0
        crc_fail_count = 0
        
        try:
            while True:
                pressure, crc_ok, msg = self.read_pressure()
                read_count += 1
                
                current_time = time.strftime("%H:%M:%S")
                
                if pressure is not None:
                    success_count += 1
                    if not crc_ok:
                        crc_fail_count += 1
                    
                    port_label, pressure_str = self.get_port_label(pressure)
                    print(f"{current_time} | {port_label}: {pressure_str} Pa | OK")
                    
                else:
                    print(f"{current_time} | 센서 읽기 실패: {msg} | ERROR")
                
                time.sleep(interval)
                
        except KeyboardInterrupt:
            print("\n" + "-" * 60)
            print("모니터링이 사용자에 의해 중단되었습니다.")
            
            # 통계 출력
            if read_count > 0:
                success_rate = (success_count / read_count) * 100
                crc_success_rate = ((success_count - crc_fail_count) / read_count) * 100 if read_count > 0 else 0
                
                print(f"\n=== 모니터링 통계 ===")
                print(f"총 읽기 횟수: {read_count}")
                print(f"성공 횟수: {success_count} ({success_rate:.1f}%)")
                print(f"CRC 성공: {success_count - crc_fail_count} ({crc_success_rate:.1f}%)")
                print(f"CRC 실패: {crc_fail_count}")
    
    def close(self):
        """I2C 버스 닫기"""
        self.bus.close()

def main():
    """메인 함수"""
    print("=== SDP810 차압 센서 모니터링 프로그램 ===")
    
    # 센서 스캔
    scanner = SDP810Scanner()
    detected_sensors = scanner.scan_all_buses()
    
    if not detected_sensors:
        print("\n센서를 찾을 수 없어 프로그램을 종료합니다.")
        print("확인사항:")
        print("- I2C가 활성화되어 있는지 확인: sudo raspi-config")
        print("- 센서 연결 상태 확인")
        print("- 전원 공급 상태 확인")
        return
    
    # 센서 선택 (자동 선택)
    if len(detected_sensors) == 1:
        selected_sensor = detected_sensors[0]
        print(f"센서 자동 선택: 버스 {selected_sensor['bus']}")
    else:
        print(f"\n{len(detected_sensors)}개의 센서가 발견되었습니다:")
        for i, sensor in enumerate(detected_sensors):
            status = "✓" if sensor['crc_ok'] else "⚠"
            print(f"  {i+1}. 버스 {sensor['bus']} - {sensor['pressure']:.2f} Pa {status}")
        
        # 첫 번째 센서 자동 선택
        selected_sensor = detected_sensors[0]
        print(f"첫 번째 센서 자동 선택: 버스 {selected_sensor['bus']}")
    
    # 모니터링 시작
    print(f"방향 모드: {DIRECTION_MODE}")
    print(f"읽기 간격: {READ_INTERVAL}초")
    
    try:
        monitor = SDP810Monitor(selected_sensor['bus'], selected_sensor['address'], DIRECTION_MODE)
        monitor.start_monitoring(READ_INTERVAL)
        
    except Exception as e:
        print(f"모니터링 중 오류 발생: {e}")
    
    finally:
        try:
            monitor.close()
        except:
            pass
        print("프로그램 종료")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n프로그램이 사용자에 의해 중단되었습니다.")
    except Exception as e:
        print(f"프로그램 오류: {e}")
    finally:
        print("프로그램 완료")