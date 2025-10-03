#simpleEddy.py
"""
개선된 Raspberry Pi I2C Master Test Code for SDP810 Slave Device
- 저수준 I2C 통신 방식 추가
- 두 가지 통신 방법 비교 테스트
- 향상된 에러 처리 및 통계
"""

import smbus2
import time
import struct
import statistics

class SDP810_I2C_Test:
    def __init__(self, bus_number=1, slave_address=0x25):
        """
        Initialize I2C communication
        
        Args:
            bus_number: I2C bus number (default: 1 for Raspberry Pi)
            slave_address: 7-bit slave address (default: 0x25)
        """
        self.bus = smbus2.SMBus(bus_number)
        self.slave_address = slave_address
        self.packet_size = 3  # Pressure(2) + CRC(1)
        
        print(f"I2C Master initialized - Bus: {bus_number}, Slave: 0x{slave_address:02X}")
    
    def calculate_crc8(self, data):
        """
        Calculate CRC-8 (SDP810 compatible)
        Polynomial: 0x31 (x^8 + x^5 + x^4 + 1)
        """
        crc = 0xFF  # Initial value
        
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x80:
                    crc = (crc << 1) ^ 0x31  # Polynomial 0x31
                else:
                    crc = crc << 1
                crc &= 0xFF  # Keep 8-bit
        
        return crc
    
    def read_pressure_block_data(self):
        """
        기존 방식: read_i2c_block_data 사용
        """
        try:
            raw_data = self.bus.read_i2c_block_data(self.slave_address, 0, self.packet_size)
            return self._process_data(raw_data, "block_data")
        except Exception as e:
            return False, 0.0, [], f"Block data error: {e}"
    
    def read_pressure_low_level(self):
        """
        권장 방식: 저수준 I2C 통신 사용
        """
        try:
            read_msg = smbus2.i2c_msg.read(self.slave_address, self.packet_size)
            self.bus.i2c_rdwr(read_msg)
            raw_data = list(read_msg)
            return self._process_data(raw_data, "low_level")
        except Exception as e:
            return False, 0.0, [], f"Low level error: {e}"
    
    def _process_data(self, raw_data, method):
        """
        공통 데이터 처리 로직
        """
        if len(raw_data) != self.packet_size:
            return False, 0.0, raw_data, f"Wrong data length: {len(raw_data)}"
        
        # Extract data
        pressure_msb = raw_data[0]
        pressure_lsb = raw_data[1]
        received_crc = raw_data[2]
        
        # Verify CRC
        calculated_crc = self.calculate_crc8([pressure_msb, pressure_lsb])
        crc_ok = calculated_crc == received_crc
        
        # Convert to pressure value
        raw_pressure = struct.unpack('>h', bytes([pressure_msb, pressure_lsb]))[0]
        pressure_pa = raw_pressure / 60.0
        
        if not crc_ok:
            return False, pressure_pa, raw_data, f"CRC error (calc: 0x{calculated_crc:02X}, recv: 0x{received_crc:02X})"
        
        return True, pressure_pa, raw_data, f"OK ({method})"
    
    def compare_methods(self, count=20):
        """
        두 가지 통신 방법 비교 테스트
        """
        print(f"\n=== 통신 방법 비교 테스트 ({count}회) ===")
        
        block_data_results = []
        low_level_results = []
        
        for i in range(count):
            print(f"[{i+1:2d}/{count}] ", end="")
            
            # 방법 1: block_data
            success1, pressure1, raw1, msg1 = self.read_pressure_block_data()
            
            time.sleep(0.01)  # 짧은 간격
            
            # 방법 2: low_level
            success2, pressure2, raw2, msg2 = self.read_pressure_low_level()
            
            if success1:
                block_data_results.append(pressure1)
            if success2:
                low_level_results.append(pressure2)
            
            # 결과 출력
            status1 = "✓" if success1 else "✗"
            status2 = "✓" if success2 else "✗"
            
            print(f"Block: {status1} {pressure1:7.2f}Pa | Low: {status2} {pressure2:7.2f}Pa")
            
            if not success1 or not success2:
                print(f"      Block: {msg1}")
                print(f"      Low  : {msg2}")
            
            time.sleep(0.1)
        
        # 통계 분석
        print(f"\n--- 통계 분석 ---")
        print(f"Block Data Method:")
        if block_data_results:
            print(f"  성공률: {len(block_data_results)}/{count} ({len(block_data_results)/count*100:.1f}%)")
            print(f"  평균: {statistics.mean(block_data_results):.2f} Pa")
            print(f"  표준편차: {statistics.stdev(block_data_results):.2f} Pa" if len(block_data_results) > 1 else "  표준편차: N/A")
        else:
            print(f"  성공률: 0/{count} (0.0%)")
        
        print(f"Low Level Method:")
        if low_level_results:
            print(f"  성공률: {len(low_level_results)}/{count} ({len(low_level_results)/count*100:.1f}%)")
            print(f"  평균: {statistics.mean(low_level_results):.2f} Pa")
            print(f"  표준편차: {statistics.stdev(low_level_results):.2f} Pa" if len(low_level_results) > 1 else "  표준편차: N/A")
        else:
            print(f"  성공률: 0/{count} (0.0%)")
    
    def test_timing_analysis(self, count=50):
        """
        타이밍 분석 테스트
        """
        print(f"\n=== 타이밍 분석 테스트 ({count}회) ===")
        
        intervals = [0.01, 0.05, 0.1, 0.2, 0.5]  # 다양한 읽기 간격
        
        for interval in intervals:
            print(f"\n--- 읽기 간격: {interval}초 ---")
            
            success_count = 0
            pressures = []
            duplicate_count = 0
            last_raw_data = None
            
            for i in range(count):
                success, pressure, raw_data, msg = self.read_pressure_low_level()
                
                if success:
                    success_count += 1
                    pressures.append(pressure)
                    
                    # 중복 데이터 검사
                    if last_raw_data and raw_data == last_raw_data:
                        duplicate_count += 1
                    
                    last_raw_data = raw_data[:]
                
                if (i + 1) % 10 == 0:
                    print(f"  진행: {i+1}/{count}")
                
                time.sleep(interval)
            
            # 결과 출력
            print(f"  성공률: {success_count}/{count} ({success_count/count*100:.1f}%)")
            print(f"  중복 데이터: {duplicate_count}/{success_count} ({duplicate_count/success_count*100:.1f}%)" if success_count > 0 else "  중복 데이터: N/A")
            
            if len(pressures) > 1:
                print(f"  압력 범위: {min(pressures):.2f} ~ {max(pressures):.2f} Pa")
                print(f"  표준편차: {statistics.stdev(pressures):.2f} Pa")
    
    def test_error_recovery(self):
        """
        에러 복구 테스트
        """
        print(f"\n=== 에러 복구 테스트 ===")
        
        # 잘못된 주소로 읽기 시도
        original_address = self.slave_address
        
        print("1. 잘못된 주소로 읽기 시도...")
        self.slave_address = 0x99  # 존재하지 않는 주소
        
        success, pressure, raw_data, msg = self.read_pressure_low_level()
        print(f"   결과: {'성공' if success else '실패'} - {msg}")
        
        # 원래 주소로 복구
        print("2. 원래 주소로 복구...")
        self.slave_address = original_address
        
        success, pressure, raw_data, msg = self.read_pressure_low_level()
        print(f"   결과: {'성공' if success else '실패'} - {msg}")
        
        if success:
            print("   ✓ 복구 성공!")
        else:
            print("   ✗ 복구 실패")
    
    def close(self):
        """
        Close I2C bus
        """
        self.bus.close()
        print("I2C bus closed")

def main():
    """
    Main test function
    """
    print("=== 개선된 SDP810 I2C 테스트 ===")
    
    # Initialize I2C
    try:
        i2c_test = SDP810_I2C_Test(bus_number=1, slave_address=0x25)
    except Exception as e:
        print(f"Failed to initialize I2C: {e}")
        print("Make sure I2C is enabled: sudo raspi-config -> Interface Options -> I2C")
        return
    
    try:
        while True:
            print("\n" + "="*60)
            print("테스트 메뉴:")
            print("1. 기본 단일 읽기 테스트")
            print("2. 통신 방법 비교 테스트")
            print("3. 타이밍 분석 테스트")
            print("4. 에러 복구 테스트")
            print("5. 종합 테스트 (모든 테스트 실행)")
            print("6. 종료")
            
            choice = input("\n선택 (1-6): ").strip()
            
            if choice == '1':
                success, pressure, raw_data, msg = i2c_test.read_pressure_low_level()
                print(f"\n단일 읽기 결과:")
                print(f"  성공: {'예' if success else '아니오'}")
                print(f"  압력: {pressure:.2f} Pa")
                print(f"  원시 데이터: {[f'0x{b:02X}' for b in raw_data]}")
                print(f"  메시지: {msg}")
            
            elif choice == '2':
                i2c_test.compare_methods(count=20)
            
            elif choice == '3':
                i2c_test.test_timing_analysis(count=30)
            
            elif choice == '4':
                i2c_test.test_error_recovery()
            
            elif choice == '5':
                print("\n=== 종합 테스트 시작 ===")
                i2c_test.compare_methods(count=10)
                i2c_test.test_timing_analysis(count=20)
                i2c_test.test_error_recovery()
                print("\n=== 종합 테스트 완료 ===")
            
            elif choice == '6':
                break
            
            else:
                print("잘못된 선택입니다!")
    
    except KeyboardInterrupt:
        print("\n프로그램이 사용자에 의해 중단되었습니다.")
    
    finally:
        i2c_test.close()
        print("테스트 완료")

if __name__ == "__main__":
    main()