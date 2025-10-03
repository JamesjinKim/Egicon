#!/usr/bin/env python3
"""
심플한 TCA9548A I2C 멀티플렉서 코드
버스 0(BH1750, 0x23) 및 버스 1(SHT40, 0x44) 탐지
"""
import smbus2
import time

class SimpleTCA9548A:
    """TCA9548A I2C 멀티플렉서 심플 클래스"""
    
    def __init__(self):
        """초기화: 버스 0과 1에서 TCA9548A 주소 탐지"""
        self.buses = {}  # {bus_number: smbus_object}
        self.address = None  # TCA9548A 주소
        self.tca_bus = None  # TCA9548A가 있는 버스
        self.connect_buses()
        self.detect_tca9548a_address()
    
    def connect_buses(self):
        """I2C 버스 0과 1에 연결"""
        self.close()
        for bus_num in [0, 1]:
            try:
                bus = smbus2.SMBus(bus_num)
                self.buses[bus_num] = bus
                print(f"버스 {bus_num} 연결 성공")
            except Exception as e:
                print(f"버스 {bus_num} 연결 실패: {e}")
    
    def detect_tca9548a_address(self):
        """TCA9548A 주소 (0x70-0x77) 탐지"""
        for bus_num in self.buses:
            bus = self.buses[bus_num]
            for addr in range(0x70, 0x78):
                try:
                    bus.write_byte(addr, 0x00)  # 모든 채널 비활성화
                    self.address = addr
                    self.tca_bus = bus_num
                    print(f"TCA9548A 발견: 버스 {bus_num}, 주소 0x{addr:02X}")
                    return
                except:
                    try:
                        bus.read_byte(addr)
                        self.address = addr
                        self.tca_bus = bus_num
                        print(f"TCA9548A 발견: 버스 {bus_num}, 주소 0x{addr:02X}")
                        return
                    except:
                        continue
        print("TCA9548A를 찾지 못했습니다.")
    
    def select_channel(self, bus_num, channel):
        """특정 버스와 채널 선택 (0-7)"""
        if bus_num not in self.buses or bus_num != self.tca_bus or not self.address:
            print(f"TCA9548A가 버스 {bus_num}에 없거나 설정되지 않음")
            return False
        
        if 0 <= channel <= 7:
            try:
                self.buses[bus_num].write_byte(self.address, 1 << channel)
                time.sleep(0.05)  # 채널 전환 안정화
                print(f"버스 {bus_num}, 채널 {channel} 선택")
                return True
            except Exception as e:
                print(f"버스 {bus_num}, 채널 {channel} 선택 실패: {e}")
                return False
        print(f"유효하지 않은 채널: {channel}")
        return False
    
    def disable_all(self, bus_num):
        """특정 버스의 모든 채널 비활성화"""
        if bus_num not in self.buses or bus_num != self.tca_bus or not self.address:
            print(f"TCA9548A가 버스 {bus_num}에 없거나 설정되지 않음")
            return False
        try:
            self.buses[bus_num].write_byte(self.address, 0x00)
            print(f"버스 {bus_num} 모든 채널 비활성화")
            return True
        except Exception as e:
            print(f"버스 {bus_num} 채널 비활성화 실패: {e}")
            return False
    
    def scan_channel(self, bus_num, channel):
        """특정 버스와 채널에서 디바이스 스캔 (BH1750, SHT40)"""
        devices = []
        if not self.select_channel(bus_num, channel):
            return devices
        
        bus = self.buses[bus_num]
        print(f"버스 {bus_num}, 채널 {channel} 스캔...")
        
        for addr in [0x23, 0x44]:  # BH1750(0x23), SHT40(0x44)
            # BH1750: read_byte
            if addr == 0x23:
                try:
                    bus.read_byte(addr)
                    devices.append(addr)
                    print(f"  BH1750 발견: 0x{addr:02X}")
                except Exception as e:
                    print(f"  BH1750 탐지 실패 at 0x{addr:02X}: {e}")
            
            # SHT40: 다중 테스트
            if addr == 0x44:
                try:
                    bus.read_byte(addr)
                    devices.append(addr)
                    print(f"  SHT40 발견 (read_byte): 0x{addr:02X}")
                except:
                    try:
                        bus.write_byte(addr, 0xFD)  # 고정밀 측정
                        time.sleep(0.02)  # 타이밍 증가
                        data = bus.read_i2c_block_data(addr, 0xFD, 6)
                        devices.append(addr)
                        print(f"  SHT40 발견 (측정): 0x{addr:02X}, 데이터: {[f'0x{b:02X}' for b in data]}")
                    except:
                        try:
                            bus.write_byte(addr, 0x89)  # 시리얼 번호
                            time.sleep(0.01)
                            data = bus.read_i2c_block_data(addr, 0x89, 6)
                            devices.append(addr)
                            print(f"  SHT40 발견 (시리얼): 0x{addr:02X}, 데이터: {[f'0x{b:02X}' for b in data]}")
                        except Exception as e:
                            print(f"  SHT40 탐지 실패 at 0x{addr:02X}: {e}")
        
        self.disable_all(bus_num)
        print(f"버스 {bus_num}, 채널 {channel} 스캔 완료: {[f'0x{addr:02X}' for addr in devices]}")
        return devices
    
    def scan_direct(self, bus_num):
        """특정 버스에서 직접 디바이스 스캔 (BH1750, SHT40)"""
        devices = []
        if bus_num not in self.buses:
            print(f"버스 {bus_num}가 연결되지 않음")
            return devices
        
        bus = self.buses[bus_num]
        print(f"버스 {bus_num} 직접 스캔...")
        
        for addr in [0x23, 0x44]:
            # BH1750
            if addr == 0x23:
                try:
                    bus.read_byte(addr)
                    devices.append(addr)
                    print(f"  BH1750 발견: 0x{addr:02X}")
                except Exception as e:
                    print(f"  BH1750 탐지 실패 at 0x{addr:02X}: {e}")
            
            # SHT40
            if addr == 0x44:
                try:
                    bus.read_byte(addr)
                    devices.append(addr)
                    print(f"  SHT40 발견 (read_byte): 0x{addr:02X}")
                except:
                    try:
                        bus.write_byte(addr, 0xFD)
                        time.sleep(0.02)
                        data = bus.read_i2c_block_data(addr, 0xFD, 6)
                        devices.append(addr)
                        print(f"  SHT40 발견 (측정): 0x{addr:02X}, 데이터: {[f'0x{b:02X}' for b in data]}")
                    except:
                        try:
                            bus.write_byte(addr, 0x89)
                            time.sleep(0.01)
                            data = bus.read_i2c_block_data(addr, 0x89, 6)
                            devices.append(addr)
                            print(f"  SHT40 발견 (시리얼): 0x{addr:02X}, 데이터: {[f'0x{b:02X}' for b in data]}")
                        except Exception as e:
                            print(f"  SHT40 탐지 실패 at 0x{addr:02X}: {e}")
        
        print(f"버스 {bus_num} 직접 스캔 완료: {[f'0x{addr:02X}' for addr in devices]}")
        return devices
    
    def scan_all(self):
        """모든 버스와 채널 스캔"""
        result = {}
        if not self.buses:
            print("연결된 I2C 버스 없음")
            return result
        
        if not self.address:
            print("TCA9548A 미탐지. 버스 직접 스캔...")
            for bus_num in self.buses:
                result[bus_num] = {'direct': self.scan_direct(bus_num)}
            return result
        
        for bus_num in self.buses:
            if bus_num != self.tca_bus:
                continue
            result[bus_num] = {}
            for channel in range(8):
                devices = self.scan_channel(bus_num, channel)
                result[bus_num][channel] = devices
        
        return result
    
    def close(self):
        """모든 버스 연결 종료"""
        for bus_num, bus in self.buses.items():
            try:
                if self.address and bus_num == self.tca_bus:
                    bus.write_byte(self.address, 0x00)
                bus.close()
                print(f"버스 {bus_num} 연결 종료")
            except Exception as e:
                print(f"버스 {bus_num} 연결 종료 실패: {e}")
        self.buses = {}

# 사용 예제
if __name__ == "__main__":
    try:
        mux = SimpleTCA9548A()
        
        print("\n=== I2C 디바이스 스캔 ===")
        scan_results = mux.scan_all()
        for bus_num in sorted(scan_results.keys()):
            print(f"\n버스 {bus_num}:")
            if 'direct' in scan_results[bus_num]:
                devices = scan_results[bus_num]['direct']
                print(f"  직접 스캔: {[f'0x{addr:02X}' for addr in devices] or '디바이스 없음'}")
            else:
                for channel in range(8):
                    devices = scan_results[bus_num][channel]
                    print(f"  채널 {channel}: {[f'0x{addr:02X}' for addr in devices] or '디바이스 없음'}")
        
    except Exception as e:
        print(f"오류: {e}")
        print("sudo 권한으로 실행: sudo python3 simpletca9548a.py")
    finally:
        mux.close()