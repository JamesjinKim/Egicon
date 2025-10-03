#!/usr/bin/env python3
"""
심플한 BH1750 조도 센서 코드
데이터시트 기반 최소 구현
"""
import smbus2
import time

class SimpleBH1750:
    """BH1750 조도 센서 심플 클래스"""
    
    def __init__(self, bus=1, address=0x23):
        """
        초기화
        bus: I2C 버스 번호 (라즈베리파이는 보통 1)
        address: I2C 주소 (0x23 또는 0x5C)
        """
        self.bus = smbus2.SMBus(bus)
        self.address = address
    
    def read_light(self):
        """
        조도값 읽기
        반환: 조도값 (lux)
        """
        try:
            # 측정 명령 전송 (One Time High Resolution Mode)
            self.bus.write_byte(self.address, 0x20)
            
            # 측정 대기 (120ms)
            time.sleep(0.12)
            
            # 2바이트 데이터 읽기
            data = self.bus.read_i2c_block_data(self.address, 0x20, 2)
            
            # 조도값 계산 (데이터시트 공식)
            lux = ((data[0] << 8) + data[1]) / 1.2
            
            return round(lux, 1)
            
        except:
            return None
    
    def close(self):
        """연결 종료"""
        self.bus.close()

# 사용 예제
if __name__ == "__main__":
    sensor = SimpleBH1750()
    
    for i in range(5):
        light = sensor.read_light()
        if light is not None:
            print(f"조도: {light} lux")
        else:
            print("측정 실패")
        time.sleep(1)
    
    sensor.close()