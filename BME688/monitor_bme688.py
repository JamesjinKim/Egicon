#!/usr/bin/env python3
"""
BME688 센서 정확한 측정 모니터링
- constants.py 파일 사용
- 온도 오프셋 적용으로 정확한 온도 측정
- 연속 모드로 실시간 변화 감지
- 온도 오프셋: -9.2도 적용으로 34.2°C → 25.0°C 보정
- 높은 정밀도: 온도 x4, 압력 x16 오버샘플링
- 정확한 캘리브레이션: GitHub 원본 공식 사용
- 가스 센서 개선: Bosch 공식으로 히터 제어
"""

import time
import smbus2
from datetime import datetime
import constants as const

class BME688Sensor:
    """BME688 센서 클래스"""
    
    def __init__(self, bus_number=0, address=0x77, temp_offset=0.0):
        self.bus_number = bus_number
        self.address = address
        self.bus = None
        self.temp_offset = temp_offset  # 센서 자체 발열 보정
        
        # 캘리브레이션 데이터 저장
        self.cal_data = const.CalibrationData()
        
    def connect(self):
        """센서 연결 및 초기화"""
        try:
            self.bus = smbus2.SMBus(self.bus_number)
            
            # 칩 ID 확인
            chip_id = self.bus.read_byte_data(self.address, const.CHIP_ID_ADDR)
            if chip_id != const.CHIP_ID:
                print(f"ERROR: BME680/688이 아닙니다. 칩 ID: 0x{chip_id:02X}")
                return False
                
            print(f"BME688 센서 연결 성공 (칩 ID: 0x{chip_id:02X})")
            
            # 소프트 리셋
            self.bus.write_byte_data(self.address, const.SOFT_RESET_ADDR, const.SOFT_RESET_CMD)
            time.sleep(0.01)
            
            # 캘리브레이션 데이터 읽기
            if not self.read_calibration():
                return False
                
            # 센서 설정
            if not self.configure_sensor():
                return False
                
            print("센서 초기화 완료")
            return True
            
        except Exception as e:
            print(f"ERROR: 센서 연결 실패: {e}")
            return False
    
    def read_calibration(self):
        """캘리브레이션 데이터 읽기"""
        try:
            print("캘리브레이션 데이터 읽는 중...")
            
            # 첫 번째 캘리브레이션 영역 읽기
            coeff1 = []
            for i in range(const.COEFF_ADDR1_LEN):
                coeff1.append(self.bus.read_byte_data(self.address, const.COEFF_ADDR1 + i))
            
            # 두 번째 캘리브레이션 영역 읽기
            coeff2 = []
            for i in range(const.COEFF_ADDR2_LEN):
                coeff2.append(self.bus.read_byte_data(self.address, const.COEFF_ADDR2 + i))
            
            # 전체 캘리브레이션 배열
            calibration = coeff1 + coeff2
            
            # CalibrationData 클래스의 set_from_array 메서드 사용
            self.cal_data.set_from_array(calibration)
            
            # 추가 보정값 읽기
            heat_range = self.bus.read_byte_data(self.address, const.ADDR_RES_HEAT_RANGE_ADDR)
            heat_value = self.bus.read_byte_data(self.address, const.ADDR_RES_HEAT_VAL_ADDR)
            sw_error = self.bus.read_byte_data(self.address, const.ADDR_RANGE_SW_ERR_ADDR)
            
            self.cal_data.set_other(heat_range, heat_value, sw_error)
            
            print(f"캘리브레이션 완료 - T1={self.cal_data.par_t1}, T2={self.cal_data.par_t2}")
            return True
            
        except Exception as e:
            print(f"ERROR: 캘리브레이션 읽기 실패: {e}")
            return False
    
    def configure_sensor(self):
        """센서 설정"""
        try:
            # 습도 오버샘플링 x2
            self.bus.write_byte_data(self.address, const.CONF_OS_H_ADDR, const.OS_2X)
            
            # 온도 x4, 압력 x16, 연속 모드 (정확도 향상)
            ctrl_meas = (const.OS_4X << const.OST_POS) | (const.OS_16X << const.OSP_POS) | const.FORCED_MODE
            self.bus.write_byte_data(self.address, const.CONF_T_P_MODE_ADDR, ctrl_meas)
            
            # IIR 필터 계수 7 (노이즈 감소)
            config = const.FILTER_SIZE_7 << const.FILTER_POS
            self.bus.write_byte_data(self.address, const.CONF_ODR_FILT_ADDR, config)
            
            # 가스 센서 설정
            self.setup_gas_sensor()
            
            print("센서 설정 완료")
            return True
            
        except Exception as e:
            print(f"ERROR: 센서 설정 실패: {e}")
            return False
    
    def setup_gas_sensor(self):
        """가스 센서 설정"""
        try:
            # 가스 히터 온도 계산 (320도)
            target_temp = 320
            amb_temp = 25  # 주변 온도 가정
            
            # Bosch 공식 사용한 히터 저항 계산
            var1 = (self.cal_data.par_gh1 / 16.0) + 49.0
            var2 = ((self.cal_data.par_gh2 / 32768.0) * 0.0005) + 0.00235
            var3 = self.cal_data.par_gh3 / 1024.0
            var4 = var1 * (1.0 + (var2 * target_temp))
            var5 = var4 + (var3 * amb_temp)
            
            res_heat = int(3.4 * ((var5 * (4.0 / (4.0 + self.cal_data.res_heat_range)) *
                    (1.0 / (1.0 + (self.cal_data.res_heat_val * 0.002)))) - 25))
            
            # 가스 히터 온도 설정
            self.bus.write_byte_data(self.address, const.RES_HEAT0_ADDR, max(0, min(255, res_heat)))
            
            # 가스 히터 지속시간 (150ms)
            duration_ms = 150
            factor = 0
            durval = 0xFF  # 기본값
            
            if duration_ms >= 0xfc0:
                durval = 0xff
            else:
                while duration_ms > 0x3F:
                    duration_ms = duration_ms // 4
                    factor += 1
                durval = duration_ms + (factor * 64)
            
            self.bus.write_byte_data(self.address, const.GAS_WAIT0_ADDR, durval)
            
            # 가스 측정 활성화
            gas_conf = const.RUN_GAS_ENABLE << const.RUN_GAS_POS
            self.bus.write_byte_data(self.address, const.CONF_ODR_RUN_GAS_NBC_ADDR, gas_conf)
            
            # 히터 제어 활성화
            self.bus.write_byte_data(self.address, const.CONF_HEAT_CTRL_ADDR, const.ENABLE_HEATER)
            
        except Exception as e:
            print(f"WARNING: 가스 센서 설정 실패: {e}")
    
    def read_field_data(self):
        """센서 데이터 읽기"""
        try:
            # 강제 측정 모드 시작
            ctrl_meas = (const.OS_4X << const.OST_POS) | (const.OS_16X << const.OSP_POS) | const.FORCED_MODE
            self.bus.write_byte_data(self.address, const.CONF_T_P_MODE_ADDR, ctrl_meas)
            
            # 측정 완료 대기
            time.sleep(0.5)  # 더 긴 대기 시간으로 정확도 향상
            
            # 상태 확인
            status = self.bus.read_byte_data(self.address, const.FIELD0_ADDR)
            new_data = status & const.NEW_DATA_MSK
            
            if not new_data:
                return None
            
            # 온도 데이터 읽기 (20비트)
            temp_data = []
            for i in range(3):
                temp_data.append(self.bus.read_byte_data(self.address, 0x22 + i))
            temp_adc = (temp_data[0] << 12) | (temp_data[1] << 4) | (temp_data[2] >> 4)
            
            # 압력 데이터 읽기 (20비트)
            press_data = []
            for i in range(3):
                press_data.append(self.bus.read_byte_data(self.address, 0x1F + i))
            press_adc = (press_data[0] << 12) | (press_data[1] << 4) | (press_data[2] >> 4)
            
            # 습도 데이터 읽기 (16비트)
            hum_data = []
            for i in range(2):
                hum_data.append(self.bus.read_byte_data(self.address, 0x25 + i))
            hum_adc = (hum_data[0] << 8) | hum_data[1]
            
            # 가스 데이터 읽기
            gas_data = []
            for i in range(2):
                gas_data.append(self.bus.read_byte_data(self.address, 0x2A + i))
            gas_adc = (gas_data[0] << 2) | (gas_data[1] >> 6)
            gas_range = gas_data[1] & 0x0F
            gas_valid = gas_data[1] & const.GASM_VALID_MSK
            heat_stable = gas_data[1] & const.HEAT_STAB_MSK
            
            return {
                'temp_adc': temp_adc,
                'press_adc': press_adc,
                'hum_adc': hum_adc,
                'gas_adc': gas_adc,
                'gas_range': gas_range,
                'gas_valid': gas_valid,
                'heat_stable': heat_stable
            }
            
        except Exception as e:
            print(f"ERROR: 데이터 읽기 실패: {e}")
            return None
    
    def compensate_temperature(self, temp_adc):
        """온도 보정 계산"""
        var1 = (temp_adc / 16384.0 - self.cal_data.par_t1 / 1024.0) * self.cal_data.par_t2
        var2 = ((temp_adc / 131072.0 - self.cal_data.par_t1 / 8192.0) * 
                (temp_adc / 131072.0 - self.cal_data.par_t1 / 8192.0)) * (self.cal_data.par_t3 * 16.0)
        
        self.cal_data.t_fine = var1 + var2
        temp_comp = (var1 + var2) / 5120.0
        
        # 온도 오프셋 적용 (센서 자체 발열 보정)
        return temp_comp + self.temp_offset
    
    def compensate_pressure(self, press_adc):
        """압력 보정 계산"""
        var1 = (self.cal_data.t_fine / 2.0) - 64000.0
        var2 = var1 * var1 * (self.cal_data.par_p6 / 131072.0)
        var2 = var2 + (var1 * self.cal_data.par_p5 * 2.0)
        var2 = (var2 / 4.0) + (self.cal_data.par_p4 * 65536.0)
        var1 = (((self.cal_data.par_p3 * var1 * var1) / 16384.0) + 
                (self.cal_data.par_p2 * var1)) / 524288.0
        var1 = (1.0 + (var1 / 32768.0)) * self.cal_data.par_p1
        
        if var1 == 0:
            return 0
        
        press_comp = 1048576.0 - press_adc
        press_comp = ((press_comp - (var2 / 4096.0)) * 6250.0) / var1
        var1 = (self.cal_data.par_p9 * press_comp * press_comp) / 2147483648.0
        var2 = press_comp * (self.cal_data.par_p8 / 32768.0)
        var3 = ((press_comp / 256.0) * (press_comp / 256.0) * 
                (press_comp / 256.0) * (self.cal_data.par_p10 / 131072.0))
        press_comp = press_comp + (var1 + var2 + var3 + (self.cal_data.par_p7 * 128.0)) / 16.0
        
        return press_comp / 100.0  # Pa를 hPa로 변환
    
    def compensate_humidity(self, hum_adc):
        """습도 보정 계산"""
        temp_scaled = self.cal_data.t_fine / 5120.0
        
        var1 = hum_adc - (self.cal_data.par_h1 * 16.0 + 
                         (self.cal_data.par_h3 / 2.0) * temp_scaled)
        var2 = var1 * ((self.cal_data.par_h2 / 262144.0) * 
                      (1.0 + (self.cal_data.par_h4 / 16384.0) * temp_scaled + 
                       (self.cal_data.par_h5 / 1048576.0) * temp_scaled * temp_scaled))
        var3 = self.cal_data.par_h6 / 16384.0
        var4 = self.cal_data.par_h7 / 2097152.0
        hum_comp = var2 + ((var3 + (var4 * temp_scaled)) * var2 * var2)
        
        return max(0.0, min(100.0, hum_comp))
    
    def compensate_gas_resistance(self, gas_adc, gas_range):
        """가스 저항 보정 계산"""
        if gas_adc == 0 or gas_range >= len(const.lookupTable1):
            return 0
        
        var1 = const.lookupTable1[gas_range]
        var2 = const.lookupTable2[gas_range]
        
        var3 = ((1340.0 + (5.0 * self.cal_data.res_heat_range)) * var1) / 65536.0
        gas_res = var3 + (var2 * gas_adc) / 512.0 + gas_adc
        
        return gas_res
    
    def read_sensor_data(self):
        """완전한 센서 데이터 읽기"""
        field_data = self.read_field_data()
        if not field_data:
            return None
        
        # 온도 보정 (가장 먼저)
        temperature = self.compensate_temperature(field_data['temp_adc'])
        
        # 압력 보정 (t_fine 사용)
        pressure = self.compensate_pressure(field_data['press_adc'])
        
        # 습도 보정 (t_fine 사용)
        humidity = self.compensate_humidity(field_data['hum_adc'])
        
        # 가스 저항 보정
        gas_resistance = 0
        if field_data['gas_valid'] and field_data['heat_stable']:
            gas_resistance = self.compensate_gas_resistance(
                field_data['gas_adc'], field_data['gas_range'])
        
        return {
            'timestamp': datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
            'temperature': temperature,
            'pressure': pressure,
            'humidity': humidity,
            'gas_resistance': gas_resistance,
            'gas_valid': bool(field_data['gas_valid']),
            'heat_stable': bool(field_data['heat_stable'])
        }
    
    def monitor(self, interval=3):
        """센서 모니터링"""
        print("BME688 센서 모니터링 시작")
        print("종료하려면 Ctrl+C를 누르세요")
        print("-" * 80)
        
        try:
            while True:
                data = self.read_sensor_data()
                if data:
                    # 공기질 평가
                    if data['gas_resistance'] > 50000:
                        air_quality = "좋음"
                    elif data['gas_resistance'] > 20000:
                        air_quality = "보통"
                    elif data['gas_resistance'] > 0:
                        air_quality = "나쁨"
                    else:
                        air_quality = "측정중"
                    
                    print(f"[{data['timestamp']}] "
                          f"온도: {data['temperature']:5.1f}°C | "
                          f"압력: {data['pressure']:7.1f}hPa | "
                          f"습도: {data['humidity']:5.1f}% | "
                          f"가스: {data['gas_resistance']:8.0f}Ω ({air_quality})")
                else:
                    print("데이터 읽기 실패")
                
                time.sleep(interval)
                
        except KeyboardInterrupt:
            print("\n모니터링 중단됨")
        except Exception as e:
            print(f"ERROR: 모니터링 오류: {e}")
        finally:
            if self.bus:
                self.bus.close()

def find_bme688():
    """BME688 센서 검색"""
    print("BME688 센서 검색 중...")
    
    for bus_num in [0, 1]:
        for addr in [const.I2C_ADDR_SECONDARY, const.I2C_ADDR_PRIMARY]:
            try:
                bus = smbus2.SMBus(bus_num)
                chip_id = bus.read_byte_data(addr, const.CHIP_ID_ADDR)
                bus.close()
                
                if chip_id == const.CHIP_ID:
                    print(f"BME688 발견: 버스 {bus_num}, 주소 0x{addr:02X}")
                    return bus_num, addr
                    
            except:
                pass
    
    print("BME688 센서를 찾을 수 없습니다")
    return None, None

def main():
    print("=" * 60)
    print("BME688 센서 정확한 측정 모니터")
    print("=" * 60)
    
    # 센서 검색
    bus_num, addr = find_bme688()
    if bus_num is None:
        print("해결 방법:")
        print("1. 하드웨어 연결 확인")
        print("2. I2C 활성화: sudo raspi-config")
        print("3. 권한 확인: sudo usermod -a -G i2c $USER")
        return
    
    """일반적인 온도 오프셋 범위
    정상 범위: +3~5°C 높게 측정
    Adafruit 권장: -5°C 오프셋
    Pimoroni 권장: -5~-8°C 오프셋
    """
    # 센서 초기화 (온도 오프셋 임의적용)
    sensor = BME688Sensor(bus_num, addr, temp_offset=-10)  # 10도 보정
    
    if not sensor.connect():
        return
    
    # 센서 안정화
    print("센서 안정화 중...")
    for i in range(3):
        sensor.read_field_data()
        time.sleep(1)
    
    # 모니터링 시작
    sensor.monitor(interval=1)  # 1초 간격으로 빠른 변화 감지

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n프로그램이 사용자에 의해 중단되었습니다.")
    finally:
        print("프로그램 종료")