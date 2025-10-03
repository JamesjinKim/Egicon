import tkinter as tk
from tkinter import ttk, messagebox
import ttkbootstrap as tb
from ttkbootstrap.constants import *
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import matplotlib.dates as mdates
from datetime import datetime, timedelta
import threading
import time
import queue
from collections import deque

# BME688 관련 imports
try:
    import smbus2
    import constants as const
    HAS_SENSOR_LIBS = True
except ImportError as e:
    print(f"센서 라이브러리 없음: {e}")
    HAS_SENSOR_LIBS = False

# 폰트 설정
try:
    import matplotlib.font_manager as fm
    korean_fonts = ['Noto Sans CJK KR', 'Noto Sans KR', 'NanumGothic', 'NanumBarunGothic', 'Malgun Gothic']
    available_fonts = [f.name for f in fm.fontManager.ttflist]
    korean_font = None
    for font in korean_fonts:
        if font in available_fonts:
            korean_font = font
            break
    if korean_font:
        plt.rcParams['font.family'] = korean_font
        print(f"한글 폰트 설정: {korean_font}")
    else:
        plt.rcParams['font.family'] = 'DejaVu Sans'
        print("한글 폰트 없음: DejaVu Sans 사용")
    plt.rcParams['axes.unicode_minus'] = False
except Exception as e:
    plt.rcParams['font.family'] = 'sans-serif'
    print(f"폰트 설정 실패: {e}")

class BME688Sensor:
    """BME688 센서 클래스"""
    
    def __init__(self, bus_number=0, address=0x77, temp_offset=0.0):
        self.bus_number = bus_number
        self.address = address
        self.bus = None
        self.temp_offset = temp_offset  # 센서 자체 발열 보정
        
        # 캘리브레이션 데이터 저장
        if HAS_SENSOR_LIBS:
            self.cal_data = const.CalibrationData()
        else:
            self.cal_data = None
        
    def connect(self):
        """센서 연결 및 초기화"""
        if not HAS_SENSOR_LIBS:
            return False
            
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
        else:
            # 가스 센서 상태 디버깅 정보
            print(f"가스 센서 상태: valid={bool(field_data['gas_valid'])}, stable={bool(field_data['heat_stable'])}, adc={field_data['gas_adc']}, range={field_data['gas_range']}")
        
        return {
            'timestamp': datetime.now(),
            'temperature': temperature,
            'pressure': pressure,
            'humidity': humidity,
            'gas_resistance': gas_resistance,
            'gas_valid': bool(field_data['gas_valid']),
            'heat_stable': bool(field_data['heat_stable'])
        }
    
    def close(self):
        """센서 연결 종료"""
        if self.bus:
            self.bus.close()

def find_bme688():
    """BME688 센서 검색"""
    if not HAS_SENSOR_LIBS:
        return None, None
        
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

class BME688MonitorGUI:
    """BME688 환경 모니터 GUI"""
    
    def __init__(self):
        self.max_points = 60
        self.timestamps = deque(maxlen=self.max_points)
        self.temperature_data = deque(maxlen=self.max_points)
        self.humidity_data = deque(maxlen=self.max_points)
        self.pressure_data = deque(maxlen=self.max_points)
        self.gas_data = deque(maxlen=self.max_points)
        
        self.sensor = None
        self.is_monitoring = False
        self.sensor_thread = None
        self.data_queue = queue.Queue()
        self.sensor_ready = False
        
        self.setup_gui()
        self.update_gui()
        
        # 센서 연결을 백그라운드에서 시도
        self.sensor_detection_thread = threading.Thread(target=self.background_sensor_detection, daemon=True)
        self.sensor_detection_thread.start()

    def setup_gui(self):
        """GUI 초기화"""
        self.root = tb.Window(
            title="BME688 환경 모니터",
            themename="solar",
            size=(800, 480),
            resizable=(False, False)
        )
        self.root.geometry("800x480+0+0")
        
        main_frame = tb.Frame(self.root, padding=8)
        main_frame.pack(fill=BOTH, expand=True)
        
        # 상단 헤더
        self.setup_header(main_frame)
        
        # 중간 컨텐츠 (좌측: 측정값, 우측: 차트)
        content_frame = tb.Frame(main_frame, height=300)
        content_frame.pack(fill=X, pady=(5, 0))
        content_frame.pack_propagate(False)
        
        self.setup_measurement_panel(content_frame)
        self.setup_chart_panel(content_frame)
        
        # 하단 상태바
        self.setup_status_bar(main_frame)
        
        # 디버그 로그
        self.setup_debug_log(main_frame)

    def setup_header(self, parent):
        """상단 헤더 설정"""
        header_frame = tb.Frame(parent)
        header_frame.pack(fill=X, pady=(0, 3))
        
        title_label = tb.Label(
            header_frame, 
            text="BME688 환경 모니터", 
            font=("DejaVu Sans", 16, "bold"),
            foreground="#2E86AB"
        )
        title_label.pack(side=LEFT)
        
        button_frame = tb.Frame(header_frame)
        button_frame.pack(side=RIGHT)
        
        self.start_button = tb.Button(
            button_frame,
            text="측정 시작",
            command=self.toggle_monitoring,
            style="success.TButton",
            width=10
        )
        self.start_button.pack(side=RIGHT, padx=(3, 0))
        
        self.sensor_status = tb.Label(
            button_frame,
            text="센서 검색 중...",
            font=("DejaVu Sans", 10),
            foreground="#F18F01"
        )
        self.sensor_status.pack(side=RIGHT, padx=(0, 3))

    def setup_measurement_panel(self, parent):
        """측정값 표시 패널 (좌측)"""
        measurement_frame = tb.LabelFrame(
            parent, 
            text="현재 측정값", 
            padding=8,
            style="info.TLabelframe"
        )
        measurement_frame.pack(side=LEFT, fill=BOTH, expand=True, padx=(0, 3))
        
        # 중요한 값들 (온도, 습도)
        important_frame = tb.Frame(measurement_frame)
        important_frame.pack(fill=X, pady=(0, 5))
        
        # 온도
        temp_frame = tb.Frame(important_frame)
        temp_frame.pack(side=LEFT, fill=X, expand=True)
        tb.Label(temp_frame, text="온도", font=("DejaVu Sans", 10, "bold")).pack()
        self.temperature_value = tb.Label(
            temp_frame, 
            text="--", 
            font=("DejaVu Sans", 12, "bold"),
            foreground="#E63946"
        )
        self.temperature_value.pack()
        tb.Label(temp_frame, text="°C", font=("DejaVu Sans", 8)).pack()
        
        # 습도
        humidity_frame = tb.Frame(important_frame)
        humidity_frame.pack(side=RIGHT, fill=X, expand=True)
        tb.Label(humidity_frame, text="습도", font=("DejaVu Sans", 10, "bold")).pack()
        self.humidity_value = tb.Label(
            humidity_frame, 
            text="--", 
            font=("DejaVu Sans", 12, "bold"),
            foreground="#2E86AB"
        )
        self.humidity_value.pack()
        tb.Label(humidity_frame, text="%", font=("DejaVu Sans", 8)).pack()
        
        # 작은 값들 (압력, 가스저항)
        small_frame = tb.Frame(measurement_frame)
        small_frame.pack(fill=X, pady=(5, 5))
        
        # 압력
        pressure_frame = tb.Frame(small_frame)
        pressure_frame.pack(side=LEFT, fill=X, expand=True)
        tb.Label(pressure_frame, text="압력", font=("DejaVu Sans", 8, "bold")).pack()
        self.pressure_value = tb.Label(
            pressure_frame, 
            text="--", 
            font=("DejaVu Sans", 12, "bold"),
            foreground="#F77F00"
        )
        self.pressure_value.pack()
        tb.Label(pressure_frame, text="hPa", font=("DejaVu Sans", 8)).pack()
        
        # 가스저항
        gas_frame = tb.Frame(small_frame)
        gas_frame.pack(side=RIGHT, fill=X, expand=True)
        tb.Label(gas_frame, text="가스저항", font=("DejaVu Sans", 8, "bold")).pack()
        self.gas_value = tb.Label(
            gas_frame, 
            text="--", 
            font=("DejaVu Sans", 12, "bold"),
            foreground="#A663CC"
        )
        self.gas_value.pack()
        tb.Label(gas_frame, text="Ω", font=("DejaVu Sans", 8)).pack()
        
        # 환경 상태
        self.environment_frame = tb.Frame(measurement_frame)
        self.environment_frame.pack(fill=X, pady=(5, 0))
        
        self.environment_label = tb.Label(
            self.environment_frame,
            text="환경 상태",
            font=("DejaVu Sans", 10, "bold")
        )
        self.environment_label.pack()
        
        self.environment_status = tb.Label(
            self.environment_frame,
            text="측정 준비",
            font=("DejaVu Sans", 10, "bold"),
            foreground="#6A994E"
        )
        self.environment_status.pack()

    def setup_chart_panel(self, parent):
        """차트 패널 설정 (우측)"""
        chart_frame = tb.LabelFrame(
            parent, 
            text="실시간 차트", 
            padding=3,
            style="info.TLabelframe"
        )
        chart_frame.pack(side=RIGHT, fill=BOTH, expand=True)
        
        # 어두운 테마 스타일
        plt.style.use('dark_background')
        self.fig = Figure(figsize=(5, 3), dpi=75, facecolor='#2E3440')
        
        # 두 개의 서브플롯
        self.ax1 = self.fig.add_subplot(211, facecolor='#2E3440')
        self.ax2 = self.fig.add_subplot(212, facecolor='#2E3440')
        self.fig.subplots_adjust(hspace=0.4)
        
        # 축 스타일
        for ax in [self.ax1, self.ax2]:
            ax.tick_params(colors='#D8DEE9', labelsize=7)
            ax.grid(True, alpha=0.3, color='#4C566A')
        
        # 온도/습도 차트
        self.ax1.set_ylabel('Temp(°C) / Humid(%)', color='#D8DEE9', fontsize=8)
        self.temp_line, = self.ax1.plot([], [], '-', color='#E63946', label='Temp', linewidth=2)
        self.humid_line, = self.ax1.plot([], [], '-', color='#2E86AB', label='Humid', linewidth=2)
        self.ax1.legend(loc='upper left', framealpha=0.8, facecolor='#3B4252', fontsize=7)
        
        # 압력 차트
        self.ax2.set_xlabel('Time', color='#D8DEE9', fontsize=8)
        self.ax2.set_ylabel('Pressure(hPa)', color='#D8DEE9', fontsize=8)
        self.pressure_line, = self.ax2.plot([], [], '-', color='#F77F00', label='Press', linewidth=2)
        self.ax2.legend(loc='upper left', framealpha=0.8, facecolor='#3B4252', fontsize=7)
        
        # 캔버스
        self.canvas = FigureCanvasTkAgg(self.fig, chart_frame)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill=BOTH, expand=True)
        
        self.setup_chart()

    def setup_chart(self):
        """차트 초기 설정"""
        now = datetime.now()
        for ax in [self.ax1, self.ax2]:
            ax.set_xlim(now - timedelta(minutes=5), now)
            ax.xaxis.set_major_formatter(mdates.DateFormatter('%H:%M'))
            ax.xaxis.set_major_locator(mdates.MinuteLocator(interval=1))
        
        self.ax1.set_ylim(0, 100)
        self.ax2.set_ylim(900, 1100)

    def setup_status_bar(self, parent):
        """하단 상태바"""
        status_frame = tb.Frame(parent)
        status_frame.pack(fill=X, pady=(5, 0))
        
        self.status_text = tb.Label(
            status_frame,
            text="준비",
            font=("DejaVu Sans", 9),
            foreground="#6A994E"
        )
        self.status_text.pack(side=LEFT)
        
        self.time_label = tb.Label(
            status_frame,
            text="",
            font=("DejaVu Sans", 9)
        )
        self.time_label.pack(side=RIGHT)

    def setup_debug_log(self, parent):
        """디버그 로그 영역 (하단)"""
        debug_frame = tb.LabelFrame(parent, text="로그", padding=8)
        debug_frame.pack(fill=BOTH, expand=True, pady=(8, 0))
        
        self.log_text = tk.Text(debug_frame, height=6, bg='#2E3440', fg='#D8DEE9', font=('monospace', 8))
        self.log_text.pack(fill=BOTH, expand=True)

    def log_message(self, message):
        """디버그 로그 메시지 추가"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        log_entry = f"[{timestamp}] {message}\n"
        self.log_text.insert(tk.END, log_entry)
        self.log_text.see(tk.END)
        
        # 로그 라인 수 제한
        lines = self.log_text.get("1.0", tk.END).split('\n')
        if len(lines) > 20:
            self.log_text.delete("1.0", "5.0")
        
        print(log_entry.strip())

    def background_sensor_detection(self):
        """백그라운드에서 센서 검색"""
        if not HAS_SENSOR_LIBS:
            self.root.after(0, lambda: self.sensor_status.config(text="라이브러리 없음", foreground="#E63946"))
            self.root.after(0, lambda: self.log_message("센서 라이브러리가 설치되지 않았습니다."))
            self.root.after(0, lambda: self.log_message("설치 방법: pip install smbus2"))
            return
            
        try:
            self.root.after(0, lambda: self.sensor_status.config(text="센서 검색 중...", foreground="#F18F01"))
            self.root.after(0, lambda: self.log_message("BME688 센서 검색 시작"))
            
            bus_num, addr = find_bme688()
            if bus_num is None:
                self.root.after(0, lambda: self.sensor_status.config(text="센서 없음", foreground="#E63946"))
                self.root.after(0, lambda: self.log_message("BME688 센서를 찾을 수 없습니다"))
                self.root.after(0, lambda: self.log_message("문제 해결 단계:"))
                self.root.after(0, lambda: self.log_message("1. 하드웨어 연결 확인: VCC→3.3V, GND→GND, SDA→GPIO2, SCL→GPIO3"))
                self.root.after(0, lambda: self.log_message("2. I2C 활성화: sudo raspi-config → Interface → I2C → Enable"))
                self.root.after(0, lambda: self.log_message("3. 권한 설정: sudo usermod -a -G i2c $USER"))
                self.root.after(0, lambda: self.log_message("4. 재부팅 후 다시 시도"))
                return
            
            # 센서 초기화 (원본 코드와 정확히 동일)
            self.sensor = BME688Sensor(bus_num, addr, temp_offset=-9.2)  # 34.2 - 25.0 = 9.2도 보정
            if self.sensor.connect():
                self.sensor_ready = True
                
                # 센서 안정화 (원본과 동일)
                self.root.after(0, lambda: self.log_message("센서 안정화 중..."))
                for i in range(3):
                    self.sensor.read_field_data()
                    time.sleep(1)
                
                # 테스트 읽기
                test_data = self.sensor.read_sensor_data()
                if test_data:
                    self.root.after(0, lambda: self.sensor_status.config(text="센서 연결됨", foreground="#6A994E"))
                    self.root.after(0, lambda: self.log_message(f"BME688 연결 성공: 버스 {bus_num}, 주소 0x{addr:02X}"))
                    self.root.after(0, lambda: self.log_message(f"초기 측정값: {test_data['temperature']:.1f}°C, {test_data['humidity']:.1f}%, {test_data['pressure']:.1f}hPa"))
                else:
                    self.root.after(0, lambda: self.log_message("센서 초기 읽기 실패"))
            else:
                self.root.after(0, lambda: self.sensor_status.config(text="센서 초기화 실패", foreground="#E63946"))
                self.root.after(0, lambda: self.log_message("센서 초기화 실패"))
                
        except Exception as e:
            error_msg = str(e)
            self.root.after(0, lambda: self.sensor_status.config(text="센서 오류", foreground="#E63946"))
            self.root.after(0, lambda: self.log_message(f"센서 검색 오류: {error_msg}"))

    def toggle_monitoring(self):
        """측정 시작/중지"""
        if not self.sensor or not self.sensor_ready:
            messagebox.showerror("오류", "센서가 연결되지 않았습니다.")
            return
            
        if not self.is_monitoring:
            self.start_monitoring()
        else:
            self.stop_monitoring()

    def start_monitoring(self):
        """측정 시작"""
        try:
            self.is_monitoring = True
            self.start_button.config(text="측정 중지", style="warning.TButton")
            self.status_text.config(text="측정 중...", foreground="#F18F01")
            self.log_message("측정 시작")
            
            self.sensor_thread = threading.Thread(target=self.sensor_loop, daemon=True)
            self.sensor_thread.start()
        except Exception as e:
            self.log_message(f"측정 시작 실패: {e}")
            messagebox.showerror("오류", f"측정 시작 실패: {e}")

    def stop_monitoring(self):
        """측정 중지"""
        self.is_monitoring = False
        self.start_button.config(text="측정 시작", style="success.TButton")
        self.status_text.config(text="측정 중지됨", foreground="#6A994E")
        self.log_message("측정 중지")

    def sensor_loop(self):
        """센서 데이터 읽기 루프"""
        try:
            self.log_message("BME688 측정 루프 시작")
            time.sleep(1)
            
            while self.is_monitoring:
                try:
                    data = self.sensor.read_sensor_data()
                    if data:
                        measurement = {
                            'timestamp': data['timestamp'],
                            'temperature': data['temperature'],
                            'humidity': data['humidity'],
                            'pressure': data['pressure'],
                            'gas_resistance': data['gas_resistance']
                        }
                        
                        # 공기질 평가
                        if data['gas_resistance'] > 50000:
                            air_quality = "좋음"
                        elif data['gas_resistance'] > 20000:
                            air_quality = "보통"
                        elif data['gas_resistance'] > 0:
                            air_quality = "나쁨"
                        else:
                            air_quality = "측정중"
                        
                        self.log_message(f"T={measurement['temperature']:.1f}°C "
                                       f"H={measurement['humidity']:.1f}% "
                                       f"P={measurement['pressure']:.1f}hPa "
                                       f"G={measurement['gas_resistance']:.0f}Ω ({air_quality})")
                        
                        self.data_queue.put(measurement)
                    else:
                        self.log_message("센서 읽기 실패")
                        
                    time.sleep(1)  # 1초마다 읽기 (원본과 동일)
                except Exception as e:
                    self.log_message(f"측정 오류: {e}")
                    time.sleep(1)
        except Exception as e:
            self.log_message(f"센서 루프 오류: {e}")

    def get_environment_status(self, temperature, humidity):
        """환경 상태 반환"""
        temp_ok = 18 <= temperature <= 26
        humid_ok = 40 <= humidity <= 60
        
        if temp_ok and humid_ok:
            return "쾌적", "#6A994E"
        elif temp_ok or humid_ok:
            return "보통", "#F18F01"
        else:
            return "불쾌적", "#E63946"

    def update_gui(self):
        """GUI 업데이트"""
        # 현재 시간 표시
        current_time = datetime.now().strftime("%H:%M:%S")
        self.time_label.config(text=current_time)
        
        # 큐에서 데이터 가져오기
        data_updated = False
        while not self.data_queue.empty():
            try:
                data = self.data_queue.get_nowait()
                
                # 데이터 유효성 확인
                if (data['temperature'] > 0 and data['temperature'] < 100 and
                    data['humidity'] > 0 and data['humidity'] < 100 and
                    data['pressure'] > 800 and data['pressure'] < 1200):
                    
                    # 데이터 저장
                    self.timestamps.append(data['timestamp'])
                    self.temperature_data.append(data['temperature'])
                    self.humidity_data.append(data['humidity'])
                    self.pressure_data.append(data['pressure'])
                    self.gas_data.append(data['gas_resistance'])
                    
                    # 값 표시 업데이트
                    self.temperature_value.config(text=f"{data['temperature']:.1f}")
                    self.humidity_value.config(text=f"{data['humidity']:.1f}")
                    self.pressure_value.config(text=f"{data['pressure']:.1f}")
                    self.gas_value.config(text=f"{data['gas_resistance']:.0f}")
                    
                    # 환경 상태 업데이트
                    status_text, status_color = self.get_environment_status(
                        data['temperature'], data['humidity']
                    )
                    self.environment_status.config(text=status_text, foreground=status_color)
                    
                    data_updated = True
                    
                    # 압력 데이터 디버깅
                    #print(f"압력 데이터 저장: {data['pressure']:.1f} hPa")
                    
                else:
                    self.log_message(f"비정상적인 데이터 무시: T={data['temperature']:.1f}, H={data['humidity']:.1f}, P={data['pressure']:.1f}")
                
            except queue.Empty:
                break
                
        # 차트 업데이트
        if data_updated:
            self.update_chart()
            
        # 1초마다 GUI 업데이트
        self.root.after(1000, self.update_gui)

    def update_chart(self):
        """차트 업데이트"""
        if len(self.timestamps) > 0:
            # 데이터 설정
            self.temp_line.set_data(self.timestamps, self.temperature_data)
            self.humid_line.set_data(self.timestamps, self.humidity_data)
            self.pressure_line.set_data(self.timestamps, self.pressure_data)
            
            # X축 범위 설정 (최근 5분)
            now = datetime.now()
            for ax in [self.ax1, self.ax2]:
                ax.set_xlim(now - timedelta(minutes=5), now)
                ax.xaxis.set_major_formatter(mdates.DateFormatter('%H:%M'))
                ax.xaxis.set_major_locator(mdates.MinuteLocator(interval=1))
            
            # Y축 범위 자동 조정
            if self.temperature_data and self.humidity_data:
                temp_range = [min(self.temperature_data), max(self.temperature_data)]
                humid_range = [min(self.humidity_data), max(self.humidity_data)]
                temp_margin = (temp_range[1] - temp_range[0]) * 0.1 or 5
                self.ax1.set_ylim(
                    min(temp_range[0] - temp_margin, 0),
                    max(humid_range[1] + 10, 100)
                )
                
            if self.pressure_data:
                pressure_range = [min(self.pressure_data), max(self.pressure_data)]
                pressure_margin = max((pressure_range[1] - pressure_range[0]) * 0.1, 10)  # 최소 10hPa 마진
                
                # 압력 범위가 정상적인지 확인 (900~1100 hPa 범위)
                if pressure_range[0] > 800 and pressure_range[1] < 1200:
                    self.ax2.set_ylim(
                        pressure_range[0] - pressure_margin,
                        pressure_range[1] + pressure_margin
                    )
                    # 디버깅 정보
                    #print(f"압력 차트 범위 업데이트: {pressure_range[0]:.1f} ~ {pressure_range[1]:.1f} hPa")
                else:
                    # 비정상적인 압력값이면 기본 범위 사용
                    self.ax2.set_ylim(900, 1100)
                    print(f"비정상적인 압력값 감지: {pressure_range}, 기본 범위(900-1100) 사용")
            else:
                # 데이터가 없으면 기본 범위 사용
                self.ax2.set_ylim(900, 1100)
                
            self.canvas.draw()

    def on_closing(self):
        """프로그램 종료 시 정리"""
        self.log_message("프로그램 종료")
        self.stop_monitoring()
        if self.sensor:
            self.sensor.close()
        time.sleep(1)
        self.root.destroy()

    def run(self):
        """프로그램 실행"""
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.root.mainloop()

if __name__ == "__main__":
    try:
        monitor = BME688MonitorGUI()
        monitor.run()
    except KeyboardInterrupt:
        print("\n프로그램이 사용자에 의해 중단되었습니다.")
    finally:
        print("프로그램 종료")