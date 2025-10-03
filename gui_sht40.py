#!/usr/bin/env python3
"""
SHT40 온습도 센서 모니터
라즈베리파이 4용 GUI 프로그램 (개선된 I2C 통신 방식)
"""

import tkinter as tk
from tkinter import ttk, messagebox
import ttkbootstrap as tb
from ttkbootstrap.constants import *
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import matplotlib.dates as mdates

# 라즈베리파이용 폰트 설정
try:
    plt.rcParams['font.family'] = 'DejaVu Sans'
    plt.rcParams['axes.unicode_minus'] = False
except:
    plt.rcParams['font.family'] = 'sans-serif'

from datetime import datetime, timedelta
import threading
import time
import queue
from collections import deque
import smbus2
import subprocess
import logging

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

class SimpleSHT40:
    """SHT40 온습도 센서 클래스 (개선된 I2C 방식)"""
    
    # I2C addresses
    DEFAULT_I2C_ADDRESS = 0x44  # SHT40 표준 주소
    
    # Commands
    CMD_MEASURE_HIGH_PRECISION = 0xFD  # High precision measurement
    CMD_MEASURE_MEDIUM_PRECISION = 0xF6  # Medium precision measurement
    CMD_MEASURE_LOW_PRECISION = 0xE0  # Low precision measurement
    CMD_READ_SERIAL_NUMBER = 0x89  # Read serial number
    CMD_SOFT_RESET = 0x94  # Soft reset
    
    def __init__(self, bus=1, address=DEFAULT_I2C_ADDRESS):
        self.bus_num = bus
        self.address = address
        self.bus = None
    
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

class SHT40Monitor:
    def __init__(self):
        # 데이터 저장용 deque (최대 60개 데이터 포인트)
        self.max_points = 60
        self.timestamps = deque(maxlen=self.max_points)
        self.temperature_data = deque(maxlen=self.max_points)
        self.humidity_data = deque(maxlen=self.max_points)
        
        # 센서 관련
        self.sensor = None
        self.sensor_bus = None
        self.sensor_address = None
        self.is_monitoring = False
        self.sensor_thread = None
        self.data_queue = queue.Queue()
        self.sensor_ready = False
        
        # GUI 초기화
        self.setup_gui()
        
        # GUI 업데이트 타이머 시작
        self.update_gui()
        
        # 센서 검색을 별도 스레드에서 실행
        self.sensor_detection_thread = threading.Thread(target=self.background_sensor_detection, daemon=True)
        self.sensor_detection_thread.start()
    
    def setup_gui(self):
        """GUI 초기화 - 라즈베리파이 화면 크기에 맞게 조정"""
        self.root = tb.Window(
            title="SHT40 온습도 센서 모니터",
            themename="solar",
            size=(800, 480),
            resizable=(False, False)
        )
        self.root.geometry("800x480+0+0")
        
        # 메인 컨테이너
        main_frame = tb.Frame(self.root, padding=8)
        main_frame.pack(fill=BOTH, expand=True)
        
        # 상단 헤더
        self.setup_header(main_frame)
        
        # 중앙 컨텐츠 (측정값 + 차트)
        content_frame = tb.Frame(main_frame, height=300)
        content_frame.pack(fill=X, pady=(5, 0))
        content_frame.pack_propagate(False)
        
        # 측정값 표시 영역 (왼쪽)
        self.setup_measurement_panel(content_frame)
        
        # 차트 영역 (오른쪽)
        self.setup_chart_panel(content_frame)
        
        # 하단 상태바
        self.setup_status_bar(main_frame)
        
        # 디버그 로그 영역
        self.setup_debug_log(main_frame)
    
    def setup_header(self, parent):
        """상단 헤더 설정"""
        header_frame = tb.Frame(parent)
        header_frame.pack(fill=X, pady=(0, 3))
        
        # 제목
        title_label = tb.Label(
            header_frame, 
            text="SHT40 온습도 센서 모니터", 
            font=("DejaVu Sans", 16, "bold"),
            foreground="#2E86AB"
        )
        title_label.pack(side=LEFT)
        
        # 버튼 프레임
        button_frame = tb.Frame(header_frame)
        button_frame.pack(side=RIGHT)
        
        # 시작/중지 버튼
        self.start_button = tb.Button(
            button_frame,
            text="측정 시작",
            command=self.toggle_monitoring,
            style="success.TButton",
            width=10
        )
        self.start_button.pack(side=RIGHT, padx=(3, 0))
        
        # 센서 상태 표시
        self.sensor_status = tb.Label(
            button_frame,
            text="센서 검색 중...",
            font=("DejaVu Sans", 10),
            foreground="#F18F01"
        )
        self.sensor_status.pack(side=RIGHT, padx=(0, 3))
    
    def setup_measurement_panel(self, parent):
        """측정값 표시 패널"""
        measurement_frame = tb.LabelFrame(
            parent, 
            text="현재 측정값", 
            padding=8,
            style="info.TLabelframe"
        )
        measurement_frame.pack(side=LEFT, fill=BOTH, expand=True, padx=(0, 3))
        
        # 온도 표시 영역
        temp_frame = tb.Frame(measurement_frame)
        temp_frame.pack(fill=X, pady=(0, 10))
        
        tb.Label(
            temp_frame, 
            text="온도", 
            font=("DejaVu Sans", 12, "bold")
        ).pack(side=LEFT)
        
        self.temperature_value = tb.Label(
            temp_frame, 
            text="--", 
            font=("DejaVu Sans", 24, "bold"),
            foreground="#E63946"
        )
        self.temperature_value.pack(side=LEFT, padx=(10, 5))
        
        tb.Label(
            temp_frame, 
            text="°C", 
            font=("DejaVu Sans", 12)
        ).pack(side=LEFT)
        
        # 습도 표시 영역
        humidity_frame = tb.Frame(measurement_frame)
        humidity_frame.pack(fill=X, pady=(0, 15))
        
        tb.Label(
            humidity_frame, 
            text="습도", 
            font=("DejaVu Sans", 12, "bold")
        ).pack(side=LEFT)
        
        self.humidity_value = tb.Label(
            humidity_frame, 
            text="--", 
            font=("DejaVu Sans", 24, "bold"),
            foreground="#2D5BFF"
        )
        self.humidity_value.pack(side=LEFT, padx=(10, 5))
        
        tb.Label(
            humidity_frame, 
            text="%RH", 
            font=("DejaVu Sans", 12)
        ).pack(side=LEFT)
        
        # 환경 상태 표시
        self.comfort_frame = tb.Frame(measurement_frame)
        self.comfort_frame.pack(fill=X, pady=(10, 0))
        
        self.comfort_label = tb.Label(
            self.comfort_frame,
            text="환경 상태",
            font=("DejaVu Sans", 11, "bold")
        )
        self.comfort_label.pack()
        
        self.comfort_status = tb.Label(
            self.comfort_frame,
            text="측정 준비",
            font=("DejaVu Sans", 12, "bold"),
            foreground="#6A994E"
        )
        self.comfort_status.pack()
        
        # 통계 정보
        stats_frame = tb.Frame(measurement_frame)
        stats_frame.pack(fill=X, pady=(15, 0))
        
        # 온도 통계
        self.temp_stats_label = tb.Label(
            stats_frame,
            text="온도 통계\n최대: -- °C\n최소: -- °C\n평균: -- °C",
            font=("DejaVu Sans", 8),
            justify="left"
        )
        self.temp_stats_label.pack(anchor=W)
        
        # 습도 통계
        self.humidity_stats_label = tb.Label(
            stats_frame,
            text="습도 통계\n최대: -- %RH\n최소: -- %RH\n평균: -- %RH",
            font=("DejaVu Sans", 8),
            justify="left"
        )
        self.humidity_stats_label.pack(anchor=W, pady=(5, 0))
    
    def setup_chart_panel(self, parent):
        """차트 패널 설정"""
        chart_frame = tb.LabelFrame(
            parent, 
            text="실시간 차트", 
            padding=3,
            style="info.TLabelframe"
        )
        chart_frame.pack(side=RIGHT, fill=BOTH, expand=True)
        
        # matplotlib 설정
        plt.style.use('dark_background')
        self.fig = Figure(figsize=(5, 3), dpi=75, facecolor='#2E3440')
        self.ax = self.fig.add_subplot(111, facecolor='#2E3440')
        
        # 차트 스타일링
        self.ax.set_xlabel('시간', color='#D8DEE9', fontsize=9)
        self.ax.set_ylabel('온도 (°C) / 습도 (%RH)', color='#D8DEE9', fontsize=9)
        self.ax.tick_params(colors='#D8DEE9', labelsize=8)
        self.ax.grid(True, alpha=0.3, color='#4C566A')
        
        # 온도 및 습도 라인
        self.temp_line, = self.ax.plot([], [], '-', color='#E63946', label='온도 (°C)', linewidth=2)
        self.humidity_line, = self.ax.plot([], [], '-', color='#2D5BFF', label='습도 (%RH)', linewidth=2)
        
        self.ax.legend(loc='upper left', framealpha=0.8, facecolor='#3B4252', fontsize=8)
        
        # 차트를 tkinter에 임베드
        self.canvas = FigureCanvasTkAgg(self.fig, chart_frame)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill=BOTH, expand=True)
        
        # 초기 차트 설정
        self.setup_chart()
    
    def setup_chart(self):
        """차트 초기 설정"""
        now = datetime.now()
        self.ax.set_xlim(now - timedelta(minutes=5), now)
        self.ax.set_ylim(0, 100)
        
        # 시간 축 포맷
        self.ax.xaxis.set_major_formatter(mdates.DateFormatter('%H:%M'))
        self.ax.xaxis.set_major_locator(mdates.MinuteLocator(interval=1))
    
    def setup_status_bar(self, parent):
        """하단 상태바"""
        status_frame = tb.Frame(parent)
        status_frame.pack(fill=X, pady=(5, 0))
        
        # 상태 텍스트
        self.status_text = tb.Label(
            status_frame,
            text="준비",
            font=("DejaVu Sans", 9),
            foreground="#6A994E"
        )
        self.status_text.pack(side=LEFT)
        
        # 현재 시간
        self.time_label = tb.Label(
            status_frame,
            text="",
            font=("DejaVu Sans", 9)
        )
        self.time_label.pack(side=RIGHT)
    
    def setup_debug_log(self, parent):
        """디버그 로그 영역"""
        debug_frame = tb.LabelFrame(parent, text="로그", padding=8)
        debug_frame.pack(fill=BOTH, expand=True, pady=(8, 0))
        
        # 로그 텍스트 영역
        self.log_text = tk.Text(debug_frame, height=6, bg='#2E3440', fg='#D8DEE9', font=('monospace', 8))
        self.log_text.pack(fill=BOTH, expand=True)
    
    def log_message(self, message):
        """디버그 로그 메시지 추가"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        log_entry = f"[{timestamp}] {message}\n"
        
        self.log_text.insert(tk.END, log_entry)
        self.log_text.see(tk.END)
        
        # 로그가 너무 많아지면 오래된 것 삭제
        lines = self.log_text.get("1.0", tk.END).split('\n')
        if len(lines) > 20:
            self.log_text.delete("1.0", "5.0")
        
        print(log_entry.strip())
    
    def background_sensor_detection(self):
        """백그라운드에서 센서 검색 및 연결 테스트"""
        try:
            # GUI 스레드에서 상태 업데이트
            self.root.after(0, lambda: self.sensor_status.config(text="센서 검색 중...", foreground="#F18F01"))
            self.log_message("I2C 버스 스캔 시작 (0x44 주소)")
            
            # 개선된 스캔 함수 사용
            found_sensor, found_bus, found_address = scan_i2c_bus()
            
            if found_sensor:
                self.log_message(f"SHT40 센서 발견: 버스 {found_bus}, 주소 0x{found_address:02X}")
                
                # 테스트 연결
                try:
                    test_sensor = SimpleSHT40(bus=found_bus, address=found_address)
                    test_sensor.connect()
                    
                    # 측정 테스트
                    temp_val, humidity_val = test_sensor.read_with_retry(precision="high")
                    test_sensor.close()
                    
                    if temp_val is not None and humidity_val is not None:
                        self.sensor_bus = found_bus
                        self.sensor_address = found_address
                        self.sensor = SimpleSHT40(bus=found_bus, address=found_address)
                        self.sensor_ready = True
                        
                        self.root.after(0, lambda: self.sensor_status.config(
                            text=f"센서 연결됨 (버스 {found_bus})", 
                            foreground="#6A994E"
                        ))
                        self.root.after(0, lambda: self.log_message(
                            f"SHT40 센서 연결 성공: 버스 {found_bus}, 주소 0x{found_address:02X}, "
                            f"온도 {temp_val}°C, 습도 {humidity_val}%RH"
                        ))
                except Exception as e:
                    self.root.after(0, lambda err=e: self.log_message(f"센서 테스트 연결 실패: {err}"))
                    self.root.after(0, lambda: self.sensor_status.config(text="센서 연결 실패", foreground="#E63946"))
            else:
                self.root.after(0, lambda: self.sensor_status.config(text="센서 없음", foreground="#E63946"))
                self.root.after(0, lambda: self.log_message("SHT40 센서를 찾을 수 없습니다. I2C가 활성화되어 있고 센서가 연결되어 있는지 확인하세요."))
                
        except Exception as e:
            self.root.after(0, lambda: self.sensor_status.config(text="연결 실패", foreground="#E63946"))
            self.root.after(0, lambda: self.log_message(f"센서 검색 오류: {e}"))
    
    def toggle_monitoring(self):
        """측정 시작/중지"""
        if not self.sensor_ready:
            # 센서가 준비되지 않았으면, 센서 검색을 다시 시도
            self.log_message("센서가 연결되지 않아 재검색을 시도합니다.")
            self.sensor_status.config(text="센서 재검색 중...", foreground="#F18F01")
            self.sensor_ready = False
            found = self.try_find_sensor()
            if not found:
                messagebox.showwarning("알림", "센서가 연결되지 않았습니다.\n센서를 연결한 후 다시 시도하세요.")
                self.sensor_status.config(text="센서 없음", foreground="#E63946")
                return
            else:
                self.sensor_status.config(text="센서 연결됨", foreground="#6A994E")
                self.log_message("센서 재검색 성공, 측정 시작 가능")
        
        if not self.is_monitoring:
            self.start_monitoring()
        else:
            self.stop_monitoring()

    def try_find_sensor(self):
        """센서 검색 및 연결 테스트(동기, 버튼 클릭 시 사용)"""
        try:
            # 개선된 스캔 함수 사용
            found_sensor, found_bus, found_address = scan_i2c_bus()
            
            if found_sensor:
                try:
                    test_sensor = SimpleSHT40(bus=found_bus, address=found_address)
                    test_sensor.connect()
                    temp_val, humidity_val = test_sensor.read_with_retry(precision="high")
                    test_sensor.close()
                    
                    if temp_val is not None and humidity_val is not None:
                        self.sensor_bus = found_bus
                        self.sensor_address = found_address
                        self.sensor = SimpleSHT40(bus=found_bus, address=found_address)
                        self.sensor_ready = True
                        self.log_message(f"센서 연결 성공: 버스 {found_bus}, 주소 0x{found_address:02X}, 온도 {temp_val}°C, 습도 {humidity_val}%RH")
                        return True
                except Exception as e:
                    self.log_message(f"센서 테스트 연결 실패: {e}")
            
            self.sensor_ready = False
            return False
        except Exception as e:
            self.log_message(f"센서 검색 실패: {e}")
            self.sensor_ready = False
            return False
    
    def start_monitoring(self):
        """측정 시작"""
        try:
            self.is_monitoring = True
            self.start_button.config(text="측정 중지", style="warning.TButton")
            self.status_text.config(text="측정 중...", foreground="#F18F01")
            self.log_message("측정 시작")
            
            # 측정 스레드 시작
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
            self.log_message("센서 연결 중...")
            
            # 센서 연결
            self.sensor.connect()
            self.log_message("센서 연결 완료")
            
            fail_count = 0  # 연속 실패 횟수
            
            # 데이터 수집 루프
            while self.is_monitoring:
                try:
                    # 개선된 측정 함수 사용
                    result = self.sensor.read_with_retry(precision="high")
                    
                    if result:
                        temperature, humidity = result
                        measurement = {
                            'timestamp': datetime.now(),
                            'temperature': temperature,
                            'humidity': humidity
                        }
                        self.data_queue.put(measurement)
                        self.log_message(f"온도: {temperature}°C, 습도: {humidity}%RH")
                        fail_count = 0  # 성공 시 실패 카운트 리셋
                    else:
                        self.log_message("온습도 측정 실패")
                        fail_count += 1
                    
                except Exception as e:
                    self.log_message(f"데이터 읽기 오류: {e}")
                    fail_count += 1
                    time.sleep(1)
                
                # 연속 3회 실패 시 센서 리셋 시도
                if fail_count >= 3:
                    self.log_message("연속 실패로 인한 센서 리셋 시도...")
                    try:
                        self.sensor.close()
                        time.sleep(0.1)
                        self.sensor = SimpleSHT40(bus=self.sensor_bus, address=self.sensor_address)
                        self.sensor.connect()
                        self.log_message("센서 리셋 성공")
                        fail_count = 0
                    except Exception as e:
                        self.log_message(f"센서 리셋 실패: {e}")
                
                # 연속 10회 실패 시 측정 중지
                if fail_count >= 10:
                    self.log_message("온습도 측정 10회 연속 실패, 측정 중지")
                    self.root.after(0, self.stop_monitoring)
                    break

                time.sleep(2)  # 2초 간격
            
            self.sensor.close()
            self.log_message("센서 연결 종료")
            
        except Exception as e:
            self.log_message(f"센서 루프 오류: {e}")
            if self.sensor:
                self.sensor.close()
    
    def get_comfort_status(self, temperature, humidity):
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
            return "쾌적", "#6A994E"
        elif temp_status in ["서늘", "따뜻"] and humidity_status in ["약간건조", "약간습함"]:
            return "양호", "#F18F01"
        elif temp_status in ["춥다", "덥다"] or humidity_status in ["건조", "습함"]:
            return "불쾌", "#E63946"
        else:
            return "보통", "#2D5BFF"
    
    def calculate_stats(self):
        """통계 계산"""
        if len(self.temperature_data) == 0:
            return None, None, None, None, None, None
        
        temp_values = list(self.temperature_data)
        humidity_values = list(self.humidity_data)
        
        temp_max = max(temp_values)
        temp_min = min(temp_values)
        temp_avg = sum(temp_values) / len(temp_values)
        
        humidity_max = max(humidity_values)
        humidity_min = min(humidity_values)
        humidity_avg = sum(humidity_values) / len(humidity_values)
        
        return temp_max, temp_min, temp_avg, humidity_max, humidity_min, humidity_avg
    
    def update_gui(self):
        """GUI 업데이트"""
        # 현재 시간 업데이트
        current_time = datetime.now().strftime("%H:%M:%S")
        self.time_label.config(text=current_time)
        
        # 센서 데이터 처리
        data_updated = False
        while not self.data_queue.empty():
            try:
                data = self.data_queue.get_nowait()
                
                # 데이터 저장
                self.timestamps.append(data['timestamp'])
                self.temperature_data.append(data['temperature'])
                self.humidity_data.append(data['humidity'])
                
                # 측정값 표시 업데이트
                self.temperature_value.config(text=f"{data['temperature']:.1f}")
                self.humidity_value.config(text=f"{data['humidity']:.1f}")
                
                # 환경 상태 업데이트
                status_text, status_color = self.get_comfort_status(data['temperature'], data['humidity'])
                self.comfort_status.config(text=status_text, foreground=status_color)
                
                # 통계 업데이트
                temp_max, temp_min, temp_avg, humidity_max, humidity_min, humidity_avg = self.calculate_stats()
                if temp_max is not None:
                    temp_stats_text = f"온도 통계\n최대: {temp_max:.1f} °C\n최소: {temp_min:.1f} °C\n평균: {temp_avg:.1f} °C"
                    self.temp_stats_label.config(text=temp_stats_text)
                    
                    humidity_stats_text = f"습도 통계\n최대: {humidity_max:.1f} %RH\n최소: {humidity_min:.1f} %RH\n평균: {humidity_avg:.1f} %RH"
                    self.humidity_stats_label.config(text=humidity_stats_text)
                
                data_updated = True
                
            except queue.Empty:
                break
        
        # 차트 업데이트
        if data_updated:
            self.update_chart()
        
        # 다음 업데이트 예약
        self.root.after(2000, self.update_gui)
    
    def update_chart(self):
        """차트 업데이트"""
        if len(self.timestamps) > 0:
            # 데이터 업데이트
            self.temp_line.set_data(self.timestamps, self.temperature_data)
            self.humidity_line.set_data(self.timestamps, self.humidity_data)
            
            # 축 범위 조정
            now = datetime.now()
            self.ax.set_xlim(now - timedelta(minutes=5), now)
            
            # Y축 범위 자동 조정
            if self.temperature_data and self.humidity_data:
                # 온도와 습도를 모두 고려한 Y축 범위 설정
                all_values = list(self.temperature_data) + list(self.humidity_data)
                max_val = max(all_values)
                min_val = min(all_values)
                
                # 범위를 조금 여유있게 설정
                range_val = max_val - min_val
                if range_val < 10:
                    range_val = 10
                
                self.ax.set_ylim(
                    max(0, min_val - range_val * 0.1), 
                    max_val + range_val * 0.1
                )
            
            # 시간 축 포맷 재설정
            self.ax.xaxis.set_major_formatter(mdates.DateFormatter('%H:%M'))
            self.ax.xaxis.set_major_locator(mdates.MinuteLocator(interval=1))
            
            # 차트 다시 그리기
            self.canvas.draw()
    
    def on_closing(self):
        """프로그램 종료 시 정리"""
        self.log_message("프로그램 종료")
        self.stop_monitoring()
        time.sleep(1)
        if self.sensor:
            self.sensor.close()
        self.root.destroy()
    
    def run(self):
        """프로그램 실행"""
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.root.mainloop()

if __name__ == "__main__":
    try:
        monitor = SHT40Monitor()
        monitor.run()
    except Exception as e:
        print(f"프로그램 시작 실패: {e}")
        # GUI 없이 기본 에러 메시지 표시
        try:
            import tkinter as tk
            from tkinter import messagebox
            root = tk.Tk()
            root.withdraw()
            messagebox.showerror("오류", f"프로그램 시작 실패:\n{e}\n\n필요한 라이브러리를 설치하세요:\npip install ttkbootstrap matplotlib smbus2")
            root.destroy()
        except:
            print("필요한 라이브러리를 설치하세요:")
            print("pip install ttkbootstrap matplotlib smbus2")