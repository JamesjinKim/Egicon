#!/usr/bin/env python3
"""
SDP810 차압센서 모니터
라즈베리파이 4용 GUI 프로그램
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
import struct
import subprocess
import logging

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

class SimpleSDP810:
    """SDP810 차압센서 클래스"""
    
    SDP810_ADDRESS = 0x25
    
    def __init__(self, bus=1, address=SDP810_ADDRESS):
        self.bus_num = bus
        self.address = address
        self.bus = None
        self.is_connected = False
    
    def _calculate_crc8(self, data):
        """CRC-8 계산 (Sensirion 표준)"""
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
    
    def connect(self):
        """센서 연결 및 초기화"""
        try:
            self.bus = smbus2.SMBus(self.bus_num)
            
            # 센서 응답 테스트
            self.bus.read_byte(self.address)
            
            # 압력 읽기 테스트
            pressure, crc_ok, message = self._read_pressure_data()
            if pressure is not None:
                self.is_connected = True
                logger.info(f"SDP810 센서 연결 성공 (압력: {pressure:.2f} Pa)")
                return True
            else:
                logger.error(f"SDP810 센서 통신 실패: {message}")
                return False
                
        except Exception as e:
            logger.error(f"SDP810 센서 연결 실패: {e}")
            return False
    
    def _read_pressure_data(self):
        """SDP810 압력 데이터 읽기"""
        try:
            # 3바이트 읽기: [pressure_msb, pressure_lsb, crc]
            read_msg = smbus2.i2c_msg.read(self.address, 3)
            self.bus.i2c_rdwr(read_msg)
            raw_data = list(read_msg)
            
            if len(raw_data) != 3:
                return None, False, f"데이터 길이 오류: {len(raw_data)}"
            
            pressure_msb = raw_data[0]
            pressure_lsb = raw_data[1]
            received_crc = raw_data[2]
            
            # CRC 검증
            calculated_crc = self._calculate_crc8([pressure_msb, pressure_lsb])
            crc_ok = calculated_crc == received_crc
            
            # 압력 계산 (SDP810-500Pa: Scale Factor = 60)
            raw_pressure = struct.unpack('>h', bytes([pressure_msb, pressure_lsb]))[0]
            pressure_pa = raw_pressure / 60.0
            
            # 범위 제한 (±500 Pa)
            pressure_pa = max(-500.0, min(500.0, pressure_pa))
            
            return pressure_pa, crc_ok, "OK"
            
        except Exception as e:
            return None, False, f"읽기 오류: {e}"
    
    def read_pressure_with_retry(self, max_retries=3):
        """CRC 오류 시 재시도하는 압력 읽기"""
        if not self.is_connected:
            return None
        
        for attempt in range(max_retries):
            pressure, crc_ok, message = self._read_pressure_data()
            
            if pressure is not None and crc_ok:
                return pressure
            else:
                if attempt < max_retries - 1:
                    time.sleep(0.05)
                    continue
        
        return None
    
    def close(self):
        """연결 해제"""
        if self.bus:
            try:
                self.bus.close()
            except Exception as e:
                logger.error(f"연결 해제 중 오류: {e}")
            finally:
                self.bus = None
                self.is_connected = False

def scan_i2c_bus():
    """I2C 버스에서 SDP810 센서 검색 (0x25 주소)"""
    found_sensor = False
    found_bus = None
    found_address = 0x25  # SDP810 고정 주소
    
    # 버스 0과 1 모두 스캔
    for bus_number in [0, 1]:
        try:
            logger.info(f"I2C 버스 {bus_number} 스캔 중...")
            result = subprocess.run(['i2cdetect', '-y', str(bus_number)], 
                                  capture_output=True, text=True, check=True)
            
            # SDP810 주소(0x25) 확인
            if "25" in result.stdout:
                logger.info(f"SDP810 센서가 주소 0x25에서 발견됨 (버스 {bus_number})")
                found_sensor = True
                found_bus = bus_number
                return found_sensor, found_bus, found_address
                
        except Exception as e:
            logger.warning(f"I2C 버스 {bus_number} 스캔 실패: {e}")
    
    if not found_sensor:
        logger.warning("어떤 I2C 버스에서도 SDP810 센서를 찾을 수 없음")
        return False, None, None
    
    return found_sensor, found_bus, found_address

class SDP810Monitor:
    def __init__(self):
        # 데이터 저장용 deque (최대 480개 데이터 포인트: 0.5초 간격으로 4분)
        self.max_points = 480
        self.timestamps = deque(maxlen=self.max_points)
        self.pressure_data = deque(maxlen=self.max_points)
        
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
            title="SDP810 차압센서 모니터",
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
            text="SDP810 차압센서 모니터", 
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
        
        # 차압 표시 영역
        pressure_frame = tb.Frame(measurement_frame)
        pressure_frame.pack(fill=X, pady=(0, 10))
        
        tb.Label(
            pressure_frame, 
            text="차압", 
            font=("DejaVu Sans", 12, "bold")
        ).pack(side=LEFT)
        
        self.pressure_value = tb.Label(
            pressure_frame, 
            text="--", 
            font=("DejaVu Sans", 24, "bold"),
            foreground="#2D5BFF"
        )
        self.pressure_value.pack(side=LEFT, padx=(10, 5))
        
        tb.Label(
            pressure_frame, 
            text="Pa", 
            font=("DejaVu Sans", 12)
        ).pack(side=LEFT)
        
        # 방향 표시
        self.direction_frame = tb.Frame(measurement_frame)
        self.direction_frame.pack(fill=X, pady=(0, 15))
        
        self.direction_label = tb.Label(
            self.direction_frame,
            text="방향",
            font=("DejaVu Sans", 11, "bold")
        )
        self.direction_label.pack()
        
        self.direction_status = tb.Label(
            self.direction_frame,
            text="측정 준비",
            font=("DejaVu Sans", 12, "bold"),
            foreground="#6A994E"
        )
        self.direction_status.pack()
        
        # 센서 정보
        info_frame = tb.Frame(measurement_frame)
        info_frame.pack(fill=X, pady=(10, 0))
        
        tb.Label(
            info_frame,
            text="SDP810-500Pa",
            font=("DejaVu Sans", 9, "bold")
        ).pack()
        
        tb.Label(
            info_frame,
            text="측정 범위: ±500 Pa",
            font=("DejaVu Sans", 8)
        ).pack()
        
        tb.Label(
            info_frame,
            text="정확도: ±1.5%",
            font=("DejaVu Sans", 8)
        ).pack()
        
        # 통계 정보
        stats_frame = tb.Frame(measurement_frame)
        stats_frame.pack(fill=X, pady=(15, 0))
        
        self.pressure_stats_label = tb.Label(
            stats_frame,
            text="차압 통계\n최대: -- Pa\n최소: -- Pa\n평균: -- Pa",
            font=("DejaVu Sans", 8),
            justify="left"
        )
        self.pressure_stats_label.pack(anchor=W)
    
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
        self.ax.set_xlabel('Time', color='#D8DEE9', fontsize=9)
        self.ax.set_ylabel('Pressure(Pa)', color='#D8DEE9', fontsize=9)
        self.ax.tick_params(colors='#D8DEE9', labelsize=8)
        self.ax.grid(True, alpha=0.3, color='#4C566A')
        
        # 차압 라인
        self.pressure_line, = self.ax.plot([], [], '-', color='#2D5BFF', label='Pressure(Pa)', linewidth=2)
        
        # 0 기준선
        self.ax.axhline(y=0, color='#E63946', linestyle='--', linewidth=1, alpha=0.5)
        
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
        self.ax.set_xlim(now - timedelta(minutes=4), now)
        self.ax.set_ylim(-50, 50)
        
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
            self.root.after(0, lambda: self.sensor_status.config(text="센서 검색 중...", foreground="#F18F01"))
            self.log_message("I2C 버스 스캔 시작 (0x25 주소)")
            
            found_sensor, found_bus, found_address = scan_i2c_bus()
            
            if found_sensor:
                self.log_message(f"SDP810 센서 발견: 버스 {found_bus}, 주소 0x{found_address:02X}")
                
                try:
                    test_sensor = SimpleSDP810(bus=found_bus, address=found_address)
                    test_sensor.connect()
                    
                    pressure_val = test_sensor.read_pressure_with_retry(max_retries=3)
                    test_sensor.close()
                    
                    if pressure_val is not None:
                        self.sensor_bus = found_bus
                        self.sensor_address = found_address
                        self.sensor = SimpleSDP810(bus=found_bus, address=found_address)
                        self.sensor_ready = True
                        
                        self.root.after(0, lambda: self.sensor_status.config(
                            text=f"센서 연결됨 (버스 {found_bus})", 
                            foreground="#6A994E"
                        ))
                        self.root.after(0, lambda: self.log_message(
                            f"SDP810 센서 연결 성공: 버스 {found_bus}, 주소 0x{found_address:02X}, "
                            f"차압 {pressure_val:.2f} Pa"
                        ))
                except Exception as e:
                    self.root.after(0, lambda err=e: self.log_message(f"센서 테스트 연결 실패: {err}"))
                    self.root.after(0, lambda: self.sensor_status.config(text="센서 연결 실패", foreground="#E63946"))
            else:
                self.root.after(0, lambda: self.sensor_status.config(text="센서 없음", foreground="#E63946"))
                self.root.after(0, lambda: self.log_message("SDP810 센서를 찾을 수 없습니다."))
                
        except Exception as e:
            self.root.after(0, lambda: self.sensor_status.config(text="연결 실패", foreground="#E63946"))
            self.root.after(0, lambda: self.log_message(f"센서 검색 오류: {e}"))
    
    def toggle_monitoring(self):
        """측정 시작/중지"""
        if not self.sensor_ready:
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
        """센서 검색 및 연결 테스트(동기)"""
        try:
            found_sensor, found_bus, found_address = scan_i2c_bus()
            
            if found_sensor:
                try:
                    test_sensor = SimpleSDP810(bus=found_bus, address=found_address)
                    test_sensor.connect()
                    pressure_val = test_sensor.read_pressure_with_retry(max_retries=3)
                    test_sensor.close()
                    
                    if pressure_val is not None:
                        self.sensor_bus = found_bus
                        self.sensor_address = found_address
                        self.sensor = SimpleSDP810(bus=found_bus, address=found_address)
                        self.sensor_ready = True
                        self.log_message(f"센서 연결 성공: 버스 {found_bus}, 차압 {pressure_val:.2f} Pa")
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
            
            fail_count = 0
            
            # 데이터 수집 루프
            while self.is_monitoring:
                try:
                    pressure = self.sensor.read_pressure_with_retry(max_retries=3)
                    
                    if pressure is not None:
                        measurement = {
                            'timestamp': datetime.now(),
                            'pressure': pressure
                        }
                        self.data_queue.put(measurement)
                        self.log_message(f"차압: {pressure:+.2f} Pa")
                        fail_count = 0
                    else:
                        self.log_message("차압 측정 실패")
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
                        self.sensor = SimpleSDP810(bus=self.sensor_bus, address=self.sensor_address)
                        self.sensor.connect()
                        self.log_message("센서 리셋 성공")
                        fail_count = 0
                    except Exception as e:
                        self.log_message(f"센서 리셋 실패: {e}")
                
                # 연속 10회 실패 시 측정 중지
                if fail_count >= 10:
                    self.log_message("차압 측정 10회 연속 실패, 측정 중지")
                    self.root.after(0, self.stop_monitoring)
                    break

                time.sleep(0.5)  # 0.5초 간격
            
            self.sensor.close()
            self.log_message("센서 연결 종료")
            
        except Exception as e:
            self.log_message(f"센서 루프 오류: {e}")
            if self.sensor:
                self.sensor.close()
    
    def get_direction_status(self, pressure):
        """차압값에 따른 방향 상태 반환"""
        if pressure >= 5:
            return "흡기(P1)", "#2D5BFF"
        elif pressure <= -5:
            return "배기(P2)", "#E63946"
        else:
            return "균형", "#6A994E"
    
    def calculate_stats(self):
        """통계 계산"""
        if len(self.pressure_data) == 0:
            return None, None, None
        
        pressure_values = list(self.pressure_data)
        
        pressure_max = max(pressure_values)
        pressure_min = min(pressure_values)
        pressure_avg = sum(pressure_values) / len(pressure_values)
        
        return pressure_max, pressure_min, pressure_avg
    
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
                self.pressure_data.append(data['pressure'])
                
                # 측정값 표시 업데이트
                self.pressure_value.config(text=f"{data['pressure']:+.2f}")
                
                # 방향 상태 업데이트
                status_text, status_color = self.get_direction_status(data['pressure'])
                self.direction_status.config(text=status_text, foreground=status_color)
                
                # 통계 업데이트
                pressure_max, pressure_min, pressure_avg = self.calculate_stats()
                if pressure_max is not None:
                    pressure_stats_text = f"차압 통계\n최대: {pressure_max:+.2f} Pa\n최소: {pressure_min:+.2f} Pa\n평균: {pressure_avg:+.2f} Pa"
                    self.pressure_stats_label.config(text=pressure_stats_text)
                
                data_updated = True
                
            except queue.Empty:
                break
        
        # 차트 업데이트
        if data_updated:
            self.update_chart()
        
        # 다음 업데이트 예약
        self.root.after(1000, self.update_gui)
    
    def update_chart(self):
        """차트 업데이트"""
        if len(self.timestamps) > 0:
            # 데이터 업데이트
            self.pressure_line.set_data(self.timestamps, self.pressure_data)
            
            # 축 범위 조정
            now = datetime.now()
            self.ax.set_xlim(now - timedelta(minutes=4), now)
            
            # Y축 범위 자동 조정
            if self.pressure_data:
                max_val = max(self.pressure_data)
                min_val = min(self.pressure_data)
                
                # 범위를 조금 여유있게 설정
                range_val = max_val - min_val
                if range_val < 20:
                    range_val = 20
                
                self.ax.set_ylim(
                    min_val - range_val * 0.2, 
                    max_val + range_val * 0.2
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
        monitor = SDP810Monitor()
        monitor.run()
    except Exception as e:
        print(f"프로그램 시작 실패: {e}")
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