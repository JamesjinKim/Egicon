#!/usr/bin/env python3
"""
BH1750 조도 센서 모니터
라즈베리파이 4용 GUI 프로그램 - 수정된 버전
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
import os

class SimpleBH1750:
    """BH1750 조도 센서 클래스 - 개선된 버전"""
    
    def __init__(self, bus=1, address=0x23):
        self.bus_num = bus
        self.address = address
        self.bus = None
        self.debug = True
    
    def log_debug(self, message):
        """디버그 로그"""
        if self.debug:
            print(f"[BH1750] {message}")
    
    def test_i2c_availability(self):
        """I2C 버스 사용 가능성 테스트"""
        try:
            # I2C 디바이스 파일 존재 확인
            i2c_device = f"/dev/i2c-{self.bus_num}"
            if not os.path.exists(i2c_device):
                raise Exception(f"I2C 디바이스 {i2c_device}가 존재하지 않습니다")
            
            # 읽기/쓰기 권한 확인
            if not os.access(i2c_device, os.R_OK | os.W_OK):
                raise Exception(f"I2C 디바이스 {i2c_device}에 대한 권한이 없습니다")
            
            self.log_debug(f"I2C 디바이스 {i2c_device} 사용 가능")
            return True
            
        except Exception as e:
            self.log_debug(f"I2C 가용성 테스트 실패: {e}")
            return False
    
    def connect(self):
        """센서 연결 - 개선된 버전"""
        try:
            # I2C 가용성 먼저 테스트
            if not self.test_i2c_availability():
                raise Exception("I2C 버스를 사용할 수 없습니다")
            
            # 기존 연결이 있으면 종료
            if self.bus:
                self.bus.close()
                self.bus = None
            
            self.log_debug(f"I2C 버스 {self.bus_num}, 주소 0x{self.address:02X} 연결 시도...")
            
            # SMBus 연결
            self.bus = smbus2.SMBus(self.bus_num)
            
            # 연결 테스트 (여러 방법 시도)
            connection_success = False
            
            # 방법 1: Power On 명령
            try:
                self.log_debug("Power On 명령 시도...")
                self.bus.write_byte(self.address, 0x01)  # Power On
                time.sleep(0.01)
                connection_success = True
                self.log_debug("Power On 명령 성공")
            except Exception as e:
                self.log_debug(f"Power On 명령 실패: {e}")
            
            # 방법 2: Reset 명령
            if not connection_success:
                try:
                    self.log_debug("Reset 명령 시도...")
                    self.bus.write_byte(self.address, 0x07)  # Reset
                    time.sleep(0.01)
                    connection_success = True
                    self.log_debug("Reset 명령 성공")
                except Exception as e:
                    self.log_debug(f"Reset 명령 실패: {e}")
            
            # 방법 3: 직접 측정 명령 (가장 확실한 방법)
            if not connection_success:
                try:
                    self.log_debug("직접 측정 명령 시도...")
                    self.bus.write_byte(self.address, 0x20)  # One Time H-Resolution Mode
                    time.sleep(0.15)  # 150ms 대기
                    
                    # 데이터 읽기 시도
                    data = self.bus.read_i2c_block_data(self.address, 0x20, 2)
                    if len(data) == 2:
                        connection_success = True
                        self.log_debug("직접 측정 명령 성공")
                except Exception as e:
                    self.log_debug(f"직접 측정 명령 실패: {e}")
            
            if not connection_success:
                raise Exception("모든 연결 방법 실패")
            
            self.log_debug("BH1750 센서 연결 성공")
            return True
            
        except Exception as e:
            self.log_debug(f"센서 연결 실패: {e}")
            if self.bus:
                self.bus.close()
                self.bus = None
            raise e
    
    def read_light_safe(self):
        """안전한 조도값 읽기 (다양한 방법 시도)"""
        if not self.bus:
            raise Exception("센서가 연결되지 않음")
        
        methods = [
            ("One Time H-Resolution", 0x20, 0.15),
            ("One Time H-Resolution2", 0x21, 0.15),
            ("One Time L-Resolution", 0x23, 0.02),
            ("Continuously H-Resolution", 0x10, 0.15)
        ]
        
        for method_name, command, wait_time in methods:
            try:
                #self.log_debug(f"{method_name} 방식으로 측정 시도...")
                
                # 측정 명령 전송
                self.bus.write_byte(self.address, command)
                time.sleep(wait_time)
                
                # 데이터 읽기 방법 1: read_i2c_block_data
                try:
                    data = self.bus.read_i2c_block_data(self.address, command, 2)
                    #self.log_debug(f"read_i2c_block_data 성공: {[f'0x{b:02X}' for b in data]}")
                except:
                    # 데이터 읽기 방법 2: 개별 read_byte
                    self.log_debug("read_i2c_block_data 실패, 개별 read_byte 시도...")
                    data = []
                    for _ in range(2):
                        byte_val = self.bus.read_byte(self.address)
                        data.append(byte_val)
                        time.sleep(0.001)
                    self.log_debug(f"개별 read_byte 성공: {[f'0x{b:02X}' for b in data]}")
                
                if len(data) >= 2:
                    # 조도값 계산
                    raw_value = (data[0] << 8) | data[1]
                    
                    # BH1750 조도 계산 공식
                    if command in [0x20, 0x21]:  # High resolution
                        lux = raw_value / 1.2
                    else:  # Low resolution
                        lux = raw_value / 1.2
                    
                    # 합리적인 범위 체크
                    if 0 <= lux <= 65535:
                        #self.log_debug(f"{method_name} 측정 성공: {lux:.1f} lux (원시값: 0x{raw_value:04X})")
                        return round(lux, 1)
                    else:
                        self.log_debug(f"측정값이 범위를 벗어남: {lux}")
                        continue
                        
            except Exception as e:
                self.log_debug(f"{method_name} 방식 실패: {e}")
                continue
        
        raise Exception("모든 측정 방법 실패")
    
    def read_light(self):
        """조도값 읽기 (호환성을 위한 래퍼)"""
        return self.read_light_safe()
    
    def close(self):
        """연연결 종료"""
        if self.bus:
            try:
                # Power Down 명령 전송
                self.bus.write_byte(self.address, 0x00)
                self.log_debug("Power Down 명령 전송")
            except:
                pass
            
            self.bus.close()
            self.bus = None
            self.log_debug("센서 연결 종료")

class BH1750Monitor:
    def __init__(self):
        # 데이터 저장용 deque (최대 60개 데이터 포인트)
        self.max_points = 60
        self.timestamps = deque(maxlen=self.max_points)
        self.light_data = deque(maxlen=self.max_points)
        
        # 센서 관련
        self.sensor = None
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
            title="BH1750 조도 센서 모니터 (개선된 버전)",
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
            text="BH1750 조도 센서 모니터 (개선됨)", 
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
        
        # 조도값 표시 (중앙 큰 글씨)
        light_frame = tb.Frame(measurement_frame)
        light_frame.pack(expand=True, fill=BOTH)
        
        # 조도값 레이블
        tb.Label(
            light_frame, 
            text="조도", 
            font=("DejaVu Sans", 14, "bold")
        ).pack(pady=(20, 5))
        
        self.light_value = tb.Label(
            light_frame, 
            text="--", 
            font=("DejaVu Sans", 36, "bold"),
            foreground="#F18F01"
        )
        self.light_value.pack(pady=10)
        
        tb.Label(
            light_frame, 
            text="lux", 
            font=("DejaVu Sans", 12)
        ).pack()
        
        # 조도 레벨 상태
        self.light_level_frame = tb.Frame(light_frame)
        self.light_level_frame.pack(fill=X, pady=(20, 0))
        
        self.light_level_label = tb.Label(
            self.light_level_frame,
            text="조도 레벨",
            font=("DejaVu Sans", 12, "bold")
        )
        self.light_level_label.pack()
        
        self.light_level_status = tb.Label(
            self.light_level_frame,
            text="측정 준비",
            font=("DejaVu Sans", 14, "bold"),
            foreground="#6A994E"
        )
        self.light_level_status.pack()
        
        # 통계 정보
        stats_frame = tb.Frame(measurement_frame)
        stats_frame.pack(fill=X, pady=(10, 0))
        
        # 최대값/최소값 표시
        self.stats_label = tb.Label(
            stats_frame,
            text="최대: -- lux\n최소: -- lux\n평균: -- lux",
            font=("DejaVu Sans", 9),
            justify="left"
        )
        self.stats_label.pack()
    
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
        self.ax.set_ylabel('조도 (lux)', color='#D8DEE9', fontsize=9)
        self.ax.tick_params(colors='#D8DEE9', labelsize=8)
        self.ax.grid(True, alpha=0.3, color='#4C566A')
        
        # 조도 라인
        self.light_line, = self.ax.plot([], [], '-', color='#F18F01', label='조도', linewidth=2)
        
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
        self.ax.set_ylim(0, 1000)
        
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
            
            # 다양한 I2C 설정으로 시도
            test_configs = [
                (1, 0x23),  # 기본 설정
                (0, 0x23),  # 버스 0
                (1, 0x5C),  # ADDR 핀 HIGH
                (0, 0x5C),  # 버스 0, ADDR 핀 HIGH
            ]
            
            sensor_found = False
            
            for bus_num, addr in test_configs:
                try:
                    self.root.after(0, lambda b=bus_num, a=addr: self.log_message(f"I2C 버스 {b}, 주소 {hex(a)} 테스트 중..."))
                    
                    # 센서 연결 테스트
                    test_sensor = SimpleBH1750(bus=bus_num, address=addr)
                    test_sensor.connect()
                    
                    # 측정 테스트
                    light_val = test_sensor.read_light()
                    test_sensor.close()
                    
                    if light_val is not None:
                        self.sensor = SimpleBH1750(bus=bus_num, address=addr)
                        self.sensor_ready = True
                        sensor_found = True
                        
                        self.root.after(0, lambda b=bus_num, a=addr: self.sensor_status.config(
                            text=f"센서 연결됨 (버스{b}:{hex(a)})", 
                            foreground="#6A994E"
                        ))
                        self.root.after(0, lambda b=bus_num, a=addr, v=light_val: self.log_message(f"BH1750 센서 연결 성공: 버스 {b}, 주소 {hex(a)}, 초기값 {v} lux"))
                        break
                        
                except Exception as e:
                    self.root.after(0, lambda b=bus_num, a=addr, err=str(e): self.log_message(f"버스 {b}, 주소 {hex(a)} 연결 실패: {err}"))
                    continue
            
            if not sensor_found:
                self.root.after(0, lambda: self.sensor_status.config(text="센서 없음", foreground="#E63946"))
                self.root.after(0, lambda: self.log_message("BH1750 센서를 찾을 수 없습니다. I2C가 활성화되어 있고 센서가 올바르게 연결되어 있는지 확인하세요."))
                
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
        test_configs = [
            (1, 0x23),  # 기본 설정
            (0, 0x23),  # 버스 0
            (1, 0x5C),  # ADDR 핀 HIGH
            (0, 0x5C),  # 버스 0, ADDR 핀 HIGH
        ]
        
        for bus_num, addr in test_configs:
            try:
                self.log_message(f"I2C 버스 {bus_num}, 주소 {hex(addr)} 테스트 중...")
                test_sensor = SimpleBH1750(bus=bus_num, address=addr)
                test_sensor.connect()
                light_val = test_sensor.read_light()
                test_sensor.close()
                
                if light_val is not None:
                    self.sensor = SimpleBH1750(bus=bus_num, address=addr)
                    self.sensor_ready = True
                    self.log_message(f"센서 연결 성공: 버스 {bus_num}, 주소 {hex(addr)}, 초기값 {light_val} lux")
                    return True
                    
            except Exception as e:
                self.log_message(f"버스 {bus_num}, 주소 {hex(addr)} 연결 실패: {e}")
                continue
        
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
                    light_value = self.sensor.read_light()
                    
                    if light_value is not None:
                        measurement = {
                            'timestamp': datetime.now(),
                            'light': light_value
                        }
                        self.data_queue.put(measurement)
                        self.log_message(f"조도: {light_value} lux")
                        fail_count = 0  # 성공 시 실패 카운트 초기화
                    else:
                        self.log_message("조도 측정 실패")
                        fail_count += 1
                    
                except Exception as e:
                    self.log_message(f"데이터 읽기 오류: {e}")
                    fail_count += 1
                    time.sleep(1)
                
                if fail_count >= 5:  # 10회에서 5회로 줄임
                    self.log_message("조도 측정 5회 연속 실패, 측정 중지")
                    self.root.after(0, self.stop_monitoring)
                    break

                time.sleep(2)  # 2초 간격
            
            self.sensor.close()
            self.log_message("센서 연결 종료")
            
        except Exception as e:
            self.log_message(f"센서 루프 오류: {e}")
            if self.sensor:
                self.sensor.close()
    
    def get_light_level_status(self, light_value):
        """조도값에 따른 상태 반환"""
        if light_value < 1:
            return "매우 어두움", "#2C2C54"
        elif light_value < 10:
            return "어두움", "#40407A"
        elif light_value < 50:
            return "희미함", "#706FD3"
        elif light_value < 200:
            return "실내조명", "#F18F01"
        elif light_value < 500:
            return "밝은실내", "#F77F00"
        elif light_value < 1000:
            return "흐린날", "#FCBF49"
        elif light_value < 10000:
            return "맑은날", "#F7DC6F"
        else:
            return "매우밝음", "#F4D03F"
    
    def calculate_stats(self):
        """통계 계산"""
        if len(self.light_data) == 0:
            return None, None, None
        
        light_values = list(self.light_data)
        return max(light_values), min(light_values), sum(light_values) / len(light_values)
    
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
                self.light_data.append(data['light'])
                
                # 측정값 표시 업데이트
                self.light_value.config(text=f"{data['light']:.1f}")
                
                # 조도 레벨 상태 업데이트
                status_text, status_color = self.get_light_level_status(data['light'])
                self.light_level_status.config(text=status_text, foreground=status_color)
                
                # 통계 업데이트
                max_val, min_val, avg_val = self.calculate_stats()
                if max_val is not None:
                    stats_text = f"최대: {max_val:.1f} lux\n최소: {min_val:.1f} lux\n평균: {avg_val:.1f} lux"
                    self.stats_label.config(text=stats_text)
                
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
            self.light_line.set_data(self.timestamps, self.light_data)
            
            # 축 범위 조정
            now = datetime.now()
            self.ax.set_xlim(now - timedelta(minutes=5), now)
            
            # Y축 범위 자동 조정
            if self.light_data:
                max_val = max(self.light_data)
                min_val = min(self.light_data)
                
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
        monitor = BH1750Monitor()
        monitor.run()
    except KeyboardInterrupt:
        print("\n프로그램이 사용자에 의해 중단되었습니다.")
    finally:
        print("프로그램 종료")