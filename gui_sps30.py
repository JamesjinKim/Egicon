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
    # 라즈베리파이에서 사용 가능한 폰트 설정
    plt.rcParams['font.family'] = 'DejaVu Sans'
    plt.rcParams['axes.unicode_minus'] = False
except:
    # 폰트 설정 실패 시 기본 폰트 사용
    plt.rcParams['font.family'] = 'sans-serif'

from datetime import datetime, timedelta
import threading
import time
import queue
import glob
from collections import deque

# SPS30 관련 imports
from shdlc_sps30 import Sps30ShdlcDevice
from sensirion_shdlc_driver import ShdlcSerialPort, ShdlcConnection
from sensirion_shdlc_driver.errors import ShdlcError

class SPS30Monitor:
    def __init__(self):
        # 데이터 저장용 deque (최대 60개 데이터 포인트 - 라즈베리파이 성능 고려)
        self.max_points = 60
        self.timestamps = deque(maxlen=self.max_points)
        self.pm1_data = deque(maxlen=self.max_points)
        self.pm25_data = deque(maxlen=self.max_points)
        self.pm4_data = deque(maxlen=self.max_points)
        self.pm10_data = deque(maxlen=self.max_points)
        
        # 센서 관련
        self.device = None
        self.port = None
        self.is_monitoring = False
        self.sensor_thread = None
        self.data_queue = queue.Queue()
        self.port_name = None
        self.sensor_ready = False
        
        # GUI 초기화 (최우선)
        self.setup_gui()
        
        # GUI 업데이트 타이머 시작
        self.update_gui()
        
        # 센서 검색을 별도 스레드에서 실행 (백그라운드)
        self.sensor_detection_thread = threading.Thread(target=self.background_sensor_detection, daemon=True)
        self.sensor_detection_thread.start()
        
    def find_serial_port(self):
        """라즈베리파이/Linux에서 USB 시리얼 포트 자동 탐색"""
        # 라즈베리파이에서 일반적인 USB 시리얼 포트 패턴
        candidates = glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyAMA*')
        if not candidates:
            return None
        return candidates[0]
    
    def setup_gui(self):
        """GUI 초기화 - 라즈베리파이 화면 크기에 맞게 조정"""
        self.root = tb.Window(
            title="SPS30 미세먼지 모니터",
            themename="solar",
            size=(800, 480),
            resizable=(False, False)  # 라즈베리파이에서는 고정 크기
        )
        self.root.geometry("800x480+0+0")
        
        # 메인 컨테이너 (패딩 축소)
        main_frame = tb.Frame(self.root, padding=8)
        main_frame.pack(fill=BOTH, expand=True)
        
        # 상단 헤더 (고정 높이)
        self.setup_header(main_frame)
        
        # 중앙 컨텐츠 (측정값 + 차트) - 높이 제한
        content_frame = tb.Frame(main_frame, height=300)  # 명시적 높이 설정
        content_frame.pack(fill=X, pady=(5, 0))
        content_frame.pack_propagate(False)  # 높이 고정
        
        # 측정값 표시 영역 (왼쪽)
        self.setup_measurement_panel(content_frame)
        
        # 차트 영역 (오른쪽)
        self.setup_chart_panel(content_frame)
        
        # 하단 상태바 (고정 높이)
        self.setup_status_bar(main_frame)
        
        # 디버그 로그 영역 (남은 공간 모두 사용)
        self.setup_debug_log(main_frame)
        
    def setup_header(self, parent):
        """상단 헤더 설정 - 라즈베리파이에 맞게 축소"""
        header_frame = tb.Frame(parent)
        header_frame.pack(fill=X, pady=(0, 3))  # 패딩 더 축소
        
        # 제목 (폰트 크기 축소)
        title_label = tb.Label(
            header_frame, 
            text="SPS30 미세먼지 모니터", 
            font=("DejaVu Sans", 16, "bold"),
            foreground="#2E86AB"
        )
        title_label.pack(side=LEFT)
        
        # 버튼 프레임
        button_frame = tb.Frame(header_frame)
        button_frame.pack(side=RIGHT)
        
        # 시작/중지 버튼 (크기 축소)
        self.start_button = tb.Button(
            button_frame,
            text="측정 시작",
            command=self.toggle_monitoring,
            style="success.TButton",
            width=10
        )
        self.start_button.pack(side=RIGHT, padx=(3, 0))  # 패딩 축소
        
        # 센서 상태 표시 (폰트 크기 축소)
        self.sensor_status = tb.Label(
            button_frame,
            text="센서 검색 중...",
            font=("DejaVu Sans", 10),
            foreground="#F18F01"
        )
        self.sensor_status.pack(side=RIGHT, padx=(0, 3))  # 패딩 축소
        
    def setup_measurement_panel(self, parent):
        """측정값 표시 패널 - 라즈베리파이 화면에 맞게 컴팩트하게"""
        measurement_frame = tb.LabelFrame(
            parent, 
            text="현재 측정값", 
            padding=8,  # 패딩 축소
            style="info.TLabelframe"
        )
        measurement_frame.pack(side=LEFT, fill=BOTH, expand=True, padx=(0, 3))
        
        # 작은 화면에 맞게 폰트 크기 조정
        label_font_size = 10
        value_font_size = 12
        unit_font_size = 8
        
        # PM2.5와 PM10을 위쪽에 큰 글씨로 (가장 중요)
        important_frame = tb.Frame(measurement_frame)
        important_frame.pack(fill=X, pady=(0, 5))  # 여백 축소
        
        # PM2.5 (왼쪽)
        pm25_frame = tb.Frame(important_frame)
        pm25_frame.pack(side=LEFT, fill=X, expand=True)
        
        tb.Label(pm25_frame, text="PM2.5", font=("DejaVu Sans", label_font_size, "bold")).pack()
        self.pm25_value = tb.Label(
            pm25_frame, 
            text="--", 
            font=("DejaVu Sans", value_font_size, "bold"),
            foreground="#E63946"
        )
        self.pm25_value.pack()
        tb.Label(pm25_frame, text="μg/m³", font=("DejaVu Sans", unit_font_size)).pack()
        
        # PM10 (오른쪽)
        pm10_frame = tb.Frame(important_frame)
        pm10_frame.pack(side=RIGHT, fill=X, expand=True)
        
        tb.Label(pm10_frame, text="PM10", font=("DejaVu Sans", label_font_size, "bold")).pack()
        self.pm10_value = tb.Label(
            pm10_frame, 
            text="--", 
            font=("DejaVu Sans", value_font_size, "bold"),
            foreground="#F77F00"
        )
        self.pm10_value.pack()
        tb.Label(pm10_frame, text="μg/m³", font=("DejaVu Sans", unit_font_size)).pack()
        
        # PM1.0과 PM4.0을 아래쪽에 작은 글씨로
        small_frame = tb.Frame(measurement_frame)
        small_frame.pack(fill=X, pady=(5, 5))  # 여백 축소
        
        # PM1.0 (왼쪽)
        pm1_frame = tb.Frame(small_frame)
        pm1_frame.pack(side=LEFT, fill=X, expand=True)
        
        tb.Label(pm1_frame, text="PM1.0", font=("DejaVu Sans", 8, "bold")).pack()
        self.pm1_value = tb.Label(
            pm1_frame, 
            text="--", 
            font=("DejaVu Sans", 12, "bold"),
            foreground="#FCBF49"
        )
        self.pm1_value.pack()
        tb.Label(pm1_frame, text="μg/m³", font=("DejaVu Sans", 8)).pack()
        
        # PM4.0 (오른쪽)
        pm4_frame = tb.Frame(small_frame)
        pm4_frame.pack(side=RIGHT, fill=X, expand=True)
        
        tb.Label(pm4_frame, text="PM4.0", font=("DejaVu Sans", 8, "bold")).pack()
        self.pm4_value = tb.Label(
            pm4_frame, 
            text="--", 
            font=("DejaVu Sans", 12, "bold"),
            foreground="#A663CC"
        )
        self.pm4_value.pack()
        tb.Label(pm4_frame, text="μg/m³", font=("DejaVu Sans", 8)).pack()
        
        # 공기질 상태
        self.air_quality_frame = tb.Frame(measurement_frame)
        self.air_quality_frame.pack(fill=X, pady=(5, 0))  # 여백 축소
        
        self.air_quality_label = tb.Label(
            self.air_quality_frame,
            text="공기질 상태",
            font=("DejaVu Sans", 10, "bold")
        )
        self.air_quality_label.pack()
        
        self.air_quality_status = tb.Label(
            self.air_quality_frame,
            text="측정 준비",
            font=("DejaVu Sans", 10, "bold"),
            foreground="#6A994E"
        )
        self.air_quality_status.pack()
        
    def setup_chart_panel(self, parent):
        """차트 패널 설정 - 라즈베리파이에 맞게 크기 조정"""
        chart_frame = tb.LabelFrame(
            parent, 
            text="실시간 차트", 
            padding=3,  # 패딩 더 축소
            style="info.TLabelframe"
        )
        chart_frame.pack(side=RIGHT, fill=BOTH, expand=True)
        
        # matplotlib 설정 (라즈베리파이 성능 고려, 크기 더 축소)
        plt.style.use('dark_background')
        self.fig = Figure(figsize=(5, 3), dpi=75, facecolor='#2E3440')  # 크기 더 축소
        self.ax = self.fig.add_subplot(111, facecolor='#2E3440')
        
        # 차트 스타일링 (폰트 크기 축소)
        self.ax.set_xlabel('Time', color='#D8DEE9', fontsize=9)
        self.ax.set_ylabel('μg/m³', color='#D8DEE9', fontsize=9)
        self.ax.tick_params(colors='#D8DEE9', labelsize=8)
        self.ax.grid(True, alpha=0.3, color='#4C566A')
        
        # 범례 설정 (얇은 선, 작은 마커)
        self.pm1_line, = self.ax.plot([], [], '-', color='#FCBF49', label='PM1.0', linewidth=1)
        self.pm25_line, = self.ax.plot([], [], '-', color='#E63946', label='PM2.5', linewidth=2)
        self.pm4_line, = self.ax.plot([], [], '-', color='#A663CC', label='PM4.0', linewidth=1)
        self.pm10_line, = self.ax.plot([], [], '-', color='#F77F00', label='PM10', linewidth=1)
        
        self.ax.legend(loc='upper left', framealpha=0.8, facecolor='#3B4252', fontsize=8)
        
        # 차트를 tkinter에 임베드
        self.canvas = FigureCanvasTkAgg(self.fig, chart_frame)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill=BOTH, expand=True)
        
        # 초기 차트 설정
        self.setup_chart()
        
    def setup_chart(self):
        """차트 초기 설정"""
        # 시간 범위 설정 (최근 5분 - 라즈베리파이 성능 고려)
        now = datetime.now()
        self.ax.set_xlim(now - timedelta(minutes=5), now)
        self.ax.set_ylim(0, 50)
        
        # 시간 축 포맷
        self.ax.xaxis.set_major_formatter(mdates.DateFormatter('%H:%M'))
        self.ax.xaxis.set_major_locator(mdates.MinuteLocator(interval=1))
        
    def setup_status_bar(self, parent):
        """하단 상태바 - 크기 축소"""
        status_frame = tb.Frame(parent)
        status_frame.pack(fill=X, pady=(5, 0))  # 패딩 축소
        
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
        """디버그 로그 영역 - 크기 확대"""
        debug_frame = tb.LabelFrame(parent, text="로그", padding=8)
        debug_frame.pack(fill=BOTH, expand=True, pady=(8, 0))  # expand=True로 남은 공간 모두 사용
        
        # 로그 텍스트 영역 (높이 확대)
        self.log_text = tk.Text(debug_frame, height=6, bg='#2E3440', fg='#D8DEE9', font=('monospace', 8))
        self.log_text.pack(fill=BOTH, expand=True)  # 남은 공간 모두 사용
        
    def log_message(self, message):
        """디버그 로그 메시지 추가"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        log_entry = f"[{timestamp}] {message}\n"
        
        self.log_text.insert(tk.END, log_entry)
        self.log_text.see(tk.END)
        
        # 로그가 너무 많아지면 오래된 것 삭제 (라즈베리파이 메모리 고려)
        lines = self.log_text.get("1.0", tk.END).split('\n')
        if len(lines) > 20:  # 라인 수 축소
            self.log_text.delete("1.0", "5.0")
        
        print(log_entry.strip())  # 콘솔에도 출력
        
    def background_sensor_detection(self):
        """백그라운드에서 센서 검색 및 연결 테스트"""
        try:
            # GUI 스레드에서 상태 업데이트
            self.root.after(0, lambda: self.sensor_status.config(text="센서 검색 중...", foreground="#F18F01"))
            
            # 센서 포트 검색
            self.port_name = self.find_serial_port()
            if not self.port_name:
                self.root.after(0, lambda: self.sensor_status.config(text="센서 없음", foreground="#E63946"))
                self.root.after(0, lambda: self.log_message("센서를 찾을 수 없습니다."))
                return
                
            self.root.after(0, lambda: self.log_message(f"센서 포트 발견: {self.port_name}"))
            
            # 센서 연결 테스트 (시간이 오래 걸리는 부분)
            self.root.after(0, lambda: self.sensor_status.config(text="연결 테스트 중...", foreground="#F18F01"))
            
            with ShdlcSerialPort(port=self.port_name, baudrate=115200) as port:
                device = Sps30ShdlcDevice(ShdlcConnection(port))
                
                # 센서 정보 확인
                serial_number = device.device_information_serial_number()
                self.sensor_ready = True
                
                # GUI 스레드에서 상태 업데이트
                self.root.after(0, lambda: self.sensor_status.config(
                    text=f"센서 연결됨 ({serial_number[:6]}...)", 
                    foreground="#6A994E"
                ))
                self.root.after(0, lambda: self.log_message(f"센서 연결 성공: {serial_number}"))
                
        except Exception as e:
            self.root.after(0, lambda: self.sensor_status.config(text="연결 실패", foreground="#E63946"))
            self.root.after(0, lambda: self.log_message(f"센서 검색 오류: {e}"))
    
    def toggle_monitoring(self):
        """측정 시작/중지"""
        if not self.port_name:
            messagebox.showerror("오류", "센서가 연결되지 않았습니다.")
            return
            
        if not self.sensor_ready:
            messagebox.showwarning("알림", "센서 연결을 확인 중입니다. 잠시 후 다시 시도해주세요.")
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
        device = None
        try:
            self.log_message("센서 연결 중...")
            
            with ShdlcSerialPort(port=self.port_name, baudrate=115200) as port:
                device = Sps30ShdlcDevice(ShdlcConnection(port))
                
                # 센서 초기화
                try:
                    device.device_reset()
                    self.log_message("센서 리셋 완료")
                    time.sleep(2)
                except Exception as e:
                    self.log_message(f"센서 리셋 실패: {e}")

                # 기본 정보 확인
                serial_number = device.device_information_serial_number()
                self.log_message(f"시리얼: {serial_number}")
                
                # 측정 시작
                device.start_measurement()
                self.log_message("센서 준비 중...")
                time.sleep(5)
                
                # 데이터 수집 루프
                while self.is_monitoring:
                    try:
                        data = device.read_measured_value()
                        if data and len(data) >= 3:  # 최소 3개 데이터면 충분
                            # 안전한 숫자 변환 함수
                            def safe_float(value):
                                try:
                                    if isinstance(value, (int, float)):
                                        return float(value)
                                    elif isinstance(value, str):
                                        return float(value)
                                    elif isinstance(value, tuple) and len(value) > 0:
                                        return float(value[0])  # 튜플의 첫 번째 값 사용
                                    elif hasattr(value, '__float__'):
                                        return float(value)
                                    else:
                                        return 0.0
                                except Exception:
                                    return 0.0
                            
                            timestamp = datetime.now()
                            pm1_val = safe_float(data[0])
                            pm25_val = safe_float(data[1])
                            pm10_val = safe_float(data[2])
                            
                            measurement = {
                                'timestamp': timestamp,
                                'pm1': pm1_val,
                                'pm25': pm25_val,
                                'pm4': 0.0,  # 3개 데이터인 경우 PM4.0 없음
                                'pm10': pm10_val
                            }
                            
                            # 데이터 개수에 따른 처리
                            if len(data) >= 4:
                                pm4_val = safe_float(data[2])
                                pm10_val = safe_float(data[3])
                                measurement['pm4'] = pm4_val
                                measurement['pm10'] = pm10_val
                                self.log_message(f"PM1.0={pm1_val:.1f} PM2.5={pm25_val:.1f} PM4.0={pm4_val:.1f} PM10={pm10_val:.1f}")
                            else:
                                # 3개 데이터: PM1.0, PM2.5, PM10
                                self.log_message(f"PM1.0={pm1_val:.1f} PM2.5={pm25_val:.1f} PM10={pm10_val:.1f}")
                            
                            self.data_queue.put(measurement)
                        else:
                            self.log_message(f"데이터 부족: 받은 개수={len(data) if data else 0}")
                        
                        time.sleep(3)  # 라즈베리파이 성능 고려하여 3초 간격
                    except Exception as e:
                        self.log_message(f"데이터 준비 중... ({e})")
                        time.sleep(1)
                
                # 측정 중지
                if device:
                    device.stop_measurement()
                    self.log_message("센서 측정 중지")
                    
        except Exception as e:
            self.log_message(f"센서 루프 오류: {e}")
            if device:
                try:
                    device.stop_measurement()
                except:
                    pass
    
    def get_air_quality_status(self, pm25_value):
        """PM2.5 값에 따른 공기질 상태 반환 (한국 기준)"""
        if pm25_value <= 15:
            return "좋음", "#6A994E"
        elif pm25_value <= 35:
            return "보통", "#F18F01"
        elif pm25_value <= 75:
            return "나쁨", "#F77F00"
        else:
            return "매우 나쁨", "#E63946"
    
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
                self.pm1_data.append(data['pm1'])
                self.pm25_data.append(data['pm25'])
                self.pm4_data.append(data['pm4'])
                self.pm10_data.append(data['pm10'])
                
                # 측정값 표시 업데이트
                self.pm1_value.config(text=f"{data['pm1']:.1f}")
                self.pm25_value.config(text=f"{data['pm25']:.1f}")
                self.pm10_value.config(text=f"{data['pm10']:.1f}")
                
                # PM4.0은 데이터가 있는 경우에만 표시
                if data['pm4'] > 0:
                    self.pm4_value.config(text=f"{data['pm4']:.1f}")
                else:
                    self.pm4_value.config(text="--")
                
                # 공기질 상태 업데이트
                status_text, status_color = self.get_air_quality_status(data['pm25'])
                self.air_quality_status.config(text=status_text, foreground=status_color)
                
                data_updated = True
                
            except queue.Empty:
                break
        
        # 차트 업데이트 (데이터가 업데이트된 경우에만)
        if data_updated:
            self.update_chart()
        
        # 다음 업데이트 예약 (라즈베리파이 성능 고려하여 2초 간격)
        self.root.after(2000, self.update_gui)
    
    def update_chart(self):
        """차트 업데이트"""
        if len(self.timestamps) > 0:
            # 데이터 업데이트
            self.pm1_line.set_data(self.timestamps, self.pm1_data)
            self.pm25_line.set_data(self.timestamps, self.pm25_data)
            self.pm10_line.set_data(self.timestamps, self.pm10_data)
            
            # PM4.0 데이터가 있는 경우에만 차트에 표시
            if any(val > 0 for val in self.pm4_data):
                self.pm4_line.set_data(self.timestamps, self.pm4_data)
                self.pm4_line.set_visible(True)
            else:
                # PM4.0 데이터가 없으면 라인 숨기기
                self.pm4_line.set_visible(False)
            
            # 축 범위 조정
            now = datetime.now()
            self.ax.set_xlim(now - timedelta(minutes=5), now)
            
            # Y축 범위 자동 조정 (0이 아닌 값들만 고려)
            if self.pm25_data:
                all_values = list(self.pm1_data) + list(self.pm25_data) + list(self.pm10_data)
                if any(val > 0 for val in self.pm4_data):
                    all_values.extend([val for val in self.pm4_data if val > 0])
                
                if all_values:
                    max_val = max(all_values)
                    self.ax.set_ylim(0, max(max_val * 1.1, 30))
            
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
        self.root.destroy()
    
    def run(self):
        """프로그램 실행"""
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.root.mainloop()

if __name__ == "__main__":
    try:
        monitor = SPS30Monitor()
        monitor.run()
    except KeyboardInterrupt:
        print("\n프로그램이 사용자에 의해 중단되었습니다.")
    finally:
        print("프로그램 종료")