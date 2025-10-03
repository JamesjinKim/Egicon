#!/usr/bin/env python3
"""
I2C 스캐너 GUI 애플리케이션 (간소화 버전)
자동으로 I2C 버스 0과 1을 스캔
SDP810 차압 센서 지원 추가
"""
import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext
import ttkbootstrap as ttkb
from ttkbootstrap.constants import *
import threading
import time
import smbus2
import signal
import sys
import struct

class I2CScanner:
    """I2C 스캐너 백엔드 클래스"""
    
    def __init__(self):
        self.buses = {}
        self.tca9548a_addresses = []
        # 인터럽트 핸들러 등록
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
    
    def connect_buses(self):
        """I2C 버스 0과 1에 연결"""
        # 기존 연결 정리
        for bus in self.buses.values():
            try:
                bus.close()
            except:
                pass
        
        self.buses = {}
        connected_buses = []
        
        for bus_num in [0, 1]:
            try:
                print(f"I2C 버스 {bus_num} 연결 시도...")
                bus = smbus2.SMBus(bus_num)
                self.buses[bus_num] = bus
                connected_buses.append(bus_num)
                print(f"I2C 버스 {bus_num} 연결 성공")
            except Exception as e:
                print(f"I2C 버스 {bus_num} 연결 실패: {e}")
        
        return connected_buses
    
    def _calculate_crc8_sdp810(self, data):
        """SDP810용 CRC-8 계산 (다항식 0x31)"""
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
    
    def test_sdp810(self, bus, addr):
        """SDP810 차압 센서 테스트"""
        try:
            print(f"SDP810 테스트 시작: 주소 0x{addr:02X}")
            
            # SDP810은 저수준 I2C 통신 사용
            read_msg = smbus2.i2c_msg.read(addr, 3)
            bus.i2c_rdwr(read_msg)
            raw_data = list(read_msg)
            
            if len(raw_data) != 3:
                print(f"  데이터 길이 오류: {len(raw_data)}바이트")
                return False
            
            pressure_msb = raw_data[0]
            pressure_lsb = raw_data[1]
            received_crc = raw_data[2]
            
            # CRC-8 검증
            calculated_crc = self._calculate_crc8_sdp810([pressure_msb, pressure_lsb])
            crc_ok = calculated_crc == received_crc
            
            # 압력값 계산
            raw_pressure = struct.unpack('>h', bytes([pressure_msb, pressure_lsb]))[0]
            pressure_pa = raw_pressure / 60.0
            
            print(f"  압력 데이터: MSB=0x{pressure_msb:02X}, LSB=0x{pressure_lsb:02X}")
            print(f"  CRC: 계산=0x{calculated_crc:02X}, 수신=0x{received_crc:02X}, 검증={'OK' if crc_ok else 'FAIL'}")
            print(f"  압력값: {pressure_pa:.1f} Pa")
            
            return True
            
        except Exception as e:
            print(f"  SDP810 테스트 실패: {e}")
            return False
    
    def scan_bus(self, bus_number, progress_callback=None):
        """특정 버스 스캔 - 다양한 I2C 통신 방식으로 시도"""
        if bus_number not in self.buses:
            return []
        
        devices = []
        bus = self.buses[bus_number]
        total = 0x77 - 0x08 + 1
        
        print(f"버스 {bus_number} 스캔 시작...")
        
        for i, addr in enumerate(range(0x08, 0x78)):
            device_found = False
            
            # SDP810 차압센서 특화 테스트 (0x25 주소)
            if addr == 0x25:
                if self.test_sdp810(bus, addr):
                    devices.append(addr)
                    device_found = True
                    print(f"버스 {bus_number}에서 SDP810 발견: 0x{addr:02X}")
            
            # 방법 1: read_byte() 시도 (SDP810이 아닌 경우)
            if not device_found:
                try:
                    bus.read_byte(addr)
                    devices.append(addr)
                    device_found = True
                    print(f"버스 {bus_number}에서 디바이스 발견 (read_byte): 0x{addr:02X}")
                except OSError as e:
                    if e.errno == 16:  # Device busy - 실제로는 디바이스 존재
                        devices.append(addr)
                        device_found = True
                        print(f"버스 {bus_number}에서 디바이스 발견 (busy): 0x{addr:02X}")
                    elif e.errno in [5, 121]:  # I/O error, Remote I/O error - 디바이스 없음
                        pass  # 정상적인 상황이므로 로그 출력하지 않음
                    else:
                        # 예상하지 못한 에러만 출력
                        print(f"  예상치 못한 OSError {e.errno} at 0x{addr:02X}: {e}")
                except Exception as e:
                    # 기타 예외도 로그 출력하지 않음 (정상적인 디바이스 없음 상황)
                    pass
            
            # 방법 2: SHT40 특화 테스트 (0x44, 0x45 주소)
            if not device_found and addr in [0x44, 0x45]:
                # SHT40 시리얼 번호 읽기 시도
                try:
                    bus.write_byte(addr, 0x89)  # 시리얼 번호 읽기 명령
                    time.sleep(0.001)
                    data = bus.read_i2c_block_data(addr, 0x89, 6)
                    devices.append(addr)
                    device_found = True
                    print(f"버스 {bus_number}에서 SHT40 발견 (시리얼 번호): 0x{addr:02X}")
                except:
                    pass
                
                # 위에서 실패했으면 측정 명령 시도
                if not device_found:
                    try:
                        bus.write_byte(addr, 0xFD)  # 고정밀 측정 명령
                        time.sleep(0.01)
                        data = bus.read_i2c_block_data(addr, 0xFD, 6)
                        devices.append(addr)
                        device_found = True
                        print(f"버스 {bus_number}에서 SHT40 발견 (측정 명령): 0x{addr:02X}")
                    except:
                        pass
                
                # 그래도 실패했으면 소프트 리셋 시도
                if not device_found:
                    try:
                        bus.write_byte(addr, 0x94)  # 소프트 리셋
                        time.sleep(0.001)
                        # 리셋 후에는 응답만 확인
                        devices.append(addr)
                        device_found = True
                        print(f"버스 {bus_number}에서 SHT40 발견 (소프트 리셋): 0x{addr:02X}")
                    except:
                        pass
            
            # 방법 3: 일반적인 레지스터 읽기 시도 (다른 주소들)
            if not device_found:
                common_registers = [0x00, 0x01, 0x0F, 0xD0, 0x75]  # 일반적인 ID 레지스터들
                for reg in common_registers:
                    try:
                        bus.read_byte_data(addr, reg)
                        devices.append(addr)
                        device_found = True
                        print(f"버스 {bus_number}에서 디바이스 발견 (레지스터 0x{reg:02X}): 0x{addr:02X}")
                        break
                    except:
                        continue
            
            if progress_callback:
                base_progress = 50 if bus_number == 1 else 0
                current_progress = int((i + 1) / total * 50)
                progress_callback(base_progress + current_progress)
        
        print(f"버스 {bus_number} 스캔 완료: {len(devices)}개 발견 {[f'0x{addr:02X}' for addr in devices]}")
        return devices
    
    def comprehensive_scan(self, progress_callback=None):
        """종합 스캔 - 버스 0과 1 자동 스캔"""
        connected_buses = self.connect_buses()
        
        print(f"연결된 버스: {connected_buses}")  # 디버그 출력
        
        if not connected_buses:
            print("사용 가능한 I2C 버스가 없습니다.")
            return None
        
        result = {
            'buses': {}
        }
        
        # 각 버스를 순서대로 스캔
        total_buses = len(connected_buses)
        for idx, bus_num in enumerate(sorted(connected_buses)):
            print(f"버스 {bus_num} 스캔 중... ({idx+1}/{total_buses})")
            
            # 각 버스에 대해 독립적인 진행률 콜백 생성
            def bus_progress_callback(progress):
                # 전체 진행률 = (완료된 버스 / 전체 버스 * 100) + (현재 버스 진행률 / 전체 버스)
                base_progress = (idx / total_buses) * 100
                current_bus_progress = (progress / total_buses)
                total_progress = int(base_progress + current_bus_progress)
                if progress_callback:
                    progress_callback(total_progress)
            
            devices = self.scan_bus(bus_num, bus_progress_callback)
            
            if devices:
                result['buses'][bus_num] = devices
                print(f"버스 {bus_num}에서 {len(devices)}개 디바이스 발견")
            else:
                print(f"버스 {bus_num}에서 디바이스를 찾지 못함")
        
        if progress_callback:
            progress_callback(100)
        
        print(f"전체 스캔 결과: {result}")  # 디버그 출력
        return result
    
    def get_device_info(self, addr):
        """디바이스 정보 반환"""
        device_info = {
            0x23: {"name": "BH1750", "type": "광센서", "description": "디지털 조도 센서", "voltage": "3.3V/5V"},
            0x25: {"name": "SDP810", "type": "차압센서", "description": "디지털 차압 센서", "voltage": "3.3V/5V"},
            0x5C: {"name": "BH1750", "type": "광센서", "description": "디지털 조도 센서 (ADDR=H)", "voltage": "3.3V/5V"},
            0x48: {"name": "ADS1115/TMP102", "type": "ADC/온도센서", "description": "16비트 ADC 또는 온도센서", "voltage": "3.3V/5V"},
            0x49: {"name": "ADS1115/TMP102", "type": "ADC/온도센서", "description": "16비트 ADC 또는 온도센서", "voltage": "3.3V/5V"},
            0x4A: {"name": "ADS1115/TMP102", "type": "ADC/온도센서", "description": "16비트 ADC 또는 온도센서", "voltage": "3.3V/5V"},
            0x4B: {"name": "ADS1115/TMP102", "type": "ADC/온도센서", "description": "16비트 ADC 또는 온도센서", "voltage": "3.3V/5V"},
            0x68: {"name": "DS3231/MPU6050", "type": "RTC/IMU", "description": "실시간 시계 또는 6축 IMU", "voltage": "3.3V/5V"},
            0x69: {"name": "MPU6050", "type": "IMU 센서", "description": "6축 관성측정장치", "voltage": "3.3V/5V"},
            0x76: {"name": "BME688", "type": "온습도환경센서", "description": "온습도기압 또는 가스센서", "voltage": "3.3V"},
            0x77: {"name": "BME688", "type": "온습도환경센서", "description": "온습도기압 또는 가스센서", "voltage": "3.3V"},
            0x5A: {"name": "MLX90614", "type": "적외선 온도센서", "description": "비접촉 적외선 온도계", "voltage": "3.3V/5V"},
            0x1D: {"name": "ADXL345", "type": "가속도센서", "description": "3축 가속도 센서", "voltage": "3.3V"},
            0x53: {"name": "ADXL345", "type": "가속도센서", "description": "3축 가속도 센서", "voltage": "3.3V"},
            0x3C: {"name": "SSD1306", "type": "OLED 디스플레이", "description": "128x64 OLED 디스플레이", "voltage": "3.3V"},
            0x3D: {"name": "SSD1306", "type": "OLED 디스플레이", "description": "128x64 OLED 디스플레이", "voltage": "3.3V"},
            # SHT40 온습도 센서 추가
            0x44: {"name": "SHT40", "type": "온습도센서", "description": "고정밀 디지털 온습도 센서", "voltage": "3.3V"},
            0x45: {"name": "SHT40", "type": "온습도센서", "description": "고정밀 디지털 온습도 센서 (ALT)", "voltage": "3.3V"},
        }
        
        # 온습도환경센서 주소들
        for tca_addr in range(0x70, 0x78):
            device_info[tca_addr] = {"name": "BME688", "type": "온습도환경센서", "description": "온습도기압 또는 가스센서", "voltage": "3.3V"}
        
        return device_info.get(addr, {"name": "Unknown", "type": "알 수 없음", "description": "알 수 없는 디바이스", "voltage": "?"})
    
    def test_specific_address(self, bus_number, addr):
        """특정 주소 직접 테스트"""
        if bus_number not in self.buses:
            print(f"버스 {bus_number}가 연결되지 않음")
            return False
        
        bus = self.buses[bus_number]
        print(f"\n=== 버스 {bus_number}, 주소 0x{addr:02X} 직접 테스트 ===")
        
        # SDP810 특별 테스트
        if addr == 0x25:
            return self.test_sdp810(bus, addr)
        
        # 테스트 1: read_byte
        try:
            result = bus.read_byte(addr)
            print(f"✅ read_byte 성공: 0x{result:02X}")
            return True
        except Exception as e:
            print(f"❌ read_byte 실패: {type(e).__name__} - {e}")
        
        # 테스트 2: write_byte + read (SHT40 시리얼 번호)
        try:
            bus.write_byte(addr, 0x89)
            time.sleep(0.001)
            data = bus.read_i2c_block_data(addr, 0x89, 6)
            print(f"✅ SHT40 시리얼 번호 읽기 성공: {[f'0x{b:02X}' for b in data]}")
            return True
        except Exception as e:
            print(f"❌ SHT40 시리얼 번호 실패: {type(e).__name__} - {e}")
        
        # 테스트 3: SHT40 측정 명령
        try:
            bus.write_byte(addr, 0xFD)
            time.sleep(0.01)
            data = bus.read_i2c_block_data(addr, 0xFD, 6)
            print(f"✅ SHT40 측정 명령 성공: {[f'0x{b:02X}' for b in data]}")
            return True
        except Exception as e:
            print(f"❌ SHT40 측정 명령 실패: {type(e).__name__} - {e}")
        
        # 테스트 4: 소프트 리셋
        try:
            bus.write_byte(addr, 0x94)
            time.sleep(0.001)
            print(f"✅ SHT40 소프트 리셋 성공")
            return True
        except Exception as e:
            print(f"❌ SHT40 소프트 리셋 실패: {type(e).__name__} - {e}")
        
        # 테스트 5: 일반적인 레지스터들
        for reg in [0x00, 0x01, 0x0F, 0xD0]:
            try:
                result = bus.read_byte_data(addr, reg)
                print(f"✅ 레지스터 0x{reg:02X} 읽기 성공: 0x{result:02X}")
                return True
            except Exception as e:
                print(f"❌ 레지스터 0x{reg:02X} 실패: {type(e).__name__} - {e}")
        
        print(f"❌ 모든 테스트 실패 - 디바이스 없음")
        return False

    def signal_handler(self, signum, frame):
        """키보드 인터럽트 (Ctrl+C) 처리"""
        print("\n🛑 인터럽트 신호 감지됨. 안전하게 종료 중...")
        self.close()
        sys.exit(0)
    
    def close(self):
        """리소스 정리 - 기존 메서드 개선"""
        print("I2C 버스 연결 정리 중...")
        for bus_num, bus in self.buses.items():
            try:
                bus.close()
                print(f"  버스 {bus_num} 연결 해제됨")
            except Exception as e:
                print(f"  버스 {bus_num} 해제 오류: {e}")
        self.buses.clear()
        print("✅ 모든 I2C 연결 정리 완료")

class I2CScannerGUI:
    """I2C 스캐너 GUI 클래스"""
    
    def __init__(self):
        # 메인 윈도우 생성
        self.root = ttkb.Window(
            title="I2C 디바이스 스캐너",
            themename="darkly",
            size=(800, 480),
            position=(0, 0)
        )
        
        self.scanner = I2CScanner()
        self.scan_result = None
        self.scanning = False
        self.current_bus = None  # 테스트용 현재 버스
        
        self.setup_ui()
        self.setup_styles()

        # 윈도우 종료 이벤트 처리
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        # 키보드 이벤트 바인딩
        self.root.bind('<Control-c>', self.keyboard_interrupt)
        self.root.focus_set()  # 키보드 포커스 설정
    
    def setup_styles(self):
        """커스텀 스타일 설정"""
        style = ttkb.Style()
        
        style.configure(
            "Header.TLabel",
            font=("Arial", 12, "bold"),
            foreground="#ffffff"
        )
        
        style.configure(
            "Card.TFrame",
            relief="raised",
            borderwidth=1
        )
    
    def setup_ui(self):
        """UI 구성"""
        main_container = ttkb.Frame(self.root, padding=5)
        main_container.pack(fill=BOTH, expand=True)
        
        # 상단 컨트롤 패널
        self.setup_control_panel(main_container)
        
        # 중앙 컨텐츠 영역
        paned = ttkb.PanedWindow(main_container, orient=HORIZONTAL)
        paned.pack(fill=BOTH, expand=True, pady=(5, 0))
        
        # 왼쪽 패널 (스캔 결과)
        left_frame = ttkb.Frame(paned, style="Card.TFrame", padding=5)
        paned.add(left_frame, weight=2)
        
        # 오른쪽 패널 (상세 정보)
        right_frame = ttkb.Frame(paned, style="Card.TFrame", padding=5)
        paned.add(right_frame, weight=1)
        
        self.setup_scan_results(left_frame)
        self.setup_device_details(right_frame)
    
    def setup_control_panel(self, parent):
        """상단 컨트롤 패널 설정"""
        control_frame = ttkb.Frame(parent, style="Card.TFrame", padding=5)
        control_frame.pack(fill=X, pady=(0, 5))
        
        # 제목
        title_label = ttkb.Label(
            control_frame,
            text="🔍 I2C 스캐너 (자동 스캔)",
            style="Header.TLabel"
        )
        title_label.pack(side=LEFT)
        
        # 우측 컨트롤들
        controls_frame = ttkb.Frame(control_frame)
        controls_frame.pack(side=RIGHT)
        
        # 스캔 버튼
        self.scan_button = ttkb.Button(
            controls_frame,
            text="스캔",
            style="success.TButton",
            command=self.start_scan,
            width=8
        )
        self.scan_button.pack(side=RIGHT, padx=(5, 0))
        
        # 진행률 표시
        self.progress_var = tk.IntVar()
        self.progress_bar = ttkb.Progressbar(
            controls_frame,
            variable=self.progress_var,
            style="success.Horizontal.TProgressbar",
            length=150
        )
        self.progress_bar.pack(side=RIGHT, padx=(5, 0))
        
        # 상태 라벨
        self.status_label = ttkb.Label(
            controls_frame,
            text="준비 - 스캔을 클릭하세요",
            foreground="#6c757d"
        )
        self.status_label.pack(side=RIGHT, padx=(5, 0))
    
    def setup_scan_results(self, parent):
        """스캔 결과 영역 설정"""
        # 헤더
        header_label = ttkb.Label(
            parent,
            text="📋 발견된 디바이스",
            style="Header.TLabel"
        )
        header_label.pack(anchor=W, pady=(0, 5))
        
        # 트리뷰 프레임 (고정 크기)
        tree_frame = ttkb.Frame(parent)
        tree_frame.pack(fill=X, pady=(0, 5))  # fill=X만 사용, expand=False로 고정
        
        # 트리뷰와 스크롤바
        self.tree = ttkb.Treeview(
            tree_frame,
            columns=("address", "type", "bus"),
            show="tree headings",
            height=8
        )
        
        # 컬럼 설정
        self.tree.heading("#0", text="디바이스명")
        self.tree.heading("address", text="주소")
        self.tree.heading("type", text="타입")
        self.tree.heading("bus", text="버스")
        
        self.tree.column("#0", width=120)
        self.tree.column("address", width=60)
        self.tree.column("type", width=80)
        self.tree.column("bus", width=60)
        
        # 스크롤바
        scrollbar = ttkb.Scrollbar(tree_frame, orient=VERTICAL, command=self.tree.yview)
        self.tree.configure(yscrollcommand=scrollbar.set)
        
        self.tree.pack(side=LEFT, fill=BOTH, expand=True)
        scrollbar.pack(side=RIGHT, fill=Y)
        
        # 이벤트 바인딩
        self.tree.bind("<<TreeviewSelect>>", self.on_device_select)
        
        # 하단 i2cdetect 테이블 (확장 가능)
        table_label = ttkb.Label(
            parent,
            text="📊 I2C 주소 맵",
            style="Header.TLabel"
        )
        table_label.pack(anchor=W, pady=(10, 3))
        
        self.i2c_table = scrolledtext.ScrolledText(
            parent,
            height=6,
            font=("Courier", 8),
            bg="#2b2b2b",
            fg="#ffffff",
            insertbackground="#ffffff"
        )
        self.i2c_table.pack(fill=BOTH, expand=True, pady=(0, 5))  # 이 부분만 확장됨
    
    def setup_device_details(self, parent):
        """디바이스 상세 정보 영역 설정"""
        # 헤더
        header_label = ttkb.Label(
            parent,
            text="ℹ️ 디바이스 정보",
            style="Header.TLabel"
        )
        header_label.pack(anchor=W, pady=(0, 5))
        
        # 정보 카드
        info_frame = ttkb.LabelFrame(
            parent,
            text="선택된 디바이스",
            padding=8
        )
        info_frame.pack(fill=X, pady=(0, 5))
        
        # 디바이스 정보 라벨들
        self.device_name_label = ttkb.Label(
            info_frame,
            text="디바이스를 선택하세요",
            font=("Arial", 10, "bold")
        )
        self.device_name_label.pack(anchor=W)
        
        self.device_address_label = ttkb.Label(
            info_frame,
            text="",
            foreground="#6c757d",
            font=("Arial", 8)
        )
        self.device_address_label.pack(anchor=W, pady=(2, 0))
        
        self.device_type_label = ttkb.Label(
            info_frame,
            text="",
            foreground="#17a2b8",
            font=("Arial", 8)
        )
        self.device_type_label.pack(anchor=W, pady=(2, 0))
        
        self.device_desc_label = ttkb.Label(
            info_frame,
            text="",
            wraplength=200,
            font=("Arial", 8)
        )
        self.device_desc_label.pack(anchor=W, pady=(5, 0))
        
        self.device_voltage_label = ttkb.Label(
            info_frame,
            text="",
            foreground="#28a745",
            font=("Arial", 8)
        )
        self.device_voltage_label.pack(anchor=W, pady=(2, 0))
        
        # 연결 정보
        connection_frame = ttkb.LabelFrame(
            parent,
            text="연결 정보",
            padding=8
        )
        connection_frame.pack(fill=X, pady=(0, 5))
        
        self.connection_info = ttkb.Label(
            connection_frame,
            text="디바이스를 선택하면\n연결 정보가 표시됩니다",
            wraplength=200,
            justify=LEFT,
            font=("Arial", 8)
        )
        self.connection_info.pack(anchor=W)
        
        # 액션 버튼들
        action_frame = ttkb.Frame(parent)
        action_frame.pack(fill=X, pady=(5, 0))
        
        self.test_button = ttkb.Button(
            action_frame,
            text="테스트",
            style="info.TButton",
            state=DISABLED,
            command=self.test_device
        )
        self.test_button.pack(fill=X, pady=(0, 3))
        
        self.refresh_button = ttkb.Button(
            action_frame,
            text="새로고침",
            style="secondary.TButton",
            command=self.start_scan
        )
        self.refresh_button.pack(fill=X)
    
    def start_scan(self):
        """스캔 시작"""
        if self.scanning:
            return
        
        self.scanning = True
        self.scan_button.config(state=DISABLED, text="스캔중...")
        self.status_label.config(text="I2C 버스 0, 1 스캔중...")
        self.progress_var.set(0)
        
        # 기존 결과 클리어
        for item in self.tree.get_children():
            self.tree.delete(item)
        
        self.i2c_table.delete(1.0, tk.END)
        
        # 백그라운드에서 스캔 실행
        thread = threading.Thread(target=self.scan_worker)
        thread.daemon = True
        thread.start()
    
    def scan_worker(self):
        """백그라운드 스캔 작업"""
        try:
            def progress_callback(value):
                self.root.after(0, lambda: self.progress_var.set(value))
            
            self.scan_result = self.scanner.comprehensive_scan(progress_callback)
            
            if self.scan_result:
                self.root.after(0, self.update_scan_results)
            else:
                self.root.after(0, lambda: self.show_error("I2C 버스에 연결할 수 없습니다.\nsudo 권한으로 실행하세요."))
                
        except Exception as e:
            self.root.after(0, lambda: self.show_error(f"스캔 오류: {str(e)}"))
        finally:
            self.root.after(0, self.scan_complete)
    
    def update_scan_results(self):
        """스캔 결과 UI 업데이트"""
        if not self.scan_result:
            print("스캔 결과가 없습니다.")
            return
        
        print(f"UI 업데이트 시작: {self.scan_result}")  # 디버그 출력
        
        total_devices = 0
        
        # 각 버스별로 디바이스 표시
        for bus_num in sorted(self.scan_result['buses'].keys()):
            devices = self.scan_result['buses'][bus_num]
            print(f"버스 {bus_num} UI 업데이트: {len(devices)}개 디바이스")
            
            bus_parent = self.tree.insert("", "end", text=f"I2C 버스 {bus_num}", open=True)
            
            for addr in sorted(devices):
                device_info = self.scanner.get_device_info(addr)
                item_id = self.tree.insert(
                    bus_parent, "end",
                    text=device_info['name'],
                    values=(f"0x{addr:02X}", device_info['type'], f"버스 {bus_num}"),
                    tags=(addr, bus_num)
                )
                print(f"  - 디바이스 추가: {device_info['name']} (0x{addr:02X})")
                total_devices += 1
        
        # i2cdetect 테이블 업데이트
        self.update_i2c_table()
        
        # 상태 업데이트
        buses_scanned = sorted(self.scan_result['buses'].keys())
        status_text = f"스캔 완료 - 버스 {buses_scanned}에서 {total_devices}개 발견"
        print(f"상태 업데이트: {status_text}")
        self.status_label.config(text=status_text)
    
    def update_i2c_table(self):
        """i2cdetect 스타일 테이블 업데이트"""
        self.i2c_table.delete(1.0, tk.END)
        
        # 버스를 순서대로 정렬하여 표시
        for bus_num in sorted(self.scan_result['buses'].keys()):
            devices = self.scan_result['buses'][bus_num]
            self.i2c_table.insert(tk.END, f"I2C 버스 {bus_num} 스캔 결과:\n")
            self.i2c_table.insert(tk.END, "=" * 50 + "\n")
            
            # 테이블 헤더
            self.i2c_table.insert(tk.END, "     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\n")
            
            # 테이블 내용
            for row in range(0x00, 0x80, 0x10):
                line = f"{row:02x}: "
                
                for col in range(0x10):
                    addr = row + col
                    
                    if addr < 0x08 or addr > 0x77:
                        line += "   "
                    elif addr in devices:
                        line += f"{addr:02x} "
                    else:
                        line += "-- "
                
                self.i2c_table.insert(tk.END, line + "\n")
            
            if devices:
                device_list = ', '.join([f'0x{addr:02X}' for addr in sorted(devices)])
                self.i2c_table.insert(tk.END, f"\n발견된 디바이스: {device_list}\n\n")
            else:
                self.i2c_table.insert(tk.END, "\n디바이스가 발견되지 않았습니다.\n\n")
    
    def on_device_select(self, event):
        """디바이스 선택 이벤트"""
        selection = self.tree.selection()
        if not selection:
            return
        
        item = selection[0]
        tags = self.tree.item(item, "tags")
        
        if not tags or len(tags) < 2:
            return
        
        try:
            addr = int(tags[0])
            bus_num = int(tags[1])
            self.current_bus = bus_num  # 테스트용 현재 버스 저장
            
            device_info = self.scanner.get_device_info(addr)
            
            # 디바이스 정보 업데이트
            self.device_name_label.config(text=device_info['name'])
            self.device_address_label.config(text=f"I2C 주소: 0x{addr:02X}")
            self.device_type_label.config(text=f"타입: {device_info['type']}")
            self.device_desc_label.config(text=device_info['description'])
            self.device_voltage_label.config(text=f"전원: {device_info['voltage']}")
            
            # 연결 정보 업데이트
            self.connection_info.config(text=f"• I2C 버스 {bus_num}에 직접 연결\n• 라이브러리에서 바로 사용 가능")
            
            self.test_button.config(state=NORMAL)
            
        except (ValueError, IndexError):
            pass
    
    def scan_complete(self):
        """스캔 완료 처리"""
        self.scanning = False
        self.scan_button.config(state=NORMAL, text="스캔")
        self.progress_var.set(100)
    
    def test_device(self):
        """선택된 디바이스 테스트"""
        selection = self.tree.selection()
        if not selection:
            return
        
        item = selection[0]
        tags = self.tree.item(item, "tags")
        
        if not tags or len(tags) < 2:
            return
        
        try:
            addr = int(tags[0])
            bus_num = int(tags[1])
            device_info = self.scanner.get_device_info(addr)
            
            # 백그라운드에서 테스트 실행
            self.test_button.config(state=DISABLED, text="테스트중...")
            
            thread = threading.Thread(
                target=self.test_device_worker,
                args=(addr, bus_num, device_info['name'])
            )
            thread.daemon = True
            thread.start()
            
        except (ValueError, IndexError):
            messagebox.showerror("오류", "디바이스 정보를 읽을 수 없습니다.")
    
    def test_device_worker(self, addr, bus_num, device_name):
        """백그라운드 디바이스 테스트"""
        try:
            if bus_num not in self.scanner.buses:
                self.root.after(0, lambda: self.show_test_error("I2C 버스 연결 실패"))
                return
            
            # 센서별 테스트 실행
            result = self.test_sensor_by_address(addr, bus_num, device_name)
            
            # 결과 표시
            self.root.after(0, lambda: self.show_test_result(device_name, addr, result))
            
        except Exception as e:
            self.root.after(0, lambda: self.show_test_error(f"테스트 오류: {str(e)}"))
        finally:
            self.root.after(0, lambda: self.test_button.config(state=NORMAL, text="테스트"))
    
    def test_sensor_by_address(self, addr, bus_num, device_name):
        """주소별 센서 테스트"""
        if bus_num not in self.scanner.buses:
            return {"error": "I2C 연결 없음"}
        
        bus = self.scanner.buses[bus_num]
        
        try:
            # SDP810 차압센서 테스트
            if addr == 0x25:
                return self.test_sdp810_detailed(bus, addr)
            
            # SHT40 온습도센서 테스트
            elif addr in [0x44, 0x45]:
                # 1단계: 직접 I2C 통신 시도
                print("SHT40 테스트 단계 1: 직접 I2C 통신")
                result = self.test_sht40_direct_i2c(bus, addr)
                if result and "success" in result:
                    return result
                
                # 2단계: 표준 SMBus 통신 시도
                print("SHT40 테스트 단계 2: 표준 SMBus 통신")
                result = self.test_sht40(bus, addr)
                if "error" not in result:
                    return result
                
                # 3단계: 간단한 연결 테스트
                print("SHT40 테스트 단계 3: 간단한 연결 테스트")
                result = self.test_sht40_simple(bus, addr)
                return result
            
            # BH1750 조도센서 테스트
            elif addr in [0x23, 0x5C]:
                return self.test_bh1750(bus, addr)
            
            # BME280/BME688 환경센서 테스트
            elif addr in [0x76, 0x77]:
                return self.test_bme_series(bus, addr)
            
            # 기본 연결 테스트
            else:
                return self.test_basic_connection(bus, addr)
                
        except Exception as e:
            return {"error": str(e)}
    
    def test_sdp810_detailed(self, bus, addr):
        """SDP810 차압 센서 상세 테스트"""
        try:
            print(f"SDP810 상세 테스트 시작: 주소 0x{addr:02X}")
            
            # 여러 번 측정하여 안정성 확인
            measurements = []
            successful_reads = 0
            crc_failures = 0
            
            for i in range(5):  # 5번 측정
                try:
                    # 저수준 I2C 통신
                    read_msg = smbus2.i2c_msg.read(addr, 3)
                    bus.i2c_rdwr(read_msg)
                    raw_data = list(read_msg)
                    
                    if len(raw_data) != 3:
                        continue
                    
                    pressure_msb = raw_data[0]
                    pressure_lsb = raw_data[1]
                    received_crc = raw_data[2]
                    
                    # CRC-8 검증
                    calculated_crc = self.scanner._calculate_crc8_sdp810([pressure_msb, pressure_lsb])
                    crc_ok = calculated_crc == received_crc
                    
                    if not crc_ok:
                        crc_failures += 1
                    
                    # 압력값 계산
                    raw_pressure = struct.unpack('>h', bytes([pressure_msb, pressure_lsb]))[0]
                    pressure_pa = raw_pressure / 60.0
                    
                    measurements.append(pressure_pa)
                    successful_reads += 1
                    
                    time.sleep(0.1)  # 100ms 간격으로 측정
                    
                except Exception as e:
                    print(f"  측정 {i+1} 실패: {e}")
                    continue
            
            if successful_reads == 0:
                return {"error": "모든 측정 실패"}
            
            # 통계 계산
            avg_pressure = sum(measurements) / len(measurements)
            
            # 압력 방향 결정
            if avg_pressure >= 0:
                port_label = "P1(흡기)"
                pressure_str = f"+{avg_pressure:.1f}"
            else:
                port_label = "P2(배기)"
                pressure_str = f"{avg_pressure:.1f}"
            
            # 현재 시간
            current_time = time.strftime("%H:%M:%S")
            
            return {
                "success": True,
                "type": "차압센서 (SDP810)",
                "values": {
                    "현재 측정값": f"{port_label}: {pressure_str} Pa",
                    "측정 시간": current_time,
                    "측정 횟수": f"{successful_reads}/5회",
                    "CRC 성공률": f"{successful_reads - crc_failures}/{successful_reads}",
                    "평균 압력": f"{avg_pressure:.2f} Pa",
                    "측정 범위": f"{min(measurements):.1f} ~ {max(measurements):.1f} Pa" if len(measurements) > 1 else "단일 측정",
                    "상태": "정상 동작"
                }
            }
            
        except Exception as e:
            print(f"SDP810 상세 테스트 실패: {e}")
            return {"error": f"SDP810 테스트 실패: {str(e)}"}
    
    def test_sht40(self, bus, addr):
        """SHT40 온습도센서 테스트 - 실제 측정값 획득"""
        try:
            print(f"SHT40 실제 측정 테스트 시작: 주소 0x{addr:02X}")
            
            # 방법 1: 표준 SHT40 측정 절차
            try:
                print("  방법 1: 표준 SHT40 측정 절차")
                
                # 1단계: 소프트 리셋
                print("    소프트 리셋...")
                bus.write_byte(addr, 0x94)
                time.sleep(0.002)  # 2ms 대기
                
                # 2단계: 측정 명령 (High Precision, No Heater)
                print("    측정 명령 전송 (0xFD)...")
                bus.write_byte(addr, 0xFD)
                time.sleep(0.02)  # 20ms 대기 (충분한 시간 확보)
                
                # 3단계: 6바이트 데이터 읽기 (명령 없이 순수 읽기)
                print("    데이터 읽기...")
                data = []
                for i in range(6):
                    byte_val = bus.read_byte(addr)
                    data.append(byte_val)
                    time.sleep(0.001)  # 각 바이트 사이 1ms 대기
                
                print(f"    읽은 데이터: {[f'0x{b:02X}' for b in data]}")
                
                if len(data) >= 6:
                    # 온도 데이터 (첫 2바이트)
                    temp_raw = (data[0] << 8) | data[1]
                    # 습도 데이터 (4-5번째 바이트, 3번째는 CRC)
                    hum_raw = (data[3] << 8) | data[4]
                    
                    # SHT40 변환 공식
                    temperature = -45 + 175 * temp_raw / 65535
                    humidity = 100 * hum_raw / 65535
                    
                    # 합리적인 범위 체크
                    if -40 <= temperature <= 125 and 0 <= humidity <= 100:
                        return {
                            "success": True,
                            "type": "온습도센서 (SHT40)",
                            "values": {
                                "온도": f"{temperature:.1f}°C",
                                "습도": f"{humidity:.1f}%RH",
                                "원시 온도": f"0x{temp_raw:04X}",
                                "원시 습도": f"0x{hum_raw:04X}",
                                "상태": "정상 (표준 방식)"
                            }
                        }
                    else:
                        print(f"    범위 벗어남: 온도={temperature:.1f}°C, 습도={humidity:.1f}%RH")
                
            except Exception as e:
                print(f"    방법 1 실패: {e}")
            
            # 방법 2: 다른 측정 명령 시도 (Medium Precision)
            try:
                print("  방법 2: Medium Precision 측정")
                
                bus.write_byte(addr, 0x94)  # 소프트 리셋
                time.sleep(0.002)
                
                bus.write_byte(addr, 0xF6)  # Medium precision 명령
                time.sleep(0.015)  # 15ms 대기
                
                data = []
                for i in range(6):
                    data.append(bus.read_byte(addr))
                    time.sleep(0.001)
                
                print(f"    Medium precision 데이터: {[f'0x{b:02X}' for b in data]}")
                
                if len(data) >= 6:
                    temp_raw = (data[0] << 8) | data[1]
                    hum_raw = (data[3] << 8) | data[4]
                    
                    temperature = -45 + 175 * temp_raw / 65535
                    humidity = 100 * hum_raw / 65535
                    
                    if -40 <= temperature <= 125 and 0 <= humidity <= 100:
                        return {
                            "success": True,
                            "type": "온습도센서 (SHT40)",
                            "values": {
                                "온도": f"{temperature:.1f}°C",
                                "습도": f"{humidity:.1f}%RH",
                                "원시 온도": f"0x{temp_raw:04X}",
                                "원시 습도": f"0x{hum_raw:04X}",
                                "상태": "정상 (Medium Precision)"
                            }
                        }
                
            except Exception as e:
                print(f"    방법 2 실패: {e}")
            
            # 모든 방법 실패
            raise Exception("모든 측정 방법 실패")
            
        except Exception as e:
            print(f"SHT40 테스트 완전 실패: {e}")
            return {"error": f"SHT40 테스트 실패: {str(e)}"}
    
    def test_bh1750(self, bus, addr):
        """BH1750 조도센서 테스트"""
        try:
            bus.write_byte(addr, 0x20)
            time.sleep(0.12)
            
            data = bus.read_i2c_block_data(addr, 0x20, 2)
            lux = ((data[0] << 8) + data[1]) / 1.2
            
            return {
                "success": True,
                "type": "조도센서 (BH1750)",
                "values": {
                    "조도": f"{lux:.1f} lux",
                    "원시데이터": f"0x{data[0]:02X}{data[1]:02X}",
                    "상태": "정상"
                }
            }
        except Exception as e:
            return {"error": f"BH1750 테스트 실패: {e}"}
    
    def test_bme_series(self, bus, addr):
        """BME280/BME688 시리즈 센서 테스트"""
        try:
            chip_id = bus.read_byte_data(addr, 0xD0)
            
            sensor_name = "BME280" if chip_id == 0x60 else "BME688" if chip_id == 0x61 else "BME 시리즈"
            
            return {
                "success": True,
                "type": f"환경센서 ({sensor_name})",
                "values": {
                    "센서": f"{sensor_name} 확인됨",
                    "칩 ID": f"0x{chip_id:02X}",
                    "측정": "온도, 습도, 기압" + (", 가스" if chip_id == 0x61 else ""),
                    "상태": "정상 연결"
                }
            }
        except Exception as e:
            return {"error": f"BME 시리즈 센서 테스트 실패: {e}"}
    
    def test_sht40_direct_i2c(self, bus, addr):
        """SHT40 직접 I2C 통신 방식 (i2cdetect와 유사)"""
        try:
            import fcntl
            import struct
            
            print(f"SHT40 직접 I2C 통신 테스트: 주소 0x{addr:02X}")
            
            # I2C_SLAVE 상수
            I2C_SLAVE = 0x0703
            
            # /dev/i2c-1 직접 열기
            bus_num = 1 if addr == 0x44 else 0  # SHT40은 보통 버스 1에 있음
            
            try:
                with open(f'/dev/i2c-{bus_num}', 'r+b', buffering=0) as i2c:
                    # 슬레이브 주소 설정
                    fcntl.ioctl(i2c, I2C_SLAVE, addr)
                    
                    # 방법 1: 소프트 리셋 + 측정
                    print("  직접 I2C: 소프트 리셋")
                    i2c.write(bytes([0x94]))  # 소프트 리셋
                    time.sleep(0.002)
                    
                    print("  직접 I2C: 고정밀 측정 명령")
                    i2c.write(bytes([0xFD]))  # 고정밀 측정
                    time.sleep(0.02)  # 20ms 대기
                    
                    print("  직접 I2C: 데이터 읽기")
                    data = i2c.read(6)  # 6바이트 읽기
                    data = list(data)
                    
                    print(f"  직접 I2C 데이터: {[f'0x{b:02X}' for b in data]}")
                    
                    if len(data) >= 6:
                        temp_raw = (data[0] << 8) | data[1]
                        hum_raw = (data[3] << 8) | data[4]
                        
                        temperature = -45 + 175 * temp_raw / 65535
                        humidity = 100 * hum_raw / 65535
                        
                        if -40 <= temperature <= 125 and 0 <= humidity <= 100:
                            return {
                                "success": True,
                                "type": "온습도센서 (SHT40)",
                                "values": {
                                    "온도": f"{temperature:.1f}°C",
                                    "습도": f"{humidity:.1f}%RH",
                                    "원시 온도": f"0x{temp_raw:04X}",
                                    "원시 습도": f"0x{hum_raw:04X}",
                                    "상태": "정상 (직접 I2C 통신)"
                                }
                            }
                
            except Exception as e:
                print(f"  직접 I2C 통신 실패: {e}")
                
        except Exception as e:
            print(f"직접 I2C 모듈 로드 실패: {e}")
        
        return None
    
    def test_sht40_simple(self, bus, addr):
        """SHT40 간단한 연결 테스트"""
        try:
            # 가장 안전한 방법: 소프트 리셋만 시도
            bus.write_byte(addr, 0x94)  # 소프트 리셋
            time.sleep(0.001)
            
            return {
                "success": True,
                "type": "온습도센서 (SHT40)",
                "values": {
                    "상태": "연결 확인됨",
                    "테스트": "소프트 리셋 응답 정상",
                    "참고": "기본 연결 테스트만 수행됨"
                }
            }
        except Exception as e:
            return {"error": f"SHT40 연결 테스트 실패: {e}"}

    def test_basic_connection(self, bus, addr):
        """기본 연결 테스트"""
        try:
            bus.read_byte(addr)
            
            return {
                "success": True,
                "type": "연결 테스트",
                "values": {
                    "상태": "연결 정상",
                    "주소": f"0x{addr:02X}",
                    "응답": "디바이스 응답함"
                }
            }
        except Exception as e:
            return {"error": f"연결 테스트 실패: {e}"}
    
    def show_test_result(self, device_name, addr, result):
        """테스트 결과 표시"""
        if "error" in result:
            messagebox.showerror("테스트 실패", f"{device_name} 테스트 실패\n\n{result['error']}")
        else:
            # 결과 창 생성
            result_window = ttkb.Toplevel(self.root)
            result_window.title(f"{device_name} 테스트 결과")
            result_window.geometry("400x300")
            result_window.transient(self.root)
            result_window.grab_set()
            
            # 결과 내용
            main_frame = ttkb.Frame(result_window, padding=15)
            main_frame.pack(fill=BOTH, expand=True)
            
            # 헤더
            header_label = ttkb.Label(
                main_frame,
                text=f"🔍 {device_name} (0x{addr:02X})",
                font=("Arial", 14, "bold")
            )
            header_label.pack(pady=(0, 10))
            
            # 타입
            type_label = ttkb.Label(
                main_frame,
                text=f"타입: {result['type']}",
                foreground="#17a2b8",
                font=("Arial", 10)
            )
            type_label.pack(pady=(0, 10))
            
            # 결과 값들
            if "values" in result:
                values_frame = ttkb.LabelFrame(main_frame, text="측정 결과", padding=10)
                values_frame.pack(fill=BOTH, expand=True, pady=(0, 10))
                
                for key, value in result["values"].items():
                    value_frame = ttkb.Frame(values_frame)
                    value_frame.pack(fill=X, pady=2)
                    
                    key_label = ttkb.Label(
                        value_frame,
                        text=f"{key}:",
                        font=("Arial", 9, "bold"),
                        width=12,
                        anchor=W
                    )
                    key_label.pack(side=LEFT)
                    
                    value_label = ttkb.Label(
                        value_frame,
                        text=str(value),
                        foreground="#28a745",
                        font=("Arial", 9)
                    )
                    value_label.pack(side=LEFT, fill=X, expand=True)
            
            # 닫기 버튼
            close_button = ttkb.Button(
                main_frame,
                text="닫기",
                command=result_window.destroy,
                style="secondary.TButton"
            )
            close_button.pack(pady=(10, 0))
    
    def show_test_error(self, error_msg):
        """테스트 오류 표시"""
        messagebox.showerror("테스트 오류", error_msg)
        self.test_button.config(state=NORMAL, text="테스트")
    
    def show_error(self, message):
        """에러 메시지 표시"""
        messagebox.showerror("스캔 오류", message)
        self.status_label.config(text="실패")
        self.scan_complete()
    
    def run(self):
        """GUI 실행"""
        try:
            self.root.mainloop()
        finally:
            self.scanner.close()

    def on_closing(self):
        """윈도우 종료 시 처리"""
        if self.scanning:
            print("⏳ 스캔 중... 잠시 대기하세요.")
            # 스캔 중이면 강제 종료하지 않고 완료 대기
            self.root.after(1000, self.on_closing)
            return
        
        print("🔄 애플리케이션 종료 중...")
        try:
            self.scanner.close()
            print("✅ 정리 완료")
        except Exception as e:
            print(f"⚠️ 종료 중 오류: {e}")
        finally:
            self.root.quit()
            self.root.destroy()

    def keyboard_interrupt(self, event=None):
        """키보드 인터럽트 처리 (GUI 환경에서)"""
        print("\n🛑 Ctrl+C 감지됨. 안전하게 종료 중...")
        self.on_closing()

if __name__ == "__main__":
    try:
        app = I2CScannerGUI()
        app.run()
    except KeyboardInterrupt:
        print("\n프로그램이 사용자에 의해 중단되었습니다.")
        sys.exit(0)
    except ImportError:
        print("ttkbootstrap가 설치되지 않았습니다.")
        print("설치: pip install ttkbootstrap")
        sys.exit(1)
    except Exception as e:
        print(f"애플리케이션 실행 오류: {e}")
        sys.exit(1)
    finally:
        print("👋 I2C 스캐너 종료됨")