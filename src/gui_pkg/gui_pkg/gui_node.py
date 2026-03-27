import os
import signal
import sys
import threading
import subprocess
import time
import rclpy
from rclpy.node import Node
 
from PyQt5.QtWidgets import (QApplication, QWidget, QVBoxLayout, QHBoxLayout,
                             QLabel, QPushButton, QMessageBox, QTextEdit)
from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtGui import QColor, QTextCharFormat, QTextCursor
 
# ==========================================
# [1] ROS2 Node 클래스
# ==========================================
class ControlPanelNode(Node):
    def __init__(self):
        super().__init__('gui_node')
        self.get_logger().info("런처 전용 GUI 노드가 시작되었습니다!")
 
# ==========================================
# [2] PyQt5 GUI 창 클래스
# ==========================================
class MainWindow(QWidget):
    update_ui_signal = pyqtSignal(int, bool)       # (index, is_running)
    node_crashed_signal = pyqtSignal(int, str)     # (index, error_message) — 비정상 종료 감지
    append_log_signal = pyqtSignal(str, str)       # (message, level) — 로그 패널 출력
 
    # 노드 상태 상수
    STATUS_STOPPED = 0
    STATUS_RUNNING = 1
    STATUS_ERROR   = 2
 
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
 
        self.nodes_config = [
            {"name": "1. 카메라 (Image Publisher)",   "pkg": "camera_perception_pkg",    "exec": "image_publisher_node",    "process": None, "status": self.STATUS_STOPPED},
            {"name": "2. YOLOv8 인식",                "pkg": "camera_perception_pkg",    "exec": "yolov8_node",             "process": None, "status": self.STATUS_STOPPED},
            {"name": "3. 차선 추출 (Lane Info)",       "pkg": "camera_perception_pkg",    "exec": "lane_info_extractor_node","process": None, "status": self.STATUS_STOPPED},
            {"name": "4. 경로 계획 (Path Planner)",    "pkg": "decision_making_pkg",      "exec": "path_planner_node",       "process": None, "status": self.STATUS_STOPPED},
            {"name": "5. 아두이노 통신 (Serial Sender)","pkg": "serial_communication_pkg", "exec": "serial_sender_node",      "process": None, "status": self.STATUS_STOPPED},
            {"name": "6. 모션 제어 (Motion Planner)",  "pkg": "decision_making_pkg",      "exec": "motion_planner_node",     "process": None, "status": self.STATUS_STOPPED},
        ]
 
        self.buttons_start  = []
        self.buttons_stop   = []
        self.labels_status  = []
 
        self.update_ui_signal.connect(self.update_node_status_ui)
        self.node_crashed_signal.connect(self.on_node_crashed)
        self.append_log_signal.connect(self.append_log)
 
        self.initUI()
 
    # ==========================================
    # UI 구성
    # ==========================================
    def initUI(self):
        self.setWindowTitle('자율주행 시스템 통합 런처')
        self.resize(560, 560)
        main_layout = QVBoxLayout()
 
        # 상단 버튼
        launcher_label = QLabel('<b>[ 시스템 노드 런처 ]</b>', self)
        main_layout.addWidget(launcher_label)
 
        btn_layout = QHBoxLayout()
        self.btn_start_all = QPushButton('🚀 전체 순서대로 시작', self)
        self.btn_start_all.setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold; padding: 10px;")
        self.btn_start_all.clicked.connect(self.start_sequence)
 
        self.btn_stop_all = QPushButton('🛑 전체 종료', self)
        self.btn_stop_all.setStyleSheet("background-color: #f44336; color: white; font-weight: bold; padding: 10px;")
        self.btn_stop_all.clicked.connect(self.stop_all_nodes)
 
        btn_layout.addWidget(self.btn_start_all)
        btn_layout.addWidget(self.btn_stop_all)
        main_layout.addLayout(btn_layout)
 
        # 유틸리티 버튼들
        util_layout = QHBoxLayout()
 
        self.btn_serial_fix = QPushButton('🔧 시리얼 포트 권한 해결 (sudo)', self)
        self.btn_serial_fix.clicked.connect(self.fix_serial_permission)
 
        self.btn_cuda_fix = QPushButton('⚡ CUDA 초기화 오류 복구 (sudo)', self)
        self.btn_cuda_fix.setStyleSheet("background-color: #FF9800; color: white; font-weight: bold;")
        self.btn_cuda_fix.clicked.connect(self.fix_cuda)
 
        util_layout.addWidget(self.btn_serial_fix)
        util_layout.addWidget(self.btn_cuda_fix)
        main_layout.addLayout(util_layout)
 
        # 개별 노드 행
        for i, config in enumerate(self.nodes_config):
            row_layout = QHBoxLayout()
 
            lbl_name = QLabel(config["name"])
            lbl_name.setFixedWidth(210)
 
            lbl_status = QLabel('🔴 중지됨')
            lbl_status.setFixedWidth(90)
 
            btn_start = QPushButton('시작')
            btn_start.clicked.connect(lambda checked, idx=i: self.start_node(idx))
 
            btn_stop = QPushButton('종료')
            btn_stop.setEnabled(False)
            btn_stop.clicked.connect(lambda checked, idx=i: self.stop_node(idx))
 
            self.labels_status.append(lbl_status)
            self.buttons_start.append(btn_start)
            self.buttons_stop.append(btn_stop)
 
            row_layout.addWidget(lbl_name)
            row_layout.addWidget(lbl_status)
            row_layout.addWidget(btn_start)
            row_layout.addWidget(btn_stop)
            main_layout.addLayout(row_layout)
 
        # ── 로그 패널 ──────────────────────────────
        log_label = QLabel('<b>[ 시스템 로그 ]</b>')
        main_layout.addWidget(log_label)
 
        self.log_area = QTextEdit()
        self.log_area.setReadOnly(True)
        self.log_area.setFixedHeight(160)
        self.log_area.setStyleSheet("background-color: #1e1e1e; color: #d4d4d4; font-family: monospace; font-size: 11px;")
        main_layout.addWidget(self.log_area)
 
        btn_clear_log = QPushButton('로그 지우기')
        btn_clear_log.setFixedHeight(24)
        btn_clear_log.clicked.connect(self.log_area.clear)
        main_layout.addWidget(btn_clear_log)
        # ───────────────────────────────────────────
 
        self.setLayout(main_layout)
        self._log("GUI 시작됨.", "info")
 
    # ==========================================
    # 로그 패널
    # ==========================================
    def _log(self, message: str, level: str = "info"):
        """백그라운드 스레드에서 안전하게 로그를 추가하는 진입점."""
        self.append_log_signal.emit(message, level)
 
    def append_log(self, message: str, level: str):
        """시그널을 통해 GUI 스레드에서 실행됨."""
        color_map = {
            "info":  "#d4d4d4",
            "ok":    "#6fd06f",
            "warn":  "#e5c07b",
            "error": "#e06c75",
        }
        color = color_map.get(level, "#d4d4d4")
        timestamp = time.strftime("%H:%M:%S")
 
        cursor = self.log_area.textCursor()
        cursor.movePosition(QTextCursor.End)
 
        fmt = QTextCharFormat()
        fmt.setForeground(QColor(color))
        cursor.setCharFormat(fmt)
        cursor.insertText(f"[{timestamp}] {message}\n")
 
        self.log_area.setTextCursor(cursor)
        self.log_area.ensureCursorVisible()
 
    # ==========================================
    # 노드 상태 UI 갱신
    # ==========================================
    def update_node_status_ui(self, index: int, is_running: bool):
        if is_running:
            self.labels_status[index].setText('🟢 실행 중')
            self.labels_status[index].setStyleSheet("color: green;")
            self.buttons_start[index].setEnabled(False)
            self.buttons_stop[index].setEnabled(True)
            self.nodes_config[index]["status"] = self.STATUS_RUNNING
        else:
            # 에러 상태가 아닌 경우에만 '중지됨'으로 바꿈
            # (on_node_crashed 가 이미 에러 표시를 했을 수 있음)
            if self.nodes_config[index]["status"] != self.STATUS_ERROR:
                self.labels_status[index].setText('🔴 중지됨')
                self.labels_status[index].setStyleSheet("color: red;")
            self.buttons_start[index].setEnabled(True)
            self.buttons_stop[index].setEnabled(False)
 
    def on_node_crashed(self, index: int, error_msg: str):
        """비정상 종료 시 호출 — GUI 스레드에서 실행됨."""
        self.nodes_config[index]["status"] = self.STATUS_ERROR
        self.nodes_config[index]["process"] = None
        self.labels_status[index].setText('🔶 에러')
        self.labels_status[index].setStyleSheet("color: orange;")
        self.buttons_start[index].setEnabled(True)
        self.buttons_stop[index].setEnabled(False)
 
        name = self.nodes_config[index]["name"]
        self._log(f"[에러] {name} 비정상 종료 — {error_msg}", "error")
 
    # ==========================================
    # 프로세스 감시 스레드
    # ==========================================
    def _watch_process(self, index: int, process):
        """
        노드 프로세스를 감시하다가, 예상치 못하게 종료되면
        node_crashed_signal 을 발행합니다.
        정상 종료(_stop_node_impl 에 의한 종료)는 process 가 None 이 된 후
        wait() 가 리턴되므로, config["process"] 비교로 구분합니다.
        """
        returncode = process.wait()  # 프로세스가 죽을 때까지 대기
 
        # _stop_node_impl 이 이미 config["process"] = None 처리를 했다면 정상 종료
        if self.nodes_config[index]["process"] is None:
            return
 
        # 여기까지 왔다면 외부(예기치 않은) 종료
        self.nodes_config[index]["process"] = None
 
        if returncode == 0:
            # 0이어도 자발적 비정상 종료일 수 있으므로 warn 처리
            self._log(f"{self.nodes_config[index]['name']} 예기치 않게 종료됨 (returncode=0)", "warn")
            self.update_ui_signal.emit(index, False)
        else:
            error_msg = f"returncode={returncode}"
            self.node_crashed_signal.emit(index, error_msg)
 
    # ==========================================
    # Wi-Fi / 권한 / CUDA 유틸리티
    # ==========================================
    def check_wifi_is_on(self):
        try:
            output = subprocess.check_output(['nmcli', 'radio', 'wifi'], text=True).strip()
            return output == 'enabled'
        except Exception as e:
            self._log(f"Wi-Fi 상태 확인 불가: {e}", "warn")
            return False
 
    def fix_serial_permission(self):
        try:
            subprocess.Popen(['gnome-terminal', '--', 'bash', '-c',
                              'echo "시리얼 포트 및 노드 권한을 부여합니다."; '
                              'sudo chmod 666 /dev/ttyACM0; '
                              'sudo chmod 777 -R ~/dynamic_obstacle_ws/src/serial_communication_pkg; '
                              'echo "완료! 창을 닫으세요."; exec bash'])
        except Exception as e:
            QMessageBox.warning(self, "오류", f"권한 명령 실행 실패: {e}")
 
    def fix_cuda(self):
        reply = QMessageBox.question(self, 'CUDA 복구',
            'NVIDIA UVM 모듈을 재시작합니다.\n실행 중인 YOLOv8 노드가 있으면 먼저 종료하세요.\n\n계속하시겠습니까?',
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        if reply == QMessageBox.No:
            return
        try:
            subprocess.Popen([
                'gnome-terminal', '--', 'bash', '-c',
                'echo "NVIDIA UVM 모듈 재시작 중..."; '
                'sudo rmmod nvidia_uvm && sudo modprobe nvidia_uvm && '
                'echo "✅ 완료! 이 창을 닫고 YOLOv8 노드를 다시 실행하세요." || '
                'echo "❌ 실패. nvidia-smi 상태를 확인하세요."; '
                'exec bash'
            ])
            self._log("CUDA 복구 명령 실행됨. 터미널 창을 확인하세요.", "warn")
        except Exception as e:
            QMessageBox.warning(self, "오류", f"명령 실행 실패: {e}")
 
    # ==========================================
    # 노드 실행 / 종료
    # ==========================================
    def start_sequence(self):
        if self.check_wifi_is_on():
            QMessageBox.critical(self, "⚠️ Wi-Fi 경고 ⚠️",
                                 "현재 Wi-Fi가 켜져 있습니다!\n학생들의 토픽과 충돌할 위험이 있습니다.\n\n우측 상단 메뉴에서 Wi-Fi를 끄고 다시 시도해 주세요.")
            return
 
        reply = QMessageBox.question(self, '순차 실행',
                                     '와이파이가 꺼져있습니다. 모든 노드를 순서대로 켤까요?\n(각 노드별로 1초씩 대기합니다)',
                                     QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        if reply == QMessageBox.No:
            return
 
        self.btn_start_all.setEnabled(False)
        threading.Thread(target=self._sequence_thread, daemon=True).start()
 
    def _sequence_thread(self):
        for i, config in enumerate(self.nodes_config):
            if config["process"] is None:
                self._run_node_process(i)
                time.sleep(1.0)
        self._log("모든 노드 실행 완료!", "ok")
 
    def _run_node_process(self, index):
        config = self.nodes_config[index]
        cmd = ['ros2', 'run', config['pkg'], config['exec']]
 
        self._log(f"시작: {config['name']}", "info")
        process = subprocess.Popen(cmd, start_new_session=True)
        config["process"] = process
        config["status"] = self.STATUS_RUNNING
 
        # 감시 스레드 시작 — 이 노드가 예기치 않게 죽으면 알려줌
        threading.Thread(
            target=self._watch_process,
            args=(index, process),
            daemon=True
        ).start()
 
        self.update_ui_signal.emit(index, True)
 
    def start_node(self, index):
        if self.nodes_config[index]["process"] is None:
            # 에러 상태였다면 초기화 후 재시작
            self.nodes_config[index]["status"] = self.STATUS_STOPPED
            self._run_node_process(index)
 
    def stop_node(self, index):
        threading.Thread(target=self._stop_node_impl, args=(index,), daemon=True).start()
 
    def _stop_node_impl(self, index):
        config = self.nodes_config[index]
        if config["process"] is None:
            return
 
        self._log(f"종료 중: {config['name']}", "info")
 
        if config["exec"] == "serial_sender_node":
            self._send_zero_motion_command()
            time.sleep(0.3)
 
        process = config["process"]
        # 먼저 None 으로 표시해야 _watch_process 가 정상 종료로 인식함
        config["process"] = None
        config["status"] = self.STATUS_STOPPED
 
        try:
            os.killpg(os.getpgid(process.pid), signal.SIGINT)
            process.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(process.pid), signal.SIGKILL)
            process.wait()
        except ProcessLookupError:
            pass
 
        self._log(f"종료 완료: {config['name']}", "info")
        self.update_ui_signal.emit(index, False)
 
    def _send_zero_motion_command(self):
        from interfaces_pkg.msg import MotionCommand
        from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
 
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )
        pub = self.ros_node.create_publisher(MotionCommand, 'topic_control_signal', qos)
        msg = MotionCommand()
        msg.steering = 0
        msg.left_speed = 0
        msg.right_speed = 0
        pub.publish(msg)
        self.ros_node.destroy_publisher(pub)
 
    def stop_all_nodes(self):
        self.btn_stop_all.setEnabled(False)
        threading.Thread(target=self._stop_all_thread, daemon=True).start()
 
    def _stop_all_thread(self):
        for i in reversed(range(len(self.nodes_config))):
            self._stop_node_impl(i)
        self.btn_start_all.setEnabled(True)
        self.btn_stop_all.setEnabled(True)
        self._log("전체 종료 완료.", "ok")
 
    def closeEvent(self, event):
        event.ignore()
        def _shutdown():
            for i in reversed(range(len(self.nodes_config))):
                self._stop_node_impl(i)
            QApplication.quit()
        threading.Thread(target=_shutdown, daemon=True).start()
 
# ==========================================
# [3] Main 함수
# ==========================================
def main(args=None):
    rclpy.init(args=args)
    ros_node = ControlPanelNode()
 
    spin_thread = threading.Thread(target=rclpy.spin, args=(ros_node,), daemon=True)
    spin_thread.start()
 
    app = QApplication(sys.argv)
    window = MainWindow(ros_node)
    window.show()
 
    sys.exit(app.exec_())
 
    ros_node.destroy_node()
    rclpy.shutdown()
 
if __name__ == '__main__':
    main()
 