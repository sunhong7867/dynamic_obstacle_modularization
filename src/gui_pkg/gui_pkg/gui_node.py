import sys
import threading
import subprocess
import time
import rclpy
from rclpy.node import Node

# PyQt5 임포트 (QSlider 등 불필요한 위젯 제거)
from PyQt5.QtWidgets import (QApplication, QWidget, QVBoxLayout, QHBoxLayout, 
                             QLabel, QPushButton, QMessageBox, QFrame)
from PyQt5.QtCore import Qt, pyqtSignal

# ==========================================
# [1] ROS2 Node 클래스
# ==========================================
class ControlPanelNode(Node):
    def __init__(self):
        super().__init__('gui_node')
        self.get_logger().info("런처 전용 GUI 노드가 시작되었습니다!")

# ==========================================
# [2] PyQt5 GUI 창 클래스 (런처 기능)
# ==========================================
class MainWindow(QWidget):
    # 백그라운드 스레드에서 GUI 업데이트를 안전하게 하기 위한 시그널
    update_ui_signal = pyqtSignal(int, bool)

    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        
        # 실행할 노드 리스트 (순서 중요!)
        self.nodes_config = [
            {"name": "1. 카메라 (Image Publisher)", "pkg": "camera_perception_pkg", "exec": "image_publisher_node", "process": None},
            {"name": "2. YOLOv8 인식", "pkg": "camera_perception_pkg", "exec": "yolov8_node", "process": None},
            {"name": "3. 차선 추출 (Lane Info)", "pkg": "camera_perception_pkg", "exec": "lane_info_extractor_node", "process": None},
            {"name": "4. 경로 계획 (Path Planner)", "pkg": "decision_making_pkg", "exec": "path_planner_node", "process": None},
            {"name": "5. 아두이노 통신 (Serial Sender)", "pkg": "serial_communication_pkg", "exec": "serial_sender_node", "process": None},
            {"name": "6. 모션 제어 (Motion Planner)", "pkg": "decision_making_pkg", "exec": "motion_planner_node", "process": None}
        ]
        
        self.buttons_start = []
        self.buttons_stop = []
        self.labels_status = []
        
        self.update_ui_signal.connect(self.update_node_status_ui)
        self.initUI()

    def initUI(self):
        self.setWindowTitle('자율주행 시스템 통합 런처')
        self.resize(500, 400) # 속도 제어 창이 빠졌으니 높이를 살짝 줄였어요!
        main_layout = QVBoxLayout()

        # --- [상단] 시스템 런처 컨트롤 ---
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

        # 시리얼 권한 부여 버튼 
        self.btn_serial_fix = QPushButton('🔧 시리얼 포트/노드 권한 문제 해결 (sudo)', self)
        self.btn_serial_fix.clicked.connect(self.fix_serial_permission)
        main_layout.addWidget(self.btn_serial_fix)

        # 개별 노드 컨트롤 목록 생성
        for i, config in enumerate(self.nodes_config):
            row_layout = QHBoxLayout()
            
            lbl_name = QLabel(config["name"])
            lbl_name.setFixedWidth(200)
            
            lbl_status = QLabel('🔴 중지됨')
            lbl_status.setFixedWidth(80)
            
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

        self.setLayout(main_layout)

    # ==========================================
    # 로직: Wi-Fi 및 노드 관리
    # ==========================================
    def check_wifi_is_on(self):
        try:
            output = subprocess.check_output(['nmcli', 'radio', 'wifi'], text=True).strip()
            return output == 'enabled'
        except Exception as e:
            self.ros_node.get_logger().error(f"Wi-Fi 상태 확인 불가: {e}")
            return False 

    def fix_serial_permission(self):
        try:
            subprocess.Popen(['gnome-terminal', '--', 'bash', '-c', 
                              'echo "시리얼 포트 및 노드 권한을 부여합니다."; sudo chmod 666 /dev/ttyACM0; sudo chmod 777 -R ~/dynamic_obstacle_ws/src/serial_communication_pkg; echo "완료! 창을 닫으세요."; exec bash'])
        except Exception as e:
            QMessageBox.warning(self, "오류", f"권한 명령 실행 실패: {e}")

    def start_sequence(self):
        # 1. 와이파이 켜져있는지 검사!
        if self.check_wifi_is_on():
            QMessageBox.critical(self, "⚠️ Wi-Fi 경고 ⚠️", 
                                 "현재 Wi-Fi가 켜져 있습니다!\n학생들의 토픽과 충돌할 위험이 있습니다.\n\n우측 상단 메뉴에서 Wi-Fi를 끄고 다시 시도해 주세요.")
            return

        reply = QMessageBox.question(self, '순차 실행', '와이파이가 꺼져있습니다. 모든 노드를 순서대로 켤까요?\n(각 노드별로 1초씩 대기합니다)',
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
        
        self.ros_node.get_logger().info("모든 노드 실행 완료!")

    def _run_node_process(self, index):
        config = self.nodes_config[index]
        cmd = ['ros2', 'run', config['pkg'], config['exec']]
        
        self.ros_node.get_logger().info(f"실행 중: {config['name']}")
        process = subprocess.Popen(cmd)
        config["process"] = process
        
        self.update_ui_signal.emit(index, True)

    def start_node(self, index):
        if self.nodes_config[index]["process"] is None:
            self._run_node_process(index)

    def stop_node(self, index):
        config = self.nodes_config[index]
        if config["process"] is not None:
            self.ros_node.get_logger().info(f"종료 중: {config['name']}")
            config["process"].terminate() 
            config["process"] = None
            
            self.update_ui_signal.emit(index, False)

    def stop_all_nodes(self):
        for i in range(len(self.nodes_config)):
            self.stop_node(i)
        self.btn_start_all.setEnabled(True)

    def update_node_status_ui(self, index, is_running):
        if is_running:
            self.labels_status[index].setText('🟢 실행 중')
            self.buttons_start[index].setEnabled(False)
            self.buttons_stop[index].setEnabled(True)
        else:
            self.labels_status[index].setText('🔴 중지됨')
            self.buttons_start[index].setEnabled(True)
            self.buttons_stop[index].setEnabled(False)

    def closeEvent(self, event):
        self.stop_all_nodes()
        event.accept()

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