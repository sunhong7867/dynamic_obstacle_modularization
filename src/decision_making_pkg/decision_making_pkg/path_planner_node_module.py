import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy
from interfaces_pkg.msg import LaneInfo, PathPlanningResult
import numpy as np
from scipy.interpolate import CubicSpline

# ==========================================
# [1] Parameter Configuration Module
class PathPlannerConfig:
    def __init__(self):
        # --- [Topic Names] ---
        self.sub_lane_1_topic = "yolov8_lane_info_1" # 전방 카메라
        self.sub_lane_2_topic = "yolov8_lane_info_2" # 후방 카메라
        self.pub_path_1_topic = "path_planning_result_1" # 전방 카메라
        self.pub_path_2_topic = "path_planning_result_2" # 후방 카메라

        # --- [Vehicle Parameters] ---
        # 이미지 상에서 차량 앞/뒤 범퍼 중심 픽셀 좌표 (x, y)
        self.car_center_1 = (320, 260)  # 전방 카메라 기준
        self.car_center_2 = (320, 260)  # 후방 카메라 기준

        # --- [Algorithm Parameters] ---
        self.min_target_points = 3      # 경로 계획을 시작할 최소 타겟 점 개수
        self.spline_resolution = 100    # 스플라인으로 부드럽게 생성할 보간 점의 개수
# ==========================================

class PathPlannerNode(Node):
    def __init__(self):
        super().__init__('path_planner_node')

        # [모듈화] 파라미터 덩어리를 불러옵니다!
        self.cfg = PathPlannerConfig()

        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )

        # Subscriber 설정
        self.lane_sub1 = self.create_subscription(
            LaneInfo, self.cfg.sub_lane_1_topic, self.lane_1_callback, self.qos_profile)
        self.lane_sub2 = self.create_subscription(
            LaneInfo, self.cfg.sub_lane_2_topic, self.lane_2_callback, self.qos_profile)

        # Publisher 설정
        self.publisher_1 = self.create_publisher(
            PathPlanningResult, self.cfg.pub_path_1_topic, self.qos_profile)
        self.publisher_2 = self.create_publisher(
            PathPlanningResult, self.cfg.pub_path_2_topic, self.qos_profile)

    def lane_1_callback(self, msg: LaneInfo):
        # 전방 카메라 데이터를 공통 함수로 넘겨요
        self.generate_and_publish_path(msg.target_points, self.cfg.car_center_1, self.publisher_1)

    def lane_2_callback(self, msg: LaneInfo):
        # 후방 카메라 데이터를 공통 함수로 넘겨요
        self.generate_and_publish_path(msg.target_points, self.cfg.car_center_2, self.publisher_2)

    # [모듈화] 중복되던 1번, 2번 함수를 하나로 합쳤어요!
    def generate_and_publish_path(self, target_points, car_center, publisher):
        
        # 타겟 지점이 충분한지 확인
        if len(target_points) < self.cfg.min_target_points:
            self.get_logger().warn("타겟 지점이 부족해서 경로를 만들 수 없습니다.")
            return

        # TargetPoint 객체에서 x, y 값 추출
        x_points = [tp.target_x for tp in target_points]
        y_points = [tp.target_y for tp in target_points]

        # 차량 중심 좌표 추가
        x_points.append(car_center[0])
        y_points.append(car_center[1])

        # y 값을 기준으로 정렬 (스플라인 계산 시 필수!)
        sorted_points = sorted(zip(y_points, x_points), key=lambda point: point[0])
        y_points, x_points = zip(*sorted_points)

        # ── 추가: 중복 y값 제거 ──────────────────────────
        seen_y = set()
        deduped = []
        for y, x in zip(y_points, x_points):
            if y not in seen_y:
                seen_y.add(y)
                deduped.append((y, x))

        if len(deduped) < self.cfg.min_target_points:
            self.get_logger().warn("중복 제거 후 타겟 지점 부족. 경로 생성 건너뜀.")
            return

        y_points, x_points = zip(*deduped)
        # ─────────────────────────────────────────────────

        # 스플라인 보간법 적용
        cs = CubicSpline(y_points, x_points, bc_type='natural')

        # 해상도 파라미터(spline_resolution)에 맞게 새로운 점들 생성
        y_new = np.linspace(min(y_points), max(y_points), self.cfg.spline_resolution)
        x_new = cs(y_new)

        # 메시지에 담아서 퍼블리시
        path_msg = PathPlanningResult()
        path_msg.x_points = list(x_new)
        path_msg.y_points = list(y_new)
        publisher.publish(path_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PathPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\nshutdown\n\n")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()