import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy
from interfaces_pkg.msg import PathPlanningResult, MotionCommand
from .lib import decision_making_func_lib as DMFL

# ==========================================
# [1] Parameter Configuration Module
class MotionPlannerConfig:
    def __init__(self):
        # --- [Topic Names] ---
        self.sub_path_1_topic = "path_planning_result_1"
        self.sub_path_2_topic = "path_planning_result_2"
        self.pub_topic = "topic_control_signal"

        # --- [Time & Mode] ---
        self.timer_period = 0.1         # 모션 제어 주기 (초)
        self.cycle_duration = 14.5      # 전진/후진 전환 시간 (초)

        # --- [Speed & Accel] ---
        self.forward_speed = 100        # 전진 목표 속도
        self.backward_speed = -100      # 후진 목표 속도
        self.max_accel_step = 10        # 속도가 부드럽게 변하는 단위

        # --- [Steering Limits] ---
        self.max_angle_forward = 45     # 전진 시 최대 조향각
        self.max_angle_backward = 50    # 후진 시 최대 조향각

        # --- [Calculation Indices] ---
        self.lookahead_index_near = -50 # 경로 기울기 계산용 시작점 인덱스
        self.lookahead_index_far = -1   # 경로 기울기 계산용 끝점 인덱스
# ==========================================

def convert_steeringangle2command(max_target_angle, target_angle):
    f = lambda x: 7 / (max_target_angle**3) * (x**3)
    ret_direction = round(f(target_angle))
    ret_direction = 7 if ret_direction >= 7 else ret_direction
    ret_direction = -7 if ret_direction <= -7 else ret_direction
    return ret_direction


class MotionPlanningNode(Node):
    def __init__(self):
        super().__init__('motion_planner_node')

        # [모듈화] 파라미터 클래스 장착!
        self.cfg = MotionPlannerConfig()

        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )

        # State 변수 초기화
        self.path_data_1 = None
        self.path_data_2 = None

        self.steering_command = 0
        self.left_speed_command = 0
        self.right_speed_command = 0
        self.current_left_speed = 0
        self.current_right_speed = 0

        self.mode = 1  # 0=전진, 1=후진 (시작은 1)
        self.start_time = None
        self.data_ready = False

        # Subscriber 설정
        self.path_sub_1 = self.create_subscription(
            PathPlanningResult, self.cfg.sub_path_1_topic, self.path1_callback, self.qos_profile)
        self.path_sub_2 = self.create_subscription(
            PathPlanningResult, self.cfg.sub_path_2_topic, self.path2_callback, self.qos_profile)

        # Publisher 설정
        self.publisher = self.create_publisher(
            MotionCommand, self.cfg.pub_topic, self.qos_profile)

        self.timer = self.create_timer(self.cfg.timer_period, self.timer_callback)

        self.last_path_time = self.get_clock().now()

    def path1_callback(self, msg: PathPlanningResult):
        self.last_path_time = self.get_clock().now() # 시간 갱신
        self.path_data_1 = list(zip(msg.x_points, msg.y_points))

    def path2_callback(self, msg: PathPlanningResult):
        self.last_path_time = self.get_clock().now() # 시간 갱신
        self.path_data_2 = list(zip(msg.x_points, msg.y_points))

    def update_speed_smoothly(self, target, current):
        # cfg.max_accel_step을 가져와서 사용해요
        if current < target:
            return min(current + self.cfg.max_accel_step, target)
        elif current > target:
            return max(current - self.cfg.max_accel_step, target)
        else:
            return current

    def timer_callback(self):
        # 데이터가 들어올 때까지 대기
        if not self.data_ready:
            if self.path_data_1 is not None and self.path_data_2 is not None:
                self.start_time = self.get_clock().now()
                self.data_ready = True
                self.get_logger().info("경로 데이터 수신 완료. 주행 시작!")
            else:
                return

        elapsed = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9

        # 지정된 시간(cycle_duration)마다 mode 전환
        if elapsed >= self.cfg.cycle_duration:
            self.current_left_speed = 0   # 전환 시점에 속도를 0으로 만들어서 멈췄다가 방향 바꿔서 출발
            self.current_right_speed = 0 
            self.mode = 1 - self.mode
            self.start_time = self.get_clock().now()

        # ----------------------------------------------------
        # [수정된 부분] 데이터 나이 계산 및 3단 분기 처리
        # ----------------------------------------------------
        data_age = (self.get_clock().now() - self.last_path_time).nanoseconds * 1e-9
        
        # 1. 타임아웃 발생 시 (안전 정지)
        if data_age > 1.0:
            self.get_logger().warn(f"차선 데이터 유실! 안전 정지! (데이터 나이: {data_age:.2f}초)")
            self.steering_command = 0
            target_left_speed = 0  # target을 0으로 줘야 부드럽게 감속하면서 멈춥니다
            target_right_speed = 0

        # 2. 경로 데이터가 아예 없는 초기 상태
        elif self.path_data_1 is None or self.path_data_2 is None:
            self.steering_command = 0
            target_left_speed = 0
            target_right_speed = 0
            
        # 3. 정상 주행 상태 (기존 경로 추종 로직)
        else:
            if self.mode == 1: # 후진 모드
                target_slope = DMFL.calculate_slope_between_points(
                    self.path_data_1[self.cfg.lookahead_index_near], 
                    self.path_data_1[self.cfg.lookahead_index_far]
                )
                self.steering_command = convert_steeringangle2command(
                    self.cfg.max_angle_forward, target_slope)
                target_left_speed = self.cfg.forward_speed
                target_right_speed = self.cfg.forward_speed
            else: # 전진 모드
                target_slope = DMFL.calculate_slope_between_points(
                    self.path_data_2[self.cfg.lookahead_index_near], 
                    self.path_data_2[self.cfg.lookahead_index_far]
                )
                self.steering_command = -1 * convert_steeringangle2command(
                    self.cfg.max_angle_backward, target_slope)
                target_left_speed = self.cfg.backward_speed
                target_right_speed = self.cfg.backward_speed
        # ----------------------------------------------------

        # 부드러운 속도 갱신 (target이 0이면 서서히 멈춤)
        self.current_left_speed = self.update_speed_smoothly(target_left_speed, self.current_left_speed)
        self.current_right_speed = self.update_speed_smoothly(target_right_speed, self.current_right_speed)

        self.left_speed_command = self.current_left_speed
        self.right_speed_command = self.current_right_speed

        self.get_logger().info(f"mode: {self.mode}, steering: {self.steering_command}, left_speed: {self.left_speed_command}, right_speed: {self.right_speed_command}")

        motion_command_msg = MotionCommand()
        motion_command_msg.steering = self.steering_command
        motion_command_msg.left_speed = self.left_speed_command
        motion_command_msg.right_speed = self.right_speed_command
        self.publisher.publish(motion_command_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MotionPlanningNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\nshutdown\n\n")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()