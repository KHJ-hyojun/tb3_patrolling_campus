"""
monitor_node - 로봇 상태 감지 노드

"""
import time
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class MonitorNode(Node):

    def __init__(self):
        super().__init__('monitor_node')

        # 실제 속도 (로봇이 실제로 내는 속도)
        self.create_subscription(Odometry, 'odom', self.on_odom, 10)

        # 명령 속도 (Nav2가 내리는 명령)
        self.create_subscription(Twist, 'cmd_vel', self.on_cmd_vel, 10)

        self.actual_speed = 0.0
        self.cmd_speed = 0.0

        # 1초마다 비교
        self.create_timer(1.0, self.check)

        self.get_logger().info('Monitor_node 시작')

        # 감시 설정
        self.speed_threshold = 0.15 # 명령-실제 차이가 이 이상이면 이상
        self.speed_duration = 3.0   # 3초 이상 지속돼야 경고
        self.min_cmd_speed = 0.05   # 명령이 이보다 작으면 감시 안 함

        # 감시 상태
        self.abnormal_since = None  # 이상이 시작된 시각
        self.alerted = False        # 경고를 이미 냈는가

        # /scan 구독
        self.create_subscription(LaserScan, 'scan', self.on_scan, 10)

        self.nearest_obstacle = 999.0

        # 장애물 감시 설정
        self.obstacle_threshold = 0.3   # 이보다 가까우면 이상 (m)
        self.obstacle_duration = 1.0    # 짧게 - 충돌은 빨리 알려야 함

        self.obstavle_since = None
        self.obstacle_alerted = False


    def on_odom(self, msg):
        # 실제 속도 저장
        self.actual_speed = msg.twist.twist.linear.x

    def on_cmd_vel(self, msg):
        # 명령 속도를 저장
        self.cmd_speed = msg.linear.x

    def on_scan(self, msg):
        # 라이다 데이터에서 최근접 거리를 뽑음
        # inf / nan 같은 무효값 걸러냄
        valid = [r for r in msg.ranges
                 if msg.range_min < r < msg.range_max]
        self.nearest_obstacle = min(valid) if valid else 999.0

    def check(self):
        now = time.time()
        diff = abs(self.cmd_speed - self.actual_speed)
        
        # 정지 명령 중에는 감시하지 않는다.
        # (멈추라고 했는데 관성로 조금 움직이는 건 정상)
        if abs(self.cmd_speed) < self.min_cmd_speed:
            self.recover(now)
            return

        if diff >= self.speed_threshold:
            # 이상상태
            if self.abnormal_since is None:
                self.abnormal_since = now   # 이상 시작 시각 기록

            elapsed = now - self.abnormal_since

            # 충분히 지속됐고, 아직 경고를 안냈으면 경고
            if elapsed >= self.speed_duration and not self.alerted:
                self.alerted = True
                self.get_logger().warn(
                    f'속도 추종 불량: 명령 {self.cmd_speed:.2f} / '
                    f'실제 {self.actual_speed:.2f} m/s '
                    f'({elapsed:.0f}초 지속)'
                )
        else:
            # 정상으로 돌아옴
            self.recover(now)
        self.check_obstacle(now)


    def recover(self, now):
        # 정상 복귀 처리, 경고를 냈었다면 해제 알림
        if self.alerted:
            duration = now - self.abnormal_since
            self.get_logger().info(f'속도 추종 정상 회복 ({duration:.0f}초 만에)')
        self.abnormal_since = None
        self.alerted = False

    def check_obstacle(self, now):
        if self.nearest_obstacle < self.obstacle_threshold:
            if self.obstacle_since is None:
                self.obstacle_since = now
            elapsed = now - self.obstacle_since
            if elapsed >= self.obstacle_duration and not self.obstacle_alerted:
                self.obstacle_alerted = True
                self.get_logger().warn(
                    f'장애물 근접: {self.nearest_obstacle:.2f} m '
                    f'(임계 {self.obstacle_threshold} m)'
                )
        else:
            if self.obstacle_alerted:
                self.get_logger().info('장애물 벗어남')
            self.obstacle_since = None
            self.obstacle_alerted = False

def main(args=None):
    rclpy.init(args=args)
    node = MonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()