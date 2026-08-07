"""
control_node - 순찰 관제 인터페이스

명령줄에서 순찰을 시작하거나 중지한다.
patrol_node 의 /set_patrol 서비스를 호출하는 클라이언트.

사용:
    ros2 run campus_patrol control_node start
    ros2 run campus_patrol control_node stop
"""

import sys

import rclpy
from rclpy.node import Node

from campus_patrol_msgs.srv import SetPatrol


class ControlNode(Node):

    def __init__(self):
        super().__init__('stopstart_control_node')
        self.client = self.create_client(SetPatrol, 'set_patrol')

    def send(self, start):
        # 서비스 서버가 뜰 때까지 최대 5초 대기
        if not self.client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(
                '/set_patrol 서비스를 찾을 수 없습니다. patrol_node 가 실행 중인가요?')
            return

        request = SetPatrol.Request()
        request.start = start

        # 요청을 보내고 응답이 올 때까지 기다린다
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        response = future.result()
        if response.success:
            self.get_logger().info(f'✓ {response.message}')
        else:
            self.get_logger().warn(f'✗ {response.message}')


def main(args=None):
    rclpy.init(args=args)

    # 명령줄 인자 확인
    argv = sys.argv[1:]
    if not argv or argv[0] not in ('start', 'stop'):
        print('사용법: ros2 run campus_patrol stopstart_control_node [start|stop]')
        rclpy.shutdown()
        return

    node = ControlNode()
    node.send(argv[0] == 'start')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()