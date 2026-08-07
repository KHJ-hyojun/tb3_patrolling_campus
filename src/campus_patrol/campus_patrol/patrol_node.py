"""
patrol_node - 순찰 노드 (1단계 : 목표 하나만 보내기)

지금은 Nav2에 좌표 하나를 보내고 결과를 받는 것까지만 한다.
순회, 재시도, 상태 발행은 다음 단계에서 붙인다.
"""

import rclpy
import time
from rclpy.node import Node
from rclpy.action import ActionClient

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped

from campus_patrol_msgs.msg import PatrolStatus
from campus_patrol_msgs.srv import SetPatrol

WAYPOINTS = [
    {'name': '정문',       'x':  1.81, 'y': -1.23},
    {'name': '공대 앞',     'x':  1.33, 'y':  1.58},
    {'name': '생명대 앞',   'x': -1.70, 'y':  1.20},
    {'name': '예술대 앞',   'x': -1.74, 'y': -1.06},
]

class PatrolNode(Node):
    def __init__(self):
        super().__init__('patrol_node')

        # Nav2의 NavigateToPose 액션 서버에 연결할 클라이언트
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.current_index = 0

        # 상태 발행
        self.status_pub = self.create_publisher(PatrolStatus, 'patrol_status', 10)
        self.state = 'IDLE'
        self.retry_count = 0
        self.cancelling = False
        self.finished = False

        # 재시도 로직
        self.max_retries = 3
        self.goal_timeout = 40.0    # 한 지점에 20초 넘으면 포기
        self.goal_handle = None     # 취소하려면 필요
        self.timeout_timer = None
        self.retry_timer = None

        # 1초마다 현재 상태를 발행
        self.create_timer(1.0, self.publish_status)

        # 목표 전송!
        self.create_service(SetPatrol, 'set_patrol', self.on_set_patrol)

        self.get_logger().info('Nav2 액션 서버를 기다리는 중...')
        self.nav_client.wait_for_server()
        self.get_logger().info('Nav2 연결됨')
        self.get_logger().info('대기 중 — /set_patrol 서비스로 시작하세요')

    def on_set_patrol(self, request, response):
        # 순찰 시작 / 중지 요청 처리
        if request.start:
            return self.start_patrol(response)
        else:
            return self.stop_patrol(response)

    def start_patrol(self, response):
        if self.state == 'PATROLLING':
            response.success = False
            response.message = '이미 순찰 중입니다.'
            return response

        if self.state == 'PAUSED':
            self.get_logger().info(
                f'순찰 재개 ({WAYPOINTS[self.current_index]["name"]}부터)'
            )
            msg = f'{WAYPOINTS[self.current_index]["name"]}부터 재개'
        else:
            self.current_index = 0
            self.patrol_start_time = time.time()
            self.get_logger().info('순찰 시작')
            msg = f'순찰 시작 (지점 {len(WAYPOINTS)}곳)'

        self.retry_count = 0
        self.finished = False
        self.cancelling = False

        self.send_goal(WAYPOINTS[self.current_index])

        response.success = True
        response.message = msg
        return response

    def stop_patrol(self, response):
        if self.state != 'PATROLLING':
            response.success = False
            response.message = f'순찰 중이 아닙니다 (현재: {self.state})'
            return response

        # 진행 중인 목표와 타이머 정리
        self.cancel_timeout()
        self.cancel_retry_timer()
        if self.goal_handle is not None:
            self.cancelling = True
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None

        self.finished = True
        self.state = 'PAUSED'
        self.get_logger().warn(
            f'순찰 중지 (현재 시험: {WAYPOINTS[self.current_index]["name"]})'
        )

        response.success = True
        response.message = f'{WAYPOINTS[self.current_index]["name"]}에서 중지'
        return response

    # 목표 전송
    def send_goal(self, waypoint):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = waypoint['x']
        goal.pose.pose.position.y = waypoint['y']
        goal.pose.pose.orientation.w = 1.0 #회전없음
        self.goal_start_time = time.time()

        self.state = 'PATROLLING'
        self.get_logger().info(
            f"{waypoint['name']} 으로 출발 "
            f"(x={waypoint['x']}, y={waypoint['y']})"
        )

        # 비동기 전송. 여기서 멈추지 않고 바로 반환
        send_future = self.nav_client.send_goal_async(goal)
        send_future.add_done_callback(self.on_goal_response)

        # 시간 초과 감시 시작
        self.start_timeout()

    # 액션 콜백 - 두단계
    def on_goal_response(self, future):
        # 1단계 : Nav2가 목표를 받아줬는지
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.cancel_timeout()
            self.get_logger().error('Nav2가 목표를 거부했습니다.')
            self.handle_failure('목표 거부')
            return

        self.goal_handle = goal_handle
        self.get_logger().info('목표 수락됨. 이동중')

        # 결과가 나오면 알려달라고 등록
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.on_result)

    def on_result(self, future):
        # 2단계 : 주행이 끝난 뒤

        if self.finished:
            return

        self.cancel_timeout()
        status = future.result().status

        if status == 5 and self.cancelling:
            self.cancelling = False
            self.get_logger().info('이전 목표 취소 확인')
            return

        self.goal_handle = None
        name = WAYPOINTS[self.current_index]['name']

        if status == 4:
            self.get_logger().info(f'{name} 도착')
            self.retry_count = 0
            self.go_next()
        else:
            self.handle_failure(f'경로 실패 (status={status})')


    def go_next(self):
        # 다음 지점으로. 끝까지 갔으면 순찰 종료.
        self.current_index += 1

        if self.current_index >= len(WAYPOINTS):
            self.state = 'DONE'
            self.finished = True
            self.print_summary()
            return

        self.send_goal(WAYPOINTS[self.current_index])

    def publish_status(self):
        # 현재 순찰 상태를 /patrol_status 로 발행.
        msg = PatrolStatus()
        msg.state = self.state
        msg.current_index = self.current_index
        msg.total = len(WAYPOINTS)
        msg.retry_count = self.retry_count

        # 인덱스가 범위를 벗어날 수 있으니 확인 후 이름 설정
        if 0 <= self.current_index < len(WAYPOINTS):
            msg.current_waypoint = WAYPOINTS[self.current_index]['name']
        else:
            msg.current_waypoint = ''

        self.status_pub.publish(msg)

    def start_timeout(self):
        # 목표 도달 감시 타이머 시작.
        self.cancel_timeout()
        self.timeout_timer = self.create_timer(self.goal_timeout, self.on_timeout)

    def cancel_timeout(self):
        # 감시 타이머 해제.
        if self.timeout_timer is not None:
            self.timeout_timer.cancel()
            self.destroy_timer(self.timeout_timer)
            self.timeout_timer = None

    def on_timeout(self):
        # 제한 시간 안에 결과가 안 온 결과
        elapsed = time.time() - self.goal_start_time
        self.get_logger().warn(
            f"{WAYPOINTS[self.current_index]['name']} 시간 초과"
            f"(설정 {self.goal_timeout}초 / 실제 {elapsed:.1f} 초)"
        )
        self.cancel_timeout()

        # Nav2에 취소를 요청
        if self.goal_handle is not None:
            self.cancelling = True
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None
        self.handle_failure('시간 초과')

    def handle_failure(self, reason):
        # 도달 실패. 한도 안이면 재시도, 넘으면 건너뛰기

        # 그전에 이전 목표가 남아있으면 정리
        if self.goal_handle is not None:
            self.cancelling = True
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None

        self.retry_count += 1
        name = WAYPOINTS[self.current_index]['name']

        if self.retry_count <= self.max_retries:
            self.get_logger().warn(
                f'{name} 재시도 {self.retry_count}/{self.max_retries} - {reason}'
            )
            self.schedule_retry()
        else:
            self.get_logger().error(
                f'{name} 재시도 한도 초과 - 건너띰 ({reason})'
            )
            self.retry_count = 0
            self.go_next()

    def schedule_retry(self):
        # 잠시뒤 재시도 (Nav2 정리 시간 확보)
        self.cancel_retry_timer() # 이전 예약이 있으면 지우려고
        self.retry_timer = self.create_timer(2.0, self.do_retry)

    def do_retry(self):
        self.cancel_retry_timer() # 이전 예약이 있으면 지우려고
        if self.finished:
            return
        self.send_goal(WAYPOINTS[self.current_index])

    def cancel_retry_timer(self):
        if self.retry_timer is not None:
            self.retry_timer.cancel()
            self.destroy_timer(self.retry_timer)
            self.retry_timer = None

    def print_summary(self):
        # 순찰 종료시 결과 요약.
        elapsed = time.time() - self.patrol_start_time
        self.get_logger().info('=' * 30)
        self.get_logger().info('순찰 완료')
        self.get_logger().info(f'   총 지점 {len(WAYPOINTS)}곳')
        self.get_logger().info(f'   총 소요 {elapsed: .1f}초')
        self.get_logger().info('=' * 30)


def main(args = None):
    rclpy.init(args=args)
    node = PatrolNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()