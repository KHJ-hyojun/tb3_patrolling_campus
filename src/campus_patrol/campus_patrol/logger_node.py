"""
logger_node - 순찰 기록 노드

/patrol_status를 구독해서 지점이 바뀔 때마다 CSV에 한 줄씩 기록한다.
순찰이 끝나면 요약을 출력한다.

patrol_node 와 분리한 이유:
    순찰 로직과 기록은 서로 다른 관심사다.
    로거를 껐다 켜도 순찰에는 영향이 없어야 한다.
"""
import csv
import os

import time
from datetime import datetime
import rclpy
from rclpy.node import Node

from campus_patrol_msgs.msg import PatrolStatus

class LoggerNode(Node):

    def __init__(self):
        super().__init__('logger_node')

        # patrol_node 가 발행하는 상태를 구독
        self.sub = self.create_subscription(
            PatrolStatus,          # 메시지 타입
            'patrol_status',       # 토픽 이름
            self.on_status,        # 메시지가 올 때 부를 함수
            10                     # 큐 크기
        )

        # 직전 상태 - 변화를 감지하려면 이전 값 들고 있어야 함
        self.prev_state = None
        self.prev_index = None
        self.prev_name = ''
        self.prev_retry = 0

        # 기록 보관
        self.records = []
        self.waypoint_start = None  # 현재 지점 시작 시각
        self.patrol_start = None    # 순찰 전체 시작 시각

        # 저장 경로
        self.log_dir = os.path.expanduser('~/campus_patrol_logs')
        os.makedirs(self.log_dir, exist_ok=True)


        self.get_logger().info('logger_node 시작 - 상태 수신 대기 중')

    def on_status(self, msg):
        # 순찰 시작 / 재개
        if self.prev_state != 'PATROLLING' and msg.state == 'PATROLLING':
            if self.prev_state == 'PAUSED':
                self.get_logger().info(
                    f'순찰 재개 -> {msg.current_waypoint} 부터'
                )
            else:
                self.get_logger().info('순찰 시작 감지')
                self.records = []
                self.patrol_start = time.time()
            self.waypoint_start = time.time()

        # 지점 바뀜
        if msg.state == 'PATROLLING' and msg.current_index != self.prev_index:
            if self.prev_index is not None:
                self.add_record()

            self.get_logger().info(
                f'지점 변경 -> {msg.current_waypoint} '
                f'({msg.current_index + 1}/{msg.total})'
            )
            self.waypoint_start = time.time()

        # 순찰 완료 / 중지
        if self.prev_state == 'PATROLLING':
            if msg.state == 'PAUSED':
                self.get_logger().warn(
                    f'순찰 중지 ({msg.current_waypoint} 에서)'
                )
            elif msg.state == 'DONE':
                self.add_record()   # 마지막 지점 기록
                self.get_logger().info('순찰 완료')
                self.show_records()
                self.save_csv()

        # 다음 비교를 위해 현재 값을 저장
        self.prev_state = msg.state
        self.prev_index = msg.current_index
        self.prev_name = msg.current_waypoint
        self.prev_retry = msg.retry_count

    def add_record(self):
        # 직전 지점을 기록에 추가.
        if self.waypoint_start is None:
            return
        duration = time.time() - self.waypoint_start
        self.records.append({
            '시각': datetime.now().strftime('%H:%M:%S'), #Hour, Minute, Second
            '순번': self.prev_index,
            '지점': self.prev_name,
            '소요시간': round(duration, 1),
            '재시도': self.prev_retry,
        })

    def show_records(self):
        # 모인 기록을 화면에 출력 (CSV 저장 전 확인용)
        self.get_logger().info('=' * 30)
        for r in self.records:
            self.get_logger().info(
                f"{r['시각']}   {r['지점']}    "
                f"{r['소요시간']}초     재시도 {r['재시도']}회"
            )
        self.get_logger().info('=' * 30)

    def save_csv(self):
        # 기록을 csv 파일로 저장
        if not self.records:
            self.get_logger().warn('기록이 없어 저장하지 않습니다.')
            return

        stamp = datetime.now().strftime('%Y%m%d_%H%M%S') #Year, Month, Day, Hour, Minute, Second
        path = os.path.join(self.log_dir, f'patrol_{stamp}.csv')

        with open(path, 'w', newline ='', encoding='utf-8-sig') as f:
            writer = csv.DictWriter(f, fieldnames=list(self.records[0].keys()))
            writer.writeheader()
            writer.writerows(self.records)
        self.get_logger().info(f'로그 파일 저장 완료: {path}')


def main(args=None):
    rclpy.init(args=args)
    node = LoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()




