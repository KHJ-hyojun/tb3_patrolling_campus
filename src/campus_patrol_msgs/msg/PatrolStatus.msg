# 순찰 진행 상황
# 발행 : patrol_node -> 토픽 /patrol_status
# 구독 : logger_node

string state            # IDLE : 대기중
                        # PATROLLING : 순찰 진행 중
                        # DONE : 순찰 종료
                        
string current_waypoint # 지금 향하는 지점 이름 (예 : "공대 앞")
int32 current_index     # 그 지점의 순번 (0번부터)
int32 total             # 전체 지점 개수
int32 retry_count       # 현재 지점에서 재시도한 횟수 (0이면 정상)