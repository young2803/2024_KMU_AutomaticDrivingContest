#!/usr/bin/env python
#-- coding:utf-8 --
####################################################################
# 프로그램이름 : parking.py
# 코드작성팀명 : 타 이거
####################################################################

#=============================================
# 함께 사용되는 각종 파이썬 패키지들의 import 선언부
#=============================================
import pygame
import rospy
from xycar_msgs.msg import xycar_motor
from planning_path_reeds_shepp import *
from reed_shepp_utils import *
from PIDcontrol import track_one_step, init_PID

#=============================================
# 모터 토픽을 발행할 것임을 선언
#============================================= 
motor_pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
xycar_msg = xycar_motor()

#=============================================
# 프로그램에서 사용할 변수, 저장공간 선언부
#=============================================
rx, ry = [], []
rxy = []
e_index = 0 # 현재 element의 인덱스
paths_length = [] # 각 element의 경로 갯수
dir_change = 1 # 현제 element와 다음 element의 방향전환 존재 여부 --> margin_index를 추가할지 아니면 0으로 할지 결정




#=============================================
# 프로그램에서 사용할 상수 선언부
#=============================================
AR = (1142, 62) # AR 태그의 위치
P_ENTRY = (1036, 162) # 주차라인 진입 시점의 좌표
P_END = (1129, 69) # 주차라인 끝의 좌표
MAP = (1200, 850) # 맵 사이즈

ENTRY_MARGIN = 70 # 도착지점 추가 마진 거리

# 전면주차, 후면 주차 여부
IS_FRONT = 1 # 1: 전면 주차, 0: 후면주차
EPSILON_X = -5 # 전후 방향 허용 오차, 좀더 들어가도 되므로 음수까지(중앙을 지나쳐야) 가야 완벽히 초록불이 들어옴

# 경로 생성시 참조할 차량의 회전 반경
TURNING_RADIUS = 270 # 회전 반경

# 장애물(벽 및 주차 벽)의 부피 추가
padding = 100 

# 마진 설정
MARGIN_LENGTH = 150 # 끝부분 마진 거리
MARGIN_INDEX = 40 # 마진 인덱스

# # PID 오류 탐색 원의 크기
DIAMETER = 100

# PID 값 1
# Kp = 0.5
# Ki = 0.0000001
# Kd = 0.001
# PID 값 2
# Kp = 0.5
# Ki = 0.006
# Kd = 0.01
# PID 값 3
# Kp = 0.5
# Ki = 0.0000001
# Kd = 0.001
# PID 값 4
# Kp = 0.5
# Ki = 0.0000001
# Kd = 0.001
# PID 
# Kp = 0.5
# Ki = 0.007
# Kd = 0.0015

# PID 값 최종
Kp = 0.9999
Ki = 0.00001
Kd = 0.0000015

#=============================================
# 모터 토픽을 발행하는 함수
# 입력으로 받은 angle과 speed 값을
# 모터 토픽에 옮겨 담은 후에 토픽을 발행함.
#=============================================
def drive(angle, speed):
    xycar_msg.angle = int(angle)
    xycar_msg.speed = int(speed)
    motor_pub.publish(xycar_msg)

#=============================================
# 경로를 생성하는 함수
# 차량의 시작위치 sx, sy, 시작각도 syaw
# 최대가속도 max_acceleration, 단위시간 dt 를 전달받고
# 경로를 리스트를 생성하여 반환한다.
#=============================================
def planning(sx, sy, syaw, max_acceleration, dt):
    global rx, ry, rxy, e_index, paths_length, dir_change
    # 변수 초기화
    init_PID()
    speed = 50
    e_index = 0
    paths_length = []
    rx, ry = [], []  # 리스트 초기화
    dir_change = 1

    print("Start Planning")
    
    # 도작 위치를 entry가기 전 entry_margin 만큼 늘린 지점으로 해서 주차를 똑바로 댈 수 있도록
    theta = np.arctan2(P_END[1] - P_ENTRY[1], P_END[0] - P_ENTRY[0])
    new_goal = (P_ENTRY[0] - ENTRY_MARGIN * np.cos(theta), P_ENTRY[1] - ENTRY_MARGIN * np.sin(theta))
    
    # 차량 기본 위치 정보 q들
    deg_car = rad2deg(np.deg2rad(syaw+90-360))
    q_car = np.array([sx, sy, deg_car])
    deg_park = rad2deg(np.arctan2(new_goal[1] - P_END[1], new_goal[0] - P_END[0])) + 180 * IS_FRONT
    q_park = np.array([new_goal[0], new_goal[1], deg_park])

    #--------reed shepp-----------#
    # 입력값 스케일 조정, turning_radius = 1 이 될 수 있도록, 함수에서 그렇게 사용하므로
    q_car = scale(q_car, TURNING_RADIUS)
    q_park = scale(q_park, TURNING_RADIUS)

    rxy = get_optimal_path(q_car, q_park, MAP, P_ENTRY, P_END, padding, TURNING_RADIUS)


    for xyi in rxy:
        # 마지막 부분일때
        if not xyi[0] is None:
            # 스케일 복구
            rx.extend(unscale(xyi[0], TURNING_RADIUS))
            ry.extend(unscale(xyi[1], TURNING_RADIUS))

            paths_length.append(len(xyi[0])-1) # 경로의 점 개수


    return rx, ry

#=============================================
# 생성된 경로를 따라가는 함수
# 파이게임 screen, 현재위치 x,y 현재각도, yaw
# 현재속도 velocity, 최대가속도 max_acceleration 단위시간 dt 를 전달받고
# 각도와 속도를 결정하여 주행한다.
#=============================================
def tracking(screen, x, y, yaw, current_speed, max_acceleration, dt):
    #global rx, ry, Kp, Ki, Kd, DIAMETER, D, dir_change_exist, dir_changed, speed, close_tolerance
    global rxy, e_index, paths_length, dir_change

    # 값들 전처리
    direction = np.deg2rad(-yaw)
    velocity = np.array([np.cos(direction), np.sin(direction)], dtype=float) * current_speed

    
    # 차량이 주차 중심에 충분히 접근하면 종료
    epsilon_y = 19 # 좌우 방향 허용 오차 거리
    # 차량 좌표계 기준으로 주차 중심 좌표 구하고 정면 거리 구하기
    # 자동차 기준 좌표계에서 주차 중심의 x값으로 거리를 측정 하므로 주차 중심의 각도는 안중요해서 0으로 뒀음
    middle_point_basis_car = change_of_basis((x, y, direction), ((P_END[0] + P_ENTRY[0])/2, (P_END[1] + P_ENTRY[1])/2, 0))
    x_ = middle_point_basis_car[0]
    y_ = middle_point_basis_car[1]

    print(x_, y_)
    # 주차 완료 일때
    if x_ < EPSILON_X and abs(y_) < epsilon_y:
        print("주차 완료~~~~~~!! ^^")
        drive(0, 0)
        
    # 주차 중일때
    else:
        # element 단위로 움직이기
        # 마진 추가
        if e_index < len(rxy)-1: # 마지막 들어가는 부분 빼고
            rxi, ryi, margin_max_index = get_list_margin((unscale(rxy[e_index][0], TURNING_RADIUS), unscale(rxy[e_index][1], TURNING_RADIUS)), MARGIN_LENGTH)
            rxi = np.concatenate((unscale(rxy[e_index][0], TURNING_RADIUS), rxi))
            ryi = np.concatenate((unscale(rxy[e_index][1], TURNING_RADIUS), ryi))
        else:
            rxi = [P_ENTRY[0], P_END[0]]
            ryi = [P_ENTRY[1], P_END[1]]
    
    
    
        # 움직임 계산
        u, sel_p = track_one_step([x, y], velocity, [rxi, ryi], Kp, Ki, Kd, DIAMETER, dt)
        if e_index < len(rxy) - 1:
            speed = 50 * rxy[e_index][2] # 차량의 방향을 설정
            u *= rxy[e_index][2] # 차량의 조향을 설정, 후진할땐 핸들을 반대로
        else:
            speed = 50 * rxy[-1][2]  # 차량의 방향을 설정
            u *= rxy[-1][2]  # 차량의 조향을 설정, 후진할땐 핸들을 반대로
    
    
        
        if e_index < len(rxy)-1:
            if rxy[e_index][2] != rxy[e_index+1][2]: # 방향 값이 다른지
                dir_change = 1
            else:
                dir_change = 0
    
    
    
            # 다음 인덱스 전환 감지
            epsilon = 20 # 도착 감지 허용 오차
            if np.linalg.norm(np.array([x, y]) - np.array([rxi[paths_length[e_index] + MARGIN_INDEX*dir_change], ryi[paths_length[e_index] + MARGIN_INDEX*dir_change]])) < epsilon:
                e_index += 1
        else:
            e_index += 1
    
    
    
        # 차량 이동
        drive(u, speed)
        
    
        #================================시각화 부분================================================================================
        # pid 탐색 반경 표시
        pygame.draw.circle(screen, (255,0,0), [x, y], DIAMETER, width=2)
        # 차량 방향 표시
        pygame.draw.line(screen, (0, 155, 155), [x, y], [x, y] + np.array([np.cos(direction), np.sin(direction)]) * DIAMETER, width=2)
        pygame.draw.line(screen, (0, 155, 155), [x, y], [x, y] - np.array([np.cos(direction), np.sin(direction)]) * DIAMETER, width=2)
        # 에러계산 선택점 표시
        pygame.draw.circle(screen, (0, 255, 0), sel_p, 10)
        # 현제 element 및 margin 표시
        for a, b in zip(rxi, ryi):
            pygame.draw.circle(screen, (0, 255, 0), [a, b], 2)
    
