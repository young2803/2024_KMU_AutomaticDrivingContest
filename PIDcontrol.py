import numpy as np


def P(Kp, error):
    return Kp * error


def I(Ki, error, accuE, dt):
    return Ki * (accuE + dt * error)


def D(Kd, error, bef_error, dt):
    return Kd * (error - bef_error) / dt


# 벡터 회전 함수
def rotate_by_vec(vec, ref_vec):
    theta = np.arctan2(ref_vec[1], ref_vec[0])
    R = np.array([[np.cos(-theta), -np.sin(-theta)],
                  [np.sin(-theta), np.cos(-theta)]])

    return R @ vec






def get_error(xy, lxy, velo, di):
    # 교점 후보 허용 오차
    tolerance = 10

    # 모든 path 점들과 거리 구하기
    dx = lxy[0] - xy[0]
    dy = lxy[1] - xy[1]
    distance = np.sqrt(dx ** 2 + dy ** 2)

    # 초기값 초기화
    distance_largers = distance.copy()
    distance_smallers = distance.copy()
    distance_on = distance.copy()

    distance_largers[(distance <= di + tolerance)] = 0 # 원 보다 멀리 있는 것들
    distance_smallers[(distance >= di - tolerance)] = 0 # 원 보다 가까이 있는 것들
    # 원 위의 점들
    distance_on[(distance > di + tolerance)] = 0
    distance_on[(distance < di - tolerance)] = 0

    if np.min(distance_largers) > di - tolerance: # 점이 밖에 있을때
        print("교점이 없습니다. 가장 가까운 점을 에러 점으로 사용합니다.")
        sel_p_index = np.max(np.argsort(distance_largers)[1:4])
        selected_point = lxy.T[sel_p_index]


    else: # 점이 원 위나 안에 있을때
        if len(np.where(np.where(distance_smallers != 0)[0] >= distance.shape[0]-2)[0]): # element 추적의 마지막 점이 원 안에 있으면 (마지막에 가까운점)
            print("element의 마지막점이 탐색원 내부에 존재합니다.")
            sel_p_index = np.max(distance.shape[0]-1)
            selected_point = lxy.T[sel_p_index]
        else: # 점이 위에있을 때
            print("교점이 있습니다. element의 가장 나중 점을 에러 점으로 사용합니다.")
            sel_p_index = np.max(np.argsort(distance_on)[-4:-1])
            selected_point = lxy.T[sel_p_index]



    # 회전 시키
    car_to_point = xy - selected_point
    rotation_vector = rotate_by_vec(car_to_point, velo)


    error = rotation_vector[1]

    return error, selected_point



# 대충 테스트 해볼 수 있는 pid 값
# Kp = 0.009
# Ki = 0.001
# Kd = 0.00002




accuE = bef_error = 0

def init_PID():
    global accuE, bef_error
    accuE = bef_error = 0

def track_one_step(pos, velo, lxy, Kp, Ki, Kd, diameter, dt):
    global accuE, bef_error
    pos = np.array(pos, dtype=float)
    lxy = np.array(lxy, dtype=float)

    error, sel_p = get_error(pos, lxy, velo, diameter)


    #==P==
    p = P(Kp, error)
    #==I==
    i = I(Ki, error, accuE, dt)
    accuE += dt * error
    #==D==
    d = D(Kd, error, bef_error, dt)
    bef_error = error

    #==컨트롤 하기==
    u = p + d + i

    return -u, sel_p
