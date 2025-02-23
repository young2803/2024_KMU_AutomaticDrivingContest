import cv2
import numpy as np
import matplotlib.pyplot as plt


def get_hw(img: np.array):
    height, width = img.shape[:2]
    return height, width


# 이미지에서 경로 등변사다리꼴 형태로 4개의 point 지정
# 카메라 위치 따라서 좌표값 수정 필요 >> 임의 지정
# ldx = 좌하점의 x좌표, hl = 사다리꼴 높이, l1 = 사다리꼴 밑변 길이, l2 = 사다리꼴 윗변 길이
# 카메라 위치 결정되면 수정 필요한 부분. 좌표 계산도 바꿔야 할 수도 있음
def get_points(img: np.array, ldx: int, hl: int, l1: int, l2: int):
    h, w = get_hw(img)
    p1 = [ldx + ((l1 - l2)/2), h-hl]  # 좌상
    p2 = [ldx, h]  # 좌하
    p3 = [ldx + l1 - ((l1 - l2)/2), h-hl]  # 우상
    p4 = [ldx + l1, h]  # 우하
    return np.float32([p1, p2, p3, p4])


def get_params(img: np.array):
    h, w = get_hw(img)

    img_p1 = [0, 0]  # 좌상
    img_p2 = [0, h]  # 좌하
    img_p3 = [w, 0]  # 우상
    img_p4 = [w, h]  # 우하

    return np.float32([img_p1, img_p2, img_p3, img_p4])


def bird_eye_view(img: np.array, ldx: int, hl: int, l1: int, l2: int):
    par = get_params(img)
    points = get_points(img, ldx, hl, l1, l2)
    mat = cv2.getPerspectiveTransform(points, par)
    h, w = get_hw(img)
    transform = cv2.warpPerspective(img, mat, (w, h))
    return transform
# mat = 변환행렬(3*3 행렬)


# 이미지 출력
if __name__ == '__main__':
    img = cv2.imread('imgs/test1.jpg')
    plt.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
    plt.show()
    img_transformed = bird_eye_view(img, 400, 400, 950, 400)
    plt.imshow(cv2.cvtColor(img_transformed, cv2.COLOR_BGR2RGB))
    plt.show()
