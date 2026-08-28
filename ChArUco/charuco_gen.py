import cv2
from cv2 import aruco

# 版面參數（公尺）：9x6 方格，方格邊長 30mm，方格內 ArUco 邊長 23mm。
SQUARES_X, SQUARES_Y = 9, 6
SQUARE_LEN, MARKER_LEN = 0.03, 0.023
IMG_SIZE = (2000, 1400)  # (寬, 高)，比例需與 9:6 相符

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)

# OpenCV 4.7+ 用新建構式 + generateImage()；4.5/4.6 用 *_create + draw()。
if hasattr(aruco, "CharucoBoard"):
    board = aruco.CharucoBoard((SQUARES_X, SQUARES_Y), SQUARE_LEN, MARKER_LEN, aruco_dict)
    img = board.generateImage(IMG_SIZE)
else:
    board = aruco.CharucoBoard_create(SQUARES_X, SQUARES_Y, SQUARE_LEN, MARKER_LEN, aruco_dict)
    img = board.draw(IMG_SIZE)

cv2.imwrite("charuco_board.png", img)
print("已生成 charuco_board.png，请打印（整张A4，别缩放）")
print(f"棋盤：{SQUARES_X}x{SQUARES_Y} 方格，方格邊長={SQUARE_LEN*1000:.0f}mm，marker邊長={MARKER_LEN*1000:.0f}mm")
print("列印後務必用尺量『方格實際邊長』，校正時要用量到的真實值，不是這裡寫的 30mm。")
