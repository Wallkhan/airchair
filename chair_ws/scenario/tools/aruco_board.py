import sys
import os
import cv2
from cv2 import aruco
import numpy as np

# Define aruco dictionary and parameters 
aruco_dict = aruco.getPredefinedDictionary( aruco.DICT_6X6_1000 )
aruco_params = aruco.DetectorParameters()

# Name for the generated board image
board_name = 'board.png'

# Create Board, given the length of the markers and the separation between markers and speific ids
markerLength = .13   # Here, our measurement unit is meters.
markerSeparation = 0.01   # Here, our measurement unit is meters.
ids = np.array([0, 1, 2, 3, 4, 5, 6, 7, 8])
board = aruco.GridBoard([3,3], markerLength, markerSeparation, aruco_dict, ids)
img = board.generateImage((1024, 1366))
img_copy = img.copy()

# Detect markers in the board image
aruco_detector = aruco.ArucoDetector(aruco_dict, aruco_params)
corners_1, ids_1, rejectedImgPoints_1 = aruco_detector.detectMarkers(img_copy)
frame_1 = aruco.drawDetectedMarkers(img_copy, corners_1, ids_1)

# Print corners and ids of the detected markers
print(f'corners_1: {len(corners_1)}, ids_1: {ids_1}, no of ids: {len(ids_1)}')
# See the detected markers
# cv2.imshow('GridBoard', img)
# cv2.imshow("Detected Markers", frame_1)
# cv2.waitKey(0)
# cv2.destroyAllWindows()

# Save Board Image
cv2.imwrite(board_name, img)
