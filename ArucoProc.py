import cv2
import cv2.aruco as aruco
import numpy as np 


# establish camera, arUco settings
# cap = cv2.VideoCapture(0)
ground = cv2.createBackgroundSubtractorMOG2(history=200, varThreshold=50)

arucoDict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
parameters = aruco.DetectorParameters()

# if the camera doesn’t open, report the error and exit program 
# if not cap.isOpened():
# 	print("Camera can't open")
# 	exit()

# Set initial working conditions for the camera (reference for calculations)
cHeightMax = 0.2921
markerLength = 0.05
minArea = 800

# if the camera is functional, set up the matrix of positions based on the camera’s range of vision
while True:
	# ret, frame = cap.read()
	# if not ret:
	# 	break
	
	frame = cv2.imread("camerasnap.png")
	if frame is None:
		print("Failed to load image")
		break
	h, w = frame.shape[:2]
	f = w
	cameraMatrix = np.array([[f, 0, w/2], [0, f, h/2], [0, 0, 1]], dtype=np.float32)
	distCoeffs = np.zeros((5, 1))

	# ArUco markers - establish the tag and identify it within the camera’s view
	gray = cv2. cvtColor(frame, cv2.COLOR_BGR2GRAY)
	corners, ids, rejected = aruco.detectMarkers(gray, arucoDict, parameters=parameters)
	print(corners)
	print(ids)
	# if the ArUco tag is detected properly, determine the position of the object based on the tag’s communicated information + initial camera view matrix, convert from pixels to meters
	if ids is not None:
		aruco.drawDetectedMarkers(frame, corners, ids)
		rvecs, tvecs, _objPoints = aruco.estimatePoseSingleMarkers(corners, markerLength, cameraMatrix, distCoeffs)
		cv2.drawFrameAxes(frame, cameraMatrix, distCoeffs, rvecs[0], tvecs[0], 0.05)

		xC, yC, zC = tvecs[0][0]
		yNew = max(cHeightMax - zC, 0)
		xNew = (corners[0][0][:, 0].mean() / w)
		xNew *= 1.0
		zNew = (corners[0][0][:, 1].mean() / h)

		# output the meter values for the object’s position
		cv2.putText(frame, f"Obj (x = {xNew:.2f}, y = {yNew:.2f}, z = {zNew:.2f})", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
		print(f"Object position (normalized frame): X = {xNew:.3f} m, Y = {yNew:.3f} m, Z = {zNew:.3f} m")

	# Display the camera’s view on the computer. if “ESC” is pressed, close the camera view and exit the program
	cv2.imshow("ArUco + Object Detection", frame)
	key = cv2.waitKey(10)
	if key == 27:
		break


# cap.release()
cv2.destroyAllWindows()

