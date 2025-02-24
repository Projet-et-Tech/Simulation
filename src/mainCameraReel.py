import cv2
import os

from utils.imageTransform import calibrationAndTransform
from utils.perspectiveCorrection import convert_2D_to_3D
from utils.arucoDetection import detectMarkerByID
from config import ROBOT_ID, CAM1_POS, PAMI_HEIGHT


def main():
    RTSP_URL = 'tcp://192.168.159.241:5001'
    os.environ['OPENCV_FFMPEG_CAPTURE_OPTIONS'] = 'rtsp_transport;udp'
    # start stream first!
    cap = cv2.VideoCapture(RTSP_URL, cv2.CAP_FFMPEG)
    #cap = cv.VideoCapture(0, cv.CAP_V4L2)

    if not cap.isOpened():
        print("Cannot open camera")
        exit()
    else:
        print("Capture opened")
        print(cap.getBackendName())
        print(cap.get(cv2.CAP_PROP_BACKEND))
        print(cap.get(cv2.CAP_PROP_VIDEO_STREAM))

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        transformed_frame1 = calibrationAndTransform(frame, 1)
        cv2.namedWindow("Table", cv2.WINDOW_NORMAL)
        cv2.imshow("Table", transformed_frame1)

        # Vérifier si frame ou transformed_frame
        detected_center = detectMarkerByID(transformed_frame1, ROBOT_ID)
        if detected_center is not None:
            print(f"Marker {ROBOT_ID} found at: {detected_center}")
        else:
            print(f"Marker {ROBOT_ID} not found in the frame.")

        true_center = convert_2D_to_3D(detected_center[0], detected_center[1], transformed_frame1, CAM1_POS, PAMI_HEIGHT)
        print(f"true center : {true_center[0]}, {true_center[1]}, {true_center[2]}")

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
