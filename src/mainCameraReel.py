import cv2
import os
import numpy as np

from utils.imageTransform import calibrationAndTransform
from utils.perspectiveCorrection import convert_2D_to_3D
from utils.arucoDetection import highlightDetected, drawCross, getDirectionAndDistance
from config import ROBOT_ID, CAM1_POS, PAMI_HEIGHT, PAMI_ID, TABLE_WIDTH

def main():
    # RTSP_URL = 'tcp://172.20.10.2:5001'
    # os.environ['OPENCV_FFMPEG_CAPTURE_OPTIONS'] = 'rtsp_transport;udp'
    # start stream first!
    # cap = cv2.VideoCapture(RTSP_URL, cv2.CAP_FFMPEG)

    cap = cv2.VideoCapture(0)

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
        # frame = cv2.flip(frame, 1)
        if not ret:
            break

        frame, _, _ = highlightDetected(frame, {ROBOT_ID, PAMI_ID})
        _, centerROBOT, cornersROBOT = highlightDetected(frame, {ROBOT_ID})
        _, centerPAMI, _ = highlightDetected(frame, {PAMI_ID})


        if centerROBOT:
            # Extract the center coordinates
            centerROBOT = tuple(list(centerROBOT.values())[0])  # Assuming only one center is detected

            # Convert corners to a NumPy array
            cornersROBOT = np.array(cornersROBOT[0][0], dtype=np.int32)

            # print("-----------")
            # print(cornersROBOT, cornersROBOT[0], cornersROBOT[0][0])

            drawCross(frame, centerROBOT, cornersROBOT)

            if centerPAMI:
                centerPAMI = tuple(list(centerPAMI.values())[0])  # Assuming only one center is detected

                angle, distance, direction = getDirectionAndDistance(centerROBOT, cornersROBOT, centerPAMI, frame.shape[0], frame.shape[1], meters_per_pixel=TABLE_WIDTH / frame.shape[1]) 
                print(f"Angle: {angle:.2f}°, Distance: {distance:.2f}m, Direction: {direction}")

                # Draw blue line between centers
                cv2.line(frame, centerROBOT, centerPAMI, (255, 0, 0), 2)

        cv2.imshow("Highlighted", frame)
        


        # transformed_frame1 = calibrationAndTransform(frame, 1)
        # if np.array_equal(transformed_frame1, frame) or transformed_frame1 is None:
        #     continue

        # # Highlight detected markers and get their centers
        # transformed_frame1, centers1, _ = highlightDetected(transformed_frame1, {ROBOT_ID, PAMI_ID})

        # # Display the transformed frame
        # cv2.namedWindow("Table", cv2.WINDOW_NORMAL)
        # cv2.imshow("Table", transformed_frame1)
        # cv2.waitKey(1)

        # # Check if either ROBOT_ID or PAMI_ID is detected
        # for marker_id in [ROBOT_ID, PAMI_ID]:
        #     if marker_id in centers1:
        #         detected_center = centers1[marker_id]

        #         # Convert detected 2D position to 3D coordinates
        #         true_center = convert_2D_to_3D(detected_center[0], detected_center[1], transformed_frame1, CAM1_POS, PAMI_HEIGHT)
        #         print(f"True center of marker {marker_id}: {true_center[0]}, {true_center[1]}")

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
