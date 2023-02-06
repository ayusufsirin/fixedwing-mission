import cv2
import time
import numpy as np
import imutils
from imutils.video import VideoStream
from imutils.video import FPS

vs = VideoStream(src=1).start()
time.sleep(2.0)
fps = FPS().start()

while True:

    frame = vs.read()
    # Split the H channel in HSV, and get the red range
    hsv = cv2.cvtColor(src=frame, code=cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)
    h[h < 170] = 0
    h[h > 180] = 0

    # normalize, do the open-morp-op
    normed = cv2.normalize(h, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8UC1)
    kernel = cv2.getStructuringElement(shape=cv2.MORPH_ELLIPSE, ksize=(3, 3))
    opened = cv2.morphologyEx(normed, cv2.MORPH_OPEN, kernel)
    res = np.hstack((h, normed, opened))
    cv2.imwrite("tmp1.png", res)

    _, contours, _ = cv2.findContours(opened, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    # print(len(contours))

    dst = frame.copy()
    areas = np.array([0])
    cnts = []
    for cnt in contours:
        # area of cnt
        epsilon = 0.01 * cv2.arcLength(cnt, True)
        cnt = cv2.approxPolyDP(cnt, epsilon, True)
        area = cv2.contourArea(cnt)
        np.append(areas, area)
        cnts.append(cnt)

        # print(cnt)

        if area > 100:
            # bounding circle
            (x, y), radius = cv2.minEnclosingCircle(cnt)
            center = (int(x), int(y))
            radius = int(radius)
            cv2.circle(dst, center, radius, (0, 255, 0), 2)
            cv2.drawContours(dst, [cnt], 0, (0, 255, 0), 3)

    # (x, y), radius = cv2.minEnclosingCircle(cnt)

    cv2.imshow("images", np.hstack([frame, dst]))
    cv2.imshow('opened', opened)

    key = cv2.waitKey(1) & 0xFF
    # if the `q` key was pressed, break from the loop
    if key == ord("q"):
        break
    # update the FPS counter
    fps.update()

    # stop the timer and display FPS information
    fps.stop()
    # print("[INFO] elapsed time: {:.2f}".format(fps.elapsed()))
    # print("[INFO] approx. FPS: {:.2f}".format(fps.fps()))

cv2.destroyAllWindows()
