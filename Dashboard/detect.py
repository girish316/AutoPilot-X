import cv2
import numpy as np
import utlis

# Global variables for lane center
lane_center = 120  # Lane center (initial position)

def getLaneCurve(img, display):
    global lane_center

    imgCopy = img.copy()
    imgResult = img.copy()

    #### STEP 1: Thresholding the image
    imgThres = utlis.thresholding(img)

    #### STEP 2: Warp the image
    hT, wT, c = img.shape
    points = utlis.valTrackbars()
    imgWarp = utlis.warpImg(imgThres, points, wT, hT)
    imgWarpPoints = utlis.drawPoints(imgCopy, points)

    #### STEP 3: Get histogram data to determine the curve
    middlePoint, imgHist = utlis.getHistogram(imgWarp, display=True, minPer=0.5, region=4)
    curveAveragePoint, imgHist = utlis.getHistogram(imgWarp, display=True, minPer=0.9)
    curveRaw = curveAveragePoint - middlePoint

    #### STEP 4: Track and smooth the curve data
    curve = curveRaw  # Calculate the curve

    #### STEP 5: Display and debug information
    if display != 0:
        imgInvWarp = utlis.warpImg(imgWarp, points, wT, hT, inv=True)
        imgInvWarp = cv2.cvtColor(imgInvWarp, cv2.COLOR_GRAY2BGR)

        imgLaneColor = np.zeros_like(img)
        imgLaneColor[:] = 0, 255, 0
        imgLaneColor = cv2.bitwise_and(imgInvWarp, imgLaneColor)

        row_y = 200  # Row of interest
        green_color = [0, 255, 0]  # Green color in BGR
        row_pixels = imgLaneColor[row_y, :, :]  # Extract the row 200 pixels
        colored_pixels = np.where(np.all(row_pixels == green_color, axis=-1))[0]  # Find green pixels

        if len(colored_pixels) > 0:
            min_x = colored_pixels[0]
            max_x = colored_pixels[-1]
            lane_center = (min_x + max_x) / 2.0  # Calculate lane center

        imgResult = cv2.addWeighted(imgResult, 1, imgLaneColor, 1, 0)

        # Display curve and lane
        midY = 450
        cv2.line(imgResult, (wT // 2, midY), (wT // 2 + (curve * 3), midY), (255, 0, 255), 5)
        cv2.line(imgResult, ((wT // 2 + (curve * 3)), midY - 25), (wT // 2 + (curve * 3), midY + 25), (0, 255, 0), 5)

    # Normalize the curve
    curve = curve / 100
    if curve > 1:
        curve = 1
    if curve < -1:
        curve = -1

    return imgResult