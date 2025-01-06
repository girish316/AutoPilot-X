import cv2
import numpy as np
 
def thresholding(img):
    imgHsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
    lowerWhite = np.array([80,0,0])
    upperWhite = np.array([255,160,255])
    maskWhite = cv2.inRange(imgHsv,lowerWhite,upperWhite)
    return maskWhite
 
def warpImg(img,points,w,h,inv = False):
    pts1 = np.float32(points)
    pts2 = np.float32([[0,0],[w,0],[0,h],[w,h]])
    if inv:
        matrix = cv2.getPerspectiveTransform(pts2, pts1)
    else:
        matrix = cv2.getPerspectiveTransform(pts1,pts2)
    imgWarp = cv2.warpPerspective(img,matrix,(w,h))
    return imgWarp
 
def nothing(a):
    pass
 
def initializeTrackbars(intialTracbarVals,wT=480, hT=240):
    cv2.namedWindow("Trackbars")
    cv2.resizeWindow("Trackbars", 360, 240)
    cv2.createTrackbar("Width Top", "Trackbars", intialTracbarVals[0],wT//2, nothing)
    cv2.createTrackbar("Height Top", "Trackbars", intialTracbarVals[1], hT, nothing)
    cv2.createTrackbar("Width Bottom", "Trackbars", intialTracbarVals[2],wT//2, nothing)
    cv2.createTrackbar("Height Bottom", "Trackbars", intialTracbarVals[3], hT, nothing)
 
def valTrackbars(wT=480, hT=240):
    widthTop = cv2.getTrackbarPos("Width Top", "Trackbars")
    heightTop = cv2.getTrackbarPos("Height Top", "Trackbars")
    widthBottom = cv2.getTrackbarPos("Width Bottom", "Trackbars")
    heightBottom = cv2.getTrackbarPos("Height Bottom", "Trackbars")
    points = np.float32([(widthTop, heightTop), (wT-widthTop, heightTop),
                      (widthBottom , heightBottom ), (wT-widthBottom, heightBottom)])
    return points
 
def drawPoints(img,points):
    for x in range(4):
        cv2.circle(img,(int(points[x][0]),int(points[x][1])),15,(0,0,255),cv2.FILLED)
    return img
 
def getHistogram(img, minPer=0.1, display=False, region=1):
    if region == 1:
        histValues = np.sum(img, axis=0)
    else:
        histValues = np.sum(img[img.shape[0] // region:, :], axis=0)

    # Get max and min intensity
    maxValue = np.max(histValues)
    minValue = minPer * maxValue

    # Get the x indices where the intensity is above the threshold
    indexArray = np.where(histValues >= minValue)
    basePoint = int(np.average(indexArray))  # This is the middle point of the lane

    # Prepare histogram image if display flag is True
    if display:
        imgHist = np.zeros((img.shape[0], img.shape[1], 3), np.uint8)
        for x, intensity in enumerate(histValues):
            # Ensure that the second coordinate (y) stays within the bounds of the image
            line_end_y = max(0, img.shape[0] - (intensity // 255 // region))
            cv2.line(imgHist, (x, img.shape[0]), (x, line_end_y), (255, 0, 255), 1)
        
        # Draw a circle at the base point for reference
        cv2.circle(imgHist, (basePoint, img.shape[0]), 20, (0, 255, 255), cv2.FILLED)
        
        return basePoint, imgHist

    return basePoint
 
def stackImages(scale,imgArray):
    rows = len(imgArray)
    cols = len(imgArray[0])
    rowsAvailable = isinstance(imgArray[0], list)
    width = imgArray[0][0].shape[1]
    height = imgArray[0][0].shape[0]
    if rowsAvailable:
        for x in range ( 0, rows):
            for y in range(0, cols):
                if imgArray[x][y].shape[:2] == imgArray[0][0].shape [:2]:
                    imgArray[x][y] = cv2.resize(imgArray[x][y], (0, 0), None, scale, scale)
                else:
                    imgArray[x][y] = cv2.resize(imgArray[x][y], (imgArray[0][0].shape[1], imgArray[0][0].shape[0]), None, scale, scale)
                if len(imgArray[x][y].shape) == 2: imgArray[x][y]= cv2.cvtColor( imgArray[x][y], cv2.COLOR_GRAY2BGR)
        imageBlank = np.zeros((height, width, 3), np.uint8)
        hor = [imageBlank]*rows
        hor_con = [imageBlank]*rows
        for x in range(0, rows):
            hor[x] = np.hstack(imgArray[x])
        ver = np.vstack(hor)
    else:
        for x in range(0, rows):
            if imgArray[x].shape[:2] == imgArray[0].shape[:2]:
                imgArray[x] = cv2.resize(imgArray[x], (0, 0), None, scale, scale)
            else:
                imgArray[x] = cv2.resize(imgArray[x], (imgArray[0].shape[1], imgArray[0].shape[0]), None,scale, scale)
            if len(imgArray[x].shape) == 2: imgArray[x] = cv2.cvtColor(imgArray[x], cv2.COLOR_GRAY2BGR)
        hor= np.hstack(imgArray)
        ver = hor
    return ver

def getLaneCurve(img, display):
    global lane_center

    imgCopy = img.copy()
    imgResult = img.copy()

    #### STEP 1: Thresholding the image
    imgThres = thresholding(img)

    #### STEP 2: Warp the image
    hT, wT, c = img.shape
    points = valTrackbars()
    imgWarp = warpImg(imgThres, points, wT, hT)
    imgWarpPoints = drawPoints(imgCopy, points)

    #### STEP 3: Get histogram data to determine the curve
    middlePoint, imgHist = getHistogram(imgWarp, display=True, minPer=0.5, region=4)
    curveAveragePoint, imgHist = getHistogram(imgWarp, display=True, minPer=0.9)
    curveRaw = curveAveragePoint - middlePoint

    #### STEP 4: Track and smooth the curve data
    curveList.append(curveRaw)
    if len(curveList) > avgVal:
        curveList.pop(0)
    curve = int(sum(curveList) / len(curveList))

    #### STEP 5: Display and debug information
    if display != 0:
        imgInvWarp = warpImg(imgWarp, points, wT, hT, inv=True)
        imgInvWarp = cv2.cvtColor(imgInvWarp, cv2.COLOR_GRAY2BGR)
        imgInvWarp[0:hT // 3, 0:wT] = 0, 0, 0

        imgLaneColor = np.zeros_like(img)
        imgLaneColor[:] = 0, 255, 0
        imgLaneColor = cv2.bitwise_and(imgInvWarp, imgLaneColor)

        row_y = 200  # Row of interest
        green_color = [0, 255, 0]  # Green color in BGR
        row_pixels = imgLaneColor[row_y, :, :]  # Extract the row 200 pixels (all columns, row 200)
        colored_pixels = np.where(np.all(row_pixels == green_color, axis=-1))[0]  # Find where the row pixels are equal to green color

        if len(colored_pixels) > 0:
            min_x = colored_pixels[0]  # First column with green color
            max_x = colored_pixels[-1]  # Last column with green color
            lane_center = (min_x + max_x) / 2.0  # Calculate the lane center

        else:
            print("No green pixels at row 200")

        imgResult = cv2.addWeighted(imgResult, 1, imgLaneColor, 1, 0)

        midY = 450
        cv2.line(imgResult, (wT // 2, midY), (wT // 2 + (curve * 3), midY), (255, 0, 255), 5)
        cv2.line(imgResult, ((wT // 2 + (curve * 3)), midY - 25), (wT // 2 + (curve * 3), midY + 25), (0, 255, 0), 5)
        for x in range(-30, 30):
            w = wT // 20
            cv2.line(imgResult, (w * x + int(curve // 50), midY - 10),
                     (w * x + int(curve // 50), midY + 10), (0, 0, 255), 2)

    #### NORMALIZATION: Ensure curve stays within expected range
    curve = curve / 100
    if curve > 1:
        curve = 1
    if curve < -1:
        curve = -1

    # Display the final result
    if display == 2:
        imgStacked = stackImages(0.7, ([img, imgWarpPoints, imgWarp],
                                             [imgHist, imgLaneColor, imgResult]))
        return imgStacked
    elif display == 1:
        steeringAdjustment = calculateSteeringAdjustment(curve)  # Renamed function
        offset = steeringAdjustment - lane_center

        # Display the offset text
        text = str(abs(offset)) + " Left" if offset < 0 else str(offset) + " Right"
        (text_width, text_height), baseline = cv2.getTextSize(text, cv2.FONT_ITALIC, 1.5, 3)
        x_pos = wT // 2 - text_width // 2
        cv2.putText(imgResult, text, (x_pos, 85), cv2.FONT_ITALIC, 1.5, (255, 0, 255), 3)

        # Draw the predicted vehicle position and lane center
        midY = 200  # Y-coordinate where the lane is potentially useful for calculations
        cv2.circle(imgResult, (int(steeringAdjustment), midY), 10, (0, 0, 255), -1)  # Red circle at predicted position
        cv2.circle(imgResult, (int(lane_center), midY), 10, (255, 0, 0), -1)  # Blue circle at Lane Center  

        # Show the result
        cv2.imshow('Result', imgResult)

    return imgResult

# Renamed function to better reflect its role
def calculateSteeringAdjustment(curve):
    global lane_center
    # This calculation represents the optimal steering or predicted movement after compensating for the curve
    steeringAdjustment = lane_center + curve * 50  # Adjust the steering prediction based on curve
    return steeringAdjustment




curveList = []
avgVal = 10
lane_center = 120  # Center of the lane in the image (can be adjusted as per the lane position)
if __name__ == '__main__':
    cap = cv2.VideoCapture("lane_detect_test/vid1.mp4")
    initialTrackBarVals = [0, 80, 20, 255]
    initializeTrackbars(initialTrackBarVals)
    frameCounter = 0

    while True:
        frameCounter += 1
        if cap.get(cv2.CAP_PROP_FRAME_COUNT) == frameCounter:
            cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            frameCounter = 0

        success, img = cap.read()
        img = cv2.resize(img, (480, 240))
        
        curve = getLaneCurve(img, 1)  # Get the curve of the lane

        cv2.waitKey(1)