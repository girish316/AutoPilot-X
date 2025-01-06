from flask import Flask, Response, render_template, request, jsonify, send_file
import cv2
import numpy as np
from io import BytesIO

app = Flask(__name__)

# Global variables
curveList = []
avgVal = 10
lane_center = 120  # Center of the lane in the image (can be adjusted as per the lane position)

def thresholding(img):
    imgHsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lowerWhite = np.array([80, 0, 0])
    upperWhite = np.array([255, 160, 255])
    maskWhite = cv2.inRange(imgHsv, lowerWhite, upperWhite)
    return maskWhite

def warpImg(img, points, w, h, inv=False):
    pts1 = np.float32(points)
    pts2 = np.float32([[0, 0], [w, 0], [0, h], [w, h]])
    if inv:
        matrix = cv2.getPerspectiveTransform(pts2, pts1)
    else:
        matrix = cv2.getPerspectiveTransform(pts1, pts2)
    imgWarp = cv2.warpPerspective(img, matrix, (w, h))
    return imgWarp

def nothing(a):
    pass


def valTrackbars(wT=480, hT=240):
    widthTop = 0
    heightTop = 80
    widthBottom = 20
    heightBottom = 255
    points = np.float32([(widthTop, heightTop), (wT - widthTop, heightTop),
                         (widthBottom, heightBottom), (wT - widthBottom, heightBottom)])
    return points

def drawPoints(img, points):
    for x in range(4):
        cv2.circle(img, (int(points[x][0]), int(points[x][1])), 15, (0, 0, 255), cv2.FILLED)
    return img

def getHistogram(img, minPer=0.1, display=True, region=1):
    if region == 1:
        histValues = np.sum(img, axis=0)
    else:
        histValues = np.sum(img[img.shape[0] // region:, :], axis=0)

    maxValue = np.max(histValues)
    minValue = minPer * maxValue

    indexArray = np.where(histValues >= minValue)
    basePoint = int(np.average(indexArray))

    imgHist = np.zeros((img.shape[0], img.shape[1], 3), np.uint8)
    for x, intensity in enumerate(histValues):
        line_end_y = max(0, img.shape[0] - (intensity // 255 // region))
        cv2.line(imgHist, (x, img.shape[0]), (x, line_end_y), (255, 0, 255), 1)
    cv2.circle(imgHist, (basePoint, img.shape[0]), 20, (0, 255, 255), cv2.FILLED)
    return basePoint, imgHist

def stackImages(scale, imgArray):
    rows = len(imgArray)
    cols = len(imgArray[0])
    rowsAvailable = isinstance(imgArray[0], list)
    width = imgArray[0][0].shape[1]
    height = imgArray[0][0].shape[0]
    if rowsAvailable:
        for x in range(0, rows):
            for y in range(0, cols):
                if imgArray[x][y].shape[:2] == imgArray[0][0].shape[:2]:
                    imgArray[x][y] = cv2.resize(imgArray[x][y], (0, 0), None, scale, scale)
                else:
                    imgArray[x][y] = cv2.resize(imgArray[x][y], (imgArray[0][0].shape[1], imgArray[0][0].shape[0]), None, scale, scale)
                if len(imgArray[x][y].shape) == 2: imgArray[x][y] = cv2.cvtColor(imgArray[x][y], cv2.COLOR_GRAY2BGR)
        imageBlank = np.zeros((height, width, 3), np.uint8)
        hor = [imageBlank] * rows
        hor_con = [imageBlank] * rows
        for x in range(0, rows):
            hor[x] = np.hstack(imgArray[x])
        ver = np.vstack(hor)
    else:
        for x in range(0, rows):
            if imgArray[x].shape[:2] == imgArray[0].shape[:2]:
                imgArray[x] = cv2.resize(imgArray[x], (0, 0), None, scale, scale)
            else:
                imgArray[x] = cv2.resize(imgArray[x], (imgArray[0].shape[1], imgArray[0].shape[0]), None, scale, scale)
            if len(imgArray[x].shape) == 2: imgArray[x] = cv2.cvtColor(imgArray[x], cv2.COLOR_GRAY2BGR)
        hor = np.hstack(imgArray)
        ver = hor
    return ver

def calculateSteeringAdjustment(curve):
    global lane_center
    steeringAdjustment = lane_center + curve * 50
    return steeringAdjustment

def getLaneCurve(img):
    global lane_center

    imgCopy = img.copy()
    imgResult = img.copy()

    imgThres = thresholding(img)

    hT, wT, c = img.shape
    points = valTrackbars()

    imgWarp = warpImg(imgThres, points, wT, hT)
    imgWarpPoints = drawPoints(imgCopy, points)

    middlePoint, imgHist = getHistogram(imgWarp, minPer=0.05, display=True, region=4)
    curveAveragePoint, imgHist = getHistogram(imgWarp, display=True, minPer=0.9)
    curveRaw = curveAveragePoint - middlePoint

    curveList.append(curveRaw)
    if len(curveList) > avgVal:
        curveList.pop(0)
    curve = int(sum(curveList) / len(curveList))

    imgInvWarp = warpImg(imgWarp, points, wT, hT, inv=True)
    imgInvWarp = cv2.cvtColor(imgInvWarp, cv2.COLOR_GRAY2BGR)
    imgInvWarp[0:hT // 3, 0:wT] = 0, 0, 0

    imgLaneColor = np.zeros_like(img)
    imgLaneColor[:] = 0, 255, 0
    imgLaneColor = cv2.bitwise_and(imgInvWarp, imgLaneColor)

    row_y = 200
    green_color = [0, 255, 0]
    row_pixels = imgLaneColor[row_y, :, :]
    colored_pixels = np.where(np.all(row_pixels == green_color, axis=-1))[0]

    if len(colored_pixels) > 0:
        min_x = colored_pixels[0]
        max_x = colored_pixels[-1]
        lane_center = (min_x + max_x) / 2.0

    imgResult = cv2.addWeighted(imgResult, 1, imgLaneColor, 1, 0)

    midY = 450
    cv2.line(imgResult, (wT // 2, midY), (wT // 2 + (curve * 3), midY), (255, 0, 255), 5)
    cv2.line(imgResult, ((wT // 2 + (curve * 3)), midY - 25), (wT // 2 + (curve * 3), midY + 25), (0, 255, 0), 5)
    for x in range(-30, 30):
        w = wT // 20
        cv2.line(imgResult, (w * x + int(curve // 50), midY - 10),
                    (w * x + int(curve // 50), midY + 10), (0, 0, 255), 2)

    curve = curve / 100
    if curve > 1:
        curve = 1
    if curve < -1:
        curve = -1

    steeringAdjustment = calculateSteeringAdjustment(curve)
    offset = steeringAdjustment - lane_center

    text = str(abs(offset)) + " Left" if offset < 0 else str(offset) + " Right"
    (text_width, text_height), baseline = cv2.getTextSize(text, cv2.FONT_ITALIC, 1.5, 3)
    x_pos = wT // 2 - text_width // 2
    cv2.putText(imgResult, text, (x_pos, 35), cv2.FONT_ITALIC, 1.5, (0, 255, 0), 3)

    return imgWarpPoints

def video_stream():
    """Read video frames, draw circle and line, and stream them to the browser."""
    cap = cv2.VideoCapture("Dashboard/vid1.mp4")

    if not cap.isOpened():
        print("❌ Failed to load the video file.")
        return

    frameCounter = 0
    while True:
        frameCounter += 1
        if cap.get(cv2.CAP_PROP_FRAME_COUNT) == frameCounter:
            cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            frameCounter = 0

        img = cv2.imread("Dashboard/image.jpg")
        img = cv2.resize(img, (480, 240))
        
        curve = getLaneCurve(img)

        ret, buffer = cv2.imencode('.jpg', curve)

        if not ret:
            print("❌ Failed to encode frame")
            continue

        frame = buffer.tobytes()

        return (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')


@app.route('/')
def index():
    """Render the main page."""
    return render_template('index.html')

@app.route('/video_feed')
def video_feed():
    """Video streaming route."""
    return Response(video_stream(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5001, debug=True)