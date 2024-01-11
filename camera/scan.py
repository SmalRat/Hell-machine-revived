import cv2

import numpy as np

def filter_blue(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV) 
    lower_blue = np.array([60, 35, 140]) 
    upper_blue = np.array([180, 255, 255]) 
    mask = cv2.inRange(hsv, lower_blue, upper_blue) 
    return  cv2.bitwise_and(img, img, mask = mask) 

def create_trapeze_POI(img):
    height, width = img.shape
    corners = np.array([[(0, height), (width, height), (width, 0), (0, 0)]])
    trapeze = np.array([[(0, height), (width, height), (480, 240), (220, 240)]])
    black_image = np.zeros_like(img)
    mask = cv2.fillPoly(black_image, trapeze, 255)
    masked = cv2.bitwise_and(img, mask)
    matrix = cv2.getPerspectiveTransform(trapeze.astype(np.float32), corners.astype(np.float32))
    return cv2.warpPerspective(masked, matrix, (width, height))


def create_frames(vid):
    while(True): 
        ret, or_frame = vid.read() 
        frame = filter_blue(or_frame)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray_or = cv2.cvtColor(or_frame, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 50, 150, apertureSize=3)
        edges = create_trapeze_POI(edges)
        gray_or = create_trapeze_POI(gray_or)
        cat = np.concatenate((gray_or, edges), axis = 0)       
        ret, cat = cv2.imencode(".jpg", cat)
        byte = cat.tobytes()

        yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + byte + b'\r\n')


