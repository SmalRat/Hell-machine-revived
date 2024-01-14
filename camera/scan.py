import cv2
import can
import numpy as np

def draw_lines(lines, img):
    if type(lines) != type(None):
        for r, theta in lines:
            theta /= (180/np.pi)
            a = np.cos(theta)
            b = np.sin(theta)
            x0 = r * a
            y0 = r * b
            pt1 = (int(x0 + 1000 * (-b)), int(y0 + 1000 * a))
            pt2 = (int(x0 - 1000 * (-b)), int(y0 - 1000 * a))
            cv2.line(img, pt1, pt2, (0,0, 255), 2)

def filter_blue(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV) 
    lower_blue = np.array([60, 35, 140]) 
    upper_blue = np.array([180, 255, 255]) 
    mask = cv2.inRange(hsv, lower_blue, upper_blue) 
    return  cv2.bitwise_and(img, img, mask = mask) 

def create_trapeze_POI(img):
    height, width = img.shape
    corners = np.array([[(0, height), (width, height), (width, 0), (0, 0)]])
    # trapeze = np.array([[(0, height), (width, height), (420, 250), (250, 250)]])
    trapeze = np.array([[(0, height), (width, height), (490, 320), (170, 320)]])
    black_image = np.zeros_like(img)
    mask = cv2.fillPoly(black_image, trapeze, 255)
    masked = cv2.bitwise_and(img, mask)
    matrix = cv2.getPerspectiveTransform(trapeze.astype(np.float32), corners.astype(np.float32))
    return cv2.warpPerspective(masked, matrix, (width, height))


def find_line_angle(lines):
    if type(lines) != type(None):
        lines = np.array([line[0] for line in lines])
        lines[:,1] *= (180/np.pi)
        lines[lines[:, 1].argsort()]
        return lines[:2]
    
def send_angle(lines):
    with can.Bus(interface='socketcan', channel='can0', bitrate=1000000) as bus:
        msg = None
        if type(lines) != type(None):
            direction = 0 if lines[0][0] > 0 else 255    
            msg = can.Message(arbitration_id=0x102, data = [direction, int(np.mean(lines, axis=0)[1])])
        else:
            msg = can.Message(arbitration_id=0x102, data = [69, 69])
        try:
            bus.send(msg)
        except can.CanError:
            print("CAN't")

    

def create_frames(vid):
    while(True): 
        ret, or_frame = vid.read()
        or_frame = cv2.flip(or_frame, 0)
        frame = filter_blue(or_frame)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray_or = cv2.cvtColor(or_frame, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 25, 150, apertureSize=3)
        gray_or = create_trapeze_POI(gray_or)
        edges = create_trapeze_POI(edges)
        lines = cv2.HoughLines(edges,1,np.pi/180,250)
        lines = find_line_angle(lines)
        draw_lines(lines, gray_or)
        send_angle(lines)
        cat = np.concatenate((gray_or, edges), axis = 0)       
        ret, cat = cv2.imencode(".jpg", cat)
        byte = cat.tobytes()

        yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + byte + b'\r\n')


