import cv2
import time
from cv2 import COLOR_RGB2GRAY
from cv2 import COLOR_RGB2BGR565
from gpiozero import Motor, Button, AngularServo
from gpiozero.pins.pigpio import PiGPIOFactory
import numpy as np
import math
import board
import adafruit_tcs34725

i2c = board.I2C() 
sensor = adafruit_tcs34725.TCS34725(i2c)
factory = PiGPIOFactory()
button = Button(20,False)            
motor_1 = Motor(forward = 13,backward = 6)
fahrtrichtung = 0
blue_color = (4,25,25)
servo = AngularServo(23,min_angle=-180,max_angle=180,min_pulse_width=5/10000,max_pulse_width=25/10000,frame_width=20/1000,pin_factory=factory)  
cap = cv2.VideoCapture(0)
ret = cap.set(3,320)
ret = cap.set(4,240) 

def forward(speed):
    """Funktion zum Vorwärtsfahren

    Argumente:
        speed (float): Werte von 0 bis 1; 1 ist das Maximum
    """    
    motor_1.forward(speed)      

def stop():
    """ Funktion zum Anhalten
    """
    motor_1.stop()

def creating_roi(frame):
    """Gibt die untere Bildhälfte zurück

    Argumente:
        frame (array): Bild

    Rückgabe:
        array: untere Bildhälfte
    """    
    height, width, _ = frame.shape
    roi = frame[int(1/2*height):height,0:width]
    return roi

def detected_edges(frame):
    """ Funktion zum Erkennen aller Kanten im Bild

    Argumente:
        frame (array): Bild

    Rückgabe:
        array: Bild mit allen erkannten Kanten
    """    
    hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
    lower_black = np.array([0, 0, 0])
    upper_black = np.array([180, 255, 75])
    mask = cv2.inRange(hsv, lower_black, upper_black)
    bild = cv2.medianBlur(mask, 5)
    kernel = np.ones((5,5),np.uint8)
    opening = cv2.morphologyEx(bild, cv2.MORPH_OPEN, kernel)
    e = cv2.Canny(image=opening, threshold1=0, threshold2=100)
    cv2.imshow('canny', e)
    return e

def detected_lines(detected_edges):
    """ Funktion zum Erkennen von Linien

    Argumente:
        detected_edges (array): Bild

    Rückgabe:
        array: Array mit Anfangs- und Endkoordinaten aller erkannten Linien 
    """    
    rho = 1  
    angle = np.pi / 180  
    d_lines = cv2.HoughLinesP(detected_edges, rho, angle, 15, None, 1, 20)
    return d_lines

def line_points(frame, line):
    """ Gibt die Anfangs- und Endkoordinaten der Begrenzungslinie zurück

    Argumente:
        frame (array): Bild
        line (array): Array mit Anfangs- und Endkoordinaten, Anstieg und Y-Achsenabschnitt der Linien

    Rückgabe:
        array: Anfangs- und Endkoordinaten der Begrenzungslinie
    """    
    height, width, _ = frame.shape
    if line[0][4]  < 0:
        x1 = 0
        y1 = int(line[0][5])
        x2 = line[1][2]
        y2 = line[1][3]  
        return ([x1, y1, x2, y2])
    elif line[0][4] > 0:
        x1 = line[0][0]
        y1 = line[0][1]  
        x2 = width
        y2 = int(line[1][4] * x2 + line[1][5])
        return ([x1, y1, x2, y2])

def max_right_line(linesP):
    """Gibt die rechte Begrenzungslinie mit der größten Y-Koordinate zurück

    Argumente:
        linesP (array): Array mit den erkannten Linien

    Rückgabe:
        array: Array mit den Anfangs- und Endkoordinaten, Anstieg, Y-Achsenabschnitt der Linie
    """
    lanesy2 =[]
    if linesP is not None:
        for lines in linesP:
            a = lines[3]
            lanesy2.append(a)
        y2index = 0 
        for i, y2 in enumerate(lanesy2):
            if y2 == max(lanesy2):
                y2index = i
        return linesP[y2index]
    else:
        return None

def min_right_line(linesP, max_right_line, height):
    """Gibt die rechte Begrenzungslinie mit der kleinsten Y-Koordinate zurück

    Argumente:
        linesP (array): Array mit den eraknnten Linien
        max_right_line (array): Array mit Anfangs- und Endkoordinaten, Anstieg und Y-Achsenabschnitt der rechten Begrenzungslinie mit der größten Y-Koordinate
        height (integer): Bildhöhe

    Rückgabe:
        array: Array mit den Anfangs- und Endkoordinaten, Anstieg, Y-Achsenabschnitt der Linie
    """
    lanesy1 =[]
    """if linesP is not None:
        for lines in linesP:"""

    if linesP is not None:
        for k,lines in enumerate(linesP):
            if lines[4] <= max_right_line[4] + 0.75*max_right_line[4] and lines[4] >= max_right_line[4] - 0.75*max_right_line[4]: 
                a = lines[1]
                lanesy1.append([k,a]) 
        y1 = height
        index = 0
        for i in lanesy1:
            if y1 >= i[1]:
                y1 = i[1]
                index = i[0]  
        return linesP[index]
    else:
        return None
    
def max_left_line(linesP):
    """Gibt die linke Begrenzungslinie mit der größten Y-Koordinate zurück

    Argumente:
        linesP (array): Array mit erkannten Linien

    Rückgabe:
        array: Array mit den Anfangs- und Endkoordinaten, Anstieg, Y-Achsenabschnitt der Linie
    """
    lanesy1 = []
    if linesP is not None:
        for lines in linesP:
            a = lines[1]
            lanesy1.append(a)
        y1index = 0 
        for i, y1 in enumerate(lanesy1):
            if y1 == max(lanesy1):
                y1index = i
        return linesP[y1index]
    else:
        return None

def min_left_line(linesP, max_left_line,height):
    """Gibt die linke Begrenzungslinie mit der kleinsten Y-Koordinate zurück

    Argumente:
        linesP (array): Array mit den eraknnten Linien
        max_right_line (array): Array mit Anfangs- und Endkoordinaten, Anstieg und Y-Achsenabschnitt der rechten Begrenzungslinie mit der größten Y-Koordinate
        height (integer): Bildhöhe

    Rückgabe:
        array: Array mit den Anfangs- und Endkoordinaten, Anstieg, Y-Achsenabschnitt der Linie
    """
    lanesy2 =[]
    if linesP is not None:
        for k,lines in enumerate(linesP):
            if lines[4] <= max_left_line[4] + 0.75*max_left_line[4] and lines[4] >= max_left_line[4] - 0.75*max_left_line[4] : 
                a = lines[1]
                lanesy2.append([k,a]) 
        y2 = height
        index = 0
        for i in lanesy2:
            if y2 >= i[1] :
                y2 = i[1]
                index = i[0]  
        return linesP[index]
    else:
        return None

def detected_lane_lines(frame, linesP):
    """ Ermittelt die Anfangs- und Endkoordinaten der Begrenzungslinien

    Argumente:
        frame (array): Bild
        linesP (array): 

    Rückgabe:
        _type_: _description_
    """
    lane_lines = []

    height, width, _ = frame.shape
    left_fit = []
    right_fit = []
    right_line = [] 
    left_line = [] 

    boundary = 1/6
    left_region_boundary = width * boundary  
    right_region_boundary = width * (1 - boundary) 
    if linesP is not None:
        for lineP in linesP:
            for x1, y1, x2, y2 in lineP:
                if x1 == x2:
                    continue
                fit = np.polyfit((x1, x2), (y1, y2), 1)
                slope = fit[0]
                intercept = fit[1]
                if slope < 0:
                    if x1 < left_region_boundary and abs(slope) > 0.05:
                        left_fit.append((x1, y1, x2, y2, slope, intercept))
                else:
                    if x2 > right_region_boundary and abs(slope) > 0.05:
                        right_fit.append((x1, y1, x2, y2, slope, intercept))
    if len(right_fit) > 0:
        right_line.append(min_right_line(right_fit,max_right_line(right_fit),height))
        right_line.append(max_right_line(right_fit)) 
    if len(left_fit) > 0:
        left_line.append(max_left_line(left_fit))
        left_line.append(min_left_line(left_fit, max_left_line(left_fit),height)) 
    if len(left_line) > 0:
        lane_lines.append(line_points(frame, left_line))
    else:
        lane_lines.append(None)
    if len(right_line) > 0:
        lane_lines.append(line_points(frame, right_line))
    else:
        lane_lines.append(None)   
    return lane_lines

def detected_lane(frame):
    """Gibt die Anfangs- und Endkoordinaten der erkannten Begrenzungslinien aus

    Argumente:
        frame (array): Bild

    Rückgabe:
        array: Anfangs- und Endkoordinaten der erkannten Begrenzungslinien
    """
    edges = detected_edges(frame)
    lines = detected_lines(edges)
    lane = detected_lane_lines(frame, lines)
    return lane

def display_lane(frame, lane):
    """Stellt die erkannten Begrenzungslinien graphisch dar

    Argumente:
        frame (array): Bild
        lane (array): Array mit Anfangs- und Endkoordinaten der Begrenzungslinien

    Rückgabe:
        array: Bild mit graphisch dargestellten Begrenzungslinien
    """
    line_image = np.zeros_like(frame)
    if lane is not None:
        for i in range(0, len(lane)):
            if lane[i] is not None:
                cv2.line(line_image, (lane[i][0], lane[i][1]), (lane[i][2], lane[i][3]), (255,0,0),2, cv2.LINE_AA)
    line_image = cv2.addWeighted(frame, 0.8, line_image, 1, 1)
    return line_image

def calculate_steering_angle(frame, lane_lines):
    """Ermittelt den Einschlagwinkel der Räder

    Argumente:
        frame (array): Bild
        lane_lines (array): Array mit Anfangs-, Endkoordinaten, Anstieg, Y-Achsenabschnitt der Begrenzungslinien

    Rückgabe:
        float: Einschlagwinkel
    """
    height, width, _ = frame.shape
    global fahrtrichtung
    if lane_lines[0] is None and lane_lines[1] is None:
        if fahrtrichtung == 'r':
                slope = -0.05
        elif fahrtrichtung == 'l':
                slope = 0.05
        else:
            steering_angle = 0
            return steering_angle
    if lane_lines[0] is None and lane_lines[1] is not None:
        if fahrtrichtung == 0:
                fahrtrichtung = 'l'
        if lane_lines[1][1] < lane_lines[1][3] + 20 and lane_lines[1][1] > lane_lines[1][3] - 20:
                slope = 0.05
        else:    
            slope = (lane_lines[1][1] - lane_lines[1][3])/(lane_lines[1][0] - lane_lines[1][2])  
    elif lane_lines[1] is None and lane_lines[0] is not None:
        if fahrtrichtung == 0:
            fahrtrichtung = 'r'
        if lane_lines[0][1] < lane_lines[0][3] + 20 and lane_lines[0][1] > lane_lines[0][3] - 20:
            slope = -0.05
        else:    
            slope = (lane_lines[0][1] - lane_lines[0][3])/(lane_lines[0][0] - lane_lines[0][2])
    elif lane_lines[0] is not None and lane_lines[1] is not None:
        if (lane_lines[0][3] < lane_lines[1][1] and fahrtrichtung == 0):
            fahrtrichtung = 'r'
        elif (lane_lines[0][3] > lane_lines[1][1] and fahrtrichtung == 0):
            fahrtrichtung = 'l'
        x1 = width/2
        y1 = height
        x2 = (lane_lines[0][2] + lane_lines[1][0])/2
        y2 = (lane_lines[0][3] + lane_lines[1][1])/2
        if x1 == x2:
            steering_angle = 0
            return steering_angle
        else:
            slope = (y1 - y2)/(x1 - x2)    
    angle_to_mid_radian = math.atan(slope)  # angle (in radian) to center vertical line
    angle_to_mid_deg = int(angle_to_mid_radian * 180.0 / math.pi)  
    steering_angle = angle_to_mid_deg  
    return steering_angle

def display_heading_line(frame, steering_angle):
    """Stellt die Fahrtroute graphisch dar

    Argumente:
        frame (array): Bild
        steering_angle (float): Einschlagwinkel der Räder

    Rückgabe:
        array: Bild mit graphisch dargestellter Fahrtroute
    """
    heading_image = np.zeros_like(frame)
    height, width, _ = frame.shape
    steering_angle_radian = steering_angle / 180.0 * math.pi
    x1 = int(width / 2)
    y1 = height
    if steering_angle == 0:
        x2 = x1
    else:
        x2 = int(-(y1 - math.tan(steering_angle_radian)*x1) / math.tan(steering_angle_radian))
    y2 = 0
    cv2.line(heading_image, (x1, y1), (x2, y2), (0,0,255), 1)
    heading_image = cv2.addWeighted(frame, 0.8, heading_image, 2, 1)
    return heading_image

def servo_steering(angle):
    """Ermittlet den Stellwinkel für den Servo aus dem ermittelten Einschlagwinkel der Räder und lässt den Servo auf die entsprechende Position fahren

    Argumente:
        angle (float): Einschlagwinkel der Räder
    """
    sangle = int(servo.angle)
    if angle < 0:
        winkel = int((90 - abs(angle)))
        if abs(sangle - winkel) > 30:
            time.sleep(0.3)
            if sangle > winkel:
                for i in range(sangle, winkel - 1, -5):
                    servo.angle = i
                    
                    
            else:
                for i in range(sangle, winkel + 1, 5):
                    servo.angle = i
                    
        else:
            servo.angle = winkel
    elif angle > 0:
        winkel = int((-90 + abs(angle)))
        if abs(sangle - winkel) > 30:
            time.sleep(0.3)
            if sangle < winkel:
                for i in range(sangle, winkel + 1, 5):
                    servo.angle = i
                    
            for i in range(sangle, winkel-1, -5):
                servo.angle = i
                
        else:
            servo.angle = winkel

    else:
        servo.angle = 0

def count_r ():
    """Gibt True zurück, wenn 3 Runden gefahren wurden

    Rückgabe:
        bool: True, wenn 3 Runden gefahren wurden; False sonst
    """
    global runden
    global start
    color_rgb = sensor.color_rgb_bytes
    print(color_rgb)
    if runden == 0:
        if color_rgb == blue_color:
            runden += 1
            start = time.time()
    else:
        stop = time.time()
        if (color_rgb == blue_color or color_rgb == (8,8,45) or color_rgb == (8,45,45) or color_rgb == (16,16,45)) and stop - start >= 1:
            runden += 1
            start = stop
    if runden >= 12:
        return True
    else:
        return False
        

# Warten auf Umlegen des Schalters[button] 
button.wait_for_press()

servo.angle = 0
runden = 0
start_time = 0

while True:
    ret, frame = cap.read()
    frame = cv2.rotate(frame, cv2.ROTATE_180)
    roi = creating_roi(frame)
    l = detected_lane(roi)
    angle = calculate_steering_angle(roi, l)
    servo_steering(angle)
    img2 = display_heading_line(roi, angle)
    img = display_lane(img2,l)
    cv2.imshow('frame', img)
    forward(0.8)
    count_r()
    print(runden)
    if count_r():
        if start_time == 0:
            start_time = time.time()
        stop = time.time()
        if stop - start_time >= 2:
            break
    if cv2.waitKey(1) & button.is_pressed == False:
        break
cap.release()
cv2.destroyAllWindows()

