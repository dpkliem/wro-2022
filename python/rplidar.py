
from math import cos, sin, pi, floor
from rplidar import RPLidar, RPLidarException
import numpy as np
import cv2
import math
from gpiozero import Motor, Button, AngularServo    #gpiozero zur Ansteuerung der GPIO-Pins
from gpiozero.pins.pigpio import PiGPIOFactory 
import time 

factory = PiGPIOFactory()
motor_1 = Motor(forward = 6,backward = 13) 
servo = AngularServo(12,min_angle=-180,max_angle=180,min_pulse_width=5/10000,max_pulse_width=25/10000,frame_width=20/1000,pin_factory=factory)
height = 400
width = 400
servo_default_angle = -40

blank_img = np.zeros((height, width, 3),np.uint8)
height, width, _ = blank_img.shape
lidar_position =[int(width/2), int(height/2)] 
cv2.circle(blank_img,lidar_position,0,(0,255,255),1) 
PORT_NAME = '/dev/ttyUSB0'
lidar = RPLidar(PORT_NAME)

def forward(speed):
    """Funktion zum Vorw�rtsfahren

    Argumente:
        speed (float): Werte von 0 bis 1; 1 ist das Maximum
    """    
    motor_1.forward(speed)      

def stop():
    """ Funktion zum Anhalten
    """
    motor_1.stop()

def find_lines (scan_data):
    for angle in range(0,165):
        distance = scan_data[angle]
        if distance > 0:                  
            radians = (angle-90) * pi / 180.0
            x = int((distance*cos(radians)*(width/2)/(4000)))
            y = int((distance * sin(radians)*(height) / (4000)))
            point = (lidar_position[0] + x ,lidar_position[1] + y)
            cv2.circle(blank_img,point,0,(255,255,255),-1)
    for angle in range(195,359):
        distance = scan_data[angle]
        if distance > 0:                  
            radians = (angle-90) * pi / 180.0
            x = int((distance*cos(radians)*(width/2)/(4000)))
            y = int((distance * sin(radians)*(height) / (4000)))
            point = (lidar_position[0] + x ,lidar_position[1] + y)
            cv2.circle(blank_img,point,0,(255,255,255),-1) 
    gray = cv2.cvtColor(blank_img,cv2.COLOR_BGR2GRAY)
    cv2.imshow('frame2',gray)
    d_lines = cv2.HoughLinesP(gray, 1, np.pi / 180 , 30, None, 1, 10)
    line_img = np.zeros((height, width, 3),np.uint8)
    cv2.circle(blank_img,lidar_position,0,(0,255,255),1) 
    if d_lines is not None:
        for line in d_lines:
            cv2.line(line_img, (int(line[0][0] ), int(line[0][1])), (int(line[0][2]), int(line[0][3])), (255,0,0),1, cv2.LINE_AA)
    cv2.imshow('frame3',line_img)
    return d_lines  

def check_line(x1, y1, x2, y2, slope, intercept):
    b = False
    global height
    global width
    default_distance = int((1000*(width/2)/4000))
    if x1 != x2 and slope != 0:
        if abs((lidar_position[1] - intercept)/slope - lidar_position[0]) <= default_distance:
            b = True
        else:
            b = False  
    elif x1 == x2 and (abs(lidar_position[0]  - x1) <=  default_distance):
            b = True     
    else:
        b = False
    return b
        

def detect_lane(linesP):
    #Sortierung der erkannten Linien nach rechter und linker Begrenzung 
    lane_lines = [] #Array zum Speichern der Begrenzungslinien
    global height
    global width
    #Arrays zum Auswerten der erkannten Linien im Bild
    left_fit = []
    right_fit = []
    right_line = [] 
    left_line = [] 

    #Festlegung der Bildbereiche, in denen die Begrenzungslinien erkannt werden sollen
    boundary = width/2 
    if linesP is not None:
        for lineP in linesP:
            for x1, y1, x2, y2 in lineP:
                fit = np.polyfit((x1, x2), (y1, y2), 1) #Funktion zum Ermitteln der Parameter der Polynomfunktionsgleichungen
                slope = fit[0]
                intercept = fit[1]
                if slope < 0:   
                    if x1 < width/2 and check_line(x1, y1, x2, y2, slope, intercept):    
                        left_fit.append((x1, y1, x2, y2, slope, intercept))
                    elif x1 > width/2 and check_line(x1, y1, x2, y2, slope, intercept):
                        right_fit.append((x1, y1, x2, y2, slope, intercept))
                elif slope > 0:   
                    if x2 < width/2 and check_line(x1, y1, x2, y2, slope, intercept):
                        left_fit.append((x1, y1, x2, y2, slope, intercept))
                    elif x2 > width/2 and check_line(x1, y1, x2, y2, slope, intercept):
                        right_fit.append((x1, y1, x2, y2, slope, intercept))
                elif slope == np.inf:
                    if x1 < width/2 and check_line(x1, y1, x2, y2, slope, intercept):    
                        left_fit.append((x1, y1, x2, y2, slope, intercept))
                    elif x1 > width/2 and check_line(x1, y1, x2, y2, slope, intercept):
                        right_fit.append((x1, y1, x2, y2, slope, intercept))
    print(right_fit)
    print(left_fit)
    if len(right_fit) > 0:
        #Ermittlung der Anfangs- und Endkoordinaten der rechten Begrenzungslinie
        right_line.append(min_line(right_fit, height))
        right_line.append(max_line(right_fit)) 
    if len(left_fit) > 0:
        #Ermittlung der Anfangs- und Endkoordinaten der linken Begrenzungslinie
        left_line.append(min_line(left_fit, height)) 
        left_line.append(max_line(left_fit))
    if len(left_line) > 0:
        #Prüfung, ob linke Begrenzungslinie erkannt
        lane_lines.append(line_points(blank_img, left_line))
    else:
        lane_lines.append(None)
    if len(right_line) > 0:
        #Prüfung, ob rechte Begrenzungslinie erkannt
        lane_lines.append(line_points(blank_img, right_line))
    else:
        lane_lines.append(None)  

    for line in lane_lines:
        if line is not None:
            cv2.line(blank_img, (int(line[0] ), int(line[1])), (int(line[2]), int(line[3])), (0,0,255),1, cv2.LINE_AA)
    return lane_lines

def line_points(frame, line):
    """ Gibt die Anfangs- und Endkoordinaten der Begrenzungslinie zurück

    Argumente:
        frame (array): Bild
        line (array): Array mit Anfangs- und Endkoordinaten, Anstieg und Y-Achsenabschnitt der Linien

    Rückgabe:
        array: Anfangs- und Endkoordinaten der Begrenzungslinie
    """  
    if line[0][4] < 0:
        x2 = line[0][2]
        y2 = line[0][3]
        x1 = line[1][0]
        y1 = line[1][1]
        if x1 == x2:
            slope = np.inf
            intercept = None
        else:
            slope = (y2 - y1)/(x2 - x1)
            intercept = y2 - slope*x2

    else:
        x1 = line[0][0]
        y1 = line[0][1]
        x2 = line[1][2]
        y2 = line[1][3]
        if x1 == x2:
            slope = np.inf
            intercept = None
        else:
            slope = (y2 - y1)/(x2 - x1)
            intercept = y2 - slope*x2
    
    return [x1, y1, x2, y2, slope, intercept]

def max_line(linesP):
    """Gibt die Begrenzungslinie mit der größten Y-Koordinate zurück

    Argumente:
        linesP (array): Array mit den erkannten Linien

    Rückgabe:
        array: Array mit den Anfangs- und Endkoordinaten, Anstieg, Y-Achsenabschnitt der Linie
    """
    lanesy2 =[] #Array zum Speichern der Y-Koordinate der Endpunkte
    if linesP is not None:
        for lines in linesP:
            if lines[4] < 0:
                a = lines[1]
            else:
                a = lines[3]
        lanesy2.append(a)   #Füllen des Array mit Y-Koordinaten der Endpunkte der erkannten Linien
        y2index = 0 
        for i, y2 in enumerate(lanesy2):
            if y2 == max(lanesy2):  #Ermittlung der größten Y-Koordinate
                y2index = i
        return linesP[y2index]  #Rückgabe der Linie mit größter Y-Koordinate im Endpunkt
    else:
        return None

def min_line(linesP, height):
    """Gibt die Begrenzungslinie mit der kleinsten Y-Koordinate zurück

    Argumente:
        linesP (array): Array mit den erkannten Linien
        max_right_line (array): Array mit Anfangs- und Endkoordinaten, Anstieg und Y-Achsenabschnitt der rechten Begrenzungslinie mit der größten Y-Koordinate
        height (integer): Bildhöhe

    Rückgabe:
        array: Array mit den Anfangs- und Endkoordinaten, Anstieg, Y-Achsenabschnitt der Linie
    """
    lanesy1 =[] #Array zum Speichern der Y-Koordinaten der Endpunkte
    if linesP is not None:
        for k,lines in enumerate(linesP):
            if lines[4] < 0:   
                a = lines[3]
            else:
                a = lines[1]
            lanesy1.append([k,a]) #Füllen des Arrays mit den Y-Koordinaten der Endpunkte der Linien, die einen ähnlichen Anstieg wie die als Argument übergebene Linie haben
        y1 = height
        index = 0
        #Ermittlung des Index
        for i in lanesy1:   
            if y1 >= i[1]:
                y1 = i[1]
                index = i[0]  
        return linesP[index]
    else:
        return None

def calculate_angle(lane):
    global height
    global width
    slope = 0
    default_distance = int((1000*(width/2)/4000))
    print(lane)
    if lane[0] is not None and lane[1] is not None:
        if lane[0][1] >= lidar_position[1] and lane[0][3] >= lidar_position[1]:#Prüfe auf Linkskurve
             slope = 'l'
             print('l')
        elif lane[1][1] >= lidar_position[1] and lane[1][3] >= lidar_position[1]:
            slope = 'r'
            print('r')
        else:
            if (lane[0][4] + lane[1][4])/2 < 0:
                slope = ((lane[1][3] + lane[0][3])/2 - height)/((lane[1][2] + lane[0][2])/2 - width/2)
            elif (lane[0][4] + lane[1][4])/2 < 0:
                slope = (height - (lane[0][3] + lane[1][3])/2 )/(- (lane[0][2] + lane[1][2])/2 + width/2)     
    elif lane[0] is None and lane[1] is not None:
        slope = lane[1][4]
    elif lane[0] is not None and lane[1] is None:
        slope = lane[0][4]
    else:
        slope = 'n'

    if slope == 'l':
        return -180
    elif slope == 'r':
        return 180
    elif slope == 'n':
        return None
    else:
        angle_to_mid_radian = math.atan(slope)  
        angle_to_mid_deg = int(angle_to_mid_radian * 180.0 / math.pi)
        return angle_to_mid_deg

def depict_heading_line(frame, steering_angle, lane):
    """Stellt die Fahrtroute graphisch dar

    Argumente:
        frame (array): Bild
        steering_angle (float): Einschlagwinkel der R�der

    R�ckgabe:
        array: Bild mit graphisch dargestellter Fahrtroute
    """
    heading_image = np.zeros_like(frame)
    height, width, _ = frame.shape
    if steering_angle is not None:
        steering_angle_radian = steering_angle / 180.0 * math.pi
        x1 = lidar_position[0] 
        y1 = lidar_position[1] 
        if lane[0] is not None and lane[1]is not None:    
            x2 = int((lane[0][2] + lane[1][2])/2)
            y2 = int((lane[0][3] + lane[1][3])/2)
        elif lane[0] is None and lane[1] is not None:
            y2 = lane[1][3]
            x2 = int((y2 - y1)/math.atan(steering_angle) + x1)
        elif lane[0] is not None and lane[1] is None:
            y2 = lane[0][3]*abs(lane[0][3])
            x2 = int((y2 - y1)/math.atan(steering_angle) + x1)   
        cv2.line(heading_image, (x1, y1), (x2, y2), (0,255,0), 1)   #Darstellung der Fahrtroute
        heading_image = cv2.addWeighted(frame, 0.8, heading_image, 2, 1)
    return heading_image

def servo_steering(angle):
    """Ermittlet den Stellwinkel f�r den Servo aus dem ermittelten Einschlagwinkel der R�der und l�sst den Servo auf die entsprechende Position fahren

    Argumente:
        angle (float): Einschlagwinkel der R�der
    """
    sangle = int(servo.angle)
    

    if angle is None:
        servo.angle = servo.angle
        #nach Rechts
    elif angle < 0:
        winkel = int((90 - abs(angle)))
        servo.angle = (winkel + servo_default_angle)
    elif angle > 0:
        winkel = int((-90 + abs(angle)))
        servo.angle = int((winkel + servo_default_angle))
    else:
        servo.angle = servo_default_angle

    print(servo.angle +40)

scan_data =[0]*360 
servo.angle = servo_default_angle
forward(0.3)
print(lidar.get_info)
while True:
    lidar.connect()
    lidar.start_motor()
    try:
        for scan in lidar.iter_scans(scan_type='express', min_len = 100, max_buf_meas=False):
            for (_, angle, distance) in scan:
                scan_data[min([359, floor(angle)])] = distance
            lane = detect_lane(find_lines(scan_data))
            angle = calculate_angle(lane)
            frame4 = depict_heading_line(blank_img, angle, lane)
            cv2.imshow('frame4', frame4)
            servo_steering(angle)
            blank_img = np.zeros((height,width,3),np.uint8)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                raise KeyboardInterrupt
    except ValueError:
        lidar.stop()
        lidar.stop_motor()
        lidar.disconnect()
        continue
    except RPLidarException:
        lidar.stop()
        lidar.stop_motor()
        lidar.disconnect()
        continue
    except KeyboardInterrupt:
        break
lidar.stop()
lidar.stop_motor()
lidar.disconnect()
cv2.destroyAllWindows()