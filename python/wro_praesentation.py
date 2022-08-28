import multiprocessing
from math import cos, sin, pi, floor
from rplidar import RPLidar
import numpy as np
import cv2
import time
import math

#Initialisierung der Verbindung mit RPLidar
PORT_NAME = '/dev/ttyUSB0'
lidar = RPLidar(PORT_NAME)

#Verbindung mit Kamera
cap = cv2.VideoCapture(0)   

#Festlegung Bildaufl�sung Kamera
ret = cap.set(3,640)
ret = cap.set(4,480) 

#Definition der Farbwerte für die Masken
lower_black = np.array([0, 0, 0])
upper_black = np.array([180, 255, 76])
lower_green = np.array([51,191,46])
upper_green = np.array([82,255,103])
lower_red1 = np.array([158, 0, 0])
upper_red1 = np.array([180, 255, 255])
low_H2 = 0
high_H2 = 5

fahrtrichtung = 0

def create_roi(frame):
    """Gibt die untere Bildhälfte zurück

    Argumente:
        frame (array): Bild

    Rückgabe:
        array: untere Bildhälfte
    """    
    height, width, _ = frame.shape
    roi = frame[int(1/2*height):height,0:width] #Nimmt untere Bildhälfte als 'Region of Interest'
    return roi

def detected_edges(frame):
    """ Funktion zum Erkennen aller Kanten im Bild

    Argumente:
        frame (array): Bild

    Rückgabe:
        array: Bild mit allen erkannten Kanten
    """    
    hsv_frame = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV) #Umwandeln von BGR in HSV Farbschema
    global lower_black
    global upper_black
    black_mask = cv2.inRange(hsv_frame, lower_black, upper_black)   #Erzeugt Maske für schwarze Begrenzungen
    #Anwendung von Bildbearbeitungsverfahren zur Rauschunterdrückung
    pic = cv2.medianBlur(black_mask, 5)
    kernel = np.ones((5,5),np.uint8)
    opening = cv2.morphologyEx(pic, cv2.MORPH_OPEN, kernel)
    #Funktion zur Kantendetektion im Bild
    e = cv2.Canny(image=opening, threshold1=0, threshold2=100)
    cv2.imshow('canny', e)  #Zeigt Bild mit erkannten Kanten
    return e

def detected_lines(detected_edges):
    """ Funktion zum Erkennen von Linien

    Argumente:
        detected_edges (array): Bild

    Rückgabe:
        array: Array mit Anfangs- und Endkoordinaten aller erkannten Linien 
    """    
    d_lines = cv2.HoughLinesP(detected_edges, 1, np.pi / 180 , 15, None, 1, 20) #Funktion zur Detektion von Linien im Bild
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
    #Passt die erkannten Begrenzungslinien auf die Bildgröße an und ermittelt Anfangs- und Endpunkte
    if line[0][4]  < 0: #für linke Begrenzung
        x1 = 0
        y1 = int(line[0][5])
        x2 = line[1][2]
        y2 = line[1][3]  
        return ([x1, y1, x2, y2]) 
    elif line[0][4] > 0:    #für rechte Begrenzung
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
    lanesy2 =[] #Array zum Speichern der Y-Koordinate der Endpunkte
    if linesP is not None:
        for lines in linesP:
            a = lines[3]
            lanesy2.append(a)   #Füllen des Array mit Y-Koordinaten der Endpunkte der erkannten Linien
        y2index = 0 
        for i, y2 in enumerate(lanesy2):
            if y2 == max(lanesy2):  #Ermittlung der größten Y-Koordinate
                y2index = i
        return linesP[y2index]  #Rückgabe der Linie mit größter Y-Koordinate im Endpunkt
    else:
        return None

def min_right_line(linesP, max_right_line, height):
    """Gibt die rechte Begrenzungslinie mit der kleinsten Y-Koordinate zurück

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
            if lines[4] <= max_right_line[4] + 0.75*max_right_line[4] and lines[4] >= max_right_line[4] - 0.75*max_right_line[4]:   
                a = lines[3]
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
    
def max_left_line(linesP):
    """Gibt die linke Begrenzungslinie mit der größten Y-Koordinate zurück

    Argumente:
        linesP (array): Array mit erkannten Linien

    Rückgabe:
        array: Array mit den Anfangs- und Endkoordinaten, Anstieg, Y-Achsenabschnitt der Linie
    """
    lanesy1 = []    #Array zum Speichern der Y-Koordinate der Anfangspunkte
    if linesP is not None:
        for lines in linesP:
            a = lines[1]
            lanesy1.append(a)   #Füllen des Array mit Y-Koordinaten der Anfangspunkte der erkannten Linien
        y1index = 0 
        for i, y1 in enumerate(lanesy1):
            if y1 == max(lanesy1):  #Ermittlung der größten Y-Koordinate
                y1index = i
        return linesP[y1index]  #Rückgabe der Linie mit größter Y-Koordinate im Anfangspunkt
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
    lanesy2 =[]  #Array zum Speichern der Y-Koordinaten der Anfangspunkte
    if linesP is not None:
        for k,lines in enumerate(linesP):
            if lines[4] <= max_left_line[4] + 0.75*max_left_line[4] and lines[4] >= max_left_line[4] - 0.75*max_left_line[4] : 
                a = lines[1]
                lanesy2.append([k,a])   #Füllen des Arrays mit den Y-Koordinaten der Anfangspunkte der Linien, die einen ähnlichen Anstieg wie die als Argument übergebene Linie haben
        y2 = height
        index = 0
        #Ermittlung des Index
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
        linesP (array): Array mit allen erkannten Linien im Bild

    Rückgabe:
        array: Array mit den Anfangs- und Endkoordinaten der erkannten Begrenzungslinien
    """
    lane_lines = [] #Array zum Speichern der Begrenzungslinien
    height, width, _ = frame.shape
    #Arrays zum Auswerten der erkannten Linien im Bild
    left_fit = []
    right_fit = []
    right_line = [] 
    left_line = [] 
    #Feslegung der Bildbereiche, in denen die BEgrenzungslinien erkannt werden sollen
    boundary = 1/3
    left_region_boundary = width * boundary  
    right_region_boundary = width * (1 - boundary) 
    if linesP is not None:
        for lineP in linesP:
            for x1, y1, x2, y2 in lineP:
                if x1 == x2:    #Ausschluss von vertikalen Linien
                    continue
                fit = np.polyfit((x1, x2), (y1, y2), 1) #Funktion zum Ermitteln der Parameter der Polynomfunktionsgleichungen
                slope = fit[0]
                intercept = fit[1]
                if slope < 0:   #Funktionen mit negativen Anstieg für linke Begrenzung
                    if x1 < left_region_boundary and abs(slope) > 0.075:    
                        left_fit.append((x1, y1, x2, y2, slope, intercept))
                else:   #Funktionen mit positiven Anstieg für rechte Begrenzung
                    if x2 > right_region_boundary and abs(slope) > 0.075:
                        right_fit.append((x1, y1, x2, y2, slope, intercept))
    if len(right_fit) > 0:
        #Ermittlung der Anfangs- und Endkoordinaten der rechten Begrenzungslinie
        right_line.append(min_right_line(right_fit,max_right_line(right_fit),height))
        right_line.append(max_right_line(right_fit)) 
    if len(left_fit) > 0:
        #Ermittlung der Anfangs- und Endkoordinaten der linken Begrenzungslinie
        left_line.append(max_left_line(left_fit))
        left_line.append(min_left_line(left_fit, max_left_line(left_fit),height)) 
    if len(left_line) > 0:
        #Prüfung, ob linke Begrenzungslinie erkannt
        lane_lines.append(line_points(frame, left_line))
    else:
        lane_lines.append(None)
    if len(right_line) > 0:
        #Prüfung, ob rechte Begrenzungslinie erkannt
        lane_lines.append(line_points(frame, right_line))
    else:
        lane_lines.append(None)   
    return lane_lines

def detected_obstacles(frame):
    """ Ermittelt, ob Objekte erkannt wurden

    Argumente:
        frame (array): Bild 

    Rückgabe:
        array: Array mit Koordinaten der linken unteren Ecke des grünen Objekts und rechter unteren Ecke des roten Objekts (falls erkannt)
    """
    #Arrays zur Auswertung der erkannten Objekte
    g_corner = [] 
    r_corner = [] 
    obstacles = [] 
    #Anwendung von Bildbearbeitungsverfahren zur Rauschunterdrückung
    blurred_frame = cv2.medianBlur(frame, 5)
    hsv=cv2.cvtColor(blurred_frame, cv2.COLOR_BGR2HSV)  #Umwandlung in HSV Farbschema
    #Maske grün
    #mit [Winkel Farbkreis (0° Rot, 120° grün, 240° blau, 360° rot-> 0-255), Farbsätigung 0-255, Helligkeit 0-255]
    #Untere Grenze:
    global lower_green
    #obere Grenze:
    global upper_green
    #Anwendung grüne Maske
    g_mask = cv2.inRange(hsv, lower_green, upper_green)
    #Maske rot
    #mit [Winkel Farbkreis (0° Rot, 120° grün, 240° blau, 360° rot-> 0-180, Farbsätigung 0-255, Helligkeit 0-255]
    #Da Rot an unterer und oberer Intervallgrenze liegt (da eigentlich Kreis) werden teils zwei Masken benötigt
    #untere Grenze1:
    global lower_red1
    #obere Grenze1:
    global upper_red1
    #untere Grenze2:
    global low_H2
    #obere Grenze2:
    global high_H2
    #Erstellung red2
    low_H, low_S, low_V = lower_red1
    high_H, high_S, high_V = upper_red1
    lower_red2 = np.array([low_H2, low_S, low_V])
    upper_red2 = np.array([high_H2, high_S, high_V])
    #Anwendung rote Maske
    r_mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    r_mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    r_mask = cv2.addWeighted(r_mask1,1,r_mask2,1,0)
    #Anzeige rote und grüne Maske
    cv2.imshow("r_mask", r_mask) 
    cv2.imshow("g_mask", g_mask)
    #Funktion zur Detektion aller grünen Objekte in Bild
    g_contours,_ = cv2.findContours(g_mask,cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    green_contours =[] 
    if g_contours is not None:
        for cnt in g_contours:
            #Berechnung der Fläche der Kontur   
            area = cv2.contourArea(cnt)
            #Aussortierung aller Konturen deren Flächeninhalt kleiner als angegebene Grenze ist
            if area > 90:
                #Abstarktion der Objekte als Rechteck
                peri = cv2.arcLength(cnt, True)
                approx = cv2.approxPolyDP(cnt, 0.02*peri,True)  
                x, y, w, h = cv2.boundingRect(approx)
                green_contours.append([x,y,w,h])
        #Ermittlung des nähsten Objektes
        gy = []
        if len(green_contours) > 0:
            for l in green_contours:
                a = l[1]
                gy.append(a)
            gyindex = 0 
            for i, y1 in enumerate(gy):
                if y1 == max(gy):
                    gyindex = i
            g_corner.append(green_contours[i][0])
            g_corner.append(green_contours[i][1]+green_contours[i][3])
            cv2.circle(frame,(int(g_corner[0]), int(g_corner[1])), 1, (255,0,0), -1) 
            obstacles.append(g_corner)
        else:
            obstacles.append(None)
    else:
        obstacles.append(None)
        
            

    #Funktion zur Detektion aller roten Objekte im Bild
    r_contours,_ = cv2.findContours(r_mask,cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    red_contours =[] 
    if r_contours is not None:
        for cnt in r_contours:
            #Berechnung der Fläche der Kontur   
            area = cv2.contourArea(cnt)
            #Aussortierung aller Konturen deren Fläche kleiner als angegebene Grenze ist
            if area > 90:
                #Abstraktion der Objekte als Rechteck
                peri = cv2.arcLength(cnt, True)
                approx = cv2.approxPolyDP(cnt, 0.02*peri,True)  
                x, y, w, h = cv2.boundingRect(approx)
                red_contours.append([x,y,w,h])
        #Ermittlung des nähsten Objekts
        ry = []
        if len(red_contours) > 0:
            for l in red_contours:
                a = l[1]
                ry.append(a)
            ryindex = 0 
            for i, y1 in enumerate(ry):
                if y1 == max(ry):
                    ryindex = i
            r_corner.append(red_contours[i][0]+red_contours[i][2])
            r_corner.append(red_contours[i][1]+red_contours[i][3])
            cv2.circle(frame,(int(r_corner[0]), int(r_corner[1])), 1, (255,0,0), -1) 
            obstacles.append(r_corner)
        else:
            obstacles.append(None)
    else:
        obstacles.append(None)
    return obstacles

def fahrtrichtung_bestimmen(lane_lines):
    """ Ermittlung der Fahrtrichtung

    Argumente:
        lane_lines (array): Array mit erkannten Begrenzungslinien
    """
    global fahrtrichtung
    #Prüfung, ob und welche Begrenzungen erkannt wurden, und Speicherung des Resultats in Variable fahrtrichtung
    if lane_lines[0] is None and lane_lines[1] is not None:
        if fahrtrichtung == 0:
            fahrtrichtung = 'l' 
    elif lane_lines[1] is None and lane_lines[0] is not None:
        if fahrtrichtung == 0:
            fahrtrichtung = 'r'
    elif lane_lines[0] is not None and lane_lines[1] is not None:
        if (lane_lines[0][3] < lane_lines[1][1] and fahrtrichtung == 0):
            fahrtrichtung = 'r'
        elif (lane_lines[0][3] > lane_lines[1][1] and fahrtrichtung == 0):
            fahrtrichtung = 'l'

def dlines (frame, lane_lines, obstacles):
    """ Gibt die entgültigen Anfangs- und Endkoordinaten unter Einbezug der erkannten Objekte zurück

    Argumente:
        frame (array): Bild 
        lane_lines (array): Array mit erkannten Begrenzungen
        obstacles (array): Array mit Eckkoordinaten der erkannten Objekte

    Rückgabe:
        array: Array mit entgültigen Anfangs- und Endkoordinaten der Begrenzungen unter Einbezug der erkannten Objekte
    """
    height, width, _ = frame.shape
    lines = []
    fahrtrichtung_bestimmen(lane_lines)
    global fahrtrichtung
    global runden
    #Prüfung aller möglichen Kombinationen an erkannten Objekten und Begrenzungen
    if obstacles[1] is not None or obstacles[0] is not None:   
        if obstacles[0] is not None and obstacles[1] is not None:  
            if  obstacles[0][1] > obstacles[1][1]:
                if lane_lines[0] is not None: 
                    y2 = lane_lines[0][3]
                    if obstacles[0][1]   == height:
                        obstacles[0][1] = 0
                    x2 = (y2 - obstacles[0][1])/((obstacles[0][1] - height)/(obstacles[0][0] - width)) + obstacles[0][0]  
                    lines.append(lane_lines[0])
                    lines.append([x2,y2,width,height])
                else:
                    lines.append(lane_lines[0])
                    lines.append([obstacles[0][0],obstacles[0][1],width,height])
            else:
                if lane_lines[1] is not None: 
                    y2 = lane_lines[1][1]
                    if obstacles[1][1]   == height:
                        obstacles[1][1] = 0 
                    x2 = (y2 - obstacles[1][1])/((obstacles[1][1] - height)/(obstacles[1][0] - 0)) + obstacles[1][0]
                    lines.append([0,height,x2,y2])
                    lines.append(lane_lines[1])
                else:
                    lines.append([0,height,obstacles[1][0],obstacles[1][1]])
                    lines.append(lane_lines[1])

        elif obstacles[0] is not None and obstacles[1] is None:
            if lane_lines[0] is not None: 
                y2 = lane_lines[0][3]
                if obstacles[0][1]   == height:
                    obstacles[0][1] = 0  
                x2 = (y2 - obstacles[0][1])/((obstacles[0][1] - height)/(obstacles[0][0] - width)) + obstacles[0][0]
                lines.append(lane_lines[0])
                lines.append([x2,y2,width,height])
            else:
                lines.append(lane_lines[0])
                lines.append([obstacles[0][0],obstacles[0][1],width,height])
        elif obstacles[0] is None and obstacles[1] is not None:
            if lane_lines[1] is not None: 
                y2 = lane_lines[1][1]
                if obstacles[1][1]   == height:
                    obstacles[1][1] = 0
                x2 = (y2 - obstacles[1][1])/((obstacles[1][1] - height)/(obstacles[1][0] - 0)) + obstacles[1][0]
                lines.append([0,height,x2,y2])
                lines.append(lane_lines[1])
            else:
                lines.append([0,height,obstacles[1][0],obstacles[1][1]])
                lines.append(lane_lines[1])
        return lines
    else:
        if lane_lines[0] is not None and lane_lines[1] is not None:  
            if lane_lines[0][3] > lane_lines[1][1]:
                y2 = lane_lines[1][1]
                x2 = (y2 - lane_lines[0][3])/((lane_lines[0][3] - lane_lines[0][1]  )/(lane_lines[0][2] - lane_lines[0][0]  )) + lane_lines[0][2]  
                lines.append([lane_lines[0][0],lane_lines[0][1],x2,y2])  
                lines.append(lane_lines[1])  
            else:
                y2 = lane_lines[0][3]
                x2 = (y2 - lane_lines[1][1])/((lane_lines[1][1] - lane_lines[1][3]  )/(lane_lines[1][0] - lane_lines[1][2]  )) + lane_lines[1][0] 
                lines.append(lane_lines[0]) 
                lines.append([x2,y2,lane_lines[1][2],lane_lines[1][3]]) 
        else:
            return lane_lines
        return lines 

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
    obstacles = detected_obstacles(frame)
    path = dlines(frame,lane,obstacles)
    return path

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
                cv2.line(line_image, (int(lane[i][0]), int(lane[i][1])), (int(lane[i][2]), int(lane[i][3])), (255,0,0),2, cv2.LINE_AA) #Funktion zur Darstellung der erkannten Begrenzungslinien im Bild
    line_image = cv2.addWeighted(frame, 0.8, line_image, 1, 1)  #Überlagerung Kamerabild und Bild mit erkannten Begrenzungen
    return line_image

def calculate_steering_angle(frame, lane_lines):
    """Ermittelt den Einschlagwinkel für die Räder

    Argumente:
        frame (array): Bild
        lane_lines (array): Array mit Anfangs-, Endkoordinaten, Anstieg, Y-Achsenabschnitt der Begrenzungslinien

    Rückgabe:
        float: Einschlagwinkel
    """
    height, width, _ = frame.shape
    global fahrtrichtung
    #Prüfung aller möglichen Kombinationen der Begrenzungslinien und Berechnung des Anstiegs entsprechend
    if lane_lines[0] is None and lane_lines[1] is None:
        if fahrtrichtung == 'r':
                slope = -0.05
        elif fahrtrichtung == 'l':
                slope = 0.05
        else:
            steering_angle = 0
            return steering_angle
    if lane_lines[0] is None and lane_lines[1] is not None:
        if lane_lines[1][1] < lane_lines[1][3] + 20 and lane_lines[1][1] > lane_lines[1][3] - 20:
                slope = 0.05
        else:    
            slope = (lane_lines[1][1] - lane_lines[1][3])/(lane_lines[1][0] - lane_lines[1][2])  
    elif lane_lines[1] is None and lane_lines[0] is not None:
        if lane_lines[0][1] < lane_lines[0][3] + 20 and lane_lines[0][1] > lane_lines[0][3] - 20:
            slope = -0.05
        else:    
            slope = (lane_lines[0][1] - lane_lines[0][3])/(lane_lines[0][0] - lane_lines[0][2])
    elif lane_lines[0] is not None and lane_lines[1] is not None:
        x1 = width/2
        y1 = height
        x2 = int((lane_lines[0][2] + lane_lines[1][0])/2)
        y2 = int((lane_lines[0][3] + lane_lines[1][1])/2)
        if x1 == x2:
            steering_angle = 0
            return steering_angle
        else:
            slope = (y1 - y2)/(x1 - x2)  
    #print(fahrtrichtung)  
    #Berechnung und Umformung des Winkels in Grad
    angle_to_mid_radian = math.atan(slope)  
    angle_to_mid_deg = int(angle_to_mid_radian * 180.0 / math.pi)  
    steering_angle = angle_to_mid_deg  
    return steering_angle

def depict_heading_line(frame, steering_angle):
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
    cv2.line(heading_image, (x1, y1), (x2, y2), (0,0,255), 1)   #Darstellung der Fahrtroute
    heading_image = cv2.addWeighted(frame, 0.8, heading_image, 2, 1)
    return heading_image

def find_lines (scan_data, img):
    """Erkennt Linien in den Messwerten des Lidarsensors

    Args:
        scan_data (array): Array mit Distanzen; Index steht f�r Winkel
        img (array): Bild, auf dem die Messwerte des Lidarsensors dargestellt werden

    Returns:
        array: Array mit Anfangs- und Endkoordinaten der erkannten Linien
    """
    height, width, _ = img.shape
    lidar_position =[int(width/2), int(height/2)]  #Feslegung der Position des Lidarsensors im Bild
    """for angle in range(0,90):   #Winkel von 0-90�
        distance = scan_data[angle]
        if distance > 0:                  
            radians = (angle-90) * pi / 180.0
            x = int((distance*cos(radians)*(width/2)/(4000)))   #Skalierung auf Bildschirmgr��e und Ermittlung der kartesischen Koordinaten
            y = int((distance * sin(radians)*(height) / (4000)))    #Skalierung auf Bildschirmgr��e und Ermittlung der kartesischen Koordinaten
            point = (lidar_position[0] + x ,lidar_position[1] + y)  #Position des Lidarsensors als Bezugspunkt
            cv2.circle(img,point,0,(255,255,255),-1)"""    #Darstellung der Punkte
    for angle in range(0,359): #Winkel von 270-359�
        distance = scan_data[angle]
        if distance > 0:                  
            radians = (angle-90) * pi / 180.0
            x = int((distance*cos(radians)*(width/2)/(5000)))   #Skalierung auf Bildschirmgr��e und Ermittlung der kartesischen Koordinaten
            y = int((distance * sin(radians)*(height) / (5000)))    #Skalierung auf Bildschirmgr��e und Ermittlung der kartesischen Koordinaten
            point = (lidar_position[0] + x ,lidar_position[1] + y)  #Position des Lidarsensors als Bezugspunkt
            cv2.circle(img,point,0,(255,255,255),3)    #Darstellung der Punkte
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    cv2.imshow('frame2',gray)   #Darstellung der Punktwolke
    d_lines = cv2.HoughLinesP(gray, 1, np.pi / 180 , 15, None, 30, 15)   #Funktion zur Ermittlung der Linien
    if d_lines is not None:
        for line in d_lines:
            cv2.line(img, (int(line[0][0] ), int(line[0][1])), (int(line[0][2]), int(line[0][3])), (255,0,0),3, cv2.LINE_AA)
    cv2.imshow('frame3',img)    #Darstellung der erkannten Linien
    return d_lines

def camera(z, exitall): 
    """Funktion zum Auswerten der Kamerabilder

    Args:
        z (integer): Z�hlvariable
        exitall (integer): Abbruchbedingung
    """

    while True:
        #z.value += 1    #Z�hler
        ret, frame = cap.read()
        frame = cv2.rotate(frame, cv2.ROTATE_180)   #Rotieren des Bildes um 180�
        roi = create_roi(frame)
        l = detected_lane(roi)
        angle = calculate_steering_angle(roi, l)
        img2 = depict_heading_line(roi, angle)
        img = display_lane(img2,l)
        cv2.imshow('frame', img)  #Darstellung des Bildes
        if cv2.waitKey(1) & 0xFF == ord('q') or exitall.value != 0: #Abbruchbedingungen
            exitall.value = 1
            break
    cap.release()
    cv2.destroyAllWindows()

def rplidar(z,exitall):
    """Funktion zum Auswerten der Daten des Lidarsensors

    Args:
        z (integer): Z�hlvariable
        exitall (integer): Abbruchbedingung
    """    
    scan_data =[0]*360  #Array f�r Distanzen
    blank_img = np.zeros((600,6000,3),np.uint8)  #Array zur Darstellung der Messwerte

    print(lidar.get_info())
    for scan in lidar.iter_scans():
        #z.value += 1    #Z�hler
        for (_, angle, distance) in scan:
            scan_data[min([359, floor(angle)])] = distance
        find_lines(scan_data,blank_img)
        blank_img = np.zeros((600,600,3),np.uint8)  #L�schen der alten Messwerte
        if cv2.waitKey(1) & 0xFF == ord('q') or exitall.value != 0: #Abbruchbedingungen
            exitall.value = 2
            break
    lidar.stop()
    lidar.disconnect()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    #Initialisierung der geteilten Variablen
    zaehler1 = multiprocessing.Value('i',0)
    zaehler2 = multiprocessing.Value('i',0)
    exitall = multiprocessing.Value('i',0)

    #Initialisierung der Prozesse
    p1 = multiprocessing.Process(target=camera,args=(zaehler1,exitall)) #Prozess f�r Kamera
    p2 = multiprocessing.Process(target=rplidar,args=(zaehler2,exitall))    #Prozess f�r Lidarsensor

    #Starten der Prozesse
    p1.start()
    p2.start()

    while True:
        #Ausgeben der Z�hlvariablen
        #print(f"camera counter: {zaehler1.value}")
        #print(f"lidar counter: {zaehler2.value}")
        if exitall.value != 0:  #Abbruchbedingung
            #Beenden der Prozesse
            p1.join()
            p2.join()
            #Schlie�en der Prozesse
            p1.terminate()
            p2.terminate()
            break