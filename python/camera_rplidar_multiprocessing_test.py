import multiprocessing
from math import cos, sin, pi, floor
from adafruit_rplidar import RPLidar
import numpy as np
import cv2
import time

#Initialisierung der Verbindung mit RPLidar
PORT_NAME = '/dev/ttyUSB0'
lidar = RPLidar(None, PORT_NAME)

#Verbindung mit Kamera
cap = cv2.VideoCapture(0)   

#Festlegung Bildauflösung Kamera
ret = cap.set(3,320)
ret = cap.set(4,240) 

def find_lines (scan_data, img):
    """Erkennt Linien in den Messwerten des Lidarsensors

    Args:
        scan_data (array): Array mit Distanzen; Index steht für Winkel
        img (array): Bild, auf dem die Messwerte des Lidarsensors dargestellt werden

    Returns:
        array: Array mit Anfangs- und Endkoordinaten der erkannten Linien
    """
    height, width, _ = img.shape
    lidar_position =[int(width/2), height]  #Feslegung der Position des Lidarsensors im Bild
    for angle in range(0,90):   #Winkel von 0-90°
        distance = scan_data[angle]
        if distance > 0:                  
            radians = (angle-90) * pi / 180.0
            x = int((distance*cos(radians)*(width/2)/(4000)))   #Skalierung auf Bildschirmgröße und Ermittlung der kartesischen Koordinaten
            y = int((distance * sin(radians)*(height) / (4000)))    #Skalierung auf Bildschirmgröße und Ermittlung der kartesischen Koordinaten
            point = (lidar_position[0] + x ,lidar_position[1] + y)  #Position des Lidarsensors als Bezugspunkt
            cv2.circle(img,point,0,(255,255,255),-1)    #Darstellung der Punkte
    for angle in range(270,359): #Winkel von 270-359°
        distance = scan_data[angle]
        if distance > 0:                  
            radians = (angle-90) * pi / 180.0
            x = int((distance*cos(radians)*(width/2)/(4000)))   #Skalierung auf Bildschirmgröße und Ermittlung der kartesischen Koordinaten
            y = int((distance * sin(radians)*(height) / (4000)))    #Skalierung auf Bildschirmgröße und Ermittlung der kartesischen Koordinaten
            point = (lidar_position[0] + x ,lidar_position[1] + y)  #Position des Lidarsensors als Bezugspunkt
            cv2.circle(img,point,0,(255,255,255),-1)    #Darstellung der Punkte
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    cv2.imshow('frame2',gray)   #Darstellung der Punktwolke
    d_lines = cv2.HoughLinesP(gray, 1, np.pi / 180 , 10, None, 5, 10)   #Funktion zur Ermittlung der Linien
    for line in d_lines:
        cv2.line(img, (int(line[0][0] ), int(line[0][1])), (int(line[0][2]), int(line[0][3])), (255,0,0),1, cv2.LINE_AA)
    cv2.imshow('frame3',img)    #Darstellung der erkannten Linien
    return d_lines

def camera(z, exitall): 
    """Funktion zum Auswerten der Kamerabilder

    Args:
        z (integer): Zählvariable
        exitall (integer): Abbruchbedingung
    """
    while True:
        z.value += 1    #Zähler
        ret, frame = cap.read()
        frame = cv2.rotate(frame, cv2.ROTATE_180)   #Rotieren des Bildes um 180°
        cv2.imshow('frame', frame)  #Darstellung des Bildes
        if cv2.waitKey(1) & 0xFF == ord('q') or exitall.value != 0: #Abbruchbedingungen
            exitall.value = 1
            break
    cap.release()
    cv2.destroyAllWindows()

def rplidar(z,exitall):
    """Funktion zum Auswerten der Daten des Lidarsensors

    Args:
        z (integer): Zählvariable
        exitall (integer): Abbruchbedingung
    """    """""" 
    scan_data =[0]*360  #Array für Distanzen
    blank_img = np.zeros((100,200,3),np.uint8)  #Array zur Darstellung der Messwerte

    print(lidar.info)
    for scan in lidar.iter_scans():
        z.value += 1    #Zähler
        for (_, angle, distance) in scan:
            scan_data[min([359, floor(angle)])] = distance
        find_lines(scan_data,blank_img)
        blank_img = np.zeros((100,200,3),np.uint8)  #Löschen der alten Messwerte
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
    p1 = multiprocessing.Process(target=camera,args=(zaehler1,exitall)) #Prozess für Kamera
    p2 = multiprocessing.Process(target=rplidar,args=(zaehler2,exitall))    #Prozess für Lidarsensor

    #Starten der Prozesse
    p1.start()
    p2.start()

    while True:
        #Ausgeben der Zählvariablen
        print(f"camera counter: {zaehler1.value}")
        print(f"lidar counter: {zaehler2.value}")
        if exitall.value != 0:  #Abbruchbedingung
            #Beenden der Prozesse
            p1.join()
            p2.join()
            #Schließen der Prozesse
            p1.terminate()
            p2.terminate()
            break
