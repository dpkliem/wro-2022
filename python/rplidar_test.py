
from math import cos, sin, pi, floor
from adafruit_rplidar import RPLidar
import numpy as np
import cv2

blank_img = np.zeros(shape=(400,400,3)) #Erstellt dreidimensionales Array mit Nullen zur Darstellung
width, height, _ = blank_img.shape

PORT_NAME = '/dev/ttyUSB0'
lidar = RPLidar(None, PORT_NAME)    #Verbindungsinilialisierung mit RPLidar
           
scan_data =[0]*360  #Array für Messwerte des Lidar-Sensors

print(lidar.info)
for scan in lidar.iter_scans(): #Messung wird durchgeführt
    for (_, angle, distance) in scan:
        scan_data[min([359, floor(angle)])] = distance  #Distanzen und Winkel werden an scan_data Array übergeben
    for angle in range(360):
        distance = scan_data[angle]
        if distance > 0:                  
            radians = angle * pi / 180.0
            x = distance * cos(radians) #Berechnung der kartesischen Koordinaten aus Distanz und Winkel
            y = distance * sin(radians) #Berechnung der kartesischen Koordinaten aus Distanz und Winkel
            point = (int(width/2 + int((x*width)/(1200*2)) ), int(height/2 + int((y*height) / (1200*2))))   #Skalierung der Messpunkte auf Arraygröße
            print(point)
            if angle == 0:
                cv2.circle(img=blank_img, center=point, radius=2, color=(0,0,255), thickness=-1)    #Zeichnen der Messpunkte auf Array
            else:    
                cv2.circle(img=blank_img, center=point, radius=2, color=(255,0,0), thickness=-1)    #Zeichnen der Messpunkte auf Array
    cv2.imshow('frame',blank_img)   #Darstellung der ermittelten Messpunkte
    blank_img = np.zeros(shape=(400,400,3)) #Löschen der alten Messung 
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

lidar.stop()
lidar.disconnect()
cv2.destroyAllWindows()