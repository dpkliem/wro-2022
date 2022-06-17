#importiere Bibliotheken
import cv2 as cv2
import numpy as np
#impotiere Video von Quelle 0
videoCapture=cv2.VideoCapture(0)

while True:
    #weise der Variablen frame das Video zu
    _, frame=videoCapture.read()
    
    big_frame = cv2.rotate(frame, cv2.ROTATE_180)

    height, widht, _=big_frame.shape  
    frame = big_frame[int(1/2*height):height,0:widht]
        

    #bei blurred_frame wird Rauschen entfernt
    blurred_frame = cv2.medianBlur(frame, 5)
    #blurred_frame = cv2.GaussianBlur(frame, (5, 5), 0)
    #hier wird blurred_frame in ein hsv Farbmodell umgewandelt
    hsv=cv2.cvtColor(blurred_frame, cv2.COLOR_BGR2HSV)
    #Maske grün
    #mit [Winkel Farbkreis (0° Rot, 120° grün, 240° blau, 360° rot-> 0-255), Farbsätigung 0-255, Helligkeit 0-255]

    #Untere Grenze:
    lower_green = np.array([60,86,0])
    #obere Grenze:
    upper_green = np.array([120,255,255])
    #wende grüne Maske an
    g_mask = cv2.inRange(hsv, lower_green, upper_green)
    
    #Maske rot
    #mit [Winkel Farbkreis (0° Rot, 120° grün, 240° blau, 360° rot-> 0-255), Farbsätigung 0-255, Helligkeit 0-255]

    #Untere Grenze1:
    lower_red1 = np.array([160, 86, 0])
    #obere Grenze1:
    upper_red1 = np.array([180, 255, 255])

    #Untere Grenze2:
    lower_red2 = np.array([0, 86, 0])
    #obere Grenze2:
    upper_red2 = np.array([10, 255, 255])

    #wende rot Maske an
    r_mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    r_mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    r_mask = cv2.addWeighted(r_mask1,1,r_mask2,1,0)
    

    #finde grüne Konturen
    g_contours,_ = cv2.findContours(g_mask,cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    i=0
    #my_barriers=[b1,b2,b3]
    for cnt in g_contours:
        #berechne Fläche der Kontur   
        area = cv2.contourArea(cnt)
        #Alle Konturen kleiner 2000 werden weg gefiltert
        if area > 3000:
            #Zeichne Konturen
            cv2.drawContours(frame, cnt, -1, (255,0,255),3)
            #bestimme obere linke Ecke und alle anderen im Bezug auf diese
            peri = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.02*peri,True)  
            x, y, w, h = cv2.boundingRect(approx)
            #Zeichne Rechteck um Körper
            cv2.rectangle(frame, (x,y), (x+w,y+h), (0,255,0),2)
            #Zeichne Mittelpunkt ein
            cv2.circle(frame,(int(x+(w/2)), int(y+(h/2))), 4, (0,255,0), -1)
            #my_barriers[i]=[1,2]
            
            
    #finde rote Konturen
    r_contours,_ = cv2.findContours(r_mask,cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    for cnt in r_contours:
        #berechne Fläche der Kontur   
        area = cv2.contourArea(cnt)
        #Alle Konturen kleiner 2000 werden weg gefiltert
        if area > 3000:
            #Zeichne Konturen
            cv2.drawContours(frame, cnt, -1, (255,0,255),3)
            #bestimme obere linke Ecke und alle anderen im Bezug auf diese
            peri = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.02*peri,True)  
            x, y, w, h = cv2.boundingRect(approx)
            #Zeichne Rechteck um Körper
            cv2.rectangle(frame, (x,y), (x+w,y+h), (0,0,255),2)
            #Zeichne Mittelpunkt ein
            cv2.circle(frame,(int(x+(w/2)), int(y+(h/2))), 4, (255,0,0), -1) 
          

    
    cv2.imshow("frame", frame)
    cv2.imshow("g_mask", g_mask)
    cv2.imshow("r_mask", r_mask)
    #cv2.imshow("hsv", hsv)
    
    
    if cv2.waitKey(1)&0xFF==ord('q'): break

videoCapture.release()
cv2.destroyAllWindows()
