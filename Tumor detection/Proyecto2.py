import cv2
import numpy as np
import math
import matplotlib.pyplot as plt

#######################  CARGA DE IMAGENES  ###################
t1 = "t1.jpg"
t2 = "t2.jpg"

Img = cv2.imread(t1, cv2.IMREAD_GRAYSCALE)
Img_RS = cv2.resize(Img, (0, 0), fx=1.5, fy=1.5)

####################  FILTROS Y SEGMENTACIÓN  #################
Img_FBL = cv2.bilateralFilter(Img_RS, 11, 85, 85)
Img_FM = cv2.medianBlur(Img_RS, 13)

_,thresh1 = cv2.threshold(Img_FBL,120,255,cv2.THRESH_BINARY) #120, 253

####################  OPERACIONES MORFOLÓGICAS  ###############

ker = np.ones((7, 7), np.uint8)
Img_Er = cv2.erode(thresh1, ker, iterations=2)
kdi = np.ones((9, 9), np.uint8)
Img_Dil = cv2.dilate(Img_Er, kdi, iterations=2)

#####################  BORDES Y CONTORNOS  #####################
Img_canny = cv2.Canny(Img_Dil, 100, 200)
copy = Img_RS.copy()
copy = cv2.cvtColor(copy, cv2.COLOR_GRAY2BGR)

contours, _ = cv2.findContours(Img_canny, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
copy = cv2.cvtColor(Img_RS, cv2.COLOR_GRAY2BGR)
cv2.drawContours(copy, contours, -1, (200, 0, 200), 2)

titles = ['Imagen Original', 'Máscara de tumor', 'Detección de Tumor']
images = [Img_RS, Img_Dil, copy]

plt.figure(figsize=(10, 5))
for i in range(3):
    plt.subplot(1, 3, i + 1)
    plt.imshow(images[i], cmap='gray' if i < 2 else None)
    plt.title(titles[i])
    plt.xticks([]), plt.yticks([])

plt.show()

#cv2.imshow("Imagen original", Img_RS)
#cv2.imshow("Imagen con filtro de media", Img_FBL)
#cv2.imshow("Mascara de tumor", thresh1)
#cv2.imshow("Ap", copy)
#cv2.waitKey(0)