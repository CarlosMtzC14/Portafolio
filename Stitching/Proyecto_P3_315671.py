import numpy as np
import cv2

Img_Or = cv2.imread("Img_Original.jpg", cv2.IMREAD_ANYCOLOR)

Pz1 = Img_Or[150:350, 600:800]
Pz1 = cv2.rotate(Pz1, cv2.ROTATE_90_CLOCKWISE)
Pz2 = Img_Or[150:350, 150:350]
Pz2 = cv2.rotate(Pz2, cv2.ROTATE_180)
Pz3 = Img_Or[500:700, 600:800]
Pz3 = cv2.rotate(Pz3, cv2.ROTATE_90_COUNTERCLOCKWISE)
Pz4 = Img_Or[500:700, 150:350]

Img_rec = Img_Or.copy()
cv2.rectangle(Img_rec, (600, 150), (800, 350), (0,0,0), -1)
cv2.rectangle(Img_rec, (150, 150), (350, 350), (0,0,0), -1)
cv2.rectangle(Img_rec, (600, 500), (800, 700), (0,0,0), -1)
cv2.rectangle(Img_rec, (150, 500), (350, 700), (0,0,0), -1)

sift = cv2.SIFT_create()
kp1, desc1 = sift.detectAndCompute(Img_Or, None)
bf = cv2.BFMatcher()

def imprimirMatches(img_pieza, nombre):

    kp2, desc2 = sift.detectAndCompute(img_pieza, None)
    matches = bf.knnMatch(desc1, desc2, k=2)

    good_matches = []
    for m, n in matches:
        if m.distance < 0.60 * n.distance:
            good_matches.append(m)

    src_pts = np.float32([kp1[m.queryIdx].pt for m in good_matches]).reshape(-1, 1, 2)
    dst_pts = np.float32([kp2[m.trainIdx].pt for m in good_matches]).reshape(-1, 1, 2)

    H, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 3.0)

    h1, w1 = Img_Or.shape[:2]
    h2, w2 = img_pieza.shape[:2]
    result_width = max(w1, w2)
    result_height = max(h1, h2)
    img_pieza_warped = cv2.warpPerspective(img_pieza, H, (result_width, result_height))
    Img_res = cv2.add(img_pieza_warped, Img_rec)
    cv2.imshow(nombre, Img_res)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

imprimirMatches(Pz1, "Pieza 1")
imprimirMatches(Pz2, "Pieza 2")
imprimirMatches(Pz3, "Pieza 3")
imprimirMatches(Pz4, "Pieza 4")