import cv2
import numpy as np
import sys

def detect_arrow_direction(image_path):
    img = cv2.imread(image_path)
    if img is None: return None
    
    h, w, _ = img.shape
    roi = img[int(h*0.2):int(h*0.7), :]
    
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    
    thresh = cv2.adaptiveThreshold(blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, 
                                   cv2.THRESH_BINARY_INV, 11, 2)

    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    candidates = []

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if 200 < area < 20000:
            x, y, w_box, h_box = cv2.boundingRect(cnt)
            aspect_ratio = float(w_box) / h_box
            
            if 1.5 < aspect_ratio < 6.0:
                score = abs(aspect_ratio - 3)
                candidates.append((score, cnt))

    if not candidates:
        print("No arrow candidates found.")
        return None

    candidates.sort(key=lambda x: x[0])
    best_cnt = candidates[0][1]
    cv2.drawContours(roi,[best_cnt],-1,(0,255,0),1)
    cv2.imshow("contour",img)
    cv2.waitKey(0)

    M = cv2.moments(best_cnt)
    if M['m00'] == 0: return None
    cx = int(M['m10'] / M['m00'])

    leftmost_x = best_cnt[best_cnt[:,:,0].argmin()][0][0]
    rightmost_x = best_cnt[best_cnt[:,:,0].argmax()][0][0]

    dist_to_left = abs(cx - leftmost_x)
    dist_to_right = abs(cx - rightmost_x)

    direction = "LEFT" if dist_to_left > dist_to_right else "RIGHT"
    
    print(f"Best Arrow Match: {direction}")
    return direction

if len(sys.argv) > 1:
    detect_arrow_direction(sys.argv[1])
