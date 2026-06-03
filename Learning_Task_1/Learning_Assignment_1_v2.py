import cv2
import numpy as np
import matplotlib.pyplot as plt
import os
import sys

def detect_arrow_direction(image_path):
    img = cv2.imread(image_path)
    if img is None: 
        return None, "NONE"

    display_img = img.copy()
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    direction = "NONE" 
    
    blurred = cv2.GaussianBlur(gray, (7, 7), 0)
    
    thresh = cv2.adaptiveThreshold(blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, 
                                   cv2.THRESH_BINARY_INV, 15, 10)

    kernel = np.ones((3,3), np.uint8)
    morphed = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(morphed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    candidates = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if 100 < area < 40000:
            x, y, w, h = cv2.boundingRect(cnt)
            aspect_ratio = float(w) / h
            
            if 1.5 < aspect_ratio < 6.0:
                rect_area = w * h
                solidity = float(area) / rect_area
                
                if solidity > 0.4:
                    M = cv2.moments(cnt)
                    if M['m00'] != 0:
                        cx = int(M['m10'] / M['m00'])
                        temp_dir = "LEFT" if cx > (x + w/2) else "RIGHT"
                        candidates.append((area, temp_dir, cnt, (x, y, w, h)))

    if candidates:
        img_center_x = img.shape[1] / 2
        candidates.sort(key=lambda c: abs((c[3][0] + c[3][2]/2) - img_center_x))
        
        _, direction, cnt, bbox = candidates[0]
        x, y, w, h = bbox
        
        cv2.drawContours(display_img, [cnt], -1, (0, 255, 0), 3)
        cv2.rectangle(display_img, (x, y), (x+w, y+h), (255, 0, 0), 2)
        cv2.putText(display_img, direction, (x, y-15), 0, 1.2, (0, 255, 0), 3)

    return direction

if __name__ == "__main__":
    print(detect_arrow_direction(sys.argv[1]))