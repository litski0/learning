import cv2
import numpy as np
import math

class RedCircleDetector:
    def __init__(self):
        self.target_color=[0,0,255]

    def _get_limits(self,color):
        c= np.uint8([[color]])
        hsvC = cv2.cvtColor(c,cv2.COLOR_BGR2HSV)
        hue = int(hsvC[0][0][0])
        lowerLimit=max(hue - 10,0),100,100
        upperLimit=min(hue + 10,179),255,255

        lowerLimit=np.array(lowerLimit,dtype=np.uint8)
        upperLimit=np.array(upperLimit,dtype=np.uint8)

        return lowerLimit,upperLimit
    
    def detect(self,frame):
 

        inverted_frame = cv2.bitwise_not(frame)
        hsv_inverted = cv2.cvtColor(inverted_frame, cv2.COLOR_BGR2HSV)
        lower_cyan = np.array([80, 100, 50])
        upper_cyan = np.array([90, 255, 255])
        mask = cv2.inRange(hsv_inverted, lower_cyan, upper_cyan)
        
        
        mask = cv2.GaussianBlur(mask, (5, 5), 0)
        cv2.imshow("Debug: The Mask", mask)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Default return values (Nothing found)
        best_target = (False, 0, 0, 0)
        
        for cnt in contours:
            # Find largest blob
            area= cv2.contourArea(cnt)
            
            if area > 500:
                perimeter = cv2.arcLength(cnt, True)
                
                
                if perimeter > 0:
                    circularity = (4 * math.pi * area) / (perimeter * perimeter)
                    
                    # Strict Circle Check
                    if 0.7 < circularity < 1.2:
                        x, y, w, h = cv2.boundingRect(cnt)
                        aspect_ratio = float(w) / h
                        
                        if 0.8 < aspect_ratio < 1.2:
                            # It is a circle!
                            (x_center, y_center), radius = cv2.minEnclosingCircle(cnt)
                            center = (int(x_center), int(y_center))
                            radius = int(radius)
                            
                            # Draw debug info on frame
                            cv2.circle(frame, center, radius, (0, 255, 0), 2)
                            cv2.circle(frame, center, 2, (0, 0, 255), 3)
                            
                            best_target = (True, x_center, y_center, radius)

        return *best_target,frame
    

#unit test    
if __name__ == "__main__":
    cap = cv2.VideoCapture(0)
    detector = RedCircleDetector()
    
    print("TEST MODE: Press 'q' to quit")
    while True:
        ret, frame = cap.read()
        if not ret: break
        
        found, x, y, r, img = detector.detect(frame)
        
        if found:
            print(f"Target at: {x}, {y} | Radius: {r}")
            
        cv2.imshow("Test", img)
        if cv2.waitKey(1) & 0xFF == ord('q'): break
    
    cap.release()
    cv2.destroyAllWindows()
