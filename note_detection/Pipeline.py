#Credit to team 2996 Cougars Gone Wired for the code

import cv2
import numpy
def run_pipeline(src):

    #blurs the image to make it easier to work with
    blur = cv2.GaussianBlur(src, (13, 13), 0)

    hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, (0, 150, 140), (12, 255, 255)) #adjust these color values to our own.
    mask = cv2.morphologyEX(mask, cv2.MORPH_CLOSE, (11, 11), iterations=10)
    


    canny = cv2.Canny(mask, 75, 180)


    contours, heiarchy = cv2.findContours(canny, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours_list = []
    area_list = []
    approx_list = []
    centers_list = []

    for contour in contours:
        approx = cv2.approxPolyDP(contour, 0.01*cv.arcLength(contour, False), False)
        area = cv2.contourArea(contour)
        if (len(approx) > 10) and (area > 100):
            contours_list.append(contour)
            approx_list.append(approx)
            area_list.append(area)

    for cnt, ara, aprx, i in zip(contours_list, area_list, approx_list, range(len(contours_list))):
        moment = cv.moments(cnt)
        cx = int(moment['m10']/moment['m00'])
        cy = int(moment['m01']/moment['m00'])
        #cv2.circle(src, (cx, cy), 2, (0, 0, 255))

        #Elliot add your distance to note estimation or use theirs
        #alternatively, we can change this to just return the x and y values and do the estimation in the main robot

        #centers_list.append((rx, ry))


    return (area_list, centers_list)


if __name__ == '__main__':
    run = True
    camera = cv2.VideoCapture(0)
    if not camera.isOpened():
        print("camera failed to open")
        run = False

    while run:
        ret, src = camera.read()
        if not ret:
            print("Bad frame... ending")
            break
        
        areas, centers = run_pipeline(src)


        
        key = cv2.waitKey(1) 
        if key == ord('q'):
            break

    cv2.destroyAllWindows()
    camera.release()




