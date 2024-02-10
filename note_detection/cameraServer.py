import cscore
import cv2 as cv
import time

HEIGHT = 640
WIDTH = 480

def start_cameraServer():
    cscore.CameraServer.enableLogging()
    output = cscore.CameraServer.putVideo("Video", WIDTH, HEIGHT)
    return output

if __name__ == '__main__':
    print("starting server")

    output = start_cameraServer()

    camera = cv.VideoCapture(0)
    start_time = time.time()
    curr_time = time.time()
    while curr_time - start_time < 50:
        output.putFrame(camera.read()[1])
        curr_time = time.time()
    print("done")
    camera.release()
