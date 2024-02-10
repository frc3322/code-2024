#Credit to team 2996 Cougars Gone Wired
import numpy as np
import cv2 as cv
import cameraServer
import Pipeline
import NTInstance



if __name__ == "__main__":
    run = True
    camera = cv.VideoCapture(0)
    if not camera.isOpened():
        print("camera failed to open")
        run = False

    areaPub, xPub, yPub = networkTables.init_vision_table()
    output = cameraServer.start_cameraServer()

    while run:
        ret, src = camera.read()
        if not ret:
            print("Bad frame... ending")
            break
        
        areas, centers = pipeline.run_pipeline(src)

        areaPub.set(areas)

        xs = [i[0] for i in centers]
        ys = [i[1] for i in centers]

        xPub.set(xs)
        yPub.set(ys)

        output.putFrame(src)

        key = cv.waitKey(1) 
        if key == ord('q'):
            break

    cv.destroyAllWindows()
    camera.release()
