#credit to Team 2996 Cougars Gone Wired
import ntcore
import time

def init_vision_table():
    inst = ntcore.NetworkTableInstance.getDefault()
    table = inst.getTable("NoteDetections")
    areaPub = table.getDoubleArrayTopic("area").publish()
    xPub = table.getDoubleArrayTopic("x").publish()
    yPub = table.getDoubleArrayTopic("y").publish()
    inst.startClient4("Notedetector")
    inst.setServerTeam(3322)
    inst.startDSClient()

    return areaPub, xPub, yPub

if __name__ == '__main__':
    areaPub, xPub, yPub = init_vision_table()

    print("start publishing")
    start_time = time.time()
    curr_time = time.time()
    while curr_time - start_time < 50:
        time.sleep(1)

        test_area = [a*100 for a in range(10)]
        test_x = [x for x in range(0, 200, 10)]
        test_y = [y for y in range(0, 200, 10)]

        areaPub.set(test_area)
        xPub.set(test_x)
        yPub.set(test_y)
        curr_time = time.time()
    print("done")
