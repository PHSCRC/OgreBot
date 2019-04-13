#!/usr/bin/env python3
'''Records measurments to a given file. Usage example:
$ ./record_measurments.py out.txt'''
import sys
from rplidar import RPLidar


PORT_NAME = '/dev/ttyUSB0'


class Reading (object) :

    def __init__ (self, angle, distance) :
        self.angle = angle
        self.distance = distance

    def __lt__ (self, other) :
        return self.angle < other.angle


def get_distances():
    distances = []
    reading = False
    try:
        for measurement in lidar.iter_measurments() :
            if measurement[0] :
                reading = True

            if reading :
                reading = not measurement[0]
                a = measurement[2]
                d = measurement[3]
                read = Reading(a, d)
                distances.append(read)
            else :
                lidar.stop()
                break


    gottenTrue = False
    toggle = 0
    try:
        for measurement in lidar.iter_measurments() :
            if (measurement[0]):
                gottenTrue = not gottenTrue
                toggle += 1
            if (gottenTrue):
                a = measurement[2]
                d = measurement[3]
                read = Reading(a, d)
                distances.append(read)
            if (toggle == 2):
                break
    except:
        pass

    distances.sort()
    return distances

def run():
    '''Main function'''
    lidar = RPLidar(PORT_NAME)
    # outfile = open(path, 'w')
    while True:
        try:
            for measurment in lidar.iter_measurments():
                line = '\t'.join(str(v) for v in measurment)
                # outfile.write(line + '\n')
        except:
            pass




    lidar.stop()
    lidar.disconnect()

    # outfile.close()

if __name__ == '__main__':
    run()
