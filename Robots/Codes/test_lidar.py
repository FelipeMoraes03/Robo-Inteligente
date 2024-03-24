import os
import ydlidar
import time
import csv
import math
import numpy as np

#                 [-90         , -60         , -30         , 0, 30       , 60       , 90       ]
INTEREST_POINTS = [-(math.pi/2), -(math.pi/3), -(math.pi/6), 0, math.pi/6, math.pi/3, math.pi/2]
REF_ANGLE = -2.1
LIDAR_INTERVAL = [-90+(REF_ANGLE*(180/math.pi)), 90+(REF_ANGLE*(180/math.pi))]
THRESHOLD = 0.03

def normalize_angle(angle):
    angle = angle + REF_ANGLE
    normalized_angle = (angle + np.pi) % (2 * np.pi) - np.pi
    return normalized_angle

def main():
    try:
        ydlidar.os_init()
        ports = ydlidar.lidarPortList()
        port = "/dev/ydlidar"
        for key, value in ports.items():
            port = value
            print(port)

        laser = ydlidar.CYdLidar()
        laser.setlidaropt(ydlidar.LidarPropSerialPort, port)
        laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, 115200)
        laser.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TRIANGLE)
        laser.setlidaropt(ydlidar.LidarPropDeviceType, ydlidar.YDLIDAR_TYPE_SERIAL)
        laser.setlidaropt(ydlidar.LidarPropScanFrequency, 10.0)
        laser.setlidaropt(ydlidar.LidarPropSampleRate, 3)
        laser.setlidaropt(ydlidar.LidarPropSingleChannel, True)
        # laser.setlidaropt(ydlidar.LidarPropMaxAngle, 360.0)
        # laser.setlidaropt(ydlidar.LidarPropMinAngle, -360.0)
        laser.setlidaropt(ydlidar.LidarPropMaxRange, 0.5)
        laser.setlidaropt(ydlidar.LidarPropMinRange, 0.12)
        laser.setlidaropt(ydlidar.LidarPropIntenstiy, False)

        ret = laser.initialize()
        if ret:
            ret = laser.turnOn()
            scan = ydlidar.LaserScan()

            dist_dic = {
                '-90': [],
                '-60': [],
                '-30': [],
                '0': [],
                '30': [],
                '60': [],
                '90': []
            }

            ang_dic = {
                '-90': [],
                '-60': [],
                '-30': [],
                '0': [],
                '30': [],
                '60': [],
                '90': []
            }

            while ret and ydlidar.os_isOk():
                r = laser.doProcessSimple(scan)
                if r:
                    print(len(scan.points))
                    for n in range(len(scan.points)):
                        ag = normalize_angle(scan.points[n].angle)

                        dt = scan.points[n].range

                        if (ag > INTEREST_POINTS[0]-THRESHOLD and ag < INTEREST_POINTS[0]+THRESHOLD):
                            dist_dic["-90"].append(dt)
                            ang_dic["-90"].append(ag)
                        elif (ag > INTEREST_POINTS[1]-THRESHOLD and ag < INTEREST_POINTS[1]+THRESHOLD):
                            dist_dic["-60"].append(dt)
                            ang_dic["-60"].append(ag)
                        elif (ag > INTEREST_POINTS[2]-THRESHOLD and ag < INTEREST_POINTS[2]+THRESHOLD):
                            dist_dic["-30"].append(dt)
                            ang_dic["-30"].append(ag)
                        elif (ag > INTEREST_POINTS[3]-THRESHOLD and ag < INTEREST_POINTS[3]+THRESHOLD):
                            dist_dic["0"].append(dt)
                            ang_dic["0"].append(ag)
                        elif (ag > INTEREST_POINTS[4]-THRESHOLD and ag < INTEREST_POINTS[4]+THRESHOLD):
                            dist_dic["30"].append(dt)
                            ang_dic["30"].append(ag)
                            print(ag)
                        elif (ag > INTEREST_POINTS[5]-THRESHOLD and ag < INTEREST_POINTS[5]+THRESHOLD):
                            dist_dic["60"].append(dt)
                            ang_dic["60"].append(ag)
                        elif (ag > INTEREST_POINTS[6]-THRESHOLD and ag < INTEREST_POINTS[6]+THRESHOLD):
                            dist_dic["90"].append(dt)
                            ang_dic["90"].append(ag)

                    break
                else: print("Failed to get Lidar Data")

            print(f'Angulo Esperado: -90, Angulo Real: {np.mean(ang_dic["-90"]) * (180/math.pi)}, Distancia Media: {np.median(dist_dic["-90"])}, Size: {len(dist_dic["-90"])}')
            print(f'Angulo Esperado: -60, Angulo Real: {np.mean(ang_dic["-60"]) * (180/math.pi)}, Distancia Media: {np.median(dist_dic["-60"])}, Size: {len(dist_dic["-60"])}')
            print(f'Angulo Esperado: -30, Angulo Real: {np.mean(ang_dic["-30"]) * (180/math.pi)}, Distancia Media: {np.median(dist_dic["-30"])}, Size: {len(dist_dic["-30"])}')
            print(f'Angulo Esperado:   0, Angulo Real: { np.mean(ang_dic["0"] ) * (180/math.pi)}, Distancia Media: { np.median(dist_dic["0"] )}, Size: {len(dist_dic["0"])  }')
            print(f'Angulo Esperado:  30, Angulo Real: { np.mean(ang_dic["30"]) * (180/math.pi)}, Distancia Media: { np.median(dist_dic["30"])}, Size: {len(dist_dic["30"]) }')
            print(f'Angulo Esperado:  60, Angulo Real: { np.mean(ang_dic["60"]) * (180/math.pi)}, Distancia Media: { np.median(dist_dic["60"])}, Size: {len(dist_dic["60"]) }')
            print(f'Angulo Esperado:  90, Angulo Real: { np.mean(ang_dic["90"]) * (180/math.pi)}, Distancia Media: { np.median(dist_dic["90"])}, Size: {len(dist_dic["90"]) }')
                
    finally:
        laser.turnOff()
        laser.disconnecting()

if __name__ == "__main__":
    main()