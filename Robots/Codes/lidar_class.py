import os
import ydlidar
import time
import csv
import math
import numpy as np

ANGLE_POINTS =    ["-90"       , "-60"       , "-30"       , "0", "30"     , "60"     , "90"     ]
INTEREST_POINTS = [-(math.pi/2), -(math.pi/3), -(math.pi/6), 0  , math.pi/6, math.pi/3, math.pi/2]
REF_ANGLE = +2.1
LIDAR_INTERVAL = [-90+(REF_ANGLE*(180/math.pi)), 90+(REF_ANGLE*(180/math.pi))]
THRESHOLD = 0.05


class LidarLoop:
    def __init__(self) -> None:
        print("\nCreating Lidar object!!!\n")
        try:
            ydlidar.os_init()
            ports = ydlidar.lidarPortList()
            port = "/dev/ydlidar"
            for key, value in ports.items():
                port = value
                print(port)

            self.laser = ydlidar.CYdLidar()
            self.laser.setlidaropt(ydlidar.LidarPropSerialPort, port)
            self.laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, 115200)
            self.laser.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TRIANGLE)
            self.laser.setlidaropt(ydlidar.LidarPropDeviceType, ydlidar.YDLIDAR_TYPE_SERIAL)
            self.laser.setlidaropt(ydlidar.LidarPropScanFrequency, 10.0)
            self.laser.setlidaropt(ydlidar.LidarPropSampleRate, 3)
            self.laser.setlidaropt(ydlidar.LidarPropSingleChannel, True)
            # self.laser.setlidaropt(ydlidar.LidarPropMaxAngle, 360.0)
            # self.laser.setlidaropt(ydlidar.LidarPropMinAngle, -360.0)
            self.laser.setlidaropt(ydlidar.LidarPropMaxRange, 0.5)
            self.laser.setlidaropt(ydlidar.LidarPropMinRange, 0.12)
            self.laser.setlidaropt(ydlidar.LidarPropIntenstiy, False)

            self.ret = self.laser.initialize()

        except:
            self.laser.turnOff()
            self.laser.disconnecting()

    def __del__(self):
        print("\nDeleting Lidar object!!!\n")
        self.laser.turnOff()
        self.laser.disconnecting()

    def normalize_angle(self, angle) -> float:
        angle = angle + REF_ANGLE
        normalized_angle = (angle + np.pi) % (2 * np.pi) - np.pi
        return normalized_angle

    def run_loop(self) -> None:
        try:
            if self.ret:
                self.ret = self.laser.turnOn()
                scan = ydlidar.LaserScan()

                print("")
                while self.ret and ydlidar.os_isOk():
                    r = self.laser.doProcessSimple(scan)

                    dist_dic = {
                        '-90': [],
                        '-60': [],
                        '-30': [],
                        '0': [],
                        '30': [],
                        '60': [],
                        '90': []
                    }

                    if r:
                        # print(len(scan.points))
                        for n in range(len(scan.points)):
                            ag = self.normalize_angle(scan.points[n].angle)
                            dt = scan.points[n].range

                            if (ag > INTEREST_POINTS[0]-THRESHOLD and ag < INTEREST_POINTS[0]+THRESHOLD):
                                dist_dic["-90"].append(dt)
                            elif (ag > INTEREST_POINTS[1]-THRESHOLD and ag < INTEREST_POINTS[1]+THRESHOLD):
                                dist_dic["-60"].append(dt)
                            elif (ag > INTEREST_POINTS[2]-THRESHOLD and ag < INTEREST_POINTS[2]+THRESHOLD):
                                dist_dic["-30"].append(dt)
                            elif (ag > INTEREST_POINTS[3]-THRESHOLD and ag < INTEREST_POINTS[3]+THRESHOLD):
                                dist_dic["0"].append(dt)
                            elif (ag > INTEREST_POINTS[4]-THRESHOLD and ag < INTEREST_POINTS[4]+THRESHOLD):
                                dist_dic["30"].append(dt)
                            elif (ag > INTEREST_POINTS[5]-THRESHOLD and ag < INTEREST_POINTS[5]+THRESHOLD):
                                dist_dic["60"].append(dt)
                            elif (ag > INTEREST_POINTS[6]-THRESHOLD and ag < INTEREST_POINTS[6]+THRESHOLD):
                                dist_dic["90"].append(dt) 
                    else: print("Failed to get Lidar Data")

                    for elmt in ANGLE_POINTS:
                        if (len(dist_dic[elmt]) > 0):
                            print(f'{elmt} -> Distancia Media: {np.median(dist_dic[elmt])}, Size: {len(dist_dic[elmt])}')
                        else:
                            print(f'{elmt} -> Distancia Media: None, Size: None')
                    print("")

        except(KeyboardInterrupt):
            pass

    def get_data(self):
        if self.ret:
            self.ret = self.laser.turnOn()
            scan = ydlidar.LaserScan()

            print("")
            r = self.laser.doProcessSimple(scan)

            dist_dic = {
                '-90': [],
                '-60': [],
                '-30': [],
                '0': [],
                '30': [],
                '60': [],
                '90': []
            }

            if r:
                # print(len(scan.points))
                for n in range(len(scan.points)):
                    ag = self.normalize_angle(scan.points[n].angle)
                    dt = scan.points[n].range
                    if (dt == 0):
                        dt = np.inf

                    if (ag > INTEREST_POINTS[0]-THRESHOLD and ag < INTEREST_POINTS[0]+THRESHOLD):
                        dist_dic["-90"].append(dt)
                    elif (ag > INTEREST_POINTS[1]-THRESHOLD and ag < INTEREST_POINTS[1]+THRESHOLD):
                        dist_dic["-60"].append(dt)
                    elif (ag > INTEREST_POINTS[2]-THRESHOLD and ag < INTEREST_POINTS[2]+THRESHOLD):
                        dist_dic["-30"].append(dt)
                    elif (ag > INTEREST_POINTS[3]-THRESHOLD and ag < INTEREST_POINTS[3]+THRESHOLD):
                        dist_dic["0"].append(dt)
                    elif (ag > INTEREST_POINTS[4]-THRESHOLD and ag < INTEREST_POINTS[4]+THRESHOLD):
                        dist_dic["30"].append(dt)
                    elif (ag > INTEREST_POINTS[5]-THRESHOLD and ag < INTEREST_POINTS[5]+THRESHOLD):
                        dist_dic["60"].append(dt)
                    elif (ag > INTEREST_POINTS[6]-THRESHOLD and ag < INTEREST_POINTS[6]+THRESHOLD):
                        dist_dic["90"].append(dt) 
            else: print("Failed to get Lidar Data")

            # for elmt in ANGLE_POINTS:
            #     if (len(dist_dic[elmt]) > 0):
            #         print(f'{elmt} -> Distancia Media: {np.median(dist_dic[elmt])}, Size: {len(dist_dic[elmt])}')
            #     else:
            #         print(f'{elmt} -> Distancia Media: None, Size: None')
            # print("")

            return [np.median(dist_dic["-90"]),
             np.median(dist_dic["-60"]),
             np.median(dist_dic["-30"]),
             np.median(dist_dic["0"]),
             np.median(dist_dic["30"]),
             np.median(dist_dic["60"]),
             np.median(dist_dic["90"])
            ]

    def get_diff(self):
        # DIREITA - ESQUERDA!!!
        dist_dic = self.get_data()
        sum_left = dist_dic[1] #+ dist_dic[0] #+ dist_dic[2]
        sum_right = dist_dic[5] #+ dist_dic[6] #+ dist_dic[4]

        rt = dist_dic[3], 0 if (sum_right == np.inf and sum_left == np.inf) else sum_right - sum_left

        # rt = [dist_dic[3],                                                                          # 0
        #       0 if (dist_dic[6] == np.inf and dist_dic[0] == np.inf) else dist_dic[6]-dist_dic[0],  # 90
        #       0 if (dist_dic[5] == np.inf and dist_dic[1] == np.inf) else dist_dic[5]-dist_dic[1],  # 60
        #       0 if (dist_dic[4] == np.inf and dist_dic[2] == np.inf) else dist_dic[4]-dist_dic[2]]  # 30]

        return rt

    def is_running(self):
        x = ydlidar.os_isOk()
        return x