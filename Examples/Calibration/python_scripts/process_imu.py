'''
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
    *
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
    * If not, see <http://www.gnu.org/licenses/>.
'''

import sys
import numpy as np
import matplotlib.pyplot as plt
import csv
import os

class dataset:

    # imu_sync = np.zeros((1,7))
    # acc = np.zeros((1,7))
    # gyro = np.zeros((1,7))

    def __init__(self, dirName):
        self.name = dirName
        self.acc = np.zeros((1, 4))
        self.gyro = np.zeros((1, 4))
        self.timesCam = np.zeros((1, 1))

        times_path = os.path.join(dirName, "cam0", "times")
        acc_path = os.path.join(dirName, "IMU", "acc")
        gyro_path = os.path.join(dirName, "IMU", "gyro")

        is_csv = False
        is_txt = False

        if os.path.exists(times_path + ".csv") and os.path.exists(acc_path + ".csv") and os.path.exists(gyro_path + ".csv"):
            is_csv  = True
        
        if os.path.exists(times_path + ".txt") and os.path.exists(acc_path + ".txt") and os.path.exists(gyro_path + ".txt"):
            is_txt  = True

        if is_csv:
            print("Loading csv data...")
            self.load_csv_data(times_path + ".csv", acc_path + ".csv", gyro_path + ".csv")
        elif is_txt:
            print("Loading txt data...")
            self.load_txt_data(times_path + ".txt", acc_path + ".txt", gyro_path + ".txt")
        else:
            print("The required data files to process IMU do not exist.")
            exit(1)

        print("Finished")

    def load_csv_data(self, times_file, acc_file, gyro_file):
        with open(times_file, "r") as file:
            i = 0
            n = 0
            reader = csv.reader(file)
            for row in reader:
                if row[0][0] == '#': continue
                if i % 2 == 0:
                    self.timesCam[n] = row
                    n += 1
                    self.timesCam = np.pad(self.timesCam, ((0, 1), (0, 0)), mode='constant', constant_values=0)
                i += 1
                print(f"Processing CSV: {i}/{n}")

        with open(acc_file, "r") as file:
            i = 0
            reader = csv.reader(file)
            for currentline in reader:
                if currentline[0][0] == '#': continue
                if len(currentline) >= 4:
                    for j in range(0, 4):
                        self.acc[i][j] = currentline[j]
                    self.acc = np.pad(self.acc, ((0, 1), (0, 0)), mode='constant', constant_values=0)
                    i += 1 

        with open(gyro_file, "r") as file:
            i = 0
            reader = csv.reader(file)
            for currentline in reader:
                if currentline[0][0] == '#': continue
                if len(currentline) >= 4:
                    for j in range(0, 4):
                        self.gyro[i][j] = currentline[j]
                    self.gyro = np.pad(self.gyro, ((0, 1), (0, 0)), mode='constant', constant_values=0)
                    i += 1 

        self.timesCam = np.delete(self.timesCam, self.timesCam.shape[0] - 1, axis=0)
        self.acc = np.delete(self.acc, self.acc.shape[0] - 1, axis=0)
        self.gyro = np.delete(self.gyro, self.gyro.shape[0] - 1, axis=0)


    def load_txt_data(self, times_file, acc_file, gyro_file):
        with open(times_file, "r") as file:
            i = 0
            next = 0
            for line in file:
                currentline = line.split(",")
                if currentline[0][0] == '#': continue
                if i%2 == 0:
                    self.timesCam[next] = currentline
                    next = next + 1
                    self.timesCam = np.pad(self.timesCam, ((0, 1), (0, 0)), mode='constant', constant_values=0)
                i = i + 1
                print(i, "/", next)

        with open(acc_file, "r") as file:
            i = 0
            for line in file:
                currentline = line.split(",")
                if currentline[0][0] == '#': continue
                for j in range(0, 4):
                    self.acc[i][j] = currentline[j]
                self.acc = np.pad(self.acc, ((0, 1), (0, 0)), mode='constant', constant_values=0)
                i = i + 1

        with open(gyro_file, "r") as file:
            i = 0
            for line in file:
                currentline = line.split(",")
                if currentline[0][0] == '#': continue
                for j in range(0, 4):
                    self.gyro[i][j] = currentline[j]
                self.gyro = np.pad(self.gyro, ((0, 1), (0, 0)), mode='constant', constant_values=0)
                i = i + 1

        self.timesCam = np.delete(self.timesCam, self.timesCam.shape[0] - 1, axis=0)
        self.acc = np.delete(self.acc, self.acc.shape[0] - 1, axis=0)
        self.gyro = np.delete(self.gyro, self.gyro.shape[0] - 1, axis=0)

    def interpolate(self):
        self.imuSync = np.zeros((self.gyro.shape[0], 7))
        print("shape = ", self.imuSync.shape)

        totAcc = self.acc.shape[0]
        totGyro = self.gyro.shape[0]

        idxAcc = 0
        idxGyro = 0
        print(self.acc[idxAcc][0])
        print(self.gyro[idxGyro][0])
        while (self.acc[idxAcc][0] > self.gyro[idxGyro][0]):
            idxGyro = idxGyro + 1

        idxSync = 0
        while (idxAcc + 1 < totAcc and idxGyro < totGyro):
            # variables for interpolation
            deltaTimeAcc = self.acc[idxAcc + 1, 0] - self.acc[idxAcc, 0]
            deltaAcc = self.acc[idxAcc + 1, 1:4] - self.acc[idxAcc, 1:4]
            while (idxGyro < totGyro and self.acc[idxAcc + 1, 0] >= self.gyro[idxGyro, 0]):
                self.imuSync[idxSync, 0] = self.gyro[idxGyro, 0]
                # Interpolate accelerometer
                self.imuSync[idxSync, 4:7] = self.acc[idxAcc, 1:4] + (
                            self.gyro[idxGyro, 0] - self.acc[idxAcc, 0]) * deltaAcc / deltaTimeAcc

                # Load gyroscope
                self.imuSync[idxSync, 1:4] = self.gyro[idxGyro, 1:4]

                idxGyro = idxGyro + 1
                idxSync = idxSync + 1

            idxAcc = idxAcc + 1

        self.imuSync = np.delete(self.imuSync, range(idxSync, totGyro), axis=0)

    def plotGyro(self):
        for i in range(1, 4):
            plt.plot(self.imuSync[:, 0], self.imuSync[:, i], label=str("acc ") + str(i))
        plt.xlabel("time (s)")
        plt.ylabel("ang. vel. (rad/s)")
        plt.title("Gyroscope")
        plt.legend()
        plt.show()

    def plotAcc(self):
        for i in range(4, 7):
            plt.plot(self.imuSync[:, 0], self.imuSync[:, i], label=str("acc ") + str(i))
        plt.xlabel("time (s)")
        plt.ylabel("acc (m/s^2)")
        plt.title("Accelerometer")
        plt.legend()
        plt.show()

    def saveSynchronized(self):
        imuName = self.name + "/imu0.csv"
        imuFile = open(imuName, "w")
        imuFile.write("#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]\n")
        for row in self.imuSync:
            i = 0
            for num in row:
                if i == 0:
                    imuFile.write(str((int)(1e9 * num)))
                    i = 1
                else:
                    imuFile.write("," + str(num))

            imuFile.write("\n")

    def saveCorrectTimes(self):
        timesName = self.name + "/cam0/corrTimes.txt"
        timesFile = open(timesName, "w")
        print("self.timesCam shape ", self.timesCam.shape)
        for row in self.timesCam:
            i = 0
            for num in row:
                timesFile.write(str((int) (num)))
                timesFile.write("\n")


if __name__ == '__main__':
    if len(sys.argv)!=2 and len(sys.argv)!=3:
        print('Number of arguments != 2 and 3')
        sys.exit()

    dirName = sys.argv[1]
    print('Processing :', dirName)

    myDataset = dataset(dirName)
    myDataset.interpolate()
    myDataset.plotAcc()
    myDataset.saveSynchronized()

    if len(sys.argv)==3:
        myDataset.saveCorrectTimes()
