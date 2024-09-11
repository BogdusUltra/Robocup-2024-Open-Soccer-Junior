import cv2
from math import *
import threading
import numpy as np
import time

class LidarData:
    def __init__(self, port):

        self.port = port
        self.data_len = []
        self.speed = []
        self.start_angle = []
        self.end_angle = []
        self.step = 0
        self.n_points = []
        self.points = []
        self.maxline = 0

        self.maxlinecoord = []
        for i in range(4):
            self.maxlinecoord.append([0,0])

        self.maxlinecoord_old = []
        for i in range(4):
            self.maxlinecoord_old.append([0,0])

        self.angleCoord = []
        for i in range(4):
            self.angleCoord.append([0,0])

        self.angleCoord_old = []
        for i in range(4):
            self.angleCoord_old.append([0,0])

        self.dists = []
        for i in range(360):
            self.dists.append(0)

        self.dists_old = []
        for i in range(360):
            self.dists_old.append(0)

        self.all_coord = []
        for i in range(360):
            self.all_coord.append([0, 0])

        self.drawcoord = []
        for i in range(360):
            self.drawcoord.append([0, 0])

        self.drawcoord_old = []
        for i in range(360):
           self.drawcoord_old.append([0, 0])

        self.points_old = np.array([[0,0], [1,0]])

        self.LidarRead = threading.Thread(target=self.__readData)
        self.LidarRead.daemon = True
        self.LidarRead.start()

    def __readData(self):
        while True:
            if int.from_bytes(self.port.read(), 'big') == 0x54:
                a = self.port.read(46 + 47 * 9)
                self.data_len = []
                self.speed = []
                self.start_angle = []
                self.points = []
                self.end_angle = []
                for i in range(10):
                    self.data_len.append(int.from_bytes(a[0 + i * 47:1 + i * 47], 'little'))
                    self.speed.append(int.from_bytes(a[1 + i * 47:3 + i * 47], 'little'))
                    self.start_angle.append(round(int.from_bytes(a[3 + i * 47:5 + i * 47], 'little') / 100, 2))
                    for j in range(12):
                        point = [0,0]
                        point[0] = int.from_bytes(a[5 + 3 * j + i * 47:7 + 3 * j + i * 47], 'little')
                        point[1] = int.from_bytes(a[7 + 3 * j + i * 47:8 + 3 * j + i * 47], 'little')
                        self.points.append(point)
                    self.end_angle.append(round(int.from_bytes(a[41 + i * 47:43 + i * 47], 'little') / 100, 2))
                    # timestamp = lidar.read(2)
                    # CRC = lidar.read()

                if self.end_angle[9] > self.start_angle[0]:
                    delta_angle = round(self.end_angle[9] - self.start_angle[0], 2)
                else:
                    delta_angle = round(self.end_angle[9] + 360 - self.start_angle[0], 2)
                self.step = round(delta_angle / 120, 2)
                self.n_points = 120

                j=0
                for i in range(self.n_points):
                    angle = self.start_angle[0] + self.step * (i - j)
                    if round(angle) > 359:
                        self.start_angle[0] = angle - 360
                        angle = self.start_angle[0] + self.step
                        j = i
                    self.dists[round(angle)] = self.points[i][0]
                    # if self.dists[round(angle)] < 300:
                    #     print(round(angle))

    # def ignore_angles(self, angles):
    #     for i in angles:
    #         k = i[1] + 1
    #         if k > 359: k = 0
    #         f = i[0] - 1
    #         if f < 0: f = 359
    #         dist_sr = (self.dists[f] + self.dists[k]) // 2
    #         for j in range(abs(i[0]-i[1])+1):
    #             self.dists[j] = dist_sr

    def coordCalc(self, x0=0, y0=0, kf=1):
        for angle in range(360):
            if self.dists[angle] != self.dists_old[angle]:
                self.dists_old[angle] = self.dists[angle]
                l = self.dists[angle] * kf
                coord = [0,0]
                if angle <= 90:
                    x = l * sin(angle * pi / 180)
                    y = l * cos(angle * pi / 180)
                    coord = [round(x), round(y)]
                if 90 < angle <= 180:
                    x = l * cos((angle - 90) * pi / 180)
                    y = l * sin((angle - 90) * pi / 180)
                    coord = [round(x), round(-y)]
                if 180 < angle <= 270:
                    x = l * sin((angle - 180) * pi / 180)
                    y = l * cos((angle - 180) * pi / 180)
                    coord = [round(-x), round(-y)]
                if 270 < angle:
                    x = l * cos((angle - 270) * pi / 180)
                    y = l * sin((angle - 270) * pi / 180)
                    coord = [round(-x), round(y)]
                self.all_coord[angle] = coord

    def simple_draw(self, frame, x0, y0, kf, step = 1):
        cv2.rectangle(frame, (0, 0), (x0*2, y0*2), (0, 0, 0), -1)
        for i in range(0, 360, step):
            self.drawcoord[i][0] = x0 + round(self.all_coord[i][0]*kf)
            self.drawcoord[i][1] = y0 - round(self.all_coord[i][1]*kf)
            cv2.line(frame, (self.drawcoord[i][0], self.drawcoord[i][1]),(x0, y0), (255, 255, 255), 1)
        return frame

    def draw(self, frame, x0=600, y0=450, kf = 1):
        cv2.circle(frame, (x0, y0), 5, (255, 255, 0), -1)
        for i in range(360):
            self.drawcoord[i][0] = x0 + round(self.all_coord[i][0] * kf)
            self.drawcoord[i][1] = y0 - round(self.all_coord[i][1] * kf)

            g = i - 1
            if g < 0: g = 359
            f = i + 1
            if f > 359: f = 0

            if self.drawcoord[i] != self.drawcoord_old[i]:
                cv2.line(frame, (self.drawcoord_old[g][0], self.drawcoord_old[g][1]),
                         (self.drawcoord_old[i][0], self.drawcoord_old[i][1]), (0, 0, 0), 1)
                cv2.line(frame, (self.drawcoord_old[f][0], self.drawcoord_old[f][1]),
                         (self.drawcoord_old[i][0], self.drawcoord_old[i][1]), (0, 0, 0), 1)
                cv2.line(frame, (self.drawcoord_old[g][0], self.drawcoord_old[g][1]),
                         (self.drawcoord[i][0], self.drawcoord[i][1]), (255, 255, 255), 1)
                cv2.line(frame, (self.drawcoord_old[f][0], self.drawcoord_old[f][1]),
                         (self.drawcoord[i][0], self.drawcoord[i][1]), (255, 255, 255), 1)
                self.drawcoord_old[i][0] = x0 + round(self.all_coord[i][0] * kf)
                self.drawcoord_old[i][1] = y0 - round(self.all_coord[i][1] * kf)
        return frame

    # def robot_coord(self, kf=0.04, shape=200):
    #     mask = np.zeros((shape, shape), dtype = "uint8")
    #
    #     coords = []
    #     for i in self.all_coord:
    #         x, y = round(shape//2 + i[0] * kf), round(shape//2 - i[1] * kf)
    #         if x < 0: x=0
    #         if y < 0: y = 0
    #         if x > shape: x = shape
    #         if y > shape: y = shape
    #         coords.append([x, y])
    #     points = np.array(coords)
    #
    #     cv2.fillPoly(mask, pts=[self.points_old], color=(0))
    #     cv2.fillPoly(mask, pts=[points], color=(255))
    #     self.points_old = points
    #
    #     max_ratio = 0.01
    #     ang_rot = 0
    #     x2, y2, w2, h2 = 0, 0, 0, 0
    #     for i in range(0, 90):
    #         rot_mask = self.rotation(mask, i)
    #         contours, _ = cv2.findContours(rot_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    #         area1 = 0
    #         x1, y1, w1, h1 = 0,0,0,0
    #         for contor in contours:
    #             x, y, w, h = cv2.boundingRect(contor)
    #             area = cv2.contourArea(contor)
    #             if area > area1:
    #                 area1 = area
    #                 x1, y1, w1, h1 = x, y, w, h
    #         area2 = w1 * h1
    #         if area2 != 0:
    #             ratio = area1/area2
    #             if ratio > max_ratio:
    #                 x2, y2, w2, h2 = x1, y1, w1, h1
    #                 max_ratio = ratio
    #                 ang_rot = i
    #
    #     if w2 > h2:
    #         ang_rot += 90
    #         Y2 = (shape // 2 - x2)
    #         X1 = (shape // 2 - y2)
    #         Y1 = (x2 + w2 - shape // 2)
    #         X2 = (y2 + h2 - shape // 2)
    #
    #     else:
    #         X1 = (shape // 2 - x2)
    #         Y1 = (shape // 2 - y2)
    #         X2 = (x2 + w2 - shape // 2)
    #         Y2 = (y2 + h2 - shape // 2)
    #
    #     mask = self.rotation(mask, ang_rot)
    #
    #     return ang_rot, X1, Y1, X2, Y2, mask

    def robot_coord(self, kf=0.04, shape=200, visible=False):
        frame1 = np.zeros((shape, shape), dtype="uint8")
        self.coordCalc(0, 0, 1)
        coords = []
        for i in self.all_coord:
            x, y = round(shape // 2 + i[0] * kf), round(shape // 2 - i[1] * kf)
            coords.append([x, y])
        cv2.fillPoly(frame1, pts=[np.array(coords)], color=255)

        contours, _ = cv2.findContours(frame1, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        area1 = 0
        for contor in contours:
            area = cv2.contourArea(contor)
            if area > area1:
                area1 = area

        points = [[0,0],[0,0],[0,0],[0,0]]
        min_x, min_y, max_x, max_y = 9999, 9999, -9999, -9999
        for i in self.all_coord:
            x, y = i[0], i[1]
            if x < min_x:
                min_x = x
                points[0] = [x,y]
            if y < min_y:
                min_y = y
                points[1] = [x,y]
            if x > max_x:
                max_x = x
                points[2] = [x,y]
            if y > max_y:
                max_y = y
                points[3] = [x,y]

        area = (max_x - min_x)*(max_y - min_y)*(kf**2)
        if area!=0 and area1 / area < 0.76:
            pass
        else:
            points[0] = [min_x, max_y]
            points[1] = [min_x, min_y]
            points[2] = [max_x, min_y]
            points[3] = [max_x, max_y]

        w1, h1 = points[3][0] - points[0][0], points[3][1] - points[0][1]
        w2, h2 = points[2][0] - points[1][0], points[2][1] - points[1][1]
        w3, h3 = points[1][0] - points[0][0], points[0][1] - points[1][1]
        w4, h4 = points[2][0] - points[3][0], points[3][1] - points[2][1]

        a = 1.1
        if w2!=0 and abs(w1/w2)>a: w2=w1
        if w1!=0 and abs(w2/w1)>a: w1=w2
        if h2!=0 and abs(h1/h2)>a: h2=h1
        if h1!=0 and abs(h2/h1)>a: h1=h2
        if w4!=0 and abs(w3/w4)>a: w4=w3
        if w3!=0 and abs(w4/w3)>a: w3=w4
        if h4!=0 and abs(h3/h4)>a: h4=h3
        if h3!=0 and abs(h4/h3)>a: h4=h4

        len12 = (w3 ** 2 + h3 ** 2) ** 0.5
        len14 = (w1 ** 2 + h1 ** 2) ** 0.5
        if len14 > len12:
            ang = 90
            center_point1 = [points[1][0] - w3 / 2, points[1][1] + h3 / 2]
            center_point2 = [points[2][0] - w4 / 2, points[2][1] + h4 / 2]
        else:
            ang = 0
            center_point1 = [points[0][0] + w1 / 2, points[0][1] + h1 / 2]
            center_point2 = [points[1][0] + w2 / 2, points[1][1] + h2 / 2]
        w, h = center_point2[0] - center_point1[0], center_point1[1] - center_point2[1]

        if h != 0:
            tg = w / h
            if ang == 90:
                ang += 90 + round(atan(tg) / pi * 180)
            else:
                ang += round(atan(tg) / pi * 180)
        # print(ang)

        new_coord = []
        for angle in range(360):
            new_angle = angle - ang
            if new_angle < 0: new_angle += 360
            l = self.dists[new_angle]
            coord = [0, 0]
            if angle <= 90:
                x = l * sin(angle * pi / 180)
                y = l * cos(angle * pi / 180)
                coord = [round(x), round(y)]
            if 90 < angle <= 180:
                x = l * cos((angle - 90) * pi / 180)
                y = l * sin((angle - 90) * pi / 180)
                coord = [round(x), round(-y)]
            if 180 < angle <= 270:
                x = l * sin((angle - 180) * pi / 180)
                y = l * cos((angle - 180) * pi / 180)
                coord = [round(-x), round(-y)]
            if 270 < angle:
                x = l * cos((angle - 270) * pi / 180)
                y = l * sin((angle - 270) * pi / 180)
                coord = [round(-x), round(y)]
            new_coord.append(coord)

        min_x, min_y, max_x, max_y = 9999, 9999, -9999, -9999
        for i in new_coord:
            x, y = i[0], i[1]
            if x < min_x:
                min_x = x
            if y < min_y:
                min_y = y
            if x > max_x:
                max_x = x
            if y > max_y:
                max_y = y
        X1, Y1, X2, Y2 = -min_x, max_y, max_x, -min_y

        if visible == True:
            frame2 = np.zeros((shape, shape), dtype="uint8")
            frame3 = np.zeros((shape, shape), dtype="uint8")

            coords = []
            for i in points:
                x, y = round(shape // 2 + i[0] * kf), round(shape // 2 - i[1] * kf)
                coords.append([x, y])
            center_point1 = [round(shape // 2 + center_point1[0] * kf), round(shape // 2 - center_point1[1] * kf)]
            center_point2 = [round(shape // 2 + center_point2[0] * kf), round(shape // 2 - center_point2[1] * kf)]
            cv2.polylines(frame2, pts=[np.array(coords)], color=255, thickness=2, isClosed=True)
            cv2.line(frame2, (center_point1[0], center_point1[1]), (center_point2[0], center_point2[1]), 255, 2)

            coords = []
            for i in new_coord:
                x, y = round(shape // 2 + i[0] * kf), round(shape // 2 - i[1] * kf)
                coords.append([x, y])
            cv2.fillPoly(frame3, pts=[np.array(coords)], color=255)

            return ang, X1, Y1, X2, Y2, frame1, frame2, frame3
        return ang, X1, Y1, X2, Y2

    # def rotation(self, frame, angle):
    #     (h,w) = frame.shape[:2]
    #     center = (w//2, h//2)
    #     rot_matrix = cv2.getRotationMatrix2D(center, angle, 1)
    #     return cv2.warpAffine(frame, rot_matrix, (w,h))

