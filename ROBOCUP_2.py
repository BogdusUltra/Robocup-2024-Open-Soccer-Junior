import cv2
import time
import numpy as np
import RobotAPI as rapi
import serial
import LidarLD06
from math import *
import threading
from numba import njit

robot = rapi.RobotAPI(flag_serial=False)
shapeX, shapeY = 480, 480
robot.set_camera(120, shapeX, shapeY, 0)
frame = robot.get_frame(wait_new_frame=0)
display_frame = np.zeros((shapeY + 90, shapeX + 260, 3), dtype = "uint8")

mask1 = np.zeros((shapeX, shapeY), dtype = "uint8")
insideCircle = 65
outsideCircle = 265
cv2.circle(mask1, (235, 250), outsideCircle, 255, -1)
cv2.circle(mask1, (235, 255), insideCircle, 0, -1)
cv2.fillPoly(mask1, pts=[np.array([[210, 300], [250, 345], [260, 345], [270,300]])], color=0)

lidarPort = serial.Serial(port='/dev/ttyUSB0', baudrate=230400, timeout=1, bytesize=8, parity='N', stopbits=1)
ld = LidarLD06.LidarData(lidarPort)
ld_frame = np.zeros((200,200,3),dtype='uint8')

port = serial.Serial("/dev/serial0", baudrate=115200, stopbits=serial.STOPBITS_ONE)

def joy_controll():
    global sp1, sp2, sp3, sp4, dribler, kicker, zummer, flag_stop, flag_start
    if len(robot.messageFromPC) == 17:
        sp1 += int(robot.messageFromPC[0:3]) - 200
        sp2 += int(robot.messageFromPC[3:6]) - 200
        sp3 += int(robot.messageFromPC[6:9]) - 200
        sp4 += int(robot.messageFromPC[9:12]) - 200
        dribler += int(robot.messageFromPC[12:13])*10
        if kicker == 0:
            kicker = int(robot.messageFromPC[13:14])
        if zummer == 0:
            zummer = int(robot.messageFromPC[14:15])
        flag_stop = int(robot.messageFromPC[15:16])
        flag_start = int(robot.messageFromPC[16:17])

def pd(angle, target):
    if target > angle:
        e = target - angle
    else:
        e = 360 + target - angle
    if e > 180:
        e = -(360 - e)
    return e

@njit()
def for_ball(hsv):
    H = hsv[:, :, 0]
    S = hsv[:, :, 1]
    V = hsv[:, :, 2]
    gray = np.zeros((shapeY, shapeX), dtype = "uint8")
    for y in range(shapeY):
        for x in range(shapeX):
            kf_len = sqrt((x - shapeX//2) ** 2 + (shapeY//2 - y) ** 2)/ sqrt((shapeX//2) ** 2 + (shapeY//2) ** 2)
            pix = round((abs(H[y][x] - 180) / 180)**9 * (S[y][x] / 255) * (V[y][x]/255)**2 * kf_len * 255 * 40)
            if pix> 255: pix =255
            gray[y][x] = pix
    return gray

class validFlag():
    def __init__(self, flag, validTimeTrue, validTimeFalse):
        self.flag = flag
        self.flagTime = False
        self.validFlag = False
        self.timer = time.time()
        self.validTimeTrue = validTimeTrue
        self.validTimeFalse = validTimeFalse

        self.thread = threading.Thread(target=self.wile)
        self.thread.start()

    def wile(self):
        while 1:
            time.sleep(0.001)
            if self.validFlag:
                if self.flag:
                    self.flagTime = False
                else:
                    if self.flagTime:
                        if self.timer + self.validTimeFalse < time.time():
                            self.validFlag = False
                            self.flagTime = False
                    else:
                        self.timer = time.time()
                        self.flagTime = True
            else:
                if self.flag:
                    if self.flagTime:
                        if self.timer + self.validTimeTrue < time.time():
                            self.validFlag = True
                            self.flagTime = False
                    else:
                        self.timer = time.time()
                        self.flagTime = True
                else:
                    self.flagTime = False

    def getValid(self, flag):
        self.flag = flag
        return self.validFlag

    def give(self, flag):
        self.flag = flag
        self.validFlag = flag
        self.flagTime = False
def detectHomeGoal(hsv, low1, up1, low2, up2, side=0):
    if side ==0:
        mask1 = cv2.inRange(hsv[shapeY//2:shapeY,0:shapeX], low1, up1)
        contours1, _ = cv2.findContours(mask1, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        max_area1 = 0
        for contor in contours1:
            area = cv2.contourArea(contor)
            if area > max_area1:
                max_area1 = area

        mask2 = cv2.inRange(hsv[shapeY//2:shapeY,0:shapeX], low2, up2)
        contours2, _ = cv2.findContours(mask2, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        max_area2 = 0
        for contor in contours2:
            area = cv2.contourArea(contor)
            if area > max_area2:
                max_area2 = area

        low3, up3, low4, up4, col = 0, 0, 0,0,''
        if max_area1 > max_area2:
            low3, up3, low4, up4, col = low1, up1, low2, up2, "blu"
        elif max_area2 > max_area1:
            low3, up3, low4, up4, col = low2, up2, low1, up1, "yel"

        return low3, up3, low4, up4, col

    else:
        mask1 = cv2.inRange(hsv[0:shapeY//2, 0:shapeX], low1, up1)
        contours1, _ = cv2.findContours(mask1, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        max_area1 = 0
        for contor in contours1:
            area = cv2.contourArea(contor)
            if area > max_area1:
                max_area1 = area

        mask2 = cv2.inRange(hsv[0:shapeY // 2, 0:shapeX], low2, up2)
        contours2, _ = cv2.findContours(mask2, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        max_area2 = 0
        for contor in contours2:
            area = cv2.contourArea(contor)
            if area > max_area2:
                max_area2 = area

        low3, up3, low4, up4, col = 0, 0, 0, 0, ''
        if max_area1 > max_area2:
            low3, up3, low4, up4, col = low1, up1, low2, up2, "blu"
        elif max_area2 > max_area1:
            low3, up3, low4, up4, col = low2, up2, low1, up1, "yel"

        return low3, up3, low4, up4, col
low_yellow = np.array([30, 95, 100])
up_yellow = np.array([60, 255, 255])
low_blue = np.array([90, 100, 100])
up_blue = np.array([100, 255, 255])
angel_HomeGoal, angel_OppGoal = 0, 0
dist_HomeGoal, dist_OppGoal = 0, 0
area_HomeGoal, area_OppGoal = 0, 0
flag_HomeGoal, flag_OppGoal = False, False
homeGoal = validFlag(flag_HomeGoal, 0.1, 0.3)
oppGoal = validFlag(flag_OppGoal, 0.1, 0.1)
side = 'None'
count_side = 0
valid_flag_HomeGoal = False
valid_flag_OppGoal = False
mask_goal = cv2.inRange(frame, low_yellow, up_yellow)
def GOAL(hsv, low, up, type=0):
    global frame, count_side, side
    angle = -1
    area = 0
    dist = ((shapeX//2)**2 + (shapeY//2)**2)**0.5
    mask = cv2.inRange(hsv, low, up)
    cv2.blur(mask, (5, 5))
    cv2.circle(mask, (shapeX//2, shapeY//2), 100, 0, -1)
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    flag = False
    x_min, y_min, x_max, y_max = shapeX+1, shapeY+1, -1, -1
    x1,x2,y1,y2 = 0,0,0,0

    max_area = 0
    max_contor = [0, 0, 0, 0]
    if type == 0:
        porog = 500
    else:
        porog = 50
    for contor in contours:
        x, y, w, h = cv2.boundingRect(contor)
        area  = cv2.contourArea(contor)
        if w * h > 100 and area > porog:
            if area > max_area:
                max_area = area
                max_contor = [x, y, w, h]
            if x < x_min: x_min = x
            if y < y_min: y_min = y
            if x + w > x_max: x_max = x + w
            if y + h > y_max: y_max = y + h
            flag = True

    if flag:
        if type == 0:
            x1, x2 = x_min, x_max
            y1, y2 = y_min, y_max
            area = (x_max - x_min)*(y_max - y_min)


        elif type == 1:
            w = x_max - x_min
            mask1 = mask[y_min:y_max, x_min:(x_min + w // 2)]
            mask2 = mask[y_min:y_max, (x_min + w // 2):x_max]

            contours1, _ = cv2.findContours(mask1, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
            sum_area1 = 0
            for contor in contours1:
                area = cv2.contourArea(contor)
                sum_area1 += area

            contours2, _ = cv2.findContours(mask2, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
            sum_area2 = 0
            for contor in contours2:
                area = cv2.contourArea(contor)
                sum_area2 += area

            if side == 'None':
                if sum_area1 > sum_area2:
                    count_side+=1
                else:
                    count_side-=1

                if count_side>5:
                    area = sum_area1
                    side = 'left'
                    count_side = 0
                elif count_side<-5:
                    area = sum_area2
                    side = 'right'
                    count_side = 0

            elif side == 'left':
                x1, x2 = x_min, (x_min+w//2)

            elif side == 'right':
                x1, x2 = (x_min + w // 2), x_max

            y1, y2 = y_min, y_max

        elif type == 2:
            x,y,w,h = max_contor[0], max_contor[1], max_contor[2], max_contor[3]
            x1, y1, x2, y2 = x, y, x+w, y+h

        elif type == 3:
            if turn == 1:
                x1, x2 = x_min, x_min
            else:
                x1, x2 = x_max, x_max
            y1,y2 = y_min, y_max

        if side != 'None' or type == 0 or 2 or 3:
            # if x1 != 0 and x2 != 0:
            #     frame[y1:y2, x1:x2] = cv2.cvtColor(mask[y1:y2, x1:x2], cv2.COLOR_GRAY2BGR)
            x_c = x1 + (x2 - x1) // 2
            y_c = y1 + (y2 - y1) // 2

            if y_c != shapeY//2:
                angle = np.arctan((x_c - shapeX//2) / (shapeY//2 - y_c)) / pi * 180

            if x_c <= shapeX//2 and y_c <= shapeY//2:
                angle = 360 + angle
            elif x_c > shapeX//2 and y_c > shapeY//2:
                angle = 180 + angle
            elif x_c <= shapeX//2 and y_c > shapeY//2:
                angle = 180 + angle

            angle = 360 - angle

            dist = sqrt((shapeX//2 - x_c) ** 2 + (shapeY//2 - y_c) ** 2)

            if draw:
                cv2.putText(frame, "d:" + str(round(dist)), ((x_c + shapeX//2)//2, (y_c + shapeY//2)//2), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                cv2.putText(frame, "a:" + str(round(angle)), ((x_c + shapeX // 2) // 2, (y_c + shapeY // 2 ) // 2 + 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                cv2.line(frame, (x_c + 5, y_c + 5), (x_c - 5, y_c - 5), (0, 0, 255), 2)
                cv2.line(frame, (x_c + 5, y_c - 5), (x_c - 5, y_c + 5), (0, 0, 255), 2)
                cv2.line(frame, (x_c, y_c), (shapeX // 2, shapeY // 2), (0, 0, 255), 1)
                cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 255), 1)

    return round(angle), round(dist), flag, area
angel_ball = 0
dist_ball = 0
flag_ball = False
ball = validFlag(flag_ball, 0.15, 1)
valid_flag_ball = False
flag_dist_ball = False
distBall = validFlag(flag_dist_ball, 0.5, 1)
valid_flag_dist_ball = False
def findBall(hsv):
    global frame

    gray = for_ball(hsv)
    mask = cv2.threshold(gray, 40, 255, cv2.THRESH_BINARY)[1]
    X, Y, W, H = 0, 0, 0, 0
    angle = 361
    min_dist = 401
    flag = False

    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    for contor in contours:
        x, y, w, h = cv2.boundingRect(contor)
        area = cv2.contourArea(contor)
        fill = area / (w * h)
        cX, cY = x + w // 2, y + h // 2
        dist = sqrt((shapeX//2 - cX) ** 2 + (shapeY//2 - cY) ** 2)
        if 1000 > area > 20 and (1 / 2) < w / h < 2 and fill > 0.4 and dist < min_dist:
            flag = True
            min_dist = dist
            centrX, centrY = cX, cY

    if flag:
        if centrY != 240:
            angle = np.arctan((centrX - shapeX//2) / (shapeY//2 - centrY)) / pi * 180

        if centrX <= shapeX//2 and centrY <= shapeY//2:
            angle = 360 + angle
        elif centrX > shapeX//2 and centrY > shapeY//2:
            angle = 180 + angle
        elif centrX <= shapeX//2 and centrY > shapeY//2:
            angle = 180 + angle

        angle = 360 - angle

        if draw:
            cv2.rectangle(frame, (X, Y), (X + W, Y + H), (255, 0, 255), 2)
            cv2.line(frame, (shapeX//2, shapeY//2), (centrX, centrY), (255, 0, 255), 1)
            cv2.putText(frame, "d:" + str(round(min_dist)), ((centrX + shapeX // 2) // 2, (centrY + shapeY // 2) // 2), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255), 2)
            cv2.putText(frame, "a:" + str(round(angle)), ((centrX + shapeX // 2) // 2, (centrY + shapeY // 2) // 2 + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255), 2)

    # frame = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    return round(angle), round(min_dist), flag
ANG, X1, Y1, X2,Y2 = 0,0,0,0,0
ANGEL_OPPGOAL = 0
t_coord = 0
fr1, fr2, fr3 = np.zeros((100, 100), dtype = "uint8"), np.zeros((100, 100), dtype = "uint8"), np.zeros((100, 100), dtype = "uint8")
flag_coord = False
def robot_coord(hsv):
    global ANG,ANGEL_OPPGOAL, X1, Y1, X2, Y2, t_coord, fr1, fr2, fr3, flag_coord, frame
    ANG, x_1, y_1, x_2, y_2, fr1, fr2, fr3 = ld.robot_coord(kf=0.02, shape=100, visible=True)
    hsv1 = hsv.copy()
    hsv2 = hsv.copy()
    x0, y0 = 0,0
    x1, y1 = round(outsideCircle * sin((290 + ANG) / 180 * pi)), round(outsideCircle * cos((290 + ANG) / 180 * pi))
    x2, y2 = round(outsideCircle * sin((70 + ANG) / 180 * pi)), round(outsideCircle * cos((70 + ANG) / 180 * pi))
    x3, y3 = x1 + round(200 * sin(ANG / 180 * pi)), y1 + round(200 * cos(ANG / 180 * pi))
    x4, y4 = x2 + round(200 * sin(ANG / 180 * pi)), y2 + round(200 * cos(ANG / 180 * pi))
    points1 = np.array([[shapeX//2 + x1, shapeY//2 - y1], [shapeX//2 + x0, shapeY//2 - y0], [shapeX//2 + x2, shapeY//2 - y2], [shapeX//2 + x4, shapeY//2 - y4], [shapeX//2 + x3, shapeY//2 - y3]])

    x5, y5 = round(outsideCircle * sin((250 + ANG) / 180 * pi)), round(outsideCircle * cos((250 + ANG) / 180 * pi))
    x6, y6 = round(outsideCircle * sin((110 + ANG) / 180 * pi)), round(outsideCircle * cos((110 + ANG) / 180 * pi))
    x7, y7 = x5 - round(200 * sin(ANG / 180 * pi)), y5 - round(200 * cos(ANG / 180 * pi))
    x8, y8 = x6 - round(200 * sin(ANG / 180 * pi)), y6 - round(200 * cos(ANG / 180 * pi))
    points2 = np.array([[shapeX//2 + x5, shapeY//2 - y5], [shapeX//2 + x0, shapeY//2 - y0], [shapeX//2 + x6, shapeY//2 - y6], [shapeX//2 + x8, shapeY//2 - y8], [shapeX//2 + x7, shapeY//2 - y7]])

    cv2.fillPoly(hsv1, pts=[points1], color=(0, 0, 0))
    cv2.fillPoly(hsv2, pts=[points2], color=(0, 0, 0))

    sum_area1 = 0
    sum_area2 = 0

    mask = cv2.inRange(hsv1, low1, up1)
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    for contor in contours:
        area = cv2.contourArea(contor)
        if area > 50:
            sum_area1 += area
        # x,y,w,h = cv2.boundingRect(contor)
        # cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 255), 1)
    mask = cv2.inRange(hsv2, low2, up2)
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    for contor in contours:
        area = cv2.contourArea(contor)
        if area > 50:
            sum_area1 += area
        # x, y, w, h = cv2.boundingRect(contor)
        # cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 1)

    mask = cv2.inRange(hsv1, low2, up2)
    contours2, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    for contor in contours2:
        area = cv2.contourArea(contor)
        if area > 50:
            sum_area2 += area
        # x, y, w, h = cv2.boundingRect(contor)
        # cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 1)
    mask = cv2.inRange(hsv2, low1, up1)
    contours2, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    for contor in contours2:
        area = cv2.contourArea(contor)
        if area > 50:
            sum_area2 += area
        # x, y, w, h = cv2.boundingRect(contor)
        # cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 255), 1)

    # print(sum_area1, sum_area2)

    if sum_area1 < 1500 and sum_area2 < 1500:
        flag_coord = False
    else:
        flag_coord = True
        if sum_area1 > sum_area2:
            X1 = x_1
            Y1 = y_1
            X2 = x_2
            Y2 = y_2
        else:
            ANG += 180
            X1 = x_2
            Y1 = y_2
            X2 = x_1
            Y2 = y_1
    x_c = (X1 + X2) / 2
    x_d = x_c - X1
    if Y1 != 0:
        tg = x_d / Y1
    else: tg = 1
    a = np.arctan(tg)
    ANGEL_OPPGOAL = ANG - round(a / 3.14 * 180)
    if ANGEL_OPPGOAL > 360:
        ANGEL_OPPGOAL -= 360
    if ANGEL_OPPGOAL < 0:
        ANGEL_OPPGOAL += 360
    return hsv2
flag_robots = False
angle_robot = 0
dist_robot = 0
x_robot, y_robot = 0, 0
old_x_robot, old_y_robot = 0, 0
Robots = validFlag(flag_robots, 0.5, 2)
valid_flag_robots = False
def check_robots(dist = 800, min_dist = 180):
    global x_robot, y_robot, old_y_robot, old_x_robot, angle_robot, dist_robot
    robots = []
    flag = False
    min_cord = min(X1, X2, Y1, Y2) * 0.9
    for i in range(0, 180):
        if i > 89:
            i+=180
        # a1 = 90 - ANG
        # a2 = 270 - ANG
        # if a1 < 0: a1+=360
        # if a2 < 0: a2 += 360
        # if a1 > 90 and a2 < 270:
        #     F = (i > a1) or (i < a2)
        # else:
        #     F = (i > a1) and (i < a2)
        # if F == False:
            # print(i)
            # ang = i + ANG
            # if ang > 359: ang -= 360
            #
            # if ang < 90:
            #     a = ang
            #     l1 = Y1 / cos(a/180*pi)
            #     l2 = X2 / cos((90 - a)/180*pi)
            # elif 90 <= ang < 180:
            #     a = ang - 90
            #     l1 = X2 / cos(a/180*pi)
            #     l2 = Y2 / cos((90 - a)/180*pi)
            # elif 180 <= ang < 270:
            #     a = ang - 180
            #     l1 = Y2 / cos(a / 180 * pi)
            #     l2 = X1 / cos((90 - a) / 180 * pi)
            # else:
            #     a = ang - 270
            #     l1 = X1 / cos(a / 180 * pi)
            #     l2 = Y1 / cos((90 - a) / 180 * pi)
            #
            # if abs(l1) < abs(l2):
            #     min_cord = abs(l1) * 0.7
            # else:
            #     min_cord = abs(l2) * 0.7

            # x, y = min_cord * sin(ang / 180 * pi), min_cord * cos(ang / 180 * pi)
            # X, Y = round((X1 + x)/(X1+X2)*140), round((Y1 - y)/(Y1+Y2)*210)
            # cords.append([X, Y])

        if dist > ld.dists[i] > min_dist and ld.dists[i] < min_cord:
            i1 = i - 1
            if i1 < 0: i1 += 360
            i3 = i + 1
            if i3 > 359: i3 -= 360

            if min_dist < ld.dists[i1] < dist and ld.dists[i1] < min_cord and min_dist < ld.dists[i3] < dist and ld.dists[i3] < min_cord:
                flag = True
                robots.append([i, ld.dists[i]])
    if flag:
        min_dist_robot = [0, 100000]
        for robot in robots:
            if robot[1] < min_dist_robot[1]:
                min_dist_robot = robot
        angle_robot, dist_robot = min_dist_robot[0], min_dist_robot[1]
        angle_robot += ANG
        if angle_robot > 359: angle_robot -= 360
        x, y = round(dist_robot * sin(angle_robot / 180 * pi)), round(
        dist_robot * cos(angle_robot / 180 * pi))
        x_robot, y_robot = X1 + x, Y1 - y
        # if state_2_robot != 9:
        #     if abs(x_robot - X_2_robot) < 100 or abs(y_robot - Y_2_robot) < 100:
        #         flag = False
        # if (abs(x_robot - old_x_robot) < 300 and abs(y_robot - old_y_robot) < 300) or (old_x_robot == 0 or old_y_robot == 0):
        #     old_x_robot, old_y_robot = x_robot, y_robot
        # else:
        #     x_robot, y_robot = old_x_robot, old_y_robot
        #     Robots.give(False)

    elif valid_flag_robots:
        x_d, y_d = X1 - x_robot, Y1 - y_robot
        dist_robot = (x_d**2 + y_d**2)**0.5
        if y_d != 0:
            angle_robot = round(np.arctan(x_d/y_d) / 3.14 * 180)

    # if abs(x_robot - old_x_robot) < 200 and abs(y_robot - old_y_robot) < 200:
    #     flag1 = True

    # old_x_robot, old_y_robot = x_robot, y_robot

    return flag

def moving(angle, sp):
    sp = -sp
    sp1, sp2, sp3, sp4 = 0, 0, 0, 0

    if angle < 0:
        angle = -angle
        k = -1
    else:
        k = 1

    if angle > 360:
        angle -= 360
    if angle < 0:
        angle += 360

    if angle > 180:
        angle = -(360 - angle)*k
    else:
        angle = angle*k

    if angle >= 0 and angle < 45:
        sp2 = round(sp * tan(radians(45 - angle)))
        sp4 = round(sp * tan(radians(45 - angle)))
        sp1 = sp
        sp3 = sp

    elif  angle >= 45 and angle < 90:
        sp2 =  round(-sp * tan(radians(angle - 45)))
        sp4 =  round(-sp * tan(radians(angle - 45)))
        sp1 = sp
        sp3 = sp

    elif angle >= 90 and angle < 135:
        sp1 =  round(sp * tan(radians(135 - angle)))
        sp3 =  round(sp * tan(radians(135 - angle)))
        sp2 = -sp
        sp4 = -sp

    elif angle >= 135 and angle <= 180:
        sp1 =  round(-sp * tan(radians(angle - 135)))
        sp3 =  round(-sp * tan(radians(angle - 135)))
        sp2 = -sp
        sp4 = -sp

    elif angle <= 0 and angle > -45:
        sp1 =  round(sp * tan(radians(45 + angle)))
        sp3 =  round(sp * tan(radians(45 + angle)))
        sp2 = sp
        sp4 = sp

    elif angle <= -45 and angle > -90:
        sp1 =  round(-sp * tan(radians(-45 - angle)))
        sp3 =  round(-sp * tan(radians(-45 - angle)))
        sp2 = sp
        sp4 = sp

    elif angle <= -90 and angle > -135:
        sp2 = round(sp * tan(radians(135 + angle)))
        sp4 = round(sp * tan(radians(135 + angle)))
        sp1 = -sp
        sp3 = -sp

    elif angle <= -135 and angle >= -180:
        sp2 = round(-sp * tan(radians(-135 - angle)))
        sp4 = round(-sp * tan(radians(-135 - angle)))
        sp1 = -sp
        sp3 = -sp

    return sp1, sp2, sp3, sp4
e_old_w = np.zeros(360)
def walls(dist=300, kp=4.0, kd=1.0, max_u=100):
    global sp1, sp2, sp3, sp4, e_old_w
    count = 0
    s1, s2, s3, s4 = 0,0,0,0
    for i in range(0, 360):
        if i > 39 and i < 49 or i > 128 and i < 138 or i > 219 and i < 229 or i > 308 and i < 318:
            pass
        else:
            # j = i + 6
            # if j > 359: j = j - 360
            # k = i - 6
            # if j < 0: j = j + 360
            flag = False
            if ld.dists[i] < dist:
                flag = True
                for f in range(6):
                    j = i + f
                    if j > 359: j = j - 360
                    k = i - f
                    if j < 0: j = j + 360
                    if ld.dists[k] > dist or ld.dists[j] > dist:
                        flag = False

            # if ld.dists[i] < dist and ld.dists[k] < dist and ld.dists[j] < dist:
            if flag:
                count += 1
                angleToWall = i
                e = dist - ld.dists[i]
                u = round(e * kp + (e - e_old_w[i]) * kd)
                e_old_w[i] = e
                u = intersect(u, 0, max_u)
                a, b, c, d = moving(angleToWall, u)
                s1 += a; s2 += b; s3 += c; s4 += d
    if count != 0:
        kf = int(count*0.02)
        s1 = s1//count*kf; s2 = s2//count*kf; s3 = s3//count*kf;  s4 = s4//count*kf
        sp1 += s1; sp2 += s2; sp3 += s3; sp4 += s4
e_old_pd1 = 0
flag_pd1 = False
Pd1 = validFlag(False, 0.5, 0)
valid_flag_pd1 = False
def pd1(ang, target, max_u, center_e, e_min, kp, kd, min_u = 0, dir=0):
    global e_old_pd1, flag_pd1, sp1, sp2, sp3, sp4
    if ang < 0:
        e = pd(-ang, target)
    else:
        e = -pd(ang, target)
    if -e_min<e<e_min:
        e = 0
    if -center_e<e<center_e:
        flag_pd1 = True
    else:
        flag_pd1 = False
    u = round(e * kp + (e - e_old_pd1) * kd)
    u = intersect(u, min_u, max_u)
    if dir == 0:
        sp1 += u
        sp2 += u
        sp3 -= u
        sp4 -= u
    elif dir == 1:
        if u > 0:
            sp1 += u
            sp2 += u
        else:
            sp3 -= u
            sp4 -= u
    elif dir == -1:
        if u < 0:
            sp1 += u
            sp2 += u
        else:
            sp3 -= u
            sp4 -= u
    e_old_pd1 = e
e_old_pd2 = 0
flag_pd2 = False
def pd2(dist ,target, max_u, center_e, e_min, kp, kd):
    global e_old_pd2, flag_pd2, sp1, sp2, sp3, sp4
    e = target - dist
    if -e_min<e<e_min:
        e = 0
    if -center_e<e<center_e:
        flag_pd2 = True
    else:
        flag_pd2 = False
    u = round(e * kp + (e - e_old_pd2) * kd)
    u = intersect(u, 0, max_u)
    sp1 += u
    sp2 += u
    sp3 += u
    sp4 += u
    e_old_pd2 = e
e_old_pd3 = 0
flag_pd3 = False
def pd3(ang, target, max_u, center_e, e_min, kp, kd):
    global e_old_pd3, flag_pd3, sp1, sp2, sp3, sp4

    e = -pd(ang, target)
    if -e_min<e<e_min:
        e = 0
    if -center_e<e<center_e:
        flag_pd3 = True
    else:
        flag_pd3 = False
    u = round(e * kp + (e - e_old_pd3) * kd)
    u = intersect(u, 0, max_u)

    sp1 += u
    sp2 -= u
    sp3 += u
    sp4 -= u
    e_old_pd3 = e
e_old_pdXY = 0
flag_pdXY = 0
def pd_XY(x1, y1, x2, y2, kp=0.3, kd=0.3, max_u = 90):
    global e_old_pdXY, flag_pdXY, sp1, sp2, sp3, sp4

    # x_min, x_max, y_min, y_max = x - e_min_x, x + e_min_x, y - e_min_y, y + e_min_y

    if X1 < x1:
        x_d = x1 - X1
    elif X1 > x2:
        x_d = x2 - X1
    else:
        x_d = 0

    if Y1 < y1:
        y_d = y1 - Y1
    elif Y1 > y2:
        y_d = y2 - Y1
    else:
        y_d = 0

    if y_d != 0 or x_d != 0:
        if y_d != 0:
            tg = x_d / y_d
            a = round(np.arctan(tg) / 3.14 * 180)
        elif x_d != 0:
            tg = y_d / x_d
            a = round(np.arctan(tg) / 3.14 * 180) + 90

        a = -a
        if y_d > 0:
            a = 180 + a
        else:
            if x_d < 0:
                a = 360 + a

        ang = (360 - ANG) + a

        if ang > 360:
            ang -= 360
        if ang < 0:
            ang += 360

        len = (x_d**2 + y_d**2)**0.5
        e = len

        u = round(e * kp + (e - e_old_pdXY) * kd)
        u = intersect(u, 0, max_u)
        s1, s2, s3, s4 = moving(angle=ang, sp=-u)
        sp1 += s1
        sp2 += s2
        sp3 += s3
        sp4 += s4
        e_old_pdXY = e
        flag_pdXY = False
    else:
        flag_pdXY = True

def intersect(sp, min=35, max=100):
    if 0< sp < min:
        sp = min
    if 0 > sp > -min:
        sp = -min
    if sp < -max:
        sp = -max
    if sp > max:
        sp = max
    return sp

but1 = 9
but2 = 9
flag_ballInDribler = False
state_2_robot = 0
X_2_robot = 0
Y_2_robot = 0
ballInDribler = validFlag(flag_ballInDribler, 1.2, 0)
valid_flag_ballInDribler = False
timerUart = time.time()
def UART(message):
    global but1, but2, timerUart, flag_ballInDribler, state_2_robot, X_2_robot, Y_2_robot
    if timerUart + 0.01 < time.time():
        port.write(message.encode('utf-8'))
        timerUart = time.time()
    if port.in_waiting > 0:
        inn = ''
        t = time.time()
        while 1:
            try:
                a = str(port.read(), "utf-8")
            except:
                print('err')
                break
            if a != '$':
                inn += a
            else:
                # print(inn)
                if len(inn) == 6:
                    but1 = int(inn[0])
                    but2 = int(inn[1])
                    flag_ballInDribler = int(inn[2])
                    state_2_robot = int(inn[3])
                    X_2_robot = round(int(inn[4:5])/9*(X1+X2))
                    Y_2_robot = round(int(inn[5:6])/9*(Y1+Y2))
                break
            if t + 0.02 < time.time():
                break
        port.reset_input_buffer()

e_old_pdX_center = 0
flag_pdX_center = False
def pd_X_center(max_u=90, center_e=300, e_min=90, kp=0.3, kd=0.3):
    global e_old_pdX_center, flag_pdX_center, sp1, sp2, sp3, sp4
    e = X1 - X2
    if -e_min < e < e_min:
        e = 0
    if -center_e < e < center_e:
        flag_pdX_center = True
    else:
        flag_pdX_center = False
    u = round(e * kp + (e - e_old_pdX_center) * kd)
    u = intersect(u, 0, max_u)
    sp1 -= u
    sp2 += u
    sp3 -= u
    sp4 += u
    e_old_pdX_center = e

# e_old_pdY_center = 0
# flag_pdY_center = False
# def pd_Y_center(max_u=90, center_e=300, e_min=90, kp=0.3, kd=0.3):
#     global e_old_pdY_center, flag_pdY_center, sp1, sp2, sp3, sp4
#     e = Y1 - Y2
#     if -e_min < e < e_min:
#         e = 0
#     if -center_e < e < center_e:
#         flag_pdY_center = True
#     else:
#         flag_pdY_center = False
#     u = round(e * kp + (e - e_old_pdY_center) * kd)
#     u = intersect(u, 0, max_u)
#     sp1 += u
#     sp2 += u
#     sp3 += u
#     sp4 += u
#     e_old_pdY_center = e

e_old_g = 0
def goal_limit(dist, angle, min_dist=155, kp=3, kd=3, max_u=100):
    global sp1, sp2, sp3, sp4, e_old_g
    x_c = (X1 + X2) // 2
    if flag_coord and (x_c - 150) < X1 < (x_c + 150):
        min_dist = min_dist - 10
    if dist < min_dist:
        e = min_dist - dist
        u = round(e * kp + (e - e_old_g) * kd)
        u = intersect(u, 0, max_u)
        s1, s2, s3, s4 = moving(angle, u)
        sp1+=s1
        sp2+=s2
        sp3+=s3
        sp4+=s4
        e_old_g = e

sp1, sp2, sp3, sp4 = 0, 0, 0, 0
zummer = 0
dribler = 0
kicker = 0
myX, myY = 9,9
myState = 9

type_kick = 0
turn = 1

flag_stop = 0
time_stop = 0

flag_start = 0
time_start = 0

first_start = 0

flag_walls = True
dist_wall = 0

role = "defend"
state = 1
flag_lost = False
timerState = 0

draw = True
home_col = ''

fps = 0
fps_count = 0
t = time.time()
t_start = time.time()

hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
findBall(hsv)
while 1:
    fps_count += 1
    if time.time() > t + 1:
        fps = fps_count
        fps_count = 0
        t = time.time()

    sp1, sp2, sp3, sp4 = 0, 0, 0, 0
    dribler, kicker, zummer = 0, 0, 0
    flag_walls = True
    dist_wall = 300

    if robot.frameGetFlag:
        frame = robot.get_frame(wait_new_frame=1)
        robot.frameGetFlag = False
        frame = cv2.bitwise_and(frame, frame, mask=mask1)
        # frame = cv2.flip(frame,1)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    if flag_start:

        myState = 0

        if first_start == 0:
            low1, up1, low2, up2, home_col = detectHomeGoal(hsv, low_blue, up_blue, low_yellow, up_yellow)
            first_start = 1

        if but1 == 0 and time_start + 0.5 < time.time():
            flag_start = 0
            time_start = time.time()

        if time.time() - t_coord > 0.1:
            robot_coord(hsv)
            t_coord = time.time()
            x_c = (X1 + X2) // 2
            y_c = (Y1 + Y2) // 2
            myX, myY = round(X1 / (X1 + X2) * 9), round(Y1 / (Y1 + Y2) * 9)

        valid_flag_ballInDribler = ballInDribler.getValid(flag_ballInDribler)
        if valid_flag_ballInDribler:
            dribler = 70
            myState = 3
        if flag_ballInDribler:
            dribler = 70
            myState = 2

        if role == "defend":
            angel_HomeGoal, dist_HomeGoal, flag_HomeGoal, area_HomeGoal = GOAL(hsv, low1, up1)
            valid_flag_HomeGoal = homeGoal.getValid(flag_HomeGoal)
            if valid_flag_HomeGoal:
                angel_ball, dist_ball, flag_ball = findBall(hsv)
                valid_flag_ball = ball.getValid(flag_ball)

                if flag_coord:
                    flag_robots = check_robots()
                    valid_flag_robots = Robots.getValid(flag_robots)

                if dist_ball < 150:
                    flag_dist_ball = True
                else:
                    flag_dist_ball = False

                valid_flag_dist_ball = distBall.getValid(flag_dist_ball)
                if flag_dist_ball or valid_flag_dist_ball:
                    if dribler == 0:
                        dribler = 15

                if state == 1:
                    if valid_flag_ballInDribler:
                        state = 4
                    elif valid_flag_ball and dist_HomeGoal < 210:
                        state = 2
                    elif valid_flag_robots and dist_HomeGoal < 210:
                        state = 2
                    else:
                        if flag_coord:
                            pd_X_center(max_u=98, kp=0.07, kd=0.07, e_min=100)
                            # pd_XY((X1+X2)//2-100, y_c, (X1+X2)//2+100, Y1+Y2, kp=0.1, kd=0.1, max_u=60)
                            # if Y1 > y_c:
                        pd1(ang=angel_HomeGoal, target=180, max_u=98, center_e=90, e_min=10, kp=0.7, kd=0.7)
                        pd2(dist=dist_HomeGoal, target=195, max_u=98, center_e=40, e_min=12, kp=3, kd=3)

                elif state == 2:
                    dist_wall = 500
                    if valid_flag_ballInDribler:
                        state = 4
                    elif 150 < angel_ball < 210:
                        distBall.give(True)
                        state = 3

                    elif valid_flag_dist_ball:
                        if state_2_robot == 2 or 3:
                            kf = 0.8
                            kf_len = round(dist_ball / 300 * 20)
                            pd3(ang=angel_ball, target=0, max_u=98, e_min=6, kd=6 * kf, kp=4 * kf, center_e=15)
                            pd1(ang=angel_HomeGoal, target=180, max_u=98, center_e=30, e_min=5, kp=2 * kf, kd=5 * kf)
                            pd2(dist=dist_HomeGoal, target=170, max_u=98, center_e=30, kp=3 * kf, kd=5 * kf, e_min=16)
                        else:
                            state = 3

                    elif valid_flag_ball:
                        kf = 0.7
                        kf_len = round(dist_ball / 300 * 20)
                        pd3(ang=angel_ball, target=0, max_u=98, e_min=6, kd=5*kf, kp=3*kf, center_e=15)
                        pd1(ang=angel_HomeGoal, target=180, max_u=98, center_e=30, e_min=5, kp=2*kf, kd=5*kf)
                        pd2(dist=dist_HomeGoal, target=170, max_u=98, center_e=30, kp=3 * kf, kd=5 * kf, e_min=16)
                    elif valid_flag_robots:
                        # if x_robot < (X1+X2)//7*3:
                        #     target = 30
                        # elif x_robot > (X1+X2)//7*4:
                        #     target = 330
                        # else:
                        target = 0
                        kf = 0.5
                        pd3(ang=angle_robot, target=target, max_u=60, e_min=10, kd=3 * kf, kp=2 * kf, center_e=20)
                        pd2(dist=dist_HomeGoal, target=160, max_u=60, center_e=30, kp=1 * kf, kd=3 * kf, e_min=15)
                        pd1(ang=angel_HomeGoal, target=180, max_u=60, center_e=30, e_min=10, kp=0.7 * kf,
                            kd=0.7 * kf)
                    else:
                        state = 1

                elif state == 3:
                    dist_wall = 200
                    if valid_flag_ballInDribler:
                        state = 4
                    elif flag_ballInDribler:
                        pass
                        # sp1, sp2, sp3, sp4 = moving(angel_ball - 3, -35)
                    elif valid_flag_ball == False or Y1<y_c or valid_flag_HomeGoal == False or valid_flag_dist_ball == False:
                        state = 2
                        distBall.give(False)
                    elif valid_flag_ball:
                        sp1, sp2, sp3, sp4 = moving(angel_ball - 3, -40)
                        pd1(ang=angel_ball, target=2, max_u=80, center_e=10, e_min=3, kp=0.8, kd=0.8)

                elif state == 4:
                    if type_kick == 0:
                        Pd1.give(False)
                        if 50 < ANG < 310:
                            type_kick = 2
                        else:
                            type_kick = 1

                    if flag_ballInDribler:
                        if type_kick == 1:
                            Pd1.validTimeTrue = 0.5
                            dribler = 70
                            angel_OppGoal, dist_OppGoal, flag_OppGoal, _ = GOAL(hsv, low2, up2, type=2)
                            # valid_flag_OppGoal = oppGoal.getValid(flag_OppGoal)
                            if flag_OppGoal:
                                pd1(ang=angel_OppGoal, target=0, max_u=40, min_u=35, center_e=5, e_min=5, kp=0.1,
                                    kd=0.1, dir=0)
                                valid_flag_pd1 = Pd1.getValid(flag_pd1)
                                if valid_flag_pd1:
                                    sp1, sp2, sp3, sp4 = 0, 0, 0, 0
                                    zummer = 1
                                    kicker = 1
                                    Pd1.give(False)
                                    type_kick = 0
                                    state = 1
                                    side = 'None'

                            elif flag_coord:
                                pd1(ang=ANGEL_OPPGOAL, target=0, max_u=40, min_u=35, center_e=10, e_min=10, kp=0.1,
                                    kd=0.1,
                                    dir=0)
                                valid_flag_pd1 = Pd1.getValid(flag_pd1)
                                if valid_flag_pd1:
                                    sp1, sp2, sp3, sp4 = 0, 0, 0, 0
                                    zummer = 1
                                    kicker = 1
                                    Pd1.give(False)
                                    type_kick = 0
                                    state = 1
                                    side = 'None'
                            else:
                                walls(dist=800, kp=1, kd=1, max_u=40)

                        elif type_kick == 2:
                            angel_OppGoal, dist_OppGoal, flag_OppGoal, _ = GOAL(hsv, low2, up2, type=3)

                            dribler = 90
                            if X1 > 1000:
                                # x = 1500
                                turn = -1
                            else:
                                # x = 500
                                turn = 1

                            Pd1.validTimeTrue = 0.5
                            if flag_OppGoal:
                                pd1(ang=angel_OppGoal, target=180, max_u=40, center_e=5, e_min=5, kp=0.5, kd=0.5)
                                valid_flag_pd1 = Pd1.getValid(flag_pd1)
                                if valid_flag_pd1 and (flag_pdXY or Y1 > y_c):
                                    state = 5
                                    timerState = time.time()
                                    Pd1.give(False)
                                    type_kick = 0

                            elif flag_coord:
                                pd1(ang=-ANGEL_OPPGOAL, target=180, max_u=40, center_e=14, e_min=14, kp=0.1, kd=0.1,
                                    dir=0)
                                valid_flag_pd1 = Pd1.getValid(flag_pd1)
                                if valid_flag_pd1:
                                    state = 5
                                    timerState = time.time()
                                    Pd1.give(False)
                                    type_kick = 0
                            else:
                                walls(dist=800, kp=1, kd=1, max_u=40)
                    else:
                        side = 'None'
                        type_kick = 0
                        state = 1

                elif state == 5:
                    # if timerState + 0 < time.time():
                    dribler = 0
                    if timerState + 0.6 < time.time():
                        if turn == -1:
                            sp1, sp2, sp3, sp4 = -100, -100, 0, 0
                        else:
                            sp1, sp2, sp3, sp4 = 0, 0, -100, -100
                        zummer = 1
                    if timerState + 1 < time.time():
                        state = 1

                goal_limit(dist=dist_HomeGoal, angle=angel_HomeGoal, min_dist=155)

            else:
                if flag_coord:
                    pd_XY((X1 + X2) // 2 - 100, (Y1 + Y2) // 2 + 300, (X1 + X2) // 2 + 100, (Y1 + Y2) // 2 + 500, kp=0.1, kd=0.1, max_u=50)
                else:
                    walls(dist=800, kp=0.2, kd=0.2)

        if flag_walls:
            walls(dist=dist_wall)
    else:
        if but1 == 0 and time_start + 0.5 < time.time():
            flag_start = 1
            time_start = time.time()
        state = 1
        side = 'None'
        type_kick = 0
        myState = 9
        myX = 9
        myY = 9

    if but1 == 0 and but2 == 0:
        flag_start = 0
        flag_stop = 0
        first_start = 0


    joy_controll()

    #стоп
    if flag_stop:
        if but2 == 0 and time_stop + 0.3 < time.time():
            flag_stop = 0
            time_stop = time.time()
        sp1, sp2, sp3, sp4, kicker = 0, 0, 0, 0, 0
        # myState = 9
    else:
        if but2 == 0 and time_stop + 0.3 < time.time():
            flag_stop = 1
            time_stop = time.time()

    #отправка
    sp1 = intersect(sp1)
    sp2 = intersect(sp2)
    sp3 = intersect(sp3)
    sp4 = intersect(sp4)
    dribler = intersect(dribler, 0, 89)
    message = str(sp1 + 200) + str(sp2 + 200) + str(sp3 + 200) + str(sp4 + 200) + str(dribler + 10) + str(kicker) + str(zummer) + str(myState) + str(myX) + str(myY) + "$"
    UART(message)

    #отрисовка
    if draw == True:
        display_frame[0:30,0:shapeX+160] = 0; display_frame[shapeY+30:shapeY+90,0:shapeX+160] = 0; display_frame[30:shapeY+30,0:80] = 0; display_frame[30:shapeY+30,shapeX+80:shapeX+160] = 0
        frame = cv2.line(frame, (shapeX//2, 0), (shapeX//2, shapeY), (0, 0, 0), 1)
        frame = cv2.line(frame, (0, shapeY//2), (shapeX, shapeY//2), (0, 0, 0), 1)

        display_frame = cv2.putText(display_frame, "X1:" + str(round(X1)), (shapeX // 2 + 320, shapeY // 2+30), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 2)
        display_frame =  cv2.putText(display_frame, "X2:" + str(round(X2)), (shapeX // 2 - 220, shapeY // 2+30), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 2)
        cv2.putText(frame, "Y2:" + str(round(Y2)), (shapeX // 2, shapeY // 2 + 230), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        cv2.putText(frame, "Y1:" + str(round(Y1)), (shapeX // 2 , shapeY // 2 - 220), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)


        display_frame[30:shapeY+30, 80:shapeX+80] = frame

        if first_start:
            y, x = 210, 140
            xc, yc = X1 + X2, Y1 + Y2
            play_frame = np.zeros((y, x, 3), dtype='uint8')
            play_frame[:, :, 1] = 255
            if home_col == 'yel':
                cv2.rectangle(play_frame, (40, 0), (x - 40, 10), (255, 0, 0), -1)
                cv2.rectangle(play_frame, (40, y - 10), (x - 40, y), (0, 255, 255), -1)
            else:
                cv2.rectangle(play_frame, (40, 0), (x - 40, 10), (0, 255, 255), -1)
                cv2.rectangle(play_frame, (40, y - 10), (x - 40, y), (255, 0, 0), -1)
            if xc and yc != 0:
                x_0, y_0 = round(X1 / xc * x), round(Y1 / yc * y)
                x_1, y_1 = round(X_2_robot / xc * x), round(Y_2_robot / yc * y)
                x_r, y_r = round(x_robot/xc*x), round(y_robot/yc*y)
            else:
                x_0, y_0 = 0, 0
                x_1, y_1 = 0, 0
                x_r, y_r = 0, 0
            x_v, y_v = x_0 + round(20 * sin((ANG) / 180 * pi)), y_0 - round(20 * cos((ANG) / 180 * pi))
            cv2.line(play_frame, (x_0, y_0), (x_v, y_v), (0, 0, 255), 2)
            cv2.circle(play_frame, (x_1, y_1), 10, (150, 150, 150), -1)
            cv2.circle(play_frame, (x_0, y_0), 10, (255, 255, 255), -1)

            if flag_robots:
                cv2.circle(play_frame, (x_r, y_r), 6, (0, 0, 255), -1)
            elif valid_flag_robots:
                cv2.circle(play_frame, (x_r, y_r), 6, (0, 0, 80), -1)
            display_frame[shapeY+60-y:shapeY+60, shapeX+260-x:shapeX+260] = play_frame
            x, y = round(outsideCircle * sin(ANG / 180 * pi)), round(outsideCircle * cos(ANG / 180 * pi))
            cv2.line(display_frame, (shapeX//2+80, shapeY//2+30), (shapeX//2 + 80 + x, shapeY//2+30 - y), (255, 255, 0), 2)
            cv2.putText(display_frame, "ANG:" + str(round(ANG)), (shapeX // 2 + 80 + x // 2, shapeY // 2 + 30 - y // 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (255, 255, 0), 2)
            x, y = round(outsideCircle * sin(ANGEL_OPPGOAL / 180 * pi)), round(
                outsideCircle * cos(ANGEL_OPPGOAL / 180 * pi))
            cv2.line(display_frame, (shapeX // 2 + 80, shapeY // 2+30),
                     (shapeX // 2 + 80 + x, shapeY // 2 +30 - y), (0, 255, 255), 1)
            cv2.putText(display_frame, "ANG:" + str(round(ANGEL_OPPGOAL)), (shapeX // 2 + 80 + x // 2, shapeY // 2 + 30 - y // 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (255, 255, 0), 1)


            display_frame[0:100, shapeX+160:shapeX+260, 1] = fr1
            display_frame[100:200, shapeX+160:shapeX+260, 1] = fr2
            display_frame[200:300, shapeX+160:shapeX+260, 1] = fr3

        robot.text_to_frame(display_frame, "FPS: " + str(fps) + " mes:" + str(message) + " but:" + str(but1)+str(but2) + " my:" + str(int(myState)), 5, 20)
        robot.text_to_frame(display_frame, "start:" + str(flag_start) + " stop:" + str(flag_stop) + " role:" + str(role) + " state:" + str(state) + " kick:" + str(type_kick), 5, shapeY+30+20)
        robot.text_to_frame(display_frame, "ball:" + str(int(flag_ball)) + str(int(valid_flag_ball)) + " dist:" + str(int(flag_dist_ball)) + str(int(valid_flag_dist_ball)) + " drib:" + str(int(flag_ballInDribler)) + str(int(valid_flag_ballInDribler))
                            + " home:" + str(int(flag_HomeGoal)) + str(int(valid_flag_HomeGoal)) + " opp:" + str(int(flag_OppGoal)) + str(int(valid_flag_OppGoal)) + " pd1:" + str(int(flag_pd1)) + str(int(valid_flag_pd1)), 5, shapeY+30+50)
        robot.set_frame(display_frame, 70)
    else:
        robot.text_to_frame(frame,
                            "FPS: " + str(fps), 5, 20)
        robot.set_frame(frame, 100)
