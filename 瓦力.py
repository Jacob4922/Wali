#!/usr/bin/python3
# coding=utf8
import sys
import cv2
import time
import math
import threading
import numpy as np

import hiwonder.Misc as Misc
import hiwonder.Board as Board
import hiwonder.Camera as Camera
import hiwonder.ActionGroupControl as AGC
import hiwonder.yaml_handle as yaml_handle

import os
sys.path.append('/home/pi/TonyPi/')
import RPi.GPIO as GPIO
import hiwonder.Sonar as Sonar
# 巡线
if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)
# 超声波避障

def setBuzzer(sleeptime):
    GPIO.setup(31, GPIO.OUT) #设置引脚为输出模式,BOARD编码31对应BCM编码6
    GPIO.output(31, 1)       #设置引脚输出高电平
#   time.sleep(sleeptime)    #设置延时
    GPIO.output(31, 0)

# # 抬起左手
# def hand_up():
#     Board.setBusServoPulse(8,330,1000)
#     time.sleep(0.3)
#     Board.setBusServoPulse(7,860,1000)
#     Board.setBusServoPulse(6,860,1000)
#     time.sleep(1)
# # 放下左手
# def hand_down():
#     Board.setBusServoPulse(7,800,1000)
#     Board.setBusServoPulse(6,575,1000)
#     time.sleep(0.3)
#     Board.setBusServoPulse(8,725,1000)
#     time.sleep(1)
# # 向左边伸手
# def hand_left():
#     Board.setBusServoPulse(8,330,1000)
#     time.sleep(0.3)
#     Board.setBusServoPulse(7,420,1000)
#     Board.setBusServoPulse(6,920,1000)
#     time.sleep(1)

distance = 99999

# 巡线
__target_color = ('black',)
# 设置检测颜色
def setLineTargetColor(target_color):
    global __target_color

    __target_color = target_color
    return (True, (), 'SetVisualPatrolColor')

lab_data = None
servo_data = None
def load_config():
    global lab_data, servo_data
    
    lab_data = yaml_handle.get_yaml_data(yaml_handle.lab_file_path)
    servo_data = yaml_handle.get_yaml_data(yaml_handle.servo_file_path)

load_config()

# 初始位置
def initMove():
    Board.setPWMServoPulse(1, servo_data['servo1'], 500)
    Board.setPWMServoPulse(2, servo_data['servo2'], 500)

line_centerx = -1
# 变量重置
def reset():
    global line_centerx
    global __target_color
    
    line_centerx = -1
    __target_color = ()
    
# app初始化调用
def init():
    print("VisualPatrol Init")
    load_config()
    initMove()

__isRunning = False
# app开始玩法调用
def start():
    global __isRunning
    reset()
    __isRunning = True
    print("VisualPatrol Start")

# app停止玩法调用
def stop():
    global __isRunning
    __isRunning = False
    print("VisualPatrol Stop")

# app退出玩法调用
def exit():
    global __isRunning
    __isRunning = False
    AGC.runActionGroup('stand_low')
    print("VisualPatrol Exit")

# 找出面积最大的轮廓
# 参数为要比较的轮廓的列表
def getAreaMaxContour(contours):
    contour_area_temp = 0
    contour_area_max = 0
    area_max_contour = None

    for c in contours:  # 历遍所有轮廓
        contour_area_temp = math.fabs(cv2.contourArea(c))  # 计算轮廓面积
        if contour_area_temp > contour_area_max:
            contour_area_max = contour_area_temp
            if contour_area_temp >= 5:  # 只有在面积大于300时，最大面积的轮廓才是有效的，以过滤干扰
                area_max_contour = c

    return area_max_contour, contour_area_max  # 返回最大的轮廓

img_centerx = 320

Z = 1
# 机器人避障子线程
def move():
    global distance
    global line_centerx
    global Z
    dist_left = []
    dist_right = []
    distance_left = 99999
    distance_right = 99999

    while True:
          if distance != 99999:
              if distance <= 500:  # 检测前方障碍物
                distance = 99999       
                if Z == 1:
            
                    for i in range(4):
                        #time.sleep(0.01)                    
                        AGC.runActionGroup('turn_left')
                        time.sleep(0.01)   
                    AGC.runActionGroup('turn_left_small_step')
                    for i in range(12):
                        AGC.runActionGroup('go_forward_fast')
                        time.sleep(0.04)
                    #time.sleep(0.07)                    
                    for i in range(4):
                         time.sleep(0.01)                     
                         AGC.runActionGroup('turn_right')
                         time.sleep(0.01)                         
                    AGC.runActionGroup('turn_right_small_step')
                        
                    Z = 2
                elif Z == 2:
                    #time.sleep(0.07)                
                    for i in range(3):
                        #time.sleep(0.01)                    
                        AGC.runActionGroup('turn_right')
                        time.sleep(0.01)                         
                    for i in range(12):
                         AGC.runActionGroup('go_forward_fast')
                         time.sleep(0.04)                       
                    for i in range(7):
                         #time.sleep(0.01)                    
                         AGC.runActionGroup('turn_left')
                         #time.sleep(0.01)
                    AGC.runActionGroup('turn_left_small_step')     
             
                    Z = 1
              else:
              
                   AGC.runActionGroup('go_forward_fast')
                   time.sleep(0.04)
                   
          else:
               time.sleep(0.1)
          if line_centerx != -1:
            if line_centerx - img_centerx < 0.1:
                for i in range(4):
                    time.sleep(0.01)                
                    AGC.runActionGroup('turn_right')
                    time.sleep(0.01)
                AGC.runActionGroup('turn_right_small_step')
                 

                    
            elif line_centerx - img_centerx > -0.1:
                for i in range(9):
                    #time.sleep(0.01)                
                    AGC.runActionGroup('turn_left')
                    #time.sleep(0.01)                     


                
# 作为子线程开启
th = threading.Thread(target=move)
th.setDaemon(True)
th.start()


roi = [ # [ROI, weight]
        (240, 280,  0, 640, 0.1), 
        (340, 380,  0, 640, 0.3), 
        (440, 480,  0, 640, 0.6)
       ]

roi_h1 = roi[0][0]
roi_h2 = roi[1][0] - roi[0][0]
roi_h3 = roi[2][0] - roi[1][0]

roi_h_list = [roi_h1, roi_h2, roi_h3]

size = (640, 480)
def run(img):
    global line_centerx
    global __target_color
    
    img_copy = img.copy()
    img_h, img_w = img.shape[:2]
    
#     cv2.line(img, (int(img_w/2), 0), (int(img_w/2), img_h), (255, 0, 255), 2)
    
    if not __isRunning or __target_color == ():
        return img
    
    frame_resize = cv2.resize(img_copy, size, interpolation=cv2.INTER_NEAREST)
    frame_gb = cv2.GaussianBlur(frame_resize, (3, 3), 3)   
            
    centroid_x_sum = 0
    weight_sum = 0
    center_ = []
    n = 0

    #将图像分割成上中下三个部分，这样处理速度会更快，更精确
    for r in roi:
        roi_h = roi_h_list[n]
        n += 1       
        blobs = frame_gb[r[0]:r[1], r[2]:r[3]]
        frame_lab = cv2.cvtColor(blobs, cv2.COLOR_BGR2LAB)  # 将图像转换到LAB空间
        
        area_max = 0
        areaMaxContour = 0
        for i in lab_data:
            if i in __target_color:
                detect_color = i
                frame_mask = cv2.inRange(frame_lab,
                                         (lab_data[i]['min'][0],
                                          lab_data[i]['min'][1],
                                          lab_data[i]['min'][2]),
                                         (lab_data[i]['max'][0],
                                          lab_data[i]['max'][1],
                                          lab_data[i]['max'][2]))  #对原图像和掩模进行位运算
                eroded = cv2.erode(frame_mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  #腐蚀
                dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))) #膨胀
        dilated[:, 0:160] = 0
        dilated[:, 480:640] = 0        
        cnts = cv2.findContours(dilated , cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_L1)[-2]#找出所有轮廓
        cnt_large, area = getAreaMaxContour(cnts)#找到最大面积的轮廓
        if cnt_large is not None:#如果轮廓不为空
            rect = cv2.minAreaRect(cnt_large)#最小外接矩形
            box = np.int0(cv2.boxPoints(rect))#最小外接矩形的四个顶点
            for i in range(4):
                box[i, 1] = box[i, 1] + (n - 1)*roi_h + roi[0][0]
                box[i, 1] = int(Misc.map(box[i, 1], 0, size[1], 0, img_h))
            for i in range(4):                
                box[i, 0] = int(Misc.map(box[i, 0], 0, size[0], 0, img_w))
                
            cv2.drawContours(img, [box], -1, (0,0,255,255), 2)#画出四个点组成的矩形
            
            #获取矩形的对角点
            pt1_x, pt1_y = box[0, 0], box[0, 1]
            pt3_x, pt3_y = box[2, 0], box[2, 1]            
            center_x, center_y = (pt1_x + pt3_x) / 2, (pt1_y + pt3_y) / 2#中心点       
            cv2.circle(img, (int(center_x), int(center_y)), 5, (0,0,255), -1)#画出中心点
            
            center_.append([center_x, center_y])                        
            #按权重不同对上中下三个中心点进行求和
            centroid_x_sum += center_x * r[4]
            weight_sum += r[4]

    if weight_sum is not 0:
        #求最终得到的中心点
        cv2.circle(img, (line_centerx, int(center_y)), 10, (0,255,255), -1)#画出中心点
        line_centerx = int(centroid_x_sum / weight_sum)  
    else:
        line_centerx = -1

    return img

if __name__ == '__main__':
    distance_list = []  # 创建空列表，存储检测的距离数据
    s = Sonar.Sonar()  # 创建一个Sonar对象s
    s.startSymphony()  # 启动超声波传感器的检测

    AGC.runActionGroup('stand_slow')  # 站起
#     time.sleep(1)  # 等待1s
#     hand_up()  # 机器人的手臂举起
    from CameraCalibration.CalibrationConfig import *
    # 加载相机校准参数
    param_data = np.load(calibration_param_path + '.npz')
    # 获取参数
    mtx = param_data['mtx_array']
    dist = param_data['dist_array']
    newcameramtx, _ = cv2.getOptimalNewCameraMatrix(mtx, dist, (640, 480), 0, (640, 480))
    mapx, mapy = cv2.initUndistortRectifyMap(mtx, dist, None, newcameramtx, (640, 480), 5)
    init()
    start()
    __target_color = ('white',)
    open_once = yaml_handle.get_yaml_data('/boot/camera_setting.yaml')['open_once']
    if open_once:
        my_camera = cv2.VideoCapture('http://127.0.0.1:8080/?action=stream?dummy=param.mjpg')
    else:
        my_camera = Camera.Camera()
        my_camera.camera_open()
    AGC.runActionGroup('stand')
    while True:
        # 读取一帧图像
        ret, img = my_camera.read()
        distance_list.append(s.getDistance())
        if ret:
            # 将读取到的图像复制到一个名为frame的变量中
            frame = img.copy()
            # 畸变矫正
            frame = cv2.remap(frame, mapx, mapy, cv2.INTER_LINEAR)
            # 调用一个名为run的函数，传入参数frame，并将返回值存储在Frame变量中
            Frame = run(frame)
            # 在名为'Frame'的窗口中显示Frame变量中的图像
            cv2.imshow('Frame', Frame)
        if len(distance_list) >= 6:
            distance = int(round(np.mean(np.array(distance_list))))
            print(distance, 'mm')
            distance_list = []
            key = cv2.waitKey(1)
            if key == 27:  # 如果用户按下的是Esc键（键码为27），则退出循环
                break
        else:
            time.sleep(0.01)

    my_camera.camera_close()
    cv2.destroyAllWindows()
