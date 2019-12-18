#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, cv2, sys, math, time
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class Pix_to_Dis:
    def __init__(self):
        self.img_left = rospy.Subscriber('/camera/color/image_raw', Image, self.left, queue_size=1)
        self.depth = rospy.Subscriber('/camera/depth/image_rect_raw', Image, self.depth_cb, queue_size=1)
        self.bridge = CvBridge()
        self.img_depth = []
        self.center_x = 0
        self.center_y = 0
        self.dep_val = 0

    def left(self, ldata):
        in_x = 0
        in_y = 0
        x = 0
        y = 0

        img_left = self.bridge.imgmsg_to_cv2(ldata,'bgr8')
        self.l_img = img_left.copy()

        self.center_x = self.l_img.shape[1]/2 #画像のx軸の中心
        self.center_y = self.l_img.shape[0]/2 #画像のy軸の中心
        in_x = self.l_img.shape[1]/4 #画像のx軸の1/4
        in_y = self.l_img.shape[0]/2 #画像のy軸の中心

        #中心のデプス
        self.depth_listener(self.center_x,self.center_y)
        center_dep = self.dep_val

        #1/4の位置のデプス
        self.depth_listener(in_x,in_y)
        in_dep = self.dep_val

        cv2.circle(self.l_img,(self.center_x, self.center_y), 10, (255, 0, 0), -1)
        cv2.putText(self.l_img,str(center_dep),(self.center_x,self.center_y),cv2.FONT_HERSHEY_TRIPLEX,1.0, (0, 0, 255), lineType=cv2.LINE_AA)
        cv2.circle(self.l_img,(in_x, in_y), 10, (255, 0, 0), -1)
        cv2.putText(self.l_img,str(in_dep),(in_x,in_y),cv2.FONT_HERSHEY_TRIPLEX,1.0, (0, 0, 255), lineType=cv2.LINE_AA)

        if np.isnan(center_dep) or np.isinf(center_dep):
            pass
        else:
            self.center_x = ((self.center_x-616.7)*center_dep)/677
            self.center_y = ((self.center_y-367.1)*center_dep)/677
            print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
            print(self.center_x,self.center_y,'center')


        if np.isnan(in_dep) or np.isinf(in_dep):
            pass
        else:
            #ZED Miniカメラの内部パラメータ
            in_x = ((in_x-616.7)*in_dep)/677　
            in_y = ((in_y-367.1)*in_dep)/677
            print(in_x,in_y,'in')

        #実際の距離の値
        x = round(self.center_x,5) - round(in_x,5)
        y = round(self.center_y,5) - round(in_y,5)

        cv2.putText(self.l_img,(str(x)+','+str(y)),(50,50),cv2.FONT_HERSHEY_TRIPLEX,1.0, (0, 0, 255), lineType=cv2.LINE_AA)
        cv2.imshow('left', self.l_img)
        cv2.waitKey(1)

    #デプス画像のコールバック
    def depth_cb(self,data):
        self.img_depth = self.bridge.imgmsg_to_cv2(data,'32FC1')
    def depth_listener(self,x,y):
        self.dep_val = float(self.img_depth[y,x])
        self.dep_val = round(self.dep_val,5)

if __name__ == '__main__':
    rospy.init_node('pix_to_dis')
    p = Pix_to_Dis()
    rospy.spin()
