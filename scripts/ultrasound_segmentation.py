#!/usr/bin/env python
# license removed for brevity
import rospy
import sys
from sensor_msgs.msg import Image
import numpy as np
import cv2
import os
from cv_bridge import CvBridge, CvBridgeError
import time

import torch
import torchvision.transforms as transforms
from modules.UNet import UNet
device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')



class UltrasoundSegmentation():
    def __init__(self):
        PATH = os.path.expanduser("~/new_arm_w_tarso_data_folder/UNet/unet_usseg_arm_phantom.pth")
        self.unet = UNet(init_features=64).to(device)
        self.unet.load_state_dict(torch.load(PATH))
        self.unet.eval()

        self.bridge = CvBridge()
        self.transform_image = transforms.Compose([
                    transforms.ToTensor(),
                    transforms.Normalize((0.5,), (0.5,))
                ])
        self.pub_img = rospy.Publisher("segmentedImg",Image, queue_size=2)
#        while(True):
#            #img_msg = rospy.wait_for_message("/imfusion/cephasonics",Image)
#            self.callback(img_msg)
        self.sub_img = rospy.Subscriber("ultrasound_img",Image,self.callback)

    def callback(self, img_msg):
        print(device)
        img_msg.encoding = 'mono8'
        try:
          img = self.bridge.imgmsg_to_cv2(img_msg)
        except CvBridgeError as e:
          print(e)

        orig_size = img.shape
        print(orig_size)

        tmp = cv2.resize(img, (256, 256), interpolation=cv2.INTER_LANCZOS4)
        img = tmp.astype(np.float) / 255


        print("started segmentation")

        x = self.transform_image(img)
        x = x.view(-1, 1, 256, 256).to(device, dtype=torch.float)
        pred_tensor = self.unet(x)


        pred = pred_tensor.view(256, 256)


        start_sending_time = time.time()
        pred = pred.cpu().detach().numpy()

        end_sending_time = time.time()
        print("sending the image and receiving back the mask time : ")
        print(end_sending_time - start_sending_time)
        pred = (pred * 255).astype(np.uint8)
        _, mask = cv2.threshold(pred, thresh=127, maxval=255, type=cv2.THRESH_BINARY)

        print("finished segmentation")


#        mask = cv2.resize(mask,(256, 375), interpolation=cv2.INTER_LANCZOS4)
        mask = cv2.resize(mask, (orig_size[1], orig_size[0]), interpolation=cv2.INTER_LANCZOS4)
#        mask = cv2.resize(mask, (375, 550), interpolation=cv2.INTER_LANCZOS4)
#        cv2.imshow("mask", mask)
#        cv2.waitKey(1)



        # the calculated mask using the segmentation network is published to the mask topic
        msg = Image()
        msg.header.stamp = img_msg.header.stamp
        msg.height = mask.shape[0]
        msg.width = mask.shape[1]
        msg.encoding = "mono8"
        msg.is_bigendian = False
        msg.step = 1 * mask.shape[1]
        msg.data = np.array(mask).tobytes()
        self.pub_img.publish(msg)


def main(args):
    rospy.init_node('Ultrasound_Segmentation_Node', anonymous=True)
    UltrasoundSegmentation()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)

##!/usr/bin/env python
#import rospy
#import torch
#import torchvision.transforms as transforms

#import cv2
#import math
#from cv_bridge import CvBridge
#from sensor_msgs.msg import Image
#from sensor_msgs.msg import PointCloud2
#from sensor_msgs.msg import PointField

#import numpy as np
#import os,sys,time

#sys.path.append(os.path.expanduser("/home/zhongliang/ros/ws_vessel/Ultrasound_Medical_Robots/network"))

#from modules.UNet import *

#device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

#class ImageBuffer:
#    def init(self):
#        self.bridge = CvBridge()
#        self.sub_img = rospy.Subscriber("/imfusion/cephasonics",Image,self.update_image)
#        #self.sub_img = rospy.Subscriber("/us_image",Image,self.update_image)
#        self.pub_img = rospy.Publisher("/mask",Image)
#        # self.pub_img_debug = rospy.Publisher("/us_image",Image)
#        self.img = None
#        self.stamp = None

#    def update_image(self,msg):
#        self.stamp = msg.header.stamp
#        msg.encoding = 'mono8'
#        #tmp = cv2.resize(self.bridge.imgmsg_to_cv2(msg,desired_encoding='mono8'),(256,256),interpolation=cv2.INTER_LANCZOS4)
#        tmp = cv2.resize(self.bridge.imgmsg_to_cv2(msg),(256,256),interpolation=cv2.INTER_LANCZOS4)
#        # us_image = self.bridge.cv2_to_imgmsg(tmp)
#        # us_image.encoding = 'mono8'
#        # self.pub_img_debug.publish(us_image)

#        self.img = torch.Tensor((tmp.astype(np.float)/255-0.5)*2).unsqueeze(0).unsqueeze(0) #batch + color

#    def get_image(self):
#        if self.img is None:
#            rospy.loginfo("Waiting for the first image.")
#            return -1
#        else:
#            return self.img,self.stamp
#            #TODO: return time stamp to avoid inaccurate tf

#    def send_image(self,img):
#        msg = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
#        self.pub_img.publish(msg)

#if name__ == '__main':
#    rospy.init_node('image_segmentation')

#    pub_pc2 = rospy.Publisher("us_vessel_pointcloud", PointCloud2, queue_size=10)

#    #init networks
#    rospy.loginfo("loading UNet")
#    PATH = os.path.expanduser("~/ros/ws_vessel/Ultrasound_Medical_Robots/network/unet_usseg_phantom.pth")
#    #PATH = os.path.expanduser("~/workspace/us_robot/network/unet_usseg_real.pth")
#    unet = UNet(init_features=64).to(device)
#    unet.load_state_dict(torch.load(PATH))
#    unet.eval()

#    rospy.loginfo("Initialization...")
#    #resize_to=[256,256]

#    img_buf = ImageBuffer()

#    while img_buf.get_image() is -1 and not rospy.is_shutdown():
#        rospy.sleep(0.2)

#    sx = rospy.get_param('/calibration/scaling_x',1.4648e-4)
#    sy = rospy.get_param('/calibration/scaling_y',1.5625e-4)
#    cx = rospy.get_param('/calibration/c_x', -0.01875)
#    cz = rospy.get_param('/calibration/cz', 0)

#    calibMtx = np.array([[sx,0,cx],[0,0,0],[0,sy,cz]])

#    rospy.loginfo("Initialized")

#    run_cntr = 1.0
#    avg_dt = 0.0
#    cx_ = None
#    cy_ = None
#    ti_ = 0
#    while not rospy.is_shutdown():
#        img, curr_stamp = img_buf.get_image()

#        img_cu = img.to(device)

#        ti = time.time()
#        with torch.no_grad():
#                pred_cu = unet(img_cu)
#        dt = time.time()-ti
#        avg_dt = (run_cntr-1)/run_cntr*avg_dt+1.0/run_cntr*dt
#        print("avg pred time: ",avg_dt)
#        #print(dt,", ",run_cntr)
#        run_cntr += 1

#        print("total time: ",ti-ti_)
#        ti_ = ti

#        pred = pred_cu.cpu()
#        pred = np.array(pred[0].permute(1, 2, 0))

#        pred = (pred*255).astype(np.uint8)
#        _,pred = cv2.threshold(pred,thresh=127,maxval=255,type=cv2.THRESH_BINARY)

#        # print(img.shape)
#        img_rgb = cv2.cvtColor(np.array((np.squeeze(img)/2+0.5)*255),cv2.COLOR_GRAY2RGB)
#        pred_rgb = cv2.cvtColor(pred,cv2.COLOR_GRAY2RGB)

#        pred_rgb[:,:,-2] = 0
#        #img_buf.send_image(img_rgb)
#        img_buf.send_image( (pred_rgb*0.2+img_rgb).astype(np.uint8) )
