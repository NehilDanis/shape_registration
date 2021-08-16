#!/usr/bin/env python
# license removed for brevity
import rospy
import sys
from sensor_msgs.msg import Image
import cv2
import os
from cv_bridge import CvBridge, CvBridgeError

# train path
path = "/home/nehil/images_arm"

class ImageSaver():
    def __init__(self):
        self.set_id = 1
        self.num_img = 1
        self.count = 157
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber("/k4a/rgb/image_rect_color", Image, self.callback)
        self.rate = rospy.Rate(0.5) # 1 Hz
        # Do stuff, maybe in a while loop

    def callback(self, img_msg):
        self.rate.sleep() # Sleeps for 1/rate sec
        '''if self.num_img <= 32:
            try:
              cv_image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
            except CvBridgeError as e:
              print(e)
            #file_name = "set_" + str(self.set_id) + "_image_" + str(self.num_img) + ".jpg"
            file_name = "img" + str(self.count) + ".jpg"
            #crop_img = cv_image[:, 420: 1520]
            scale_percent = 50

            #calculate the 50 percent of original dimensions
            width = int(cv_image.shape[1] * 1/3)
            height = int(cv_image.shape[0] * 1/3)

            # dsize
            dsize = (width, height)

            # resize image
            #cv_image = cv2.resize(cv_image, dsize)
            cv2.imwrite(os.path.join(path, file_name), cv_image)
            self.num_img += 1
            self.count += 1
        elif self.num_img == 33:
            key = raw_input("press q to quit or change the position of the camera and press c to continue...")
            if key == "c":
                print("heyy")
            self.num_img = 1
            self.set_id+= 1'''

        try:
          cv_image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
        except CvBridgeError as e:
          print(e)
        file_name = "img" + str(self.count) + ".jpg"
        cv2.imshow("img", cv_image)
        cv2.waitKey(1)
        cv2.imwrite(os.path.join(path, file_name), cv_image)
        self.count += 1

def main(args):
    rospy.init_node('ImageSaverNode', anonymous=True)
    pp = ImageSaver()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)

