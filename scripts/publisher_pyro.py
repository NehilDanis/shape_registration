#!/usr/bin/env python
# license removed for brevity
import rospy
import sys
#import cv2
from sensor_msgs.msg import Image
import Pyro5
import Pyro5.api
import numpy as np
#from cv_bridge import CvBridge, CvBridgeError

import msgpack
import msgpack_numpy as m
m.patch()
Pyro5.config.SERIALIZER = "msgpack"




class PublisherPyro(object):
    def __init__(self, uri):
        self.called = False
        self.pyro_server = Pyro5.api.Proxy(uri)
        self.sub = rospy.Subscriber("/k4a/rgb/image_rect_color", Image, self.callback)
        self.pub = rospy.Publisher("mask", Image, queue_size=1)

    def callback(self, img_msg):
        self.pyro_server._pyroClaimOwnership()
        #img1 = np.frombuffer(img_msg.data, dtype=np.uint8)
        #print(img1.shape)
        img = np.frombuffer(img_msg.data, dtype=np.uint8).reshape((img_msg.height, img_msg.width, -1))
        #img = np.ones((200, 200))
        #img[:,:50] = np.zeros((200, 50))
        mask = self.pyro_server.segment_arm(img.copy())

        # the calculated mask using the segmentation network is published to the mask topic
        msg = Image()
        msg.header.stamp = rospy.Time.now()
        msg.height = mask.shape[0]
        msg.width = mask.shape[1]
        msg.encoding = "mono8"
        msg.is_bigendian = False
        msg.step = 1 * mask.shape[1]
        msg.data = np.array(mask).tobytes()
        self.pub.publish(msg)


def main(args):
    rospy.init_node('Pyro_Publisher_Node', anonymous=True)
    #uri = find_arm_segmentation_server()
    PublisherPyro('PYRONAME:segmentation.arm_segment')
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)

