#!/usr/bin/env python
#
# Copyright (c) 2013, HiveGround Co., Ltd.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#     * Neither the name of the HiveGround Co., Ltd., nor the name of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Mahisorn Wongphati

PKG = "workspace_calibrator" # this package name
import roslib; roslib.load_manifest(PKG)

import rospy
import sensor_msgs.msg
import sensor_msgs.srv
import message_filters

import os
import Queue
import threading
import functools

import cv2
import numpy as np
import cv_bridge
import tf

SEARCH_SIZE = 50

class WorkspaceCalibrator():
    def __init__(self, size_x, size_y, window, source, target):
        self.size_x = size_x
        self.size_y = size_y
        self.window = window
        self.source_frame = source
        self.target_frame = target
        
        rospy.loginfo("workspace size x {} m y {} m".format(self.size_x, self.size_y))
        rospy.loginfo("search window {}x{}".format(self.window, self.window))
        self.br = cv_bridge.CvBridge()
        cv2.namedWindow("display", cv2.WINDOW_NORMAL)
        cv2.setMouseCallback("display", self.onMouse)
                
        MAX = 1000
        cv2.createTrackbar("x", "display", MAX/2, MAX, self.onTrackbar)
        cv2.createTrackbar("y", "display", MAX/2, MAX, self.onTrackbar)
        cv2.createTrackbar("z", "display", MAX/2, MAX, self.onTrackbar)
        cv2.createTrackbar("roll", "display", 1000, MAX, self.onTrackbar)
        cv2.createTrackbar("pitch", "display", MAX/2, MAX, self.onTrackbar)
        cv2.createTrackbar("yaw", "display", 750, MAX, self.onTrackbar)
        
        
        self.got_camera_info = False
                       
        image_sub = message_filters.Subscriber("image", sensor_msgs.msg.Image)
        image_sub.registerCallback(self.callbackImage)
        info_sub = message_filters.Subscriber("camera_info", sensor_msgs.msg.CameraInfo)
        info_sub.registerCallback(self.callbackCameraInfo)                      
        
        
        
                
        self.points = []        
        self.object_corners = np.array([(0,0,0),
                                        (0, self.size_y, 0),
                                        (self.size_x, self.size_y, 0),
                                        (self.size_x, 0, 0)], np.float32)
        self.tfbr = tf.TransformBroadcaster()

    def onTrackbar(self, value):                
        pass
        
        
        
    def onMouse(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:                        
            if len(self.points) == 4:
                del self.points[:] 
            self.points.append((x, y))
                
        
            
    def waitKey(self):
        key = cv2.waitKey(10)
        if key != -1:
            key = (key % 256) #Bug in opencv python?             
            if key in [27, ord("q")]:
                rospy.signal_shutdown("Quit")
        return key

    def makeGray(self, msg):
        """
        Convert a message into a 8-bit 1 channel monochrome OpenCV image
        """
        # as cv_bridge automatically scales, we need to remove that behavior
        if msg.encoding.endswith("16"):
            mono16 = self.br.imgmsg_to_cv(msg, "mono16")
            mono8 = cv.CreateMat(mono16.rows, mono16.cols, cv.CV_8UC1)
            cv.ConvertScale(mono16, mono8)
            return mono8
        elif "FC1" in msg.encoding:
            # floating point image handling
            img = self.br.imgmsg_to_cv(msg, "passthrough")
            mono_img = cv.CreateMat(img.rows, img.cols, cv.CV_8UC1)
            _, max_val, _, _ = cv.MinMaxLoc(img)
            scale = 255.0 / max_val if max_val > 0 else 1.0
            cv.ConvertScale(img, mono_img, scale)
            return mono_img
        else:
            return self.br.imgmsg_to_cv(msg, "mono8")
        
    def callbackCameraInfo(self, msg):        
        if not self.got_camera_info:                                      
            self.camera_matrix = np.array(msg.K, np.float32).reshape(3, 3)
            #self.dist_coeff = np.array(msg.D, np.float32)
            self.dist_coeff = np.zeros((5,1))
            #print self.camera_matrix
            #print self.dist_coeff                
            self.got_camera_info = True            
    
    def callbackImage(self, msg):        
        cv_image = self.makeGray(msg)      
        
         # Convert the image to a Numpy array since most cv2 functions
        # require Numpy arrays.
        cv_image = np.array(cv_image, dtype=np.uint8)
        image_color = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2BGR)
                                
        if len(self.points) != 0:
            for p in self.points:
                cv2.circle(image_color, p, 3, (0,255,0))      
                cv2.rectangle(image_color,
                              (p[0] - (SEARCH_SIZE / 2), p[1] - (SEARCH_SIZE / 2)),
                              (p[0] + (SEARCH_SIZE / 2), p[1] + (SEARCH_SIZE / 2)),
                              (0, 0, 255))                            

                
        if len(self.points) != 4:            
            cv2.imshow("display", image_color)
            self.waitKey()
            return
        
        corners = np.array(self.points, np.float32)
        cv2.cornerSubPix(cv_image, corners, (self.window, self.window), (-1,-1), 
                         (cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER, 30, 0.1))
        cv2.drawChessboardCorners(image_color, (2,2), corners, True)
        
        
        ret, rvec, tvec = cv2.solvePnP(self.object_corners, corners, self.camera_matrix, self.dist_coeff)
        t = tf.transformations.translation_matrix((tvec[0], tvec[1], tvec[2]))
        q = tf.transformations.quaternion_from_euler(rvec[0], rvec[1], rvec[2])
        result_tf = tf.transformations.concatenate_matrices(t, tf.transformations.quaternion_matrix(q))
        
        droll = ((cv2.getTrackbarPos("roll", "display") - 500) / 500.0) * np.pi
        dpitch = ((cv2.getTrackbarPos("pitch", "display") - 500) / 500.0) * np.pi
        dyaw = ((cv2.getTrackbarPos("yaw", "display") - 500) / 500.0) * np.pi
        dx = ((cv2.getTrackbarPos("x", "display") - 500) / 500.0)
        dy = ((cv2.getTrackbarPos("y", "display") - 500) / 500.0)
        dz = ((cv2.getTrackbarPos("z", "display") - 500) / 500.0)
        dt = tf.transformations.translation_matrix((dx, dy, dz))
        dq = tf.transformations.quaternion_from_euler(droll, dpitch, dyaw)
        dtf = tf.transformations.concatenate_matrices(dt, tf.transformations.quaternion_matrix(dq))                
        result_tf = tf.transformations.concatenate_matrices(result_tf, dtf)
        result_tf = tf.transformations.inverse_matrix(result_tf)
        result_t = tf.transformations.translation_from_matrix(result_tf)
        result_q = tf.transformations.quaternion_from_matrix(result_tf)             
        self.tfbr.sendTransform(result_t, result_q, rospy.Time.now(), self.source_frame, self.target_frame)                                
        cv2.imshow("display", image_color)
        self.waitKey()    
    
def main():    
    from optparse import OptionParser
    parser = OptionParser() 
    parser.add_option("-x", "--size_x", dest="size_x", help="workspace width (m)")
    parser.add_option("-y", "--size_y", dest="size_y", help="workspace height (m)")
    parser.add_option("-w", "--window", dest="window", default = 5, help="corner search window (3, 5, 7, ...)")
    parser.add_option("-s", "--source", dest="source_frame", help="source frame for tf broadcast e.g. /camera_rgb_optical_frame")
    parser.add_option("-t", "--target", dest="target_frame", help="target frame for tf broadcast e.g. /world")
        
    
    options, args = parser.parse_args()
    if not options.size_x or not options.size_y or not options.source_frame or not options.target_frame: 
        print "incorrect number of arguments"
        parser.print_help()  
        return
          
    rospy.init_node("workspace_calibrator")    
    node = WorkspaceCalibrator(options.size_x, 
                               options.size_y, 
                               options.window,
                               options.source_frame,
                               options.target_frame)
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except Exception, e:
        import traceback
        traceback.print_exc()
    

