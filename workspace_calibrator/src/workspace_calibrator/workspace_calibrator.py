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

PKG = 'workspace_calibrator' # this package name
import roslib; roslib.load_manifest(PKG)

import rospy
import sensor_msgs.msg
import sensor_msgs.srv
import message_filters

import os
import Queue
import threading
import functools

import cv
import cv2
import cv_bridge

class WorkspaceCalibrator():
    def __init__(self, size_x, size_y):
        rospy.loginfo("workspace size x {} m y {} m".format(size_x, size_y))
        self.br = cv_bridge.CvBridge()
        cv.NamedWindow("display", cv.CV_WINDOW_AUTOSIZE)
        self.font = cv.InitFont(cv.CV_FONT_HERSHEY_SIMPLEX, 0.20, 1, thickness = 2)
        cv.SetMouseCallback("display", self.onMouse)
                       
        msub = message_filters.Subscriber('image', sensor_msgs.msg.Image)
        msub.registerCallback(self.callbackImage)
    def onMouse(self, event, x, y, flags, param):
        if event == cv.CV_EVENT_LBUTTONDOWN:
            rospy.loginfo('x {} y {}'.format(x, y)) 
    
    def waitKey(self):
        key = cv.WaitKey(10)
        if key != -1:
            key = (key % 256) #Bug in opencv python?             
            if key in [27, ord('q')]:
                rospy.signal_shutdown("Quit")
        return key
        
    def callbackImage(self, msg):
        cv_image = self.br.imgmsg_to_cv(msg, "bgr8")
        cv.ShowImage("display", cv_image)
        self.waitKey()    
    
def main():    
    from optparse import OptionParser
    parser = OptionParser() 
    parser.add_option("-x", "--size_x", dest="size_x", help="workspace width (m)")
    parser.add_option("-y", "--size_y", dest="size_y", help="workspace height (m)")
    
    
    options, args = parser.parse_args()
    if not options.size_x or not options.size_y: 
        parser.error("incorrect number of arguments")
        parser.print_help()
    
    rospy.init_node('workspace_calibrator')    
    node = WorkspaceCalibrator(options.size_x, options.size_y)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except Exception, e:
        import traceback
        traceback.print_exc()
    

