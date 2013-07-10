#!/usr/bin/env python

# Import required Python code.
import roslib; roslib.load_manifest('kinect_skeleton_interaction')
import rospy
import sys
import kinect_msgs.msg
import tf


class SkeletonTFPublisher():
    def __init__(self):
        self.skeleton_sub = rospy.Subscriber("skeletons", kinect_msgs.msg.Skeletons , self.callbackSkeleton)
        self.tf_br = tf.TransformBroadcaster()
        self.parent_frame = rospy.get_param("~parent_frame")        
        
    def callbackSkeleton(self, msg):
        self.publishTransforms(msg.skeletons)
        for skeleton in msg.skeletons:
            if skeleton.skeleton_tracking_state == kinect_msgs.msg.Skeleton.SKELETON_TRACKED:
                pass
                               
    def publishTransforms(self, skeletons):
        for i in range(len(skeletons)):                        
            if skeletons[i].skeleton_tracking_state == kinect_msgs.msg.Skeleton.SKELETON_TRACKED:               
                time = rospy.Time.now()                                               
                #upper
                self.publishTransform(i, skeletons[i].skeleton_positions[kinect_msgs.msg.Skeleton.SKELETON_POSITION_HEAD], "head", self.parent_frame, time)
                self.publishTransform(i, skeletons[i].skeleton_positions[kinect_msgs.msg.Skeleton.SKELETON_POSITION_SHOULDER_CENTER], "shoulder_center", self.parent_frame, time)
                self.publishTransform(i, skeletons[i].skeleton_positions[kinect_msgs.msg.Skeleton.SKELETON_POSITION_SHOULDER_LEFT], "shoulder_left", self.parent_frame, time)
                self.publishTransform(i, skeletons[i].skeleton_positions[kinect_msgs.msg.Skeleton.SKELETON_POSITION_SHOULDER_RIGHT], "shoulder_right", self.parent_frame, time)
                self.publishTransform(i, skeletons[i].skeleton_positions[kinect_msgs.msg.Skeleton.SKELETON_POSITION_ELBOW_LEFT], "elbow_left", self.parent_frame, time)
                self.publishTransform(i, skeletons[i].skeleton_positions[kinect_msgs.msg.Skeleton.SKELETON_POSITION_ELBOW_RIGHT], "elbow_right", self.parent_frame, time)
                self.publishTransform(i, skeletons[i].skeleton_positions[kinect_msgs.msg.Skeleton.SKELETON_POSITION_HAND_LEFT], "hand_left", self.parent_frame, time)
                self.publishTransform(i, skeletons[i].skeleton_positions[kinect_msgs.msg.Skeleton.SKELETON_POSITION_HAND_RIGHT], "hand_right", self.parent_frame, time)
                
                #lower
                self.publishTransform(i, skeletons[i].skeleton_positions[kinect_msgs.msg.Skeleton.SKELETON_POSITION_SPINE], "spine", self.parent_frame, time)
                self.publishTransform(i, skeletons[i].skeleton_positions[kinect_msgs.msg.Skeleton.SKELETON_POSITION_HIP_CENTER], "hip_center", self.parent_frame, time)
                self.publishTransform(i, skeletons[i].skeleton_positions[kinect_msgs.msg.Skeleton.SKELETON_POSITION_HIP_LEFT], "hip_left", self.parent_frame, time)
                self.publishTransform(i, skeletons[i].skeleton_positions[kinect_msgs.msg.Skeleton.SKELETON_POSITION_HIP_RIGHT], "hip_right", self.parent_frame, time)
                self.publishTransform(i, skeletons[i].skeleton_positions[kinect_msgs.msg.Skeleton.SKELETON_POSITION_KNEE_LEFT], "knee_left", self.parent_frame, time)
                self.publishTransform(i, skeletons[i].skeleton_positions[kinect_msgs.msg.Skeleton.SKELETON_POSITION_KNEE_RIGHT], "knee_right", self.parent_frame, time)
                self.publishTransform(i, skeletons[i].skeleton_positions[kinect_msgs.msg.Skeleton.SKELETON_POSITION_ANKLE_LEFT], "ankle_left", self.parent_frame, time)
                self.publishTransform(i, skeletons[i].skeleton_positions[kinect_msgs.msg.Skeleton.SKELETON_POSITION_ANKLE_RIGHT], "ankle_right", self.parent_frame, time)
                self.publishTransform(i, skeletons[i].skeleton_positions[kinect_msgs.msg.Skeleton.SKELETON_POSITION_FOOT_LEFT], "foot_left", self.parent_frame, time)
                self.publishTransform(i, skeletons[i].skeleton_positions[kinect_msgs.msg.Skeleton.SKELETON_POSITION_FOOT_RIGHT], "foot_right", self.parent_frame, time)
                               
    def publishTransform(self, id, position, child, parent, time):                      
        #convert Kinect SDK frame to depth_frame
        translation = (position.translation.x, position.translation.y, position.translation.z)
        self.tf_br.sendTransform(translation, (0, 0, 0, 1), time, "{}_{}".format(child, id), parent)
        pass
            

# Main function.    
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('skeleton_tf_publisher')
    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        ne = SkeletonTFPublisher()
        rospy.spin()
    except rospy.ROSInterruptException: pass
    
