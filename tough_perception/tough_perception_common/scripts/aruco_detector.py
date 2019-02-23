#!/usr/bin/env python
import rospy
from fiducial_msgs.msg import FiducialTransform, FiducialTransformArray
from geometry_msgs.msg import Pose, PoseStamped
import tf
from threading import Thread, Lock
import time

MARKER_DICT = { 10:"object1",
                15:"object2"}

class ArucoDetector(object):
    def __init__(self,  broadcast_tf=False, 
                        base_frame="multisense/head", 
                        fiducial_tf_topic="/fiducial_transforms"):

        rospy.init_node('aruco_detector') 
        self.fiducial_tf_topic = fiducial_tf_topic
        self.broadcast_tf = broadcast_tf
        self.base_frame = base_frame
        self.markers = list()
        self.mutex = Lock()

        if self.broadcast_tf:
            self.br = tf.TransformBroadcaster()

        rospy.Subscriber(self.fiducial_tf_topic, FiducialTransformArray, self.__fiducial_cb)
        # rospy.spin()

    def __fiducial_cb(self, msgs):
        self.mutex.acquire()
        markers = []
        for msg in msgs.transforms:
            _id = MARKER_DICT[msg.fiducial_id]
            _tf = msg.transform
            _tr = _tf.translation
            _rot = _tf.rotation
            
            markers.append(msg) 

            if self.broadcast_tf:
                self.br.sendTransform((_tr.x,_tr.y,_tr.z),
                                (_rot.x,_rot.y,_rot.z,_rot.w),
                                rospy.Time.now(),
                                _id,
                                self.base_frame)
        self.markers = markers
        self.mutex.release()

    def get_detected_objects(self, MARKER_DICT=None):
        _detected_objects = list()
        self.mutex.acquire()
        for marker in self.markers:
            _detected_objects.append(self.__create_pose_msg(marker))
        self.mutex.release()
        return _detected_objects
    
    def __create_pose_msg(self, fiducial_msg):
        obj_pose = PoseStamped()
        obj_pose.header.stamp = rospy.Time.now()
        obj_pose.header.frame_id = self.base_frame
        obj_pose.pose.position.x = fiducial_msg.transform.translation.x
        obj_pose.pose.position.y = fiducial_msg.transform.translation.y
        obj_pose.pose.position.z = fiducial_msg.transform.translation.z

        obj_pose.pose.orientation.x = fiducial_msg.transform.rotation.x
        obj_pose.pose.orientation.y = fiducial_msg.transform.rotation.y
        obj_pose.pose.orientation.z = fiducial_msg.transform.rotation.z
        obj_pose.pose.orientation.w = fiducial_msg.transform.rotation.w

        return obj_pose
    
    def __del__(self):
        rospy.signal_shutdown("shutting down")

if __name__ == '__main__':
    print("Aruco detector node")

    detector = ArucoDetector()

    while True:
        try:
            _objects = detector.get_detected_objects(MARKER_DICT)
            print("detected_objects",_objects)
            time.sleep(0.5)
        except Exception as e: 
            quit()
