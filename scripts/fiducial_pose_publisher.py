#!/usr/bin/env python

#based on ground_truth_pose_publisher from ANYbotics elevation_mapping

import rospy
import geometry_msgs.msg
import std_msgs
import tf

def callback(newPose):
    global publisher, tfListener, br

    fiducial_frame_code = str(rospy.get_param('~fiducial_code', 'fid100'))
    fiducial_frame_code_reoriented = fiducial_frame_code+'_reoriented'
    fiducial_frame_code_fix = fiducial_frame_code+'_fix'
    cov = rospy.get_param('~fiducial_covariance', 0.05)
    x_trans = rospy.get_param('~x_trans', 0.1285)
    y_trans = rospy.get_param('~y_trans', 0.0)
    z_trans = rospy.get_param('~z_trans', -0.151)

    try:
        (trans, rot) = tfListener.lookupTransform('odom', fiducial_frame_code, rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        return
    
    #ignore roll & pitch components
    rotation_with_rp = tf.transformations.euler_from_quaternion(rot)
    rotation = tf.transformations.quaternion_from_euler(-rotation_with_rp[0], -rotation_with_rp[1], 0)
    br.sendTransform((0,0,0),
                     rotation,
                     rospy.Time.now(),
                     fiducial_frame_code_reoriented,
                     fiducial_frame_code)
    br.sendTransform((x_trans, y_trans, z_trans),
                     (0,0,0,1),
                     rospy.Time.now(),
                     fiducial_frame_code_fix,
                     fiducial_frame_code_reoriented)

    try:
        (trans, rot) = tfListener.lookupTransform('odom', fiducial_frame_code_fix, rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        return
    pose = geometry_msgs.msg.PoseWithCovarianceStamped()
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = 'odom'
    pose.pose.pose.position.x = trans[0]
    pose.pose.pose.position.y = trans[1]
    pose.pose.pose.position.z = trans[2]
    pose.pose.pose.orientation.x = rot[0]
    pose.pose.pose.orientation.y = rot[1]
    pose.pose.pose.orientation.z = rot[2]
    pose.pose.pose.orientation.w = rot[3]

    pose.pose.covariance = [cov, 0, 0, 0, 0, 0, 
                            0, cov, 0, 0, 0, 0, 
                            0, 0, cov, 0, 0, 0, 
                            0, 0, 0, cov, 0, 0, 
                            0, 0, 0, 0, cov, 0, 
                            0, 0, 0, 0, 0, cov]

    publisher.publish(pose)
    
#Main function initializes node and subscribers and starts the ROS loop
def main_program():
    global publisher, tfListener, br
    rospy.init_node('fiducial_pose_publisher')
    tfListener = tf.TransformListener()
    br = tf.TransformBroadcaster()
    publisher = rospy.Publisher('fiducial_pose', geometry_msgs.msg.PoseWithCovarianceStamped, queue_size=10)
    rospy.Timer(rospy.Duration(0.05), callback)
    rospy.spin()
        
if __name__ == '__main__':
    try:
        main_program()
    except rospy.ROSInterruptException: pass
