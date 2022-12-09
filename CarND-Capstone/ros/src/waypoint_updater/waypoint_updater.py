#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32
from scipy.spatial import KDTree
import numpy as np

import math

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 100 # Number of waypoints we will publish. You can change this number
MAX_DECEL=.5


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint',Int32, self.traffic_cb)
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        self.pose=None
        self.wp=None
        self.wp_arr=[]
        self.wp_tree=None
        self.stop_line_id=-1
        self.loop_rate = 10  # 10 Hz
        rospy.logwarn("Waypoint-updater frequency set to {}Hz".format(self.loop_rate))

        self.main_loop()
    
    def main_loop(self):
        rate=rospy.Rate(self.loop_rate)
        while not rospy.is_shutdown():
            if (self.pose and self.wp_tree):
                self.publish_wp()
            rate.sleep()
            
    def publish_wp(self):
        wp_ahead_msg=self.calc_wp()
        self.final_waypoints_pub.publish(wp_ahead_msg)
    
    def calc_wp(self):
        msg=Lane()
        c_id=self.get_closest_wp()
        f_id=c_id+LOOKAHEAD_WPS
        wp_ahead=self.wp.waypoints[c_id:f_id]
        if (self.stop_line_id!=-1 and self.stop_line_id<(f_id)):
            rospy.logwarn("Red Traffic light detected!")
            wp_ahead=self.reduce_speed(wp_ahead,c_id)
        msg.waypoints=wp_ahead
        return msg
    
    def get_closest_wp(self):
        # Find closest waypoint
        closest_id=self.wp_tree.query(self.pose)[1]
        
        # Check if closest is in front or behind of car
        closest=np.array(self.wp_arr[closest_id])
        previous=np.array(self.wp_arr[closest_id-1])
        car=np.array(self.pose)
        rel_cl=closest-car
        rel_pr=previous-car
        cos_theta=np.dot(rel_cl,rel_pr)/np.abs(np.linalg.norm(rel_cl)*np.linalg.norm(rel_pr))
        
        if (cos_theta>0): # Angle is less than 90, both points are in same dirn i.e. behind the car
            closest_id=(closest_id+1)% len(self.wp_arr)
        return closest_id
    
    def reduce_speed(self,wp_ahead,c_id):
        tmp=[]
        rel_stop_id=max((self.stop_line_id-2)-c_id,0)
        for i,wp in enumerate(wp_ahead):
            p=Waypoint()
            p.pose=wp.pose
            dist=self.distance(wp_ahead,i,rel_stop_id)
            vel=math.sqrt(2*MAX_DECEL*dist)
            if i>rel_stop_id: #If the car overshoots, it stops even in front of line
                vel=0
            if vel<1.: #Complete stop
                vel=0
            if vel>wp.twist.twist.linear.x: #Abide Speed limit
                vel=wp.twist.twist.linear.x
            p.twist.twist.linear.x=vel
            tmp.append(p)
        return tmp
            
        
    def pose_cb(self, msg):
        # Store current pose
        self.pose=[msg.pose.position.x,msg.pose.position.y]

    def waypoints_cb(self, waypoints):
        # Stores waypoints
        self.wp=waypoints;
        
        # Get all x,y coord
        for i in waypoints.waypoints:
            self.wp_arr.append([i.pose.pose.position.x,i.pose.pose.position.y])
        
        # store in a KD tree for efficient lookup of closest point
        self.wp_tree=KDTree(self.wp_arr)
        print("Waypoint Array Initialized, Total pts: "+str(len(self.wp_arr)))
        rospy.loginfo("Waypoint Array Initialized")

    def traffic_cb(self, msg):
        self.stop_line_id=msg.data
        #rospy.logwarn("Stop_line_id:"+str(self.stop_line_id))
        
    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
