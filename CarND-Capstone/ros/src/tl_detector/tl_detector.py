#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
from scipy.spatial import KDTree

STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)
        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)
        self.is_site=self.config["is_site"]
        rospy.loginfo("Is Site: {}".format(self.is_site))

        self.bridge = CvBridge ()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()
        
        self.pose = None
        self.wp = None
        self.wp_arr=[]
        self.wp_tree=None
        self.camera_image = None
        self.lights = []
        
        self.state = TrafficLight.UNKNOWN
        self.red_wp = -1
        self.state_count = 0
        self.loop_rate = 10  # 10 Hz
        rospy.logwarn("Traffic-Light-Detector frequency set to {}Hz".format(self.loop_rate))

        self.main_loop()

    def main_loop(self):
        rate=rospy.Rate(self.loop_rate)
        while not rospy.is_shutdown():
            if (self.pose and self.wp):
                light_wp, state = self.process_traffic_lights()

                if self.state != state: #New state detected
                    self.state_count = 0
                    self.state = state
                    self.red_wp = -1
                elif self.state_count >= STATE_COUNT_THRESHOLD: #Same state detected multiple times
                    light_wp = light_wp if state == TrafficLight.RED else -1
                    self.red_wp = light_wp
                    self.upcoming_red_light_pub.publish(Int32(self.red_wp))
                else:
                    self.upcoming_red_light_pub.publish(Int32(self.red_wp))
                self.state_count += 1
            rate.sleep()
    
    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        closest_light = None
        closest_line_id = None

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        if(self.pose and self.wp_tree and self.lights):
            car_id = self.get_closest_waypoint(self.pose)
            
            min_diff=len(self.wp_arr) # Very large val
            for i,light in enumerate(self.lights):
                stop_line=stop_line_positions[i]
                stop_line_id=self.get_closest_waypoint(stop_line)
                d=stop_line_id-car_id
                
                if d>=0 and d<min_diff: # Car is behind traffic light
                    min_diff=d
                    closest_light=light
                    closest_line_id=stop_line_id

        #TODO find the closest visible traffic light (if one exists)

        if closest_light:
            state = self.get_light_state(closest_light)
            return closest_line_id, state

        return -1, TrafficLight.UNKNOWN

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        return light.state
        
        if(not self.has_image):
            return TrafficLight.UNKNOWN

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")
        return self.light_classifier.get_classification(cv_image)


    def get_closest_waypoint(self, pose):
        return self.wp_tree.query(pose)[1]
    
    def pose_cb(self, msg):
        self.pose = [msg.pose.position.x,msg.pose.position.y]

    def waypoints_cb(self, waypoints):
        self.wp = waypoints
        self.wp_arr=[]
        for i in waypoints.waypoints:
            self.wp_arr.append([i.pose.pose.position.x,i.pose.pose.position.y])   # Get all x,y coord
        self.wp_tree=KDTree(self.wp_arr)  # store in a KD tree for efficient lookup of closest point

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        self.has_image = True
        self.camera_image = msg

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
