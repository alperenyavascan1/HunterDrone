#!/usr/bin/env python2


from __future__ import division
from tf.transformations import quaternion_from_euler
from threading import Thread
from std_msgs.msg import Header
from std_msgs.msg import String
from six.moves import xrange
from pymavlink import mavutil
from mavros_test_common import MavrosTestCommon
from geometry_msgs.msg import PoseStamped, Quaternion
from mavros_msgs.msg import MountControl
import numpy as np
import math
import rospy

offset_data = [0.0, 0.0]
gimbal_yaw = 0
gimbal_pitch = 0

new_offset_arrived = False
PKG = 'px4'
positionDestination = np.array((1.0,1.0,1.0))
def gimbal_callback(darkros_data):
    global new_offset_arrived
    global offset_data    
    new_offset_arrived = True
    offset_data[0] = float(darkros_data.data.split(' ')[0])
    offset_data[1] = float(darkros_data.data.split(' ')[1])
    #data_to_parse = str(offset_data.data)
    #splitted_data = data_to_parse.split(' ')
    #rospy.loginfo("splitted data is: " + str(splitted_data))
    #x_offset = str(splitted_data[0])
    #y_offset = str(splitted_data[1])
    #splitted_data[0] = float(x_offset)
    #splitted_data[1] = float(y_offset)
    #rospy.loginfo('x_offset: %f y_offset %f', offset_data[0], offset_data[0])
    #rospy.loginfo('%s', data_to_parse)


def move_callback(data):
    global positionDestination
    positionDestination = np.fromstring(str(data.data),dtype = float) - np.array((3,0,0,0))
    #rospy.loginfo("next destination is "+ str(positionDestination))
    
class MavrosOffboardPosctlTest(MavrosTestCommon):
    """
    Tests flying a path in offboard control by sending position setpoints
    via MAVROS.

    For the test to be successful it needs to reach all setpoints in a certain time.

    FIXME: add flight path assertion (needs transformation from ROS frame to NED)
    """

    def setUp(self):
        super(MavrosOffboardPosctlTest, self).setUp()

        self.pos = PoseStamped()
        self.radius = 0.5

        self.pos_setpoint_pub = rospy.Publisher('/uav1/mavros/setpoint_position/local', PoseStamped, queue_size=1)
        self.gimbal_setangle_pub = rospy.Publisher('/uav1/mavros/mount_control/command', MountControl, queue_size=10)

        self.twist_vector_pub=rospy.Publisher('twistexample', String, queue_size=10)
        # send setpoints in seperate thread to better prevent failsafe
        self.pos_thread = Thread(target=self.send_pos, args=())
        self.pos_thread.daemon = True
        self.pos_thread.start()
        rospy.Subscriber('yolomsg', String, gimbal_callback)
        rospy.Subscriber('chatter', String, move_callback)
        self.follow = True

    def tearDown(self):
        super(MavrosOffboardPosctlTest, self).tearDown()

    #
    # Helper methods
    #
    def send_pos(self):
        rate = rospy.Rate(10)  # Hz
        self.pos.header = Header()
        self.pos.header.frame_id = "base_footprint"

        while not rospy.is_shutdown():
            self.pos.header.stamp = rospy.Time.now()
            self.pos_setpoint_pub.publish(self.pos)
            try:  # prevent garbage in console output when thread is killed
                rate.sleep()
            except rospy.ROSInterruptException:
                pass

    def is_at_position(self, x, y, z, offset):
        """offset: meters"""
        rospy.logdebug(
            "current position | x:{0:.2f}, y:{1:.2f}, z:{2:.2f}".format(
                self.local_position.pose.position.x, self.local_position.pose.
                position.y, self.local_position.pose.position.z))

        desired = np.array((x, y, z))
        pos = np.array((self.local_position.pose.position.x,
                        self.local_position.pose.position.y,
                        self.local_position.pose.position.z))
        return np.linalg.norm(desired - pos) < offset

    
    def test_posctl(self):
        """Test offboard position control"""

        # make sure the simulation is ready to start the mission
        self.wait_for_topics(60)
        self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,
                                   10, -1)

        self.log_topic_vars()
        self.set_mode("OFFBOARD", 5)
        self.set_arm(True, 5)

        rospy.loginfo("run mission")
        rate = rospy.Rate(10)

        yaws = (0,45,90)
        self.follow_at_a_safe_point(57)
        #TODO: add the following lines to a function
        # while True:
        #     #self.reach_position(5,5,5,300)
        #     for i in xrange(len(positions)):
        #         # q = quaternion_from_euler(0.0, 0.0, np.deg2rad(90.0))
        #         # p.pose.orientation = Quaternion(*q)
        #         # self.reach_position(positionDestination[0],positionDestination[1],positionDestination[2],300)
        #         tempDest = positionDestination
        #         xx = tempDest[0] - self.pos.pose.position.x
        #         yy = tempDest[1] - self.pos.pose.position.y
        #         zz = tempDest[2] - self.pos.pose.position.z
        #         yaw = math.atan2(yy,xx) # as radians
        #         if(yaw<0):
        #             yaw=yaw+math.pi+math.pi
        #             pass

        #         log = ""
        #         log = log+"position:"
        #         log = log+str(self.pos.pose.position)
        #         log = log+" |other position is: "
        #         log = log+str(tempDest)
        #         log = log+ " angle is: "
        #         log = log+str(yaw)
        #         rospy.loginfo(log)

                
        #         # Turn to the rabbit drone.
        #         quaternion = quaternion_from_euler(0, 0, yaw)
        #         self.pos.pose.orientation = Quaternion(*quaternion)

        #         # Go to the position of the rabbit drone, not called for now.
        #         if(False):
        #             self.reach_position(tempDest[0],tempDest[1],tempDest[2],300)
                
                
        #         rate.sleep()
        #         pass

        self.set_mode("AUTO.LAND", 5)
        self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,
                                   45, 0)
        self.set_arm(False, 5)
    def follow_at_a_safe_point(self,distance):
        global new_offset_arrived
        global gimbal_yaw
        global gimbal_pitch
        rate = rospy.Rate(7.5)
        while(self.follow):
            dest = positionDestination
            yaw = math.atan2(dest[1]-self.pos.pose.position.y,dest[0]-self.pos.pose.position.x) # as radians
            if(yaw<0):
                yaw=yaw+math.pi+math.pi
                pass
            quaternion = quaternion_from_euler(0, 0, yaw)
            self.pos.pose.orientation = Quaternion(*quaternion)
            z = dest[2] + 0.5
            rabbit_y = dest[1]
            rabbit_x = dest[0]
            dx = rabbit_x - self.pos.pose.position.x
            dy = rabbit_y - self.pos.pose.position.y
            current_distance = math.sqrt(dx*dx+dy*dy)
            if(current_distance<distance):
                pass
            else:
                if new_offset_arrived:
                    x_offset = offset_data[0]
                    if x_offset <= 0.125:
                        gimbal_yaw = 0
                    elif 0.125 < x_offset < 0.25:
                        gimbal_yaw = 0
                    elif 0.25 <= x_offset < 0.375:
                        gimbal_yaw = 0
                    elif 0.375 <= x_offset < 0.5:
                        gimbal_yaw = 0
                    elif x_offset == 0.5:
                        gimbal_yaw = 0
                    elif 0.5 < x_offset < 0.625:
                        gimbal_yaw = 0
                    elif 0.625 < x_offset < 0.75:
                        gimbal_yaw = 0
                    elif 0.75 < x_offset < 0.875:
                        gimbal_yaw = 0
                    elif x_offset >= 0.875:
                        gimbal_yaw = 0
                    else:
                        gimbal_yaw = 0
                if new_offset_arrived:
                    y_offset = offset_data[1]
                    if y_offset <= 0.125:
                        gimbal_pitch = 0
                    elif 0.125 < y_offset < 0.25:
                        gimbal_pitch = 0
                    elif 0.25 <= y_offset < 0.375:
                        gimbal_pitch = 0
                    elif 0.375 <= y_offset < 0.5:
                        gimbal_pitch = 0
                    elif offset_data[1] == 0.5:
                        gimbal_pitch = 0
                    elif 0.5 < y_offset < 0.625:
                        gimbal_pitch = 0
                    elif 0.625 <= y_offset < 0.75:
                        gimbal_pitch = 0
                    elif 0.75 <= y_offset < 0.875:
                        gimbal_pitch = 0
                    elif y_offset >= 0.875:
                        gimbal_pitch = 0
                    else:
                        gimbal_pitch = 0
                
                # With the determined angles, rotate gimbal.
                gimbal_message = MountControl()
                gimbal_message.mode = 2
                gimbal_message.roll = 0
                gimbal_message.yaw = gimbal_yaw
                gimbal_message.pitch = gimbal_pitch
                self.gimbal_setangle_pub.publish(gimbal_message)
                rospy.loginfo("Trying to rotate gimbal with yaw" + str(gimbal_yaw) + "pitch" + str(gimbal_pitch))
                new_offset_arrived = False
                
                # first eyes, then head. Here is where we return head.
                must_stop_distance_x = math.cos(yaw) * distance
                must_stop_distance_y = math.sin(yaw) * distance
                x = rabbit_x - must_stop_distance_x
                y = rabbit_y - must_stop_distance_y
                self.pos.pose.position.x = x
                self.pos.pose.position.y = y
                self.pos.pose.position.z = z

                pass
          
            rate.sleep()


            pass
        rospy.loginfo("Stopped following")
        pass


     	rospy.spin()
if __name__ == '__main__':
    import rostest
    rospy.init_node('testi_node', anonymous=True)
    rostest.rosrun(PKG, 'mavros_offboard_posctl_test',
                   MavrosOffboardPosctlTest)
