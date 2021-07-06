#!/usr/bin/env python

import math
import rospy
import time

from mavros_msgs.msg import MountControl
from geometry_msgs.msg import PoseStamped, Quaternion, TwistStamped, Twist
from std_msgs.msg import Header
from std_msgs.msg import String

offset_data = [0.0, 0.0, 0, 0.0]
search=0
alphadrone=0
betadrone=0 
alphagimbal=0
betagimbal=0


def callback1(data):
    global offset_data
    global search
    searchold=offset_data[0]
    new_offset_data= True
    offset_data[0] = float(data.data.split(' ')[0])
    offset_data[1] = float(data.data.split(' ')[1])
    offset_data[2] = float(data.data.split(' ')[2])

    
    if(searchold==offset_data[0]):
        search=search+1
    else:
        search=0


def callpose(data1):
    global alphadrone
    global betadrone

    wdegree=2*math.acos(data1.pose.orientation.w)
    zdegree=2*math.asin(data1.pose.orientation.z)
    k=0
    alphadrone=math.cos(wdegree) 
    if((math.degrees(wdegree)<180 and zdegree<0) or (math.degrees(wdegree)>180 and zdegree>0)):   
       betadrone=-1*math.sqrt(1-(alphadrone*alphadrone))
    else:
       betadrone=math.sqrt(1-(alphadrone*alphadrone))

 

def callpose1(data2):
    global alphagimbal
    global betagimbal

    wdegree=2*math.acos(data2.w)
    zdegree=2*math.asin(data2.z)
    k=0
    alphagimbal=math.cos(wdegree) 
    if((math.degrees(wdegree)<180 and zdegree<0) or (math.degrees(wdegree)>180 and zdegree>0)):   
       betagimbal=-1*math.sqrt(1-(alphagimbal*alphagimbal))
    else:
       betagimbal=math.sqrt(1-(alphagimbal*alphagimbal))




def move_forward():
    searched_angle = 0
    pub = rospy.Publisher('/uav1/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
    gimbal_setangle_pub = rospy.Publisher('/uav1/mavros/mount_control/command', MountControl, queue_size=10)
    rospy.init_node('velocity', anonymous=True)
    rospy.Subscriber('/uav1/mavros/local_position/pose', PoseStamped, callpose) 
    rospy.Subscriber("twistexample", String, callback1)
    rospy.Subscriber('/uav1/mavros/mount_control/orientation', Quaternion, callpose1) 

    msg = TwistStamped()
    rate = rospy.Rate(10) # 10hz
    msg.header=Header()
    msg.twist=Twist()
    gimbal_message = MountControl()
    gimbal_message.header = Header()
    gimbal_message.mode = 2
    a=0
    I=0.0
    Mv_bar=0.0
    e_prev=0
    Mv=0.0
    Iang=0
    eang_prev=0
    Mvang=0
    t=0
    gimbal_yaw=0
    gimbal_pitch=0
    
    while not rospy.is_shutdown():
        msg.header.stamp= rospy.Time.now()
        msg.header.seq=1
        gimbal_message.header.stamp= rospy.Time.now()
        gimbal_message.header.seq=1

     
        gimbal_message.roll = 0.0
        
        rospy.loginfo("%f %f %f %f %f %f",alphadrone,betadrone,alphagimbal,betagimbal, (alphadrone*alphagimbal-betadrone*betagimbal),-1*(alphadrone*betagimbal+betadrone*alphagimbal))  

        if(search>10):
           rospy.loginfo("search")
           msg.twist.linear.x=0.0
           msg.twist.linear.y=0.0
           msg.twist.linear.z=0.0
           I=0
           Iang=0
           t=0
                       

        elif(offset_data[0]!=0.0):
           e=gimbal_pitch
           P=0.08*e
           I=I+0.003*e*(0.33)
           D=0.001*(e-e_prev)/0.33
           Mv=Mv_bar + P + I + D
           gimbal_pitch+=(offset_data[1]-0.5)*-30
           msg.twist.linear.x = 20*(alphadrone*alphagimbal-betadrone*betagimbal);
           msg.twist.linear.y = -20*(alphadrone*betagimbal+betadrone*alphagimbal);
           msg.twist.linear.z = Mv;
           rospy.loginfo("%f %f %f %f",Mv,P,I,D)
           e_prev=e


        if(search>10):
           msg.twist.angular.x = 0.0;
           msg.twist.angular.y = 0.0;
           msg.twist.angular.z = 0.0; 
           gimbal_yaw+=10
           searched_angle+=10
           if(searched_angle%1080<360):
              gimbal_pitch = 0
              pass
           elif(searched_angle%1080<720):
              gimbal_pitch = -45;
              pass
           else:
              gimbal_pitch = 0
              #TODO yukari tasi
              pass
           
        elif(offset_data[0]!=0.0):
           searched_angle = 0
           eang=(180-gimbal_yaw%360)
           Pang=0.01*eang
           Iang=Iang+0.001*eang*0.2
           Dang=0.001*(eang-eang_prev) 
           Mvang=Pang+Iang+Dang
           gimbal_yaw+=(offset_data[0]-0.5)*50
           msg.twist.angular.x = 0.0;
           msg.twist.angular.y = 0.0;
           msg.twist.angular.z = Mvang;

           eang_prev=eang
        else:
           searched_angle = 0
           msg.twist.angular.x = 0.0;
           msg.twist.angular.y = 0.0;
           msg.twist.angular.z = 0.0;      
        
        gimbal_message.pitch = gimbal_pitch
        gimbal_message.yaw = gimbal_yaw

        gimbal_setangle_pub.publish(gimbal_message)
        pub.publish(msg)	

        time.sleep(0.5)
        rate.sleep()
if __name__ == '__main__':
    try:
        move_forward()
    except rospy.ROSInterruptException:
        pass
