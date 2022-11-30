#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import sys, select, os
import tty, termios
import spacy
import speech_recognition as sr
from darknet_ros_msgs.msg import BoundingBoxes

BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.84

WAFFLE_MAX_LIN_VEL = 0.26
WAFFLE_MAX_ANG_VEL = 1.82

LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.05

found = False
searching = False
    

msg = """
Welcome to the Vafor!
---------------------------
Press 'y' to make an inquiry
Press 'u' for home position
Press 'Space' to close grip

CTRL-C to quit
"""

e = """
Communications Failed
"""

from std_msgs.msg import Float64MultiArray

def getKey():

    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def vels(target_linear_vel, target_angular_vel):
    return "currently:\tlinear vel %s\t angular vel %s " % (target_linear_vel,target_angular_vel)

def arms(angle1, angle2, angle3, angle4):
    tmp = "\n angle1 : %s \n angle2 : %s \n angle3 : %s \n angle4 : %s \n\n" % (angle1, angle2, angle3, angle4)
    return tmp

def makeSimpleProfile(output, input, slop):
    if input > output:
        output = min( input, output + slop )
    elif input < output:
        output = max( input, output - slop )
    else:
        output = input

    return output

def constrain(input, low, high):
    if input < low:
      input = low
    elif input > high:
      input = high
    else:
      input = input

    return input

def checkLinearLimitVelocity(vel):
    if turtlebot3_model == "burger":
      vel = constrain(vel, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)
    elif turtlebot3_model == "waffle" or turtlebot3_model == "waffle_pi":
      vel = constrain(vel, -WAFFLE_MAX_LIN_VEL, WAFFLE_MAX_LIN_VEL)
    else:
      vel = constrain(vel, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)

    return vel

def checkAngularLimitVelocity(vel):
    if turtlebot3_model == "burger":
      vel = constrain(vel, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)
    elif turtlebot3_model == "waffle" or turtlebot3_model == "waffle_pi":
      vel = constrain(vel, -WAFFLE_MAX_ANG_VEL, WAFFLE_MAX_ANG_VEL)
    else:
      vel = constrain(vel, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)

    return vel

#function to get the coco.names file
def get_coco_names():
    names = {}
    with open("coco.names", 'r') as f:
        for id, name in enumerate(f):
            names[id] = name[0:-1]
    return names

def getTarget():
    sentence = ""
    target = ""
    while target == "":
        try:
            with sr.Microphone() as source2:
                r.adjust_for_ambient_noise(source2, duration=0.5)
                
                audio2 = r.listen(source2)
                
                sentence = r.recognize_google(audio2)
                sentence = sentence.lower()

                print("Did you say ",sentence)
                
        except sr.RequestError as e:
            print("Could not request results; {0}".format(e))
            
        except sr.UnknownValueError:
            print("unknown error occurred")
            
        doc = nlp(sentence)

        for token in doc:
            if token.dep_ == 'dobj' and token.text in names.values():
                target = token.text
                
    return target

def callback(data):
    global found
    global searching
    
    for box in data.bounding_boxes:
        if box.Class == target and box.xmin < 640 and box.xmax > 640:
            print("Found ", box.Class, '\n')
            found = True
            sub.unregister()
    

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('vafor')
    vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    arm_pub = rospy.Publisher('joint_trajectory_point', Float64MultiArray, queue_size=10)
    grib_pub = rospy.Publisher('gripper_position', Float64MultiArray, queue_size=10)

    turtlebot3_model = rospy.get_param("model", "burger")

    #initialize voice recognizer and language parser
    r = sr.Recognizer()
    nlp = spacy.load('en_core_web_lg')

    #initialize coco.names
    names = get_coco_names()
    target = ""

    status = 0
    target_linear_vel   = 0.0
    target_angular_vel  = 0.0
    control_linear_vel  = 0.0
    control_angular_vel = 0.0

    default_angles = 0.0, -1.2, 0.78, 0.51
    reach_angles = 0.0, 1.53, -1.44, -0.09
    angle1, angle2, angle3, angle4 = default_angles

    grib_angle = 0.025

    try:
        print(msg)
        while not rospy.is_shutdown():
            key = getKey()
            if searching and not found:
                target_angular_vel = ANG_VEL_STEP_SIZE
                target_linear_vel   = 0.0
            elif searching and found:
                target_linear_vel   = 0.0
                control_linear_vel  = 0.0
                target_angular_vel  = 0.0
                control_angular_vel = 0.0
                searching = False
                
            if key == 'y':
                print("How can I help you?")
                target = getTarget()
                print("Target is ",target)
                searching = True
                found = False
                sub = rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, callback)
                
                
            elif key == 'u' :
                print("Stoppping")
                target_linear_vel   = 0.0
                control_linear_vel  = 0.0
                target_angular_vel  = 0.0
                control_angular_vel = 0.0
                angle1, angle2, angle3, angle4 = default_angles
                print(vels(target_linear_vel, target_angular_vel))
            
            elif key == 'r' :
                print("Reaching")
                angle1, angle2, angle3, angle4 = reach_angles
                print(arms(angle1, angle2, angle3, angle4))

            elif key == 'w' :
                target_linear_vel = checkLinearLimitVelocity(target_linear_vel + LIN_VEL_STEP_SIZE)
                status = status + 1
                print(vels(target_linear_vel,target_angular_vel))
            elif key == 'x' :
                target_linear_vel = checkLinearLimitVelocity(target_linear_vel - LIN_VEL_STEP_SIZE)
                status = status + 1
                print(vels(target_linear_vel,target_angular_vel))
            elif key == 'a' :
                target_angular_vel = checkAngularLimitVelocity(target_angular_vel + ANG_VEL_STEP_SIZE)
                status = status + 1
                print(vels(target_linear_vel,target_angular_vel))
            elif key == 'd' :
                target_angular_vel = checkAngularLimitVelocity(target_angular_vel - ANG_VEL_STEP_SIZE)
                status = status + 1
                print(vels(target_linear_vel,target_angular_vel))
            elif key == 's' :
                target_linear_vel   = 0.0
                control_linear_vel  = 0.0
                target_angular_vel  = 0.0
                control_angular_vel = 0.0
                angle1, angle2, angle3, angle4 = default_angles
                print(vels(target_linear_vel, target_angular_vel))
            elif key == '0':
                angle1 += 0.03
                print(arms(angle1, angle2, angle3, angle4))
            elif key == '1':
                angle2 += 0.03
                print(arms(angle1, angle2, angle3, angle4))
            elif key == '4':
                angle3 += 0.03
                print(arms(angle1, angle2, angle3, angle4))
            elif key == '7':
                angle4 += 0.03
            elif key == '.':
                angle1 -= 0.03
                print(arms(angle1, angle2, angle3, angle4))
            elif key == '2':
                angle2 -= 0.03
                print(arms(angle1, angle2, angle3, angle4))
            elif key == '5':
                angle3 -= 0.03
                print(arms(angle1, angle2, angle3, angle4))
            elif key == '8':
                angle4 -= 0.03
                print(arms(angle1, angle2, angle3, angle4))
            elif key == ' ':
                if grib_angle == 0.025:
                    grib_angle = -0.1
                else:
                    grib_angle = 0.025
                
            else:
                if (key == '\x03'):
                    break

            if status == 20 :
                print(msg)
                status = 0

            twist = Twist()

            control_linear_vel = makeSimpleProfile(control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE/2.0))
            twist.linear.x = control_linear_vel; twist.linear.y = 0.0; twist.linear.z = 0.0

            control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE/2.0))
            twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = control_angular_vel

            vel_pub.publish(twist)

            arm_msg = Float64MultiArray()
            arm_msg.data = [-1, angle1, angle2, angle3, angle4]
            arm_pub.publish(arm_msg)
            
            grib_msg = Float64MultiArray()
            grib_msg.data = [grib_angle]
            grib_pub.publish(grib_msg)


            

    except:
        print("Some error:", e)

    finally:
        twist = Twist()
        twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
        vel_pub.publish(twist)

        arm_msg = Float64MultiArray()
        arm_msg.data = [-1, 0.0, -1.2, 0.78, 0.51]
        arm_pub.publish(arm_msg)

        grib_msg = Float64MultiArray()
        grib_msg.data = [0.025]
        grib_pub.publish(grib_msg)


    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
