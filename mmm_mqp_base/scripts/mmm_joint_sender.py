#!/usr/bin/env python

from copy import deepcopy
from os import kill

from click import command
import rospy

import Tkinter as tk
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point32
import moveit_commander
import sys
from math import pi
from threading import Thread
from std_srvs.srv import Empty
from tf.transformations import euler_from_quaternion, quaternion_from_euler

j1Angle = 0
j2Angle = 1
j3Angle = 2
j4Angle = 3
j5Angle = 4
j6Angle = 5
group_name = "manipulator"
altTop = False
toggleButtonState = True
eoatPub = None

def main():
    master = tk.Tk()
    master.title("MMM Simple Sender")
    master.protocol("WM_DELETE_WINDOW", lambda: killPorgram(master))

    valid = master.register(validateJointInput)

    frameLeft = tk.Frame(master)
    frameLeft.grid(column=0,row=0, sticky="nsew")

    framecenter = tk.Frame(master)
    framecenter.grid(column=1,row=0, sticky="nsew")

    frameleftAlt = tk.Frame(master)
    frameleftAlt.grid(column=0,row=0, sticky="nsew")
    frameLeft.tkraise()

    frameRight = tk.Frame(master)
    frameRight.grid(column=2,row=0, sticky="nsew")

    eoatFrame = tk.Frame(master)
    eoatFrame.grid(column=3,row=0, sticky="nsew")

    moveit_commander.roscpp_initialize(sys.argv)
    group = moveit_commander.MoveGroupCommander(group_name)

    #frame left
    titleLabelLeft = tk.Label(frameLeft,text="Send Joint Targets")
    titleLabelLeft.pack(pady = (10,20),padx = (10,10))
    titleLabelLeft.config(font=("Courier", 15))

    l1 = tk.Label(frameLeft,text="Joint 1")
    l2 = tk.Label(frameLeft,text="Joint 2")
    l3 = tk.Label(frameLeft,text="Joint 3")
    l4 = tk.Label(frameLeft,text="Joint 4")
    l5 = tk.Label(frameLeft,text="Joint 5")
    l6 = tk.Label(frameLeft,text="Joint 6")

    e1 = tk.Entry(frameLeft, validate="key", validatecommand=(valid, '%P', 1))
    l1.pack(pady=10)
    e1.pack()
    e1.insert(-1,"0")
    e2 = tk.Entry(frameLeft, validate="key", validatecommand=(valid, '%P', 2))
    l2.pack(pady=10)
    e2.pack()
    e2.insert(-1,"0")
    e3 = tk.Entry(frameLeft, validate="key", validatecommand=(valid, '%P', 3))
    l3.pack(pady=10)
    e3.pack()
    e3.insert(-1,"0")
    e4 = tk.Entry(frameLeft, validate="key", validatecommand=(valid, '%P', 4))
    l4.pack(pady=10)
    e4.pack()
    e4.insert(-1,"0")
    e5 = tk.Entry(frameLeft, validate="key", validatecommand=(valid, '%P', 5))
    l5.pack(pady=10)
    e5.pack()
    e5.insert(-1,"0")
    e6 = tk.Entry(frameLeft, validate="key", validatecommand=(valid, '%P', 6))
    l6.pack(pady=10)
    e6.pack()
    e6.insert(-1,"0")

    jButton = tk.Button(frameLeft, 
                        text="Send Target Angel", 
                        command= lambda: jointAngleButtonEnvoke(group,e1.get(),e2.get(),e3.get(),e4.get(),e5.get(),e6.get())
                        )
    jButton.pack(pady=20)

    #alt left frame start 
    titleLabelLeftalt = tk.Label(frameleftAlt,text="Send Work Space Targets")
    titleLabelLeftalt.pack(pady = (10,20),padx = (10,10))
    titleLabelLeftalt.config(font=("Courier", 15))
    l1alt = tk.Label(frameleftAlt,text="X")
    l2alt = tk.Label(frameleftAlt,text="Y")
    l3alt = tk.Label(frameleftAlt,text="Z")
    l4alt = tk.Label(frameleftAlt,text="Rotation X")
    l5alt = tk.Label(frameleftAlt,text="Rotation Y")
    l6alt = tk.Label(frameleftAlt,text="Rotation Z")
    e1alt = tk.Entry(frameleftAlt, validate="key", validatecommand=(valid, '%P', "X"))
    l1alt.pack(pady=10)
    e1alt.pack()
    e1alt.insert(-1,"0.815")
    e2alt = tk.Entry(frameleftAlt, validate="key", validatecommand=(valid, '%P', "Y"))
    l2alt.pack(pady=10)
    e2alt.pack()
    e2alt.insert(-1,"0")
    e3alt = tk.Entry(frameleftAlt, validate="key", validatecommand=(valid, '%P', "Z"))
    l3alt.pack(pady=10)
    e3alt.pack()
    e3alt.insert(-1,"1.185")
    e4alt = tk.Entry(frameleftAlt, validate="key", validatecommand=(valid, '%P', "rX"))
    l4alt.pack(pady=10)
    e4alt.pack()
    e4alt.insert(-1,"0")
    e5alt = tk.Entry(frameleftAlt, validate="key", validatecommand=(valid, '%P', "rY"))
    l5alt.pack(pady=10)
    e5alt.pack()
    e5alt.insert(-1,"90")
    e6alt = tk.Entry(frameleftAlt, validate="key", validatecommand=(valid, '%P', "rZ"))
    l6alt.pack(pady=10)
    e6alt.pack()
    e6alt.insert(-1,"0")

    jButton = tk.Button(frameleftAlt, 
                        text="Send Target Position", 
                        command= lambda: targetPoseButtonEnvoke(group,e1alt.get(),e2alt.get(),e3alt.get(),e4alt.get(),e5alt.get(),e6alt.get())
                        )
    jButton.pack(pady=20)

    #frame center start
    titleLabelcenter = tk.Label(framecenter,text="Current Joint Angles")
    titleLabelcenter.pack(pady = (10,20),padx = (10,10))
    titleLabelcenter.config(font=("Courier", 15))

    l1center = tk.Label(framecenter,text="Joint 1")
    l2center = tk.Label(framecenter,text="Joint 2")
    l3center = tk.Label(framecenter,text="Joint 3")
    l4center = tk.Label(framecenter,text="Joint 4")
    l5center = tk.Label(framecenter,text="Joint 5")
    l6center = tk.Label(framecenter,text="Joint 6")

    e1center = tk.Label(framecenter, borderwidth=1, relief="solid", width=25)
    l1center.pack(pady=10)
    e1center.pack()
    e2center = tk.Label(framecenter, borderwidth=1, relief="solid", width=25)
    l2center.pack(pady=10)
    e2center.pack()
    e3center = tk.Label(framecenter, borderwidth=1, relief="solid", width=25)
    l3center.pack(pady=10)
    e3center.pack()
    e4center = tk.Label(framecenter, borderwidth=1, relief="solid", width=25)
    l4center.pack(pady=10)
    e4center.pack()
    e5center = tk.Label(framecenter, borderwidth=1, relief="solid", width=25)
    l5center.pack(pady=10)
    e5center.pack()
    e6center = tk.Label(framecenter, borderwidth=1, relief="solid", width=25)
    l6center.pack(pady=10)
    e6center.pack()
    
    jButtoncenter = tk.Button(framecenter, 
                        text="Switch Goal to Work Space", 
                        command= lambda: frameToggle(frameleftAlt,frameLeft, jButtoncenter)
                        )
    jButtoncenter.pack(pady=20)

    #frame right start
    titleLabelright = tk.Label(frameRight,text="Current End Effector Pose")
    titleLabelright.pack(pady = (10,20),padx = (10,10))
    titleLabelright.config(font=("Courier", 15))

    l1right = tk.Label(frameRight,text="X")
    l2right = tk.Label(frameRight,text="Y")
    l3right = tk.Label(frameRight,text="Z")
    l4right = tk.Label(frameRight,text="Rotation X")
    l5right = tk.Label(frameRight,text="Rotation Y")
    l6right = tk.Label(frameRight,text="Rotation Z")

    e1right = tk.Label(frameRight, borderwidth=1, relief="solid", width=25)
    l1right.pack(pady=10)
    e1right.pack()
    e2right = tk.Label(frameRight, borderwidth=1, relief="solid", width=25)
    l2right.pack(pady=10)
    e2right.pack()
    e3right = tk.Label(frameRight, borderwidth=1, relief="solid", width=25)
    l3right.pack(pady=10)
    e3right.pack()
    e4right = tk.Label(frameRight, borderwidth=1, relief="solid", width=25)
    l4right.pack(pady=10)
    e4right.pack()
    e5right = tk.Label(frameRight, borderwidth=1, relief="solid", width=25)
    l5right.pack(pady=10)
    e5right.pack()
    e6right = tk.Label(frameRight, borderwidth=1, relief="solid", width=25)
    l6right.pack(pady=10)
    e6right.pack()

    #end of arm tooling display
    #frame center start
    eoatTitel = tk.Label(eoatFrame,text="End of Arm Tooling")
    eoatTitel.pack(pady = (10,20),padx = (10,10))
    eoatTitel.config(font=("Courier", 15))
    # current pose read out
    eoatLeftFingerLabel =  tk.Label(eoatFrame,text="Left (mm from Center)").pack(pady=1)
    eoatLeftFingerValue = tk.Label(eoatFrame, text="No pose recived", borderwidth=1, relief="solid", width=25)
    eoatLeftFingerValue.pack(pady=1)
    eoatRightFingerLabel =  tk.Label(eoatFrame,text="Right (mm from Center)").pack(pady=1)
    eoatRightFingerValue = tk.Label(eoatFrame, text="No pose recived", borderwidth=1, relief="solid", width=25)
    eoatRightFingerValue.pack(pady=1)
    # set current pose
    eoatLeftFingerInLabel =  tk.Label(eoatFrame,text="Left Dist From Center").pack(pady=1)
    eoatLeftPoseIn = tk.Entry(eoatFrame, width=25)
    eoatLeftPoseIn.pack(pady=1)
    eoatRightFingerInLabel =  tk.Label(eoatFrame,text="Right Dist From Center").pack(pady=1)
    eoatRightPoseIn = tk.Entry(eoatFrame, width=25)
    eoatRightPoseIn.pack(pady=1)
    eoatSpeedInLabel =  tk.Label(eoatFrame,text="Speed").pack(pady=1)
    eoatSpeedIn = tk.Entry(eoatFrame, width=25)
    eoatSpeedIn.pack(pady=1)
    eoatSendPose = tk.Button(eoatFrame, 
                        text="Send Target Pose", 
                        command= lambda:  eoatPublisher(float(eoatLeftPoseIn.get()), float(eoatRightPoseIn.get()), float(eoatSpeedIn.get())), 
                        width=15
                        )
    eoatSendPose.pack(pady=1)
    # set tool offset
    eoatToolOffsetLable =  tk.Label(eoatFrame,text="Currnet Tool Offset value").pack(pady=1)
    eoatToolOffsetValue = tk.Label(eoatFrame, text="0", borderwidth=1, relief="solid", width=25)
    eoatToolOffsetValue.pack(pady=1)
    eoatRightFingerInLabel =  tk.Label(eoatFrame,text="Current Offset (mm)").pack(pady=1)
    eoatRightPoseIn = tk.Entry(eoatFrame, width=25, justify="center")
    eoatRightPoseIn.pack(pady=1)
    eoatSetOffset = tk.Button(eoatFrame, 
                        text="Set Tool offset", 
                        command= lambda: eoatPublisher(float(eoatRightPoseIn.get()), float(eoatRightPoseIn.get()), -111), 
                        width=15
                        )
    eoatSetOffset.pack(pady=1)

    # Status Readout
    eoatStatusReadout =  tk.Label(eoatFrame,text="EOAT Status: none recived", 
                                            borderwidth=1, 
                                            relief="solid", 
                                            width=25, 
                                            wraplength=200, 
                                            justify="center")
    eoatStatusReadout.pack(pady=1)

    # calabrate
    eoatCalabrate = tk.Button(eoatFrame, 
                        text="Calabrate", 
                        command= lambda: eoatPublisher(0.0, 0.0, -732), 
                        width=15
                        )
    eoatCalabrate.pack(pady=1)
    # Estop
    eoatEstop = tk.Button(eoatFrame, 
                        text="ESTOP", 
                        command= lambda: eoatPublisher(0.0, 0.0, -911), 
                        width=15
                        )
    eoatEstop.pack(pady=1)
    #end eoat frame
    masterThread = None

    rospy.init_node('joint_publisher_gui')
    pub = rospy.Publisher("pointcloudOnOff", Bool, queue_size=10)
    
    global toggleButtonState, eoatPub
    toggle_btn = tk.Button(frameRight, text="Start Scan", relief="raised",command = lambda: Thread(target=scanButton).start())
    toggle_btn.pack(pady=20)
    rate = rospy.Rate(60)
    masterThread = None
    while not rospy.is_shutdown():
        try:
            eoatPub = rospy.Publisher("mmm_eoat_command", Point32)
            rospy.Subscriber("joint_states", JointState, lambda msg: Thread(currentJointStateCallback(msg, e1center,e2center,e3center,e4center,e5center,e6center,group,e1right,e2right,e3right,e4right,e5right,e6right)).start())
            rospy.Subscriber("mmm_eoat_position", Point32, lambda msg: Thread(eoatCallback(msg,eoatLeftFingerValue,eoatRightFingerValue,eoatStatusReadout,eoatCalabrate,eoatSetOffset,eoatSendPose)).start())
            
            #start window
            masterThread = Thread(master.mainloop()).start()
            rate.sleep()
        except KeyboardInterrupt:
            print("Recived Keyboard Interrupt =============================================")
            masterThread.join()
            master.destroy()
    masterThread.join()
    master.destroy()

# function for calabrating the end of arm tooling
def eoatPublisher(_x, _y, _z):
    global eoatPub
    point = Point32()
    point.x = _x
    point.y = _y
    point.z = _z
    eoatPub.publish(point)

# end of arm tooling callback function
def eoatCallback(msg, leftLabel,rightLabel,statusLabel,calabrateButton,offsetButton,sendPoseButton):
    leftLabel.config(text=str(msg.x))
    rightLabel.config(text=str(msg.y))
    state = int(msg.z)
    if(state == 0):
        statusLabel.config(text = "Waiting For Instruction")
        calabrateButton['state'] = tk.NORMAL
        offsetButton['state'] = tk.NORMAL
        sendPoseButton['state'] = tk.NORMAL
    else:
        #if the eoat isn't ready for a command disable the buttons
        calabrateButton['state'] = tk.DISABLED
        offsetButton['state'] = tk.DISABLED
        sendPoseButton['state'] = tk.DISABLED
    if(state == 1):
        statusLabel.config(text = "Instruction Execution in Progress")
    elif(state == 2):
        statusLabel.config(text = "A Manipulator has exceeded its limits")
    elif(state == 3):
        statusLabel.config(text = "EOAT In Stopped State")
    elif(state == 4):
        statusLabel.config(text = "EOAT Initializing")
    elif(state == 5):
        statusLabel.config(text = "Test Routine in Progress")
    elif(state == 6):
        statusLabel.config(text = "Tool offsets Set. Max Position now set to x y")
    elif(state == 7):
        statusLabel.config(text = "Invalid command recieved")
    elif(state == 8):
        statusLabel.config(text = "Calibration In Progress")
    elif(state == 9):
        statusLabel.config(text = "Unknown Error")
    elif(state == 911):
        statusLabel.config(text = "Emergency Stop")
    
#for toggling point cloud combinding
def scanButton():
    rospy.wait_for_service('mmm_scan_service')
    try:
        scan_routine = rospy.ServiceProxy('mmm_scan_service', Empty)
        scan_routine()
        return 
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def frameToggle(frameleftAlt,frameLeft,jButtoncenter):
    global altTop
    if altTop:
        frameLeft.tkraise()
        altTop = False
        jButtoncenter["text"] = 'Switch Goal to Work Space'

    else:
        frameleftAlt.tkraise()
        altTop = True
        jButtoncenter["text"] = 'Switch Goal to Joint Space'

def killPorgram(master):
    rospy.signal_shutdown("GUI Closed")
    master.destroy()



def validateJointInput(char, joint):
    # print("joint " + joint + " entry set to " + char)
    try:
        if(char is "-"):
            return True
        if len(char) > 0:
            float(char)
        return True
    except Exception:
        return False


    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
def jointAngleButtonEnvoke(group,j1,j2,j3,j4,j5,j6):
    print("Joint Angles Submitted")
    group.clear_pose_targets()
    for degrees in (j1,j2,j3,j4,j5,j6):
        if degrees:
            degrees = (float(degrees) * 180)/pi
        else:
            degrees = float(0)
    joint_goal = group.get_current_joint_values()
    joint_goal[0] = float(j1) * (pi/180)
    joint_goal[1] = float(j2) * (pi/180)
    joint_goal[2] = float(j3) * (pi/180)
    joint_goal[3] = float(j4) * (pi/180)
    joint_goal[4] = float(j5) * (pi/180)
    joint_goal[5] = float(j6) * (pi/180)

    group.go(joint_goal, wait=False)

    group.stop()

def targetPoseButtonEnvoke(group,X,Y,Z,rX,rY,rZ):
    print("Target Pose Submitted")
    pose_goal = group.get_current_pose()
    q = quaternion_from_euler(float(rX) * (pi/180),float(rY) * (pi/180),float(rZ) * (pi/180))
    pose_goal.pose.orientation.x = q[0]
    pose_goal.pose.orientation.y = q[1]
    pose_goal.pose.orientation.z = q[2]
    pose_goal.pose.orientation.w = q[3]
    pose_goal.pose.position.x = float(X)
    pose_goal.pose.position.y = float(Y)
    pose_goal.pose.position.z = float(Z)
    group.set_pose_target(pose_goal)
    planSuccess = group.plan()
    group.go(wait=False)

    group.stop()

def currentJointStateCallback(msg,e1,e2,e3,e4,e5,e6,group,e1right,e2right,e3right,e4right,e5right,e6right):
    positions = msg.position
    newPos = list()
    for rad in positions:
        newPos.append(round( rad * (180/pi), 3 ))
    positions = newPos
    e1.config(text= str(positions[0]))
    e2.config(text= str(positions[1]))
    e3.config(text= str(positions[2]))
    e4.config(text= str(positions[3]))
    e5.config(text= str(positions[4]))
    e6.config(text= str(positions[5]))

    currentPoseStateCallback(group,e1right,e2right,e3right,e4right,e5right,e6right)

def currentPoseStateCallback(group,e1,e2,e3,e4,e5,e6):
    pose = group.get_current_pose().pose
    orientation_q = pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    roll    = roll * (180/pi)
    pitch   = pitch * (180/pi)
    yaw     = yaw * (180/pi)
    e1.config(text=str(round(pose.position.x,4)))
    e2.config(text=str(round(pose.position.y,4)))
    e3.config(text=str(round(pose.position.z,4)))
    e4.config(text=str(round(roll,4)))
    e5.config(text=str(round(pitch,4)))
    e6.config(text=str(round(yaw,4)))
    

def posesEqual(pose1,e1,e2,e3,e4,e5,e6):
    if pose1.position.x == e1.get() and pose1.position.y == e2.get() and pose1.position.z == e3.get():
        if pose1.orientation.x == e4.get() and pose1.orientation.y == e5.get() and pose1.orientation.z == e6.get():
            return True
    return False
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass