#!/usr/bin/env python
from copy import deepcopy
from time import sleep
import rospy

import Tkinter as tk
from tkFileDialog import askopenfilename
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point32
import moveit_commander
import sys
from math import pi
from threading import Thread, Event
from std_srvs.srv import Empty
from tf.transformations import euler_from_quaternion, quaternion_from_euler

j1Angle = 0
j2Angle = 1
j3Angle = 2
j4Angle = 3
j5Angle = 4
j6Angle = 5
group_name = "manipulator"
group = None
altTop = False
toggleButtonState = True
eoatPub = None
mmmFilePath = ""
EXECUTE_FILE_FLAG = Event()
eoatState = 'none'

# gui constants
READOUT_BACKROUND_COLOR = '#3c3c3c' #Backround color of a value readout
MAIN_BACKROUND_COLOR = '#000000'    #Backround color of the main window
ENTRY_BACKROUND_COLOR = "#9c9c9c"   #backround color of an editable field
BUTTON_BACKROUND_COLOR = "#437bba"  #normal backround color of the button
BUTTON_CLICKED_COLOR = "#234266"    #color of the button while clicked on

MAIN_FONT_COLOR = '#ececec'         #Main font color
READOUT_FONT_COLOR = "#fcfcfc"      #Readout font color
BUTTON_FONT_COLOR = "#121212"       #Font Color for buttons
ENTRY_FONT_COLOR = "#121212"        #Font color for entry fields
TITLE_FONT_COLOR = '#93c0f5'        #Font color for title

READOUT_WIDTH = 25                  #pixel width of a value readout
ENTRY_WIDTH = 25                    #pixel width of a editable field
LABEL_WIDTH = 15                    #the width of a label
SQUARE_BUTTON_SIZE = 15             #small button size (the stepping buttons)
TEXT_AREA_HEIGHT = 10               #Hight of the text display
TEXT_AREA_WIDTH = 40                #width of the text display

def main():
    #setup ros
    rospy.init_node('joint_publisher_gui')
    rate = rospy.Rate(60)
    # initialize move it planner
    moveit_commander.roscpp_initialize(sys.argv)
    global group
    group = moveit_commander.MoveGroupCommander(group_name)

    #setup Tkinter (calling it tk from her on out)
    mainTk = tk.Tk()
    mainTk.title("MMM Simple Sender")
    mainTk.config(bg=MAIN_BACKROUND_COLOR)
    pixelVirtual = tk.PhotoImage(width=1, height=1) #for making the buttons in pixel sizes
    #make tk text entry validator
    validNumberStringValidator = mainTk.register(validateJointInput)
    leftMainFrame = tk.Frame(mainTk)
    leftMainFrame.config(bg=MAIN_BACKROUND_COLOR)
    leftMainFrame.grid(column=0,row=0, sticky="nsew")

    curWorkspacePoseFrame = tk.Frame(mainTk)
    curWorkspacePoseFrame.config(bg=MAIN_BACKROUND_COLOR)
    curWorkspacePoseFrame.grid(column=2,row=0, sticky="nsew")

    switchSendingFrame = tk.Frame(mainTk)
    switchSendingFrame.config(bg=MAIN_BACKROUND_COLOR)
    switchSendingFrame.grid(column=0,row=1, sticky="nsew")

    # STEPPING CONTROLS FRAME
    mainJoystickFrame = tk.Frame(leftMainFrame,bg=MAIN_BACKROUND_COLOR)
    mainJoystickFrame.grid(column=0,row=1,pady=10)

    # Pose stepping frame
    #for holding the label 
    jsTableTitle = tk.Label(mainJoystickFrame,text="Step Pose (m)", bg=MAIN_BACKROUND_COLOR, fg=TITLE_FONT_COLOR)
    jsTableTitle.config(font=("Courier", 15, "bold"))
    jsTableTitle.grid(column=0,row=0,padx=(0,20))

    #for holding all the butttons
    joystickFrame=tk.Frame(mainJoystickFrame,bg=MAIN_BACKROUND_COLOR)
    joystickFrame.grid(column=0,row=2)

    #for holding the step size
    joystickStepSizeFrame=tk.Frame(mainJoystickFrame,bg=MAIN_BACKROUND_COLOR)
    joystickStepSizeFrame.grid(column=0,row=1,pady=5)

    joystickStepSizeLabel = tk.Label(joystickStepSizeFrame,text="Move Distance (m)",bg=MAIN_BACKROUND_COLOR,fg=MAIN_FONT_COLOR)
    joystickStepSizeEntry = tk.Entry(joystickStepSizeFrame, validate="key", validatecommand=(validNumberStringValidator, '%P', 1), justify="center",bd=0,highlightthickness=0,bg=ENTRY_BACKROUND_COLOR,fg=ENTRY_FONT_COLOR)
    joystickStepSizeLabel.grid(column=0,row=0)
    joystickStepSizeEntry.grid(column=0,row=1)
    joystickStepSizeEntry.insert(-1,"0.01")

    #im sorry, this is when i realized I should have made a class structure for each element used
    joyStickHome = tk.Button(joystickFrame, text=u"\u2302",bg=BUTTON_BACKROUND_COLOR,fg=BUTTON_FONT_COLOR, command= lambda: jointAngleButtonEvoke(group,0,0,0,0,0,0))
    joyStickHome.config(activebackground=BUTTON_CLICKED_COLOR,highlightbackground=MAIN_BACKROUND_COLOR,bd=0, height=SQUARE_BUTTON_SIZE, width=SQUARE_BUTTON_SIZE, compound="c", image=pixelVirtual)
    joyStickUp = tk.Button(joystickFrame, text=u"\u2197",bg=BUTTON_BACKROUND_COLOR,fg=BUTTON_FONT_COLOR, command= lambda:stepCurrentPose(group, joystickStepSizeEntry.get(), 'Z'))
    joyStickUp.config(activebackground=BUTTON_CLICKED_COLOR,highlightbackground=MAIN_BACKROUND_COLOR,bd=0, height=SQUARE_BUTTON_SIZE, width=SQUARE_BUTTON_SIZE , compound="c", image=pixelVirtual)
    joyStickDown = tk.Button(joystickFrame, text=u"\u2199",bg=BUTTON_BACKROUND_COLOR,fg=BUTTON_FONT_COLOR, command= lambda:stepCurrentPose(group, joystickStepSizeEntry.get(), '-Z'))
    joyStickDown.config(activebackground=BUTTON_CLICKED_COLOR,highlightbackground=MAIN_BACKROUND_COLOR,bd=0, height=SQUARE_BUTTON_SIZE, width=SQUARE_BUTTON_SIZE , compound="c", image=pixelVirtual)
    joyStickForward = tk.Button(joystickFrame, text=u"\u2191",bg=BUTTON_BACKROUND_COLOR,fg=BUTTON_FONT_COLOR, command= lambda:stepCurrentPose(group, joystickStepSizeEntry.get(), 'X'))
    joyStickForward.config(activebackground=BUTTON_CLICKED_COLOR,highlightbackground=MAIN_BACKROUND_COLOR,bd=0, height=SQUARE_BUTTON_SIZE, width=SQUARE_BUTTON_SIZE , compound="c", image=pixelVirtual)
    joyStickBackward = tk.Button(joystickFrame, text=u"\u2193",bg=BUTTON_BACKROUND_COLOR,fg=BUTTON_FONT_COLOR, command= lambda:stepCurrentPose(group, joystickStepSizeEntry.get(), '-X'))
    joyStickBackward.config(activebackground=BUTTON_CLICKED_COLOR,highlightbackground=MAIN_BACKROUND_COLOR,bd=0, height=SQUARE_BUTTON_SIZE, width=SQUARE_BUTTON_SIZE , compound="c", image=pixelVirtual)
    joyStickRight = tk.Button(joystickFrame, text=u"\u2192",bg=BUTTON_BACKROUND_COLOR,fg=BUTTON_FONT_COLOR, command= lambda:stepCurrentPose(group, joystickStepSizeEntry.get(), '-Y'))
    joyStickRight.config(activebackground=BUTTON_CLICKED_COLOR,highlightbackground=MAIN_BACKROUND_COLOR,bd=0, height=SQUARE_BUTTON_SIZE, width=SQUARE_BUTTON_SIZE , compound="c", image=pixelVirtual)
    joyStickLeft = tk.Button(joystickFrame, text=u"\u2190",bg=BUTTON_BACKROUND_COLOR,fg=BUTTON_FONT_COLOR, command= lambda:stepCurrentPose(group, joystickStepSizeEntry.get(), 'Y'))
    joyStickLeft.config(activebackground=BUTTON_CLICKED_COLOR,highlightbackground=MAIN_BACKROUND_COLOR,bd=0, height=SQUARE_BUTTON_SIZE, width=SQUARE_BUTTON_SIZE , compound="c", image=pixelVirtual)

    joyStickHome.grid(column=1,row=1)
    joyStickUp.grid(column=2,row=0)
    joyStickDown.grid(column=0,row=2)
    joyStickForward.grid(column=1,row=0)
    joyStickBackward.grid(column=1,row=2)
    joyStickRight.grid(column=2,row=1)
    joyStickLeft.grid(column=0,row=1)

    # Rotation Step Controller frame
    #for holding the label 
    jsTableTitle = tk.Label(mainJoystickFrame,text="Step Angle (deg)", bg=MAIN_BACKROUND_COLOR, fg=TITLE_FONT_COLOR)
    jsTableTitle.config(font=("Courier", 15, "bold"))
    jsTableTitle.grid(column=1,row=0,pady=5)

    #for holding all the buttons
    rotationFrame=tk.Frame(mainJoystickFrame, bg=MAIN_BACKROUND_COLOR)
    rotationFrame.grid(column=1,row=2)

    #for holding the step size entry
    rotationStepSizeFrame=tk.Frame(mainJoystickFrame, bg=MAIN_BACKROUND_COLOR)
    rotationStepSizeFrame.grid(column=1,row=1)

    rotationStepSizeLabel = tk.Label(rotationStepSizeFrame,text="Rotation Step (degrees)", bg=MAIN_BACKROUND_COLOR, fg=MAIN_FONT_COLOR)
    rotationStepSizeEntry = tk.Entry(rotationStepSizeFrame, validate="key", validatecommand=(validNumberStringValidator, '%P', 1),bd=0,highlightthickness=0, justify="center",bg=ENTRY_BACKROUND_COLOR,fg=ENTRY_FONT_COLOR)
    rotationStepSizeLabel.grid(column=0,row=0)
    rotationStepSizeEntry.grid(column=0,row=1)
    rotationStepSizeEntry.insert(-1,"1.0")

    rotationStickHome = tk.Button(rotationFrame, text=u"\u2302",bg=BUTTON_BACKROUND_COLOR,fg=BUTTON_FONT_COLOR, command= lambda: jointAngleButtonEvoke(group,0,0,0,0,0,0))
    rotationStickHome.config(activebackground=BUTTON_CLICKED_COLOR,highlightbackground=MAIN_BACKROUND_COLOR,bd=0, height=SQUARE_BUTTON_SIZE, width=SQUARE_BUTTON_SIZE, compound="c", image=pixelVirtual)
    rotationStickUp = tk.Button(rotationFrame, text=u"\u21B0",bg=BUTTON_BACKROUND_COLOR,fg=BUTTON_FONT_COLOR, command= lambda:stepCurrentRotation(group, rotationStepSizeEntry.get(), 'Roll'))
    rotationStickUp.config(activebackground=BUTTON_CLICKED_COLOR,highlightbackground=MAIN_BACKROUND_COLOR,bd=0, height=SQUARE_BUTTON_SIZE, width=SQUARE_BUTTON_SIZE, compound="c", image=pixelVirtual)
    rotationStickDown = tk.Button(rotationFrame, text=u"\u21B3",bg=BUTTON_BACKROUND_COLOR,fg=BUTTON_FONT_COLOR, command= lambda:stepCurrentRotation(group, rotationStepSizeEntry.get(), '-Roll'))
    rotationStickDown.config(activebackground=BUTTON_CLICKED_COLOR,highlightbackground=MAIN_BACKROUND_COLOR,bd=0, height=SQUARE_BUTTON_SIZE, width=SQUARE_BUTTON_SIZE, compound="c", image=pixelVirtual)
    rotationStickForward = tk.Button(rotationFrame, text=u"\u293A",bg=BUTTON_BACKROUND_COLOR,fg=BUTTON_FONT_COLOR, command= lambda:stepCurrentRotation(group, rotationStepSizeEntry.get(), 'Pitch'))
    rotationStickForward.config(activebackground=BUTTON_CLICKED_COLOR,highlightbackground=MAIN_BACKROUND_COLOR,bd=0, height=SQUARE_BUTTON_SIZE, width=SQUARE_BUTTON_SIZE, compound="c", image=pixelVirtual)
    rotationStickBackward = tk.Button(rotationFrame, text=u"\u293B",bg=BUTTON_BACKROUND_COLOR,fg=BUTTON_FONT_COLOR, command= lambda:stepCurrentRotation(group, rotationStepSizeEntry.get(), '-Pitch'))
    rotationStickBackward.config(activebackground=BUTTON_CLICKED_COLOR,highlightbackground=MAIN_BACKROUND_COLOR,bd=0, height=SQUARE_BUTTON_SIZE, width=SQUARE_BUTTON_SIZE, compound="c", image=pixelVirtual)
    rotationStickRight = tk.Button(rotationFrame, text=u"\u2938",bg=BUTTON_BACKROUND_COLOR,fg=BUTTON_FONT_COLOR, command= lambda:stepCurrentRotation(group, rotationStepSizeEntry.get(), 'Yaw'))
    rotationStickRight.config(activebackground=BUTTON_CLICKED_COLOR,highlightbackground=MAIN_BACKROUND_COLOR,bd=0, height=SQUARE_BUTTON_SIZE, width=SQUARE_BUTTON_SIZE, compound="c", image=pixelVirtual)
    rotationStickLeft = tk.Button(rotationFrame, text=u"\u2939",bg=BUTTON_BACKROUND_COLOR,fg=BUTTON_FONT_COLOR, command= lambda:stepCurrentRotation(group, rotationStepSizeEntry.get(), '-Yaw'))
    rotationStickLeft.config(activebackground=BUTTON_CLICKED_COLOR,highlightbackground=MAIN_BACKROUND_COLOR,bd=0, height=SQUARE_BUTTON_SIZE, width=SQUARE_BUTTON_SIZE, compound="c", image=pixelVirtual)
    
    rotationStickHome.grid(column=1,row=1)
    rotationStickUp.grid(column=2,row=0)
    rotationStickDown.grid(column=0,row=2)
    rotationStickForward.grid(column=1,row=0)
    rotationStickBackward.grid(column=1,row=2)
    rotationStickRight.grid(column=2,row=1)
    rotationStickLeft.grid(column=0,row=1)

    # Joint Space sender Frame
    # create tk frame
    jointSpacePoseFrame = tk.Frame(leftMainFrame)
    jointSpacePoseFrame.config(bg=MAIN_BACKROUND_COLOR)
    jointSpacePoseFrame.grid(column=0,row=0, sticky="nsew")
    #make title
    titleLabelLeft = tk.Label(jointSpacePoseFrame,text="Send Joint Targets", bg=MAIN_BACKROUND_COLOR,fg=TITLE_FONT_COLOR)
    titleLabelLeft.grid(pady = 5,column=0,row=0)
    titleLabelLeft.config(font=("Courier", 15, "bold"))
    #make frame to hold the input table
    jointTableFrame = tk.Frame(jointSpacePoseFrame, bg=MAIN_BACKROUND_COLOR)
    jointTableFrame.grid(column=0,row=1,pady=5)

    # Create all the labels
    l1 = tk.Label(jointTableFrame,text="Joint 1", bg=MAIN_BACKROUND_COLOR,fg=MAIN_FONT_COLOR,width=LABEL_WIDTH,anchor="e")
    l2 = tk.Label(jointTableFrame,text="Joint 2", bg=MAIN_BACKROUND_COLOR,fg=MAIN_FONT_COLOR,width=LABEL_WIDTH,anchor="e")
    l3 = tk.Label(jointTableFrame,text="Joint 3", bg=MAIN_BACKROUND_COLOR,fg=MAIN_FONT_COLOR,width=LABEL_WIDTH,anchor="e")
    l4 = tk.Label(jointTableFrame,text="Joint 4", bg=MAIN_BACKROUND_COLOR,fg=MAIN_FONT_COLOR,width=LABEL_WIDTH,anchor="e")
    l5 = tk.Label(jointTableFrame,text="Joint 5", bg=MAIN_BACKROUND_COLOR,fg=MAIN_FONT_COLOR,width=LABEL_WIDTH,anchor="e")
    l6 = tk.Label(jointTableFrame,text="Joint 6", bg=MAIN_BACKROUND_COLOR,fg=MAIN_FONT_COLOR,width=LABEL_WIDTH,anchor="e")
    #create all the entry's and attach validator
    e1 = tk.Entry(jointTableFrame, validate="key", validatecommand=(validNumberStringValidator, '%P', 1), justify="center",bd=0,highlightthickness=0,bg=ENTRY_BACKROUND_COLOR,fg=ENTRY_FONT_COLOR,width=ENTRY_WIDTH)
    e2 = tk.Entry(jointTableFrame, validate="key", validatecommand=(validNumberStringValidator, '%P', 2), justify="center",bd=0,highlightthickness=0,bg=ENTRY_BACKROUND_COLOR,fg=ENTRY_FONT_COLOR,width=ENTRY_WIDTH)
    e3 = tk.Entry(jointTableFrame, validate="key", validatecommand=(validNumberStringValidator, '%P', 3), justify="center",bd=0,highlightthickness=0,bg=ENTRY_BACKROUND_COLOR,fg=ENTRY_FONT_COLOR,width=ENTRY_WIDTH)
    e4 = tk.Entry(jointTableFrame, validate="key", validatecommand=(validNumberStringValidator, '%P', 4), justify="center",bd=0,highlightthickness=0,bg=ENTRY_BACKROUND_COLOR,fg=ENTRY_FONT_COLOR,width=ENTRY_WIDTH)
    e5 = tk.Entry(jointTableFrame, validate="key", validatecommand=(validNumberStringValidator, '%P', 5), justify="center",bd=0,highlightthickness=0,bg=ENTRY_BACKROUND_COLOR,fg=ENTRY_FONT_COLOR,width=ENTRY_WIDTH)
    e6 = tk.Entry(jointTableFrame, validate="key", validatecommand=(validNumberStringValidator, '%P', 6), justify="center",bd=0,highlightthickness=0,bg=ENTRY_BACKROUND_COLOR,fg=ENTRY_FONT_COLOR,width=ENTRY_WIDTH)
    #populate the table
    l1.grid(column=0,row=1)
    e1.grid(column=1,row=1)
    l2.grid(column=0,row=2)
    e2.grid(column=1,row=2)
    l3.grid(column=0,row=3)
    e3.grid(column=1,row=3)
    l4.grid(column=0,row=4)
    e4.grid(column=1,row=4)
    l5.grid(column=0,row=5)
    e5.grid(column=1,row=5)
    l6.grid(column=0,row=6)
    e6.grid(column=1,row=6)
    #set the initial value of the entry's
    e1.insert(-1,"0")
    e2.insert(-1,"0")
    e3.insert(-1,"0")
    e4.insert(-1,"0")
    e5.insert(-1,"0")
    e6.insert(-1,"0")

    jButtonAngle = tk.Button(jointTableFrame, 
                        text="Send Target", 
                        command= lambda: jointAngleButtonEvoke(group,e1.get(),e2.get(),e3.get(),e4.get(),e5.get(),e6.get()),
                        bg=BUTTON_BACKROUND_COLOR,fg=BUTTON_FONT_COLOR,activebackground=BUTTON_CLICKED_COLOR,highlightbackground=MAIN_BACKROUND_COLOR,bd=0
                        )
    jButtonAngle.grid(column=0,row=7)
    switchToPose = tk.Button(jointTableFrame, 
                        text="Switch To Sending Pose Goals", 
                        command= lambda: frameToggle(workSpacePoseFrame,jointSpacePoseFrame, current="ANGLE"),
                        bg=BUTTON_BACKROUND_COLOR,fg=BUTTON_FONT_COLOR,activebackground=BUTTON_CLICKED_COLOR,highlightbackground=MAIN_BACKROUND_COLOR,bd=0
                        )
    switchToPose.grid(column=1,row=7)


    # Work space target spender frame 
    workSpacePoseFrame = tk.Frame(leftMainFrame)
    workSpacePoseFrame.config(bg=MAIN_BACKROUND_COLOR)
    workSpacePoseFrame.grid(column=0,row=0, sticky="nsew")
    #make title
    titleLabelLeftalt = tk.Label(workSpacePoseFrame,text="Send Work Space Targets", bg=MAIN_BACKROUND_COLOR,fg=TITLE_FONT_COLOR)
    titleLabelLeftalt.grid(pady = 5,column=0,row=0)
    titleLabelLeftalt.config(font=("Courier", 15, "bold"))
    #make frame to hold the input table
    poseEntryTableFrame = tk.Frame(workSpacePoseFrame, bg=MAIN_BACKROUND_COLOR)
    poseEntryTableFrame.grid(column=0,row=1,pady=5)
    #make the lables
    l1alt = tk.Label(poseEntryTableFrame,text="Position X",bg=MAIN_BACKROUND_COLOR,fg=MAIN_FONT_COLOR,width=LABEL_WIDTH,anchor="e")
    l2alt = tk.Label(poseEntryTableFrame,text="Position Y",bg=MAIN_BACKROUND_COLOR,fg=MAIN_FONT_COLOR,width=LABEL_WIDTH,anchor="e")
    l3alt = tk.Label(poseEntryTableFrame,text="Position Z",bg=MAIN_BACKROUND_COLOR,fg=MAIN_FONT_COLOR,width=LABEL_WIDTH,anchor="e")
    l4alt = tk.Label(poseEntryTableFrame,text="Rotation X",bg=MAIN_BACKROUND_COLOR,fg=MAIN_FONT_COLOR,width=LABEL_WIDTH,anchor="e")
    l5alt = tk.Label(poseEntryTableFrame,text="Rotation Y",bg=MAIN_BACKROUND_COLOR,fg=MAIN_FONT_COLOR,width=LABEL_WIDTH,anchor="e")
    l6alt = tk.Label(poseEntryTableFrame,text="Rotation Z",bg=MAIN_BACKROUND_COLOR,fg=MAIN_FONT_COLOR,width=LABEL_WIDTH,anchor="e")

    e1alt = tk.Entry(poseEntryTableFrame, validate="key", validatecommand=(validNumberStringValidator, '%P', "X"), justify="center",bd=0,highlightthickness=0,bg=ENTRY_BACKROUND_COLOR,fg=ENTRY_FONT_COLOR,width=ENTRY_WIDTH)
    e2alt = tk.Entry(poseEntryTableFrame, validate="key", validatecommand=(validNumberStringValidator, '%P', "Y"), justify="center",bd=0,highlightthickness=0,bg=ENTRY_BACKROUND_COLOR,fg=ENTRY_FONT_COLOR,width=ENTRY_WIDTH)
    e3alt = tk.Entry(poseEntryTableFrame, validate="key", validatecommand=(validNumberStringValidator, '%P', "Z"), justify="center",bd=0,highlightthickness=0,bg=ENTRY_BACKROUND_COLOR,fg=ENTRY_FONT_COLOR,width=ENTRY_WIDTH)
    e4alt = tk.Entry(poseEntryTableFrame, validate="key", validatecommand=(validNumberStringValidator, '%P', "rX"), justify="center",bd=0,highlightthickness=0,bg=ENTRY_BACKROUND_COLOR,fg=ENTRY_FONT_COLOR,width=ENTRY_WIDTH)
    e5alt = tk.Entry(poseEntryTableFrame, validate="key", validatecommand=(validNumberStringValidator, '%P', "rY"), justify="center",bd=0,highlightthickness=0,bg=ENTRY_BACKROUND_COLOR,fg=ENTRY_FONT_COLOR,width=ENTRY_WIDTH)
    e6alt = tk.Entry(poseEntryTableFrame, validate="key", validatecommand=(validNumberStringValidator, '%P', "rZ"), justify="center",bd=0,highlightthickness=0,bg=ENTRY_BACKROUND_COLOR,fg=ENTRY_FONT_COLOR,width=ENTRY_WIDTH)

    l1alt.grid(column=0,row=1)
    e1alt.grid(column=1,row=1)
    l2alt.grid(column=0,row=2)
    e2alt.grid(column=1,row=2)
    l3alt.grid(column=0,row=3)
    e3alt.grid(column=1,row=3)
    l4alt.grid(column=0,row=4)
    e4alt.grid(column=1,row=4)
    l5alt.grid(column=0,row=5)
    e5alt.grid(column=1,row=5)
    l6alt.grid(column=0,row=6)
    e6alt.grid(column=1,row=6)

    e1alt.insert(-1,"0.815")
    e2alt.insert(-1,"0")
    e3alt.insert(-1,"1.185")
    e4alt.insert(-1,"0")
    e5alt.insert(-1,"90")
    e6alt.insert(-1,"0")

    jButtonPose = tk.Button(poseEntryTableFrame, 
                        text="Send Target Position", 
                        command= lambda: targetPoseButtonEvoke(group,e1alt.get(),e2alt.get(),e3alt.get(),e4alt.get(),e5alt.get(),e6alt.get()),
                        bg=BUTTON_BACKROUND_COLOR,fg=BUTTON_FONT_COLOR,activebackground=BUTTON_CLICKED_COLOR,highlightbackground=MAIN_BACKROUND_COLOR,bd=0
                        )
    jButtonPose.grid(column=0,row=7)
    switchToAngle = tk.Button(poseEntryTableFrame, 
                        text="Switch To Sending Joint Goals", 
                        command= lambda: frameToggle(workSpacePoseFrame,jointSpacePoseFrame, current="POSE"),
                        bg=BUTTON_BACKROUND_COLOR,fg=BUTTON_FONT_COLOR,activebackground=BUTTON_CLICKED_COLOR,highlightbackground=MAIN_BACKROUND_COLOR,bd=0
                        )
    switchToAngle.grid(column=1,row=7)

    # Current Robot Values Frame
    # current joint values
    curRobotInfoFrame = tk.Frame(mainTk)
    curRobotInfoFrame.config(bg=MAIN_BACKROUND_COLOR)
    curRobotInfoFrame.grid(column=1,row=0, sticky="nsew")

    titleLabelcenter = tk.Label(curRobotInfoFrame,text="Current Joint Angles",justify="right",anchor="e",bg=MAIN_BACKROUND_COLOR,fg=TITLE_FONT_COLOR)
    titleLabelcenter.grid(pady = (10,20),padx = (10,10),column=0,row=0)
    titleLabelcenter.config(font=("Courier", 15, "bold"))

    jaTableFrame = tk.Frame(curRobotInfoFrame, bg=MAIN_BACKROUND_COLOR)
    jaTableFrame.grid(column=0,row=1)

    jaTableLabelA = tk.Label(jaTableFrame,text="Joint #", bg=MAIN_BACKROUND_COLOR, fg=TITLE_FONT_COLOR,anchor="e",width=LABEL_WIDTH)
    l1center = tk.Label(jaTableFrame,text="Joint 1", bg=MAIN_BACKROUND_COLOR, fg=MAIN_FONT_COLOR,width=LABEL_WIDTH,anchor="e")
    l2center = tk.Label(jaTableFrame,text="Joint 2", bg=MAIN_BACKROUND_COLOR, fg=MAIN_FONT_COLOR,width=LABEL_WIDTH,anchor="e")
    l3center = tk.Label(jaTableFrame,text="Joint 3", bg=MAIN_BACKROUND_COLOR, fg=MAIN_FONT_COLOR,width=LABEL_WIDTH,anchor="e")
    l4center = tk.Label(jaTableFrame,text="Joint 4", bg=MAIN_BACKROUND_COLOR, fg=MAIN_FONT_COLOR,width=LABEL_WIDTH,anchor="e")
    l5center = tk.Label(jaTableFrame,text="Joint 5", bg=MAIN_BACKROUND_COLOR, fg=MAIN_FONT_COLOR,width=LABEL_WIDTH,anchor="e")
    l6center = tk.Label(jaTableFrame,text="Joint 6", bg=MAIN_BACKROUND_COLOR, fg=MAIN_FONT_COLOR,width=LABEL_WIDTH,anchor="e")

    e1center = tk.Label(jaTableFrame, borderwidth=1, relief="solid", width=READOUT_WIDTH,bg=READOUT_BACKROUND_COLOR,fg=READOUT_FONT_COLOR)
    e2center = tk.Label(jaTableFrame, borderwidth=1, relief="solid", width=READOUT_WIDTH,bg=READOUT_BACKROUND_COLOR,fg=READOUT_FONT_COLOR)
    e3center = tk.Label(jaTableFrame, borderwidth=1, relief="solid", width=READOUT_WIDTH,bg=READOUT_BACKROUND_COLOR,fg=READOUT_FONT_COLOR)
    e4center = tk.Label(jaTableFrame, borderwidth=1, relief="solid", width=READOUT_WIDTH,bg=READOUT_BACKROUND_COLOR,fg=READOUT_FONT_COLOR)
    e5center = tk.Label(jaTableFrame, borderwidth=1, relief="solid", width=READOUT_WIDTH,bg=READOUT_BACKROUND_COLOR,fg=READOUT_FONT_COLOR)
    e6center = tk.Label(jaTableFrame, borderwidth=1, relief="solid", width=READOUT_WIDTH,bg=READOUT_BACKROUND_COLOR,fg=READOUT_FONT_COLOR)

    jaTableLabelB = tk.Label(jaTableFrame,text="Joint Value (deg)", bg=MAIN_BACKROUND_COLOR, fg=TITLE_FONT_COLOR,anchor="e",width=LABEL_WIDTH)

    jaTableLabelA.grid(column=0,row=0)
    jaTableLabelB.grid(column=1,row=0)
    l1center.grid(column=0,row=1)
    e1center.grid(column=1,row=1)
    l2center.grid(column=0,row=2)
    e2center.grid(column=1,row=2)
    l3center.grid(column=0,row=3)
    e3center.grid(column=1,row=3)
    l4center.grid(column=0,row=4)
    e4center.grid(column=1,row=4)
    l5center.grid(column=0,row=5)
    e5center.grid(column=1,row=5)
    l6center.grid(column=0,row=6)
    e6center.grid(column=1,row=6)

    # Work Space pose
    titleLabelright = tk.Label(curRobotInfoFrame,text="Current Workspace Pose", bg=MAIN_BACKROUND_COLOR, fg=TITLE_FONT_COLOR,anchor='e')
    titleLabelright.grid(pady = (20,5),padx = (10,10),column=0,row=2)
    titleLabelright.config(font=("Courier", 15, "bold"))

    poseTableFrame = tk.Frame(curRobotInfoFrame, bg=MAIN_BACKROUND_COLOR)
    poseTableFrame.grid(column=0,row=4)

    l1right = tk.Label(poseTableFrame,text="Position X", bg=MAIN_BACKROUND_COLOR, fg=MAIN_FONT_COLOR,width=LABEL_WIDTH,anchor="e")
    l2right = tk.Label(poseTableFrame,text="Position Y", bg=MAIN_BACKROUND_COLOR, fg=MAIN_FONT_COLOR,width=LABEL_WIDTH,anchor="e")
    l3right = tk.Label(poseTableFrame,text="Position Z", bg=MAIN_BACKROUND_COLOR, fg=MAIN_FONT_COLOR,width=LABEL_WIDTH,anchor="e")
    l4right = tk.Label(poseTableFrame,text="Rotation X", bg=MAIN_BACKROUND_COLOR, fg=MAIN_FONT_COLOR,width=LABEL_WIDTH,anchor="e")
    l5right = tk.Label(poseTableFrame,text="Rotation Y", bg=MAIN_BACKROUND_COLOR, fg=MAIN_FONT_COLOR,width=LABEL_WIDTH,anchor="e")
    l6right = tk.Label(poseTableFrame,text="Rotation Z", bg=MAIN_BACKROUND_COLOR, fg=MAIN_FONT_COLOR,width=LABEL_WIDTH,anchor="e")

    e1right = tk.Label(poseTableFrame, borderwidth=1, relief="solid", width=READOUT_WIDTH,bg=READOUT_BACKROUND_COLOR,fg=READOUT_FONT_COLOR)
    e2right = tk.Label(poseTableFrame, borderwidth=1, relief="solid", width=READOUT_WIDTH,bg=READOUT_BACKROUND_COLOR,fg=READOUT_FONT_COLOR)
    e3right = tk.Label(poseTableFrame, borderwidth=1, relief="solid", width=READOUT_WIDTH,bg=READOUT_BACKROUND_COLOR,fg=READOUT_FONT_COLOR)
    e4right = tk.Label(poseTableFrame, borderwidth=1, relief="solid", width=READOUT_WIDTH,bg=READOUT_BACKROUND_COLOR,fg=READOUT_FONT_COLOR)
    e5right = tk.Label(poseTableFrame, borderwidth=1, relief="solid", width=READOUT_WIDTH,bg=READOUT_BACKROUND_COLOR,fg=READOUT_FONT_COLOR)
    e6right = tk.Label(poseTableFrame, borderwidth=1, relief="solid", width=READOUT_WIDTH,bg=READOUT_BACKROUND_COLOR,fg=READOUT_FONT_COLOR)

    poseTableLabelA = tk.Label(poseTableFrame,text="Axis", bg=MAIN_BACKROUND_COLOR, fg=TITLE_FONT_COLOR,width=LABEL_WIDTH,anchor="e")
    poseTableLabelB = tk.Label(poseTableFrame,text="Value (deg/m)", bg=MAIN_BACKROUND_COLOR, fg=TITLE_FONT_COLOR,width=LABEL_WIDTH,anchor="e")

    poseTableLabelA.grid(column=0,row=0)
    poseTableLabelB.grid(column=1,row=0)
    l1right.grid(column=0,row=1)
    e1right.grid(column=1,row=1)
    l2right.grid(column=0,row=2)
    e2right.grid(column=1,row=2)
    l3right.grid(column=0,row=3)
    e3right.grid(column=1,row=3)
    l4right.grid(column=0,row=4)
    e4right.grid(column=1,row=4)
    l5right.grid(column=0,row=5)
    e5right.grid(column=1,row=5)
    l6right.grid(column=0,row=6)
    e6right.grid(column=1,row=6)

    #end of arm display

    # end of arm tooling control display
    eoatFrame = tk.Frame(mainTk, bg=MAIN_BACKROUND_COLOR)
    eoatFrame.config(bg=MAIN_BACKROUND_COLOR)
    eoatFrame.grid(column=3,row=0, sticky="nsew")
    #title
    eoatTitle = tk.Label(eoatFrame,text="End of Arm Tooling", bg=MAIN_BACKROUND_COLOR, fg=TITLE_FONT_COLOR,anchor='e')
    eoatTitle.grid(column=0,row=0)
    eoatTitle.config(font=("Courier", 15, "bold"))

    #frame for holding pose table
    eoatPoseFrame = tk.Frame(eoatFrame, bg=MAIN_BACKROUND_COLOR)
    eoatPoseFrame.grid(column=0,row=2)
    #title for pose input frame
    eoatTitle = tk.Label(eoatFrame,text="Current Fingers Pose", bg=MAIN_BACKROUND_COLOR, fg=TITLE_FONT_COLOR)
    eoatTitle.grid(column=0,row=1)
    eoatTitle.config(font=("Courier", 10, "bold"))

    # table labels for eoat table
    eoatPoseTableLabel =  tk.Label(eoatPoseFrame, text="mm from center", borderwidth=0, relief="solid", width=LABEL_WIDTH,bg=MAIN_BACKROUND_COLOR,fg=MAIN_FONT_COLOR)
    eoatPoseTableLabel.grid(column=1,row=0)
    # current pose read out of eoat
    tk.Label(eoatPoseFrame,text="Left", bg=MAIN_BACKROUND_COLOR, fg=MAIN_FONT_COLOR, width=LABEL_WIDTH,anchor="e").grid(column=0,row=1)
    tk.Label(eoatPoseFrame,text="Right", bg=MAIN_BACKROUND_COLOR, fg=MAIN_FONT_COLOR, width=LABEL_WIDTH,anchor="e").grid(column=0,row=2)

    eoatLeftFingerValue = tk.Label(eoatPoseFrame, text="No pose received", borderwidth=1, relief="solid", width=READOUT_WIDTH,bg=READOUT_BACKROUND_COLOR,fg=READOUT_FONT_COLOR)
    eoatRightFingerValue = tk.Label(eoatPoseFrame, text="No pose received", borderwidth=1, relief="solid", width=READOUT_WIDTH,bg=READOUT_BACKROUND_COLOR,fg=READOUT_FONT_COLOR)

    eoatLeftFingerValue.grid(column=1,row=1)
    eoatRightFingerValue.grid(column=1,row=2)

    #frame for hold pose input table
    eoatPoseInputFrame = tk.Frame(eoatFrame, bg=MAIN_BACKROUND_COLOR)
    eoatPoseInputFrame.grid(column=0,row=4)
    #title for pose input frame
    eoatTitle = tk.Label(eoatFrame,text="Send Fingers Target Pose", bg=MAIN_BACKROUND_COLOR, fg=TITLE_FONT_COLOR)
    eoatTitle.grid(column=0,row=3)
    eoatTitle.config(font=("Courier", 10, "bold"))

    # table labels for eoat input
    eoatInputTableLabel =  tk.Label(eoatPoseInputFrame, text="mm from center", borderwidth=0, relief="solid", width=LABEL_WIDTH,bg=MAIN_BACKROUND_COLOR,fg=MAIN_FONT_COLOR)
    eoatInputTableLabel.grid(column=1,row=0)
    # set current pose
    tk.Label(eoatPoseInputFrame,text="Left", bg=MAIN_BACKROUND_COLOR, fg=MAIN_FONT_COLOR, width=LABEL_WIDTH,anchor="e").grid(column=0,row=1)
    tk.Label(eoatPoseInputFrame,text="Right", bg=MAIN_BACKROUND_COLOR, fg=MAIN_FONT_COLOR, width=LABEL_WIDTH,anchor="e").grid(column=0,row=2)
    tk.Label(eoatPoseInputFrame,text="Speed", bg=MAIN_BACKROUND_COLOR, fg=MAIN_FONT_COLOR, width=LABEL_WIDTH,anchor="e").grid(column=0,row=3)

    eoatLeftPoseIn = tk.Entry(eoatPoseInputFrame, validate="key", validatecommand=(validNumberStringValidator, '%P', 1),justify="center",bd=0,highlightthickness=0,bg=ENTRY_BACKROUND_COLOR,fg=ENTRY_FONT_COLOR,width=ENTRY_WIDTH)
    eoatRightPoseIn = tk.Entry(eoatPoseInputFrame, validate="key", validatecommand=(validNumberStringValidator, '%P', 1), justify="center",bd=0,highlightthickness=0,bg=ENTRY_BACKROUND_COLOR,fg=ENTRY_FONT_COLOR,width=ENTRY_WIDTH)
    eoatSpeedIn = tk.Entry(eoatPoseInputFrame, validate="key", validatecommand=(validNumberStringValidator, '%P', 1), justify="center",bd=0,highlightthickness=0,bg=ENTRY_BACKROUND_COLOR,fg=ENTRY_FONT_COLOR,width=ENTRY_WIDTH)

    eoatRightPoseIn.insert(-1,"0")
    eoatSpeedIn.insert(-1,"0")
    eoatLeftPoseIn.insert(-1,"0")

    eoatLeftPoseIn.grid(column=1,row=1)
    eoatRightPoseIn.grid(column=1,row=2)
    eoatSpeedIn.grid(column=1,row=3)

    eoatSendPose = tk.Button(eoatPoseInputFrame, 
                        text="Send Target Pose", 
                        command= lambda:  eoatPublisher(float(eoatLeftPoseIn.get()), float(eoatRightPoseIn.get()), float(eoatSpeedIn.get())),
                        bg=BUTTON_BACKROUND_COLOR,fg=BUTTON_FONT_COLOR,activebackground=BUTTON_CLICKED_COLOR,highlightbackground=MAIN_BACKROUND_COLOR,bd=0
                        )
    eoatSendPose.grid(column=1,row=4)

    #frame for hold too offset input table
    eoatOffsetInputFrame = tk.Frame(eoatFrame, bg=MAIN_BACKROUND_COLOR)
    eoatOffsetInputFrame.grid(column=0,row=6)
    #title
    eoatTitle = tk.Label(eoatFrame,text="Tool Offset", bg=MAIN_BACKROUND_COLOR, fg=TITLE_FONT_COLOR)
    eoatTitle.grid(column=0,row=5)
    eoatTitle.config(font=("Courier", 10, "bold"))
    # set tool offset
    tk.Label(eoatOffsetInputFrame, text="mm from center", borderwidth=0, relief="solid", width=LABEL_WIDTH,bg=MAIN_BACKROUND_COLOR,fg=MAIN_FONT_COLOR).grid(column=1,row=0)
    tk.Label(eoatOffsetInputFrame,text="New value", bg=MAIN_BACKROUND_COLOR, fg=MAIN_FONT_COLOR, width=LABEL_WIDTH,anchor="e").grid(column=0,row=2)
    tk.Label(eoatOffsetInputFrame,text="Current Offset", bg=MAIN_BACKROUND_COLOR, fg=MAIN_FONT_COLOR, width=LABEL_WIDTH,anchor="e").grid(column=0,row=1)

    eoatToolOffsetIn = tk.Entry(eoatOffsetInputFrame, justify="center", validate="key", validatecommand=(validNumberStringValidator, '%P', 1),bd=0,highlightthickness=0,bg=ENTRY_BACKROUND_COLOR,fg=ENTRY_FONT_COLOR,width=ENTRY_WIDTH)
    eoatToolOffsetValue = tk.Label(eoatOffsetInputFrame, text="0", borderwidth=1, relief="solid", width=READOUT_WIDTH,bg=READOUT_BACKROUND_COLOR,fg=READOUT_FONT_COLOR)
    eoatToolOffsetValue.grid(column=1,row=1)
    eoatToolOffsetIn.grid(column=1,row=2)
    eoatToolOffsetIn.insert(-1,"0")
    eoatSetOffset = tk.Button(eoatOffsetInputFrame, 
                        text="Set Tool offset", 
                        command= lambda: eoatPublisher(float(eoatToolOffsetIn.get()), float(eoatToolOffsetIn.get()), -111),
                        bg=BUTTON_BACKROUND_COLOR,fg=BUTTON_FONT_COLOR,activebackground=BUTTON_CLICKED_COLOR,highlightbackground=MAIN_BACKROUND_COLOR,bd=0
                        )
    eoatSetOffset.grid(column=1,row=3)

    #frame for hold misc eoat things
    eoatMiscFrame = tk.Frame(eoatFrame, bg=MAIN_BACKROUND_COLOR)
    eoatMiscFrame.grid(column=0,row=8)
    #title
    eoatTitle = tk.Label(eoatFrame,text="MISC. EOAT", bg=MAIN_BACKROUND_COLOR, fg=TITLE_FONT_COLOR)
    eoatTitle.grid(column=0,row=7)
    eoatTitle.config(font=("Courier", 10, "bold"))
    # Status Readout
    tk.Label(eoatMiscFrame,text="Current State", bg=MAIN_BACKROUND_COLOR, fg=MAIN_FONT_COLOR, width=LABEL_WIDTH,anchor="e").grid(column=0,row=0)
    eoatStatusReadout =  tk.Label(eoatMiscFrame,text="EOAT Status: none received",
                        bg=READOUT_BACKROUND_COLOR,width=READOUT_WIDTH,fg=READOUT_FONT_COLOR)
    eoatStatusReadout.grid(column=1,row=0)

    # calabrate
    eoatCalabrate = tk.Button(eoatMiscFrame, 
                        text="Calibrate", 
                        command= lambda: eoatPublisher(0.0, 0.0, -732),
                        bg=BUTTON_BACKROUND_COLOR,fg=BUTTON_FONT_COLOR,activebackground=BUTTON_CLICKED_COLOR,highlightbackground=MAIN_BACKROUND_COLOR,bd=0
                        )
    eoatCalabrate.grid(column=0,row=1)
    # Estop
    eoatEstop = tk.Button(eoatMiscFrame, 
                        text="ESTOP", 
                        command= lambda: eoatPublisher(0.0, 0.0, -911),
                        bg=BUTTON_BACKROUND_COLOR,fg=BUTTON_FONT_COLOR,activebackground=BUTTON_CLICKED_COLOR,highlightbackground=MAIN_BACKROUND_COLOR,bd=0
                        )
    eoatEstop.grid(column=1,row=1)
    #end eoat frame

    #Automatic functionality frame
    #make base frame
    automationFrame = tk.Frame(mainTk, bg=MAIN_BACKROUND_COLOR)
    automationFrame.grid(column=4,row=0,padx=(5,9),sticky="nsew")
    #title
    automationTitle = tk.Label(automationFrame,text="Automated Functions", bg=MAIN_BACKROUND_COLOR, fg=TITLE_FONT_COLOR,anchor='e',font=("Courier", 15, "bold"))
    automationTitle.grid(column=0,row=0)

    #misc buttons frame
    miscButtonsFrame = tk.Frame(automationFrame, bg=MAIN_BACKROUND_COLOR)
    miscButtonsFrame.grid(column=0,row=1, sticky="nsew")
    # start work piece scan button
    toggle_btn = tk.Button(miscButtonsFrame, text="Start Scan", relief="raised",command = lambda: Thread(target=scanButton).start(),
                        bg=BUTTON_BACKROUND_COLOR,fg=BUTTON_FONT_COLOR,activebackground=BUTTON_CLICKED_COLOR,highlightbackground=MAIN_BACKROUND_COLOR,bd=0)
    toggle_btn.grid(column=0,row=0)
    
    #File input frame
    fileInputFrame = tk.Frame(automationFrame, bg=MAIN_BACKROUND_COLOR)
    fileInputFrame.grid(column=0,row=2, sticky="nsew")
    #title for file in
    fileInputTitle = tk.Label(fileInputFrame,text="Commands File", bg=MAIN_BACKROUND_COLOR, fg=TITLE_FONT_COLOR,anchor='e',font=("Courier", 15, "bold"))
    fileInputTitle.grid(column=0,row=0)
    #text area to show loaded file
    fileTextArea = tk.Text(fileInputFrame,height=TEXT_AREA_HEIGHT,width=TEXT_AREA_WIDTH,state=tk.DISABLED,wrap="word")
    fileTextArea.grid(column=0,row=1)
    #file button frame
    fileInputButtonFrame = tk.Frame(fileInputFrame, bg=MAIN_BACKROUND_COLOR)
    fileInputButtonFrame.grid(column=0,row=2, sticky="nsew")
    global mmmFilePath, EXECUTE_FILE_FLAG
    loadFileButton = tk.Button(fileInputButtonFrame, text="Load File", relief="raised", command=lambda: openCommandFile(fileTextArea),
                        bg=BUTTON_BACKROUND_COLOR,fg=BUTTON_FONT_COLOR,activebackground=BUTTON_CLICKED_COLOR,highlightbackground=MAIN_BACKROUND_COLOR,bd=0)
    loadFileButton.grid(column=0,row=0)
    executeFileButton = tk.Button(fileInputButtonFrame, text="Run File", relief="raised", command=lambda: handleFileExecution(),
                        bg=BUTTON_BACKROUND_COLOR,fg=BUTTON_FONT_COLOR,activebackground=BUTTON_CLICKED_COLOR,highlightbackground=MAIN_BACKROUND_COLOR,bd=0)
    executeFileButton.grid(column=1,row=0)
    stopFileButton = tk.Button(fileInputButtonFrame, text="Stop File", relief="raised", command=stopFileExecution,
                        bg=BUTTON_BACKROUND_COLOR,fg=BUTTON_FONT_COLOR,activebackground=BUTTON_CLICKED_COLOR,highlightbackground=MAIN_BACKROUND_COLOR,bd=0)
    stopFileButton.grid(column=2,row=0)


    #make ros publisher(s)
    global toggleButtonState, eoatPub
    eoatPub = rospy.Publisher("mmm_eoat_command", Point32, queue_size=1)
    pub = rospy.Publisher("pointcloudOnOff", Bool, queue_size=10)
    
    #start window
    #prepare a way to cleanly shutdown the window
    rospy.on_shutdown(lambda: mainTk.quit())
    #prepare a way to start the ros subscribers after the blocking code has been run
    mainTk.after(110,lambda:rospy.Subscriber("joint_states", JointState, lambda msg: Thread(currentJointStateCallback(msg, e1center,e2center,e3center,e4center,e5center,e6center,group,e1right,e2right,e3right,e4right,e5right,e6right)).start(), queue_size=1))
    mainTk.after(120,lambda:rospy.Subscriber("mmm_eoat_position", Point32, lambda msg: Thread(eoatCallback(msg,eoatLeftFingerValue,eoatRightFingerValue,eoatStatusReadout,eoatCalabrate,eoatSetOffset,eoatSendPose)).start()))
    # BLOCKING CODE
    # start the tk gui window
    mainTk.mainloop()
    EXECUTE_FILE_FLAG.clear()

def openCommandFile(textArea):
    global mmmFilePath
    mmmFilePath = askopenfilename(initialdir = "~/catkin_ws/src/mmm_mqp_base/commandFiles",title = "Select file",filetypes = (("MMM files","*.mmm"),("all files","*.*")))
    textArea['state']=tk.NORMAL
    textArea.delete(1.0,'end')
    textArea.insert(1.0,"FILEPATH:" + mmmFilePath + "\n####FILESTART####\n" + open(mmmFilePath,'r').read())
    textArea['state']=tk.DISABLED

def stopFileExecution():
    global EXECUTE_FILE_FLAG
    EXECUTE_FILE_FLAG.clear()

def handleFileExecution():
    global mmmFilePath,EXECUTE_FILE_FLAG, group
    EXECUTE_FILE_FLAG.set()
    Thread(target=fileExecutionThread, args=(mmmFilePath,EXECUTE_FILE_FLAG,group)).start()

def fileExecutionThread(mmmFilePath,EXECUTE_FILE_FLAG,group):
    print("Starting File Execution thread, FILE: " + mmmFilePath)
    file = open(mmmFilePath,'r').read().replace('\n','')
    commands = file.split(';')
    while(EXECUTE_FILE_FLAG.isSet()):    
        for i, command in enumerate( commands):
            print(str(i) + " " + command + " | " + str(rospy.get_time()))
            commandArray = command.split(':')
            if (not EXECUTE_FILE_FLAG.isSet()):
                break
            if (commandArray[0] == "POSE"):
                targetPoseButtonEvoke(group,commandArray[1],
                                            commandArray[2],
                                            commandArray[3],
                                            commandArray[4],
                                            commandArray[5],
                                            commandArray[6])
            elif(commandArray[0] == "JOINT"):
                jointAngleButtonEvoke(group,commandArray[1],
                                            commandArray[2],
                                            commandArray[3],
                                            commandArray[4],
                                            commandArray[5],
                                            commandArray[6])
            elif(commandArray[0] == "HOME"):
                jointAngleButtonEvoke(group,0,0,0,0,0,0)
            elif(commandArray[0] == "SCAN"):
                scanButton()
            elif(commandArray[0] == "EOAT"):
                eoatPublisher(float(commandArray[1]),float(commandArray[2]),float(commandArray[3]))
            elif(commandArray[0] == "EOATWAIT"):
                eoatPublisherBlocking(float(commandArray[1]),float(commandArray[2]),float(commandArray[3]))
            elif(commandArray[0] == "WAIT"):
                sleep(float(commandArray[1]))
            elif(commandArray[0] == "STEP"):
                # distance, direction
                stepCurrentPose(group,commandArray[1],commandArray[2])
            elif(commandArray[0] == "ROTATE"):
                stepCurrentRotation(group,commandArray[1],commandArray[2])
            elif(commandArray[0] == "SQUISH"):
                dir = commandArray[1]
                dist = round(float(commandArray[2]),3)
                stepSize = round(float(commandArray[3]),3)
                inPose = float(commandArray[4])
                outPose = float(commandArray[5])
                eoatPublisherBlocking(inPose,inPose,10)
                eoatPublisherBlocking(outPose,outPose,10)
                for i in range(0,int(dist * 1000), int(stepSize * 1000)):
                    stepCurrentPose(group,stepSize,dir)
                    eoatPublisherBlocking(inPose,inPose,10)
                    eoatPublisherBlocking(outPose,outPose,10)
                    if (not EXECUTE_FILE_FLAG.isSet()):
                        break
                else:
                    if not (i / 1000) == dist:
                        stepSize = dist - (float(i) / 1000)
                        stepCurrentPose(group,stepSize,dir)
                        eoatPublisherBlocking(inPose,inPose,10)
                        eoatPublisherBlocking(outPose,outPose,10)
            elif(commandArray[0] == "FLATTEN"):
                dir = commandArray[1]
                dist = round(float(commandArray[2]),3)
                stepSize = round(float(commandArray[3]),3)
                downHeight = float(commandArray[4])
                upHeight = float(commandArray[5])
                stepCurrentPose(group,downHeight,"-Z")
                stepCurrentPose(group,upHeight,"Z")
                for i in range(0,int(dist * 1000), int(stepSize * 1000)):
                    stepCurrentPose(group,stepSize,dir)
                    stepCurrentPose(group,downHeight,"-Z")
                    stepCurrentPose(group,upHeight,"Z")
                    if (not EXECUTE_FILE_FLAG.isSet()):
                        break
                else:
                    if not (i / 1000) == dist:
                        stepRemainder = dist - (float(i) / 1000)
                        stepCurrentPose(group,stepRemainder,dir)
                        stepCurrentPose(group,downHeight,"-Z")
                        stepCurrentPose(group,upHeight,"Z")
        EXECUTE_FILE_FLAG.clear()
    else:
        print("Ending file Execution thread, FILE: " + mmmFilePath)
        return 

def eoatPublisherBlocking(_x, _y, _z):
    global eoatState
    eoatPublisher(_x, _y, _z)
    while eoatState == "Wait":
        sleep(0.01)

# function for calibrating the end of arm tooling
def eoatPublisher(_x, _y, _z):
    global eoatPub
    point = Point32()
    point.x = _x
    point.y = _y
    point.z = _z
    eoatPub.publish(point)

# end of arm tooling callback function
def eoatCallback(msg, leftLabel,rightLabel,statusLabel,calabrateButton,offsetButton,sendPoseButton):
    global eoatState
    leftLabel.config(text=str(msg.x))
    rightLabel.config(text=str(msg.y))
    state = int(msg.z)
    if(state == 0):
        statusLabel.config(text = "Waiting For Instruction")
        calabrateButton['state'] = tk.NORMAL
        offsetButton['state'] = tk.NORMAL
        sendPoseButton['state'] = tk.NORMAL
        eoatState = "Ready"
    else:
        #if the eoat isn't ready for a command disable the buttons
        calabrateButton['state'] = tk.DISABLED
        offsetButton['state'] = tk.DISABLED
        sendPoseButton['state'] = tk.DISABLED
        eoatState = "Wait"
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
        statusLabel.config(text = "Invalid command received")
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

def frameToggle(workSpacePoseFrame,jointSpacePoseFrame,current="NONE"):
    global altTop
    if current == "POSE":
        jointSpacePoseFrame.tkraise()
        altTop = False

    elif current == "ANGLE":
        workSpacePoseFrame.tkraise()
        altTop = True

def killProgram(mainTk):
    rospy.signal_shutdown("GUI Closed")
    mainTk.destroy()

def validateJointInput(STR, joint):
    #STR is the proposed string
    try: # try and make the STR a float and if that fails its proably not a number so don't let it change
        STR = STR.replace("-","")
        STR = STR.replace(".","")
        if len(STR) > 0:
            float(STR)
        return True
    except Exception:
        return False

def jointAngleButtonEvoke(group,j1,j2,j3,j4,j5,j6):
    print("Joint Angles Submitted")
    group.clear_pose_targets()
    #convert the given degrees to rads
    for degrees in (j1,j2,j3,j4,j5,j6):
        if degrees:
            degrees = (float(degrees) * 180)/pi
        else:
            degrees = float(0)
    #set the target goal with the given radian degrees
    joint_goal = group.get_current_joint_values()
    joint_goal[0] = float(j1) * (pi/180)
    joint_goal[1] = float(j2) * (pi/180)
    joint_goal[2] = float(j3) * (pi/180)
    joint_goal[3] = float(j4) * (pi/180)
    joint_goal[4] = float(j5) * (pi/180)
    joint_goal[5] = float(j6) * (pi/180)

    group.go(joint_goal, wait=True)

    group.stop()

def targetPoseButtonEvoke(group,X,Y,Z,rX,rY,rZ):
    group.stop()
    group.clear_pose_targets()
    pose_goal = deepcopy(group.get_current_pose())
    q = quaternion_from_euler(float(rX) * (pi/180),float(rY) * (pi/180),float(rZ) * (pi/180))
    pose_goal.pose.orientation.x = q[0]
    pose_goal.pose.orientation.y = q[1]
    pose_goal.pose.orientation.z = q[2]
    pose_goal.pose.orientation.w = q[3]
    pose_goal.pose.position.x = float(X)
    pose_goal.pose.position.y = float(Y)
    pose_goal.pose.position.z = float(Z)

    (plan, fraction) = group.compute_cartesian_path([group.get_current_pose().pose,pose_goal.pose],0.01,0.0)
    group.execute(plan,wait=True)

def stepCurrentPose(group,deltaString,direction):
    group.stop()
    group.clear_pose_targets()
    delta = float(deltaString)
    pose_goal = deepcopy(group.get_current_pose())
    if (direction.upper() == "X"):
        pose_goal.pose.position.x += delta
    elif (direction.upper() == "Y"):
        pose_goal.pose.position.y += delta
    elif (direction.upper() == "Z"):
        pose_goal.pose.position.z += delta
    elif (direction.upper() == "-X"):
        pose_goal.pose.position.x -= delta
    elif (direction.upper() == "-Y"):
        pose_goal.pose.position.y -= delta
    elif (direction.upper() == "-Z"):
        pose_goal.pose.position.z -= delta

    (plan, fraction) = group.compute_cartesian_path([group.get_current_pose().pose,pose_goal.pose],0.01,0.0)
    group.execute(plan,wait=True)

def stepCurrentRotation(group,deltaString,direction):
    group.stop()
    group.clear_pose_targets()
    print("Target Rotation Step Submitted")

    pose_goal = deepcopy(group.get_current_pose())
    orientation_q = pose_goal.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    roll, pitch, yaw = euler_from_quaternion (orientation_list)
    roll = float(roll) * (180/pi)
    pitch = float(pitch) * (180/pi)
    yaw = float(yaw) * (180/pi)
    delta = float(deltaString)

    # be the change you want to see
    if (direction.upper() == "ROLL"):
        roll += float(delta)
    elif (direction.upper() == "PITCH"):
        pitch += float(delta)
    elif (direction.upper() == "YAW"):
        yaw += float(delta)
    elif (direction.upper() == "-ROLL"):
        roll -= float(delta)
    elif (direction.upper() == "-PITCH"):
        pitch -= float(delta)
    elif (direction.upper() == "-YAW"):
        yaw -= float(delta)
    #set the new values
    q = quaternion_from_euler(roll* (pi/180),pitch* (pi/180),yaw* (pi/180))
    pose_goal.pose.orientation.x = q[0]
    pose_goal.pose.orientation.y = q[1]
    pose_goal.pose.orientation.z = q[2]
    pose_goal.pose.orientation.w = q[3]

    (plan, fraction) = group.compute_cartesian_path([group.get_current_pose().pose,pose_goal.pose],0.01,0.0)
    group.execute(plan,wait=True)

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
    
#unused but intended to check if current pose is equal to pose1
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