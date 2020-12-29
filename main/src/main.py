#!/usr/bin/env python3

import rospy
import time
import RPi.GPIO as GPIO
from std_msgs.msg import Float64
import actionlib
import actions.msg
from std_msgs.msg import String


# ------------------- Class ---------------------


# Class Labirinto : Control class of Labirinto
class Labirinto :



    # constructor of Class Labirinto
    def __init__(self):

        self.trajectoryList = [[1, 1, 1]]
        self.noWalls = False
        self.decisionFILO = []

        self.ultrasonicRangeFront = [0]
        self.ultrasonicRangeRight = [0]
        self.ultrasonicRangeLeft = [0]

        self.pubLed = rospy.Publisher("/led", String, queue_size=10)

        self.ultrasonicSuscriberFront = rospy.Subscriber('/ultrasonic/Front', Float64, self.updateRange, ('front'))
        self.ultrasonicSuscriberRight = rospy.Subscriber('/ultrasonic/Right', Float64, self.updateRange, ('right'))
        self.ultrasonicSuscriberLeft = rospy.Subscriber('/ultrasonic/Left', Float64, self.updateRange, ('left'))

        self.turnClient = actionlib.SimpleActionClient('TurnActionNode', actions.msg.turnAction)
        self.straightDriveDistClient = actionlib.SimpleActionClient('StraightDriveDistActionNode', actions.msg.straightDriveDistAction)
        self.straightDriveClient = actionlib.SimpleActionClient('StraightDriveActionNode', actions.msg.straightDriveAction)

        rospy.loginfo("All requests to servers sended")

        self.turnClient.wait_for_server()
        rospy.loginfo("Feedback of turnClient received")
        self.straightDriveDistClient.wait_for_server()
        rospy.loginfo("Feedback of straightDriveDistClient")
        self.straightDriveClient.wait_for_server()

        rospy.loginfo("Feedback of all servers received")

        while len(self.ultrasonicRangeFront) < 5 and len(self.ultrasonicRangeLeft) < 5 and len(self.ultrasonicRangeRight) < 5:
            rospy.loginfo("wait for measures")
            time.sleep(0.1)

    # Callback function for Ultrasonic Suscribers
    def updateRange(self, data, args):
        rospy.loginfo(data)
        if args == 'front':
            self.ultrasonicRangeFront.append(data.data)
            if len(self.ultrasonicRangeFront) > 5:
                del self.ultrasonicRangeFront[0]
        elif args == 'right':
            self.ultrasonicRangeRight.append(data.data)
            if len(self.ultrasonicRangeRight) > 5:
                del self.ultrasonicRangeRight[0]
        elif args == 'left':
            self.ultrasonicRangeLeft.append(data.data)
            if len(self.ultrasonicRangeLeft) > 5:
                del self.ultrasonicRangeLeft[0]
        else:
            print("Error passing arguments")



    # Method : Verification of path with the last five measures
    def pathVerification(self, ultrasonicRangeList):
        averageCount = 0
        for i in ultrasonicRangeList:
            averageCount += i
        average = averageCount / len(ultrasonicRangeList)
        rospy.loginfo(average)

        if average > 20:
            return True
        else:
            return False



    # Method : if no sensors detect wall, verify if it's an intersection or the finish of the Maze
    def noWallVerification(self):

        # action Forward
        print('Forward')
        self.actionCalling('forward')
        self.addTrajectory()
        if self.trajectoryList[-1] == [True, True, True]:
            self.trajectoryList[-1].append(1)
            print('noWalllls')
            return True
        else :
            return False



    # Method : add path possibilities to the Trajectory list
    def addTrajectory(self):
        self.trajectoryList.append([self.pathVerification(self.ultrasonicRangeLeft), self.pathVerification(self.ultrasonicRangeFront), self.pathVerification(self.ultrasonicRangeRight)])



    # Method : closing all non explored intersections for the back trajectory
    def closeIntersection(self):
        print('closing all intersection for return to Start')
        for i, elt in enumerate(self.trajectoryList):
            decision = elt[3]
            self.trajectoryList[i] = [False, False, False]
            self.trajectoryList[i][decision] = True
            self.trajectoryList[i].append(decision)



    # Method : deciding next movement from the Trajectory list
    def pathDecision(self, pathPossibilites, normalDrive, returnToStart = False):

        if pathPossibilites == [True, True, True] and not returnToStart:
            self.trajectoryList[-1].append(1)
            if self.noWallVerification():
                return 'noWalls'
            else:
                return self.pathDecision(self.trajectoryList[-1], True)
        elif pathPossibilites[1]:
            if normalDrive:
                if len(self.trajectoryList[-1]) > 3:
                    self.trajectoryList[-1][3] = 1
                else:
                    self.trajectoryList[-1].append(1)
            return 'forward'
        elif pathPossibilites[2]:
            if normalDrive:
                if len(self.trajectoryList[-1]) > 3:
                    self.trajectoryList[-1][3] = 2
                else:
                    self.trajectoryList[-1].append(2)
            return 'right'
        elif pathPossibilites[0]:
            if normalDrive:
                if len(self.trajectoryList[-1]) > 3:
                    self.trajectoryList[-1][3] = 0
                else:
                    self.trajectoryList[-1].append(0)
            return 'left'
        elif pathPossibilites == [False, False, False]:
            return 'noPath'
        else:
            return 'error'



    # Method : Action calling and controlling
    def actionCalling(self, movement):
        if movement == 'forward':
            # action forward with ultrasonic
            goal = actions.msg.straightDriveDistGoal(distance = 20)
            self.straightDriveDistClient.send_goal_and_wait(goal)

            goal = actions.msg.straightDriveGoal()
            self.straightDriveClient.send_goal_and_wait(goal)
           #  while not self.straightDriveClient.wait_for_result():
                #if self.pathVerification(self.ultrasonicRangeLeft) or self.pathVerification(self.ultrasonicRangeRight):
                    #verify drived distance since the path detection

            goal = actions.msg.straightDriveDistGoal(distance = 20)
            self.straightDriveDistClient.send_goal_and_wait(goal)

        elif movement == 'right':
            # action turn right 90°
            goal = actions.msg.turnGoal(turn_angle = 90)
            self.turnClient.send_goal_and_wait(goal)
            # action forward with ultrasonic
            self.actionCalling('forward')
        elif movement == 'left':
            # action turn left 90°
            goal = actions.msg.turnGoal(turn_angle = -90)
            self.turnClient.send_goal_and_wait(goal)
            # action forward with ultrasonic
            self.actionCalling('forward')
        elif movement == 'UTurn':
            self.pubLed.publish('blinkFast')
            # action turn 180°
            goal = actions.msg.turnGoal(turn_angle = 180)
            self.turnClient.send_goal_and_wait(goal)
            self.actionCalling('forward')
            # action forward with ultrasonic



    # Method : Normal Drive, maze exploring
    def normalDrive(self):
        self.pubLed.publish('solid')
        rospy.loginfo("Normal Drive")
        intersection = False
        while  not rospy.is_shutdown():
            if not intersection:
                self.addTrajectory()
            rospy.loginfo(self.pathDecision(self.trajectoryList[-1], True))
            nextMovement = self.pathDecision(self.trajectoryList[-1], True)
            if nextMovement == 'noPath':
                intersection = self.invertedDrive(False)
            elif nextMovement == 'noWalls':
                self.closeIntersection()
                self.invertedDrive(True)
            else :
                print(nextMovement)
                self.actionCalling(nextMovement)
                intersection = False



    # Method : inverted Drive, follow the explored Trajectory back, until intersection or until Start Point
    def invertedDrive(self, returnToStart):
        self.pubLed.publish('blinkSlow')
        rospy.loginfo("Inverted Drive")
        if returnToStart:
            # action U Turn
            print('Uturn')
            self.actionCalling('UTurn')
            # action Forward
            print('Forward')
            self.actionCalling('forward')
            del self.trajectoryList[-1]
        else:
            # action U Turn
            print('UTurn')
            self.actionCalling('Uturn')
            del self.trajectoryList[-1]

        # loop until Start if no intersection occur
        while len(self.trajectoryList) > 0:
            lastDecision = self.trajectoryList[-1][3]
            lastMovement = [self.trajectoryList[-1][2], self.trajectoryList[-1][1], self.trajectoryList[-1][0]]
            self.trajectoryList[-1][lastDecision] = False
            lastPathPossibilities = list(self.trajectoryList[-1])
            del lastPathPossibilities[3]
            intersection = self.pathDecision(lastPathPossibilities, False)

            if returnToStart:
                print('Return to start')
                movement = self.pathDecision(lastMovement, False, True)
                # action F R L
                print(movement)
                self.actionCalling(movement)
                del self.trajectoryList[-1]
            elif intersection == 'noPath':
                movement = self.pathDecision(lastMovement, False)
                # action F R L
                print(movement)
                self.actionCalling(movement)
                del self.trajectoryList[-1]
            else:
                print("action to initial pose")
                # action go initial Pose
                if lastDecision == 0:
                    goal = actions.msg.turnGoal(turn_angle = -90)
                    self.turnClient.send_goal_and_wait(goal)
                elif lastDecision == 1:
                    goal = actions.msg.turnGoal(turn_angle = 180)
                    self.turnClient.send_goal_and_wait(goal)
                elif lastDecision == 2:
                    goal = actions.msg.turnGoal(turn_angle = 90)
                    self.turnClient.send_goal_and_wait(goal)
                #return to normalDrive
                break

        return True



# ------------------- Main ---------------------




if __name__ == '__main__':
    try:

        # initialization Node
        rospy.init_node('main', anonymous=True)	

        labirinto = Labirinto()
        labirinto.normalDrive()

    except rospy.ROSInterruptException:
        pass
    finally:
        GPIO.cleanup()
