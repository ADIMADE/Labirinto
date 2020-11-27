#import rospy
import time
#import RPi.GPIO as GPIO
#from std_msgs.msg import Float64


# ------------------- Class ---------------------


# Class Labirinto : Control class of Labirinto
class Labirinto:

    # constructor of Class Labirinto
    def __init__(self):

        self.trajectoryList = []
        self.noWalls = False
        self.decisionFILO = []

        self.ultrasonicRangeFront = []
        self.ultrasonicRangeRight = []
        self.ultrasonicRangeLeft = []

        # self.ultrasonicSuscriberFront = rospy.Subscriber('/ultrasonic/front', Float64, self.updateRange, ('front'))
        # self.ultrasonicSuscriberRight = rospy.Subscriber('/ultrasonic/right', Float64, self.updateRange, ('right'))
        # self.ultrasonicSuscriberLeft = rospy.Subscriber('/ultrasonic/left', Float64, self.updateRange, ('left'))

    # Callback function for Ultrasonic Suscribers
    # def updateRange(self, data, args):
    #     if args == 'front':
    #         self.ultrasonicRangeFront.append(data)
    #         if len(self.ultrasonicRangeFront[0]) > 5:
    #             del self.ultrasonicRangeFront[0]
    #     elif args == 'right':
    #         self.ultrasonicRangeRight.append(data)
    #         if len(self.ultrasonicRangeRight[0]) > 5:
    #             del self.ultrasonicRangeRight[0]
    #     elif args == 'left':
    #         self.ultrasonicRangeLeft.append(data)
    #         if len(self.ultrasonicRangeLeft[0] > 5):
    #             del self.ultrasonicRangeLeft[0]
    #     else:
    #         print("Error passing arguments")

    # Method : Verification of path with the last five measures
    def pathVerification(self, ultrasonicRangeList):
        # average = 0
        # for i in ultrasonicRangeList:
        #     average += i
        # average / len(ultrasonicRangeList)

        if input('chemin possible oui non ?') == 't':
            return True
        else:
            return False


    # Method : if no sensors detect wall, verify if it's an intersection or the finish of the Maze
    def noWallVerification(self):

        # action Forward
        print('Forward')
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
            print("movement forward without ultrasonic")
            print("movement forward with ultrasonic")

            print("waiting on intersection")
        elif movement == 'right':
            # action turn right 90°
            print("movement turning right")

            # action forward with ultrasonic
            print("movement forward without then with ultrasonic")
        elif movement == 'left':
            # action turn left 90°
            print("movement turning left")

            # action forward with ultrasonic
            print("movement forward without then with ultrasonic")
        elif movement == 'UTurn':
            # action turn 180°
            print("movement turning 180°")

            print("movement forward without then with ultrasonic")
            # action forward with ultrasonic

    # Method : Normal Drive, maze exploring
    def normalDrive(self):
        intersection = False
        while True:
            if not intersection:
                self.addTrajectory()
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
                time.sleep(1)


    # Method : inverted Drive, follow the explored Trajectory back, until intersection or until Start Point
    def invertedDrive(self, returnToStart):
        if returnToStart:
            # action U Turn
            print('Uturn')
            self.actionCalling('UTurn')
            # action Forward
            print('Forward')
            self.actionCalling('forward')
            del self.trajectoryList[-1]
        else:
            del self.trajectoryList[-1]
            time.sleep(1)
            # action U Turn
            print('UTurn')
            self.actionCalling('Uturn')

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
                print(movement)
                self.actionCalling(movement)
                # action F R L
                del self.trajectoryList[-1]
                time.sleep(1)
            elif intersection == 'noPath':
                movement = self.pathDecision(lastMovement, False)
                print(movement)
                self.actionCalling(movement)
                # action F R L
                del self.trajectoryList[-1]
                time.sleep(1)
            else:
                #-----TO WORK ON-------------
                print("action to initial pose")
                # action go initial Pose
                if lastDecision == 0:
                    print("movement turning left")
                elif lastDecision == 1:
                    print("movement turning right")
                elif lastDecision == 2:
                    print("movement turning 180°")
                time.sleep(1)
                break

        return True



# ------------------- Main ---------------------


if __name__ == '__main__':
    try:

        labirinto = Labirinto()
        labirinto.normalDrive()

    finally:
        print('finish')
