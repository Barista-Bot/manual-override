#to add in the robot class
def commandCallback(self, data):
    if data = 'pause':
        self.pause = 1
        while self.pause:
            pass
    elif data = 'resume'
        self.pause = 0
    elif data = 'restart'
        self.pause = 0
        self.restart = 1
    elif data = 'stop'
        rospy.logwarn('STOP received, exiting immediately')
        exit()
    else:
        pass

#check for restart in inner loop, exit to outer loop
