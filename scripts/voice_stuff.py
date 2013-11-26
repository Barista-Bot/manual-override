#add in voice_control_server.py
def commandCallback(data):
    if data = 'pause':
        pause = 1 #GLOBAL?
        while pause:
            pass
    elif data = 'resume':
        pause = 0
    elif data = 'restart':
        #what do?
        pass
    elif data = 'stop'
        #exit?
        rospy.logwarn('Received STOP, exiting now')
        exit()
    else:
        pass

def sayCallback(data):
    googleTTS(data)


#at the start somewhere
rospy.Subscribe('~say', String, sayCallback)
pub_speech = rospy.Publisher('~speech', String)

#line 56
pub_speech.publish(hypothesis)
