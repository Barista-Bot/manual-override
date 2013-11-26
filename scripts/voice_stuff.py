#add in voice_control_server.py
def commandCallback(data):
if data.data == 'pause':
    rospy.loginfo('PAUSING')
    pause = 1
        
elif data.data == 'resume':
    rospy.loginfo('RESUMING')
    pause = 0
elif data.data == 'restart':
    rospy.loginfo('RESTARTING')
    pause = 0
    restart = 1
elif data.data == 'stop':
    rospy.signal_shutdown('STOP received, exiting immediately')
    exit()
else:
    rospy.loginfo('Ignoring invalid ' + data.data + 'command')
    pass

def sayCallback(data):
    googleTTS(data)


#at the start somewhere
rospy.Subscribe('~say', String, sayCallback)
pub_speech = rospy.Publisher('~speech', String)

#line 56
pub_speech.publish(hypothesis)
