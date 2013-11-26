#!/usr/bin/env python

import roslib; roslib.load_manifest('global_control')

import rospy

from std_msgs.msg import String
from voice_control.srv import *
from coffee_machine_control.srv import *

class GlobalControl:
    
    def __init__(self):
        
        self.our_packages = ('movement', 'user_identification', 'voice_control')
        self.pub_movement_commands = rospy.Publisher('/movement/control', String)
        self.pub_voice_commands = rospy.Publisher('/voice_control/commands', String)
        self.pub_user_id_commands = rospy.Publisher('/user_identification/commands', String)
        self.pub_say = rospy.Publisher('/voice_control/say', String)

    def say(self, args):

        print 'Robot says "' + args[4:] + '"'
        self.pub_say.publish(args[4:])

        return

    def pause(self, args):
        package = args[6:]
        if package == 'movement':
            self.pub_movement_commands.publish('pause')
        elif package == 'voice_control':
            self.pub_voice_commands.publish('pause')
        elif package == 'user_identification':
            self.pub_uder_id_commands = rospy.publish('pause')
        else:
            print package + ' is not a package we control'

        return

    def resume(self, args):
        package = args[7:]
        if package == 'movement':
            self.pub_movement_commands.publish('resume')
        elif package == 'voice_control':
            self.pub_voice_commands.publish('resume')
        elif package == 'user_identification':
            self.pub_uder_id_commands = rospy.publish('resume')
        else:
            print package + ' is not a package we control'

        return

    def restart(self, args):
        package = args[8:]
        if package == 'movement':
            self.pub_movement_commands.publish('restart')
        elif package == 'voice_control':
            self.pub_voice_commands.publish('restart')
        elif package == 'user_identification':
            self.pub_uder_id_commands = rospy.publish('restart')
        else:
            print package + ' is not a package we control'

        return
    
    def stop(self, args):
        package = args[5:]
        if package == 'movement' or package == '':
            self.pub_movement_commands.publish('stop')
        elif package == 'voice_control':
            self.pub_voice_commands.publish('stop')
        elif package == 'user_identification':
            self.pub_uder_id_commands = rospy.publish('stop')
        else:
            print package + ' is not a package we control'

        return

    def serve(self, args):
        coffee = args[6:]
        print 'Serving coffee "' + coffee + '"'
        rospy.wait_for_service('coffee_machine')
        srv = rospy.ServiceProxy('coffee_machine', coffee_machine)
        try:
            resp = srv(coffee)
            print resp
        except rospy.ServiceExecution, e:
            print 'Service call failed: ' + e

    def interact(self, args):
        rospy.wait_for_service('voice_control')
        srv = rospy.ServiceProxy('voice_control', voice_control)
        try:
            success = srv()
            print 'Voice control called: ' + str(success)
        except:
            print 'No response from voice control'

    def runner(self):
        
        while not rospy.is_shutdown():
            #wait around for callbacks

            command = raw_input('>> ')

            words = command.split()
            print words

            try:
                callMethod = getattr(self, words[0])
                callMethod(command)
            except AttributeError:
                print 'Sorry, "' + words[0] + '" is not a valid command'
                 

if __name__ == '__main__':
    rospy.init_node('global_control')
    control = GlobalControl()
    control.runner()
