#!/usr/bin/env python2

import os
from gi.repository import GLib, Gtk, GdkPixbuf
import rospy
import std_srvs.srv
import std_msgs.msg
import sensor_msgs.msg
import coffee_machine_control.srv
import time
import cv_bridge
import numpy as np
from PIL import Image
import StringIO
import subprocess

class MainWindow(Gtk.Window):

    def __init__(self):
        package_dir = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
        layout_file = os.path.join(package_dir, 'gui_layout', 'barista_bot_control.glade')
        self.builder = Gtk.Builder()
        self.builder.add_from_file(layout_file)
        self.builder.connect_signals(self)
        self.initialise_ros_ip_entry()
        self.initialise_webcam_feed()
        self.builder.get_object("window").show()

        self.pub_say = rospy.Publisher('/voice_control_server/say', std_msgs.msg.String)
        self.pub_voice_commands = rospy.Publisher('/voice_control_server/commands', std_msgs.msg.String)
        self.pub_wit_override = rospy.Publisher('/voice_control_server/wit_override', std_msgs.msg.String)

        rospy.Subscriber('/voice_control_server/speech', std_msgs.msg.String, self.voice_control_last_heard_callback, queue_size=1)

        GLib.timeout_add(100, self.update_webcam_feed)
        GLib.timeout_add_seconds(1, self.spinOnce)
        self.spinOnce()

    def initialise_ros_ip_entry(self):
        entry = self.builder.get_object("ros_master_uri_entry")
        try:
            ros_ip = os.environ['ROS_MASTER_URI']
        except KeyError:
            ros_ip = "NOT SET"
        entry.set_text(ros_ip)

    def ros_camera_callback(self, ros_frame):
        self.ros_frame = ros_frame

    def update_webcam_feed(self):
        ros_frame = self.ros_frame
        if ros_frame is not None:
            np_frame = np.asarray(self.cvBridge.imgmsg_to_cv(ros_frame, "rgb8"))
            image = Image.fromarray(np_frame)
            im_buff = StringIO.StringIO()
            image.save(im_buff, format="ppm")
            ppm_image = im_buff.getvalue()
            im_buff.close()
            loader = GdkPixbuf.PixbufLoader.new_with_type('pnm')
            loader.write(ppm_image)
            self.cam_pixbuf = loader.get_pixbuf()
            loader.close()
            # self.cam_pixbuf = self.cam_pixbuf.scale_simple(800, 600, GdkPixbuf.InterpType.BILINEAR)
            self.builder.get_object("webcam_feed").set_from_pixbuf(self.cam_pixbuf)
        return True

    def initialise_webcam_feed(self):
        self.cvBridge = cv_bridge.CvBridge()
        self.ros_frame = None
        topic = "/user_identification/video"
        rospy.Subscriber(topic, sensor_msgs.msg.Image, self.ros_camera_callback, queue_size=1)

    def voice_recording_start_clicked_cb(self, *args):
        self.send_service_command('/voice_control/recording_start')
        
    def voice_recording_stop_clicked_cb(self, *args):
        self.send_service_command('/voice_control/recording_stop')

    def robot_say_button_clicked_cb(self, *args):
        textbox = self.builder.get_object('robot_say_textbox')
        text = textbox.get_text()
        self.pub_say.publish(text)

    def serve_coffee_button_clicked_cb(self, *args):
        service = rospy.ServiceProxy('coffee_machine', coffee_machine_control.srv.coffee_machine)
        resp = service("espresso")

    def interaction_pause_button_clicked_cb(self, *args):
        self.pub_voice_commands.publish("pause")

    def interaction_resume_button_clicked_cb(self, *args):
        self.pub_voice_commands.publish("resume")

    def wit_override_test(self, *args):
        wit_override_say_goodbye()

    def wit_override_say_goodbye(self, *args):
        self.pub_wit_override.publish("{'intent': 'good_bye'}")

    def wit_override_greet(self, *args):
        self.pub_wit_override.publish("{'intent': 'hello'}")

    def wit_override_order_espresso(self, *args):
        self.pub_wit_override.publish("{'intent': 'request', 'entities': {'Coffee': {'value' : 'espresso'}}}")

    def wit_override_order_caramel_latte(self, *args):
        self.pub_wit_override.publish("{'intent': 'request', 'entities': {'Coffee': {'value' : 'caramel'}}}")

    def wit_override_order_vanilla_latte(self, *args):
        self.pub_wit_override.publish("{'intent': 'request', 'entities': {'Coffee': {'value' : 'vanilla'}}}")

    def wit_override_order_mocha(self, *args):
        self.pub_wit_override.publish("{'intent': 'request', 'entities': {'Coffee': {'value' : 'mocha'}}}")

    def wit_override_intent_affirmation(self, *args):
        self.pub_wit_override.publish("{'intent': 'affirmative'}")

    def wit_override_intent_negation(self, *args):
        self.pub_wit_override.publish("{'intent': 'negative'}")

    def voice_control_last_heard_callback(self, msg):
        last_heard_view = self.builder.get_object("last_heard_body")
        last_heard_view.set_text(msg.data)

    def send_service_command(self, service_name):
        try:
            rospy.ServiceProxy(service_name, std_srvs.srv.Empty)()
        except rospy.service.ServiceException:
            statusbar = self.builder.get_object('statusbar')
            statusbar.push(1, "Warning: Service " + service_name + " is not available.")
            GLib.timeout_add_seconds(1, lambda: statusbar.pop(1))

    def on_window_destroy(self, *args):
        Gtk.main_quit(*args)

    def spinOnce(self, *args):
        active_nodes = subprocess.check_output(["rosnode", "list"])
        active_nodes_view = self.builder.get_object("active_nodes_body")
        active_nodes_view.set_text(active_nodes)
        return True
    

if __name__ == '__main__':
    import signal
    def kill_handler(*args):
        Gtk.main_quit()
    signal.signal(signal.SIGINT, kill_handler)

    rospy.init_node('barista_bot_control_gui')
    MainWindow()
    Gtk.main()
