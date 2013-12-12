#!/usr/bin/env python2

import os
from gi.repository import GLib, Gdk, Gtk, GdkPixbuf
import rospy
import std_srvs.srv
import std_msgs.msg
import sensor_msgs.msg
import coffee_machine_control.srv
import voice_control.srv
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

        self.pub_say = rospy.Publisher('/voice_control/say', std_msgs.msg.String)
        self.pub_voice_commands = rospy.Publisher('/voice_control/commands', std_msgs.msg.String)
        self.pub_wit_override = rospy.Publisher('/voice_control/wit_override', std_msgs.msg.String)

        rospy.Subscriber('/voice_control/speech', std_msgs.msg.String, self.voice_control_last_heard_callback, queue_size=1)

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

    def webcam_event_box_button_press_event_cb(self, *args):
        if self.subscribing_to_webcam:
            self.subscribing_to_webcam = False
            self.sub_webcam.unregister()
            pixbuf = self.builder.get_object("webcam_feed").get_pixbuf()
            if pixbuf:
                pixbuf.saturate_and_pixelate(pixbuf, saturation=1, pixelate=True)
                self.builder.get_object("webcam_feed").set_from_pixbuf(pixbuf)
        else:
            self.initialise_webcam_feed()


    def update_webcam_feed(self):
        if self.subscribing_to_webcam:
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
                self.builder.get_object("webcam_feed").set_from_pixbuf(self.cam_pixbuf)
        return True

    def initialise_webcam_feed(self):
        self.cvBridge = cv_bridge.CvBridge()
        self.ros_frame = None
        topic = "/user_identification/video"
        self.sub_webcam = rospy.Subscriber(topic, sensor_msgs.msg.Image, self.ros_camera_callback, queue_size=1)
        self.subscribing_to_webcam = True

    def interaction_start_button_clicked_cb(self, *args):
        self.send_service_command('/voice_control/start')

    def interaction_end_button_clicked_cb(self, *args):
        self.pub_wit_override.publish("{'intent': 'good_bye'}")

    def interaction_pause_button_clicked_cb(self, *args):
        self.pub_voice_commands.publish("pause")

    def interaction_resume_button_clicked_cb(self, *args):
        self.pub_voice_commands.publish("resume")

    def voice_recording_start_clicked_cb(self, *args):
        self.send_service_command('/voice_control/recording_start')
        
    def voice_recording_stop_clicked_cb(self, *args):
        self.send_service_command('/voice_control/recording_stop')

    def robot_say_textbox_activate_cb(self, *args):
        textbox = self.builder.get_object('robot_say_textbox')
        text = textbox.get_text()
        self.pub_say.publish(text)

    def user_override_hello_button_clicked_cb(self, *args):
        self.pub_wit_override.publish("{'intent': 'hello'}")

    def user_override_yes_button_clicked_cb(self, *args):
        self.pub_wit_override.publish("{'intent': 'affirmative'}")

    def user_override_no_button_clicked_cb(self, *args):
        self.pub_wit_override.publish("{'intent': 'negative'}")

    def user_override_espresso_button_clicked_cb(self, *args):
        self.pub_wit_override.publish("{'intent': 'request', 'entities': {'Coffee': {'value' : 'espresso'}}}")

    def user_override_caramel_button_clicked_cb(self, *args):
        self.pub_wit_override.publish("{'intent': 'request', 'entities': {'Coffee': {'value' : 'caramel'}}}")

    def user_override_vanilla_button_clicked_cb(self, *args):
        self.pub_wit_override.publish("{'intent': 'request', 'entities': {'Coffee': {'value' : 'vanilla'}}}")

    def user_override_mocha_button_clicked_cb(self, *args):
        self.pub_wit_override.publish("{'intent': 'request', 'entities': {'Coffee': {'value' : 'mocha'}}}")

    def user_override_name_textbox_activate_cb(self, *args):
        name = self.builder.get_object("user_override_name_textbox").get_text()
        self.pub_wit_override.publish("{'intent': 'name', 'entities': {'contact': {'value' : '" + name + "'}}}")

    def voice_control_last_heard_callback(self, msg):
        last_heard_view = self.builder.get_object("last_heard_body")
        last_heard_view.set_text(msg.data)

    def window_key_press_event_cb(self, widget, event):
        key = Gdk.keyval_name(event.keyval)
        if key == 'r':
            self.voice_recording_start_clicked_cb()
        elif key == 't':
            self.voice_recording_stop_clicked_cb()

    def send_service_command(self, service_name, srv_type=std_srvs.srv.Empty):
        try:
            return rospy.ServiceProxy(service_name, srv_type)()
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

        status = self.send_service_command('/voice_control/interaction_status', voice_control.srv.interaction_status)
        if status:
            interaction_status_view = self.builder.get_object("interaction_status_body")
            if status.is_active:
                status_str = "Interacting"
                if status.level != -1:
                    status_str += "\nLevel: " + str(status.level)
                if status.user_id != -1:
                    status_str += "\nUser ID: " + str(status.user_id)
                if status.user_name != '':
                    status_str += "\nName: " + status.user_name
            else:
                status_str = "Idle"
            interaction_status_view.set_text(status_str)
        return True
    

if __name__ == '__main__':
    import signal
    def kill_handler(*args):
        Gtk.main_quit()
    signal.signal(signal.SIGINT, kill_handler)

    rospy.init_node('barista_bot_control_gui')
    MainWindow()
    Gtk.main()
