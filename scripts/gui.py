#!/usr/bin/env python2

import os
from gi.repository import GLib, Gtk, GdkPixbuf
import rospy
import std_srvs.srv
import sensor_msgs.msg
import time
import cv_bridge
import numpy as np
from PIL import Image
import StringIO

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
        GLib.timeout_add(100, self.update_webcam_feed)

    def initialise_ros_ip_entry(self):
        entry = self.builder.get_object("ros_ip_entry")
        try:
            ros_ip = os.environ['ROS_IP']
        except KeyError:
            ros_ip = "127.0.0.1"
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
            self.cam_pixbuf = self.cam_pixbuf.scale_simple(800, 600, GdkPixbuf.InterpType.BILINEAR)
            self.builder.get_object("webcam_feed").set_from_pixbuf(self.cam_pixbuf)
        return True

    def initialise_webcam_feed(self):
        self.cvBridge = cv_bridge.CvBridge()
        topic = "/usb_cam/image_raw"
        rospy.Subscriber(topic, sensor_msgs.msg.Image, self.ros_camera_callback, queue_size=1)

    def voice_recording_start_clicked_cb(self, *args):
        rospy.ServiceProxy('/voice_control/recording_start', std_srvs.srv.Empty)()
        
    def voice_recording_stop_clicked_cb(self, *args):
        rospy.ServiceProxy('/voice_control/recording_stop', std_srvs.srv.Empty)()

    def on_window_destroy(self, *args):
        Gtk.main_quit(*args)

rospy.init_node('barista_bot_control_gui')
MainWindow()
Gtk.main()