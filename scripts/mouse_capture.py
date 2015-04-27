#!/usr/bin/env python
import sys
import rospy
from PySide import QtGui, QtCore
from sensor_msgs.msg import PointCloud2
from sensor_msgs.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Empty
import numpy as np


class MouseCapture(QtGui.QLabel):
    def __init__(self, frame_id, scale, offset, *args, **kwargs):
        super(MouseCapture, self).__init__(*args, **kwargs)
        self.setMouseTracking(True)
        self.frame_id = frame_id
        self.scale = scale
        self.offset = np.array(offset)

        self.point_pub = rospy.Publisher('intersected_points', PointCloud2)
        self.click_pub = rospy.Publisher('click', Empty)

    def mouseMoveEvent(self, e):
        pos = np.array(e.pos().toTuple())
        pos[1] = self.size().height() - pos[1]  # make +y up
        pos_world = pos * self.scale + self.offset
        cloud = create_cloud_xyz32(
            rospy.Header(frame_id=self.frame_id, stamp=rospy.Time.now()),
            np.array([[
                pos_world[0],
                pos_world[1],
                0
            ]]))

        self.point_pub.publish(cloud)

    def mousePressEvent(self, e):
        self.click_pub.publish()

    def maybeQuit(self):
        if rospy.is_shutdown():
            self.showNormal()
            QtGui.QApplication.quit()
            return True
        return False


if __name__ == '__main__':
    argv = rospy.myargv()
    app = QtGui.QApplication(argv)
    t = QtCore.QTimer()
    rospy.init_node('mouse_to_point')
    frame_id = rospy.get_param('~frame_id')
    offset_x = rospy.get_param('~offset_x', 0)
    offset_y = rospy.get_param('~offset_y', 0)
    scale = rospy.get_param('~scale', 0.001)
    ss = MouseCapture(frame_id, scale, (offset_x, offset_y))
    t.timeout.connect(ss.maybeQuit)
    t.start(500)
    ss.showFullScreen()
    sys.exit(app.exec_())
