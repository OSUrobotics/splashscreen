#!/usr/bin/env python
import sys
import rospy
from PySide import QtGui, QtCore
from std_msgs.msg import Empty, String


class Splashscreen(QtGui.QLabel):
    hideScreen = QtCore.Signal(int)
    updateMessage = QtCore.Signal(str)

    def __init__(self, msg, *args, **kwargs):

        super(Splashscreen, self).__init__(msg, *args, **kwargs)
        rospy.init_node('splashscreen')
        rospy.Subscriber('click', Empty, self.clickCb)
        rospy.Subscriber('~message', String, self.updateMessageCb)
        self.advance_pub = rospy.Publisher('~advance', Empty)

        self.setFont(QtGui.QFont('Arial', 42))
        self.setAlignment(QtCore.Qt.AlignCenter)

        self.updateMessage.connect(self.doUpdateMessage)
        self.hideScreen.connect(self.mousePressEvent)

    def updateMessageCb(self, msg):
        self.updateMessage.emit(msg.data)

    def doUpdateMessage(self, msg):
        self.setText(msg)
        self.showFullScreen()

    def clickCb(self, msg):
        self.hideScreen.emit(0)

    def mousePressEvent(self, e):
        self.advance()

    def keyPressEvent(self, e):
        # quit on escape or space
        if (e.key() == 16777216) or (e.key() == 32):
            self.advance()

    def advance(self):
        self.hide()
        self.advance_pub.publish()

if __name__ == '__main__':
    argv = rospy.myargv()
    app = QtGui.QApplication(argv)
    t = QtCore.QTimer()
    t.timeout.connect(lambda: rospy.is_shutdown() and QtGui.QApplication.quit())
    t.start(500)
    message = argv[1] if len(argv) > 1 else ''
    ss = Splashscreen(message)
    if len(message) > 0:
        ss.showFullScreen()
    sys.exit(app.exec_())
