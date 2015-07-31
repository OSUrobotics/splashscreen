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
        self.initedWithMsg = len(msg) > 0
        rospy.init_node('splashscreen')
        self.startupTime = rospy.Time.now()
        rospy.Subscriber('click', Empty, self.clickCb)
        rospy.Subscriber('~message', String, self.updateMessageCb)
        self.advance_pub = rospy.Publisher('~advance', Empty, queue_size=1)

        self.setFont(QtGui.QFont('Arial', 42))
        self.setAlignment(QtCore.Qt.AlignCenter)

        self.updateMessage.connect(self.doUpdateMessage)
        self.hideScreen.connect(self.mousePressEvent)

    def updateMessageCb(self, msg):
        if not self.initedWithMsg or (rospy.Time.now() - self.startupTime).to_sec() > 0.1:
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
        if (rospy.Time.now() - self.startupTime).to_sec() > 0.1:
            self.showNormal()
            self.hide()
            self.advance_pub.publish()

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
    message = argv[1] if len(argv) > 1 else ''
    ss = Splashscreen(message)
    t.timeout.connect(ss.maybeQuit)
    t.start(500)
    if len(message) > 0:
        ss.showFullScreen()
    else:
        ss.showMinimized()
    sys.exit(app.exec_())
