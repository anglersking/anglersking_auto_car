#!/usr/bin/env python
#autocar
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
 
 
class Follower:
  def __init__(self):
    self.bridge = cv_bridge.CvBridge()
    self.image_sub = rospy.Subscriber('/camera', Image, self.image_callback)
    self.image_pub = rospy.Publisher('/image_hsv', Image, queue_size=2)
    self.cmd_vel_pub = rospy.Publisher('/smoother_cmd_vel',Twist, queue_size=1)
    self.twist = Twist()


    self.forward_velocity = rospy.get_param('~forward_velocity', 0.2)
    self.scale_diversion = rospy.get_param('~scale_diversion', 0.4)

 
  def image_callback(self, msg):
    image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_yellow = numpy.array([ 0,  0,  0])
    upper_yellow = numpy.array([180, 255, 46])
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

    # BEGIN CROP
    h, w, d = image.shape
    search_top = 3*h/4
    search_bot = search_top + 20
    mask[0:search_top, 0:w] = 0
    mask[search_bot:h, 0:w] = 0
    # END CROP
    # BEGIN FINDER
    M = cv2.moments(mask)
    if M['m00'] > 0:
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
        vtherror = cx - w/2
        print cx,cy
        self.twist.linear.x = self.forward_velocity
        self.twist.angular.z = -float(vtherror) / 300 * self.scale_diversion   # 400: 0.1, 300: 0.15, 250, 0.2
        self.cmd_vel_pub.publish(self.twist)
    else:
        print 'not found line'
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        self.cmd_vel_pub.publish(self.twist)


    hsv_image = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
    self.image_pub.publish(hsv_image)


 
rospy.init_node('follower')
follower = Follower()
rospy.spin()
