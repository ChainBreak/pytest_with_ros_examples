import rospy

from std_msgs.msg import String

class StringReverserNode():

    def __init__(self):

        self.string_subscriber = rospy.Subscriber("/forward_string",String,callback=self.forward_string_callback,queue_size=10)

        self.string_publisher = rospy.Publisher("/backward_string",String,queue_size=10)

    def forward_string_callback(self,msg):
        msg = self.reverse_string_msg(msg)
        self.string_publisher.publish(msg)
        print("callback",msg)

    def reverse_string_msg(self,string_msg):
        string_msg.data = self.reverse_string( string_msg.data )
        return string_msg

    def reverse_string(self,string):
        return string[::-1]