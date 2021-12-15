import rospy

from std_msgs.msg import String

def main():
    rospy.init_node("string_reverser_node")

    string_reverser = StringReverser()

    rospy.spin()

class StringReverser():

    def __init__(self):

        self.important_parameter = rospy.get_param("important_parameter")

        self.string_subscriber = rospy.Subscriber("/forward_string", String, callback=self.forward_string_callback, queue_size=10)

        self.string_publisher = rospy.Publisher("/backward_string", String, queue_size=10)

    def forward_string_callback(self,msg):
        msg = self.reverse_string_msg(msg)
        self.string_publisher.publish(msg)
        

    def reverse_string_msg(self,string_msg):
        string_msg.data =  string_msg.data[::-1]
        return string_msg

if __name__ == "__main__":
    main()