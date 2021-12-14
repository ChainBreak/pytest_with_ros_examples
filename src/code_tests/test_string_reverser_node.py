import pytest
import rospy
import rosgraph
import subprocess
import time
from ..string_reverser_node import StringReverserNode

from std_msgs.msg import String


@pytest.fixture(scope="module")
def roscore():
    try:
        roscore_process = subprocess.Popen("roscore")
        rospy.init_node("test")
        yield
    finally:
        roscore_process.terminate()
        roscore_process.wait()


@pytest.fixture()
def string_reverser(): 
    return StringReverserNode()


@pytest.fixture()
def string_publisher(): 
    return rospy.Publisher("/forward_string",String,queue_size=10)


@pytest.mark.parametrize("forward, backward",
[
    ("abc","cba"),
    ("12 3","3 21"),
])
def test_reverse_string(forward, backward, string_reverser):
    assert string_reverser.reverse_string(forward) == backward, "String not reversed"


@pytest.mark.parametrize("forward, backward",
[
    ("abc","cba"),
    ("12 3","3 21"),
])
def test_reverse_string_msg(forward, backward, string_reverser):
    forward_string_msg = String(forward)
    backward_string_msg = string_reverser.reverse_string_msg(forward_string_msg)
    assert backward_string_msg.data == backward, "String not reversed"


# @pytest.mark.parametrize("forward, backward",
# [
#     ("abc","cba"),
#     ("12 3","3 21"),
# ])
# def test_string_reverser_node(forward, backward, roscore, string_publisher, string_reverser):

  
#     string_publisher.publish(String(forward))
#     rospy.spin()

#     backward_string_msg = rospy.wait_for_message("/backward_string",String)
#     assert backward_string_msg.data == backward, "String not reversed"

