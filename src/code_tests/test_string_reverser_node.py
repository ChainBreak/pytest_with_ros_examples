import pytest
import rospy
import rosgraph
import subprocess
import time
from pathlib import Path
from ..string_reverser_node import StringReverserNode

from std_msgs.msg import String


@pytest.fixture(scope="module")
def roscore():
    try:
        process = subprocess.Popen("roscore")
        rospy.init_node("test")
        yield
    finally:
        process.terminate()
        process.wait()


@pytest.fixture(scope="module")
def running_string_reverser_node():
    try:
        file_path = Path(__file__).parent.parent/"string_reverser_node.py"
        process = subprocess.Popen(["python" , str(file_path)])
        yield
    finally:
        process.terminate()
        process.wait()


@pytest.fixture()
def string_publisher(forward_string):
    try:
        process = subprocess.Popen(["rostopic", "pub", "-r", "10", "/forward_string", "std_msgs/String", forward_string])
        yield
    finally:
        process.terminate()
        process.wait()


@pytest.fixture()
def string_reverser(): 
    return StringReverserNode()


@pytest.mark.parametrize("forward_string, backward_string",
[
    ("abc","cba"),
    ("12 3","3 21"),
])
def test_reverse_string_msg(forward_string, backward_string, string_reverser):
    forward_string_msg = String(forward_string)
    backward_string_msg = string_reverser.reverse_string_msg(forward_string_msg)
    assert backward_string_msg.data == backward_string, "String not reversed"


@pytest.mark.parametrize("forward_string, backward_string",
[
    ("abc","cba"),
    ("12 3","3 21"),
])
def test_string_reverser_node( backward_string, roscore, string_publisher, running_string_reverser_node):
    backward_string_msg = rospy.wait_for_message("/backward_string",String)
    assert backward_string_msg.data == backward_string, "String not reversed"
  
    

