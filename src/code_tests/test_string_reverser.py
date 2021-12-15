import pytest
import rospy
import rosgraph
import subprocess

from pathlib import Path
from ..string_reverser import StringReverser

from std_msgs.msg import String

### FIXTURES START HERE ###

@pytest.fixture()
def string_reverser(roscore,ros_params): 
    return StringReverser()


@pytest.fixture(scope="module")
def string_reverser_node(roscore, ros_params):
    """
    Starts a string reverser node in a background process.
    Stops it once the tests are done.
    """
    try:
        file_path = Path(__file__).parent.parent/"string_reverser.py"
        process = subprocess.Popen(["python" , str(file_path)])
        yield
    finally:
        process.terminate()
        process.wait()


@pytest.fixture(scope="module")
def ros_params(roscore): 
    rospy.set_param("important_parameter",42)


@pytest.fixture()
def string_publisher(forward_string):
    """
    Starts a rostopic publisher in a background process.
    It publishes the forward_string at a fixed rate.
    Stops it once the tests are done.
    """
    try:
        process = subprocess.Popen(["rostopic", "pub", "-r", "10", "/forward_string", "std_msgs/String", forward_string])
        yield
    finally:
        process.terminate()
        process.wait()

@pytest.fixture(scope="module")
def init_node(roscore):
    rospy.init_node("test")



### TESTS START HERE ####

@pytest.mark.parametrize("forward_string, backward_string",
[
    ("abc","cba"),
    ("12 3","3 21"),
])
def test_string_reverser_logic(forward_string, backward_string, string_reverser):
    forward_string_msg = String(forward_string)
    backward_string_msg = string_reverser.reverse_string_msg( forward_string_msg)

    assert backward_string_msg.data == backward_string, "String not reversed"


@pytest.mark.parametrize("forward_string, backward_string",
[
    ("abc","cba"),
    ("12 3","3 21"),
])
def test_string_reverser_rostopic_interface( backward_string, string_publisher, string_reverser_node, init_node):
    
    backward_string_msg = rospy.wait_for_message("/backward_string", String, timeout=1.0)
    
    assert backward_string_msg.data == backward_string, "String not reversed"
  
    

