import pytest
import subprocess
import rospy
import time

# conftest.py is a special file that pytest will 
# load and make available for all other tests in this directory

@pytest.fixture(scope="module")
def roscore():
    """
    Starts a roscore in a background process.
    Stops it once the tests are done.
    """
    try:
        process = subprocess.Popen("roscore")
        wait_for_roscore_to_be_running(2)
        yield
    finally:
        process.terminate()
        process.wait()


def wait_for_roscore_to_be_running(timeout):
    
    start_time = time.time()

    while time.time() - start_time < timeout:
        try:
            # Fails if roscore is not running
            rospy.get_published_topics()
            
            # Roscore must be running to get here
            return
        except:
            time.sleep(0.01)

    raise Exception("roscore not running")
