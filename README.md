# Examples of Using PyTest with ROS
This repo gives some examples of how to use pytest with a ROS.

ROS nodes can be harder to test becuase they need roscore, publishers, global parameters and services all available for it to work.

# Why Test

Tests do more than prove your code works.

Tests enable clean code!

When there are good tests, you will feel comfortable refactoring code to make it cleaner without the fear of breaking it.

When you write testable code, it will naturally be more modular. Complicated things are hard to test, so you will naturally write simpler things that are easier to test.


# Run PyTest
``` pytest ```

# Run PyTest and show print statements
``` pytest -s ```

# Test a specific module
``` pytest src/code_tests/test_string_reverser.py ```

# Run a specific test in a specific module
``` pytest src/code_tests/test_string_reverser.py::test_string_reverser_logic ```

# Test a module that matches a key
``` pytest -k test_string_reverser.py ```

# Run a test in a module that matches a query keys
``` pytest -k test_string_reverser.py -k test_string_reverser_logic ```
