**JACart Electronics Repo**

Contain Python and Arduino controller code for managing the Teleop board.

**Repo Contents**

cart_teleop_dac:/ Updated Arduino teleop code that utilizes DACs instead of PWM signals. This version should be the one used going forward.

cart_teleop_new_board_latest-1.1:/ Arduino code for the Teleop controller. Received commands via USB/Serial. Older code version that utilizes PWM to approximate analog output to motor controller. Should now be deprecated.

python_teleop:/ Stand-alone python controller for teleop_board
