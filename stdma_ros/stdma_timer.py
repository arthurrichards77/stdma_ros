# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool


class StdmaTimer(Node):

    def __init__(self):
        self.node_name = 'stdma_timer'
        super().__init__(self.node_name)
        self.signal = False
        self.publisher_ = self.create_publisher(Bool, 'stdma/timer', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Bool()
        msg.data = self.signal
        self.publisher_.publish(msg)
        self.signal = not self.signal


def main(args=None):
    rclpy.init(args=args)

    stdma_timer = StdmaTimer()

    rclpy.spin(stdma_timer)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    stdma_timer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
