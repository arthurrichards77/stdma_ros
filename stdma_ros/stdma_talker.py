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

import os
import re
import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class StdmaTalker(Node):

    def __init__(self):
        self.node_name = f'stdma_talker_{os.getpid()}'
        super().__init__(self.node_name)
        self.frame = 0
        self.slot = 0
        self.num_slots = 10
        self.state = 'listen'
        self.my_slot = -1
        self.slot_allocations = [None]*self.num_slots
        self.inbox = []
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 1.0  # seconds
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.subscriber_callback,
            10)
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        received_messages = self.get_messages()
        previous_slot = (self.slot-1) % self.num_slots
        if len(received_messages)==1:
            msg = received_messages[0]
            sender = re.search('stdma_talker_[0-9]+', msg.data).group()
            self.slot_allocations[previous_slot] = sender
            self.get_logger().info('Slot %d belongs to %s, got %s' % (previous_slot, sender, msg.data))                
            if sender==self.node_name:
                # means I've got my own slot
                self.state = 'in'
                self.get_logger().info('Got own message back - successful entry')
        elif len(received_messages)==0:
            self.get_logger().info('Slot %d empty' % previous_slot)
            self.slot_allocations[self.slot] = None
        else:
            self.get_logger().info('Slot %d collision' % previous_slot)
            for msg in received_messages:
                self.get_logger().info('Got message %s' % msg.data)
        if self.state=='check':
            # means my message didn't get through
            self.state = 'listen'
            self.get_logger().info('Collision - back to listening')
        if self.state != 'listen' and self.slot == self.my_slot:
            # time to send a message
            msg = String()
            msg.data = 'Hello from %s : frame %d slot %d' % (self.node_name,self.frame,self.slot)
            self.publisher_.publish(msg)
            if self.state=='enter':
                self.state=='check'
            self.get_logger().info('Sent: "%s" mode : "%s"' % (msg.data, self.state))
        self.slot += 1
        if self.slot==self.num_slots:
            self.slot = 0
            self.frame += 1
            if self.state=='listen':
                # have now listened to a whole slot - time to try and get in
                self.state='enter'
                self.get_logger().info('Listened for whole frame - ready to enter')
                available_slots = [ii for ii in range(self.num_slots) if self.slot_allocations[ii] is None]
                if available_slots:
                    self.my_slot = available_slots[0]
        self.get_logger().info('State %s my slot %d allocs %s' % (self.state, self.my_slot, self.slot_allocations.__str__()))


    def subscriber_callback(self, msg):
        self.inbox.append(msg)

    def get_messages(self):
        received_messages = []
        num_messages = len(self.inbox)
        for ii in range(num_messages):
            received_messages.append(self.inbox.pop(0))
        return received_messages

def main(args=None):
    rclpy.init(args=args)

    stdma_talker = StdmaTalker()

    rclpy.spin(stdma_talker)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    stdma_talker.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
