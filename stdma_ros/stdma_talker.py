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
import random
import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32, Bool

class StdmaTalker(Node):

    def __init__(self):
        self.node_id = os.getpid()
        random.seed(self.node_id)
        super().__init__('stdma_talker_%d' % self.node_id)
        self.frame = 0
        self.slot = -1
        self.num_slots = 10
        self.state = 'listen'
        self.my_slot = -2
        self.slot_allocations = [None]*self.num_slots
        self.inbox = []
        self.control_sub = self.create_subscription(
            Int32,
            'stdma/control',
            self.control_callback,
            10)
        self.control_pub = self.create_publisher(Int32,'stdma/control',10)
        self.timer_sub = self.create_subscription(
            Bool,
            'stdma/timer',
            self.timer_callback,
            10)

    def control_callback(self, msg):
        self.inbox.append(msg)

    def get_messages(self):
        received_messages = []
        num_messages = len(self.inbox)
        for ii in range(num_messages):
            received_messages.append(self.inbox.pop(0))
        return received_messages

    def timer_callback(self, msg):
        if msg.data:
            # rising edge - start/end of slot
            self.end_slot_callback()
        else:
            # falling edge - middle of slot
            self.mid_slot_callback()

    def end_slot_callback(self):
        if self.slot==-1:
            # first ever run - need to wait for end of first slot
            self.slot = 0
            self.get_logger().info('Start of slot 0 frame 0')
        else:
            self.get_logger().info('End of slot %d frame %d' % (self.slot, self.frame))
            # end of slot - check received messages
            received_messages = self.get_messages()
            num_messages = len(received_messages)
            if num_messages==0:
                self.get_logger().info('Slot %d looks empty' % self.slot)
                self.slot_allocations[self.slot] = None
            elif num_messages==1:
                msg = received_messages[0]
                sender = msg.data
                self.get_logger().info('Slot %d allocated to %d' % (self.slot, sender))
                self.slot_allocations[self.slot] = sender
                if sender==self.node_id:
                    # my own message - means successfully in channel
                    self.state = 'in'
                    self.get_logger().info('Secured my own slot')
            elif num_messages>1:
                # more than one message in a slot - collision
                colliding_ids = [m.data for m in received_messages]
                self.get_logger().warning('Collision between %s' % colliding_ids.__str__())
                self.slot_allocations[self.slot] = None
                if self.node_id in colliding_ids:
                    self.get_logger().warning('Lost my slot due to collision')
                    self.state = 'listen'
                    self.my_slot = -2
            if self.state == 'check':
                # successful join would have changed state to 'in' 
                # collision would have changed to 'listen'
                # so this must mean lost data
                self.state = 'listen'
                self.my_slot = -2
            # update for next slot
            self.slot += 1
            if self.slot==self.num_slots:
                self.slot = 0
                self.frame += 1
                if self.state == 'listen':
                    # have listened to a whole frame - time to try and get in
                    self.state = 'enter'
                    available_slots = [ii for ii in range(self.num_slots) if self.slot_allocations[ii]==None]
                    self.my_slot = random.sample(available_slots, 1)[0]
                    self.get_logger().info('Will try to enter at slot %d' % self.my_slot)
            self.get_logger().info('Slot allocations: %s' % self.slot_allocations.__str__())
            self.get_logger().info('State: "%s" my slot: %d' % (self.state, self.my_slot))

    def mid_slot_callback(self):
        if self.slot>=0:
            self.get_logger().info('Middle of slot %d frame %d' % (self.slot, self.frame))
            if self.slot==self.my_slot:
                msg = Int32()
                msg.data = self.node_id
                self.control_pub.publish(msg)
                self.state = 'check'
                self.get_logger().info('Sent my control message')


    def rubbish():
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
