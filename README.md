# stdma_ros
## ROS2 implementation of STDMA

[Self-organized time-division multiple access](https://en.wikipedia.org/wiki/Self-organized_time-division_multiple_access) is a way of multiple stations sharing a common channel.  
The channel is shared by dividing it into a number of time frames which are subdivided into time slots.  The number and duration of frames and slots are pre-agreed e.g. by standard.
Roughly, the algorithm for a station joining is:
1. Listen for one whole frame and identify unused slots
2. Enter the channel by choosing an available slot at random and broadcasting a message in it
3. If that message gets through uncontested, slot is secured and can be used repeatedly in each frame
4. If it doesn't get through, perhaps because another joining station broadcast over it, choose another slot and retry in the next frame
5. Repeat every frame until departure

## Why do this in ROS?

The [Robot Operating System](https://www.ros.org) provides communications between processes.  The targeted version, ROS2, does this by employing DDS message handling.
So ROS naturally manages channel access for us and we have no problem with multiple stations sharing a channel.

However, for certain robot coordination and self-organization challenges, it is useful to organize agents into an agreed unique sequence without having a centralized arbiter and enabling asynchronous departures and arrivals.
STDMA provides a basis for doing that.

See [Asadi and Richards, Scalable Model Predictive Control for Constrained Systems](https://doi.org/10.1016/j.automatica.2018.03.050) for an example of using STDMA to coordinate multiple agents.
Here, the agreed sequence represented the order of replanning among a dynamically-scaled group of agents sharing resources, like aircraft sharing space.  

## How it works

A central `timer` ROS node broadcast a common timing signal on the `stdma/timer` topic.  Then multiple `talker` nodes subscribe to the `timer` topic to synchronize their clocks.
The `timer` signal is a simple square wave, alternatinb `True` and `False`.  Rising edge `True` messages mark the slot boundaries and falling edge `False` message mark the slot midpoints.

On each slot boundary, every particpating `talker` node checks for received messages over the `stdma/control` topic during the slot and updates its understanding of the current allocations:
- Zero messages: slot is empty
- One message: slot successfully allocated to the sender of the message
- Two or more messages: message collision, slot not allocated
Once a talker has listened for a whole frame, it chooses a slot is understands to be available, and then sends its ID in that slot in the middle of the next frame.
- If that is the only message received at the end of the frame, the slot is secured, and will be used repeatedly
- If other messages were also received at the end of the frame, there has been a collision, so the talker returns to listening mode and will choose a fresh slot at the end of the frame
- If no messages were received, the message was lost, so the slot is not secure and the talker returns to listening mode 

## Getting started

1. Install ROS2.  This was tested on [Foxy](https://docs.ros.org/en/foxy/Installation.html)
2. Clone this repository into the `src' directory of a [ROS2 overlay](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html)
3. Build the code using `colcon build --packages-select stdma_ros`
4. Run a sinle timer node using `ros2 run stdma_ros timer`
5. Run talker nodes in fresh terminals using `ros2 run stdma_ros talker`
6. Try using the console in `rqt` to watch the `info` messages all together.

## To do

- Probably too much `info` going by
- Need to add a third channel for sending the actual desired messages
- Talker initialization relies on `os.getpid()` to generate unique IDs, which could be problematic in some scenarios
