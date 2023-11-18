# Training

Repository for learning and training using a git repository.

## Getting Started

Download links:

SSH clone URL: ssh://git@git.jetbrains.space/brt-driverless/trn/Training.git

HTTPS clone URL: https://git.jetbrains.space/brt-driverless/trn/Training.git

## Interface_example

There are my custom msg and srv.

To show the msg, use

    $ ros2 interface show interface_example/msg/MsgTest

## Interface_test_example 
 
There are publisher and subscriber for these custom msgs and srv.

To run the pub and sun, use

	$ ros2 run interface_test_example publisher

	$ ros2 run interface_test_example subscriber

To use the launch file, use 

	$ ros2 launch interface_test_example pub_sub_launch.py 

