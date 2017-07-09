Guassian Process Adaptive Sampling (GPAS) package

1. Introduction

This is prototyping code for implementing adaptive sampling on GPs. It implements upper-confidence-bound (UCB) search on a Gaussian Process (GP) using receding-horizon cross-entropy (CE) trajectory optimization. It contains a general GP implenetation for any dimensional system, the demonstration scripts are for 2-d scalar fields loaded from an arbitrary ppm file (or other Matlab-compatible graphics file). 

2. The demonstration code is setup as three separate ROS nodes: 

a) GPAS node: performing the search 
b) ENV node: environmental node returning the sensed scalar field
c) ODOM node: odometry or state estimate node returning the current robot state
These three nodes must be run simultaneously for the demonstration.

2.1 Quick Test:

To automatically run all three with a single command open a terminal window and run:
$ ./gpas_test_k1.sh

which would test GPAS on a simulated field loaded from the file 'data/k1.ppm' which contains two interesting regions of high concentrations


2.2 Detailed Test:

You can alternatively open three separate matlab sessions, and run each node separately so that the output of each can be tracked/debugged in separate windows if necessary:

1) Matlab Window#1

>> opt.envFile='data/do1.ppm'
>> env_node(opt)


2) Matlab Window#2

>> odom_node([])


3) Matlab Window#3

>> opt.envFile='data/do1.ppm'
>> gpas_node(opt)

------------


3. Application to the real system

To run with the real system but with simulated environmental data we use a custom "options" file i.e.:

1) Window 1:

>> opts = Options;  env_node(opts);

2) Window 2:

>> opts = Options;  gpas_node(opts);


---------------

To run on real system with real data, then only gpas_node is run; env data and odom data come directly from the vehicle

1) Window 1: 

>> opts = Options;  gpas_node(opts);
