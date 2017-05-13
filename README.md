README

Work in progress...

Attempt at simulating an automaticly generated inspection path using a Intel Realsense sr300 device to observe an object. 



ikPlanner.cpp - for each point, compute IK, select closest configuration to current configuration, plan motion to that configuration

trac_planner.cpp - for each point compute IK closest to some nominal, then for each configuration plan motion to that point.  

planner.cpp - (Not really) attempt at: using compute cartesian path to simulate a continous linear path. 

desc_planner.cpp - attempt at: using descartes package to simulate a continous linear path.


to launch:

step 1:

--- TODO: make consistent launching of these

