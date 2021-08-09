#!/usr/bin/env python3
#Author: lyleokoth

from RRbotKinematics.rrbot_IK import ForwardKinematics
import numpy as np

rrbot_dimensions = [1,1,0.05]

#print(ForwardKinematics(rrbot_dimensions).rotation_base_to_l1(0))
#print(ForwardKinematics(rrbot_dimensions).rotation_l1_to_l2(1.57))
print(ForwardKinematics(rrbot_dimensions).rotation_base_to_endeffector(1.57,1.57))