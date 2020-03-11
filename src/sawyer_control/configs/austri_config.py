from sawyer_control.configs.base_config import *
import numpy as np
from gym.spaces import Box

TORQUE_SAFETY_BOX_LOWS = np.array([0.4, -0.25, 0.2])
TORQUE_SAFETY_BOX_HIGHS = np.array([0.7, 0.25, 0.7])
TORQUE_SAFETY_BOX = Box(TORQUE_SAFETY_BOX_LOWS, TORQUE_SAFETY_BOX_HIGHS, dtype=np.float32)

# POSITION_SAFETY_BOX_LOWS = np.array([ 0.65, -0.03,  0.3])
# POSITION_SAFETY_BOX_HIGHS = np.array([0.782, 0.07, 0.36])

# POSITION_SAFETY_BOX_LOWS = np.array([ 0.34, -0.34,  0.015])
# POSITION_SAFETY_BOX_HIGHS = np.array([0.83, 0.34, 0.7])

POSITION_SAFETY_BOX_LOWS = np.array([0.45, -0.30, 0.015])
POSITION_SAFETY_BOX_HIGHS = np.array([0.85, 0.30, 0.7])

POSITION_SAFETY_BOX = Box(POSITION_SAFETY_BOX_LOWS, POSITION_SAFETY_BOX_HIGHS, dtype=np.float32)

# POSITION_RESET_POS = np.array([0.73, 0.0,  0.34222245])

POSITION_RESET_POS = np.array([0.53, 0.0,  0.15])

## Right down [ 0.34, -0.34,  0.015]
## Left down [ 0.34, 0.34,  0.015  ] 
## Left top [ 0.83, 0.34, 0.16]
## Right top [ 0.83, -0.34, 0.16]

## 
##  Reset pos 0.53, 0.0,  0.15
## 