from enum import Enum

class ReedsSheppPathType(Enum):
    # CSC 유형 (Curve-Straight-Curve)
    LSL = 1  # Left-Straight-Left
    LSR = 2  # Left-Straight-Right
    
    # CCC 유형 (Curve-Curve-Curve)
    L_X_R_X_L = 3  # Left-(-)Right-(-)Left
    L_X_RL = 4 #Left-(-)Right-Left
    LR_X_L = 5 #Left-Right-(-)Left
    LR_X_LL = 6 # Left-(-)Right-(-)Left-Left
    
    # CCCC 유형 (Curve-Curve-Curve-Curve)
    LR_X_L_X_R = 7
    # CCSC 유형 (Curve-Curve-Straight-Curve)
    L_X_R90SL = 8
    LSR90_X_R = 9
    L_X_R90SR = 10
    LSL90_X_R = 11
    
    # CCSCC 유형 (Curve-Curve-Straight-Curve-Curve)
    L_X_R90SL_X_R = 12