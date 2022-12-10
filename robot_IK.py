import numpy as np
import math

def ik (targXYZ, Mode):
    # input targXYZ = [x, y, z] m
    global c
    L1 = c[0]; L2 = c[1]; L3 = c[2]; L4 = c[3]
    x = targXYZ[0]; y = targXYZ[1]; z = targXYZ[2]

    if Mode == 1: # Horizontal Case

######################
        theta1 = math.atan2(y,x)
    
        c3 = (x ** 2 + y ** 2 + z ** 2 + L1 ** 2 + L4 ** 2 - L2 ** 2 - L3 ** 2 - 2 * z * L1 - 2 * L4 * np.sqrt(x ** 2 + y ** 2)) / (2 * L2 * L3)
        s3 = -np.sqrt(1 - c3 ** 2); # Two Solns: s3 = sqrt(1 - c3 ** 2)
        theta3 = math.atan2(s3,c3)
        
        c2 = (L2 + L3 * c3) * (np.sqrt(x ** 2 + y ** 2) - L4) + L3 * s3 * (z - L1)
        s2 = (L2 + L3 * c3) * (z - L1) - L3 * s3 * (np.sqrt(x ** 2 + y ** 2) - L4)
        theta2 = math.atan2(s2,c2)
        
        theta4 = - theta2 - theta3

########################

    elif Mode == 2: # Vertical Case

        theta1 = math.atan2(y,x)
    
        c3 = (x ** 2 + y ** 2 + z ** 2 - L2 ** 2 - L3 ** 2 + (L1 - L4) ** 2 + 2 * z * (L4 - L1)) / (2 * L2 * L3)
        s3 = -np.sqrt(1 - c3 ** 2); # Two Solns: s3 = sqrt(1 - c3 ** 2)
        theta3 = math.atan2(s3,c3)
        
        c2 = (L2 + L3 * c3) * np.sqrt(x ** 2 + y ** 2) + L3 * s3 * (z - L1 + L4)
        s2 = (L2 + L3 * c3) * (z - L1 + L4) - L3 * s3 * np.sqrt(x ** 2 + y ** 2)
        theta2 = math.atan2(s2,c2)

        theta4 = - np.pi / 2 - theta2 - theta3

    else:
        print('Unauthorized Mode?')

    # Calculate jAng for motors as int
    jAng_temp = np.array([theta1, theta2, theta3, theta4])
    jAng = np.round(jAng_temp / np.pi * 2048 + 2048).astype(int)

    # output jAng = [t1, t2, t3, t4],  0 < t1,t2,t3,t4 < 4095
    jAng = jAng.tolist()

    return jAng 


# Test Only
if __name__ == '__main__':
    c = [0.10, 0.13, 0.13, 0.06] # Length Parameters of Manipulator
    Mode = 2 # or 2
    targXYZ = np.array([0.15, 0, 0.08])
    jAng = ik(targXYZ, Mode)
    print(jAng)
    print(type(jAng))
    
