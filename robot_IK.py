import numpy as np
import math

def ik (targXYZ, Mode):
    # input targXYZ = [x, y, z] m
    global c
    L1 = c[0]; L2 = c[1]; L3 = c[2]; L4 = c[3]
    x = targXYZ[0]; y = targXYZ[1]; z = targXYZ[2]

    if Mode == 1: # Horizontal Case
        theta1 = math.atan2(y, x)
        r1 = math.sqrt(x ** 2 + y ** 2) - L4
        r2 = math.sqrt(r1 ** 2 + (z - L1) ** 2)

        c3 = (r2 ** 2 - L2 ** 2 - L3 ** 2) / (2 * L2 * L3)
        s3 = -math.sqrt(1 - c3 ** 2); # Two Solns: s3 = math.sqrt(1 - c3 ** 2)

        theta3 = math.atan2(s3, c3)

        alpha = math.atan2(z - L1, r1)
        cb = (L2 ** 2 + r2 ** 2 - L3 ** 2) / (2 * L2 * r2)
        sb = - L3 * s3 / r2
        beta = math.atan2(sb, cb)
        theta2 = alpha + beta

        theta4 = - theta2 - theta3

    elif Mode == 2: # Vertical Case
        theta1 = math.atan2(y, x)
        r1 = math.sqrt(x ** 2 + y ** 2)
        r2 = math.sqrt(r1 ** 2 + (z - L1 + L4) ** 2)

        c3 = (r2 ** 2 - L2 ** 2 - L3 ** 2) / (2 * L2 * L3)
        s3 = -math.sqrt(1 - c3 ** 2); # Two Solns: s3 = sqrt(1 - c3^2)

        theta3 = math.atan2(s3, c3)

        alpha = math.atan2(z - L1 + L4, r1)
        cb = (L2 ** 2 + r2 ** 2 - L3 ** 2) / (2 * L2 * r2)
        sb = - L3 * s3 / r2
        beta = math.atan2(sb, cb)
        theta2 = alpha + beta

        theta4 = - math.pi / 2 - theta2 - theta3

    else:
        print('Unauthorized Mode?')

    jAng_temp = np.array([theta1, theta2, theta3, theta4])
    
    jAng = np.round(jAng_temp / math.pi * 2048 + 2048)
    # output jAng = [t1, t2, t3, t4],  0 < t1,t2,t3,t4 < 4095
    jAng = jAng.tolist()
    # Assume jAng is angles within the range of [-pi, pi] <--> [0, 4095] ?    

    return jAng 


# Test Only
if __name__ == '__main__':
    c = [0.085, 0.130, 0.130, 0.004] # Length Parameters of Manipulator (verified already)
    Mode = 1 # or 2
    targXYZ = np.array([0.15, 0, 0.08]) # Attention: workspace limit
    jAng = ik (targXYZ, Mode)
    print(jAng)
    print(type(jAng))
    
