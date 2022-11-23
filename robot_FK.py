import numpy as np
import math

def fk (jAng):
    # input jAng = [t1,t2,t3,t4] is a list. 0 < t1,t2,t3,t4 < 4095
    
    jAng = np.array(jAng)
    jAng = (jAng - 2048) / 2048 * math.pi
    
    global c
    # global c = [L1, L2, L3, L4] from main.py: geometric parameters of the Manipulator
    d1 = c[0]; a2 = c[1]; a3 = c[2]; a4 = c[3]
    # Assume jAng is angles within the range of [-pi, pi] <--> [0, 4095] ?
    theta1 = jAng[0]; theta2 = jAng[1]; theta3 = jAng[2]; theta4 = jAng[3]

    # DH parameters
    DH = np.array([[0, 0, d1, theta1], [0, math.pi/2, 0, theta2], [a2, 0, 0, theta3], \
        [a3, 0, 0, theta4], [a4, 0, 0, 0]])
    a = DH[:,0]; alpha = DH[:,1]; d = DH[:,2]; theta = DH[:,3]

    # initial
    To = np.eye(4)

    for j in range(5):
    
        Ti = np.array([[math.cos(theta[j]), -math.sin(theta[j]), 0, a[j]],
            [math.sin(theta[j])*math.cos(alpha[j]), math.cos(theta[j])*math.cos(alpha[j]), \
            -math.sin(alpha[j]), -math.sin(alpha[j])*d[j]],
            [math.sin(theta[j])*math.sin(alpha[j]), math.cos(theta[j])*math.sin(alpha[j]), \
            math.cos(alpha[j]), math.cos(alpha[j])*d[j]],
            [0, 0, 0, 1]])
        To = np.dot(To, Ti)

    # output pos  = [x,y,z] --> end effector
    pos_temp = To[0:3,3]
    # keep only 3 digits
    pos = [float('{:.3f}'.format(i)) for i in pos_temp]
    return pos


# Test Only
if __name__ == '__main__':
    c = [0.087, 0.111, 0.078, 0.020] # Length Parameters of Manipulator
    jAng = [math.pi/6, -math.pi/2, math.pi/4, -math.pi]
    pos = fk (jAng, c = [0.087, 0.111, 0.078, 0.020])
    print(pos)
    print(type(pos))
    
