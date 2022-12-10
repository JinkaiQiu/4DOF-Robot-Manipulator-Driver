import numpy as np

def fk (jAng):
    # input jAng = [t1,t2,t3,t4], 0 < t1,t2,t3,t4 < 4095
    
    jAng = np.array(jAng)
    jAng = (jAng - 2048) / 2048 * np.pi

    global c
    d1 = c[0]; a2 = c[1]; a3 = c[2]; a4 = c[3]
    # Assume jAng is angles within the range of [-pi, pi] <--> [0, 4095] ?
    theta1 = jAng[0]; theta2 = jAng[1]; theta3 = jAng[2]; theta4 = jAng[3]

    # DH parameters
    DH = np.array([[0, 0, d1, theta1], [0, np.pi/2, 0, theta2], [a2, 0, 0, theta3], \
        [a3, 0, 0, theta4], [a4, 0, 0, 0]])
    a = DH[:,0]; alpha = DH[:,1]; d = DH[:,2]; theta = DH[:,3]

    # initial
    To = np.eye(4)

    for j in range(5):
    
        Ti = np.array([[np.cos(theta[j]), -np.sin(theta[j]), 0, a[j]],
            [np.sin(theta[j])*np.cos(alpha[j]), np.cos(theta[j])*np.cos(alpha[j]), \
            -np.sin(alpha[j]), -np.sin(alpha[j])*d[j]],
            [np.sin(theta[j])*np.sin(alpha[j]), np.cos(theta[j])*np.sin(alpha[j]), \
            np.cos(alpha[j]), np.cos(alpha[j])*d[j]],
            [0, 0, 0, 1]])
        To = np.dot(To, Ti)

    # output pos  = [x,y,z] --> end effector
    pos_temp = To[0:3,3]
    # keep only 4 digits
    pos = [float('{:.4f}'.format(i)) for i in pos_temp]
    return pos

# Test Only
if __name__ == '__main__':
    c = [0.10, 0.13, 0.13, 0.06] # Length Parameters of Manipulator
    jAng = [2048, 3072, 2048, 1024] # jAng = [0, pi/2, 0, 0] as home position
    pos = fk (jAng)
    print(pos)
    print(type(pos))

    
