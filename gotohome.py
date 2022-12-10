# The function generates the trajectory form Current Position to Home

import numpy as np

def gotohome(P_current, home, N = 101):
    Px = P_current[0]; Py = P_current[1]; Pz = P_current[2]
    Hx = home[0]; Hy = home[1]; Hz = home[2]

    # Goal Trajectory: Cosine Function --- z = -Acos(wt)+A
    T = np.sqrt((Px - Hx) ** 2 + (Py - Hy) ** 2); # T is half period of Cosine Function
    t = np.linspace(0,T,N)
    A = (Hz - Pz) / 2
    w = np.pi / T

    x = Px - (np.linspace(Hx, Px, N) - Hx)
    y = Py - (np.linspace(Hy, Py, N) - Hy)
    z = Pz - A * np.cos(w * t) + A
    traj = np.vstack((x, y, z))

    return traj