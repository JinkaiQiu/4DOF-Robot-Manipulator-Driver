import numpy as np
import math

def homegoto(P_start, home, N = 101):
    Px = P_start[0]; Py = P_start[1]; Pz = P_start[2]
    Hx = home[0]; Hy = home[1]; Hz = home[2]

    # Goal Trajectory: Cosine Function --- z = Acos(wt)+A
    T = np.sqrt((Px - Hx) ** 2 + (Py - Hy) ** 2); # T is half period of Cosine Function
    t = np.linspace(0,T,N)
    A = (Hz - Pz) / 2
    w = np.pi / T

    x = np.linspace(Hx, Px, N)
    y = np.linspace(Hy, Py, N)
    z = Pz + A * np.cos(w * t) + A
    traj = np.vstack((x, y, z))

    return traj
