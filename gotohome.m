function [x, y, z] = gotohome(P_current,home,N)

Px = P_current(1); Py = P_current(2); Pz = P_current(3);
Hx = home(1); Hy = home(2); Hz = home(3);

% Goal Trajectory: Cosine Function --- z = -Acos(wt)+A
T = sqrt((Px-Hx)^2 + (Py-Hy)^2); % T is half period of Cosine Function
t = linspace(0,T,N);
A = (Hz - Pz) / 2;
w = pi / T;

x = Px - (linspace(Hx,Px,N) - Hx);
y = Py - (linspace(Hy,Py,N) - Hy);
z = Pz - A * cos(w * t) + A;

end