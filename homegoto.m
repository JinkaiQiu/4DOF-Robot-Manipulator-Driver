function [x, y, z] = homegoto(P_start,home,N)

Px = P_start(1); Py = P_start(2); Pz = P_start(3);
Hx = home(1); Hy = home(2); Hz = home(3);

% Goal Trajectory: Cosine Function --- z = Acos(wt)+A
T = sqrt((Px-Hx)^2 + (Py-Hy)^2); % T is half period of Cosine Function
t = linspace(0,T,N);
A = (Hz - Pz) / 2;
w = pi / T;

x = linspace(Hx,Px,N);
y = linspace(Hy,Py,N);
z = Pz + A * cos(w * t) + A;

end