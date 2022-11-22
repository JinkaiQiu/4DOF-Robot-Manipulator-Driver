function [fx,fy,fz,T] = FK_lyc(joint)
global c
d1 = c(1); a2 = c(2); a3 = c(3); a4 = c(4);
theta1 = joint(1); theta2 = joint(2); theta3 = joint(3); theta4 = joint(4);

% DH parameters
DH = [0 0 d1 theta1;0 pi/2 0 theta2;a2 0 0 theta3;a3 0 0 theta4;a4 0 0 0]; %
a = DH(:,1); alpha = DH(:,2); d = DH(:,3); theta = DH(:,4);

% initial
To = eye(4);
T = cell(1,6); % 4 Joints + 1 Base + 1 End Effector
T{1} = To; % {Base} frame
fx = zeros(1,6); fy = zeros(1,6); fz = zeros(1,6);

for j = 1:5
    
    Ti = [cos(theta(j)) -sin(theta(j)) 0 a(j);
        sin(theta(j))*cos(alpha(j)) cos(theta(j))*cos(alpha(j)) ...
        -sin(alpha(j)) -sin(alpha(j))*d(j);
        sin(theta(j))*sin(alpha(j)) cos(theta(j))*sin(alpha(j)) ...
        cos(alpha(j)) cos(alpha(j))*d(j);0 0 0 1];
    To = To*Ti;
    fx(j+1) = To(1,4); % frame x coordiante 1x6 vector
    fy(j+1) = To(2,4); % frame y coordiante
    fz(j+1) = To(3,4); % frame z coordiante
    T{j+1} = To;
    
end

end