function [theta1,theta2,theta3,theta4] = IK_lyc(p, Mode)
global L1 L2 L3 L4

x = p(1); y = p(2); z = p(3);

if Mode == 1    
    %%%%%%%%%% Algebraic
    theta1 = atan2(y,x);
    
    c3 = (x^2 + y^2 + z^2 + L1^2 + L4^2 - L2^2 - L3^2 - 2 * z * L1 - 2 * L4 * sqrt(x^2 + y^2)) / (2 * L2 * L3);
    s3 = -sqrt(1 - c3^2); % Two Solns: s3 = sqrt(1 - c3^2);
    theta3 = atan2(s3,c3);
    
    c2 = (L2 + L3 * c3) * (sqrt(x^2 + y^2) - L4) + L3 * s3 * (z - L1);
    s2 = (L2 + L3 * c3) * (z - L1) - L3 * s3 * (sqrt(x^2 + y^2) - L4);
    theta2 = atan2(s2,c2);
    
    theta4 = - theta2 - theta3;
    %%%%%%%%%%
    
elseif Mode == 2    
    %%%%%%%%%% Algebraic
    theta1 = atan2(y,x);
    
    c3 = (x^2 + y^2 + z^2 - L2^2 - L3^2 + (L1 - L4)^2 + 2 * z * (L4 - L1)) / (2 * L2 * L3);
    s3 = -sqrt(1 - c3^2); % Two Solns: s3 = sqrt(1 - c3^2);
    theta3 = atan2(s3,c3);
    
    c2 = (L2 + L3 * c3) * sqrt(x^2 + y^2) + L3 * s3 * (z - L1 + L4);
    s2 = (L2 + L3 * c3) * (z - L1 + L4) - L3 * s3 * sqrt(x^2 + y^2);
    theta2 = atan2(s2,c2);
    
    theta4 = - pi/2 - theta2 - theta3;
    %%%%%%%%%%
else
    disp('Unauthorized Mode?')
end


end
