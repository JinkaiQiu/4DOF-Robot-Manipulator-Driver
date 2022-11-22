function [theta1,theta2,theta3,theta4] = IK_lyc(p, Mode)
global c
L1 = c(1); L2 = c(2); L3 = c(3); L4 = c(4);
x = p(1); y = p(2); z = p(3);

if Mode == 1
    theta1 = atan2(y,x);
    r1 = sqrt(x^2 + y^2) - L4;
    r2 = sqrt(r1^2 + (z - L1)^2);

    c3 = (r2^2 - L2^2 - L3^2) / (2 * L2 * L3);
    s3 = -sqrt(1 - c3^2); % Two Solns: s3 = sqrt(1 - c3^2);

    theta3 = atan2(s3,c3);

    alpha = atan2(z - L1, r1);
    cb = (L2^2 + r2^2 - L3^2) / (2 * L2 * r2);
    sb = - L3 * s3 / r2;
    beta = atan2(sb, cb);
    theta2 = alpha + beta;

    theta4 = - theta2 - theta3;
    
elseif Mode == 2
    theta1 = atan2(y,x);
    r1 = sqrt(x^2 + y^2);
    r2 = sqrt(r1^2 + (z - L1 + L4)^2);

    c3 = (r2^2 - L2^2 - L3^2) / (2 * L2 * L3);
    s3 = -sqrt(1 - c3^2); % Two Solns: s3 = sqrt(1 - c3^2);

    theta3 = atan2(s3,c3);

    alpha = atan2(z - L1 + L4, r1);
    cb = (L2^2 + r2^2 - L3^2) / (2 * L2 * r2);
    sb = - L3 * s3 / r2;
    beta = atan2(sb, cb);
    theta2 = alpha + beta;

    theta4 = - pi / 2 - theta2 - theta3;
    
else
    disp('Unauthorized Mode?')
end

end