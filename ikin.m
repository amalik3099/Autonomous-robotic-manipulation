function [ result ] = ikin( M )

x = M(1);
y = M(2);
z = M(3);

L1 = 135;
a2 = 175;
a3 = 169.28;
S = z - L1;
r = sqrt(x^2 + y^2);
L4 = sqrt(S^2 + r^2);

alpha = acos((a2^2 + L4^2 -a3^2)/(2*a2*L4));
beta = atan2(S, r);
gamma = acos((a2^2 + a3^2 -L4^2)/(2*a2*a3));

t1 = -atan2(y,x);
t3 = gamma -pi/2;
t2 = alpha + beta; 

t1 = t1*4096/(2*pi);
t2 = t2*4096/(2*pi);
t3 = t3*4096/(2*pi);

result = [t1,t2,t3];

if (t1 > 990 || t1 < -990)  
    error('Theta 1 Exceeded Workspace Limits')
end
if (t2 > 1130 || t2 < -80)  
    error('Theta 2 Exceeded Workspace Limits')
end
if (t3 > 2470 || t3 < -350)  
    error('Theta 3 Exceeded Workspace Limits')
end

