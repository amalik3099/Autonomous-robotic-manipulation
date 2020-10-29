function [ X ] = jacob1( q, qDot )
%JACOB1 calculates the vector of task space velocities using Jacobian and
%joint velocities (Qdot)

J = jacob0([q(1) q(2) q(3)]);
T = [qDot(1); qDot(2); qDot(3)];
X = J*T;
end
