function [ qd ] = trajectoryPlan( t0, tf, v0, vf, q0, qf )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

A = [1 t0 (t0^2)  (t0)^3; ...
     0 1 2*t0 3*(t0^2); ... 
     1 tf (tf^2) (tf^3); ...
     0 1 2*tf 3*(tf^2);];
 
C = [q0; v0; qf; vf];
 
B = inv(A)*C;

t = linspace(t0,tf,10*(tf-t0));
c = ones(size(t));

qd = B(1).*c + B(2).*t +B(3).*t.^2 + B(4).*t.^3;

end