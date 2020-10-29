function [ qd ] = trajectoryPlanQuin( t0, tf, v0, vf, q0, qf, a0, af )

A = [1 t0 (t0^2)  (t0)^3 (t0)^4 (t0)^5; ...
     0 1 2*t0 3*(t0^2) 4*(t0)^3 5*(t0)^4; ... 
     0 0 2 6*t0 12*(t0)^2 20*(t0)^3; ...
     1 tf tf^2 tf^3 tf^4 tf^5; ...
     0 1 2*tf 3*(tf^2) 4*tf^3 5*tf^4; ...
     0 0 2 6*tf 12*tf^2 20*tf^3];
 
C = [q0; v0; a0; qf; vf; af];
 
B = inv(A)*C;

t = linspace(t0,tf,8);
c = ones(size(t));

qd = B(1).*c + B(2).*t +B(3).*t.^2 + B(4).*t.^3 + B(5)*t.^4 + B(6)*t.^5;

end