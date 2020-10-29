function [ M ] = fwkin3001( q )
%Calculates 4x4 transformation matrix and matrix of Px,Py,Pz for each
%transformation
    
L1 = 135;
L2 = 175;
L3 = 169.28;

t1 = q(1);    
t2 = q(2);    
t3 = q(3);
% End effector Transformation 
T = tdh_test(-t1, L1,(pi./2), 0) * ...        
    tdh_test(t2, 0, 0, L2) * ...        
    tdh_test(t3 - pi/2, 0, 0, L3 ); 
% Joint 2 Transformation
S = tdh_test(-t1, L1, (pi./2), 0) * ...        
    tdh_test(t2, 0, 0, L2);
% Base Transformation
W = tdh_test(-t1, L1, (pi./2), 0);
% Matrix of Px, Py, Pz of respective transformation
M = [W(1,4), S(1,4), T(1,4);
    W(2,4), S(2,4),T(2,4);
    W(3,4), S(3,4),T(3,4)];

end

