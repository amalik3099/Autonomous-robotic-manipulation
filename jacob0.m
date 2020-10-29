function [ M ] = jacob0(q)
%Calculates the 6x3 Jacobian matrix 

L1 = 135;
L2 = 175;
L3 = 169.28;

t1 = q(1);    
t2 = q(2);    
t3 = q(3);

% M matrix calculated by matlab using syms
M = [ (4232*cos(t3 - pi/2)*((4967757600021511*cos(t1)*sin(t2))/81129638414606681695789005144064 - cos(t2)*sin(t1)))/25 + (869357580003764425*cos(t1)*sin(t2))/81129638414606681695789005144064 - 175*cos(t2)*sin(t1) + (4232*sin(t3 - pi/2)*((4967757600021511*cos(t1)*cos(t2))/81129638414606681695789005144064 + sin(t1)*sin(t2)))/25, (869357580003764425*cos(t2)*sin(t1))/81129638414606681695789005144064 - 175*cos(t1)*sin(t2) - (4232*cos(t3 - pi/2)*(cos(t1)*sin(t2) - (4967757600021511*cos(t2)*sin(t1))/81129638414606681695789005144064))/25 - (4232*sin(t3 - pi/2)*(cos(t1)*cos(t2) + (4967757600021511*sin(t1)*sin(t2))/81129638414606681695789005144064))/25, - (4232*cos(t3 - pi/2)*(cos(t1)*sin(t2) - (4967757600021511*cos(t2)*sin(t1))/81129638414606681695789005144064))/25 - (4232*sin(t3 - pi/2)*(cos(t1)*cos(t2) + (4967757600021511*sin(t1)*sin(t2))/81129638414606681695789005144064))/25;
 (4232*sin(t3 - pi/2)*(cos(t1)*sin(t2) - (4967757600021511*cos(t2)*sin(t1))/81129638414606681695789005144064))/25 - (4232*cos(t3 - pi/2)*(cos(t1)*cos(t2) + (4967757600021511*sin(t1)*sin(t2))/81129638414606681695789005144064))/25 - 175*cos(t1)*cos(t2) - (869357580003764425*sin(t1)*sin(t2))/81129638414606681695789005144064, (869357580003764425*cos(t1)*cos(t2))/81129638414606681695789005144064 + (4232*cos(t3 - pi/2)*((4967757600021511*cos(t1)*cos(t2))/81129638414606681695789005144064 + sin(t1)*sin(t2)))/25 - (4232*sin(t3 - pi/2)*((4967757600021511*cos(t1)*sin(t2))/81129638414606681695789005144064 - cos(t2)*sin(t1)))/25 + 175*sin(t1)*sin(t2),   (4232*cos(t3 - pi/2)*((4967757600021511*cos(t1)*cos(t2))/81129638414606681695789005144064 + sin(t1)*sin(t2)))/25 - (4232*sin(t3 - pi/2)*((4967757600021511*cos(t1)*sin(t2))/81129638414606681695789005144064 - cos(t2)*sin(t1)))/25;
                                                                                                                                                                                                                                                                                                                                 0,                                                                                                                                                                                                                                                 175*cos(t2) + (4232*cos(t2)*cos(t3 - pi/2))/25 - (4232*sin(t2)*sin(t3 - pi/2))/25,                                                                                                                                                                   (4232*cos(t2)*cos(t3 - pi/2))/25 - (4232*sin(t2)*sin(t3 - pi/2))/25;
                                                                                                                                                                                                                                                                                                                                 0,                                                                                                                                                                                                                                                                                                                          -sin(t1),                                                                                                                                                                                                                              -sin(t1);
                                                                                                                                                                                                                                                                                                                                 0,                                                                                                                                                                                                                                                                                                                          -cos(t1),                                                                                                                                                                                                                              -cos(t1);
                                                                                                                                                                                                                                                                                                                                 1,                                                                                                                                                                                                                                                                                 4967757600021511/81129638414606681695789005144064,                                                                                                                                                                                     4967757600021511/81129638414606681695789005144064];
%]

D = det([M(1,1), M(1,2), M(1,3);
        M(2,1), M(2,2), M(2,3);
        M(3,1), M(3,2), M(3,3)])
end

