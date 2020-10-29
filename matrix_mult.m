syms t1 t2 t3 

L1 = 135;
L2 = 175;
L3 = 169.28;



bm = tdh_test(-t1, L1,(pi./2), 0) * ...
     tdh_test(t2, 0, 0, L2) * ...
     tdh_test(t3 - pi/2, 0, 0, L3 )
 
 
175*cos(t1)*cos(t2) + (4232*cos(t3 - pi/2)*(cos(t1)*cos(t2)- (4232*sin(t3 - pi/2)*(cos(t1)*sin(t2)
(4232*cos(t3 - pi/2)*( - cos(t2)*sin(t1)))/25 + ( - 175*cos(t2)*sin(t1) + (4232*sin(t3 - pi/2)*(sin(t1)*sin(t2)))/25
175*sin(t2) + (4232*cos(t2)*sin(t3 - pi/2))/25 + (4232*cos(t3 - pi/2)*sin(t2))/25 + 135
1
