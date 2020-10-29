function [ result ] = tdh_test(theta, d, alpha, a )
%TDH_TEST Summary of this function goes here
%   Detailed explanation goes here
result = [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
           sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
           0,             sin(alpha),             cos(alpha),            d;
           0,                      0,                      0,            1];
            
end

