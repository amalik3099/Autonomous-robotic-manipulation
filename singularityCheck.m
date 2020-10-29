function [ singularity ] = singularityCheck( j1, j2, j3 )
%SINGULARITYCHECK Summary of this function goes here
%   Detailed explanation goes here

singularity = false(1);
threshold = 0.5;

if (j1 < 0 + threshold && j1 > 0 - threshold) && (j2 < 0 + threshold && j2 > 0 - threshold) && (j3 < pi/2 + threshold && j3 > pi/2 - threshold)
    singularity = true(1);
end

if (j1 < 0 + threshold && j1 > 0 - threshold) && (j2 < pi/2 + threshold && j2 > pi/2 - threshold) && (j3 < pi/2 + threshold && j3 > pi/2 - threshold)
    singularity = true(1);
end

end

