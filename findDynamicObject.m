function [objectStats] = findDynamicObject(imOrig, T_checker_to_robot, T_cam_to_checker, cameraParams)

%%  1. First things first - undistort the image using the camera parameters
[im, ~] = undistortImage(imOrig, cameraParams, 'OutputView', 'full');

%%  2. Segment the image to find the objects of interest.
R = T_cam_to_checker(1:3,1:3);
t = T_cam_to_checker(1:3,4);
%  [Your image processing code goes here]
B = imfilter(im,[1/9,1/9,1/9;1/9,1/9,1/9;1/9,1/9,1/9;]);
objectStats = [];

Greens = createGreenMask(B);
Blues = createBlueMask(B);
Yellows = createYellowMask(B);
Sizes = createSizeMask(B);

%allColors = or(or(Blues,Yellows), Greens);
imshow(imOrig)

%Finds sizes of circles and highlights bases with red circle
[centersDark, radiiDark] = imfindcircles(Sizes,[30 75],'ObjectPolarity','dark');
viscircles(centersDark, radiiDark,'Color','r');

Rmin = 18;
Rmax = 40;
%Finds circles centers
[centersYellow, radiiBright] = imfindcircles(Yellows,[Rmin Rmax],'ObjectPolarity','bright');
viscircles(centersYellow, radiiBright,'Color','y');

[centersBlue, radiiBright] = imfindcircles(Blues,[Rmin Rmax],'ObjectPolarity','bright');
viscircles(centersBlue, radiiBright,'Color','b');

[centersGreen, radiiBright] = imfindcircles(Greens,[Rmin Rmax],'ObjectPolarity','bright');
viscircles(centersGreen, radiiBright,'Color','g');


counter = 0;
%Calculates the task space location of the centroids in relation to checker

if isempty(centersYellow) == 0
    yellowWorldPoints = pointsToWorld(cameraParams,R,t,centersYellow);
    for i = 1:size(yellowWorldPoints,1)
        objectStats(i,1) = 0; % yellow = 0
        objectStats(i,2) = yellowWorldPoints(i,1);
        objectStats(i,3) = yellowWorldPoints(i,2);
        
    end
    counter = i;
end

if isempty(centersBlue) == 0
    blueWorldPoints = pointsToWorld(cameraParams,R,t,centersBlue);
    for i = 1:size(blueWorldPoints,1)
        objectStats(i+counter,1) = 1; % blue = 1
        objectStats(i+counter,2) = blueWorldPoints(i,1);
        objectStats(i+counter,3) = blueWorldPoints(i,2);
        
    end
    counter = counter + i;
end

if  isempty(centersGreen) == 0
    greenWorldPoints = pointsToWorld(cameraParams,R,t,centersGreen);
    for i = 1:size(greenWorldPoints,1)
        objectStats(i+counter,1) = 2; % green = 2
        objectStats(i+counter,2) = greenWorldPoints(i,1);
        objectStats(i+counter,3) = greenWorldPoints(i,2);
        
    end
end

%x = [127 43 560 486];
%y = [123 440 442 133];
%insideWorkspace = poly2mask(x,y,488,651);

%sizesWithinWorkspace = bsxfun(@times, Sizes, cast(insideWorkspace, 'like', Sizes));
%sizeCentroids = regionprops(sizesWithinWorkspace,'centroid');

end

