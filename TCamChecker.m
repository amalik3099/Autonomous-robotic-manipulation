load('calibrationSession.mat')

if exist('cam') == 0
    cam = webcam();
end    
% camParameters;
% getCamToCheckerboard(cam, calibrationSession.CameraParameters);
% 
% T_cam_to_checker = getCamToCheckerboard(cam, cameraParams)
T_cam_to_checker = [
    0.0118   -0.8305    0.5568   96.5275;
    0.9998    0.0170    0.0042  105.2922;
    -0.0129    0.5567    0.8306  277.6281;
    0         0         0    1.0000];

%T_checker_to_0 = getCamToCheckerboard(,cameraParams);

R = T_cam_to_checker(1:3,1:3);
t = T_cam_to_checker(1:3,4);

worldPoints = pointsToWorld(calibrationSession.CameraParameters,R,t,[126,156;60,425;302,265;481,159])

T = tdh_test(pi/2, 0, 0, 101.6) * ...        
    tdh_test(-pi/2, 0, 0, 52.8) * ...        
    tdh_test(0, 0, pi, 0 )

% Q = tdh_test(0, 0,-pi/2, 0) * ...        
%     tdh_test(0, 101.6, 0, 0) * ...        
%     tdh_test(0, 0, 0, 52.8 ) * ...
%     tdh_test(0,0,-pi/2,0) * ...
%     tdh_test(pi, 0,0,0)
