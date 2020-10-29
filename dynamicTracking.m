clear
clear java
clear classes;

vid = hex2dec('3742');
pid = hex2dec('0007');

disp (vid);
disp (pid);

javaaddpath ../lib/SimplePacketComsJavaFat-0.6.4.jar;
import edu.wpi.SimplePacketComs.*;
import edu.wpi.SimplePacketComs.device.*;
import edu.wpi.SimplePacketComs.phy.*;
import java.util.*;
import org.hid4java.*;
version -java
myHIDSimplePacketComs=HIDfactory.get();
myHIDSimplePacketComs.setPid(pid);
myHIDSimplePacketComs.setVid(vid);
myHIDSimplePacketComs.connect();

delete PacketData01.csv;
delete baseAngles01.csv;
dlmwrite('baseAngles01.csv',0,'-append')

% Create a PacketProcessor object to send data to the nucleo firmware
pp = PacketProcessor(myHIDSimplePacketComs);

%Initialize camera
load('calibrationSession.mat');
if exist('cam') == 0
    cam = webcam();
end
camParameters;
t0_to_checker = tdh_test(pi/2, 0, 0, 101.6) * ...
    tdh_test(-pi/2, 0, 0, 52.8) * ...
    tdh_test(0, 0, pi, 0 );
T_cam_to_checker = [
    0.0118   -0.8305    0.5568   96.5275;
    0.9998    0.0170    0.0042  105.2922;
    -0.0129    0.5567    0.8306  277.6281;
    0         0         0    1.0000];

try
    SERV_ID = 01;
    STATUS_SERV_ID = 03;            % we will be talking to server ID 01 on
    % the Nucleo
    
    DEBUG   = false;          % enables/disables debug prints
    
    % Instantiate a packet - the following instruction allocates 64
    % bytes for this purpose. Recall that the HID interface supports
    % packet sizes up to 64 bytes.
    packet = zeros(15, 1, 'single');
    
    %Sends PID values in packet to PidConfigServer
    for x = 0:3
        packet((x*3)+1)=0.005;%0.007
        packet((x*3)+2)=0.00000001;
        packet((x*3)+3)=0.03;%0.03
    end
    pp.write(02, packet);
    
    csvValues = [];
    packetDataTime = [];
    axis0 = [];
    axis1 = [];
    axis2 = [];
    endEffectorX = [];
    endEffectorY = [];
    endEffectorZ = [];
    global axis0vel
    global axis1vel
    global axis2vel
    global axis0current
    global axis1current
    global axis2current
    axis0current = 0;
    axis1current = 0;
    axis2current = 0;
    tic
    n = 0;
    state = 'searching';
    counter = 0;
    homeX = 175;
    homeY = 0;
    homeZ = 90;
    timeSpan = 1.6;
    timeBetweenSetpoints = timeSpan/7;
    
    endEffectorX = homeX;
    endEffectorY = homeY;
    endEffectorZ = homeZ;
    while 1
        switch(state)
            case 'searching'
                imOrig = snapshot(cam);
                objectDetails = findDynamicObject(imOrig,t0_to_checker, T_cam_to_checker, calibrationSession.CameraParameters);
                state = 'sortingObjects';
                
            case 'sortingObjects'
                %imOrig = snapshot(cam);
                %findDynamicObject(imOrig,t0_to_checker, T_cam_to_checker, calibrationSession.CameraParameters)
                disp('sortingObjects')
                objectLocation = [objectDetails(1,2) , objectDetails(1,3), 30];
                state = 'trajectoryPlanToObject';
                
            case 'trajectoryPlanToObject'
                %imOrig = snapshot(cam);
                %findDynamicObject(imOrig,t0_to_checker, T_cam_to_checker, calibrationSession.CameraParameters)
                startTime = toc;
                disp('trajectory planning')
                quinPosition0 = trajectoryPlanQuin(startTime, startTime + timeSpan, 0, 0, homeX, 230-objectLocation(1)+52.8,0,0);
                quinPosition1 = trajectoryPlanQuin(startTime, startTime + timeSpan, 0, 0, homeY, objectLocation(2)+101.6,0,0);
                quinPosition2 = trajectoryPlanQuin(startTime, startTime + timeSpan, 0, 0, homeZ, objectLocation(3),0,0);
                state = 'moving';
                
            case 'moving'
                for k = 1:size(quinPosition1,2)
                    thetas = ikin([quinPosition0(k), quinPosition1(k), quinPosition2(k)]);
                    theta1 = thetas(1);
                    theta2 = thetas(2);
                    theta3 = thetas(3);
                    timer = toc;
                    
                    packet = zeros(15, 1, 'single');
                    packet(1) = theta1;
                    packet(4) = theta2;
                    packet(7) = theta3;
                    packet(11) = 1;
                    
                    %disp(packet)
                    
                    % Send packet to the server and get the response
                    %pp.write sends a 15 float packet to the micro controller
                    %pp.write(STATUS_SERV_ID, packet);
                    
                    if ~singularityCheck(axis0current, axis1current, axis2current)
                        pp.write(SERV_ID, packet);
                    end
                    
                    pause(0.003); % Minimum amount of time required between write and read
                    
                    %pp.read reads a returned 15 float packet from the nucleo.
                    returnPacket = pp.read(SERV_ID);
                    %toc

                    
                    if DEBUG
                        disp('Sent Packet:');
                        disp(packet);
                        disp('Received Packet:');
                        disp(returnPacket);
                    end
                    toc;
                    %while toc - timer< timeBetweenSetpoints
                        %imOrig = snapshot(cam);
                        %findDynamicObject(imOrig,t0_to_checker, T_cam_to_checker, calibrationSession.CameraParameters)
                    %end
                    %n = n+1;
                end
                state = 'searching';
                
            otherwise
                state = 'searching';
        end
    end
%end
%     dlmwrite('baseAngles01.csv',angles,'-append')
%     dlmwrite('PacketData01.csv',transpose(csvValues),'-append')
 catch exception
     getReport(exception)
%     disp('Exited on error, clean shutdown');
 end
%
% Clear up memory upon termination
pp.shutdown()
%
% toc