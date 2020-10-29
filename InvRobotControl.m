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
    
    % The following code generates a sinusoidal trajectory to be
    % executed on joint 1 of the arm and iteratively sends the list of
    % setpoints to the Nucleo firmware.
    pos0 = [57.8, 178.4272, -6.1063,91.5213, 181.4136]; %  HomeX = 175 (57.8 to componsate for 180-x+52.8)
    pos1 = [0-101.6, -237.5360, -231.5775,-109.4344, 16.1003]; %  HomeY = 0
    pos2 = [-15, -15 , -15, -15, -15]; %     HomeZ = 30
    
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
    
    for w = 1:size(pos1,2)-1
        startTime = toc;
        % Quintic Polynomial
        %         quinPosition0 = trajectoryPlanQuin(startTime, toc + 0.8, 0, 0, pos0(w), pos0(w+1),0,0);
        %         quinPosition1 = trajectoryPlanQuin(startTime, toc + 0.8, 0, 0, pos1(w), pos1(w+1),0,0);
        %         quinPosition2 = trajectoryPlanQuin(startTime, toc + 0.8, 0, 0, pos2(w), pos2(w+1),0,0);
        % Cubic polynomial
        cubicPosition0 = trajectoryPlan(startTime, toc + 1, 0, 0, 180-pos0(w)+52.8, 180-pos0(w+1)+52.8);
        cubicPosition1 = trajectoryPlan(startTime, toc + 1, 0, 0, pos1(w)+101.6, pos1(w+1)+101.6);
        cubicPosition2 = trajectoryPlan(startTime, toc + 1, 0, 0, pos2(w), pos2(w+1));
        
        for k = 1:size(cubicPosition1,2)
            thetas = ikin([cubicPosition0(k), cubicPosition1(k), cubicPosition2(k)]);
            theta1 = thetas(1);
            theta2 = thetas(2);
            theta3 = thetas(3);
            
            while toc < n + 1
                imOrig = snapshot(cam);
                objectDetails = findObjs(imOrig,t0_to_checker, T_cam_to_checker, calibrationSession.CameraParameters)
                
                packet = zeros(15, 1, 'single');
                packet(1) = theta1;
                packet(4) = theta2;
                packet(7) = theta3; %pos2(k)
                if mod(n,2) == 1
                    packet(11) = 0;
                end
                if mod(n,2) == 0
                    packet(11) = 1;
                end
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
                
                % takes returnPacket data and adds it to csv file
                csvValues = [csvValues, returnPacket];
                
                packetDataTime = [packetDataTime,toc];
                % records joint angles and timestamp to csv file
                %data = csvread('PacketData01.csv');
                
                axis0current = returnPacket(1)*(2*pi)/4096;
                axis1current = returnPacket(4)*(2*pi)/4096;
                axis2current = returnPacket(7)*(2*pi)/4096;
                
                axis0vel = diff(axis0);
                axis1vel = diff(axis1);
                axis2vel = diff(axis2);
                
                axis0acc = diff(axis0vel);
                axis1acc = diff(axis1vel);
                axis2acc = diff(axis2vel);
                
                axis0 = [axis0, axis0current];
                axis1 = [axis1, axis1current];
                axis2 = [axis2, axis2current];
                
                B = fwkin3001([axis0current,axis1current, axis2current]);
                endEffectorX = [endEffectorX, B(1,3)];
                endEffectorY = [endEffectorY, B(2,3)];
                endEffectorZ = [endEffectorZ, B(3,3)];
                angles = [transpose(packetDataTime),transpose(axis0),transpose(axis1),transpose(axis2),...
                    transpose(endEffectorX), transpose(endEffectorY),transpose(endEffectorZ)];
                
                %anglesData = csvread('baseAngles01.csv');
                
                %3D plot
                %PositionPlot3D([axis0current,axis1current, axis2current])
                
                
                %3D plot of end effector path
                %                 x = [packetDataTime, packetDataTime, packetDataTime]
                %                 y = [endEffectorX, endEffectorY, endEffectorZ]
                %                 plot3(endEffectorX, endEffectorY, endEffectorZ)
                %                 xlim([-300,300])
                %                 ylim([-300,300])
                %                 zlim([-150,600])
                
                % Plot for joint angles
                %                 x1 = packetDataTime;
                %                 y0 = axis0;
                %                 y1 = axis1;
                %                 y2 = axis2;
                %                 plot(x1,y0,'b', 'LineWidth', 2)
                %                 hold on
                %                 plot(x1,y1,'r', 'LineWidth', 2)
                %                 hold off
                %                 hold on
                %                 plot(x1,y2,'g', 'LineWidth', 2)
                %                 hold off
                %                 xlabel("Time(seconds)")
                %                 ylabel("Angle(rad)")
                %                 legend("Joint 1", "Joint 2", "Joint 3")
                
                %Plot of End Effector X and Y and Z positions
                %                 y8 = endEffectorX;
                %                 y9 = endEffectorY;
                %                 y10 = endEffectorZ;
                %                 plot(x1,y8,'b','LineWidth',2)
                %                 hold on
                %                 plot(x1,y9,'r','LineWidth',2)
                %                 hold off
                %                 hold on
                %                 plot(x1,y10,'g','LineWidth',2)
                %                 hold off
                %                 xlabel("Time(seconds)")
                %                 ylabel("End Effector Position(mm)")
                %                 legend("End Effector X", "End Effector Y", "End Effector Z")
                
                %Plot of End Effector X and Z positions
                %                   y3 = endEffectorX;
                %                   y4 = endEffectorZ;
                %                   plot(x1,y3,'b','LineWidth',2)
                %                   hold on
                %                   plot(x1,y4,'r','LineWidth',2)
                %                   hold off
                %                   xlabel("Time(seconds)")
                %                   ylabel("End Effector Position(mm)")
                %                   legend("End Effector X", "End Effector Z")
                
                % Plot of Joint Velocities
                %                                 x2 = packetDataTime(:,1:end-2);
                %                                 y5 = axis0vel;
                %                                 y6 = axis1vel;
                %                                 y7 = axis2vel;
                %                                 plot(x2,y5,'b', 'LineWidth', 2)
                %                                 hold on
                %                                 plot(x2,y6,'r', 'LineWidth', 2)
                %                                 hold off
                %                                 hold on
                %                                 plot(x2,y7,'g', 'LineWidth', 2)
                %                                 hold off
                %                                 xlabel("Time(seconds)")
                %                                 ylabel("Joint velocities(mm/sec)")
                %                                 legend("Joint 1", "Joint 2", "Joint 3")
                
                % Plot of Joint Acceleration
                %                 x2 = packetDataTime(:,1:end-3);
                %                 y5 = axis0acc;
                %                 y6 = axis1acc;
                %                 y7 = axis2acc;
                %                 plot(x2,y5,'b', 'LineWidth', 2)
                %                 hold on
                %                 plot(x2,y6,'r', 'LineWidth', 2)
                %                 hold off
                %                 hold on
                %                 plot(x2,y7,'g', 'LineWidth', 2)
                %                 hold off
                %                 xlabel("Time(seconds)")
                %                 ylabel("Joint Acceleration(mm/sec^2)")
                %                 legend("Joint 1", "Joint 2", "Joint 3")
                
                
                % Plot of End Effector X position and setpoint points
                %                 y8 = endEffectorX;
                %                 plot(x1,y8,'b','LineWidth',2)
                %                 hold on
                %                 plot(3.0, 275,'r*')
                %                 hold of
                %                 hold on
                %                 plot(1.6, 115.91,'g*')
                %                 hold off
                %                 hold on
                %                 plot(3.77, 185,'m*')
                %                 hold off
                %                 xlabel("Time(seconds)")
                %                 ylabel("End Effector Position(mm)")
                %                 legend("End Effector X")
                
                % plots of step responses for three joints
                %           subplot(3,1,1);
                %           x = packetDataTime;
                %           y1 = data(1:end,1);
                %           plot(x,y1,'r','LineWidth',2)
                %           y=pos0(k);
                %           h=line([0,size(pos0,2)],[y,y]);
                %           h.LineWidth = 2;
                %
                %           subplot(3,1,2);
                %           y2 = data(1:end,3);
                %           plot(x,y2,'r','LineWidth',2)
                %           y=pos1(k);
                %           l=line([0,size(pos0,2)],[y,y]);
                %           l.LineWidth = 2;
                %
                %           subplot(3,1,3);
                %           y3 = data(1:end,5);
                %           plot(x,y3,'r','LineWidth',2)
                %           y=pos2(k);
                %           e=line([0,size(pos0,2)],[y,y]);
                %           e.LineWidth = 2;
                
                
                
                if DEBUG
                    disp('Sent Packet:');
                    disp(packet);
                    disp('Received Packet:');
                    disp(returnPacket);
                end
                
                toc
                %pause(1) %timeit(returnPacket) !FIXME why is this needed?
            end
            n = n+1;
        end
    end
    dlmwrite('baseAngles01.csv',angles,'-append')
    dlmwrite('PacketData01.csv',transpose(csvValues),'-append')
    %-------------------------------------
    %   anglesData = csvread('baseAngles01.csv');
    %   y8 = endEffectorX;
    %   plot(x1,y8,'b','LineWidth',2)
    %   for l = 1:30
    %       hold on
    %       plot(anglesData(l*20,1), anglesData(l*20,5),'r*')
    %       hold off
    %   end
    %   xlabel("Time(seconds)")
    %   ylabel("End Effector Position(mm)")
    %   legend("End Effector X")
    %---------------------------------
catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end

% Clear up memory upon termination
pp.shutdown()

toc