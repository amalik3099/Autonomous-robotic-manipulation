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
%time = 0:6;
try
  SERV_ID = 03;            % we will be talking to server ID 01 on
                           % the Nucleo

  DEBUG   = true;          % enables/disables debug prints

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
  pos0 = [0, 0, 0, 0, 0]; % 0, 296.25, 786.5, 56.17, -232, -709.75, 0       		
  pos1 = [0, 124.05, 141.55, 600, 124.05]; % 0, 136, 288.75, 149.05, 497.25, 269.25, 0       
  pos2 = [0, -356.82, 306.18, -277.32, -356.82]; % 0, 34.75,707.75, 333.43, 61.25, 662.25, 0 
  % 0	124.05	-356.82 
  %0	141.55	306.18
  %0	772.3	-277.32
  csvValues = [];
  packetDataTime = [];
  axis0 = [];
  axis1 = [];
  axis2 = [];
  endEffectorX = [];
  endEffectorY = [];
  endEffectorZ = [];
  axis0velocities = [];
  axis1velocities = [];
  axis2velocities = [];
  tic
  n = 0;
  for w = 1:size(pos1,2)-1
      startTime = toc;
      cubicPosition0 = trajectoryPlan(startTime, toc + 0.15, 0, 0, pos0(w), pos0(w+1));
      cubicPosition1 = trajectoryPlan(startTime, toc + 0.15, 0, 0, pos1(w), pos1(w+1));
      cubicPosition2 = trajectoryPlan(startTime, toc + 0.15, 0, 0, pos2(w), pos2(w+1));

      for k = 1:size(cubicPosition1,2)
          while toc < n + 1
              packet = zeros(15, 1, 'single');
              packet(1) = cubicPosition0(k);               
              packet(4) = cubicPosition1(k);
              packet(7) = cubicPosition2(k); %pos2(k)
              % Send packet to the server and get the response      
              %pp.write sends a 15 float packet to the micro controller
               pp.write(SERV_ID, packet); 
               pp.write(01, packet);

               pause(0.003); % Minimum amount of time required between write and read

               %pp.read reads a returned 15 float packet from the nucleo.
               returnPacket = pp.read(SERV_ID);
              toc

              if DEBUG
                  disp('Sent Packet:');
                  disp(packet);
                  disp('Received Packet:');
                  disp(returnPacket);
                  % takes returnPacket data and adds it to csv file
                  csvValues = [csvValues, returnPacket];

                  packetDataTime = [packetDataTime,toc];
                  % records joint angles and timestamp to csv file
                  %data = csvread('PacketData01.csv');

                  axis0current = returnPacket(1)*(2*pi)/4096;
                  axis1current = returnPacket(3)*(2*pi)/4096;
                  axis2current = returnPacket(5)*(2*pi)/4096;

                  axis0vel = diff(axis0);
                  axis1vel = diff(axis1);
                  axis2vel = diff(axis2);

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
                  %PositionPlot3D([axis0current,axis1current, axis2current])

    % plots for end effector positions and joint angles and joint velocities
    % Plot for joint angles
                  %subplot(3,1,1)
                    x1 = packetDataTime;
                  y0 = axis0;
                  y1 = axis1;
                  y2 = axis2;
                  plot(x1,y0,'b', 'LineWidth', 2)
                  hold on
                  plot(x1,y1,'r', 'LineWidth', 2)
                  hold off
                  hold on
                  plot(x1,y2,'g', 'LineWidth', 2)
                  hold off
                  xlabel("Time(seconds)")
                  ylabel("Angle(rad)")
                  legend("Joint 1", "Joint 2", "Joint 3")
    %Plot of End Effector X and Z positions              
%                   subplot(3,1,2)
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
%                  subplot(3, 1, 3)
%                   x2 = packetDataTime(:,1:end-2);
%                   y5 = axis0vel;
%                   y6 = axis1vel;
%                   y7 = axis2vel;
%                   plot(x2,y5,'b', 'LineWidth', 2)
%                   hold on
%                   plot(x2,y6,'r', 'LineWidth', 2)
%                   hold off
%                   hold on
%                   plot(x2,y7,'g', 'LineWidth', 2)
%                   hold off
%                   xlabel("Time(seconds)")
%                   ylabel("Joint velocities(mm/sec)")
%                   legend("Joint 1", "Joint 2", "Joint 3")
                  
%                  subplot(4,1,4)
    % Plot of End Effector X position and setpoint points
%                     y8 = endEffectorX;
%                     plot(x1,y8,'b','LineWidth',2)
%                     hold on 
%                     plot(3.0, 275,'r*')
%                     hold of
%                   hold on 
%                   plot(1.6, 115.91,'g*')
%                   hold off
%                   hold on 
%                   plot(3.77, 185,'m*')
%                   hold off
%                   xlabel("Time(seconds)")
%                   ylabel("End Effector Position(mm)")
%                   legend("End Effector X")

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