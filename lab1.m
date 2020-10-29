%%
% RBE3001 - Laboratory 1 
% 
% Instructions
% ------------
% Welcome again! This MATLAB script is your starting point for Lab
% 1 of RBE3001. The sample code below demonstrates how to establish
% communication between this script and the Nucleo firmware, send
% setpoint commands and receive sensor data.
% 
% IMPORTANT - understanding the code below requires being familiar
% with the Nucleo firmware. Read that code first.

% Lines 15-37 perform necessary library initializations. You can skip reading
% to line 38.
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

  % The following code generates a sinusoidal trajectory to be
  % executed on joint 1 of the arm and iteratively sends the list of
  % setpoints to the Nucleo firmware. 
  viaPts = [0, -400, 400, -400, 400, 0];
  tic
  n = 0;
  for k = viaPts
      
      while toc < n + 1
          packet = zeros(15, 1, 'single');
          packet(1) = k;
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
              csvValues = transpose(returnPacket);
              dlmwrite('PacketData01.csv',csvValues,'-append')
              % plots base joint angle vs time
              data = csvread('PacketData01.csv');
              dataSize = (size(data(1:end,1)))-1;
              time = 0:dataSize;
              delete baseAngles01.csv;
              for j = 1:dataSize+1
                time(j) = (toc/time(end))*j;
                data(j) = data(j)*(2*pi)/4096;
                dlmwrite('baseAngles01.csv',data(j,1),'-append')
              end
              plot(time,data(1:end,1))
              xlabel("Time (seconds)")
              ylabel("Base Joint Angle (radians)")

          end

          for x = 0:3
              packet((x*3)+1)=0.1;
              packet((x*3)+2)=0;
              packet((x*3)+3)=0;
          end

          toc
          %pause(1) %timeit(returnPacket) !FIXME why is this needed?
      end
      n = n+1;
  end
  
catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end

% Clear up memory upon termination
pp.shutdown()

toc
