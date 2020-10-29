delete tictoc.csv;
pos0 = [175, 286]; % 0, 296.25, 786.5, 56.17, -232, -709.75, 0
pos1 = [0, 88]; % 0, 136, 288.75, 149.05, 497.25, 269.25, 0
pos2 = [30, 52]; % 0, 34.75,707.75, 333.43, 61.25, 662.25, 0

thetas = ikin([pos0(k), pos1(k), pos2(k)]);
theta1 = thetas(1);
theta2 = thetas(2);
theta3 = thetas(3);

timeStamp = [];

loopTime = [];
for g=1:500
    tic
    initialTime = toc;
    
    packet = zeros(15, 1, 'single');
    packet(1) = theta1;
    packet(4) = theta2;
    packet(7) = theta3; %pos2(k)
    pp.write(01, packet);
    
    pause(0.003); % Minimum amount of time required between write and read
    
    %pp.read reads a returned 15 float packet from the nucleo.
    returnPacket = pp.read(01);
    endTime = toc;
    
    loopTime = [loopTime, endTime-initialTime];
    
    dlmwrite('tictoc.csv',transpose(loopTime),'-append')
    
end
averageTimes = mean(loopTime)
standardDeviation = std(loopTime)
minTime = min(loopTime)
maxTime = max(loopTime)

histogram(loopTime)
