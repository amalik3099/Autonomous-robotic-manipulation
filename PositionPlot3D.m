function [  ] = PositionPlot3D( q )

global axis0vel;
global axis1vel;
global axis2vel;

global axis0current;
global axis1current;
global axis2current;

B = fwkin3001([q(1),q(2),q(3)]);

X = [0; B(1,1); B(1,2); B(1,3)];
Y = [0; B(2,1); B(2,2); B(2,3)];
Z = [0; B(3,1); B(3,2); B(3,3)];



%adds velocity vector at end effector
if size(axis0vel) > 0
    Xdots = jacob1([q(1),q(2),q(3)], [axis0vel(end),axis1vel(end),axis2vel(end)]);
    plot3(X,Y,Z, 'Marker', 'o')
    hold on
    %25 is the scale factor
    quiver3(B(1,3),B(2,3),B(3,3),Xdots(1,1),Xdots(2,1),Xdots(3,1),25)
    hold off
    
    if singularityCheck(axis0current, axis1current, axis2current)
        hold on
        text (100,100, 'Error! Reached Singularity', 'FontSize', 13, 'Color', 'r')
        hold off
    end
end


xlabel('X')
ylabel('Y')
zlabel('Z')

xlim([-400,400])
ylim([-400,400])
zlim([-250,700])

end