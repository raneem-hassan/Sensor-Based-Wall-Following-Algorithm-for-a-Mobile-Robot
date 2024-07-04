clc
clear 

theta0 = -17;

Time = [1];
Theta = [theta0];
Theta_Output = [theta0];
t = 1;
dt =0.1;
wall = 10;
width = 7.5;
height = 5;
p1 = [0, 0];
p2 = [width, 0];
p3 = [width, height];
p4 = [0, height];
rectangleVertices = [p1; p2; p3;p4; p1];

% plot (rectangleVertices(:,1),rectangleVertices(:,2))
% hold on
% axis equal;
epsilon = 0.01;
% while abs(Theta(t)) > epsilon
i = 1;

while t < 10
    rotationMatrix = [cosd(Theta(i)), -sind(Theta(i)); sind(Theta(i)), cosd(Theta(i))];
    rectangleVertices = (rotationMatrix * rectangleVertices')';
    m = (rectangleVertices(4,2)-rectangleVertices(1,2))/(rectangleVertices(4,1)-rectangleVertices(1,1));
    x1 = ((wall-rectangleVertices(4,2))/m)+rectangleVertices(4,1);
    d1 = sqrt((x1-rectangleVertices(4,1))^2 + (10-rectangleVertices(4,2))^2);
    x2 = ((wall-rectangleVertices(3,2))/m)+rectangleVertices(3,1);
    d2 = sqrt((x2-rectangleVertices(3,1))^2 + (10-rectangleVertices(3,2))^2);
    theta = atand((d1-d2)/width);
%     Theta = [Theta theta];
    kP = 0.1;
    kD = 0;
    kI = 0;
    PControl = theta * kP;
    DControl = ((theta - Theta_Output(i))/dt)*kD;
    IControl = sum (Theta_Output(1:i))*kI;
    PIDControl = PControl+IControl+DControl;
    Theta = [Theta -PIDControl];
    Theta_Output = [Theta_Output theta];
    t = t+dt;
    i = i +1;
    Time = [Time t];
%while (Theta(t)~= 0)    
end

% plot (Time,Theta_Output,Time ,zeros (1,length(Time)),'r--') 
% axis([1 10 theta0 max(Theta_Output)+1])
% ylabel('Theta [degrees]')
% xlabel('time [s]')
% title('PID Controller Response (kP = 0.1, kD = -0.06, kI = 0)')
% legend('Theta','Setpoint','Location','southeast')
% figure;

Wall = [0,10;100,10;100,10.5;0,10.5;0,10];



width = 7.5;
height = 5;
p1 = [0, -height/2];
p2 = [width, -height/2];
p3 = [width, height/2];
p4 = [0, height/2];
rectangleVertices = [p1; p2; p3;p4; p1];
translationIncrement = [1, 0]; % [dx, dy]
nF = 5; % Number of frames
videoFile = 'movie0.1_0.avi';
writerObj = VideoWriter(videoFile);
open(writerObj);

for i=1:length (Theta)
    subplot (2,1,1)
    plot ([0; 100],[0; 0],'r--')
    hold on 
    fill (Wall(:,1),Wall(:,2),'k')
    axis ([0 100 -2 10.5])
    hold on 
    rotationMatrix = [cosd(Theta(i)), -sind(Theta(i)); sind(Theta(i)), cosd(Theta(i))];
    rectangleVertices = (rotationMatrix * rectangleVertices')';
    translationIncrement =(rotationMatrix * translationIncrement')'
    rectangleVertices = rectangleVertices + translationIncrement;
    fill (rectangleVertices(:,1),rectangleVertices(:,2),'b')
    axis equal
    hold on
    drawnow
    hold off
    
    subplot (2,1,2)
    plot (Time(i),Theta_Output(i),'b.',Time,zeros(1,length(Time)),'r--')
    xlim ([1,10])
    ylim ([min(Theta_Output)-1,max(Theta_Output)+1])
    title ('P Controller Response')
    ylabel('theta [degrees]')
    xlabel('time [s]')
    drawnow
    hold all
    Case(i) = getframe(gcf);
    writeVideo(writerObj, Case(i));  % Write the frame to the video
%     Case(i) = getframe(gcf);
%     hold off
end
close(writerObj);