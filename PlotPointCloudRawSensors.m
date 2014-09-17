%
% Load Data
%
VectorNav_Logfile = '/home/dereck/Documents/DataSets/signs/vn.csv';
ReadVectorNavLog

Hokuyo_Logfile = '/home/dereck/Documents/DataSets/signs/lidar_data.csv';
ReadHokuyoLog


%
% Fuse GPS and IMU data
%

% Remove all Lidar Data before and after the IMU is available
I = or(Lidar_Timestamp < IMU_Timestamp(1), Lidar_Timestamp > IMU_Timestamp(end));
Lidar_Timestamp(I) = [];
Lidar_Angles(I) = [];
Lidar_Ranges(I) = [];
Lidar_X(I) = [];
Lidar_Y(I) = [];
clear I

% Get the GPS based position of the sensor for each hit
P = interp1(GPS_Timestamp, GPS_MetricPose, Lidar_Timestamp);

% Get the linear interpolation of the orientation from the IMU
Q = interp1(IMU_Timestamp, IMU_Q, Lidar_Timestamp);
[rz, ry, rx] = quat2angle(Q);

% Rotate all points by their orientation
p1 = quatrotate(Q, [Lidar_X, Lidar_Y, zeros(size(Lidar_X))]);

% Translate all points by their translation
pointcloud = p1 + P;




%
% Plots ( Lidar )
%

% Plot of all hits on a 2D polar plot
figure(1);
clf;
polar(Lidar_Angles, Lidar_Ranges, '.b');
view(-90,90);


% Plot the interpolated pose points and the GPS path
figure(2)
clf
plot3(P(:,1), P(:,2), P(:,3), '.r', 'MarkerSize', 40) 
hold on;
plot3(GPS_MetricPose(:,1), GPS_MetricPose(:,2), GPS_MetricPose(:,3), '-b') 
title('Lidar Pose & GPS Track');
grid


% Plot the interpolated orientations and the original orientation path
figure(3)
clf
nScale = 0.8;
hold on
axis equal
grid
title('IMU Orientation & GPS Pose')

PlotTraj3D(IMU_MetricPose(:,1), IMU_MetricPose(:,2), IMU_MetricPose(:,3), ...
           IMU_Pitch, IMU_Roll, IMU_Yaw, nScale);
       
PlotTraj3D(P(:,1), P(:,2), P(:,3), rx, ry, rz, nScale);


% Plot a single scan
figure(4); 
clf ;
for j = 1:1000:(size(pointcloud,1)-1000)
    
    i = j + (1:1000);
    plot3(p1(i,1), p1(i,2), p1(i,3), '.b', 'MarkerSize', 2);
    grid;
    axis equal;
    hold on;       
    axis([-1 1 -1 1 -1 .1] * 20);
    title('Lidar Scan Animation')
    
    nScale = 0.8;
    PlotTraj3D(0, 0, 0, rx(j), ry(j), rz(j), nScale); 

    %view([-90 90]) % From Top
    %view([-90 0])  % From Left Side
    view([0 0])    % From Back
    
    drawnow;
    pause(0.01);
    clf
end
    

figure(5)
clf 
grid
axis equal
plot3(pointcloud(:,1), pointcloud(:,2), pointcloud(:,3), '.b', 'MarkerSize', 2)
hold on;      
       
       