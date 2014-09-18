% PlotRawFusion.m
%  Very basic fusion of the raw Vectornav and Hokuyo sensor data without heuristics.
%  Plots the interpolated lidar pos over the GPS track
%  Plots the interpolated lidar pos and orientation over the GPS track
%  Plots the Full point cloud.
%  Plots the orientation, pose, and each scan one at a time. (animation)

% Load Data
Header
ReadVectorNavLog
ReadHokuyoLog


%
% Fuse GPS and IMU data
%

% Remove all Lidar Data before and after the IMU is available
I = or(Lidar_Timestamp < IMU_Timestamp(1), Lidar_Timestamp > IMU_Timestamp(end));
Lidar_ScanIndex(I) = [];
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
% Plots
%

% Plot the interpolated pose points and the GPS path
figure(1)
clf
plot3(P(:,1), P(:,2), P(:,3), '.r', 'MarkerSize', 40) 
hold on;
plot3(GPS_MetricPose(:,1), GPS_MetricPose(:,2), GPS_MetricPose(:,3), '-b') 
title('Lidar Pose & GPS Track');
grid



% Plot the interpolated orientations and the original orientation path
figure(2)
clf
nScale = 0.8;
hold on
axis equal
grid
title('IMU Orientation & GPS Pose')

PlotTraj3D(IMU_MetricPose(:,1), IMU_MetricPose(:,2), IMU_MetricPose(:,3), ...
           IMU_Pitch, IMU_Roll, IMU_Yaw, nScale);
       
PlotTraj3D(P(:,1), P(:,2), P(:,3), rx, ry, rz, nScale);



% Plot the full point cloud
figure(3)
clf 
grid
axis equal
plot3(pointcloud(:,1), pointcloud(:,2), pointcloud(:,3), '.b', 'MarkerSize', 2)
hold on; 



% Plot a single scan without translation
figure(4); 
nScanIndex = unique(Lidar_ScanIndex);
for i = 1:length(nScanIndex)
    
    % Retrieve each scan's points
    nIndex = nScanIndex(i);
    I = nIndex == Lidar_ScanIndex;
    
    % Plot this scan
    clf ;
    plot3(p1(I,1), p1(I,2), p1(I,3), '.b', 'MarkerSize', 2);
    grid;
    hold on;       
    title('Lidar Scan Animation')
    
    % Show the oreintation at this scan
    nScale = 5;
    rx1 = rx(I);
    ry1 = ry(I);
    rz1 = rz(I);
    PlotTraj3D(0, 0, 0, rx1(1), ry1(1), rz1(1), nScale); 

    axis([-1 1 -1 1 -1 1] * 15);
    %axis equal;
    
    %view([-90 90]) % From Top
    %view([-90 0])  % From Left Side
    %view([0 0])    % From Back
    view([-25 -30]) % Signs vn_1 custom
    
    drawnow;
    pause(0.01);
end
         


% Plot a single scan with translation
figure(5); 
nScanIndex = unique(Lidar_ScanIndex);
for i = 1:length(nScanIndex)
    
    % Retrieve each scan's points
    nIndex = nScanIndex(i);
    I = nIndex == Lidar_ScanIndex;
    
    % Plot this scan
    clf ;
    plot3(pointcloud(I,1), pointcloud(I,2), pointcloud(I,3), '.b', 'MarkerSize', 2);
    grid;
    axis equal;
    hold on;       
    axis([-1 1 -1 1 -1 .1] * 20);
    title('Lidar Scan Animation')
    
    % Show the oreintation at this scan
    nScale = 0.8;
    P1  = P(I,:);
    rx1 = rx(I);
    ry1 = ry(I);
    rz1 = rz(I);
    PlotTraj3D(P1(1,1), P1(1,3), P1(1,3), rx1(1), ry1(1), rz1(1), nScale); 

    view([-90 90]) % From Top
    %view([-90 0])  % From Left Side
    %view([0 0])    % From Back
    
    drawnow;
    pause(0.01);
end
       
       