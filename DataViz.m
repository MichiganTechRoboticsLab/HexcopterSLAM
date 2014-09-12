

%
% Load Data
%
VectorNav_Logfile = '/home/dereck/Documents/DataSets/signs/vn.csv';
ReadVectorNavLog

Hokuyo_Logfile = '/home/dereck/Documents/DataSets/signs/lidar_data.csv';
ReadHokuyoLog


% %
% % Show the GPS Data
% %
% figure(1)
% clf
% plot(GPS_Longitude, GPS_Lattitude, '.r', 'MarkerSize', 3) 
% plot_google_map('MapType', 'hybrid')  % Dont forget to add to path
% title('GPS Track');
% 
% figure(2)
% clf
% plot3(GPS_Longitude, GPS_Lattitude, GPS_Altitude, '.r', 'MarkerSize', 20) 
% title('GPS Track (3D)');
% grid
% 
% figure(3)
% clf
% subplot(3,1,1);
% plot(GPS_Timestamp, GPS_Longitude, '.r')
% title('GPS Longitude');
% subplot(3,1,2);
% plot(GPS_Timestamp, GPS_Lattitude, '.g')
% title('GPS Latitude');
% subplot(3,1,3);
% plot(GPS_Timestamp, GPS_Altitude, '.b')
% title('GPS Altitude');
% 
% 
% 
% %
% % Show the IMU Data
% %
figure(4)
clf
subplot(3,1,1);
plot(IMU_Timestamp, IMU_Pitch, '.r')
title('IMU Pitch');
subplot(3,1,2);
plot(IMU_Timestamp, IMU_Roll, '.g')
title('GPS Roll');
subplot(3,1,3);
plot(IMU_Timestamp, IMU_Yaw, '.b')
title('IMU Yaw');
% 
% 
% 
% % Identify missing data
% figure(5)
% clf  
% hold on  
% subplot(1,2,1)
% plot(diff(GPS_Timestamp), '.r')
% title('Missing Data Identification (GPS)')
% subplot(1,2,2)
% plot(diff(IMU_Timestamp), '.b')
% title('Missing Data Identification (IMU)')
% 
% 
% 
% Plot all orientations on same spot
figure(6)
clf
PlotPose3D(0,0,0, 0,0,0, 1);
hold on
view(63, 24)
axis([-1 1 -1 1 -1 1])
title('IMU Orientation')
grid

zv = zeros(size(IMU_Pitch));
PlotTraj3D(zv, zv, zv, IMU_Pitch, IMU_Roll, IMU_Yaw, 0.9);
% 
% 
% 
% %
% % Show the interpolated GPS Location for each IMU message
% %
% figure(7)
% clf
% subplot(3,1,1);
% plot(IMU_Timestamp, IMU_MetricPose(:,1), '.r')
% title('IMU Pitch');
% subplot(3,1,2);
% plot(IMU_Timestamp, IMU_MetricPose(:,2), '.g')
% title('GPS Roll');
% subplot(3,1,3);
% plot(IMU_Timestamp, IMU_MetricPose(:,3), '.b')
% title('IMU Yaw');
% 
% 
% 
% %
% % Plot all orientations on the GPS Path
% %
% figure(8)
% clf
% nScale = 0.8;
% hold on
% %view(63, 24)
% axis equal
% grid
% title('IMU Orientation & GPS Pose')
% 
% PlotTraj3D(IMU_MetricPose(:,1), IMU_MetricPose(:,2), IMU_MetricPose(:,3), ...
%            IMU_Pitch, IMU_Roll, IMU_Yaw, nScale);
% 



%
% Plot Lidar Data (2D) 
%

% Grab all range measurements from Lidar data
r = Lidar_Log(:, Lidar_Col_Ranges:end);

% Put all ranges in one column
R = reshape(r', [], 1);

% Make a second column of all of the angles for each measurement
a = Lidar_Angles;
A = repmat(a, size(r,1), 1);

% Generate column for a Timestamp for every point
t1 = Lidar_Log(:, Lidar_Col_Timestamp);
t2 = repmat(t1', size(r,2), 1);
T = reshape(t2, [], 1);

% Remove invalid data
I = or(R >= 60, R <= 0.35);
R(I) = [];
A(I) = [];
T(I) = [];

% Remove all Lidar Data before and after the IMU is available
I = or(T < IMU_Timestamp(1), T > IMU_Timestamp(end));
R(I) = [];
A(I) = [];
T(I) = [];


% Convert to Cartisian Coordinates
[X, Y] = pol2cart(A + pi/2, R);


% DEBUG ========================================================
DEBUGroll = interp1(IMU_Timestamp, IMU_Roll, T);
[X2, Y2] = pol2cart(A + pi/2 - DEBUGroll, R);
[X3, Y3] = pol2cart(A + pi/2 + DEBUGroll, R);
% ================================================================


% Get the GPS based position of the sensor for each hit
P = interp1(GPS_Timestamp, GPS_MetricPose, T);


% Generate Quaternions for linear interpolation of rotations
IMU_Q = angle2quat(IMU_Yaw, IMU_Roll, IMU_Pitch );

% Get the linear interpolation of the orientation from the IMU
Q = interp1(IMU_Timestamp, IMU_Q, T);
[rz, ry, rx] = quat2angle(Q);

% Rotate all points by it's orientation
p1 = quatrotate(Q, [X, Y, zeros(size(X))]);


% Translate all points by it's translation
%p2 = p1 + P;
p2 = p1;


%
% Plots ( Lidar )
%

% % Plot of all hits on a 2D polar plot
% figure(11);
% clf;
% polar(A, R, '.b');
% view(-90,90);


% % Plot the interpolated pose points and the GPS path
% figure(12)
% clf
% plot3(P(:,1), P(:,2), P(:,3), '.r', 'MarkerSize', 20) 
% hold on;
% plot3(GPS_MetricPose(:,1), GPS_MetricPose(:,2), GPS_MetricPose(:,3), '-b') 
% title('Lidar Pose & GPS Track');
% grid
% 
% 
% % Plot the interpolated orientations and the original orientation path
% figure(13)
% clf
% nScale = 0.8;
% hold on
% %view(63, 24)
% axis equal
% grid
% title('IMU Orientation & GPS Pose')
% 
% PlotTraj3D(IMU_MetricPose(:,1), IMU_MetricPose(:,2), IMU_MetricPose(:,3), ...
%            IMU_Pitch, IMU_Roll, IMU_Yaw, nScale);
%        
% PlotTraj3D(P(:,1), P(:,2), P(:,3), ...
%            rx, ry, rz, nScale*1.2);


% Plot a single scan  
figure(14); 
figure(15);
for j = 1:1000:(size(p2,1)-1000)
    set(0, 'CurrentFigure', 14);
    clf ;
    nScale = 0.8;
    i = j + (1:1000);
    plot3(p2(i,1), p2(i,2), p2(i,3), '.b', 'MarkerSize', 2);
    grid;
    axis equal;
    hold on;   
    axis([-1 1 -1 1 -1 1] * 10);
    PlotTraj3D(0, 0, 0, rx(j), ry(j), rz(j), nScale); 
    %PlotTraj3D(P(i,1), P(i,2), P(i,3), rx(i), ry(i), rz(i), nScale); 
    %plot3(P(1:i,1), P(1:i,2), P(1:i,3), '-k', 'MarkerSize', 2) ;

    view([180 0])
    drawnow;
    

    set(0, 'CurrentFigure', 15);
    clf
    plot(X(i), -Y(i), '.b', 'MarkerSize', 2);
    hold on;
    plot(X2(i), -Y2(i), '.r', 'MarkerSize', 2);
    plot(X3(i), -Y3(i), '.g', 'MarkerSize', 2);
    plot(0,0, '.k');
    axis([-1 1 -1 1 -6 1] * 5);
    
    pause(0.01);
end

% figure(15)
% clf 
% grid
% axis equal
% plot3(p2(:,1), p2(:,2), p2(:,3), '.b', 'MarkerSize', 2)
% hold on;      
%        
       
       