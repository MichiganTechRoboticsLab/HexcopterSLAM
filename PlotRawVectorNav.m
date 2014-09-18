% PlotRawVectorNav.m
%  Sample code for displaying the raw vector nav data. Currently 8 different plots:
%  GPS path over a google map
%  3D plot of gps path
%  Raw sensor data for the LLA and IMU signals
%  Missing data plot (based on timestamps)
%  A plot of the orientations from the IMU in 3D without translation.
%  A Plot of the flat (metric) gps track interpolated for each IMU message (accounts for short missing GPS locks)
%  A plot of the orientations from the IMU over the GPS track


% Load Data
Header
ReadVectorNavLog


% Add google map plot function to path if not already present
if ~exist('plot_google_map.m', 'file')
    if exist('plot_google_map', 'dir') 
        addpath('plot_google_map');
    end
end


% Plot GP track over a Google Maps image
figure(1)
clf
plot(GPS_Longitude, GPS_Lattitude, '.r', 'MarkerSize', 3) 
plot_google_map('MapType', 'hybrid')  % Dont forget to add to path
title('GPS Track');


% Plot the GPS track in 3D
figure(2)
clf
plot3(GPS_Longitude, GPS_Lattitude, GPS_Altitude, '.r', 'MarkerSize', 20) 
title('GPS Track (3D)');
grid


% Plot the raw GPS location data
figure(3)
clf
subplot(3,1,1);
plot(GPS_Timestamp, GPS_Longitude, '.r')
title('GPS Longitude');
subplot(3,1,2);
plot(GPS_Timestamp, GPS_Lattitude, '.g')
title('GPS Latitude');
subplot(3,1,3);
plot(GPS_Timestamp, GPS_Altitude, '.b')
title('GPS Altitude');


% Plot the raw IMU data
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


% Identify any missing data
figure(5)
clf  
hold on  
subplot(2,1,1)
plot(diff(GPS_Timestamp), '.r')
title('Missing Data Identification (GPS)')
subplot(2,1,2)
plot(diff(IMU_Timestamp), '.b')
title('Missing Data Identification (IMU)')


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

% Show the interpolated GPS Location for each IMU message
figure(7)
clf
subplot(3,1,1);
plot(IMU_Timestamp, IMU_MetricPose(:,1), '.r')
title('IMU GPS Metric Pose (X)');
subplot(3,1,2);
plot(IMU_Timestamp, IMU_MetricPose(:,2), '.g')
title('IMU GPS Metric Pose (Y)');
subplot(3,1,3);
plot(IMU_Timestamp, IMU_MetricPose(:,3), '.b')
title('IMU GPS Metric Pose (Z)');


% Plot the trajectory of the IMU & GPS with orientations.
figure(8)
clf
nScale = 0.8;
hold on
%view(63, 24)
axis equal
grid
title('IMU Orientation & GPS Pose')

PlotTraj3D(IMU_MetricPose(:,1), IMU_MetricPose(:,2), IMU_MetricPose(:,3), ...
           IMU_Pitch, IMU_Roll, IMU_Yaw, nScale);
       
       
clear zv nScale



