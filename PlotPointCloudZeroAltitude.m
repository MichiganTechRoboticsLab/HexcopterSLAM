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

% Ignore Altitude
P(:, 3) = 0;

% Get the linear interpolation of the orientation from the IMU
Q = interp1(IMU_Timestamp, IMU_Q, Lidar_Timestamp);
[rz, ry, rx] = quat2angle(Q);

% Rotate all points by their orientation
p1 = quatrotate(Q, [Lidar_X, Lidar_Y, zeros(size(Lidar_X))]);

% Translate all points by their translation
pointcloud = p1 + P;



% Plot
pclviewer(pointcloud)       
       
       
       