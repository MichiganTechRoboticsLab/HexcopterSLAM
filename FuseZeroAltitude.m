% FuseZeroAltitude.m
%  Very basic fusion of the Vectornav and Hokuyo sensor data
%  It simply ignores the GPS altidude data and replaces it with zeros.



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
Fusion_Position = interp1(GPS_Timestamp, GPS_MetricPose, Lidar_Timestamp);


% Ignore Altitude
Fusion_Position(:,3) = 0;


% Get the orientation from the IMU for each hit
Fusion_Q = interp1(IMU_Timestamp, IMU_Q, Lidar_Timestamp);

% Rotate all points by their orientation
Fusion_PointsRotated = quatrotate(Fusion_Q, [Lidar_X, Lidar_Y, zeros(size(Lidar_X))]);

% Translate all points by their translation
Fusion_pointcloud = Fusion_PointsRotated + Fusion_Position;
