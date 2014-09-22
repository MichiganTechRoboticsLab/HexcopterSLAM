% FuseLidarAltitude.m
%  Very basic fusion of the Vectornav and Hokuyo sensor data
%  It simply ignores the GPS altidude data and replaces it with an
%  estimated altitude from the Lidar data.
%
%  Currently I simply use the point with the lowest Z value as the altitude
%  estimate for each scan.



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

% Get the orientation from the IMU for each hit
Fusion_Q = interp1(IMU_Timestamp, IMU_Q, Lidar_Timestamp);

% Rotate all points by their orientation
Fusion_PointsRotated = quatrotate(Fusion_Q, [Lidar_X, Lidar_Y, zeros(size(Lidar_X))]);



% Find the Altitude from the Lidar data
%   This code could be made more clear and faster but I'll have to do it
%   later when I have more time, its good 'nuff for now.
nScanLen = [];
Z = zeros(10000,1);
for i = 1:length(Fusion_PointsRotated)
    % Get this point's scan index
    nScan = Lidar_ScanIndex(i);
    
    % Scan only directly under the UAV
    I = Fusion_PointsRotated(:,)
    
    % Find the lowest Z for this scan
    Z(nScan) = min(Fusion_PointsRotated(i,3), Z(nScan));
end


% Use the sensed altitude for the pose
for i = 1:length(Fusion_Position)
    % Get this point's scan index
    nScan = Lidar_ScanIndex(i);
        
    % Find the lowest Z for this scan
    Fusion_Position(i,3) = -Z(nScan);
end



% Translate all points by their translation
Fusion_pointcloud = Fusion_PointsRotated + Fusion_Position;

