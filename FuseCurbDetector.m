% FuseCurbDetector.m
%  Basic fusion of the Vectornav and Hokuyo sensor data
%  This is a dataset-specific algorithm for processing the "signs" dataset
%
%  I use the lidar to get an estimate of the altitude (See
%  FuseLidarAltitude.m for an example)
%
%  Then find the curb in the lidar data to provide alignment of the data in 
%  the x axis so to speak. WIP



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






%
% Find the curb in the data
%

% Translate all points into the point cloud for the starting point
pc = Fusion_PointsRotated + Fusion_Position;

% Debug plot (ROI)
figure(1);

% Loop through all scans
nScanIndex = unique(Lidar_ScanIndex);
for i = 1:length(nScanIndex)

    % Retrieve each scan's points
    nIndex = nScanIndex(i);
    I = nIndex == Lidar_ScanIndex;
    cs = pc(I,:);
    
    % Remove points from outside the ROI
    I = (cs(:,1) < -1)  | (cs(:,1) > 15) | ...
        (cs(:,3) > 1);
    cs(I,:) = [];

    
    % Debug plot    
    set(0, 'CurrentFigure', 1);
    clf;
    subplot(2,1,1);
    plot( cs(:,1),  cs(:,3), '.b');
    axis equal;
    title(num2str(nIndex));
        
    subplot(2,1,2);
    n = 1;
    plot( cs((n+1):end,1),  diff(cs(:,3),n), '-r');
    drawnow();
    pause(0.01);
end
pointcloud = pc;






% Translate all points by their translation
Fusion_pointcloud = Fusion_PointsRotated + Fusion_Position;



% Generate RPY for each point
[rz, ry, rx] = quat2angle(Fusion_Q);
Fusion_RPY = [ry, rx, rz];
clear rx ry rz;
