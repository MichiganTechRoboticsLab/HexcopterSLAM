% Load Data
Header
ReadVectorNavLog
ReadHokuyoLog


% Fuse GPS and IMU data

% Remove all Lidar Data before and after the IMU is available
I = (Lidar_Timestamp < IMU_Timestamp(1)) | (Lidar_Timestamp > IMU_Timestamp(end));
Lidar_ScanIndex(I) = [];
Lidar_Timestamp(I) = [];
Lidar_Angles(I) = [];
Lidar_Ranges(I) = [];
Lidar_X(I) = [];
Lidar_Y(I) = [];
clear I


% Get the linear interpolation of the orientation from the IMU
Q = interp1(IMU_Timestamp, IMU_Q, Lidar_Timestamp);

% Rotate all points by their orientation
p = quatrotate(Q, [Lidar_X, Lidar_Y, zeros(size(Lidar_X))]);

% Get the GPS based position of the sensor for each hit
P = interp1(GPS_Timestamp, GPS_MetricPose, Lidar_Timestamp);


%
% Find the Altitude from the Lidar data
%
nScanLen = [];
Z = zeros(10000,1);
for i = 1:length(p)
    % Get this point's scan index
    nScan = Lidar_ScanIndex(i);
        
    % Find the lowest Z for this scan
    Z(nScan) = min(p(i,3), Z(nScan));
end

% Use the sensed altitude for the pose
for i = 1:length(p)
    % Get this point's scan index
    nScan = Lidar_ScanIndex(i);
        
    % Find the lowest Z for this scan
    P(i,3) = -Z(nScan);
end


%
% Find the curb in the lidar data
%

% Translate all points by their GPS translation
pc = p + P;

% Remove Trees
I = pc(:,3) > 3;
pc(I,:) = [];

% Generate a list of all scan indexes
nScanIndex = unique(Lidar_ScanIndex);

figure(1);
for i = 1:length(nScanIndex)
   
    % Retrieve each scan's points
    nIndex = nScanIndex(i);
    I = nIndex == Lidar_ScanIndex;
    cs = pc(I,:);
    
    % remove any points too far from the ground.
    I = cs(:,3) > 1;
    cs(I,:) = [];
    

    % Debug plot
    clf;
    plot( cs(:,1),  cs(:,3), '.b');
    axis([-2 15 -1 1]);
    title(num2str(nIndex));
    drawnow();
    pause(0.01);
        
end

pointcloud = pc;


% Plot
% figure(1)
% clf
% plot(Lidar_Timestamp, P(i,3));

pclviewer(pointcloud')       
%savepcd('cloud.pcd', pointcloud');
