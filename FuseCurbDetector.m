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

% Initialize the Curb Location
Curb_X = 0;
Curb_Point = [];

% Loop through all scans
nScanIndex = unique(Lidar_ScanIndex);
for i = 1:length(nScanIndex)
    
    % Retrieve each scan's points
    nIndex = nScanIndex(i);
    I = nIndex == Lidar_ScanIndex;
    cs = pc(I,:);
    
    
    
    % Remove points from outside the ROI
    nROI_width = 0.75;
    I = (cs(:,1) < (Curb_X - nROI_width))  | (cs(:,1) > (Curb_X + nROI_width)) | ...
        (cs(:,3) > 1);
    cs(I,:) = [];
    
    % Find Curb with First Difference 
    x = cs(2:end,1);
    y = diff(cs(:,3),n);
    [val, ind] = max(y);
    Curb_X = x(ind);
    Curb_Point(i,:) = cs(ind,:);
    
    
    
    
%     % Debug plots    
%     set(0, 'CurrentFigure', 1);
%     clf;
%     
%     % Curb ROI Display
%     subplot(2,1,1);
%     plot( cs(:,1),  cs(:,3), '.b');
%     axis equal;
%     title(num2str(nIndex));
%     
%     % First Diff display
%     subplot(2,1,2);
%     plot( x, y, '-r');
%     
%     % Curb Location Display  
%     subplot(2,1,1);
%     hold on;
%     plot([Curb_X Curb_X], [min(cs(:,3)) max(cs(:,3))], '-g');
%     
%     subplot(2,1,2);
%     hold on;
%     plot([Curb_X Curb_X], [min(y) max(y)], '-g');
%     
%     % Update Displays
%     drawnow();
%     pause(0.05);
end
pointcloud = pc;


% Update the points in the point cloud to align the curb.
nScanIndex = unique(Lidar_ScanIndex);
for i = 1:length(nScanIndex)
    
    % Retrieve each scan's points
    nIndex = nScanIndex(i);
    I = nIndex == Lidar_ScanIndex;
    
    % Update the postition
    Fusion_Position(I, 1) = Fusion_Position(I, 1) - Curb_Point(i,1);
end


% Translate all points by their translation
Fusion_pointcloud = Fusion_PointsRotated + Fusion_Position;

% Generate RPY for each point
[rz, ry, rx] = quat2angle(Fusion_Q);
Fusion_RPY = [ry, rx, rz];
clear rx ry rz;


return

%
% Debug Plots
%

% Plot the full point cloud
figure(2)
clf 
grid
plot3(Fusion_pointcloud(:,1), Fusion_pointcloud(:,2), Fusion_pointcloud(:,3), '.b', 'MarkerSize', 2)
axis equal
hold on; 
plot3(Curb_Point(:,1), Curb_Point(:,2), Curb_Point(:,3), '-g');


%% Plot a single scan with translation
figure(3); 
nScanIndex = unique(Lidar_ScanIndex);
for i = 1:length(nScanIndex)

    % Retrieve each scan's points
    nIndex = nScanIndex(i);
    I = nIndex == Lidar_ScanIndex;
    
    % Plot this scan
    set(0, 'CurrentFigure', 3);
    clf ;
    plot3(Fusion_pointcloud(I,1), Fusion_pointcloud(I,2), Fusion_pointcloud(I,3), '.b', 'MarkerSize', 5);
    grid;
    axis equal;
    hold on;       
    nWindowSize = 5;
    axis([Curb_Point(i,1)-nWindowSize Curb_Point(i,1)+nWindowSize ...
          Curb_Point(i,2)-nWindowSize Curb_Point(i,2)+nWindowSize ...
          Curb_Point(i,3)-nWindowSize Curb_Point(i,3)+nWindowSize]);
    title('Lidar Scan Animation (Rotation + Translation)')
    
    % Show the orientation at this scan
    nScale = 1;
    p1  = Fusion_Position(I,:);
    rx1 = Fusion_RPY(I,2);
    ry1 = Fusion_RPY(I,1);
    rz1 = Fusion_RPY(I,3);
    PlotTraj3D(p1(1,1), p1(1,2), p1(1,3), rx1(1), ry1(1), rz1(1), nScale); 
    
    
    % Show the curb location
    plot3(Curb_Point(i,1), Curb_Point(i,2), Curb_Point(i,3), 'Og', 'MarkerSize', 20);

    
    %view([-90 90]) % From Top
    %view([-90 0])  % From Left Side
    %view([0 0])    % From Back
    
    drawnow;
    pause(0.01);
end



