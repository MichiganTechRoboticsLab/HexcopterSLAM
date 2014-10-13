% FuseLidarAltitudeFilter.m
%  Adjusts the current fusion estimate by estimating the altitude from the 
%  lidar data directly under the UAV.
%
%  This script assumes that a FuseXYZ filter has been executed on the data
%  such as FuseRaw or FuseLinear

% Width of lidar data from the center of the scan to use for 
% altitude estimation
if ~exist('Fuse_LidarAltitude_Width', 'var')
   Fuse_LidarAltitude_Width = 100; % meters 
end

% Estimated Altitude
Z = zeros(max(Lidar_ScanIndex),1);

% Loop through every scan
nScanIndex = unique(Lidar_ScanIndex);
for i = 1:length(nScanIndex)
    % Retrieve each scan's points
    nIndex = nScanIndex(i);
    I = nIndex == Lidar_ScanIndex;
    p = Fusion_PointsRotated(I,:);
    
    % Ignore all points not under the UAV
    I = sqrt(p(:,1).^2 + p(:,2).^2) < Fuse_LidarAltitude_Width;
    p = p(I,:);
    
    if 0
        figure(20);
        clf
        plot3(p(:,1), p(:,2), p(:,3), '.')
        axis equal
        drawnow
        pause(0.1);
    end 
    
    % Find the lowest Z for this scan, and use that for our estimate
    % If you get an error about an empty matrix here, enlarge the ROI.
    Z(nIndex) = min(p(:,3));
end

% Loop through each poins
for i = 1:length(Fusion_Position)
    % Get this point's scan index
    nScan = Lidar_ScanIndex(i);
    
    % Find the altitude for this scan
    Fusion_Position(i,3) = -Z(nScan);
end

% Translate all points by their translation
Fusion_pointcloud = Fusion_PointsRotated + Fusion_Position;






