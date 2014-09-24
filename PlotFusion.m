% PlotFusion.m
%  Select a Fusion algorithm below and this script will plot the results. 
%  This will help keep the plots consistant across algorithm tests and
%  help keep the algorithm separte from the plotting features so that
%  the code is easier to read and ready to be used for whatever comes next.
%
%  This file:
%  Plots the interpolated lidar pos over the GPS track
%  Plots the interpolated lidar pos and orientation over the GPS track
%  Plots the Full point cloud.
%  Plots the orientation, pose, and each scan one at a time. (animation)

% Load Data
SetParameters
ReadVectorNavLog
ReadHokuyoLog

% Fuse GPS and IMU data

% List of filters/estimators
%FuseRaw
%FuseZeroAltitude
%FuseLinearPath
%FuseLidarAltitude
%FuseLidarAltitude
%FuseLidarAltitudeFilter
%FuseCurbDetector
%FuseBridgeDetector

switch DatasetName
    case 'curbs'      
        FuseLinearPath
        FuseLidarAltitudeFilter
        FuseSingleFirstDiffFilter
        
    case 'bridge'        
        FuseLinearPath
        FuseLidarAltitudeFilter
        FuseSingleFirstDiffFilter
        
    case 'signs'
        FuseRaw
        FuseLidarAltitudeFilter
        FuseSingleFirstDiffFilter
        
    otherwise
        FuseRaw
end



%
% Plots
%

% Plot the interpolated pose points and the GPS path
figure(1)
clf
plot3(Fusion_Position(:,1), Fusion_Position(:,2), Fusion_Position(:,3), '-r', 'MarkerSize', 4) 
hold on;
plot3(GPS_MetricPose(:,1), GPS_MetricPose(:,2), GPS_MetricPose(:,3), '-b') 
title('Lidar Pose & GPS Track');
legend('Fusion', 'GPS');
axis equal
grid



% Plot the interpolated orientations and the original orientation path
% figure(2)
% clf
% PlotTraj3D(IMU_MetricPose, IMU_Q, 1);
% hold on
% 
% PlotTraj3D(Fusion_Position, Fusion_Q, 1);
% axis equal
% grid
% title('IMU Orientation & GPS Pose')


% % Plot the full point cloud
% figure(3)
% clf 
% grid
% plot3(Fusion_pointcloud(:,1), Fusion_pointcloud(:,2), Fusion_pointcloud(:,3), '.b', 'MarkerSize', 2)
% axis equal
% hold on; 



% View in PCL_Veiwer
% Add library to path if not already present
if ~exist('pclviewer.m', 'file')
    if exist('matpcl', 'dir') 
        addpath('matpcl');
    end
end
pclviewer(Fusion_pointcloud')  


return

%% Plot a single scan without translation
figure(4); 
nScanIndex = unique(Lidar_ScanIndex);
for i = 1:length(nScanIndex)
    
    % Retrieve each scan's points
    nIndex = nScanIndex(i);
    I = nIndex == Lidar_ScanIndex;
    
    % Plot this scan
    set(0, 'CurrentFigure', 4);
    clf ;
    plot3(Fusion_PointsRotated(I,1), Fusion_PointsRotated(I,2), Fusion_PointsRotated(I,3), '.b', 'MarkerSize', 5);
    grid;
    hold on;       
    title('Lidar Scan Animation (Rotation Only)')
    
    % Show the oreintation at this scan
    q = Fusion_Q(I,:);
    PlotTraj3D([0, 0, 0], q(1,:), 1); 
    axis([-5 6 -2 2 -10 0 ]); 
    %axis equal;
    
    %view([-90 90]) % From Top
    %view([-90 0])  % From Left Side
    view([0 0])    % From Back
    %view([-25 0])   % Signs vn_1 dataset
    
    drawnow;
    pause(0.01);
end
         

%% Plot a single scan with translation
figure(5); 
nScanIndex = unique(Lidar_ScanIndex);
for i = 1:length(nScanIndex)

    % Retrieve each scan's points
    nIndex = nScanIndex(i);
    I = nIndex == Lidar_ScanIndex;
    
    % Plot this scan
    set(0, 'CurrentFigure', 5);
    clf ;
    plot3(Fusion_pointcloud(I,1), Fusion_pointcloud(I,2), Fusion_pointcloud(I,3), '.b', 'MarkerSize', 2);
    grid;
    axis equal;
    hold on;       
    axis([-1 1 -1 1 -1 .1] * 20);
    title('Lidar Scan Animation (Rotation + Translation)')
    
    % Show the oreintation at this scan
    nScale = 1;
    q = Fusion_Q(I,:);
    PlotTraj3D(IMU_MetricPose(1,:), q(1,:), 1); 

    view([-90 90]) % From Top
    %view([-90 0])  % From Left Side
    %view([0 0])    % From Back
    
    drawnow;
    pause(0.01);
end
       
       