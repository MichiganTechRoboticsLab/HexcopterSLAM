% FuseLinearPath.m
%  Very basic fusion of the Vectornav and Hokuyo sensor data
%  It simply ignores the VectorNav INS data and replaces it with line defined
%  by it's endpoints: [Fuse_StartPos; Fuse_EndPos]
%
%  It also ignores the IMU orientation and replaces it with a linear
%  interpolation of the first and last IMU orientation, assuming that it
%  doesn't really change much. This could be replaced by an average of all
%  data, or something and put in it's own file later.
% 
% This script also replaces the Lidar Timestamps with a linear
% interpolation of the beginning and end times. It was found that the
% timestamps can get delayed based on the CPU load of the embedded system
% which results in 'gaps' in the final point cloud. It may be possible to
% move this to it's own script and change it to only interpolate the time
% stamps of the bad data.
%
% If no endpoints are specified, the first and last GPS pose are used.



% Default Values
if ~exist('Fuse_StartPos', 'var')
   Fuse_StartPos = IMU_MetricPose(1,:);
end
if ~exist('Fuse_EndPos', 'var')
   Fuse_EndPos = IMU_MetricPose(end,:);
end



% Remove all Lidar Data before and after the IMU is available
I = or(Lidar_Timestamp < IMU_Timestamp(1), Lidar_Timestamp > IMU_Timestamp(end));
Lidar_ScanIndex(I) = [];
Lidar_Timestamp(I) = [];
Lidar_Angles(I) = [];
Lidar_Ranges(I) = [];
Lidar_X(I) = [];
Lidar_Y(I) = [];
clear I

% Linearize the lidar timestamps
start_T  = Lidar_Timestamp(1,:);
end_T    = Lidar_Timestamp(end,:);
Lidar_Timestamp = (start_T:(end_T-start_T)/(size(Lidar_Timestamp,1)-1):end_T)';

% Generate the position based on a line
start_T  = GPS_Timestamp(1,:);
end_T    = GPS_Timestamp(end,:);
Fusion_Position = interp1([start_T end_T], [Fuse_StartPos; Fuse_EndPos], Lidar_Timestamp);

% Get rid of the orientation data
start_T  = IMU_Timestamp(1,:);
end_T    = IMU_Timestamp(end,:);
Fusion_Q = interp1([start_T end_T], [IMU_Q(1,:); IMU_Q(end,:)], Lidar_Timestamp);

% Rotate all points by their orientation
Fusion_PointsRotated = quatrotate(Fusion_Q, [Lidar_X, Lidar_Y, zeros(size(Lidar_X))]);

% Translate all points by their translation
Fusion_pointcloud = Fusion_PointsRotated + Fusion_Position;


clear start_T end_T