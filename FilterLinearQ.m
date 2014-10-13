% FilterLinearQ.m
%  Replaces the Orientation with a constant estimate. 
% This is primarally for when the IMU data was unavailable.


% Default Parameters
if ~exist('IMU_RollBias', 'var')
    IMU_RollBias  = mean(IMU_Roll);
end
if ~exist('IMU_PitchBias', 'var')
    IMU_PitchBias = mean(IMU_Pitch);
end
if ~exist('IMU_YawBias', 'var')
    IMU_YawBias   = mean(IMU_Yaw);
end
    
% Replace Existing Estimate
q = angle2quat( -deg2rad(IMU_PitchBias), -deg2rad(IMU_RollBias), -deg2rad(IMU_YawBias), 'XYZ');
Fusion_Q = repmat(q, size(Fusion_Q,1), 1);
clear q;

%
% Update Pointcloud
%

% Rotate all points by their orientation
Fusion_PointsRotated = quatrotate(Fusion_Q, [Lidar_X, Lidar_Y, zeros(size(Lidar_X))]);

% Translate all points by their translation
Fusion_pointcloud = Fusion_PointsRotated + Fusion_Position;