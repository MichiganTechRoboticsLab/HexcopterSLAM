

% Linearize the lidar timestamps
start_T  = Lidar_Timestamp(1,:);
end_T    = Lidar_Timestamp(end,:);
Lidar_Timestamp = (start_T:(end_T-start_T)/(size(Lidar_Timestamp,1)-1):end_T)';

% Timestamp associated with the Fused Pose data.
Fusion_Timestamp = Lidar_Timestamp;