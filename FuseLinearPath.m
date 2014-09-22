% FuseLinearPath.m
%  Very basic fusion of the Vectornav and Hokuyo sensor data
%  It simply ignores the GPS postion data and replaces it with line defined
%  by it's endpoints: [Fuse_StartPos; Fuse_EndPos]


% Remove all Lidar Data before and after the IMU is available
I = or(Lidar_Timestamp < IMU_Timestamp(1), Lidar_Timestamp > IMU_Timestamp(end));
Lidar_ScanIndex(I) = [];
Lidar_Timestamp(I) = [];
Lidar_Angles(I) = [];
Lidar_Ranges(I) = [];
Lidar_X(I) = [];
Lidar_Y(I) = [];
clear I




% Generate the position based on a line
start_T  = GPS_Timestamp(1,:);
end_T    = GPS_Timestamp(end,:);
Fusion_Position = interp1([start_T end_T], [Fuse_StartPos; Fuse_EndPos], Lidar_Timestamp);


% Get rid of the oreintation data too
start_T  = IMU_Timestamp(1,:);
end_T    = IMU_Timestamp(end,:);
Fusion_Q = interp1([start_T end_T], [IMU_Q(1,:); IMU_Q(end,:)], Lidar_Timestamp);




% Rotate all points by their orientation
Fusion_PointsRotated = quatrotate(Fusion_Q, [Lidar_X, Lidar_Y, zeros(size(Lidar_X))]);

% Translate all points by their translation
Fusion_pointcloud = Fusion_PointsRotated + Fusion_Position;
