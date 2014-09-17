
if ~exist(VectorNav_Logfile, 'file')
    error('VectorNav Logfile not found')
end

VectorNav_log = load(VectorNav_Logfile);


%
% Reformat Data
%

% Remove Nulls from GPS
i = VectorNav_log(:,6) ~= 0;

% Combine Timestamp into single element
GPS_Timestamp = VectorNav_log(i,1) + VectorNav_log(i,2) * 10E-7;

% Extract GPS coodinates
GPS_Lattitude = VectorNav_log(i,6);
GPS_Longitude = VectorNav_log(i,7);
GPS_Altitude  = VectorNav_log(i,8);

% Extract IMU Orientation
IMU_Timestamp = VectorNav_log(:,1) + VectorNav_log(:,2) * 10E-7;
IMU_Yaw   = deg2rad(VectorNav_log(:,3));
IMU_Pitch = deg2rad(VectorNav_log(:,4));
IMU_Roll  = deg2rad(VectorNav_log(:,5));

% Convert from LLA to Metric Coodinates
GPS_MetricPose = lla2flat([GPS_Lattitude GPS_Longitude GPS_Altitude], ... 
                          [GPS_Lattitude(1) GPS_Longitude(1)],  -90, -GPS_Altitude(1));

% I dont know why the resulting X & Z is negative.... 
GPS_MetricPose(:,1) = -1 * GPS_MetricPose(:,1);
GPS_MetricPose(:,3) = -1 * GPS_MetricPose(:,3);

% Interpolate the GPS pose for each IMU orientation (metric)
IMU_MetricPose = interp1(GPS_Timestamp, GPS_MetricPose, IMU_Timestamp);

% Generate Quaternions for linear interpolation of rotations
IMU_Q = angle2quat(  -IMU_Pitch, -IMU_Roll, -IMU_Yaw, 'XYZ');

% Cleanup workspace
clear i
