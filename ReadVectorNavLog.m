% ReadVectorNavLog.m
%  Reads the vectornav log file into matlab and makes variables for each data type.  
%  Generates a single timestamp value from the two log file columns, converts the 
%  LLA to metric positions, and converts the IMU's RPY  valued into quaternions. 
%  It also filters out all zero LLA values from the gps data. So the IMU and GPS
%  data arrays may not be the same length!


% Check that a log file was specified
if ~exist(VectorNav_Logfile, 'file')
    error('VectorNav Logfile not found')
end

% Read the log file
VectorNav_log = load(VectorNav_Logfile);

%
% Reformat Data
%

% Ignore entries where no GPS data was available
i = VectorNav_log(:,6) ~= 0;

% Combine Timestamp into single element
GPS_Timestamp = VectorNav_log(i,1) + VectorNav_log(i,2) * 10E-7;

% Extract GPS coodinates
GPS_Lattitude = VectorNav_log(i,6);
GPS_Longitude = VectorNav_log(i,7);
GPS_Altitude  = VectorNav_log(i,8);


% Ignore entries where no IMU data was available
i = VectorNav_log(:,3) ~= 0;

% Extract IMU Orientation
IMU_Timestamp = VectorNav_log(i,1) + VectorNav_log(i,2) * 10E-7;
IMU_Yaw   = deg2rad(VectorNav_log(i,3));
IMU_Pitch = deg2rad(VectorNav_log(i,4));
IMU_Roll  = deg2rad(VectorNav_log(i,5));


% Convert from LLA to Flat Earth (Metric) coodinates
GPS_MetricPose = lla2flat([GPS_Lattitude GPS_Longitude GPS_Altitude], ... 
                          [GPS_Lattitude(1) GPS_Longitude(1)],  0, -GPS_Altitude(1));

% Flat earth uses a (Northing, Easting, - Altitude) format, 
% so X and Y need to be swapped 
GPS_MetricPose = [GPS_MetricPose(:,2) GPS_MetricPose(:,1) GPS_MetricPose(:,3)];
GPS_MetricPose(:,3) = 1 * GPS_MetricPose(:,3);

% Interpolate the GPS pose for each IMU orientation (metric)
IMU_MetricPose = interp1(GPS_Timestamp, GPS_MetricPose, IMU_Timestamp);

% TH: normalize the Yaw to assume the Hex is moving forward
% DAW: I dont think this works for the general case. The VectorNav manual
%      says 0deg should point north. I think we need to have a parameter
%      for each dataset to add some bias to the yaw to compensate.
% IMU_Yaw = IMU_Yaw - IMU_Yaw(1);

% Generate Quaternions for linear interpolation of rotations
% (NOTE: Matlab quatRotate uses a NED coord system, so the usual rotations
% will produce a negitive result, that's why the inputs are negated here)
% The IMU_Q variable should only be used with matlab functions.
IMU_Q = angle2quat( -IMU_Pitch, -IMU_Roll, -IMU_Yaw, 'XYZ');

% Cleanup workspace
clear i
