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


% Default Parameters
if ~exist('VectorNav_ROI_Start', 'var')
    VectorNav_ROI_Start = 1;
end
if ~exist('VectorNav_ROI_End', 'var')
    VectorNav_ROI_End  = size(VectorNav_log, 1);
end
if ~exist('IMU_YawBias', 'var')
    IMU_YawBias = 0;
end
if ~exist('IMU_RollBias', 'var')
    IMU_RollBias = 0;
end


% ROI
VectorNav_log = VectorNav_log(VectorNav_ROI_Start:VectorNav_ROI_End, :);


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
IMU_Yaw   = deg2rad(VectorNav_log(i,3)) + deg2rad(IMU_YawBias);
IMU_Pitch = deg2rad(VectorNav_log(i,4));
IMU_Roll  = deg2rad(VectorNav_log(i,5)) + deg2rad(IMU_RollBias);


if size(GPS_Lattitude,1) > 0 
    % Convert from LLA to Flat Earth (Metric) coodinates
    GPS_MetricPose = lla2flat([GPS_Lattitude GPS_Longitude GPS_Altitude], ... 
                              [GPS_Lattitude(1) GPS_Longitude(1)],  0, -GPS_Altitude(1));

    % Flat earth uses a (Northing, Easting, - Altitude) format, 
    % so X and Y need to be swapped 
    GPS_MetricPose = [GPS_MetricPose(:,2) GPS_MetricPose(:,1) GPS_MetricPose(:,3)];
    GPS_MetricPose(:,3) = GPS_MetricPose(:,3);
else
    % When no GPS signal is available, default to zero for the whole
    % dataset so that the rest of the code doesn't fault on an empty matrix
    GPS_Lattitude  = [0 0 0; 0 0 0];
    GPS_Longitude  = [0 0 0; 0 0 0];
    GPS_Altitude   = [0 0 0; 0 0 0];
    GPS_MetricPose = [0 0 0; 0 0 0];
    GPS_Timestamp  = [IMU_Timestamp(1);  IMU_Timestamp(end)];
end

% Interpolate the GPS pose for each IMU orientation (metric)
IMU_MetricPose = interp1(GPS_Timestamp, GPS_MetricPose, IMU_Timestamp);

% Generate Quaternions for linear interpolation of rotations
% NOTE: Matlab quatRotate uses a NED coord system, so the usual rotations
% will produce a negitive result, that's why the inputs are negated here.
% The IMU_Q variable should only be used with matlab functions, If you want
% to use custom equations, be prepared that the rotation will be backwards.
IMU_Q = angle2quat( -IMU_Pitch, -IMU_Roll, -IMU_Yaw, 'XYZ');

% Cleanup workspace
clear i
