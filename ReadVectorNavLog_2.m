% ReadVectorNavLog_2.m
%
%  Second version of the read vectornav file
%
%  Reads the vectornav log file into matlab and makes variables for each data type.  
%  Generates a single timestamp value from the two log file columns, converts the 
%  LLA to metric positions, and converts the IMU's RPY  valued into quaternions. 
%  It also filters out all zero LLA values from the gps data. So the IMU and GPS
%  data arrays may not be the same length!


% Extract GPS coodinates

% Ignore entries where no GPS data was available
i = VectorNav_log(:,9) ~= 0;

GPS_Timestamp = VectorNav_log(i,1);
GPS_Lattitude = VectorNav_log(i,9);
GPS_Longitude = VectorNav_log(i,10);
GPS_Altitude  = VectorNav_log(i,11);



% Extract IMU Orientation

% Ignore entries where no IMU data was available
i = VectorNav_log(:,5) ~= 0;

IMU_Timestamp = VectorNav_log(i,1);

% Quaternion format: X Y Z W
IMU_Q = VectorNav_log(i,5:8);

% Convert to YPR
[IMU_Yaw, IMU_Roll, IMU_Pitch] = quat2angle(IMU_Q);

% NOTE: Matlab quaternion functions use a NED coord system, so the usual 
% rotations will produce negitive results, that's why they are negated here.
IMU_Yaw   = -1 * IMU_Yaw;
IMU_Roll  = -1 * IMU_Roll;
IMU_Pitch = -1 * IMU_Pitch;



% Convert from LLA to Flat Earth (Metric) coodinates
if size(GPS_Lattitude,1) > 0 
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


% Cleanup workspace
clear i
