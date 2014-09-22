
% Current dataset files


VectorNav_Logfile = 'C:\Users\Dereck\Documents\DataSets\bridge\vn.csv';
Hokuyo_Logfile = 'C:\Users\Dereck\Documents\DataSets\bridge\lidar_data.csv';

%
% Dataset ROI 
%

% Only process the interesting/(easy?) portion of the dataset

% Bridge Dataset (First Pass Only)
VectorNav_ROI_Start = 3000;
VectorNav_ROI_End = 10000;


%
% Replace the GPS path with a line
%

% BridgeDataset (First Pass Only)
Fuse_StartPos = [0 0  0];
Fuse_EndPos   = [0 60 0];



%
% IMU Bias
%

% Signs Dataset (Second Pass Only)
% IMU_YawBias = 20; 

% Bridge Dataset (First Pass Only)
IMU_YawBias  = 110;
IMU_RollBias = 13.2;






