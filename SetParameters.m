% Set up dataset-specific parameters

% I put all of my datasets in their own folders, the folder name is the
% dataset name. The new sensor logging script should make a new folder for
% every dataset with a timestamp for a dataset name. The timestamp will 
% probalby be wrong because their is no RTC on the jetson. Simply copy
% these datasets to your datasets folder and change the plot.

% Current dataset name
% Always clear the workspace when changing datasets!
DatasetName = ['railyard' filesep 'uav1'];


% Set path based on OS
% You will need to point this to the root of your dataset collection
if ispc()
    DataPath = 'C:\Users\Dereck\My Documents\DataSets\';
    VectorNav_Logfile = [DataPath DatasetName '\vn.csv'];
    Hokuyo_Logfile = [DataPath DatasetName '\lidar_data.csv']; 
    Camera_Path = [DataPath DatasetName '\pics\'];
end
if isunix()
    DataPath = '/home/dereck/DataSets/';
    VectorNav_Logfile = [DataPath DatasetName '/vn.csv'];
    Hokuyo_Logfile = [DataPath DatasetName '/lidar_data.csv'];  
end


% Kludge parameters for each dataset
switch DatasetName 
    case ['railyard' filesep 'uav1']
        VectorNav_LogFormat = 2;
        Lidar_LogFormat = 2;
        
        % Both Passes
        %VectorNav_ROI_Start = 23500;
        %VectorNav_ROI_End   = 36000;
        
        % First Pass
        VectorNav_ROI_Start = 23500;
        VectorNav_ROI_End   = 28500;        
        
        % IMU Bias
        IMU_YawBias    = -40;
        IMU_RollBias   = -2;
        IMU_PitchBias  =  0;
        
        % First Diff Filter
        Fuse_Diff_ROI_Z_max = 2;
        Fuse_Curb_X = 250;
        Fuse_Diff_ROI_width = 20;
        
    case 'railyard\up'
        VectorNav_LogFormat = 0;
        Lidar_LogFormat = 2;
                
        
    case 'construction'
        VectorNav_LogFormat = 2;
        Lidar_LogFormat = 2;
        
        % First Flight
        %VectorNav_ROI_Start = 15000;
        %VectorNav_ROI_End   = 44000;
        
        % First Pass
        VectorNav_ROI_Start = 15000;
        VectorNav_ROI_End   = 18600;
        
        IMU_RollBias  = 0;
        IMU_PitchBias = -90;
        IMU_YawBias   = 180+12;
        
        Fuse_LidarAltitude_Width = 200;
        
        Fuse_Curb_X = 0;         
        Fuse_Diff_ROI_width = 30;
        Fuse_Diff_ROI_Z_min = 2; 
        Fuse_Diff_ROI_Z_max = 6;
        
    case 'curbs'
        VectorNav_LogFormat = 1;
        Lidar_LogFormat = 1;
        
        VectorNav_ROI_Start = 4000;
        VectorNav_ROI_End   = 6000;
                
        IMU_RollBias = -35.6;
        IMU_YawBias  = 101;
        
        Fuse_StartPos = [0 0  0];
        Fuse_EndPos   = [0 40 0];
        
        Fuse_LidarAltitude_Width = 2;
        
        Fuse_Curb_X = 288;         
        Fuse_Diff_ROI_width = 50;
        Fuse_Diff_ROI_Z_min = 0; 
        Fuse_Diff_ROI_Z_max = 0.3;
        
    case 'bridge'
        VectorNav_LogFormat = 1;
        Lidar_LogFormat = 1;
        
        VectorNav_ROI_Start = 3000;
        VectorNav_ROI_End = 10000;
        
        IMU_RollBias = 11;
        IMU_YawBias  = 115;
        
        Fuse_StartPos = [0 0  0];
        Fuse_EndPos   = [0 60 0];
        
        Fuse_LidarAltitude_Width = 2;
        
        Fuse_Curb_X = 200;         
        Fuse_Diff_ROI_width = 30;
        Fuse_Diff_ROI_Z_min = 0; 
        Fuse_Diff_ROI_Z_max = 0.5;
        
    case 'signs'
        VectorNav_LogFormat = 1;
        Lidar_LogFormat = 1;
        
        VectorNav_ROI_Start =  8500;
        VectorNav_ROI_End   = 10750;
        
        IMU_RollBias = -1.2;
        IMU_YawBias = 7; 
                
        Fuse_LidarAltitude_Width = 10;
        
        Fuse_Curb_X = 250;         
        Fuse_Diff_ROI_width = 20;
        Fuse_Diff_ROI_Z_min = -0.5; 
        Fuse_Diff_ROI_Z_max =  1.5;
        
end







