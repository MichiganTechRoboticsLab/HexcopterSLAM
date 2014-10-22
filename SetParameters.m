
% Current dataset files
DatasetName = 'railyard\up';


% Set path based on OS
if ispc()
    DataPath = 'C:\Users\Dereck\Documents\DataSets\';
    VectorNav_Logfile = [DataPath DatasetName '\vn.csv'];
    Hokuyo_Logfile = [DataPath DatasetName '\lidar_data.csv']; 
    Camera_Path = [DataPath DatasetName '\pics\'];
end
if isunix()
    VectorNav_Logfile = ['/home/dereck/Documents/DataSets' DatasetName '/vn.csv'];
    Hokuyo_Logfile = ['/home/dereck/Documents/DataSets' DatasetName '/lidar_data.csv'];  
end


% Kludge parameters for each dataset
switch DatasetName
    case 'railyard\up'
        VectorNav_LogFormat = 2;
        Lidar_LogFormat = 2;
        
        Fuse_StartPos = [0 0  0];
        Fuse_EndPos   = [0 40 0];
        
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







