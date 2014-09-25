
% Current dataset files

DatasetName = 'bridge';

VectorNav_Logfile = ['C:\Users\Dereck\Documents\DataSets\' DatasetName '\vn.csv'];
Hokuyo_Logfile = ['C:\Users\Dereck\Documents\DataSets\' DatasetName '\lidar_data.csv'];


% Kludge parameters for each dataset
switch DatasetName
    case 'curbs'
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







