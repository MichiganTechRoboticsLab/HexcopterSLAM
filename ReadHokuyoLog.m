

% Check that the File Exists
if ~exist(Hokuyo_Logfile, 'file')
    error('Hokuyo Logfile not found')
end

%
% Read the Lidar Data from HDD only when nessisary
%
if exist('Lidar_Log', 'var')
    disp('INFO: Lidar data already loaded, skipping import from HDD.')
else
    % Sensor Parameters
    Lidar_AngleStart = -2.3562;
    Lidar_AngleEnd   =  2.3562;
    
    % Column Indexes
    Lidar_Col_Timestamp = 1;
    Lidar_Col_Ranges    = 2;

    % Resulting Logfile
    Lidar_Log = [];
    
    % Open the file
    fid = fopen(Hokuyo_Logfile);

    % State vars
    nLine = 0;
    nScan = 0;
    nPoint = 0;
    nPoints = [];

    %Read each line
    tLine = fgets(fid);
    while ischar(tLine)
        nLine = nLine + 1;

        if strcmp(tLine(1:2), '-1')
            % Beginning of a new scan
            [items, ~] = sscanf(tLine, '%d,%d,%d');

            % Store the number of ranges for this scan
            if nScan > 0 
                nPoints(nScan) = nPoint;
                nPoint = 0;
            end
            
            % Start a new scan by storing the timestamp on a new row
            nScan = nScan + 1;
            Lidar_Log(nScan, Lidar_Col_Timestamp) = items(2) + items(3) * 10E-7;
                        
        else
            % New data point
            [items, ~] = sscanf(tLine, '%d,%f');
            
            % Store this measurement (Convert from mm to m)
            Lidar_Log(nScan,Lidar_Col_Ranges + nPoint) = items(1) * 1e-3;
            nPoint = nPoint + 1;
        end

        % Read next line
        tLine = fgets(fid);

        % Show progress
        if mod(nLine, 10000) == 0
            disp(nLine)
        end
    end

    fclose(fid);
    
    % Store number of points per scan
    Lidar_nPoints = nPoints(1);
    
    % Remove incomplete scans
    I = nPoints ~= Lidar_nPoints;
    Lidar_Log(I, :) = [];    

    % Remove extra columns
    if size(Lidar_Log, 2) > Lidar_Col_Ranges + Lidar_nPoints
        Lidar_Log(:, Lidar_Col_Ranges + Lidar_nPoints:end) = [];
    end
    
    % Precalculate the angles for each range
    da = (Lidar_AngleEnd - Lidar_AngleStart) / (Lidar_nPoints - 1);
    Lidar_Angles = (Lidar_AngleStart:da:Lidar_AngleEnd)';
    
    % Cleanup workspace
    clearvars fid nLine nScan nPoint tLine items nPoints I da
end