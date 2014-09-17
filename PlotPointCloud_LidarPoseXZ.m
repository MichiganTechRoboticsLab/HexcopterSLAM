%
% Load Data
%
VectorNav_Logfile = 'C:\Users\Dereck\Documents\DataSets\signs\vn.csv';
ReadVectorNavLog

Hokuyo_Logfile = 'C:\Users\Dereck\Documents\DataSets\signs\lidar_data.csv';
ReadHokuyoLog


%
% Fuse GPS and IMU data
%

% Remove all Lidar Data before and after the IMU is available
I = or(Lidar_Timestamp < IMU_Timestamp(1), Lidar_Timestamp > IMU_Timestamp(end));
Lidar_ScanIndex(I) = [];
Lidar_Timestamp(I) = [];
Lidar_Angles(I) = [];
Lidar_Ranges(I) = [];
Lidar_X(I) = [];
Lidar_Y(I) = [];
clear I


% Get the linear interpolation of the orientation from the IMU
Q = interp1(IMU_Timestamp, IMU_Q, Lidar_Timestamp);

% Rotate all points by their orientation
p = quatrotate(Q, [Lidar_X, Lidar_Y, zeros(size(Lidar_X))]);

% Get the GPS based position of the sensor for each hit
P = interp1(GPS_Timestamp, GPS_MetricPose, Lidar_Timestamp);


%
% Find the Altitude from the Lidar data
%
nScanLen = [];
Z = zeros(7000,1);
for i = 1:length(p)
    % Get this point's scan index
    nScan = Lidar_ScanIndex(i);
        
    % Find the lowest Z for this scan
    Z(nScan) = min(p(i,3), Z(nScan));
end

%
% Use the sensed altitude for the pose
%
for i = 1:length(p)
    % Get this point's scan index
    nScan = Lidar_ScanIndex(i);
        
    % Find the lowest Z for this scan
    P(i,3) = -Z(nScan);
end





%% Find Translation from lidar scan matching
figure(2)
prevScan = [];
currScan = [];
currScanIndex = 0;
F = [];

mapSize = 20; % meters
mapRes  = 100; % pixels per meter


for i = 1:length(p)
    % Get this point's scan index
    nScan = Lidar_ScanIndex(i);
    
    % Match the two previous scans.
    if currScanIndex ~= nScan && size(prevScan, 1) > 0
         
        %
        % Chamfer Distance Objective function
        %
        
        % translate points to an image pixels
        mx = round((prevScan(:,1) + mapSize/2) * mapRes);
        my = round((prevScan(:,2) + mapSize/2) * mapRes);
        
        % Remove out of bounds pixels
        I = (mx < 1) | (mx > (mapSize * mapRes));
        mx(I) = [];
        my(I) = [];
        I = (my < 1) | my > (mapSize * mapRes);
        mx(I) = [];
        my(I) = []; 
        
        % Generate a chamfer distance image
        map = zeros(mapSize * mapRes);       
        map(sub2ind(size(map), my, mx)) = 1; 
        map_bw = bwdist(map);
        
        X = -50:50;
        f = zeros(size(X));
        for x = 1:length(X);
            % Convert currScan to pixels
            mx2 = round(X(x) + (currScan(:,1) + mapSize/2) * mapRes);
            my2 = round((currScan(:,2) + mapSize/2) * mapRes);

            % Remove out of bounds pixels
            I = (mx2 < 1) | (mx2 > (mapSize * mapRes));
            mx2(I) = [];
            my2(I) = [];
            I = (my2 < 1) | (my2 > (mapSize * mapRes));
            mx2(I) = [];
            my2(I) = []; 
            
            d = map_bw(sub2ind(size(map_bw), my2, mx2));
            f(x) = sum(d);
%             
%             D = [mx2 my2 d];
%             
%             % PLOT
%             set(0, 'CurrentFigure', 2);
%             clf;
%             contour(map_bw);
%             hold on;
%             plot(mx, my, '.r', 'MarkerSize', 5);
%             plot(mx2, my2, '.g', 'MarkerSize', 5);
%             title([num2str(X(x)) ': ' num2str(f(x))]);
%             drawnow();
%             pause(0.5);
        end
        % Sum all distances for the objective function
        F(end+1,:) = f; 
        
%         figure(1)
%         clf
%         plot(X, f) 
%         hold on
%         plot(F) 
%         plot(FZ)
        
        
        % Plot it        
%         set(0, 'CurrentFigure', 2);
%         clf;
%         contour(map_bw);
%         hold on;
%         plot(mx, my, '.r', 'MarkerSize', 1);
%         plot(mx2, my2, '.r', 'MarkerSize', 1);
%         title([num2str(nScan) ': ' num2str(f(end))])
        disp(nScan);
        %drawnow();
        %pause(0.1);
    end
    
    
    
    
    
    
    % Start Collecting the next scan
    if currScanIndex ~= nScan
       prevScan = currScan;
       currScanIndex = nScan;
       currScan = [];
    end
    
    % Add this point to the current scan list
    currScan(end+1,:) = [Lidar_X(i), Lidar_Y(i), 0];
end







%% Translate all points by their translation
pointcloud = p + P;


% % Plot
% figure(1)
% clf
% plot(Lidar_Timestamp, P(i,3));

pclviewer(pointcloud')       
       
       
       