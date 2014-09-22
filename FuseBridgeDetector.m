% FuseBridgeDetector.m
%  This script finds the bridge in the lidar data and shifts the pose 
%  Estimate so that the profile of the bridge is aligned across scans.
%
%  Before calling this script, use one of the other fuse scripts to
%  generate the initial point cloud, and this script will then apply 
%  dataset specific huristics to finish the alignment process. 



%
% Find the curb in the data
%

% make a copy of the point cloud
pc = Fusion_pointcloud;

% Debug plot (ROI)
figure(1);

% Initialize the Curb Location 
Curb_X = 200; % Signs Dataset vn_1

% This is the point in each scan that is detected as the curb
Curb_Point = [];


% Loop through all scans
nScanIndex = unique(Lidar_ScanIndex);
for i = 1:length(nScanIndex)
    
    % Retrieve each scan's points
    nIndex = nScanIndex(i);
    I = nIndex == Lidar_ScanIndex;
    cs = pc(I,:);
    
    % Progress
    if ~mod(i, 1000)
        disp(i)
    end
    
    
    % Remove Points outside of the Z Axis ROI
    I = cs(:,3) < 0 | cs(:,3) > 0.3;
    cs(I,:) = [];
    
    % Remove points from outside the ROI
    nROI_width = 50;
    
    start_ROI = Curb_X - nROI_width;
    if start_ROI < 1 
        start_ROI = 1;
    end
    end_ROI = start_ROI + 2*nROI_width;
    if end_ROI > size( cs, 1)
        end_ROI = size( cs, 1);
    end
    
    % Find points to search for the curbin
    ROIx = start_ROI:end_ROI;
    ROIy = cs(start_ROI:end_ROI, :);
    
    % Low-Pass Filter
    a = 0.98;
    f = filter(a, [1 a-1], ROIy(:,3));
    
    % Find Curb with First Difference 
    d = diff(f);
    
    % Low-Pass Filter
    a = 0.3;
    F = filter(a, [1 a-1], d);
    
    % Find curb
    [val, ind] = max(F);
    xMax = ROIx(ind);
    Curb_Point(i,:) = cs(xMax,:);
    
    % Update the ROI center
    Curb_X = xMax;
       
%     
% 
%     Debug plots    
%     set(0, 'CurrentFigure', 1);
%     clf;
%     
%     Curb ROI Display
%     subplot(4,1,1);
%     plot(ROIx, ROIy(:,3), '.b');
%     axis equal;
%     title(['Scan:' num2str(nIndex)]);
%     
%     Low pass filter
%     subplot(4,1,2);
%     plot(ROIx, f, '.b');
%     title('Low Pass Filter');
%     
%     First Diff display
%     subplot(4,1,3);
%     plot(ROIx(2:end), d, '-r');
%     title('First Order Diff');
%     
%     Low pass filter
%     subplot(4,1,4);
%     plot(ROIx(2:end), F, '-r');
%     title('Low Pass Filter');
%     
%     
%     Curb Location Displays 
%     subplot(4,1,1);
%     hold on;
%     plot([xMax xMax], [min(ROIy(:,3)) max(ROIy(:,3))], '-g');
%     
%     subplot(4,1,2);
%     hold on;
%     plot([xMax xMax], [min(f) max(f)], '-g');
%     
%     subplot(4,1,3);
%     hold on;
%     plot([xMax xMax], [min(d) max(d)], '-g');
%     
%     subplot(4,1,4);
%     hold on;
%     plot([xMax xMax], [min(F) max(F)], '-g');
%     
% 
%     Update Displays
%     drawnow();
%     pause(0.05);
    
end
pointcloud = pc;


% Translate each scan so that the curb point lines up
start_pt = Curb_Point(1,:);
end_pt   = Curb_Point(end,:);

% Use the start point as the origin
cp = bsxfun(@minus, Curb_Point, start_pt);
ep = end_pt - start_pt;

% Change B into a normalized vector
EP = ep/norm(ep);

% Dot Products
dp = [];
for i = 1:size(cp, 1)
    dp(i,:) = dot(cp(i,:),EP);
end

% Find the new snap-fit point
cp = dp * EP;

% Translate back to original frame
cp = bsxfun(@plus, cp, start_pt);



% 
% % Debug Plot
% figure(2)
% clf 
% % Show the detected curb locations
% plot3(Curb_Point(:,1), Curb_Point(:,2), Curb_Point(:,3), '-b', 'MarkerSize', 5)
% axis equal
% hold on
% plot3(Curb_Point(:,1), Curb_Point(:,2), Curb_Point(:,3), '.b', 'MarkerSize', 5)
% grid
% 
% % Show the linear line to fit the data to
% Curb_line = [start_pt; end_pt];
% plot3(Curb_line(:,1), Curb_line(:,2), Curb_line(:,3), '-g', 'MarkerSize', 20);
% plot3(start_pt(1), start_pt(2), start_pt(3), 'Og', 'MarkerSize', 15);
% plot3(end_pt(1),   end_pt(2),   end_pt(3),   'Or', 'MarkerSize', 15');
% 
% % Show the interpolated postitions
% plot3(cp(:,1), cp(:,2), cp(:,3), '.r', 'MarkerSize', 10);






% Update the points in the point cloud to align the curb.
nScanIndex = unique(Lidar_ScanIndex);
for i = 1:length(nScanIndex)
    
    % Retrieve each scan's points
    nIndex = nScanIndex(i);
    I = nIndex == Lidar_ScanIndex;
    
    % Update the postition
    Fusion_Position(I, :) = bsxfun(@minus, Fusion_Position(I, :), (Curb_Point(i,:) - cp(i,:)));
end


% Translate all points by their translation
Fusion_pointcloud = Fusion_PointsRotated + Fusion_Position;

% Generate RPY for each point
[rz, ry, rx] = quat2angle(Fusion_Q);
Fusion_RPY = [ry, rx, rz];
clear rx ry rz;




return

%
% Debug Plots
%

% Plot the full point cloud
figure(2)
clf 
grid
plot3(Fusion_pointcloud(:,1), Fusion_pointcloud(:,2), Fusion_pointcloud(:,3), '.b', 'MarkerSize', 2)
axis equal
hold on; 
plot3(Curb_Point(:,1), Curb_Point(:,2), Curb_Point(:,3), '-g');


% Plot a single scan with translation
figure(3); 
nScanIndex = unique(Lidar_ScanIndex);
for i = 1:length(nScanIndex)

    % Retrieve each scan's points
    nIndex = nScanIndex(i);
    I = nIndex == Lidar_ScanIndex;
    
    % Plot this scan
    set(0, 'CurrentFigure', 3);
    clf ;
    plot3(Fusion_pointcloud(I,1), Fusion_pointcloud(I,2), Fusion_pointcloud(I,3), '.b', 'MarkerSize', 5);
    grid;
    axis equal;
    hold on;       
    nWindowSize = 5;
    axis([Curb_Point(i,1)-nWindowSize Curb_Point(i,1)+nWindowSize ...
          Curb_Point(i,2)-nWindowSize Curb_Point(i,2)+nWindowSize ...
          Curb_Point(i,3)-nWindowSize Curb_Point(i,3)+nWindowSize]);
    title('Lidar Scan Animation (Rotation + Translation)')
    
    % Show the orientation at this scan
    nScale = 1;
    p1  = Fusion_Position(I,:);
    rx1 = Fusion_RPY(I,2);
    ry1 = Fusion_RPY(I,1);
    rz1 = Fusion_RPY(I,3);
    PlotTraj3D(p1(1,1), p1(1,2), p1(1,3), rx1(1), ry1(1), rz1(1), nScale); 
    
    
    % Show the curb location
    plot3(Curb_Point(i,1), Curb_Point(i,2), Curb_Point(i,3), 'Og', 'MarkerSize', 20);

    
    %view([-90 90]) % From Top
    %view([-90 0])  % From Left Side
    %view([0 0])    % From Back
    
    drawnow;
    pause(0.01);
end



