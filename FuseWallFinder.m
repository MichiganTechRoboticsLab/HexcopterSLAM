% Fuse Wall Finder.m
%  Use a wall for aligning scans.


% Default Parameters
if ~exist('Fuse_wall_X', 'var')
   Fuse_Curb_X = 0; 
end
if ~exist('Fuse_Diff_ROI_width', 'var')
   Fuse_Diff_ROI_width = 30;
end
if ~exist('Fuse_Diff_ROI_Z_min', 'var')
   Fuse_Diff_ROI_Z_min = 2; 
end
if ~exist('Fuse_Diff_ROI_Z_max', 'var')
   Fuse_Diff_ROI_Z_max = 5;
end



%
% Find the curb in the data
%

% make a copy of the point cloud
pc = Fusion_pointcloud;

% Debug plot (ROI)
figure(1);

% This is the point in each scan that is detected as the curb
Curb_Point = [];
CurbX = Fuse_Curb_X;

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
    
    
    % Remove Points outside of the ROI
    I = cs(:,3) < Fuse_Diff_ROI_Z_min | cs(:,3) > Fuse_Diff_ROI_Z_max;
    cs(I,:) = [];
    
    start_ROI = CurbX - Fuse_Diff_ROI_width;
    if start_ROI < 1 
        start_ROI = 1;
    end
    end_ROI = start_ROI + 2*Fuse_Diff_ROI_width;
    if end_ROI > size( cs, 1)
        end_ROI = size( cs, 1);
    end
    
    ROIx = start_ROI:end_ROI;
    ROIy = cs(start_ROI:end_ROI, :);
    
    
    [c, maxX] = max(ROIy(:,3));
    
    % Update the ROI center
    CurbX = ROIx(maxX);
  
    % Find the wall
    Curb_Point(i,:) = cs(CurbX,:);
    
    
% 
%     Debug plots
% 
    if 0
        set(0, 'CurrentFigure', 1);
        clf;
        
    %   ROI Display
        subplot(1,1,1);
        plot(ROIy(:,1), ROIy(:,3), '.b');
        title(['Scan:' num2str(nIndex)]);




    %   Curb Location Displays 
        subplot(1,1,1);
        hold on;
        plot([ROIy(CurbX,1) ROIy(CurbX,1)], [min(ROIy(:,3)) max(ROIy(:,3))], '-g');

    %   Update Displays
        drawnow();
        pause(0.1);
    end
end
pointcloud = pc;


% Translate each scan so that the curb point lines up
% (scratch/point2line.m - may help understand this code segment)
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



% Low-pass the position vectors
a = 0.5;
Fusion_Position = filter(a, [1 a-1], Fusion_Position);



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





% Debug Plot
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






