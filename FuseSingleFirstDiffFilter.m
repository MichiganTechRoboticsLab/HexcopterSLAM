% FuseSingleFirstDiffFilter.m
%  This filter searches an ROI for the largest first difference and then
%  uses that point to allign the data across all scans. This works well
%  when there is a single rising edge that appears through the entire
%  dataset and the path of the UAV is nearly linear. Curbs on the edges of
%  roads, for example, work well. 
%
%  Before calling this script, use one of the other fuse scripts to
%  generate the initial point cloud, and this script will then apply 
%  dataset specific huristics to finish the alignment process. 
%
%  After finding the point where the first difference is maximum, the point
%  is snapped to a line defined by the first and last detected 'curb' point
%  in the dataset. It would be nice to be able to super sample the region
%  around the curb to get a better location estimate somehow.



% Default Parameters
if ~exist('Fuse_Curb_X', 'var')
   Fuse_Curb_X = 0; 
end
if ~exist('Fuse_Diff_ROI_width', 'var')
   Fuse_Diff_ROI_width = 1000;
end
if ~exist('Fuse_Diff_ROI_Z_min', 'var')
   Fuse_Diff_ROI_Z_min = -100; 
end
if ~exist('Fuse_Diff_ROI_Z_max', 'var')
   Fuse_Diff_ROI_Z_max = 100;
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
    
    start_ROI = Fuse_Curb_X - Fuse_Diff_ROI_width;
    if start_ROI < 1 
        start_ROI = 1;
    end
    end_ROI = start_ROI + 2*Fuse_Diff_ROI_width;
    if end_ROI > size( cs, 1)
        end_ROI = size( cs, 1);
    end
    
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
    Fuse_Curb_X = xMax;
  
    
% 
%     Debug plots
% 
    if 0
        set(0, 'CurrentFigure', 1);
        clf;

    %     Curb ROI Display
        subplot(4,1,1);
        plot(cp(:,3), '.r');
        plot(ROIx, ROIy(:,3), '.b');
        title(['Scan:' num2str(nIndex)]);

    %     Low pass filter
        subplot(4,1,2);
        plot(ROIx, f, '.b');
        title('Low Pass Filter');

    %     First Diff display
        subplot(4,1,3);
        plot(ROIx(2:end), d, '-r');
        title('First Order Diff');

    %     Low pass filter
        subplot(4,1,4);
        plot(ROIx(2:end), F, '-r');
        title('Low Pass Filter');


    %     Curb Location Displays 
        subplot(4,1,1);
        hold on;
        plot([xMax xMax], [min(ROIy(:,3)) max(ROIy(:,3))], '-g');

        subplot(4,1,2);
        hold on;
        plot([xMax xMax], [min(f) max(f)], '-g');

        subplot(4,1,3);
        hold on;
        plot([xMax xMax], [min(d) max(d)], '-g');

        subplot(4,1,4);
        hold on;
        plot([xMax xMax], [min(F) max(F)], '-g');


    %     Update Displays
        drawnow();
        pause(0.01);
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






