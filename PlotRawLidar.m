% PlotRawLidar.m
%  Plots the raw Lidar data all on one plot, and one scan at a time.


% Load Data
SetParameters
ReadHokuyoLog


% Plot of all hits on a 2D polar plot
figure(1);
clf;
polar(Lidar_Angles, Lidar_Ranges, '.b');


% Identify missing data / Bad timestamps
figure(2)
clf  
plot(diff(Lidar_Timestamp), '.r')
title('Missing Data Identification (GPS)')


% Plot each scan individually
nScanIndex = unique(Lidar_ScanIndex);

figure(3);
for i = 1:length(nScanIndex)
   
    % Retrieve each scan's points
    nIndex = nScanIndex(i);
    I = nIndex == Lidar_ScanIndex;
    la = Lidar_Angles(I,:);
    lr = Lidar_Ranges(I,:);
    
    % Plot the scan
    set(0, 'CurrentFigure', 3);
    clf;
    polar(0, 25, '.r'); % axis limit hack
    hold on
    polar(la, lr, '.b');
    %view(-90,90);
    drawnow();
    pause(0.01);
end
