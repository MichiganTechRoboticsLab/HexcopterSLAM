

%
% Load Data
%
VectorNav_Logfile = '/home/dereck/Documents/DataSets/signs/vn.csv';
ReadVectorNavLog

Hokuyo_Logfile = '/home/dereck/Documents/DataSets/signs/lidar_data.csv';
ReadHokuyoLog


% %
% % Show the GPS Data
% %
% figure(1)
% clf
% plot(GPS_Longitude, GPS_Lattitude, '.r', 'MarkerSize', 3) 
% plot_google_map('MapType', 'hybrid')  % Dont forget to add to path
% title('GPS Track');
% 
% figure(2)
% clf
% plot3(GPS_Longitude, GPS_Lattitude, GPS_Altitude, '.r', 'MarkerSize', 20) 
% title('GPS Track (3D)');
% grid
% 
% figure(3)
% clf
% subplot(3,1,1);
% plot(GPS_Timestamp, GPS_Longitude, '.r')
% title('GPS Longitude');
% subplot(3,1,2);
% plot(GPS_Timestamp, GPS_Lattitude, '.g')
% title('GPS Latitude');
% subplot(3,1,3);
% plot(GPS_Timestamp, GPS_Altitude, '.b')
% title('GPS Altitude');
% 
% 
% 
% %
% % Show the IMU Data
% %
% figure(4)
% clf
% subplot(3,1,1);
% plot(IMU_Timestamp, IMU_Pitch, '.r')
% title('IMU Pitch');
% subplot(3,1,2);
% plot(IMU_Timestamp, IMU_Roll, '.g')
% title('GPS Roll');
% subplot(3,1,3);
% plot(IMU_Timestamp, IMU_Yaw, '.b')
% title('IMU Yaw');
% 
% 
% 
% % Idnetify missing data
% figure(5)
% clf  
% hold on  
% subplot(1,2,1)
% plot(diff(GPS_Timestamp), '.r')
% title('Missing Data Identification (GPS)')
% subplot(1,2,2)
% plot(diff(IMU_Timestamp), '.b')
% title('Missing Data Identification (IMU)')
% 
% 
% 
% % Plot all orientations on same spot
% figure(6)
% clf
% PlotPose3D(0,0,0, 0,0,0, 1);
% hold on
% view(63, 24)
% axis([-1 1 -1 1 -1 1])
% title('IMU Orientation')
% grid
% 
% zv = zeros(size(IMU_Pitch));
% PlotTraj3D(zv, zv, zv, IMU_Pitch, IMU_Roll, IMU_Yaw, 0.9);
% 
% 
% 
% %
% % Show the interpolated GPS Location for each IMU message
% %
% figure(7)
% clf
% subplot(3,1,1);
% plot(IMU_Timestamp, IMU_MetricPose(:,1), '.r')
% title('IMU Pitch');
% subplot(3,1,2);
% plot(IMU_Timestamp, IMU_MetricPose(:,2), '.g')
% title('GPS Roll');
% subplot(3,1,3);
% plot(IMU_Timestamp, IMU_MetricPose(:,3), '.b')
% title('IMU Yaw');
% 
% 
% 
% %
% % Plot all orientations on the GPS Path
% %
% figure(8)
% clf
% nScale = 0.8;
% hold on
% %view(63, 24)
% axis equal
% grid
% title('IMU Orientation & GPS Pose')
% 
% PlotTraj3D(IMU_MetricPose(:,1), IMU_MetricPose(:,2), IMU_MetricPose(:,3), ...
%            IMU_Pitch, IMU_Roll, IMU_Yaw, nScale);
% 



%
% Plot Lidar Data
% 
figure(9)
clf
figure(10)
clf
tic
for i = 1:size(Lidar_Log, 1)
    % Remove out of range data
    a = Lidar_Angles;
    r = Lidar_Log(i, Lidar_Col_Ranges:end);
    I = or(r >= 60, r <= 0.35);
    a(I) = [];
    r(I) = [];

%     % Plot raw data
%     figure(9);
%     subplot(2,1,1);
%     plot(Lidar_Angles, Lidar_Log(i, Lidar_Col_Ranges:end), '.b');
%     title(['Raw Data: Index ' num2str(i)]);
%     xl = xlim;
%     hold on
%     
%     % Plot Filtered data
%     subplot(2,1,2);
%     if size(r,2) > 0
%         plot(a,r, '.b');
%         xlim(xl);
%         ylim([0 max(r)]);
%         title('Removed Out of Bounds Data');
%     end 
%     hold on
%     
%     % Plot Polar graph of data
%     figure(10);
%     polar(a, r, '.b');
%     hold on
%     %view(-90,90)
%     
    %drawnow();
    disp(i);
    %pause(0.1);
end
toc


