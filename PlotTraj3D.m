function h = PlotTraj3D( x, y, z, rx, ry, rz, l)
%PLOTTRAJ3D  Plot a Trajectory in 3D with rotations
%   Dereck Wonnacott (c) 2014

% Inspired by http://www.mathworks.com/matlabcentral/fileexchange/24589-kinematics-toolbox/content/kinematics/screws/drawframe.m

    % Check for missing inputs
    if ~exist('rx', 'var') 
        rx = zeros(size(x));
    end
    if ~exist('ry', 'var') 
        ry = zeros(size(x));
    end
    if ~exist('rz', 'var') 
        rz = zeros(size(x));
    end
    if ~exist('l', 'var') 
        l = 0.25;
    end

    % Check array lengths
    nLen = length(x);
    if nLen ~= length(y)  || nLen ~= length(z)  || ...
       nLen ~= length(rx) || nLen ~= length(ry) || nLen ~= length(rz)
        error('Vector Length Mismatch')
    end
    
    

    % Generate Rotation Matrix for axis vectors (RPY)
    % http://www.mathworks.com/matlabcentral/fileexchange/35970-calcuate-euler-angles-from-rotation-matrix
    rx = - rx;
    ry = - ry;
    rz = - rz;
    R = [ 
          cos(ry).*cos(rz)                            , -cos(ry).*sin(rz)                            ,  sin(ry),  ...
          cos(rx).*sin(rz) + cos(rz).*sin(rx).*sin(ry),  cos(rx).*cos(rz) - sin(rx).*sin(ry).*sin(rz), -cos(ry).*sin(rx),  ...
          sin(rx).*sin(rz) - cos(rx).*cos(rz).*sin(ry),  cos(rz).*sin(rx) + cos(rx).*sin(ry).*sin(rz),  cos(rx).*cos(ry) ];
    M = reshape(R', 3, 3, []);

    
    % Rotate Axis Points
    Qx = zeros(nLen, 3);
    Qy = Qx;
    Qz = Qx;
    for i = 1:nLen
        Qx(i,:) = [l 0 0] * M(:,:,i);
        Qy(i,:) = [0 l 0] * M(:,:,i);
        Qz(i,:) = [0 0 l] * M(:,:,i);
    end
    
    % Translate to position
    Qx(:,1) = Qx(:,1) + x;
    Qx(:,2) = Qx(:,2) + y;
    Qx(:,3) = Qx(:,3) + z;
    
    Qy(:,1) = Qy(:,1) + x;
    Qy(:,2) = Qy(:,2) + y;
    Qy(:,3) = Qy(:,3) + z;
    
    Qz(:,1) = Qz(:,1) + x;
    Qz(:,2) = Qz(:,2) + y;
    Qz(:,3) = Qz(:,3) + z;
    
    
    % Plot em
    h(1) = plot3(Qx(:,1), Qx(:,2), Qx(:,3), 'r');
    h(2) = plot3(Qy(:,1), Qy(:,2), Qy(:,3), 'g');
    h(3) = plot3(Qz(:,1), Qz(:,2), Qz(:,3), 'b');
    h(4) = plot3(x, y, z, 'k');
    
    % Plot Initial Axis Markers
    h(5) = line([x(1) Qx(1,1)], [y(1) Qx(1,2)], [z(1) Qx(1,3)], 'Color', 'r', 'LineWidth', 2);
    h(6) = line([x(1) Qy(1,1)], [y(1) Qy(1,2)], [z(1) Qy(1,3)], 'Color', 'g', 'LineWidth', 2);
    h(7) = line([x(1) Qz(1,1)], [y(1) Qz(1,2)], [z(1) Qz(1,3)], 'Color', 'b', 'LineWidth', 2);
end

