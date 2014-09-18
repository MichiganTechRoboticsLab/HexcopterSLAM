function PlotPose3D( x, y, z, rx, ry, rz, l)
%PLOTPOSE  Plot a pose in 3D
%   Dereck Wonnacott (c) 2014

    % Check for missing inputs
    if nargin < 4 
        rx = 0;
    end
    if nargin < 5 
        ry = 0;
    end
    if nargin < 6 
        rz = 0;
    end
    if nargin < 7 
        l = 0.25;
    end
    
    % Generate Rotation Matrix for axis vectors (Euler Angles)
%     R =  [ cos(ry) * cos(rz)  cos(rx)*sin(rz) + sin(rx)*sin(ry)*cos(rz)  sin(rx)*sin(rz) - cos(rx)*sin(ry)*cos(rz)    
%           -cos(ry) * sin(rz)  cos(rx)*cos(rz) - sin(rx)*sin(ry)*sin(rz)  sin(rx)*cos(rz) + cos(rx)*sin(ry)*sin(rz)
%            sin(ry)           -sin(rx)*cos(ry)                            cos(rx)*cos(ry) ];
    
    % Generate Rotation Matrix for axis vectors (RPY)
    % http://www.mathworks.com/matlabcentral/fileexchange/35970-calcuate-euler-angles-from-rotation-matrix
    rx = - rx;
    ry = - ry;
    rz = - rz;
    R = [[ cos(ry)*cos(rz)                          , -cos(ry)*sin(rz),                            sin(ry)] 
         [ cos(rx)*sin(rz) + cos(rz)*sin(rx)*sin(ry),  cos(rx)*cos(rz) - sin(rx)*sin(ry)*sin(rz), -cos(ry)*sin(rx)] 
         [ sin(rx)*sin(rz) - cos(rx)*cos(rz)*sin(ry),  cos(rz)*sin(rx) + cos(rx)*sin(ry)*sin(rz),  cos(rx)*cos(ry)]];

    
    % Generate axis vectors
    ax = [l 0 0] * R + [x y z];
    ay = [0 l 0] * R + [x y z];
    az = [0 0 l] * R + [x y z];
    
    % Plot time
    hold on;
    axis equal;
    plot3([x ax(1)], [y ax(2)], [z ax(3)], 'r');
    plot3([x ay(1)], [y ay(2)], [z ay(3)], 'g');
    plot3([x az(1)], [y az(2)], [z az(3)], 'b');
end