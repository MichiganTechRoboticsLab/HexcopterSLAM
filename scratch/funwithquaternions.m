clear all;
close all;
clc

% YPR angles to rotate
yaw   = 0;
pitch = pi/4;
roll  = 0;

% Points to rotate
A = [[ 1 0 0]
     [ 0 1 0]
     [ 0 0 1]
     [-1 0 0]];

%A = rand(50,3) * 2*pi;

        
% using rotation matrices =============================
Ryaw   = [ cos(yaw)  -sin(yaw)   0
           sin(yaw)   cos(yaw)   0
           0          0          1];
       
Rpitch = [ cos(pitch) 0          sin(pitch)
           0          1          0
          -sin(pitch) 0          cos(pitch)];
      
Rroll  = [ 1          0          0
           0          cos(roll) -sin(roll)
           0          sin(roll)  cos(roll)];
       
R=Ryaw*Rpitch*Rroll;
rotM = (R*A')';



% Using Quaternions =============================
Q = angle2quat(yaw, pitch, roll);
[R1 R2 R3] = quat2angle(Q); % gets the right angle... but

% Compare results
AngleCheckQ = [abs(yaw   - R1) < 0.0001;
               abs(pitch - R2) < 0.0001;
               abs(roll  - R3) < 0.0001]

          
% This rotates with negative angles
Q = angle2quat(-yaw, -pitch, -roll);
rotQ = quatrotate(Q, A);

% Compare results
RotationCheck1 = abs(rotQ - rotM) < 0.0001



% Using DCM =============================
D = angle2dcm(yaw, pitch, roll);
[R1 R2 R3] = dcm2angle(D);

% Compare results
AngleCheckDCM = [abs(yaw   - R1) < 0.0001;
                 abs(pitch - R2) < 0.0001;
                 abs(roll  - R3) < 0.0001]
           
% Rotate vectors
rotD = (D*A')';

% Check Results 
RotationCheckDCM1 = abs(rotD - rotM) < 0.0001


% Check this function too
dq = quat2dcm(Q);     
RotationCheckDCM2 = abs(D - dq) < 0.0001
           

%
% Short version I need to negate the inputs into angle2quat for quatrotate
% to work.
%

% YPR angles to rotate
yaw   = pi/4;
pitch = 0;
roll  = 0;

% Point to rotate
A = [ 1 0 0]
%A = rand(50,3) * 2*pi;

% Rotate
Q = angle2quat(yaw, pitch, roll);
B = quatrotate(Q, A)

% Rotate (Negitive)
Q = angle2quat(-yaw, -pitch, -roll);
C = quatrotate(Q, A)



