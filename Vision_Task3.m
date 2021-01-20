clear all
close all
clc
% General Settings
squareSize = 22; % in millimeters
% Read image
imRobotStation = imread('CheckerBoard.png');
figure; imshow(imRobotStation);
title('Input Image');
% Load camera parameters. The intrinsics will be used for further
% calculations.
load LabCameraParams.mat
% Detect the checkerboard corners in the images.
[imagePoints, boardSize] = detectCheckerboardPoints(imRobotStation);
% Generate the world coordinates of the checkerboard corners in the
% pattern-centric coordinate system, with the upper-left corner at (0,0).
worldPoints = generateCheckerboardPoints(boardSize, squareSize);
% Compute rotation and translation of camera wrt world
[R, t] = extrinsics(imagePoints, worldPoints, cameraParams);
% Verification: Map points in the image coordinates to points in the
% world coordinate
for i = 1:length(worldPoints)
worldPointsMapped(i,:) = pointsToWorld(cameraParams, R, t,...
imagePoints(i,:));
end
figure, plot (worldPoints(:,1),worldPoints(:,2),'x')
hold on, plot (worldPointsMapped(:,1),worldPointsMapped(:,2),'x')
legend('Points in Image Coordinate','Points in World Coordinate')

% start of Task 3
% 4 corner points wrt. robot frame
Xr = [-78.3873,-56.3907,-78.7713,-56.7747];
Yr = [177.619,177.235,155.623,155.239];
Zr = [-20.0013,-19.9636,-20.0404,-20.0027];
% List of joint angles and Cartesian positions of tool tip
Cartersian = [Xr;Yr;Zr];
theta_1 = [113.8130,107.6495,116.8470,110.0887];
theta_2 = [43.5326,45.2994,47.6261,49.3891];
theta_3 = [-98.8290,102.8517,-108.3607,-112.5781];
JointAngels = [theta_1;theta_2;theta_3];

% 4 corner points wrt. world frame
X = [22.0006,44.0047,21.9882,44.0049];
Y = [21.9567,21.9661,43.9583,43.9598];


coefficient = [sum(X.*X), sum(Y.*X), sum(X);
    sum(X.*Y), sum(Y.*Y), sum(Y);
    sum(X), sum(Y), 4];

mat_1 = (inv(coefficient)*[sum(Xr.*X); sum(Xr.*Y); sum(Xr)]);
mat_2 = (inv(coefficient)*[sum(Yr.*X); sum(Yr.*Y); sum(Yr)]);
mat_3 = (inv(coefficient)*[sum(Zr.*X); sum(Zr.*Y); sum(Zr)]);

r_13 = mat_1(1)*mat_1(2);
r_23 = mat_2(1)*mat_2(2);
r_33 = mat_3(1)*mat_3(2);

trans_matrix = [mat_1(1), mat_1(2), r_13, mat_1(3);
    mat_2(1), mat_2(2), r_23, mat_2(3);
    mat_3(1), mat_3(2), r_33, mat_3(3);
    0, 0, 0, 1];
Rrob = trans_matrix(1:3, 1:3);
trob = trans_matrix(1:3, 4);

disp('Transformation Matrix:')
disp(trans_matrix)
disp(Rrob)
disp(trob)

