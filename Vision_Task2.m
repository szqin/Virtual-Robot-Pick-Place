%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This code is to calibrate the camera %
% given the images %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear
close all
clc
% General Settings
numImages = 6;
squareSize = 22; % in millimetres
% Read files
files = cell(1, numImages);
for i = 1:numImages
files{i} = fullfile(sprintf('Image%d.png', i));
end
% Display one of the calibration images
I = imread(files{1});
figure; imshow(I);
title('One of the Calibration Images');
% Detect the checkerboard corners in the images.
[imagePoints, boardSize] = detectCheckerboardPoints(files);
% Generate the world coordinates of the checkerboard corners in the
% pattern-centric coordinate system, with the upper-left corner at (0,0).
worldPoints = generateCheckerboardPoints(boardSize, squareSize);
% Calibrate the camera.
imageSize = [size(I, 1), size(I, 2)];
cameraParams = estimateCameraParameters(imagePoints, worldPoints, ...
'ImageSize', imageSize);
% Evaluate calibration accuracy.
figure; showReprojectionErrors(cameraParams);
title('Reprojection Errors');
% Show checkerboard wrt camera
figure;
showExtrinsics(cameraParams, 'CameraCentric');
% Show camera wrt checkerboard
figure;
showExtrinsics(cameraParams, 'PatternCentric');

% own calibration codes start here:
% Corner Points
X = worldPoints(:,1:2:end);
Y = worldPoints(:,2:2:end);
X = X(:)';
Y = Y(:)';
% rearrange the row and column of imagepoints
imagePoints = permute(imagePoints,[2 1 3]);

phi = zeros(54*2,9,6); % creat six 108x9 zeros matrices

% Step 4 b) Build the matrix of phi
for layer_num = 1:6
    for odd_row = 1:2:108
        i = (odd_row+1)/2;
        phi(odd_row,:,layer_num) = [0,0,0,X(i),Y(i),1,... %odd row
            -imagePoints(2,i,layer_num)*X(i),...
            -imagePoints(2,i,layer_num)*Y(i),...
            -imagePoints(2,i,layer_num)];
        
        phi(odd_row+1,:,layer_num)=[X(i),Y(i),1,0,0,0,... %even row
            -imagePoints(1,i,layer_num)*X(i),...
            -imagePoints(1,i,layer_num)*Y(i),...
            -imagePoints(1,i,layer_num)];
    end
end

% Step 4 c) Calculate the h vector using Singular Value Decomposition
h = zeros(9,6);
H = zeros(3,3,6);
for layer_num = 1:6
    [U,S,V] = svd(phi(:,:,layer_num));
    % obtain h vector, 9 is the last column
    h(:,layer_num) = V(:,9);
    % obtain the camera matrix
    H(:,:,layer_num)=[h(1,layer_num),h(2,layer_num),h(3,layer_num);
        h(4,layer_num),h(5,layer_num),h(6,layer_num);
        h(7,layer_num),h(8,layer_num),h(9,layer_num)];
end

% Step 4 d) use the known h values to calculate vT11 vT22 vT12
vT11 = zeros(1,6,6);
vT22 = zeros(1,6,6);
vT12 = zeros(1,6,6);

for layer_num = 1:6
    vT11(:,:,layer_num) = vTij(1,1,layer_num,H);
    vT22(:,:,layer_num) = vTij(2,2,layer_num,H);
    vT12(:,:,layer_num) = vTij(1,2,layer_num,H);
end

% Step 5 Build the v matrix
v = zeros(10,6);
for layer_num = 1:6
    v(2*layer_num-1,:) = vT12(:,:,layer_num);
    v(2*layer_num,:) = vT11(:,:,layer_num)-vT22(:,:,layer_num);
end

% Step 6 Calculate the b vector using SVD method
[U,S,V] = svd(v);
b = V(:,6);
b11 = b(1); %make elements clear for further calculation
b12 = b(2);
b13 = b(4);
b21 = b(2);
b22 = b(3);
b23 = b(5);
b31 = b(4);
b32 = b(5);
b33 = b(6);

% Step 7 Recover the intrinsic parameter
y0 = (b12*b13-b11*b23)/(b11*b22-b12^2);
lambda = b33-(b13^2+y0*(b12*b13-b11*b23))/b11;
alpha = sqrt(lambda/b11);
beta = sqrt((lambda*b11)/(b11*b22-b12^2));
gamma = -(b12*alpha^2*beta)/lambda;
x0 = (gamma*y0)/alpha-(b13*alpha^2)/lambda;
K = [alpha,gamma,x0 ; 0,beta,y0 ; 0,0,1];
display(K)

% Step 8 For the image where the checkerboard is roughly at the same
% height as the object, recover the extrinsic parameters
trans_matrix = zeros(3,4,6); %create 6 3x4 zeros matrices
for layer_num = 1:6
    sigma = 1/norm(inv(K)*H(:,1,layer_num));
    r1 = sigma*inv(K)*H(:,1,layer_num);
    r2 = sigma*inv(K)*H(:,2,layer_num);
    r3 = cross(r1,r2);
    PCWorg = sigma*inv(K)*H(:,3,layer_num);
    trans_matrix(:,1,layer_num) = r1;
    trans_matrix(:,2,layer_num) = r2;
    trans_matrix(:,3,layer_num) = r3;
    trans_matrix(:,4,layer_num) = PCWorg;
    %print results
    disp("Rotation and translation of the N0." + layer_num +' image:')
    disp(trans_matrix(:,:,layer_num))
end

% declare a new function vTij(i,j,layer_num,H)
function v = vTij(i,j,layer_num,H)
    v = transpose([H(1,i,layer_num)*H(1,j,layer_num);
    H(1,i,layer_num)*H(2,j,layer_num)+...
    H(2,i,layer_num)*H(1,j,layer_num);
    H(2,i,layer_num)*H(2,j,layer_num);
    H(1,i,layer_num)*H(3,j,layer_num)+...
    H(3,i,layer_num)*H(1,j,layer_num);
    H(2,i,layer_num)*H(3,j,layer_num)+...
    H(3,i,layer_num)*H(2,j,layer_num);
    H(3,i,layer_num)*H(3,j,layer_num)]);
end

