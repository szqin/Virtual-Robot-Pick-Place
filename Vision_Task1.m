clear
close all
clc

%%%%%%%%%%%%%%%%%
% Corner Ponts %
%%%%%%%%%%%%%%%%%

% Data from https://www.microsoft.com/en-us/research/project/a-flexible-new-technique-for-camera-calibration-2/?from=http%3A%2F%2Fresearch.microsoft.com%2F%7Ezhang%2Fcalib%2F

load Model.txt;

X = Model(:,1:2:end);
Y = Model(:,2:2:end);

X = X(:)';
Y = Y(:)';

numberImages = 5; 

load data1.txt;
load data2.txt;
load data3.txt;
load data4.txt;
load data5.txt;

x = data1(:,1:2:end);
y = data1(:,2:2:end);
x = x(:)';
y = y(:)';
imagePoints(:,:,1) = [x;y];

x = data2(:,1:2:end);
y = data2(:,2:2:end);
x = x(:)';
y = y(:)';
imagePoints(:,:,2) = [x;y];

x = data3(:,1:2:end);
y = data3(:,2:2:end);
x = x(:)';
y = y(:)';
imagePoints(:,:,3) = [x;y];

x = data4(:,1:2:end);
y = data4(:,2:2:end);
x = x(:)';
y = y(:)';
imagePoints(:,:,4) = [x;y];

x = data5(:,1:2:end);
y = data5(:,2:2:end);
x = x(:)';
y = y(:)';
imagePoints(:,:,5) = [x;y];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Continue with your own code %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
phi = zeros(64*4*2,9,5); % creat five 512x9 zeros matrices

% Step 4 b) Build the matrix of phi
for layer_num = 1:5 
    for odd_row = 1:2:512
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
h = zeros(9,5);
H = zeros(3,3,5);
for layer_num = 1:5
    [U,S,V] = svd(phi(:,:,layer_num));
    % obtain h vector, 9 is the last column
    h(:,layer_num) = V(:,9);
    % obtain the camera matrix
    H(:,:,layer_num)=[h(1,layer_num),h(2,layer_num),h(3,layer_num);
        h(4,layer_num),h(5,layer_num),h(6,layer_num);
        h(7,layer_num),h(8,layer_num),h(9,layer_num)];
end

% Step 4 d) use the known h values to calculate vT11 vT22 vT12
vT11 = zeros(1,6,5);
vT22 = zeros(1,6,5);
vT12 = zeros(1,6,5);

for layer_num = 1:5
    vT11(:,:,layer_num) = vTij(1,1,layer_num,H);
    vT22(:,:,layer_num) = vTij(2,2,layer_num,H);
    vT12(:,:,layer_num) = vTij(1,2,layer_num,H);
end

% Step 5 Build the v matrix
v = zeros(10,6);
for layer_num = 1:5
    v(2*layer_num-1,:) = vT12(:,:,layer_num);
    v(2*layer_num,:) = vT11(:,:,layer_num)-vT22(:,:,layer_num);
end

% Step 6 Calculate the b vector using SVD method
[U,S,V] = svd(v);
b = V(:,6);
b11 = b(1); % assign elements to b11-b33
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
trans_matrix = zeros(3,4,5); %create 5 3x4 zeros matrices
for layer_num = 1:5
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