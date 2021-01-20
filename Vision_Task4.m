clc
clear
close all

% load previous files
load LabCameraParams.mat
load R.mat
load t.mat
load Rrob.mat
load trob.mat

% import image and convert into grey
I = imread('task_4.bmp');
figure, imshow(I),title("Original Image")

IRed = double(I(:,:,1));
IGreen = double(I(:,:,2));
IBlue = double(I(:,:,3));
IGrey = (IRed+IGreen+IBlue)/3;
I = uint8(IGrey);
figure, imshow(I),title("Grey")

% Thresholding
I = double(I);
[m,n] = size(I);
INegative = ones(m, n) * 255  - I;
INegative = uint8(INegative);
I = double(INegative);
[m, n] = size(I);
IThres = zeros(m, n);
for i = 1:m
   for j = 1:n
       if I(i,j) > 180
           IThres(i,j) = 255;
       else 
           IThres(i,j) = 0;
       end
   end
end
IThres = uint8(IThres);
figure, imshow(IThres),title("Threshold")

% Binary
Ibw = imbinarize(IThres);
figure,imshow(Ibw);
title('Binary')

%%% find bounding box

imin_old = m+1;
imax_old = 0;
jmin_old = n+1;
jmax_old = 0;
for i = 1:m % rows from up to down
    for j = 1:n % columns from left to right
        if Ibw(i,j) == 1
            if i < imin_old
                imin_old = i;
            end
            if i > imax_old
                imax_old = i;
            end
            if j < jmin_old
                jmin_old = j;
            end
            if j > jmax_old
                jmax_old = j;
            end
        end
    end
end
imin = imin_old;     % first row
imax = imax_old;    % last row
jmin = jmin_old;    % first column
jmax = jmax_old;    % last column

%hold on, plot([jmin jmin jmax jmax jmin],[imin imax imax imin imin],'g')

% cut image
if (imax-imin)>(jmax-jmin)
    object1 = [imin, (imax+imin)/2;
               jmin, jmax];
    object2 = [(imax+imin)/2, imax;
               jmin, jmax];
else
    object1 = [imin, imax;
                jmin, (jmax+jmin)/2];
    object2 = [imin, imax;
                (jmax+jmin)/2,jmax];
end


%find bounding box of object one
imin_o = m+1;
imax_o = 0;
jmin_o = n+1;
jmax_o = 0;


fromi = round(object1(1,1));
toi = round(object1(1,2));
fromj = round(object1(2,1));
toj = round(object1(2,2));

for i = fromi: toi% rows from up to down
    for j = fromj:toj % columns from left to right
        if Ibw(i,j) == 1
            if i < imin_o
                imin_o = i;
            end
            if i > imax_o
                imax_o = i;
            end
            if j < jmin_o
                jmin_o = j;
            end
            if j > jmax_o
                jmax_o = j;
            end
        end
    end
end
imin1 = imin_o;     % first row
imax1 = imax_o;    % last row
jmin1 = jmin_o;    % first column
jmax1 = jmax_o;    % last column

hold on, plot([jmin1 jmin1 jmax1 jmax1 jmin1],[imin1 imax1 imax1 imin1 imin1],'g')

%find bounding box of the object two
imin_o = m+1;
imax_o = 0;
jmin_o = n+1;
jmax_o = 0;

fromi = round(object2(1,1));
toi = round(object2(1,2));
fromj = round(object2(2,1));
toj = round(object2(2,2));

for i = fromi: toi% rows from up to down
    for j = fromj:toj % columns from left to right
        if Ibw(i,j) == 1
            if i < imin_o
                imin_o = i;
            end
            if i > imax_o
                imax_o = i;
            end
            if j < jmin_o
                jmin_o = j;
            end
            if j > jmax_o
                jmax_o = j;
            end
        end
    end
end

imin2 = imin_o;    % first row
imax2 = imax_o;    % last row
jmin2 = jmin_o;    % first column
jmax2 = jmax_o;    % last column

hold on, plot([jmin2 jmin2 jmax2 jmax2 jmin2],[imin2 imax2 imax2 imin2 imin2],'g')
