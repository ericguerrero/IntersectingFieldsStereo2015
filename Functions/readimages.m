function [I1, I2] = readimages(t1, t2, Secuencia)
path = ['\Secuencias\' Secuencia '\'];
nameA1 = num2str(t1(1),'%010d');
nameB1 = num2str(t1(2),'%06d');
nameA2 = num2str(t2(1),'%010d');
nameB2 = num2str(t2(2),'%06d');
I1=imread([path,'cam01_', nameA1,'.', nameB1,'.pgm']);
I2=imread([path,'cam02_', nameA2,'.', nameB2,'.pgm']);
I1=single(mat2gray(rgb2gray(I1)));
I2=single(mat2gray(rgb2gray(I2)));

% I1u = undistort(I1, cam1);figure;imshow(I1);figure();imshow(I1u)
% [I1u,newOrigin1] = undistortImage(I1,cameraParams1);figure();imshow(I1);figure();imshow(I1u)
% [I2u,newOrigin2] = undistortImage(I2,cameraParams2);figure();imshow(I2);figure();imshow(I2u)
end