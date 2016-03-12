I1 = imread('/home/sahdev/Desktop/Spring 2016/Autonomous Driving_UofT/Project/KITTI/data_left/0000000000.png');
I2 = imread('/home/sahdev/Desktop/Spring 2016/Autonomous Driving_UofT/Project/KITTI/data_right/0000000000.png');
I1_g = rgb2gray(I1);
I2_g = rgb2gray(I2);



% tic
% d = disparity(I1,I2);
% imshow(d);
% toc

disparityRange = [0, 64];
disparityMap = disparity(rgb2gray(I1), rgb2gray(I2), 'DisparityRange', ...
    disparityRange);
imshow(disparityMap, disparityRange, 'InitialMagnification', 50);
colormap('jet');
colorbar;
title('Disparity Map');

K = [9.037596e+02 0.000000e+00 6.957519e+02;
    0.000000e+00 9.019653e+02 2.242509e+02;
    0.000000e+00 0.000000e+00 1.000000e+00];
R = [9.995599e-01 1.699522e-02 -2.431313e-02;
    -1.704422e-02 9.998531e-01 -1.809756e-03;
    2.427880e-02 2.223358e-03 9.997028e-01];
T = [-4.731050e-01 5.551470e-03 -5.250882e-03];

Z = double(disparityMap);
X = zeros(375,1242);
Y = zeros(375,1242);
for i=1:375
    for j=1:1242
        X(i,j) = j;
        Y(i,j) = i;
    end
end

P_X = X ./Z;
P_Y = Y./Z;
Points = zeros(375,1242,3);
for i=1:375
    for j=1:1242
        Points(i,j,:) = [P_X(i,j) P_Y(i,j) 1];
    end
end


% S_03: 1.392000e+03 5.120000e+02
% K_03: 9.037596e+02 0.000000e+00 6.957519e+02 0.000000e+00 9.019653e+02 2.242509e+02 0.000000e+00 0.000000e+00 1.000000e+00
% D_03: -3.639558e-01 1.788651e-01 6.029694e-04 -3.922424e-04 -5.382460e-02
% R_03: 9.995599e-01 1.699522e-02 -2.431313e-02 -1.704422e-02 9.998531e-01 -1.809756e-03 2.427880e-02 2.223358e-03 9.997028e-01
% T_03: -4.731050e-01 5.551470e-03 -5.250882e-03
% S_rect_03: 1.242000e+03 3.750000e+02
% R_rect_03: 9.998321e-01 -7.193136e-03 1.685599e-02 7.232804e-03 9.999712e-01 -2.293585e-03 -1.683901e-02 2.415116e-03 9.998553e-01
% P_rect_03: 7.215377e+02 0.000000e+00 6.095593e+02 -3.395242e+02 0.000000e+00 7.215377e+02 1.728540e+02 2.199936e+00 0.000000e+00 0.000000e+00 1.000000e+00 2.729905e-03
