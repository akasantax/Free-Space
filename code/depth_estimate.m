%% reading the stereo images
I1 = imread('/home/sahdev/Desktop/Spring 2016/Autonomous Driving_UofT/Project/KITTI/dataset2/data_left/0000000004.png');
I2 = imread('/home/sahdev/Desktop/Spring 2016/Autonomous Driving_UofT/Project/KITTI/dataset2/data_right/0000000004.png');
% I1 = imread('/home/sahdev/Desktop/Spring 2016/Autonomous Driving_UofT/Project/codes_mine/displets/data/Kitti/testing/image_0/000000_11.png');
% I2 = imread('/home/sahdev/Desktop/Spring 2016/Autonomous Driving_UofT/Project/codes_mine/displets/data/Kitti/testing/image_1/000000_11.png');


% I1_g = rgb2gray(I1);
% I2_g = rgb2gray(I2);

resolution_x = 1242;
resolution_y = 375;

%% computing disparity
disparityRange = [0, 64];
% disparityMap = disparity(rgb2gray(I1), rgb2gray(I2), 'DisparityRange', ...
%     disparityRange);
disparityMap = disparity(rgb2gray(I1), rgb2gray(I2), 'DisparityRange', ...
    disparityRange);
imshow(disparityMap, disparityRange, 'InitialMagnification', 50);
colormap('jet');
colorbar;
title('Disparity Map');

%% computing the Measurements, mk ; points (u,v,d)
focal = 500;
baseline = 0.54;

U = zeros(resolution_x*resolution_y,1);
V = zeros(resolution_x*resolution_y,1);
D = zeros(resolution_x*resolution_y,1);
M = zeros(resolution_x*resolution_y,3);
m = zeros(resolution_y,resolution_x,3);

focal = 500;
base_line = 0.54;
% disparityMap = I;
for i=1:resolution_y
    for j=1:resolution_x
        U((i-1)*resolution_x+j,1) = j;
        V((i-1)*resolution_x+j,1) = i;
        D((i-1)*resolution_x+j,1) = disparityMap(i,j);
        
        m(i,j,1) = j;
        m(i,j,2) = i;
        m(i,j,3) = disparityMap(i,j);
        
        M((i-1)*resolution_x+j,1) = j;
        M((i-1)*resolution_x+j,2) = i;
        M((i-1)*resolution_x+j,3) = (focal*baseline)/disparityMap(i,j);  
    end
end
% X = (M(:,3)./focal).* M(:,1);
% Y = (M(:,3)./focal).* M(:,2);
% Z = M(:,3);

X = M(:,1);
Y = M(:,2);
Z = M(:,3);
% X = X - resolution_x/2;

depth_threshold = 240;

res = size(Z);
for i=1:res(1,1)
    if(Z(i,1) == Inf || Z(i,1) > depth_threshold)
        Z(i,1) = depth_threshold;
%     elseif (Z(i,1) <10)
%         Z(i,j) = 0;
    end
end

M2 = [X,Y,Z];

figure,
pc = pointCloud(M);
pc2 = pointCloud(M2);
pcshow(M2);
pc_denoise = pcdenoise(pc);
figure,pcshow(pc);
pcshow(pc_denoise);
figure;
scatter3(X,Z,Y);
 u0 = resolution_x/2;
 v0 = resolution_y/2;
  
 grid_x = 124; %1242 pixels along x = 
 grid_z = 48;  % 240 pixels in depth = 48 grids 5 in each
 grid = zeros(grid_z,grid_x);
 res = 10;
 
 Likelihood_Dij = zeros(grid_z,grid_x); 
 res_x = 10;
 res_z = 5;
 
 for k=1:resolution_x * resolution_y
    x = M2(k,1);
    z = floor(M2(k,3));
    d = D(k,1);
    
    CovM =  [M2(k,1) ; M2(2,2) ; d];
    covm = cov(CovM);
    [i,j] = find_grid(x,z);
    u_ij = floor (  ((i-1)*res_x + i*res_x)/2 );
    d_ij = floor (  ((j-1)*res_z + j*res_z)/2  );
    delU = u_ij - x;
    delD = d_ij - d;
    
    Likelihood_Dij(j,i) = Likelihood_Dij(j,i) + gaussianPDF(delU,delD,covm);
 end

 %% visualizing likelihood
 Lxyz = zeros(grid_x*grid_z,3);
 
 for i=1:grid_x
     for j=1:grid_z
         
         Lxyz((i-1)*grid_z+j,1) = i;
         Lxyz((i-1)*grid_z+j,2) = j;
         Lxyz((i-1)*grid_z+j,3) = Likelihood_Dij(j,i);
         
     end
 end
 
 Likelihood = (Likelihood_Dij > 0);
 figure, imshow(Likelihood);
 
 %% Dynamic programming to segment the image
 
 % to implement cost(ijkl) = Ed_ij + Es_ijkl
 % computing Ed_ij
 Ed_ij = zeros(grid_z,grid_x);
 
 for i=1:grid_z
     for j=1:grid_x
         temp = 1/Likelihood_Dij(i,j);
         if(temp == Inf)
             temp = 99999;
         end
         Ed_ij(i,j) = temp;
     end
 end
 
 % computing Es_ijkl
 Ts = 1220; % some threshold which saturates the cost function - empirically defined 
 Cs = 4;    % constant cost parameter penalizing jumps in depth
 
 
 
%  S_jl = ;
 
 
 
 [minn,vertex] = min(Ed_ij(10:46,:));
 graphIt = zeros(grid_z,grid_x);
 for i=1:grid_x     
     graphIt(vertex(1,i),i) = 1;         
 end
 
 
 figure, imshow(graphIt);
 Likelihood_Dij = Likelihood_Dij*100000;
 %% try with simple thresholding too.
 
 threshold = 0.00001;
  graph_It = zeros(grid_z,grid_x);
 
 for i=1:grid_x
     flag = 1;
     for j=1:grid_z
         if(flag==0)
             break;
         end
         if (Likelihood_Dij(grid_z-j+1,i) > 0.001)
             graph_It(j,i) = 1;
             flag = 0;
         end
     end
 end
 
 
 
 
 
 
 
 
 
 %% garbage
%  for i=1:grid_x
%      for j=1:grid_z
%          
%          l1 = (i-1) * res;
%          l2 = (i) * res;
%          l3 = (j-1) * res;
%          l4 = (j) * res;
%          
%          grid_ij = M(l1:l2, : , l3:l4);
%          
% %          u_ij = 
% %          z_ij = 
% %          d_ij = 
%          
%          % check existence of a pixel here to compute the occupancy
%          
%          
%          
% %          grid_ij = m( l1:l2 , l3:l4 ,: );
% %          
% %          p_ij = m( floor(((i-1)*res+i*res)/2) , floor(((j-1)*res+j*res)/2) ,:);
% %         
%          
%      end
%  end
%  
 
 
 
 
%   R2 = [9.995599e-01 1.699522e-02 -2.431313e-02 ;
%      -1.704422e-02 9.998531e-01 -1.809756e-03 ;
%      2.427880e-02 2.223358e-03 9.997028e-01];
%  T2 = [-4.731050e-01 5.551470e-03 -5.250882e-03];
%  
% 
%  
 
%  Instrinsic1 = [9.597910e+02 0.000000e+00 6.960217e+02 ;
%                 0.000000e+00 9.569251e+02 2.241806e+02 ;
%                 0.000000e+00 0.000000e+00 1.000000e+00];
%  Intrinsic2 = [9.037596e+02 0.000000e+00 6.957519e+02 ;
%                0.000000e+00 9.019653e+02 2.242509e+02 ;
%                0.000000e+00 0.000000e+00 1.000000e+00];  
%  
%  camera1 = cameraParameters();
%  
%  stereoParams = stereoParameters(camera1,camera2,R2,T2);

 
%  calib = loadCalibrationCamToCam('/home/sahdev/Desktop/Spring 2016/Autonomous Driving_UofT/Project/KITTI/dataset2/2011_09_26/calib_cam_to_cam.txt');
 
%  S_02: 1.392000e+03 5.120000e+02
% K_02: 9.597910e+02 0.000000e+00 6.960217e+02 0.000000e+00 9.569251e+02 2.241806e+02 0.000000e+00 0.000000e+00 1.000000e+00
% D_02: -3.691481e-01 1.968681e-01 1.353473e-03 5.677587e-04 -6.770705e-02
% R_02: 9.999758e-01 -5.267463e-03 -4.552439e-03 5.251945e-03 9.999804e-01 -3.413835e-03 4.570332e-03 3.389843e-03 9.999838e-01
% T_02: 5.956621e-02 2.900141e-04 2.577209e-03
% S_rect_02: 1.242000e+03 3.750000e+02
% R_rect_02: 9.998817e-01 1.511453e-02 -2.841595e-03 -1.511724e-02 9.998853e-01 -9.338510e-04 2.827154e-03 9.766976e-04 9.999955e-01
% P_rect_02: 7.215377e+02 0.000000e+00 6.095593e+02 4.485728e+01 0.000000e+00 7.215377e+02 1.728540e+02 2.163791e-01 0.000000e+00 0.000000e+00 1.000000e+00 2.745884e-03
%  
 
 

% S_03: 1.392000e+03 5.120000e+02
% K_03: 9.037596e+02 0.000000e+00 6.957519e+02 0.000000e+00 9.019653e+02 2.242509e+02 0.000000e+00 0.000000e+00 1.000000e+00
% D_03: -3.639558e-01 1.788651e-01 6.029694e-04 -3.922424e-04 -5.382460e-02
% R_03: 9.995599e-01 1.699522e-02 -2.431313e-02 -1.704422e-02 9.998531e-01 -1.809756e-03 2.427880e-02 2.223358e-03 9.997028e-01
% T_03: -4.731050e-01 5.551470e-03 -5.250882e-03
% S_rect_03: 1.242000e+03 3.750000e+02
% R_rect_03: 9.998321e-01 -7.193136e-03 1.685599e-02 7.232804e-03 9.999712e-01 -2.293585e-03 -1.683901e-02 2.415116e-03 9.998553e-01
% P_rect_03: 7.215377e+02 0.000000e+00 6.095593e+02 -3.395242e+02 0.000000e+00 7.215377e+02 1.728540e+02 2.199936e+00 0.000000e+00 0.000000e+00 1.000000e+00 2.729905e-03
