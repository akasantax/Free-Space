tic
%% reading the stereo images
I1 = imread('/home/sahdev/Desktop/Spring 2016/Autonomous Driving_UofT/Project/codes_mine/displets/data/Kitti/testing/image_0/000013_10.png');
I2 = imread('/home/sahdev/Desktop/Spring 2016/Autonomous Driving_UofT/Project/codes_mine/displets/data/Kitti/testing/image_1/000013_10.png');
I3 = imread('/home/sahdev/Desktop/Spring 2016/Autonomous Driving_UofT/Project/codes_mine/displets/data/Kitti/testing/image_0/000000_11.png');
disparityMap2 = imread('/home/sahdev/Desktop/Spring 2016/Autonomous Driving_UofT/Project/codes_mine/displets/data/Kitti/testing/dispmaps/cnn/disp_0/000013_10.png');

% 
% 
% I1 = imread('/home/sahdev/Desktop/dATASETSS/Dataset_Arena_workplace/Left/L17h 53m 9sLimg129.png');
% I2 = imread('/home/sahdev/Desktop/dATASETSS/Dataset_Arena_workplace/Right/R17h 53m 9s Limg129.png');

% I1_g = rgb2gray(I1);
% I2_g = rgb2gray(I2);



%% computing disparity
disparityRange = [0, 64];
% disparityMap = disparity(rgb2gray(I1), rgb2gray(I2), 'DisparityRange', ...
%     disparityRange);
disparityMap = disparity((I1), (I2), 'DisparityRange', ...
    disparityRange);
imshow(disparityMap, disparityRange, 'InitialMagnification', 50);
colormap('jet');
colorbar;
title('Disparity Map');



dispThresh1 = 2000;
dispThresh2 = 20000;

D1 = disparityMap2 < dispThresh1;
D2 = disparityMap2 > dispThresh2;

res2 = size(D1);
for i=1:res2(1,1)
    for j=1:res2(1,2)
        if(D1(i,j) ==  1)
            disparityMap2(i,j) = dispThresh1;
        elseif (D2(i,j) ==  1)
             disparityMap2(i,j) = dispThresh2;
        end
    end
end

disparityMap2 = disparityMap2./200;
imshow(disparityMap2, [0,100], 'InitialMagnification', 50);
colormap('jet');
colorbar;
title('Disparity Map');  



%% computing the Measurements, mk ; points (u,v,d)
focal = 960;  % focal length in pixels
 grid_x = 124; %1242 pixels along x = 124 grids 10 each
 grid_z = 50;  % 240 pixels in depth = 48 grids 5 in each
 depth_threshold = 500;
baseline = 0.54;
resolution_x = 1241;
resolution_y = 375;


% U stores the x coordinate of the image
% V stores the y coordinate of the image
% D stores the disparities of the image


U = zeros(resolution_x*resolution_y,1);
V = zeros(resolution_x*resolution_y,1);
D = zeros(resolution_x*resolution_y,1);
M = zeros(resolution_x*resolution_y,3);
m = zeros(resolution_y,resolution_x,3);

 
 
% disparityMap = I;
for i=1:resolution_y
    for j=1:resolution_x
        U((i-1)*resolution_x+j,1) = j;
        V((i-1)*resolution_x+j,1) = i;
        D((i-1)*resolution_x+j,1) = disparityMap2(i,j);
        
        m(i,j,1) = j;
        m(i,j,2) = i;
        m(i,j,3) = disparityMap2(i,j);
        
        z = 10 *(focal*baseline)/disparityMap2(i,j); % 10 is scaling factor added
        x = j; % *(z/focal)  
        y = i; % *(z/focal)
        M((i-1)*resolution_x+j,1) = x; 
        M((i-1)*resolution_x+j,2) = y;
        M((i-1)*resolution_x+j,3) = z;  
    end
end
% X = (M(:,3)./focal).* M(:,1);
% Y = (M(:,3)./focal).* M(:,2);
% Z = M(:,3);

X = M(:,1);
Y = M(:,2);
Z = M(:,3);
% X = X - resolution_x/2;



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
% pcshow(M2);
pc_denoise = pcdenoise(pc);
% figure,pcshow(pc);
pcshow(pc_denoise), title('Thresholded depth!!');
% figure;
% scatter3(X,Z,Y);
 u0 = resolution_x/2;
 v0 = resolution_y/2;
  
%% computing occupancy likelihoods
 
 grid = zeros(grid_z,grid_x);
 res = 10;
 
 Likelihood_Dij = zeros(grid_z,grid_x); 
 res_x = 10;
 res_z = 10;
 toc
 
 for k=1:resolution_x * resolution_y
    x = M2(k,1);
    z = floor(M2(k,3));
    d = D(k,1);
    
    CovM =  [M2(k,1) ; M2(2,2) ; d];
    covm = cov(CovM);
    [i,j] = find_grid(x,z,res_x,res_z);
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
 figure, imshow(Likelihood), title('Binary Occupancy.');
 
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
 
%  figure, imshow(graphIt);

Likelihood_Dij = Likelihood_Dij*100000;
 %% implementing transitions (from white to black and vice versa) based free space estimation
 image = Likelihood_Dij == 0;
 image_target = zeros(grid_z,grid_x);
 
 target_pts = zeros(1,2);
 ant = 1;
 for i=1:grid_x
     cnt=0;
     
     for j=1:grid_z-1
         
         if(image(j,i) == 0 && image(j+1,i) ==1)
             cnt=cnt+1;
         elseif(image(j,i) == 1 && image(j+1,i) ==0)
             cnt=cnt+1;
         end
         if(cnt == 2)
             image_target(j,i) = 1;
             target_pts(ant,1) = j;
             target_pts(ant,2) = i;
             ant = ant+1;
             break;
         end
%          if(image(j,i) == 1 && cnt == 0)
%              cnt=cnt+1;           
%          elseif(cnt==1 && image(j,i) == 0)
%              image_target(j,i) = 1;
%              cnt=0;
%              break;
%          end
     end     
 end
 
 figure, imshow(image_target), title('coorrect one');
 figure, imshow(Likelihood_Dij,[0,10]), title('likelihood analog');
 
 
 %% segmentation of the free space
 tic
 actual_pts = zeros(1,2);
 
size_tpts =  size(target_pts);
  for i=1:size_tpts(1,1)
      for j=1:res_x
          [tx,ty] = findImageCoordinate(disparityMap2,target_pts(i,2),target_pts(i,1));
          actual_pts((i-1)*res_x+j,1) = tx;
          actual_pts((i-1)*res_x+j,2) = ty;      
      end
  end
figure, imshow(I1), hold on, plot(actual_pts(:,1),375 - actual_pts(:,2),'Color','red','LineWidth',2);
  
toc
  %% plotting free space on the image
%   [m,indx2] = max(image_target);
%   grid_coordinates = zeros(grid_x,2);
%   
%   % getting x and z from the occupancy grid map
%   for i=1:grid_x
%       grid_coordinates(i,1) = i;
%       grid_coordinates(i,2) = indx(1,i);      
%   end
%   for k=1:grid_x
%       
%       i = grid_coordinates(k,1) * res_x;
%       j = grid_coordinates(k,2) * res_z;
%       
%       y_s = findPoints(i,j,res_x,res_z,M2);
%       
%       x = M2(k,1);
%       y = M2(k,1);
%       z = floor(M2(k,3));
%       d = D(k,1);
% 
%       [i,j] = find_grid(x,z,res_x,res_z);
%       u_ij = floor (  ((i-1)*res_x + i*res_x)/2 );
%       d_ij = floor (  ((j-1)*res_z + j*res_z)/2  );
%   end  
  
 
 %% try with simple thresholding too.
  
%  threshold = 1;
%   graph_It = zeros(grid_z,grid_x);
%  
%  for i=1:grid_x
%      flag = 1;
%      for j=4:grid_z
%          if(flag==0)
%              break;
%          end
%          if (Likelihood_Dij(grid_z-j+4,i) > threshold)
%              graph_It(j,i) = 1;
%              flag = 0;
%          end
%      end
%  end
%  
% %  figure, imshow(graph_It);
%  
%  
%  
%  
%  
%  
%  
%  %% play around with the likelihoods
%  graph2 = zeros(grid_z,grid_x);
%  for i=1:grid_x
%      [m1,ind] = max(Likelihood_Dij(:,i));
%      graph2(ind,i) = 1;
%  end
%  
%  %% play around with likelihoods 2
%  pos = 1;
%  graph3 = zeros(grid_z,grid_x);
%  for i=1:grid_x
%      for j=1:grid_z
%         if(Likelihood_Dij(grid_z-j+1,i) > 1)
%             pos=j;
%             break;
%         end
%      end
%      graph3(pos,i) = 1;
%  end
