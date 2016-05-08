
% function model_fitting_RANSAC_linear(I1,I2)

%% Find Corresponding Interest Points Between Pair of Images
% Read the stereo images.
% I1 = rgb2gray(imread('/home/sahdev/Desktop/Spring 2016/Autonomous Driving_UofT/Project/codes_mine/displets/data/Kitti/testing/image_0/000000_10.png'));
% I2 = rgb2gray(imread('/home/sahdev/Desktop/Spring 2016/Autonomous Driving_UofT/Project/codes_mine/displets/data/Kitti/testing/image_0/000000_10.png'));
% I1 = (imread('/home/sahdev/Desktop/Spring 2016/Autonomous Driving_UofT/datasets/2011_09_28 (2)/2011_09_28_drive_0038_sync/image_00/data/0000000107.png'));
% I2 = (imread('/home/sahdev/Desktop/Spring 2016/Autonomous Driving_UofT/datasets/2011_09_28 (2)/2011_09_28_drive_0038_sync/image_00/data/0000000108.png'));
% I1 = (imread('/home/sahdev/Desktop/Spring 2016/Autonomous Driving_UofT/datasets/2011_09_26 (2)/2011_09_26_drive_0046_sync/image_00/data/0000000000.png'));
% I2 = (imread('/home/sahdev/Desktop/Spring 2016/Autonomous Driving_UofT/datasets/2011_09_26 (2)/2011_09_26_drive_0046_sync/image_00/data/0000000001.png'));
I1 = (imread('/home/sahdev/Desktop/Spring 2016/Autonomous Driving_UofT/datasets/2011_09_26/2011_09_26_drive_0035_sync/image_00/data/0000000002.png'));
I2 = (imread('/home/sahdev/Desktop/Spring 2016/Autonomous Driving_UofT/datasets/2011_09_26/2011_09_26_drive_0035_sync/image_00/data/0000000003.png'));

% disparity_Map = imread('/home/sahdev/Desktop/Spring 2016/Autonomous Driving_UofT/Project/codes_mine/displets/data/Kitti/testing/dispmaps/sgm/disp_0/000132_10.png');

threshold_residual = 22;

%% Find the features.
points1 = detectHarrisFeatures(I1);
points2 = detectHarrisFeatures(I2);
points1a = detectBRISKFeatures(I1);
points2a = detectBRISKFeatures(I2);


points3 = detectMSERFeatures(I1);
points4 = detectMSERFeatures(I2);
points3a = detectSURFFeatures(I1);
points4a = detectSURFFeatures(I2);

%% Extract the neighborhood features.
[features1,valid_points1] = extractFeatures(I1,points1);
[features2,valid_points2] = extractFeatures(I2,points2);
[features1a,valid_points1a] = extractFeatures(I1,points1a);
[features2a,valid_points2a] = extractFeatures(I2,points2a);


[features3,valid_points3] = extractFeatures(I1,points3);
[features4,valid_points4] = extractFeatures(I2,points4); 
[features3a,valid_points3a] = extractFeatures(I1,points3a);
[features4a,valid_points4a] = extractFeatures(I2,points4a);  

%% Match the features.
indexPairs1 = matchFeatures(features1,features2);   
indexPairs2 = matchFeatures(features3,features4); 

indexPairs1a = matchFeatures(features1a,features2a);   
indexPairs2a = matchFeatures(features3a,features4a);   

%% Retrieve the locations of the corresponding points for each image.
matchedPoints1 = valid_points1(indexPairs1(:,1),:);
matchedPoints2 = valid_points2(indexPairs1(:,2),:); 
matchedPoints3 = valid_points3(indexPairs2(:,1),:);
matchedPoints4 = valid_points4(indexPairs2(:,2),:);

matchedPoints1a = valid_points1a(indexPairs1a(:,1),:);
matchedPoints2a = valid_points2a(indexPairs1a(:,2),:); 
matchedPoints3a = valid_points3a(indexPairs2a(:,1),:);
matchedPoints4a = valid_points4a(indexPairs2a(:,2),:);

%%  Visualize the corresponding points. You can see the effect of translation between the two images despite several erroneous matches.
figure; showMatchedFeatures(I1,I2,matchedPoints1,matchedPoints2), title('Harris Features');
figure; showMatchedFeatures(I1,I2,matchedPoints3,matchedPoints4), title('MSER Features');
figure; showMatchedFeatures(I1,I2,matchedPoints1a,matchedPoints2a); title('BRISK Features');
figure; showMatchedFeatures(I1,I2,matchedPoints3a,matchedPoints4a); title('SURF Features');

pts1 = matchedPoints1.Location;
pts2 = matchedPoints2.Location;
pts1a = matchedPoints1a.Location;
pts2a = matchedPoints2a.Location;

pts3 = matchedPoints3.Location;
pts4 = matchedPoints4.Location;
pts3a = matchedPoints3a.Location;
pts4a = matchedPoints4a.Location;

pts1 = [pts1 ;pts3];
pts2 = [pts2 ;pts4];
pts1a = [pts1a ;pts3a];
pts2a = [pts2a ;pts4a];

pts1 = [pts1 ;pts1a];
pts2 = [pts2 ;pts2a];


flow_x = pts1(:,1) - pts2(:,1);
flow_y = pts1(:,2) - pts2(:,2);
clusters = 2;
mag  = flow_x.^2 + flow_y.^2;
flow = [flow_x flow_y pts1(:,1) pts1(:,2)];
[idx,Centroid] = kmeans(flow,clusters);




features = [mag pts1(:,1) pts1(:,2)];


%% Model fitting begins here



matchedPointsOriginal = pts1;
matchedPointsDistorted = pts2;


[tform,inlierPtsDistorted,inlierPtsOriginal] = estimateGeometricTransform(matchedPointsDistorted,....
    matchedPointsOriginal,'similarity','MaxDistance',70,'Confidence',99.9);
f1 = 254+ones(375,1242);
figure; showMatchedFeatures(f1,f1,inlierPtsOriginal,inlierPtsDistorted, 'PlotOptions',{'yo','b+','g-'});
title('Matched inlier points');

%% gg

flow_x = inlierPtsOriginal(:,1) - inlierPtsDistorted(:,1);
flow_y = inlierPtsOriginal(:,2) - inlierPtsDistorted(:,2);
clusters = 2;
mag  = flow_x.^2 + flow_y.^2;
mag = sqrt(mag);
orientation_angles_all = atand(flow_y./flow_x);




m1 = [inlierPtsOriginal inlierPtsDistorted mag flow_x flow_y];

sz = size(I1);
size_x = sz(1,2);
size_y = sz(1,1);


sample_size = 100;


iterations = 40;
errors = zeros(iterations,1);
num_params = 3;
model_params = zeros(iterations,num_params);

errors_orientation = zeros(iterations,1);
model_params_orientation = zeros(iterations,6);

%% running the RANSAC way to find optimal model for predicting flow give (x,y)
for i = 1:iterations
    data = datasample(m1,sample_size,'Replace',false);
    data(:,1) = data(:,1) - size_x/2;
    data(:,2) = data(:,2) - size_y/2;
    data(:,3) = data(:,3) - size_x/2;
    data(:,4) = data(:,4) - size_y/2;  
    X = [data(:,1) data(:,2)];
    m_y = [data(:,3) data(:,4)];
    y =  data(:,5);
    modelFun = @(b,x)b(1)+(b(2)*x(:,1))+(b(3)*x(:,2));
    beta0 = [0; 0; 0;]';
    mdl3 = NonLinearModel.fit(X,y,modelFun,beta0);
    coeff = mdl3.Coefficients.Estimate;
    
    
    %% finding ORIENTATION to fit a model to predict orientation of flow
    orientation_x = data(:,6);
    orientation_y = data(:,7);
    orientation_angles = atand(orientation_y./orientation_x);
    modelFun2 = @(c,x)c(1)+(c(2)*x(:,1).^2)+(c(3)*x(:,2).^2) + (c(4)*x(:,1).*x(:,2)) + (c(5)*x(:,1)) + (c(6)*x(:,2));
    beta1 = [0; 0; 0; 0; 0; 0]';
    X = [data(:,1) data(:,2)];
    mdl4 = NonLinearModel.fit(X,orientation_angles,modelFun2,beta1);
    coeff2 = mdl4.Coefficients.Estimate;
    
    X_all = inlierPtsOriginal;
    X_all(:,1) = X_all(:,1) - size_x/2;
    X_all(:,2) = X_all(:,2) - size_y/2;    
    
    predicted_all_orientation = coeff2(1,1) + (coeff2(2,1).*X_all(:,1).^2) + (coeff2(3,1).*X_all(:,2).^2) + (coeff2(4,1).*X_all(:,1).*X_all(:,2)) + (coeff2(5,1).*X_all(:,1)) + (coeff2(6,1).*X_all(:,2)); 
    error_orientation = abs(predicted_all_orientation - orientation_angles_all);
    errors_orientation(i,1) = sum(error_orientation);    
    model_params_orientation(i,:) = coeff2;
    
    %% fitting the MAGNITUDE of flow model
    
    
    predicted = coeff(1,1) + (coeff(2,1)*X(:,1)) + (coeff(3,1)*X(:,2));

    predicted = zeros(sample_size,1);

    for j=1:sample_size
        predicted(j,1) = coeff(1,1) + (coeff(2,1)*X(j,1)) + (coeff(3,1)*X(j,2));
    end   
    X_all = inlierPtsOriginal;
    X_all(:,1) = X_all(:,1) - size_x/2;
    X_all(:,2) = X_all(:,2) - size_y/2;    
    
    predicted_all = coeff(1,1) + (coeff(2,1).*X_all(:,1)) + (coeff(3,1).*X_all(:,2)); 
    error = abs(predicted_all - mag);
    errors(i,1) = sum(error);    
    model_params(i,:) = coeff;    
end

%% finding the optimal model for MAGNITUDE of flow over the iterations by RANSAC
[min1,idx] = min(errors);

final_model_params = model_params(idx,:);
final_model_params = final_model_params';

predicted_final = final_model_params(1,1) + (final_model_params(2,1).*X_all(:,1)) +...
    (final_model_params(3,1).*X_all(:,2)) ; 

error_final = abs(predicted_final - mag);
size_mag = numel(mag);
j=1;

for i=1:size_mag
    if(error_final(i,1) > threshold_residual)
        pos(j,1) = i;
        j = j+1;
    end
end

%% extracting segmented points and storing their coordinates in an array
segmented_pts = zeros(1,2);
j=j-1;
for i=1:j
    segmented_pts(i,1) = m1(pos(i),1);
    segmented_pts(i,2) = m1(pos(i),2);
end


figure, imshow(I1), hold on,
plot(segmented_pts(:,1),segmented_pts(:,2),'+','Color','red'), title('Magnitude based segmentation');




%% finidng the optimal model for the ORIENTATION of flow using iterations by RANSAC
[min2,idx2] = min(errors_orientation);

final_model_params2 = model_params_orientation(idx2,:);
final_model_params2 = final_model_params2';

predicted_final_orientation = final_model_params2(1,1) + (final_model_params2(2,1).*X_all(:,1).^2) +...
    (final_model_params2(3,1).*X_all(:,2).^2) + (final_model_params2(4,1).*X_all(:,1).*X_all(:,2)) +...
    (final_model_params2(5,1).*X_all(:,1)) + (final_model_params2(6,1).*X_all(:,2)); 

error_final_orientation = abs(predicted_final_orientation - orientation_angles_all);
size_mag = numel(orientation_angles_all);
j=1;
threshold_residual = 30;
for i=1:size_mag
    if(error_final_orientation(i,1) > threshold_residual)
        pos(j,1) = i;
        j = j+1;
    end
end
%% extracting segmented points and storing their coordinates in an array
segmented_pts = zeros(1,2);
j=j-1;
for i=1:j
    segmented_pts(i,1) = m1(pos(i),1);
    segmented_pts(i,2) = m1(pos(i),2);
end


figure, imshow(I1), hold on,
plot(segmented_pts(:,1),segmented_pts(:,2),'+','Color','yellow'), title('Orientation based segmentation');

%% playing around a bit - running the RANSAC way to find optimal model for predicting orientations of flow given (x,y)









%% restting to analyse from variable data
data(:,1) = data(:,1) + size_x/2;
data(:,2) = data(:,2) + size_y/2;
data(:,3) = data(:,3) + size_x/2;
data(:,4) = data(:,4) + size_y/2;

X = [data(:,1) data(:,2)];
m_y = [data(:,3) data(:,4)];
y =  data(:,5);

%% showing the results
