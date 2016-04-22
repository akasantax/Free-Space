%% Find Corresponding Interest Points Between Pair of Images
% Read the stereo images.
I1 = rgb2gray(imread('/home/sahdev/Desktop/Spring 2016/Autonomous Driving_UofT/Project/KITTI/dataset2/data_left/0000000003.png'));
I2 = rgb2gray(imread('/home/sahdev/Desktop/Spring 2016/Autonomous Driving_UofT/Project/KITTI/dataset2/data_right/0000000003.png'));
%% Find the features.
points1 = detectHarrisFeatures(I1);
points2 = detectHarrisFeatures(I2);

points3 = detectMSERFeatures(I1);
points4 = detectMSERFeatures(I2);


%% Extract the neighborhood features.
[features1,valid_points1] = extractFeatures(I1,points1);
[features2,valid_points2] = extractFeatures(I2,points2);  

[features3,valid_points3] = extractFeatures(I1,points3);
[features4,valid_points4] = extractFeatures(I2,points4);  
%% Match the features.
indexPairs1 = matchFeatures(features1,features2);   
indexPairs2 = matchFeatures(features3,features4);   

%% Retrieve the locations of the corresponding points for each image.
matchedPoints1 = valid_points1(indexPairs1(:,1),:);
matchedPoints2 = valid_points2(indexPairs1(:,2),:); 
matchedPoints3 = valid_points3(indexPairs2(:,1),:);
matchedPoints4 = valid_points4(indexPairs2(:,2),:); 



%%  Visualize the corresponding points. You can see the effect of translation between the two images despite several erroneous matches.
figure; showMatchedFeatures(I1,I2,matchedPoints1,matchedPoints2);
figure; showMatchedFeatures(I1,I2,matchedPoints3,matchedPoints4);

pts1 = matchedPoints1.Location;
pts2 = matchedPoints2.Location;

pts3 = matchedPoints3.Location;
pts4 = matchedPoints4.Location;

pts1 = [pts1 ;pts3];
pts2 = [pts2 ;pts4];


flow_x = pts1(:,1) - pts2(:,1);
flow_y = pts1(:,2) - pts2(:,2);
clusters = 3;
mag  = flow_x.^2 + flow_y.^2;
flow = [flow_x flow_y];
[idx,Centroid] = kmeans(flow,clusters);

%% plotting the clusters

len = numel(pts1)/2;
cluster1 = zeros(1,2);
cluster2 = zeros(1,2);
cluster3 = zeros(1,2);

cnt = 1;
bnt = 1;
ant = 1;

for i=1:len
    if(idx(i,1) == 1)
        cluster1(cnt,:) = pts1(i,:);
        cnt = cnt + 1;
    elseif (idx(i,1) == 2)
        cluster2(bnt,:) = pts1(i,:);
        bnt = bnt + 1;
    elseif (idx(i,1) == 3)
        cluster3(ant,:) = pts1(i,:);
        ant = ant + 1;
    end
end



figure, imshow(I1), hold on,
plot(cluster1(:,1),cluster1(:,2),'+','Color','red'),    
plot(cluster2(:,1),cluster2(:,2),'+','Color','yellow');  
plot(cluster3(:,1),cluster3(:,2),'+','Color','cyan'); 

RGB = insertText(I1,pts1,floor(mag),'AnchorPoint','LeftBottom','FontSize',6);
figure, imshow(RGB);
hold off;


