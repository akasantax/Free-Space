%% Find MSER Regions in an Image

function P = findFeaturePoints2(I)
%% Read image and detect MSER regions.
%     I = imread('/home/sahdev/Desktop/Spring 2016/Autonomous Driving_UofT/Project/KITTI/dataset2/right/data/0000000004.png');
     I2 = imread('/home/sahdev/Desktop/Spring 2016/Autonomous Driving_UofT/Project/KITTI/dataset2/right/data/0000000005.png');
    
    regions = detectMSERFeatures(I);
    regions2 = detectMSERFeatures(I2);
 
%% Visualize MSER regions which are described by pixel lists stored inside the returned 'regions' object.
%     figure; imshow(I); hold on;
%     plot(regions, 'showPixelList', true, 'showEllipses', false);
 
%% Display ellipses and centroids fit into the regions.
    figure; imshow(I); hold on;
    plot(regions); % by default, plot displays ellipses and centroids
 
%% getting points   
    
    threshol_num_pixels = 60;
    len = size(regions);
    len2 = size(regions2);
    
    num_pts = zeros(1,1);
    P = zeros(1,2);
    cnt = 0;
    for i=1:len(1,1)
        points_list = regions.PixelList(i,1);
        
        size2 = numel(points_list)/2;
        if(size2 < threshol_num_pixels)            
            continue;
        end
        
        xs = sum(points_list(:,1));
        ys = sum(points_list(:,2));

        x = xs/size2;
        y = ys/size2;
        if (x>1210 || x<50 || y<50 || y>350)
            continue;
            
        end
        
        cnt = cnt +1;
        num_pts(cnt,1) = size2;
        
        P(cnt,1) = floor(x);
        P(cnt,2) = floor(y);
        
    end
    
    
    
%     figure, imshow(I), hold on,
%     plot(P(:,1),P(:,2),'+','Color','red')
    
% end
    
