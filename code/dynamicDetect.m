tic

max_pixel_flow = 5; % this is w, this means actually a movement of 2*max_pixel_flow +1 for the object to be tracked;
patch_match_size = 3; % this is p, the size of the patch would be patch_match_size x patch_match_size
boundary_effect = 15;
threshold = 1800; % this is the threshold for the harris corner detector

srcFiles = dir('/home/sahdev/Desktop/Spring 2016/Autonomous Driving_UofT/Project/KITTI/dataset2/data_right/*.png');  % the folder in which ur images exists
n = length(srcFiles);
labels = cell(n,1);

parfor i = 1 : n
    filename = strcat('/home/sahdev/Desktop/Spring 2016/Autonomous Driving_UofT/Project/KITTI/dataset2/data_right/',srcFiles(i).name);
    labels{i} = cellstr(filename);
end
toc
I = imread(char(labels{1}));
sz = size(I);
sz_x = sz(1,1);
sz_y = sz(1,2);

boundary_effect = floor(sz_x/30);

frames = uint8(zeros(sz_x,sz_y,n));
parfor i=1:n
    frame_i = imread(char(labels{i}));
    t_s = size(frame_i,3);
    if(t_s == 3)
        frame_i = rgb2gray(frame_i);
    end
    frames(:,:,i) = frame_i;
    
end
toc

%%
strt2 = 10;
end2 = 14;
clusters = 2;
numDescriptors = 100;

Centroid = zeros(2,clusters);
temp_pts = zeros(1,2);
for i=strt2:end2
    temp1 = frames(:,:,i);
    temp2 = frames(:,:,i+1);    
    
    pts = findFeaturePoints2(temp1);
    pts2 = findFeaturePoints2(temp2);
    num_pts = numel(pts)/2;
    num_pts2 = numel(pts2)/2;
      
    [target_pts] = computeMapping(pts, pts2);
   
    
    flow_compute = target_pts - pts;
    
    %% logic for the actual corner updates
    matched_Points = target_pts;
    temp_pts = pts;
    
    %% cluster analysis begins
    
    mag = flow_compute(:,1).^2 + flow_compute(:,2).^2;
    
%     flow2 = [mag,flow_compute, matched_Points];
     flow2 = [mag,flow_compute];
     
     if(i == strt2)
        [idx,Centroid] = kmeans(flow2,clusters);
     end
     
     [idx2,Centroid2] = kmeans(flow2,clusters);
     temp_Centroid2 = Centroid2;
     temp_idx2 = idx2;
         
    cnt=1;bnt=1;ant=1;ent =1; fnt=1;gnt=1;hnt=1;
    
    Features1 = zeros(1,2);
    Features2 = zeros(1,2);
    Features3 = zeros(1,2);
    Features4 = zeros(1,2);
    Features5 = zeros(1,2);
    Features6 = zeros(1,2);
    
    cluster1=0;
    cluster2=0;
    for j=1:num_pts
        if(temp_idx2(j) == 1)
            Features1(cnt,:) = matched_Points(j,:);
            cnt = cnt+1;
            cluster1= cluster1 + (matched_Points(j,1)-temp_pts(j,1))^2 + (matched_Points(j,2)-temp_pts(j,2))^2;
        elseif(temp_idx2(j) == 2)
            Features2(bnt,:) = matched_Points(j,:);
            bnt = bnt+1;
            cluster2= cluster2 + (matched_Points(j,1)-temp_pts(j,1))^2 + (matched_Points(j,2)-temp_pts(j,2))^2;
        elseif(temp_idx2(j) == 3)
            Features3(ant,:) = matched_Points(j,:);
            ant = ant+1;
        end        
    end
    
    cluster1=cluster1/cnt;
    cluster2=cluster2/bnt;
    
    
    % reinitialize temp_pts for the next iteration
    temp_pts = matched_Points;
    
    %% plotting the convex hull
    
    bounds=zeros(1,2);
    bounds2=zeros(1,2);
    
    if (strt2 == i)
        if(cluster1 > cluster2)
            flag=1;
        else
            flag=0;
        end
    end
    
    
    if(flag == 1)
        if(size(Features1,1)>2)  
            k1 = convhull(Features1(:,1),Features1(:,2));
            bounds = zeros(numel(k1),2);
            for j2=1:numel(k1)
                bounds(j2,1) = Features1(k1(j2),1);
                bounds(j2,2) = Features1(k1(j2),2);
            end 
        end
    else
        if(size(Features2,1)>2)
             k2 = convhull(Features2(:,1),Features2(:,2));
            bounds = zeros(numel(k2),2);
            for j2=1:numel(k2)
                bounds2(j2,1) = Features2(k2(j2),1);
                bounds2(j2,2) = Features2(k2(j2),2);
            end 
        end

    end
    
    figure, imshow(temp1), hold on,
    plot(bounds(:,1),bounds(:,2),'Color','white'),    
    plot(bounds2(:,1),bounds2(:,2),'Color','cyan'),   
    
    plot(Features1(:,1),Features1(:,2),'+','Color','yellow'),
    plot(Features2(:,1),Features2(:,2),'+','Color','red'),
    
%     plot(actual_corners(:,1),actual_corners(:,2),'+','Color','cyan'),   
    
    plot(Features3(:,1),Features3(:,2),'+','Color','blue'),
    plot(Features4(:,1),Features4(:,2),'+','Color','green'),
    plot(Features5(:,1),Features5(:,2),'+','Color','cyan'),
    plot(Features6(:,1),Features6(:,2),'+','Color','white'),
    hold off;

end
toc

