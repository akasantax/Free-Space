PATH = '/home/sahdev/Desktop/Spring 2016/Autonomous Driving_UofT/datasets/';
folders = {'2011_09_26 (2)/2011_09_26_drive_0046_sync/image_00/data/',...
    '2011_09_26 (3)/2011_09_26_drive_0052_sync/image_00/data/',...
    '2011_09_28 (8)/2011_09_28_drive_0021_sync/image_00/data'};



% Parameters:
folder_index = 2;  % to access that particular foler for cloudy
folder_index2 = 2;

temp = [PATH folders{folder_index}];
imagefiles = dir([temp '/*.png']);
n = length(imagefiles);
 
labels = cell(n,1);
cnt = 1;
for  i=1:n;
    
    temp = [PATH folders{folder_index}];
    imagefiles = dir([temp '/*.png']);
    labels{cnt} = cellstr([temp '/' imagefiles(i).name]);
    cnt = cnt + 1;
     
end
threshold = 35;
I1 = imread(char(labels{2}));
I2 = imread(char(labels{3}));
 model_fitting_RANSAC_linear(I1,I2, threshold)
 
 
 for i=1:n-2
    I1 = imread(char(labels{i}));
    I2 = imread(char(labels{i+1}));
    model_fitting_RANSAC_linear(I1,I2, threshold)
 end
