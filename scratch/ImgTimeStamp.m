% This fuction finds the time stamp for a sequence of frames
% ImagePath : is the path of the folder mcontaining the frames
% ImageType : is the type of images (e.g. ImageType = 'bmp')
% Y = is vector containing the time stamps
function [Y] = ImgTimeStamp(ImagePath, ImageType)

if ImagePath(end)=='\'
    ImagePath = ImagePath(1:end-1);
end

imagefiles = dir([ImagePath,'\*.',ImageType]);
nfiles = length(imagefiles);    % Number of files found

for ii = 1:1:nfiles
    %extract just the name
    Name = imagefiles(ii).name;

    %find commas
    a = find(Name == ',');

    %grab the picnum
    strnum = Name(1:(a(1)-1));
    picnum = str2double(strnum);

    %grab seconds
    strsec = Name((a(1)+1):a(2)-1);
    strusec= Name(a(2)+1:end-4);

    Image_Timestamp(ii) = str2double(strsec) + str2double(strusec)*1e-7;
    
end
Y = Image_Timestamp; 
end