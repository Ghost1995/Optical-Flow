%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This code runs optical flow estimation using different methods, both
% built-in MATLAB functions and the user defined function
% 
% Input:
%   opticalFlowType --> Defined the optical flow function to be used
%         imgFolder --> Location of the images
% 
% Submitted by: Ashwin Goyal (UID - 115526297)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function estimateOpticalFlow(opticalFlowType, imgFolder)

    % Read images
    imgFiles = dir([imgFolder '*.png']);
    
    % Define optical flow method
    if strcmpi(opticalFlowType,'Lucas-Kanade') || strcmpi(opticalFlowType,'LK')
        opticFlow = opticalFlowLK;
        opticalFlowType = 'LK';
    elseif strcmpi(opticalFlowType,'Farneback')
        opticFlow = opticalFlowFarneback;
    elseif strcmpi(opticalFlowType,'Horn-Schunck') || strcmpi(opticalFlowType,'HS')
        opticFlow = opticalFlowHS;
        opticalFlowType = 'HS';
    else
        estOpticalFlow_LK(imgFolder,[4 4],6,0.0039);
        return;
    end
    
    % Find image set
    index = find(imgFolder == '\' | imgFolder == '/',2,'last');
    imgSet = imgFolder(index(1)+1:index(2)-1);
    
    % Create video object
    vidObj = VideoWriter(['..\output\' imgSet '_'  opticalFlowType '_MATLAB.mp4'],'MPEG-4');
    open(vidObj)
    figure('units','normalized','outerposition',[0 0 1 1])
    for i = 1:length(imgFiles)
        % Read the image frame
        I = imread([imgFolder imgFiles(i).name]);
        flow = estimateFlow(opticFlow,I);
        imshow(I)
        hold on
        if strcmpi(opticalFlowType,'LK')
            plot(flow,'DecimationFactor',[4 4],'Scale',6)
        elseif strcmpi(opticalFlowType,'Farneback')
            plot(flow,'DecimationFactor',[8 8])
        elseif strcmpi(opticalFlowType,'HS')
            plot(flow,'DecimationFactor',[4 4],'Scale',50)
        end
        hold off
        saveas(gca,['..\output\' imgSet '_' opticalFlowType '_MATLAB_'  imgFiles(i).name(1:end-3) 'jpg'])
        for j = 1:30
            writeVideo(vidObj,getframe(gca))
        end
    end
    close(vidObj)

end