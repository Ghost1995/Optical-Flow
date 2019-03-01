%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This code runs optical flow estimation using user defined Lucas-Kanade
% Method
% 
% Input:
%     imgFolder --> Location of the images
%   vectorSpace --> Defines the spacing between optical flow vectors
%         scale --> Scaling of the optical flow vectors
%      treshold --> Eigen value threshold; prevents inv. of singular matrix
% 
% Submitted by: Ashwin Goyal (UID - 115526297)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function estOpticalFlow_LK(imgFolder, vectorSpace, scale, threshold)

    % Read images
    imgFiles = dir([imgFolder '*.png']);
    
    % Find image set
    index = find(imgFolder == '\' | imgFolder == '/',2,'last');
    imgSet = imgFolder(index(1)+1:index(2)-1);
    
    % Define kernels for gradient and smoothing
    gradKernel = [-1 8 0 -8 1]/12;
    gaussKernel = [1 4 6 4 1]/16;
    
    % Check inputs
    if isempty(vectorSpace)
        vectorSpace = [1 1];
    end
    if isempty(scale)
        scale = 1;
    end
    if isempty(threshold)
        threshold = 1e-4;
    end
    
    % Create video object
    vidObj = VideoWriter(['..\output\' imgSet '_LK.mp4'],'MPEG-4');
    open(vidObj)
    figure('units','normalized','outerposition',[0 0 1 1])
    for i = 1:length(imgFiles)
        % Read the image frame
        if i == 1
            Inew = im2double(imread([imgFolder imgFiles(i).name]));
            Iold = zeros(size(Inew));
        else
            Iold = Inew;
            Inew = im2double(imread([imgFolder imgFiles(i).name]));
        end
        I_padded = padarray(Inew,[2 2],'both');
        
        % Take gradient across columns
        Ix = zeros(size(I_padded,1),size(Inew,2));
        for j = 3:size(I_padded,2)-2
            Ix(:,j-2) = sum(I_padded(:,j-2:j+2).*gradKernel,2);
        end
        Ix = Ix(3:size(Ix,1)-2,:);
        
        % Take gradient across rows
        Iy = zeros(size(Inew,1),size(I_padded,2));
        for j = 3:size(I_padded,1)-2
            Iy(j-2,:) = sum(I_padded(j-2:j+2,:).*gradKernel');
        end
        Iy = Iy(:,3:size(Iy,2)-2);
        
        % Take gradient across time
        It = Inew - Iold;
        
        % Compute Ix*Ix, Iy*Iy, Ix*It, Iy*It, Ix*Iy
        Ix2 = padarray(Ix.*Ix,[2 2],'both');
        Iy2 = padarray(Iy.*Iy,[2 2],'both');
        IxIy = padarray(Ix.*Iy,[2 2],'both');
        IxIt = padarray(Ix.*It,[2 2],'both');
        IyIt = padarray(Iy.*It,[2 2],'both');
        
        % Apply gaussian filter column-wise
        for j = 3:size(I_padded,2)-2
            Ix2(:,j) = sum(Ix2(:,j-2:j+2).*gaussKernel,2);
            Iy2(:,j) = sum(Iy2(:,j-2:j+2).*gaussKernel,2);
            IxIy(:,j) = sum(IxIy(:,j-2:j+2).*gaussKernel,2);
            IxIt(:,j) = sum(IxIt(:,j-2:j+2).*gaussKernel,2);
            IyIt(:,j) = sum(IyIt(:,j-2:j+2).*gaussKernel,2);
        end
        Ix2 = Ix2(:,3:size(I_padded,2)-2);
        Iy2 = Iy2(:,3:size(I_padded,2)-2);
        IxIy = IxIy(:,3:size(I_padded,2)-2);
        IxIt = IxIt(:,3:size(I_padded,2)-2);
        IyIt = IyIt(:,3:size(I_padded,2)-2);
        
        % Apply gaussian filter row-wise
        for j = 3:size(I_padded,1)-2
            Ix2(j-2,:) = sum(Ix2(j-2:j+2,:).*gaussKernel');
            Iy2(j-2,:) = sum(Iy2(j-2:j+2,:).*gaussKernel');
            IxIy(j-2,:) = sum(IxIy(j-2:j+2,:).*gaussKernel');
            IxIt(j-2,:) = sum(IxIt(j-2:j+2,:).*gaussKernel');
            IyIt(j-2,:) = sum(IyIt(j-2:j+2,:).*gaussKernel');
        end
        Ix2 = Ix2(3:size(I_padded,1)-2,:);
        Iy2 = Iy2(3:size(I_padded,1)-2,:);
        IxIy = IxIy(3:size(I_padded,1)-2,:);
        IxIt = IxIt(3:size(I_padded,1)-2,:);
        IyIt = IyIt(3:size(I_padded,1)-2,:);
        
        % Solve the linear equations for velocity computation
        delta = Ix2.*Iy2 - IxIy.*IxIy;
        A = (Ix2 + Iy2)/2;
        B = sqrt(4*IxIy.*IxIy + (Ix2 - Iy2).*(Ix2 - Iy2))/2;
        eig1 = A + B;
        eig2 = A - B;
        Vx = zeros(size(Inew));
        Vy = zeros(size(Inew));
        for j = 1:size(Inew,1)
            for k = 1:size(Inew,2)
                if eig1(j,k) >= threshold
                    if eig2(j,k) >= threshold
                        Vx(j,k) = -(IyIt(j,k)*IxIy(j,k) - IxIt(j,k)*Iy2(j,k))/delta(j,k);
                        Vy(j,k) = -(IxIy(j,k)*IxIt(j,k) - Ix2(j,k)*IyIt(j,k))/delta(j,k);
                    else
                        Ixy_x2 = IxIy(j,k) + Ix2(j,k);
                        Ixy_y2 = Iy2(j,k) + IxIy(j,k);
                        norm = Ixy_x2*Ixy_x2 + Ixy_y2*Ixy_y2;
                        if norm >= 0
                            temp = (IyIt(j,k) + IxIt(j,k))/norm;
                            Vx(j,k) = Ixy_x2*temp;
                            Vy(j,k) = Ixy_y2*temp;
                        else
                            Vx(j,k) = 0;
                            Vy(j,k) = 0;
                        end
                    end
                else
                    Vx(j,k) = 0;
                    Vy(j,k) = 0;
                end
            end
        end
        
        % Plot the optical flow
        V_rows = 1:vectorSpace(2):size(Inew,1);
        V_cols = 1:vectorSpace(1):size(Inew,2);
        [X,Y] = meshgrid(V_cols,V_rows);
        Vx = Vx(V_rows,V_cols)*scale;
        Vy = Vy(V_rows,V_cols)*scale;
        
        % Show the image
        imshow(Inew)
        hold on
        quiver(X(:),Y(:),Vx(:),Vy(:),0);
        hold off
        
        % Save the image and video
        saveas(gca,['..\output\' imgSet '_LK_' imgFiles(i).name(1:end-3) 'jpg'])
        for j = 1:30
            writeVideo(vidObj,getframe(gca))
        end
    end
    close(vidObj)

end


