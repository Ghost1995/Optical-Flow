%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This code runs all functions of homework 1
% 
% Submitted by: Ashwin Goyal (UID - 115526297)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Estimate optical flow using user-defined Lucas-Kanade method
estimateOpticalFlow('','..\input\eval-data-gray\Grove\')
estimateOpticalFlow('','..\input\eval-data-gray\Wooden\')

% Estimate optical flow using MATLAB built-in Lucas-Kanade method
estimateOpticalFlow('Lucas-Kanade','..\input\eval-data-gray\Grove\')
estimateOpticalFlow('Lucas-Kanade','..\input\eval-data-gray\Wooden\')

% Estimate optical flow using MATLAB built-in Farneback method
estimateOpticalFlow('Farneback','..\input\eval-data-gray\Grove\')
estimateOpticalFlow('Farneback','..\input\eval-data-gray\Wooden\')

% Estimate optical flow using MATLAB built-in Horn-Schunck method
estimateOpticalFlow('Horn-Schunck','..\input\eval-data-gray\Grove\')
estimateOpticalFlow('Horn-Schunck','..\input\eval-data-gray\Wooden\')