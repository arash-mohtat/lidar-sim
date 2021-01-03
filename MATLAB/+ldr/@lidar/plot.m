function h_lidar = plot(this,h_axes)
%   PUBLIC METHOD of CLASS LIDAR in the ldr namespace
% plots the lidar cylinder in axes h_axes and return the handle pointing to the graphic
% object

disp('Ongoing work');

% draw upright cylider
[X,Y,Z] = cylinder([1 1],100);
h_cyl = surf(X,Y,Z);