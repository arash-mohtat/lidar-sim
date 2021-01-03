function xyzImage = ptcldCols2Image(xyzCols,lidarTag)
% function for reordering a matrix with xyz columns from a pointcloud frame 
% ordered based on time-stamp into an nLaser-by-w image, e.g. 16-by-w 
%(w is the width of frame in pixels), image with three channels, 
% i.e. (:,:,i) with i=1,2,3 associated with x,y,z respectively.
%
% This works with the assumption that the firing order is 0:15 for
% channelIDs, the column starts from channel 0 fired and no firing is
% missed (fires can be missed provided that a NaN is put there).

switch lidarTag
    case 'VLP16' % this info can come from a lidar class object but we intentionally wanna use only the lidar tag and keep it independent from the lidar class
        [~,~,ID_vertAngl_vertCorr]=ldr.VLP16parameters();
        [~,reorderedFiringCycle]=sort(ID_vertAngl_vertCorr(:,2),'descend');
        nChannel=16;
    otherwise
        error('LiDAR tag not supported.');
end

width=floor(length(xyzCols)/nChannel);
reorderedIndices=repmat(reorderedFiringCycle,width,1)...
                 +reshape(repmat(0:nChannel:nChannel*(width-1),nChannel,1),[],1);

xyzImage=NaN*ones(nChannel,width,3);             
for i=1:3
    croppedColumn=xyzCols(1:nChannel*width,i);
    xyzImage(:,end:-1:1,i)=reshape(croppedColumn(reorderedIndices),nChannel,width); % end:-1:1 in the 2nd dimension assumes a CCW lidar rotation around the upward z-axis
end

