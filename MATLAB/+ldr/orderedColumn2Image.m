function imageData = orderedColumn2Image(column,lidarTag)
% function for reordering a column (range, intensity, x, y, z, etc) from a pointcloud frame 
% ordered based on time-stamp into an nChannel-by-w image (w is the width of frame in pixels)
% this works with the assumption that the firing order is 0:15 for
% channelIDs, the column starts from channel 0 fired and no firing is
% missed (fires can be missed provided that a NaN is put there).

switch lidarTag
    case 'VLP16' % this info can come from a lidar class object but we intentionally wanna use only the lidar tag and keep it independent from the lidar class
        [~,~,ID_vertAngl_vertCorr]=VLP16parameters();
        [~,reorderedFiringCycle]=sort(ID_vertAngl_vertCorr(:,2),'descend');
        nChannel=16;
    otherwise
        error('LiDAR tag not supported.');
end

width=floor(length(column)/nChannel);
reorderedIndices=repmat(reorderedFiringCycle,width,1)...
                 +reshape(repmat(0:nChannel:nChannel*(width-1),nChannel,1),[],1);

croppedColumn=column(1:nChannel*width);
imageData=reshape(croppedColumn(reorderedIndices),nChannel,width);

