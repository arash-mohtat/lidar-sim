function outputs=capturePtClds(this,dwnSmplFctr,timeVector,h_fig)
%   PUBLIC METHOD of CLASS WORLD
% scans the targets by all lidars and returns one frame sweep per target per lidar.

disp('Ongoing work');
outputs=[];

% deal with missing inputs (default values)
if nargin<4
    h_fig =gcf;
end
if nargin<=2
    timeVector = 0;
end
if nargin == 1
    dwnSmplFctr = 1;
end

counter = 0;
h_line = [];
% NOTE: 16 is hard-coded for now (things will get a bit tricky if all
% lidars dont have the same number of channels)!
for n=1:length(this.lidars)*length(this.targets)*16
    h_line = [h_line,plot3(0,0,0,'b.-','parent',h_fig.CurrentAxes)];
end
h_scans = reshape(h_line,16,length(this.lidars),length(this.targets));


for t=timeVector
    for ldr_idx=1:length(this.lidars)
        tForm_vhcl = this.motions.vehicle(t);
        this.lidars(ldr_idx).vehicle.tForm = tForm_vhcl; % update the vehicle tForm in each lidar (this is stupid the lidar class should fetch the info from the world object!)
        updateVehicle(this,tForm_vhcl,h_fig);
        for trgt_idx=1:length(this.targets)
            counter = counter+1;
            % method scanTarget should find the subset of the respective
            % lidar's fov that sees the respective target, or it should receive
            % it as an extra input (the latter actually makes more sense in 
            % case the world class wants to implement some obstruction 
            % calculation type of thing!)
            [pcapTable,xyzImage_ldr,xyzImage_wrld,RIFimage]=this.lidars(ldr_idx).scanTarget(this.targets(trgt_idx),dwnSmplFctr); 
            %plot3(xyzImage_wrld.registered(:,:,1)',xyzImage_wrld.registered(:,:,2)',xyzImage_wrld.registered(:,:,3)','b.-');
            
            for channel=1:16
                set(h_scans(channel,ldr_idx,trgt_idx),'xdata',xyzImage_wrld.registered(channel,:,1),...
                    'ydata',xyzImage_wrld.registered(channel,:,2),...
                    'zdata',xyzImage_wrld.registered(channel,:,3))
            end    
            
            outputs.images.xyz_ldr{counter} = xyzImage_ldr;
            outputs.images.xyz_wrld{counter} = xyzImage_wrld;
            outputs.images.RIF{counter} = RIFimage; 
            outputs.pcapTable{counter} = pcapTable;
        end
    end
    drawnow;
end