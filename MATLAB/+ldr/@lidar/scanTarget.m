function [pcapTable,ptCld_ldr,ptCld_wrld,RIFimage]=scanTarget(this,target,dwnSmplFctr)
%   PUBLIC METHOD of CLASS LIDAR
% scans the target and returns one frame worth of information.
% 'this' is the pointer to the lidar handle object, while 'target' is an
% external input by the user defining the scan target (polygons and spheres
% supported at the moment).
% The scan frame starts from this.fov(1) and ends at this.fov(2) at the 
% speed defined by this.rotation.omega. All other necessary information
%(such as calibrations and faults) are retrived from 'this'.
%
% Example:
% myLidar=lidar('VLP16');
% myLidar.fov=[20 130];
% target=struct('shape','polygon','params',[-1 2 -1;2 0 -1;0 2 1]'); 
% [pcapTable,~,ptCld_wrld]=myLidar.scanTarget(target);

%% Initialization
%disp('Scanning Target ...');
if nargin==2
    dwnSmplFctr=1;
end

% retrieve lidar information
firingSequenceCycle=this.firingSequenceCycle;
dt=dwnSmplFctr*this.dt; % simulate at courser resolution in time (similar to RPM*dwnSmplFctr)
ID_vertAngl_vertCorr=this.ID_vertAngl_vertCorr;
regCals=table2array(this.intrinsicCals.registered);
actCals=table2array(this.intrinsicCals.actual);
RPM=this.rotation.RPM;
if isnumeric(RPM)
    omega=@(AZ)6*RPM; % factor 6 is to convert RPM to deg/s
elseif isa(RPM,'function_handle')
    omega=@(AZ)6*this.RPM(AZ); % factor 6 is to convert RPM to deg/s
else
    error('Unrecognized RPM setting for lidar.'); % this should have been caught by the set.rotation method
end

%% LiDAR Spin
% simulate lidar rotation (add physics of spinning, inertia, friction, etc
% here. For now, only angular speed healthy or faulty signature enabled)
AZ=this.fov(1);
while true
    AZ=[AZ;AZ(end)+omega(AZ(end))*dt];
    if AZ(end)>=this.fov(2)
        break;
    end
end
N_tot=length(AZ); % total number of clock counts (including laser recharging)

%% Laser Ray Physics

% stack laser rays
numOfCycles=ceil(N_tot/length(firingSequenceCycle));
firingSequence=repmat(firingSequenceCycle,1,numOfCycles);
firingSequence=firingSequence(1:N_tot); % crop the extra part
N=N_tot-sum(isnan(firingSequence)); % number of points with actual laser firing
rays_act=struct('AZ',zeros(N,1),'EL',zeros(N,1),'Zcorr',zeros(N,1)); % this is what is actually being emitted
rays_reg=struct('AZ',zeros(N,1),'EL',zeros(N,1),'Zcorr',zeros(N,1)); % this is what the lidar has registered (thinks it's emitting)
rangeCorr_act = struct('add',zeros(N,1),'mult',zeros(N,1));  % this describes the actual error introduced in range measurements when collecting time-of-flight data (actual target distances)
rangeCorr_reg = struct('add',zeros(N,1),'mult',zeros(N,1));  % this describes how the lidar's firmware believes it should compensate for those errors (ideally by applying the inverse)

counter=0; % counter for laser firing
timeStamp=zeros(N,1);
for index=1:N_tot
    t=(index-1)*dt;
    fireID=firingSequence(index);
    rowIndex=find(ID_vertAngl_vertCorr(:,1)==fireID);    
    if ~isempty(rowIndex)
        counter=counter+1;
        timeStamp(counter)=t;
        rays_act.AZ(counter)=AZ(index)+actCals(rowIndex,4); % we stack only azimuth values during actual firing (not when laser ID=NaN)
        rays_reg.AZ(counter)=AZ(index)+regCals(rowIndex,4);
        rays_act.EL(counter)=ID_vertAngl_vertCorr(rowIndex,2)+actCals(rowIndex,2);
        rays_reg.EL(counter)=ID_vertAngl_vertCorr(rowIndex,2)+regCals(rowIndex,2);
        rays_act.Zcorr(counter)=ID_vertAngl_vertCorr(rowIndex,3)+actCals(rowIndex,3);
        rays_reg.Zcorr(counter)=ID_vertAngl_vertCorr(rowIndex,3)+regCals(rowIndex,3);
        rangeCorr_act.add(counter) = actCals(rowIndex,5);
        rangeCorr_act.mult(counter) = actCals(rowIndex,6);
        rangeCorr_reg.add(counter) = regCals(rowIndex,5);
        rangeCorr_reg.mult(counter) = regCals(rowIndex,6);
    end
end

% Vectorized calculation of pointcloud points from rays hitting the target
% (ray casting) - should be upgraded to class-based modeling of laser which
% contain methods for modeling ray, beam or cone tracing that can compute
% light returns (reflection, refraction, etc) and intensity

%[xyz_act,xyz_reg,dist,int]=calcRayIntersect_vec_faulty(rays_act,target,rays_reg); % note: xyz is in lidar coords
%[xyz_act,range_act]=ldr.castRaysOnTarget(rays_act,target);
targetCopy=target.copy;
targetCopy.changeCoords(ldr.invAffine(this.extrinsicTform.actual)); % target needs to be described in lidar's coordinates
%targetCopy.changeCoords(this.extrinsicTform.actual); % target needs to be described in lidar's coordinates
[~,range_act]=targetCopy.castRaysUpon(rays_act);
intensity_act=NaN*ones(N,1); % not implemented yet

% add noise to range and intensity returns
range_meas=rangeCorr_act.mult.*(range_act+rangeCorr_act.add)+0;
xyz_act=ldr.spher2Cart(range_meas,rays_act.AZ,rays_act.EL,rays_act.Zcorr);
intensity_reg=intensity_act+0; % maybe we can model some noise from power electronics with a covariance that includes off-diagonal terms;
%and add another noise due to the environment (temp, humidity, etc) with say diagonal-only covariance
% later on such insight can be used to design health indicators, etc
% (reverse inferral)

%% Package Outputs

% construct registered xyz pointcloud
range_reg = rangeCorr_reg.mult.*(range_meas+rangeCorr_reg.add); % this is where the lidar's firmware tries to compensate for range measurement errors based on registered calibration values
xyz_reg=ldr.spher2Cart(range_reg,rays_reg.AZ,rays_reg.EL,rays_reg.Zcorr);

% time-stamped-ordered pcapTable
pcapTable=array2table([timeStamp,rays_reg.AZ,rays_reg.EL,range_reg,intensity_reg],...  % pcap table contains spherical coord measurements
      'VariableNames',{'TimeStamp','Azimuth_deg','Elevation_deg','Distance_m','Intensity'});

% 3D pointcloud data in different coordinates
regT_tr=(this.extrinsicTform.registered)'; % transposed registered transformation matrix ldr2wrld
actT_tr=(this.extrinsicTform.actual)'; % transposed actual transformation matrix ldr2wrld
XYZ_reg=horzcat(xyz_reg,ones(counter,1))*regT_tr(:,1:3);
XYZ_act=horzcat(xyz_act,ones(counter,1))*actT_tr(:,1:3);

% rearrange 3D ptCloud data based on laser channels (ptCloud Images)
ptCld_ldr=struct('registered',ldr.ptcldCols2Image(xyz_reg,this.tag),...
                 'actual',ldr.ptcldCols2Image(xyz_act,this.tag)); 
ptCld_wrld=struct('registered',ldr.ptcldCols2Image(XYZ_reg,this.tag),...
                  'actual',ldr.ptcldCols2Image(XYZ_act,this.tag)); 

% Range/Intensity/Fire-order Images
RIFimage=ldr.ptcldCols2Image(horzcat(range_reg,intensity_reg,(1:N)'),this.tag);

% Output confirmation
%disp('Target successfully scanned.');

