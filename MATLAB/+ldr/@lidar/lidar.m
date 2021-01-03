classdef lidar < matlab.mixin.SetGet
    % LIDAR CLASS
    %   A rotary lidar with encapsulated data and methods.
    % A. Mohtat, Mid 2019
    
    properties (Access = public)
        tag='VLP16'; % tag
        fov=[0,360]; % azimuth field of view range
        vehicle=struct('ID',[],'tForm',eye(4,4)); % vehicle object to which lidar is attached to
        T_ldr2vhcl=eye(4,4); % 4-by-4 Affine Transformation Matrix for the lidar w.r.t. vehicle coord frame       
        dt; % length between firing events in sec
        intrinsicCals=struct('registered',[],'actual',[]); % intrinsic calibration (registered vs. actual fields, each to contain an nChannel-by-4 matrix of [laserID,EL,vert,AZ])
        rotation=struct('RPM',300,'other',[]); % rotational information ('RPM' field shall be a constant value or a function handle omega(AZ); 'other' field reserved for future)
        laser=struct('type',"ray",'rendering',"ray casting",'parameters',[]); % structure cointaing laser beam information (dimensions, dissipation properties, etc) or laser object encapsulating that and also methods for casting, tracing, etc
        noise=struct('range',[],'intensity',[]); % noises affecting registered range and intesity measurements
    end
    
    properties (SetAccess = private) % better to use (Dependent) possibly
        extrinsicTform=struct('registered',eye(4,4),'actual',eye(4,4)); % extrinsic ldr2wrld transformation (registered vs. actual fields, each to contain a 4-by-4 matrix)
        nChannel=16; % number of vertical channels
        firingSequenceCycle; % firing order of laser IDs (one full cycle)
        ID_vertAngl_vertCorr; % nChannel-by-3 matrix with [laserID,EL_Angle,vertPos] columns (nominal)
    end
    
    methods
        function this = lidar(tag)
            % Ctor: Construct an instance of this class
            %   inputs: expects one single mandatory input 'tag'
            % use set('property1','value1','property2','value2',...) to
            % modify other fields
            if nargin~=1
                error('Only a tag argument supported for construction.');
            elseif ~strcmpi(tag,'VLP16')
                error('Only VLP16 supported for now.');
            else
                nChannel=16; %=this.nChannel
                [this.firingSequenceCycle,this.dt,this.ID_vertAngl_vertCorr]=ldr.VLP16parameters();
                temp=zeros(nChannel,6);
                temp(:,6) = 1;
                temp(:,1)=0:nChannel-1;
                table_temp = array2table(temp,'VariableNames',{'channelID','dEL','dVer','dAZ','range_add','range_mult'});
                this.intrinsicCals.registered=table_temp;
                this.intrinsicCals.actual=table_temp;
                this.noise.range=struct('type',"Gaussian",'parameters',[zeros(nChannel,1),zeros(nChannel,nChannel)]); % parameters: [mu,sigma]
                this.noise.intensity=struct('type',"none",'parameters',[]);
                this.extrinsicTform=this.T_ldr2vhcl*this.vehicle.tForm;
            end
        end
        
        %% Set methods for (some) public data members
        function set.tag(this,val) % no need to return this (as our class is a handle class)
            if ~strcmpi(val,'VLP16')
                error('Only VLP16 supported for now.');
            else
                this.tag=val;
                this.nChannel=16;
                [this.firingSequenceCycle,this.dt,this.ID_vertAngl_vertCorr]=ldr.VLP16parameters();
            end
        end
        
        function set.vehicle(this,val)
            if ~isfield(val,'tForm')
                error('No tForm field in vehicle object');
            elseif ~isnumeric(val.tForm) || ~all(size(val.tForm)==[4 4])
                error('Invalid tForm field in vehicle object');
            else
                this.vehicle=val; % vhcl2wrld
                T_ldr2wrld=val.tForm*this.T_ldr2vhcl; % this might create order dependency if multiple properties set simultaneously (use a checker function at the end of all set functions)
                this.extrinsicTform=struct('registered',T_ldr2wrld,'actual',T_ldr2wrld);
            end
        end
        
        function set.T_ldr2vhcl(this,val)
            if ~isnumeric(val) || ~all(size(val)==[4 4])
                error('Invalid ldr2vhcl affine transformation matrix.');
            else
                this.T_ldr2vhcl=val;
                T_ldr2wrld=val*this.vehicle.tForm;
                this.extrinsicTform=struct('registered',T_ldr2wrld,'actual',T_ldr2wrld);
            end
        end
        
        %% Other (public) methods
        
        function this=injectFault(this,faultType,parameters) % probably no need to output this (handle class)
            disp('just testing fault injection');
        end
        
        % function signature for public method 'plot' defined in a separate M-File in the @lidar folder. 
        h_lidar = plot(this,h_axes);
        
        % function signature for public method 'scanTarget' defined in a separate M-File in the @lidar folder. 
        [pcapTable,ptCld_lidar,ptCld_world,RIFimage]=scanTarget(this,targetObj,dwnSmplFctr); 
    end
end

