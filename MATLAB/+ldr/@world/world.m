classdef world < matlab.mixin.SetGet
    % WORLD CLASS
    %   world with encapsulated data (vehicles, targets, lidars) and methods (for visualization, etc).
    %
    % A. Mohtat, Mid 2019
    
    properties (Access = public)
        vehicle=[]; % vehicle object
        targets=[]; % array containing target objects     
        lidars=[]; % array containing lidar object handles
        motions=struct('vehicle',[],'targets',[]); % prescribed motion for vehicle and targets
    end
    
    properties (SetAccess = private)
        reserved=[];
    end
    
    methods
        function this = world(vhcl,trgts,ldrs)
            % Ctor: Construct an instance of this class
            %   Expects up to 3 inputs: vehicle, targets and lidars
            
            if nargin==0
                this.vehicle=struct('ID',1,'tForm',eye(4,4),'dimensions',[4 1.8 1.5]);
                this.vehicle.tForm(3,4)=1.5/2;
            elseif nargin==1
                set(this,'vehicle',vhcl);
            elseif nargin==2
                set(this,'vehicle',vhcl,'targets',trgts);
            elseif nargin==3
                set(this,'vehicle',vhcl,'targets',trgts,'lidars',ldrs);
            end
            % for too many inputs an error will be automatically generated
        end
        
        %% Set methods for public data members
        function set.vehicle(this,val)
            if ~isstruct(val)
                error('Vehicle needs to be a struct');
            elseif ~isfield(val,'tForm') || ~isfield(val,'dimensions')
                error('Vehicle struct needs at least the fields: tForm and dimensions');
            end
            % add other tests for tForm  and dimensions validity
            this.vehicle=val;
        end
        
        function set.targets(this,val) % no need to return this (as our class is a handle class)
            % add tests for targets
            this.targets=val;
        end
        
        function set.lidars(this,val) % no need to return this (as our class is a handle class)
            % add tests for lidars
            this.lidars=val;
        end
        
        
        %% Other (public) methods
        
        % function signature for public method 'visualize' defined in a separate M-File in the @world folder. 
        h_fig = initScene(this,dispVehMode,xyz_lims)
        
        % function signature for updating vehicle graphics (the vehicle
        % should be defined later as a special kind of target so it can enjoy
        % plotting facilities from a target!)
        h_vhcl=updateVehicle(this,tForm,h_fig,displayMode)
        
        % function signature for public method 'capturePtClds' defined in a separate M-File in the @world folder. 
        [outputs]=capturePtClds(this,dwnSmplFctr,timeVector,h_fig); 
    end
end

