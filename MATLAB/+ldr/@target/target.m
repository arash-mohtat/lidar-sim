classdef target < matlab.mixin.SetGet & matlab.mixin.Copyable
    % TARGET CLASS
    %   target object with encapsulated data and methods
    % ideally target should be a super class (Abstract Base Class) and
    % different primitives should inherit from it and define their specific
    % implementations of plot, castRays, etc. But, in this version, target
    % is a concrete class with switch-case implementation for 'spehere' and
    % 'polygon' primitives
    %
    % A. Mohtat, Mid 2019
    
    properties (Access = public)
        primitive=''; % shape primitive
        parameters=[]; % array containing target primitive parameters     
        reflectivity=[]; % material reflectivity
    end
    
    properties (SetAccess = private)
        reserved=[];
    end
    
    methods
        function this = target(prmtv,par,rflct)
            % Ctor: Construct an instance of this class
            %   Expects up to 3 inputs: vehicle, targets and lidars
            
            if nargin==0
                error('Target needs to be constructed based on a primitive shape');
            elseif nargin==1
                set(this,'primitive',prmtv);
            elseif nargin==2
                set(this,'primitive',prmtv,'parameters',par);
            elseif nargin==3
                set(this,'primitive',prmtv,'parameters',par,'reflectivity',rflct);
            end
            % for too many inputs an error will be automatically generated
        end
        
        %% Set methods for public data members
        function set.primitive(this,val)
            if ~strcmpi(val,'sphere') && ~strcmpi(val,'polygon') && ~strcmpi(val,'plane')
                error('Primitive shape not supported.');
            end
            % add other tests if necessay
            this.primitive=val;
        end
        
        function set.parameters(this,val)
            % add tests for parameters
            this.parameters=val;
        end
        
        function set.reflectivity(this,val)
            % add tests for reflectivity
            this.reflectivity=val;
        end
        
        
        %% Other (public) methods
        
        % function signature for public method 'plot' defined in a separate M-File in the @target folder. 
        h_target = plot(this,h_axes);
        
        % function signature for public method 'changeCoords' defined in a separate M-File in the @target folder. 
        changeCoords(this,tFrom); % target is a handle class, so this method changes the original instantiated object 
        
        % function signature for public method 'castRaysUpon' defined in a separate M-File in the @target folder. 
        [xyz,rho]=castRaysUpon(this,rays); 
    end
end

