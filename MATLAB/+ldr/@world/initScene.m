function h_fig = initScene(this,dispVehMode,xyz_lims)
%   PUBLIC METHOD of CLASS WORLD
% initializes the entire world scene (the scene is initialized, 
% the figure handle is returned and all necessary graphics information 
% is stored in the figure's userdata.
%  xyz_lims is a 3 by 2 matrix with xlim, ylim and zlim rows

disp('just testing visualization');

% create figure
h_fig = figure('name','World Visualization','renderer','openGL');
% h_axes = axes('dataaspect',[1 1 1],'next','add',...
%               'xlimmode','auto','ylimmode','auto',...
%               'zlimmode','manual','zlim',[-2 2]);

h_axes = axes('dataaspect',[1 1 1],'next','add',...
              'xlimmode','manual','xlim',xyz_lims(1,:),...
              'ylimmode','manual','ylim',xyz_lims(2,:),...
              'zlimmode','manual','zlim',xyz_lims(3,:));
          
view(3);
box on;

if nargin < 2
    dispVehMode = 'bounding-box';
end
h_hg=updateVehicle(this,this.vehicle.tForm,h_fig,dispVehMode);
% h_hg = hgtransform('Parent',h_axes);
% set(h_vhcl,'Parent',h_hg)

% visualize all targets
n_trgts = numel(this.targets); % number of targets
h_trgts = [];
for n=1:n_trgts
    h_trgts = [h_trgts,this.targets(n).plot(h_axes)];
end

% store all graphic handles in figure's userdata
set(h_fig,'userdata',struct('handles',[h_hg,h_trgts])) %,'vhcl_vertices',h_vhcl.Vertices));
hold on