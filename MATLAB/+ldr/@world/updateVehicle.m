function h_hg=updateVehicle(this,tForm,h_fig,displayMode)


if isempty(h_fig.UserData)
    if nargin <4
        displayMode = 'bounding-box';
    end
    switch displayMode
        case 'bounding-box'
            d = this.vehicle.dimensions;
            [x_vhcl,y_vhcl,z_vhcl] = meshgrid(d(1)/2*[-1 1],d(2)/2*[-1 1],d(3)*[0 1]); % d(3)/2*[-1 1]
            f = [1 2 4 3;2 6 8 4;4 3 7 8;1 5 7 3;1 2 6 5;5 6 8 7];
            v = [x_vhcl(:),y_vhcl(:),z_vhcl(:)];
        case 'STL'
            out = load('hatchbackData.mat');
            v = out.v;
            f = out.f;
            v = v(:,[2 1 3])/25;
            v(:,1)= mean(v(:,1)) - v(:,1);
            v(:,2)= v(:,2)-mean(v(:,2));
        otherwise
            error('Display mode not recognized')
    end     
    v_new = [v,ones(size(v,1),1)]*tForm(1:3,:)';
    h_vhcl=patch('Faces',f,'Vertices',v_new,'FaceColor','r');
    alpha('color');
    alphamap('decrease');
    
    h_axes = findobj(h_fig,'type','axes'); % slightly more robust than gca
    h_hg = hgtransform('Parent',h_axes);
    set(h_vhcl,'Parent',h_hg)
else
%     v = h_fig.UserData.vhcl_vertices;
%     v_new = [v,ones(size(v,1),1)]*tForm(1:3,:)';
%     h_vhcl = h_fig.UserData.handles(1);
%     set(h_vhcl,'Vertices',v_new)
    h_hg = h_fig.UserData.handles(1);
    set(h_hg,'Matrix',tForm)
end