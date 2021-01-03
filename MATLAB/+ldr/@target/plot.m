function h_target = plot(this,h_axes)
%   PUBLIC METHOD of CLASS TARGET in the ldr namespace
% plots the targets in axes h_axes and return the handle pointing to the graphic
% object

disp('Ongoing work');

switch this.primitive
    case 'sphere'
        [xx,yy,zz] = sphere;
        R=this.parameters(4);
        h_target = mesh(R*xx+this.parameters(1),R*yy+this.parameters(2),R*zz+this.parameters(3),'parent',h_axes);
    case 'polygon'
        vertices=this.parameters';
        h_target = patch('Faces',1:size(vertices,1),'Vertices',vertices,'FaceColor','y','FaceAlpha',0.3,'parent',h_axes);
    otherwise
        error('unsupported geometric primitive.');
end