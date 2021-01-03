function changeCoords(this,tForm)
%   PUBLIC METHOD of CLASS TARGET in the ldr namespace
% changes the coordinates in which the instantiated target object is
% described in, by overwriting the parameters data-member.
% If you want to keep the original object (which is usually defined in
% world coordinates), make a copy and then change the copy.
%
% Example:
% myTarget=ldr.target('polygon',[-1 2 -1;2 0 -1;0 2 1]');
% myTargetCopy=myTarget.copy; % =copy(myTarget)
% myTargetCopy.changeCoords(eye(4,4)); % this is an identity transformation

% A better style of programming compared to having this switch-case
% statements, would be to have an abstract base class 'Target' with a pure
% virtual 'changeCoords' method, and then have derived classes 'sphere' and
% 'polygon' define changeCoords concretely (and separately). This might be
% implemented in a future version.

tForm_tr=tForm'; % transposed affine transformation
switch this.primitive
    case 'sphere'
        origin=this.parameters(1:3);
        radius=this.parameters(4);
        origin_new=[origin(:)',1]*tForm_tr(:,1:3); % origin in new coordinates
        this.parameters=[origin_new,radius];
    case 'polygon'
        edges=this.parameters'; % each row is XYZ of edges
        edges_new=[edges,ones(size(edges,1),1)]*tForm_tr(:,1:3); % edges in new coordinates
        this.parameters=edges_new'; % overwrite new edges in the parameters data member
    otherwise
        error('unsupported geometric primitive.');
end