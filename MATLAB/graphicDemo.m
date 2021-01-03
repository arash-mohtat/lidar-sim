% This M-script shows how to use the +ldr package.
% It shows a vehicle on a rotating platform with two downward-facing lidars
% on its rooftop scanning the ground plane a spherical target that can be moved with the mouse!
%
% Arash Mohtat, mid 2019

vhcl=struct('ID',1,'tForm',eye(4,4),'dimensions',[4 1.8 1.5]);
sphr=ldr.target('sphere',[1 3.5 1 1]);
grnd=ldr.target('polygon',6*[-1 -1 0;1 -1 0;1 1 0;-1 1 0]');


ldr1=ldr.lidar('VLP16');
s=1/sqrt(2);
set(ldr1,'vehicle',vhcl,'T_ldr2vhcl',[1 0 0 0;0 s s 0.5;0 -s s 1.6;0 0 0 1],...
    'fov',[-30 210]);  % [-30,90]


wrld=ldr.world(vhcl,[sphr,grnd],[ldr1]);
wrld.motions.vehicle = @(t)[cos(pi/8*t),-sin(pi/8*t),0,0; ...
                            sin(pi/8*t), cos(pi/8*t),0,0; ...
                            0 0 1 0;0 0 0 1];  % 0 0 1 0.75;0 0 0 1];
h_fig = wrld.initScene('STL',[-6 6;-6 6;-2 2]);
uic = uicontrol(h_fig,'Style','Push','String','Play','Callback',...
                      @(src,evnt)playCallback(src,evnt,wrld,h_fig))
                %'wrld.capturePtClds(5,0:0.2:32,h_fig);');
            
h_axes = findobj(h_fig,'type','axes'); % slightly more robust than gca
set(h_fig,'WindowButtonDownFcn',@(src,evnt)startMoving(src,evnt,h_fig,h_axes,sphr));


function playCallback(src,evnt,wrld,h_fig)
    h_lines = findall(h_fig,'type','line');
    delete(h_lines);
    wrld.capturePtClds(10,0:0.2:32,h_fig);
end

function startMoving(src,evnt,h_fig,h_axes,sphr)
    % Set callbacks
    set(h_fig,'WindowButtonMotionFcn',@(src,evnt)moveSphere(src,evnt,h_fig,h_axes,sphr));
    set(h_fig,'WindowButtonUpFcn',@(src,evnt)stopSphere(src,evnt,h_fig,h_axes));
end

function moveSphere(src,evnt,h_fig,h_axes,sphr)
    
    sat = @(x,L,U)x.*(x>L & x<U)+L.*(x<=L)+U.*(x>=U);

    % Do "smart" positioning of the object, relative to starting point...
    pos = get(h_axes,'CurrentPoint');
    trgt_pos = mean(pos);
    trgt_pos(3)=1;
    trgt_pos = sat(trgt_pos,[-6 -6 -2],[6 6 2]); 
    h_sphere = h_fig.UserData.handles(2);
    xx = get(h_sphere,'xdata');
    yy = get(h_sphere,'ydata');
    zz = get(h_sphere,'zdata');
%     xx = xx-mean(xx(:));
%     yy = yy-mean(yy(:));
%     zz = zz-mean(zz(:));
    set(h_sphere,'xdata',xx-mean(xx(:))+trgt_pos(1),...
        'ydata',yy-mean(yy(:))+trgt_pos(2),...
        'zdata',zz-mean(zz(:))+trgt_pos(3));
    sphr.parameters(1:3) = trgt_pos;
    %disp(pos)
end

    
function stopSphere(src,evnt,h_fig,h_axes)
    % Clean up the evidence ...
    set(h_fig,'WindowButtonUpFcn','');
    set(h_fig,'WindowButtonMotionFcn','');
end


