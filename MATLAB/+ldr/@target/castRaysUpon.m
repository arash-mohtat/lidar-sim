function [xyz,rho]=castRaysUpon(this,rays)
% cast vector of rays on the target object and calculates vector of ranges
% (rho) and the corresponding xyz-pointcloud points in lidar coordinates.

N=length(rays.EL);
if (length(rays.AZ)~=N) || (length(rays.Zcorr)~=N)
    error('rays.EL, rays.AZ and ray.Zcorr must have same length for all laser beams!');
end

rho=NaN*ones(N,1);
int=NaN*ones(N,1);

ce=cosd(rays.EL); % N-by-1 column vector
se=sind(rays.EL); % N-by-1 column vector
ca=cosd(rays.AZ); % N-by-1 column vector
sa=sind(rays.AZ); % N-by-1 column vector

if strcmpi(this.primitive,'sphere')
    
    X0=this.parameters(1); % scalar
    Y0=this.parameters(2); % scalar
    Z0=this.parameters(3); % scalar
    R=this.parameters(4);  % scalar
   
    % solve "dist^2-2*b*dist+g=0" for min positive dist for all rays simultaneously
    b=X0*ce.*ca+Y0*ce.*sa+(Z0-rays.Zcorr).*se;  
    g=X0^2+Y0^2+(Z0-rays.Zcorr).^2-R^2;
    cond=(b>0 & (b.^2-g)>=0);
    rho(cond)=b(cond)-sqrt(b(cond).^2-g(cond));
    xyz=[rho.*ce.*ca,rho.*ce.*sa,rho.*se+rays.Zcorr]; % =spherical2Cart(rho,rays_act.AZ,rays_act.EL,rays_act.Zcorr)

elseif strcmpi(this.primitive,'polygon')
    
    A=this.parameters;
    [valid,n_unit,d]=ldr.isValidPolygon(A); % if target is defined as a class, this function will be called in its constructor
    if valid
        rho=(d-n_unit(3)*rays.Zcorr)./(n_unit(1)*ce.*ca+n_unit(2)*ce.*sa+n_unit(3)*se); % vector of rho (range) values that define all rays intersections with the polygon's plane
        rho(rho<0)=NaN; % excludes the intersection from backward extension of rays
    end
    xyz=[rho.*ce.*ca,rho.*ce.*sa,rho.*se+rays.Zcorr]; % =spherical2Cart(rho,rays_act.AZ,rays_act.EL,rays_act.Zcorr)

    cond=ldr.areInPolygon(xyz,A);
    rho(~cond)=NaN;
    xyz(~cond,:)=NaN;
    
elseif strcmpi(this.primitive,'plane') % very similar to the polygon's case (without checking the within-polygon condition)
    
    n_unit = this.parameters(1:3);
    d = this.parameters(4);        
    rho=(d-n_unit(3)*rays.Zcorr)./(n_unit(1)*ce.*ca+n_unit(2)*ce.*sa+n_unit(3)*se); % vector of rho (range) values that define all rays intersections with the polygon's plane
    rho(rho<0)=NaN; % excludes the intersection from backward extension of rays
    xyz=[rho.*ce.*ca,rho.*ce.*sa,rho.*se+rays.Zcorr]; % =spherical2Cart(rho,rays_act.AZ,rays_act.EL,rays_act.Zcorr)
   
else
    error('Unsupported shape primitive.');
end
