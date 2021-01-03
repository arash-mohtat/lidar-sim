function XYZ=spher2Cart(rho,AZ,EL,Zcorr)

if nargin == 3
    Z_corr=0; % takes care of the single-origin spher2Cart transformation
end

ce=cosd(EL); % N-by-1 column vector
se=sind(EL); % N-by-1 column vector
ca=cosd(AZ); % N-by-1 column vector
sa=sind(AZ); % N-by-1 column vector
XYZ=[rho.*ce.*ca,rho.*ce.*sa,rho.*se+Zcorr];