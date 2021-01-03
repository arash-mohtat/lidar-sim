function flag=areInPolygon(XYZ,A)
% Efficient vectorized method to determine if multiple points XYZ are contained in 
% a polygon defined by matrix A.
% This function should ideally become a method of class polygon<target
%
% A. Mohtat, Mid 2019

numOfPoints=size(XYZ,1);
%flag=true(numOfPoints,1);
[valid,n_unit,d]=ldr.isValidPolygon(A);
if ~valid
    flag=false(numOfPoints,1);
else
    % check if XYZ points are in the same plane as the polygon
    flag=(abs(XYZ*n_unit-d)<1e-12);
end

P=eye(3,3)-n_unit*n_unit';
T23=orth(P); % 2-nd and 3rd columns of the xyz to d-alpha-beta transformation matrix
AB_Q=XYZ(flag,:)*T23; % alpha-beta coords of XYZ (only points on plane)
AB_A=A'*T23; % alpha-beta coords of vertices
AB_C=mean(AB_A); % alpha-beta coords of center of area of polygon

res_Q=lineEq(AB_Q(:,1),AB_Q(:,2),AB_A);
res_C=repmat(lineEq(AB_C(1),AB_C(2),AB_A),size(res_Q,1),1);

flag(flag)=all(res_Q.*res_C>0,2); % checks if each row has all trues (meaning each point Q is on the correct side of all polygon edges)


function res=lineEq(a,b,AB_A)

N=size(a,1); % must=size(b,1); number of points to be checked against the polygon
n=size(AB_A,1); % number of polygon's vertices
a=repmat(a,1,n);
b=repmat(b,1,n);
b_Ai=repmat(AB_A(:,2)',N,1);
b_Aj=b_Ai(:,[2:end,1]);

a_Ai=repmat(AB_A(:,1)',N,1);
a_Aj=a_Ai(:,[2:end,1]);

slopes=(b_Aj-b_Ai)./(a_Aj-a_Ai);
res=b-b_Ai-slopes.*(a-a_Ai);  % even if not repmatted, b and a will be replicated horizontally n times automatically in this expression (but we prefer to do it explicitly, as it would be difficult to accomplish the next 2 lines of code otherwise)
idx_inf=~isfinite(slopes);
res(idx_inf)=a(idx_inf)-a_Ai(idx_inf); % a is replicated horizontally n times automatically

