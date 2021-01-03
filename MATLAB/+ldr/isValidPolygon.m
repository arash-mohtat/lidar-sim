function [valid,n,d]=isValidPolygon(A)
% derived from depricated isValidPolygon.m and packaged in the ldr namespace
% this function should become a method of class polygon<target
%
% checks if 3-by-N matrix A defines a valid polygon and
% returns validity flag (true or false) and the unit normal vector 'n'.
% All columns of A, i.e. the polygon vertices, must lie in a
% plane (a warning is issued otherwise, and flag is set to false); no 3 of
% them should be collinear (especially the 1st three used to define the plane);
% and, they should define a convex polygon with vertices ordered sequentially (CW or CCW).

valid=false;
n=[NaN;NaN;NaN];

N=size(A,2); % number of vertices
if size(A,1)~=3 || N<3
    warning('not enough rows or columns in matrix A');
else
    n=cross(A(:,2)-A(:,1),A(:,3)-A(:,1));
    if norm(n)<1e-12
        warning('The first 3 points of A are almost collinear.');
    else
        n=n/norm(n);
        valid=true;
    end
end

% check if all points lie in the same plane (for more than 3 points)
d=dot(A(:,1),n); % perpendicular distance of coords origin to plane
if valid && N>3
    
%     for i=4:N
%         if abs(dot(A(:,i),n)-d)>1e-12
%             warning(['Vertix #',num2str(i),' is out of the plane defined by first 3 vertices.']);
%             valid=false;
%             break; % no need to check remaining vertices
%         end
%     end
    if any(abs(n'*A(:,4:N)-d)>1e-12)
        warning('At least one vertix is out of the plane defined by the first 3 vertices.');
        valid=false;
    end
end