function iA=invAffine(A)

R=A(1:3,1:3)';
iA=[R,-R*A(1:3,4);0 0 0 1];