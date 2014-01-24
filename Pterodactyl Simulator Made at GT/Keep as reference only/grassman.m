function result = grassman(A,B);
Avec = [A(2),A(3),A(4)];
Bvec = [B(2),B(3),B(4)];

result(1) = A(1)*B(1) + dot(Avec,Bvec);
result(2:4) = A(1)*Bvec + B(1)*Avec + cross(Avec,Bvec);