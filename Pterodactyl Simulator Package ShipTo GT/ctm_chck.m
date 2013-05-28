% Function: [ctm]=ctm_chck(ctm_in)
%
% Description: This function checks if the Coordinate Transformation Matrix 
%              is orthonormal. If not, the function derives the "closest" 
%              orthonormal matrix using Gram-Schmidt Orthogonalization.
%
%              Gram-Schmidt Orthogonalization
%              In the Gram-Schmidt orthogonalization process, a set of unit
%              vectors are constructed, one at a time in the following way:
%              1. Start by selecting one of the vectors, say, x1. Normalize
%                 this vector by defining  
%                     e1=x1/norm(x1)
%              2. Select a second vector, x2, and subtract from this vector
%                 the part that is parallel to e1; i.e.,
%                     e2=(I-P1)*x2
%                 where I is the n x n identity matrix and P1=e1'*e1 is the 
%                 projection operator associated with the first unit vector.
%                 Thus , P1*x2 is the component of x2 that is parallel to e1.
%                 The vector e2 is then normalized;
%                     e2=e2/norm(e2)
%              3. For the third vector, x3, subtract out the components
%                 parallel to the unit vectors already obtained,
%                     e3=(I-P1-P2)*x3
%                 and once again normalize,
%                     e3=e3/norm(e3)
%                 If x3 is a linear combination of x1 and x2; i.e., x3 lies
%                 in the plane defined by x1 and x2, then this projection 
%                 operation will yield a null vector for e3, and this vector
%                 will not be included in the set of orthonormal basis vectors.
%              4. Continue the procedure until the entire set of vectors, x(i),
%                 has been exhausted. The number of units vectors obtained by
%                 implementing this process is equal to the dimensionality of
%                 the space containing the original vectors. 
%
% References:     Numerical Methods with MATLAB
%                 A Resource for Scientists and Engineers
%                 G.J. Borse, Lehigh University
%                 PWS Publishing Company
%                 20 Park Plaza, Boston, MA 02116
%                 ISBN 0-534-93822-1. pp.29-31.
%            
%                 Optimization Theory With Applications
%                 Donald A. Pierre
%                 Dover Publications, Inc., 
%                 31 East 2nd Street, Mineola, N.Y. 11501
%                 ISBN 0-486-65205-X, pp.557-558
%
% Input: [ctm] is a (3 x 3) matrix that specifies the coordinate 
%        transformation and "should" satisfy the following property:
%                ctm * ctm' = I(3 x 3) Identity Matrix

% Output: The output of the program is the Coordinate Transformation Matrix
%         after the orthonormality check and correction if required. If the
%         the vector components are linearly independent, the output is obtained
%         by the Gram-Schimdt process. If the input matrix is rank deficient, two 
%         different options are possible.  If the rank of the input matrix is 
%         two, only one of the vector components is linearly dependent. In this 
%         case, a new vector is formed to be mutually orthogonal to the two 
%         linearly independent vectors. This new set of vectors is then converted 
%         into an orthonormal matrix by the Gram-Schimdt Process.  If the input 
%         matrix has a rank of one, then all vectors are linearly dependent.  
%         For this case, this program sets the output matrix to the Identity 
%         Matrix.
%
% Last Modified:
%     07/31/98 - A. Mur-Dongil
%     02/15/98 - G. Chamitoff (Error Messages and Comment Revisions,
%                              Removal of BREAK commands)
%

function [ctm]=ctm_chck(ctm_in)

ctm_in = real(ctm_in);

x1=ctm_in(1,:)';
x2=ctm_in(2,:)';
x3=ctm_in(3,:)';

A=rank(ctm_in);
if(A==1),
    ctm=eye(3);
    disp('Message from CTM_CHECK - Rank 1 CTM set to Identity');
else
    if (A==2),
        if (real(norm(cross(x1,x2)))~=0),
            x3=cross(x1,x2);
        elseif (real(norm(cross(x1,x3)))~=0),
            x2=cross(x3,x1);
        elseif (real(norm(cross(x2,x3)))~=0),
            x1=cross(x2,x3);
        end;
    end;      

    % Normalize the first vector to unit length
    e1=x1/real(norm(x1));
    % Project out only the part of C(2,:) that is orthogonal to e1.
    % Then, normalize the second vector to unit length.
    e2=(eye(3)-e1*e1')*x2;
    e2=e2/real(norm(e2));
    % Project out the part of C(3:3,:) that is orthogonal to e1 and e2.
    % Then, normalize the third vector to unit length.
    e3=(eye(3)-e1*e1'-e2*e2')*x3;
    e3=e3/real(norm(e3));
    % Develop a matrix whose colums are orthonormal basis vectors
    ctm=[e1 e2 e3]';
end;

return;

      
       
       