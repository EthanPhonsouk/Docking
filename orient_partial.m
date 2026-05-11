%% Author: Faris Stitou

%Create the partial derivative matrix seen in 3.21b
function hip = orient_partial(bguess, p)

    %Collect all necessary values to create the matrix
    I = eye(3);
    p = p(:);
    bcross = skewp(bguess);
    pcross = skewp(p);
    

    %Create the partial derivative matrix seen in 3.21b
    
    
    % hip = (4 / (1 + (p'*p))^2) * bcross * (1 - (p'*p))*I - 2*pcross + 2*(p*p');
    
    leftside = (4 / (1 + (p'*p))^2) * bcross;
    rightside = (1 - (p'*p))*I - 2*pcross + 2*((p*p'));
    hip = leftside * rightside;
end