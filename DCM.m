%% Author: Faris Stitou
%DCM Function 3.6

function C = DCM(p)
    p = p(:);
    I = eye(3);
    pcross = skewp(p);
    num = 8 * (pcross * pcross) - 4 * (1 - p' * p) * pcross; 
    denom = (1 + p' * p) * (1 + p' * p); 
    C = I + num/denom;
end