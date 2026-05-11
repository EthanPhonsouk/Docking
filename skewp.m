%Skew symmetric matrix seen in function 3.7
% takes in vector p and forces it to be vertical then returns the pcross
% that is seen in the paper

function threepcross = skewp(p)
    p = p(:);
    threepcross = [0 -p(3)  p(2); p(3) 0 -p(1); -p(2) p(1) 0];
end
    
    
    
    