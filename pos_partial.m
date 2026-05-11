%% Author: Faris Stitou

%Create the partial derivative matrix seen in 3.21a
%Change inputs of Ri and Rs to just pos_partial(..., dist) so that I dont
%have to keep calling Ri and Rs
function hiRs = pos_partial(Bi, C, Rs, Ri)

    %Collect all necessary values to create the matrix
    I = eye(3);

    Rs = Rs(:);
    Xs = Rs(1); Ys = Rs(2); Zs = Rs(3);

    Ri = Ri(:);
    Xi = Ri(1); Yi = Ri(2); Zi = Ri(3);
    
    %Create the partial derivative matrix seen in 3.21a
    denom = sqrt((Xi - Xs)^2 + (Yi - Ys)^2 + (Zi - Zs)^2);
    hiRs = (-C*(I - Bi*Bi')) / denom;
end