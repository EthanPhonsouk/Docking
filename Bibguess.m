%% Author: Faris Stitou

function [Bi, bguess] = Bibguess(p, Rs, Ri)
    %vector 3.1
    %Xs Ys Zs are the unknowns that MUST be solved for
    Rs = Rs(:);
    %disp(Rs);
    Xs = Rs(1); Ys = Rs(2); Zs = Rs(3);

    %vector 3.2
    %Xi Yi Zi are known but not just yet
    Ri = Ri(:);
    %disp(Ri);
    Xi = Ri(1); Yi = Ri(2); Zi = Ri(3);
    
    C = DCM(p);
    %disp(C);
    %-------------------------Now we math-------------------------%

    %Initialize Bi (3.4) and bguess (3.8 / 3.12)
    Bi_denom = ((Xi - Xs)^2) + ((Yi - Ys)^2) + ((Zi - Zs)^2);
    Bi_vector = [Xi - Xs; Yi - Ys; Zi - Zs];
    Bi = (1/(sqrt(Bi_denom))) * Bi_vector;
    bguess = C * Bi;
    %disp(bguess)
    %disp(Bi)
end

    

    