%% Author: Faris Stitou

%Create the measurement sensitivity matrix seen in 3.19

%I think this is the main loop so I might not even need have p, Ri and Rs as
%inputs but rather I can hardcode them all in

%I hardcoded in Ri and I think I might aswell hardcode in p and Ri but I
%also added in the residual estimate error as seen in function 3.15

function [H, deltab] = MSM(p, Rs, bmeasure, Ri)

    %--------------------Code for building H and Delta_b------------------
    H = [];
    bguess_all = [];
    
    N = 6; %Number of beacons
    C = DCM(p);
    
    format long
    %create the measurement sensitivity matrix, H, seen in 3.19
    for i = 1:N
        beacon_Ri = Ri(i, :)';
        [Bi, bguess] = Bibguess(p, Rs, beacon_Ri);

        hiRs = pos_partial(Bi, C, Rs, beacon_Ri);
        hip = orient_partial(bguess, p);
        
        H = [H; hiRs, hip];
        
        %3.15
        bguess_all = [bguess_all; bguess]; 
        %disp(H); %comment/uncomment for testing purposes
    end
    
    %Finish up 3.15
    deltab = bmeasure - bguess_all;
end