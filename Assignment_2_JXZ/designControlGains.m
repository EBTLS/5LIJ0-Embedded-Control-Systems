function [K,F] = designControlGains(Ad,Bd,Cd,Q,R,type)

%% STUDENTS NEED TO WRITE CODE FOR THIS FUNCTION
%% BEGIN: SOLUTION CODE

if nargin<=3
    Q=10*(Cd')*Cd;
end

if nargin<=4
    R=3;
end

if nargin<=5
    type="LQR";
end

if (type=="LQR")
    %% LQR Controller Design
    % Design using dare
    [X,L,G] = dare(Ad, Bd, Q,R);
    K_controlled = -G;
    n=size(Cd,2);
    F = 1/(Cd*inv(eye(n)-(Ad+Bd*K_controlled))*Bd);
    K = K_controlled; 

elseif (type=="LQI")
    
end
%%END: SOLUTION CODE