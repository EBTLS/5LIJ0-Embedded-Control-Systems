function [K,F] = designControlGains(Ad,Bd,Cd,Q,R)

%% STUDENTS NEED TO WRITE CODE FOR THIS FUNCTION
%% BEGIN: SOLUTION CODE
n=length(Ad);

Q = 10*(C_controlled')*C_controlled;


R =10;
%% Design using dare
[X,L,G] = dare(Ad, Bd, Q,R);
K_controlled = -G;
%F = 1/(Cd*inv(eye(n)-(Ad+Bd*K_controlled))*Bd);
F = 0;
K = K_controlled;   
%%END: SOLUTION CODE