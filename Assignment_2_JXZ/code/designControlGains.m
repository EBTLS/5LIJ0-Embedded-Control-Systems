function [K,F] = designControlGains(Ad,Bd,Cd,case_id,type)

%% STUDENTS NEED TO WRITE CODE FOR THIS FUNCTION
%% BEGIN: SOLUTION CODE

% if nargin<=3
%     Q=10*(Cd')*Cd;
% end
% 
% if nargin<=4
%     R=3;
% end

if case_id==1
    
    Q=5*(Cd')*Cd; % 0.675s
%     Q=10*Cd'*Cd;  %0.557s
%     Q=20*Cd'*Cd; %0.459s
%     Q=100*Cd'*Cd; %0.294s
    R=3;
    
elseif case_id==2
    
%     Q=5*Cd'*Cd; %0.595s;
%     Q=10*Cd'*Cd; %0.475s 
    Q=20*Cd'*Cd; %0.375s
%     Q=100*Cd'*Cd; %0.206s
    R=3;
    
elseif case_id==3
    
%       Q=5*Cd'*Cd; % 0.55s
      Q=7*Cd'*Cd; % 0.605s 
%         Q=10*Cd'*Cd; % 0.545s not stable 
%       Q=20*Cd'*Cd; % 0.44s not stable
%     Q=100*Cd'*Cd; %0.275s, not stable
    R=3;
    
elseif case_id==4
    
    Q=7*Cd'*Cd; % 0.605s
    R=3;
    
elseif case_id==5
    
%     Q=5*Cd'*Cd; % 0.607s
%     Q=7*Cd'*Cd; % 0.545s 
%     Q=10*Cd'*Cd; % 0.48s 
    Q=20*Cd'*Cd; % 0.382s not stable
%     Q=100*Cd'*Cd; %0.217s, not stable
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