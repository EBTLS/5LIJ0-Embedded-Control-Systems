%%%%%
%%% 5LIJ0 Embedded Control Systems
%%% Assignment 1
%%% Author: Jiaxuan Zhang, Yiting Li
%%%%%

%% Clear

clear
clc

%% Configuration

Design_number = 3;

%% Design 1
if Design_number == 1
    
    
    %% Parameters
    % Dual Rotary System Parameters
    DR.Km = 4.4 * 1e-2;
    DR.J1 = 3.75 * 1e-6; 
    DR.J2 = 3.75 * 1e-6;
    DR.b = 1 * 1e-5;
    DR.k = 0.2656;
    DR.d = 3.125 * 1e-5;
    DR.nx = 5;
    DR.nu = 1;

    % DC Motor Speed Control System Parameters
    DCM.K = 0.01;
    DCM.J = 0.01;
    DCM.b = 0.1;
    DCM.R = 1;
    DCM.L = 0.5;
    DCM.nx = 3;
    DCM.nu = 1;

    % sampling time(s)
    DR.h = 2e-3;
    DCM.h = 2 * DR.h;
    DR.tau = 0.9375e-3;
    DCM.tau=0.8e-3;

    %% State Space Model
    % Dual Rotary System
    DR_CS.A = [0, 0, 1, 0;
               0, 0, 0, 1;
               -DR.k / DR.J1, DR.k / DR.J1, - (DR.d + DR.b) / DR.J1, (DR.d + DR.b) / DR.J1;
               DR.k / DR.J2, -DR.k / DR.J2, (DR.d + DR.b) / DR.J2, - (DR.d + DR.b) / DR.J2];
    DR_CS.B = [0;
               0;
               DR.Km / DR.J1;
               0];
    DR_CS.C = [1 1 0 0];

    % DC Motor Speed Control System
    DCM_CS.A = [-DCM.b / DCM.J, DCM.K / DCM.J;
                -DCM.K / DCM.L, -DCM.R / DCM.L];
    DCM_CS.B = [0;
                1 / DCM.L];
    DCM_CS.C = [1 0];

    %% Discrete-time model without delay
    % DR Control System
    % continuous-time
    DR_CS.sysc = ss(DR_CS.A, DR_CS.B, DR_CS.C, 0);

    % to discrete
    DR_CS.sysd = c2d(DR_CS.sysc, DR.h);
    DR_CS.phi = DR_CS.sysd.a; 
    DR_CS.Gamma = DR_CS.sysd.b; 
    DR_CS.Cd = DR_CS.sysd.c;

    % with delay
    % DR_CS.Gamma0 = c2d(DR_CS.sysc, DR.h-DR.tau).b;
    % DR_CS.Gamma1 = DR_CS.Gamma-DR_CS.Gamma0;
    % simpilfied calculation
    DR_CS.Gamma0 = DR.tau * DR_CS.B + 0.5 * DR_CS.A * (DR.h^2 - (DR.h - DR.tau)^2) * DR_CS.B;
    DR_CS.Gamma1 = (DR.h - DR.tau) * DR_CS.B + 0.5 * DR_CS.A * (DR.h - DR.tau)^2 * DR_CS.B;

    % augmentation
    DR_CS.phi_aug= [DR_CS.phi DR_CS.Gamma1; zeros(1,DR.nx)];
    DR_CS.Gamma_aug= [DR_CS.Gamma0; 1];
    DR_CS.Cd_aug= [DR_CS.Cd 0];

    % Desired closed-loop poles and pole placement
    DR_CS.alpha_aug = [0.65 0.65 0.65 0.65 0.65];

    % feedback vector
    DR_CS.K = -acker(DR_CS.phi_aug, DR_CS.Gamma_aug, DR_CS.alpha_aug);

    % feedforward gain
    temp = (eye(DR.nx) - DR_CS.phi_aug - DR_CS.Gamma_aug * DR_CS.K) \ DR_CS.Gamma_aug;
    DR_CS.F = 1 / (DR_CS.Cd_aug * temp);
    clear temp

    % SC_plot(DR, DR_CS, [0;0;0;0;0], "DR")


    %%  DCM Control System
    % continuous-time
    DCM_CS.sysc = ss(DCM_CS.A, DCM_CS.B, DCM_CS.C, 0);

    % to discrete
    DCM_CS.sysd = c2d(DCM_CS.sysc, DCM.h);
    DCM_CS.phi_aug = DCM_CS.sysd.a; 
    DCM_CS.Gamma_aug = DCM_CS.sysd.b; 
    DCM_CS.Cd = DCM_CS.sysd.c;

    % with delay
    DCM_CS.Gamma0_aug = (DCM_CS.A) \ (expm(DCM_CS.A * (DCM.h-DCM.tau))-expm(DCM_CS.A * 0)) * DCM_CS.B;
    DCM_CS.Gamma1_aug = (DCM_CS.A) \ (expm(DCM_CS.A * DCM.h)-expm(DCM_CS.A * (DCM.h - DCM.tau))) * DCM_CS.B;

    % augmentation
    DCM_CS.phi_aug = [DCM_CS.phi_aug DCM_CS.Gamma1_aug; zeros(1,DCM.nx)];
    DCM_CS.Gamma_aug= [DCM_CS.Gamma0_aug; 1];
    DCM_CS.Cd_aug = [DCM_CS.Cd 0];

    % Desired closed-loop poles and pole placement
    DCM_CS.alpha_aug = [0.98 0.98 0.98];

    % feedback vector
    DCM_CS.K = -acker(DCM_CS.phi_aug, DCM_CS.Gamma_aug, DCM_CS.alpha_aug);

    % feedforward gain
    temp = (eye(DCM.nx) - DCM_CS.phi_aug - DCM_CS.Gamma_aug * DCM_CS.K) \ DCM_CS.Gamma_aug;
    DCM_CS.F = 1 / (DCM_CS.Cd_aug * temp);

    clear temp

    % SC_plot(DCM, DCM_CS, [0;0;0], "DCM")
    % Simulink Simulation
    assignment1_2022_Simulink_init_Dualrotary(DR.tau, DR.h, DR_CS.K, 2*DR_CS.F)
    assignment1_2022_Simulink_init_DCmotor(DCM.tau, DCM.h, DCM_CS.K, DCM_CS.F)
    
    Hc = DR.h;
    Hc_DC = DCM.h;


%% Design 2
elseif Design_number==2
    
    
    %% Parameters
    % Dual Rotary System Parameters
    DR.Km = 4.4 * 1e-2;
    DR.J1 = 3.75 * 1e-6; 
    DR.J2 = 3.75 * 1e-6;
    DR.b = 1 * 1e-5;
    DR.k = 0.2656;
    DR.d = 3.125 * 1e-5;
    DR.nx = 4;
    DR.nu = 1;

    % DC Motor Speed Control System Parameters
    DCM.K = 0.01;
    DCM.J = 0.01;
    DCM.b = 0.1;
    DCM.R = 1;
    DCM.L = 0.5;
    DCM.nx = 2;
    DCM.nu = 1;

    % sampling time(s)
    DR.h = 2e-3;
    DCM.h = 2 * DR.h;
    DR.tau = 0.9375e-3;
    DCM.tau=0.8e-3;

    %% State Space Model
    % Dual Rotary System
    DR_CS.A = [0, 0, 1, 0;
            0, 0, 0, 1;
            -DR.k / DR.J1, DR.k / DR.J1, - (DR.d + DR.b) / DR.J1, (DR.d + DR.b) / DR.J1;
            DR.k / DR.J2, -DR.k / DR.J2, (DR.d + DR.b) / DR.J2, - (DR.d + DR.b) / DR.J2];
    DR_CS.B = [0;
            0;
            DR.Km / DR.J1;
            0];
    DR_CS.C = [1 1 0 0];

    % DC Motor Speed Control System
    DCM_CS.A = [-DCM.b / DCM.J, DCM.K / DCM.J;
            -DCM.K / DCM.L, -DCM.R / DCM.L];
    DCM_CS.B = [0;
            1 / DCM.L];
    DCM_CS.C = [1 0];


    %% DR Control System
    % continuous-time
    DR_CS.sysc = ss(DR_CS.A, DR_CS.B, DR_CS.C, 0);

    % to discrete
    DR_CS.sysd = c2d(DR_CS.sysc, DR.h);
    DR_CS.phi = DR_CS.sysd.a; 
    DR_CS.Gamma = DR_CS.sysd.b; 
    DR_CS.Cd = DR_CS.sysd.c;

    % Desired closed-loop poles and pole placement
    DR_CS.alpha = [0.7 0.7 0.7 0.7];
    % failed pole configuration
    % DR_CS.alpha = [0.65 0.65 0.65 0.65];

    % feedback vector5
    DR_CS.K = -acker(DR_CS.phi, DR_CS.Gamma, DR_CS.alpha)

    % feedforward gain
    temp = (eye(4) - DR_CS.phi - DR_CS.Gamma * DR_CS.K) \ DR_CS.Gamma;
    DR_CS.F = 1 / (DR_CS.Cd * temp);

    assignment1_2022_Simulink_init_Dualrotary(DR.tau,DR.h,DR_CS.K, DR_CS.F*2)

    %% DCM Control System
    % continuous-time
    DCM_CS.sysc = ss(DCM_CS.A, DCM_CS.B, DCM_CS.C, 0);

    % to discrete
    DCM_CS.sysd = c2d(DCM_CS.sysc, DCM.h);
    DCM_CS.phi = DCM_CS.sysd.a; 
    DCM_CS.Gamma = DCM_CS.sysd.b; 
    DCM_CS.Cd = DCM_CS.sysd.c;

    % Desired closed-loop poles and pole placement
    DCM_CS.alpha=[0.985,0.985]
    % failed pole configuration: input exceed limitation
    % DCM_CS.alpha = [0.98 0.98];

    % feedback vector
    DCM_CS.K = -acker(DCM_CS.phi, DCM_CS.Gamma, DCM_CS.alpha);

    % feedforward gain
    temp = inv(eye(2) - DCM_CS.phi - DCM_CS.Gamma * DCM_CS.K) * DCM_CS.Gamma;
    DCM_CS.F = 1 / (DCM_CS.Cd * temp);

    % DCM_CS.full_sysc = ss(DCM_CS.phi + DCM_CS.Gamma * DCM_CS.K, DCM_CS.Gamma * DCM_CS.F, DCM_CS.Cd, 0);
    % DCM_CS.full_sysd = c2d(DCM_CS.full_sysc, DCM.h);
    % DCM_CS.full_tf = tf(DCM_CS.full_sysd);

    % Simulink Simulation
    assignment1_2022_Simulink_init_DCmotor(DCM.tau,DCM.h,DCM_CS.K,DCM_CS.F)

    % SC_plot(DCM,DCM_CS,[0;0],"DCM")
    
    Hc = DR.h;
    Hc_DC = DCM.h;



    
%% Design 3    
elseif Design_number==3
    
    %% Parameters
    % Dual Rotary System Parameters
    DR.Km = 4.4 * 1e-2;
    DR.J1 = 3.75 * 1e-6; 
    DR.J2 = 3.75 * 1e-6;
    DR.b = 1 * 1e-5;
    DR.k = 0.2656;
    DR.d = 3.125 * 1e-5;
    DR.nx = 5;
    DR.nu = 1;

    % DC Motor Speed Control System Parameters
    DCM.K = 0.01;
    DCM.J = 0.01;
    DCM.b = 0.1;
    DCM.R = 1;
    DCM.L = 0.5;
    DCM.nx = 3;
    DCM.nu = 1;

    % sampling time(s)
    DR.h = 4e-3;
    DCM.h = 2 * DR.h;

    DR.tau = 0.9375e-3;
    DCM.tau = 0.8e-3;

    DCM.tau = 2.8875e-3;

    % DR.tau = 2e-3;
    % DCM.tau = 2e-3;


    %% State Space Model
    % Dual Rotary System
    DR_CS.A = [0, 0, 1, 0;
               0, 0, 0, 1;
               -DR.k / DR.J1, DR.k / DR.J1, - (DR.d + DR.b) / DR.J1, (DR.d + DR.b) / DR.J1;
               DR.k / DR.J2, -DR.k / DR.J2, (DR.d + DR.b) / DR.J2, - (DR.d + DR.b) / DR.J2];
    DR_CS.B = [0;
               0;
               DR.Km / DR.J1;
               0];
    DR_CS.C = [1 1 0 0];

    % DC Motor Speed Control System
    DCM_CS.A = [-DCM.b / DCM.J, DCM.K / DCM.J;
                -DCM.K / DCM.L, -DCM.R / DCM.L];
    DCM_CS.B = [0;
                1 / DCM.L];
    DCM_CS.C = [1 0];


    %% Discrete-time model without delay
    % DR Control System
    % continuous-time
    DR_CS.sysc = ss(DR_CS.A, DR_CS.B, DR_CS.C, 0);

    % to discrete
    DR_CS.sysd = c2d(DR_CS.sysc, DR.h);
    DR_CS.phi = DR_CS.sysd.a; 
    DR_CS.Gamma = DR_CS.sysd.b; 
    DR_CS.Cd = DR_CS.sysd.c;

    % with delay
    sys = c2d(DR_CS.sysc, DR.h-DR.tau);
    DR_CS.Gamma0 = sys.b;
    DR_CS.Gamma1 = DR_CS.Gamma - DR_CS.Gamma0;
    clear sys

    % augmentation
    DR_CS.phi_aug = [DR_CS.phi DR_CS.Gamma1; zeros(1,DR.nx)];
    DR_CS.Gamma_aug = [DR_CS.Gamma0; 1];
    DR_CS.Cd_aug = [DR_CS.Cd 0];

    % Desired closed-loop poles and pole placement
    DR_CS.alpha_aug = [0.4 0.4 0.4 0.4 0.4];
    % alternative [0.6 0.6 0.65 0.65 0.65]

    % feedback vector
    DR_CS.K = -acker(DR_CS.phi_aug, DR_CS.Gamma_aug, DR_CS.alpha_aug);

    % feedforward gain
    temp = (eye(DR.nx) - DR_CS.phi_aug - DR_CS.Gamma_aug * DR_CS.K) \ DR_CS.Gamma_aug;
    DR_CS.F = 1 / (DR_CS.Cd_aug * temp);
    clear temp


    %%  DCM Control System
    % continuous-time
    DCM_CS.sysc = ss(DCM_CS.A, DCM_CS.B, DCM_CS.C, 0);

    % to discrete
    DCM_CS.sysd = c2d(DCM_CS.sysc, DCM.h);
    DCM_CS.phi_aug = DCM_CS.sysd.a; 
    DCM_CS.Gamma = DCM_CS.sysd.b; 
    DCM_CS.Cd = DCM_CS.sysd.c;

    % with delay
    DCM_CS.Gamma0_aug = (DCM_CS.A) \ (expm(DCM_CS.A * (DCM.h-DCM.tau))-expm(DCM_CS.A * 0)) * DCM_CS.B;
    DCM_CS.Gamma1_aug = (DCM_CS.A) \ (expm(DCM_CS.A * DCM.h)-expm(DCM_CS.A * (DCM.h - DCM.tau))) * DCM_CS.B;

    % augmentation
    DCM_CS.phi_aug = [DCM_CS.phi_aug DCM_CS.Gamma1_aug; zeros(1,DCM.nx)];
    DCM_CS.Gamma_aug = [DCM_CS.Gamma0_aug; 1];
    DCM_CS.Cd_aug = [DCM_CS.Cd 0];

    % Desired closed-loop poles and pole placement
    DCM_CS.alpha_aug = [0.963 0.963 0.963];
    % alternative [0.984 0.984 0.984]

    % feedback vector
    DCM_CS.K = -acker(DCM_CS.phi_aug, DCM_CS.Gamma_aug, DCM_CS.alpha_aug);

    % feedforward gain
    temp = (eye(DCM.nx) - DCM_CS.phi_aug - DCM_CS.Gamma_aug * DCM_CS.K) \ DCM_CS.Gamma_aug;
    DCM_CS.F = 1 / (DCM_CS.Cd_aug * temp);

    clear temp

    % Simulink Simulation
    assignment1_2022_Simulink_init_Dualrotary(DR.tau, DR.h, DR_CS.K, 2*DR_CS.F)
    assignment1_2022_Simulink_init_DCmotor(DCM.tau, DCM.h, DCM_CS.K, DCM_CS.F)

    Hc = DR.h;
    Hc_DC = DCM.h;

    
%% Design 4
elseif Design_number==4
    
    
    %% Parameters
    % Dual Rotary System Parameters
    DR.Km = 4.4 * 1e-2;
    DR.J1 = 3.75 * 1e-6; 
    DR.J2 = 3.75 * 1e-6;
    DR.b = 1 * 1e-5;
    DR.k = 0.2656;
    DR.d = 3.125 * 1e-5;
    DR.nx = 4;
    DR.nu = 1;

    % DC Motor Speed Control System Parameters
    DCM.K = 0.01;
    DCM.J = 0.01;
    DCM.b = 0.1;
    DCM.R = 1;
    DCM.L = 0.5;
    DCM.nx = 2;
    DCM.nu = 1;

    % sampling time(s)
    DR.h = 4e-3;
    DCM.h = 2 * DR.h;

    DR.tau = 0;
    DCM.tau = 0;


    %% State Space Model
    % Dual Rotary System
    DR_CS.A = [0, 0, 1, 0;
                0, 0, 0, 1;
                -DR.k / DR.J1, DR.k / DR.J1, - (DR.d + DR.b) / DR.J1, (DR.d + DR.b) / DR.J1;
                DR.k / DR.J2, -DR.k / DR.J2, (DR.d + DR.b) / DR.J2, - (DR.d + DR.b) / DR.J2];
    DR_CS.B = [0;
                0;
                DR.Km / DR.J1;
                0];
    DR_CS.C = [1 1 0 0];

    % DC Motor Speed Control System
    DCM_CS.A = [-DCM.b / DCM.J, DCM.K / DCM.J;
                -DCM.K / DCM.L, -DCM.R / DCM.L];
    DCM_CS.B = [0;
                1 / DCM.L];
    DCM_CS.C = [1 0];


    %% DR Control System
    % continuous-time
    DR_CS.sysc = ss(DR_CS.A, DR_CS.B, DR_CS.C, 0);

    % to discrete
    DR_CS.sysd = c2d(DR_CS.sysc, DR.h);
    DR_CS.phi = DR_CS.sysd.a; 
    DR_CS.Gamma = DR_CS.sysd.b; 
    DR_CS.Cd = DR_CS.sysd.c;

    % Desired closed-loop poles and pole placement
    DR_CS.alpha = [0.45 0.45 0.45 0.45];
    % [0.4 0.4 0.4 0.4 0.4];
    % [0.45 0.45 0.45 0.45];

    % feedback vector5
    DR_CS.K = -acker(DR_CS.phi, DR_CS.Gamma, DR_CS.alpha);

    % feedforward gain
    temp = (eye(DR.nx) - DR_CS.phi - DR_CS.Gamma * DR_CS.K) \ DR_CS.Gamma;
    DR_CS.F = 1 / (DR_CS.Cd * temp);


    %% DCM Control System
    % continuous-time
    DCM_CS.sysc = ss(DCM_CS.A, DCM_CS.B, DCM_CS.C, 0);

    % to discrete
    DCM_CS.sysd = c2d(DCM_CS.sysc, DCM.h);
    DCM_CS.phi = DCM_CS.sysd.a; 
    DCM_CS.Gamma = DCM_CS.sysd.b; 
    DCM_CS.Cd = DCM_CS.sysd.c;

    % Desired closed-loop poles and pole placement
    DCM_CS.alpha = [0.97 0.97];
    % [0.963 0.963 0.963];
    % [0.968 0.968]; 

    % feedback vector
    DCM_CS.K = -acker(DCM_CS.phi, DCM_CS.Gamma, DCM_CS.alpha);

    % feedforward gain
    temp = inv(eye(DCM.nx) - DCM_CS.phi - DCM_CS.Gamma * DCM_CS.K) * DCM_CS.Gamma;
    DCM_CS.F = 1 / (DCM_CS.Cd * temp);

    % Simulink Simulation
    assignment1_2022_Simulink_init_Dualrotary(0.9375e-3, DR.h, DR_CS.K, DR_CS.F*2)
    assignment1_2022_Simulink_init_DCmotor(0.8e-3, DCM.h, DCM_CS.K, DCM_CS.F)




end