function [phi_aug, Gamma_aug, C_aug] = augmentSystem(tau, h, pipeline_flag)
    [A, B, C, D] = systemModel();
    %% STUDENTS NEED TO WRITE CODE FOR THIS FUNCTION
    %% BEGIN: SOLUTION CODE

    %% pipeline model
    if (pipeline_flag == 1)
        p = ceil(tau / h);
        nx = size(A, 2);
        nu = size(B, 2);

        sysc = ss(A, B, C, D);
        sysd = c2d(sysc, h);

        phi = sysd.a;
        gamma = sysd.b;

        phi_aug = [phi, gamma, zeros(nx, nu * (p - 1));
                zeros(nu * (p - 1), nx), zeros(nu * (p - 1), nu), eye(nu * (p - 1), nu * (p - 1));
                zeros(nu, nx), zeros(nu, nu), zeros(nu, nu * (p - 1))];
        Gamma_aug = [zeros(nx, nu);
                zeros(nu * (p - 1), nu);
                eye(nu, nu)];
        C_aug = [C, zeros(1, nu * (p))];

        %% For Non-Pipeline Model
    else
        sys_check = ss(A, B, C, D, 'InputDelay', tau);
        sysd1 = c2d(sys_check, h);
        sysd_aug = absorbDelay(sysd1);
        phi_aug = sysd_aug.a;
        Gamma_aug = sysd_aug.b;
        C_aug = sysd_aug.c;
    end

    %%END: SOLUTION CODE
