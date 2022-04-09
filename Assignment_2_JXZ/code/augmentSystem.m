function [phi_aug, Gamma_aug, C_aug] = augmentSystem(tau, h, pipeline_flag)
    [A, B, C, D] = systemModel();
    %% STUDENTS NEED TO WRITE CODE FOR THIS FUNCTION
    %% BEGIN: SOLUTION CODE

    %% pipeline model
    if (pipeline_flag == 1)
        for i=1:length(tau)
            p = ceil(tau(i) / h);
            nx = size(A, 2);
            nu = size(B, 2);

            sysc = ss(A, B, C, D);
            sysd = c2d(sysc, h);

            phi = sysd.a;
            gamma = sysd.b;

            phi_aug{i} = [phi, gamma, zeros(nx, nu * (p - 1));
                    zeros(nu * (p - 1), nx), zeros(nu * (p - 1), nu), eye(nu * (p - 1), nu * (p - 1));
                    zeros(nu, nx), zeros(nu, nu), zeros(nu, nu * (p - 1))];
            Gamma_aug{i} = [zeros(nx, nu);
                    zeros(nu * (p - 1), nu);
                    eye(nu, nu)];
            C_aug{i} = [C, zeros(1, nu * (p))];
        end

        %% For Non-Pipeline Model
    else
        for i=1:length(tau)
            sys_check = ss(A, B, C, D, 'InputDelay', tau(i));
            sysd1 = c2d(sys_check, h);
            sysd_aug = absorbDelay(sysd1);
            phi_aug{i} = sysd_aug.a;
            Gamma_aug{i} = sysd_aug.b;
            C_aug{i} = sysd_aug.c;
        end
    end
    
    if length(tau)==1
    
        phi_aug=cell2mat(phi_aug);
        Gamma_aug=cell2mat(Gamma_aug);
        C_aug=cell2mat(C_aug);
             
    end

    %%END: SOLUTION CODE
