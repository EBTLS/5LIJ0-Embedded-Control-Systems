function [phi_aug, Gamma_aug, C_aug] = augmentSystem(tau,h,pipeline_flag)
[A,B,C,D] = systemModel();
%% STUDENTS NEED TO WRITE CODE FOR THIS FUNCTION
%% BEGIN: SOLUTION CODE

%% pipeline model
if (pipeline_flag==1)
    
    
%% For Non-Pipeline Model
else
     sys_check = ss(A,B,C,D,'InputDelay',tau);
     sysd1 = c2d(sys_check,h);
     sysd_aug=absorbDelay(sysd1);
     phi_aug=sysd_aug.a;
     Gamma_aug=sysd_aug.b;
     C_aug=sysd_aug.c;
end
%%END: SOLUTION CODE