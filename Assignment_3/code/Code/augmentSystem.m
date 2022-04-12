function LKAS_CS = augmentSystem(tau, h, n_pipeline, LKAS_CS)
    n = LKAS_CS.nx;
    if (n_pipeline <= 1)
    % sequential style
        for i=1:length(tau)
            sysc = ss(LKAS_CS.A, LKAS_CS.B, LKAS_CS.C, LKAS_CS.D, 'InputDelay', tau);
            sysd = c2d(sysc, h);
            % absorbDelay is the key function to augment        
            sysd_aug = absorbDelay(sysd);
            
            Phi_aug{i} = sysd_aug.a;
            Gamma_aug{i} = sysd_aug.b;
            C_aug{i} = sysd_aug.c;
        end
    else
    % pipelined style
        for i=1:length(tau)
            disp("pipe")
            nf = max(1, ceil(tau/h));
            nf_wc = max(nf);
            
            sysc = ss(LKAS_CS.A, LKAS_CS.B, LKAS_CS.C, LKAS_CS.D);
            sysd = c2d(sysc, h);
            Ad = sysd.a; Bd = sysd.b; Cd = sysd.c;
            phi2 = [zeros(nf_wc-1,n)  zeros(nf_wc-1,1)     eye(nf_wc-1, nf_wc-1)];
            phi3 = [zeros(1,n)        zeros(1,1)           zeros(1,nf_wc-1)];
%             phi{i} = [Ad zeros(n,nf_wc-nf(i)) Bd;
%                                   phi2;
%                                   phi3];
            Phi_aug{i} = [Ad Bd zeros(n, nf_wc-1);
                          phi2;
                          phi3];
                          
            Gamma_aug{i} = [zeros(n, 1);
                            zeros(nf_wc-1, 1);
                            1];
            
            C_aug{i} = [Cd zeros(1, nf_wc)];


        end 
    end
    
    LKAS_CS.Phi_aug = Phi_aug;
    LKAS_CS.Gamma_aug = Gamma_aug;
    LKAS_CS.C_aug = C_aug;
end
