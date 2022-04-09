function [K, F, cqlf_Ai] = designControlGainsLQI(Ad, Bd, Cd, Q, R)

    n=length(Ad);
    %Design using dare
    [~, ~, G] = dare(Ad, Bd, Q, R);
    K_controlled = -G;
    
    F = 1 / (Cd * inv(eye(n) - (Ad + Bd * K_controlled)) * Bd);
    K = K_controlled;   
    cqlf_Ai= Ad + (Bd * K);
end    