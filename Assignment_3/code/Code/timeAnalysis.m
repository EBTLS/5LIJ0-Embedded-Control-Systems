function LKAS = timeAnalysis(LKAS)
    % execution time for each task
    Time.t_A = 0.5;
    Time.t_C = 0.016;
    Time.t_isp1 = 10.3;
    Time.t_isp2 = 8.65;
    Time.t_isp3 = 5;
    Time.t_RoID = 0.3;
    Time.t_RoIP = 1.4;
    Time.t_RoIM = 0.16;
    
    LKAS.tau = (Time.t_isp1 + LKAS.n_ROI * Time.t_isp2 / LKAS.n_parallelization+ Time.t_isp3 + ...
                Time.t_RoID + LKAS.n_ROI * Time.t_RoIP / LKAS.n_parallelization+ Time.t_RoIM + Time.t_A + Time.t_C)* ...
                1e-3;
    LKAS.time = Time;
    
    LKAS.h = LKAS.fh * ceil(LKAS.tau / LKAS.fh / LKAS.n_pipeline);
    fprintf('System time analysis: tau=%.3f, h=%.3f\n',LKAS.tau, LKAS.h);
    LKAS.tau = LKAS.h * LKAS.n_pipeline;
end