function simulateDIC(Initial_value, CONTROLLER_TYPE, h, tauSystemScenarios, phi, Gamma, C_aug, K, F, PATTERN, SIMULATION_TIME, Reference, fh)

%% LOAD THE SYSTEM MODEL
[A,~,~,~,~]=systemModel();
nSimulationSteps=ceil(SIMULATION_TIME/fh);

%% BEGIN: Iterate for each pattern
[timing_pattern] = expressionToTimingPattern(PATTERN,length(tauSystemScenarios));
num_plots=length(timing_pattern);

for loop=1:num_plots
    for i=1:nSimulationSteps
        if i==1 
          %%initialization of variables 
          trackCurrentPeriod=1; %checkSPADe --> to keep track of current h, as i progresses at fh
          timeScenario=1; %tS --> to keep track of time with respect to scenarios
          timeY(1) = 0;
          timeU(1) = 0;
          j=1; %to keep track of the length(timing_pattern[])
          x0 = zeros(length(A),1);  %initialise for the state matrix A
          x0(3) = Initial_value;
%           x0
          z0 = [x0; 0]; %augmented state matrix; due to implementation-aware matrices additional zeros are not needed
          e(1) = C_aug{1}*z0- Reference(1);
        end

        if i==trackCurrentPeriod
            m=timing_pattern{loop}(j);
%            mse_reference(timeScenario)=reference_resized(i);
            y(timeScenario) = C_aug{m}*z0;  
            if CONTROLLER_TYPE==2 %LQI          
               e(timeScenario+1) = e(timeScenario) + y(timeScenario) - Reference;
               u(timeScenario) = K{m}*[z0;e(timeScenario)];
            else %LQR
                u(timeScenario) = K{m}*z0+F{m}*Reference;            
            end
            z_1 = phi{m}*z0 + Gamma{m}*u(timeScenario);
            z0 = z_1;
            trackCurrentPeriod=trackCurrentPeriod+ceil((h(m)/fh));
            if i>1 %update simulation times
                timeY(timeScenario) = timeY(timeScenario-1) + h(m);
                timeU(timeScenario) = timeY(timeScenario-1) + tauSystemScenarios(m); 
            end
            timeScenario=timeScenario+1;  
            if j==length(timing_pattern{loop}) %update pattern sequence
                j=1;
            else
                j=j+1;
            end
        end %i=
        
          %% Storing values needed for plotting
    time{loop}=timeY;
    yL{loop}=y;
    time_u{loop}=timeU;
    df{loop}=u;
      
    end
end


plotDIC(PATTERN,time,time_u,yL,df);

