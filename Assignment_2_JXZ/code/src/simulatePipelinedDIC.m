function simulatePipelinedDIC(h,tauSystemScenarios,phi,Gamma,C_aug,K,F,SIMULATION_TIME,reference,CONTROLLER_TYPE)

%% LOAD THE SYSTEM MODEL
[A,~,~,~]=systemModel();
nf=max(ceil(tauSystemScenarios/h),1);
nSimulationSteps=ceil(SIMULATION_TIME/h);

% x0 = [0;0;0.00;0;];
% z0 = [x0; zeros(gamma,1)];

z0 = zeros(length(cell2mat(phi)),1);  %initialise for the state matrix A
% user code
z0(3)=-1;
x(1) = cell2mat(C_aug)*z0;
time(1) = 0;
e(1) = cell2mat(C_aug)*z0;
input(1)=0
 

for i=1:nSimulationSteps
       
   if i==1
       time(1) = 0;
   else 
       time(i) = time(i-1) + h;
       
   end
   
    y(i) = cell2mat(C_aug)*z0;
    
    if CONTROLLER_TYPE==1
        u = cell2mat(K)*z0+cell2mat(F)*reference;
    else
        e(i+1) = e(i) + y(i) - reference;
        u = cell2mat(K)*[z0;e(i)];        
    end
    input(i) = u;
        
    z_1 = cell2mat(phi)*z0 + cell2mat(Gamma)*u;
    z0 = z_1;
   
end

plot(time, y,'b', time, reference,'r');
xlabel('time (sec)') 
ylabel('y') 
grid on
figure
plot(time, input,'b');
xlabel('time (sec)') 
ylabel('u') 
grid on
