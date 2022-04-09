function test()

format short;
clc;
clear all;

%% Response time analysis -- preemptive case

%% period, deadline, wcet and prioriy
 T = [10, 0.1, 0.05, 1;
      10, 0.2, 0.05, 2;
      10, 1.2, 0.5, 3;    
      10, 4, 0.5, 4;
    ];
 

number_of_tasks = length(T(:,1));

for task = 1:number_of_tasks
    R(task) = 0;      
    while(1)
        R_pre = R(task);
        HP_waiting = 0;
        for i = 1:number_of_tasks
            if (i ~= task) && (T(i,4)<T(task,4))
                HP_waiting = HP_waiting + ceil(R(task)/T(i,1))*T(i,3);           
            end             
        end    
        R(task) = T(task,3) + HP_waiting;
%         %% already missed deadline
%         if R(task) > T(task,2)
%             R(task) = -1;
%             break;
%         end
        %% termination condition
        if (R(task) == R_pre)
            break;            
        end
    end
end



R

