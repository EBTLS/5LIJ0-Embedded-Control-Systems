function a = SC_plot(DR, DR_SC, x0, tag)
    time(1) = 0;
    x_1 = [];
    for i=1:3000
        if i==1
            time(1) = 0;
        else
            time(i) = time(i-1) + DR.h;
        end    
        r(i) = 1;
        y(i) = DR_SC.C*x0;
        u(i) = DR_SC.K*x0 + DR_SC.F*r(i);
        x_1 = [x_1, DR_SC.phi*x0 + DR_SC.Gamma*u(i)];
        x0 = x_1(:,i);
    end

    figure;
    plot(time, y(1,:),'b', time, r(i),'r')
    grid on
    title("System output")
    xlabel("time/s")
    ylabel("output")
    
    switch tag
        case "DR"
            figure('Position', [10 10 900 400]);
            subplot(1,2,1)
            plot(time, u)
            title("Control input")
            xlabel("time/s")
            ylabel("input")
            grid on
            subplot(1,2,2)
            plot(time, x_1(3,:),'DisplayName', 'x3')
            hold on
            plot(time, x_1(4,:),'DisplayName', 'x4')
            legend
            grid on   
            title("State input")
            xlabel("time/s")
            ylabel("state")
            
        case "DCM"
            figure;
            plot(time, u)
            grid on
            title("Control input")
            xlabel("time/s")
            ylabel("input")
    end

    a = 1;
end