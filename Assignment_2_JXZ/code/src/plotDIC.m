function plotDIC(pattern,time,time_u,yL,df)
%% Plot results: yL output, df output, MSE, ST
%BEGIN:ADD ^\omega to pattern for legends in the fig
for i=1:length(pattern)
    if isa(pattern{i},'char')
        pattern{i}=sprintf('(%s)^{\\omega}',pattern{i});
    else
        pattern{i}=sprintf('[%s]^{\\omega}',num2str(pattern{i}));
    end
end
%END:ADD ^\omega
%% plot time vs yL
s = rng; %random seed to have same colours for both the plots. E.g. rng(1,'twister');
plotPublication(pattern,time,yL,'time (s)', 'y', 'y',s); 
%% plot time vs df
rng(s); %setting the same random seed to have same colours for the second plot
plotPublication(pattern,time_u,df,'time (s)', 'u', 'u',s); 
