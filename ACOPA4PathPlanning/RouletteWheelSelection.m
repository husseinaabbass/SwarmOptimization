% revised at 8 pm 28.8.2018
% this function is for roulette wheel selection
% it is called from BSO.m, ABC.m, GSO.m
% P: the probability
% j: the selected individual
function SelectedIndex=RouletteWheelSelection(Prob)
    r=rand.*sum(Prob);
    CumProb=cumsum(Prob,2);
    SelectedIndex=find(r<=CumProb,1,'first');
    
    %Prob=Prob/(sum(Prob)+realmin);
    %SelectedIndex=randsrc(1,1,[1:numel(Prob);Prob]); % requires
    %sum(Prob)=1;
    
end


