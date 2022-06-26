% note: this is for minimum problems
% Version: 4.9.2018 
% this function is to find the better one between a and b
% Copyright reserved 
% Contact: (jing.liu5@student.adfa.edu.au, h.abbass@adfa.edu.au) 
function [better, betterFit]=GreedySelection(a,aFit,b,bFit)

    if  aFit<bFit
        better=a;
        betterFit=aFit;
    else
        better=b;
        betterFit=bFit;
    end

end