% m: array Input (the fitness values of solutions, minimize problem)
% n: tournament size
function [index] = TournamentSelection(m, n)
len=length(m);

temp=ceil(len*rand(1,n))   ;%randi(len,1,n);
[~, ind]=min(m(temp));
index=temp(ind);

end

