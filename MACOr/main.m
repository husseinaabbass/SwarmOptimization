% code for 
% Liu, J., Anavatti, S., Garratt, M., & Abbass, H. A. (2022). Multi-operator
% continuous ant colony optimisation for real world problems. Swarm and Evolutionary Computation, 69, 100984.
% Copyright reserved 
% Please cite this paper if you use this code.
% Contact: (jing.liu5@unsw.edu.au; h.abbass@unsw.edu.au) 
clear all;
clc;
AlgorithmName={'MACOr'};
TrilTimes=1;
MaxFEs=150000;
SwarmSize=100;  % the archive size
n_Ants=100;
zeta=0.6;
q=0.2;
warning('off','all');
time_used=zeros(1, TrilTimes);

for AlgorithmIndex=[1 ] % for each algorithm
     disp(['Algorithm ', num2str(AlgorithmIndex), ' ', char(AlgorithmName(AlgorithmIndex))]);
     Algorithm=str2func(char(AlgorithmName(AlgorithmIndex)));

     for ProblemIndex=[1 2 4:22]    % 1 2 4:22
         disp(['ProblemIndex ', num2str(ProblemIndex)]);
         data(ProblemIndex)=feval(Algorithm,ProblemIndex,TrilTimes, MaxFEs, SwarmSize, n_Ants, zeta, q); %optimize function ind_fun using algorithm ind_alg      
     end 
     t=clock;
     %file_name=strcat('Result', num2str(flag_LS), char(AlgorithmName(AlgorithmIndex)),'.mat'); % record results of  all functions by all algorithms (average, std, time)
     file_name=strcat('Result', char(AlgorithmName(AlgorithmIndex)), num2str(t(4:5), '%02d'),num2str(round(t(6)), '%02d'),'.mat'); % record results of  all functions by all algorithms (average, std, time) 
     save(file_name, 'data');
end 


