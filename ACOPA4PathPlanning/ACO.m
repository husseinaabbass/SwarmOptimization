% Note: this ACO is for continuous problems, is different from the original
% ACO for discrete problems
% Version: 4.9.2018 
% This is ACO function to optimize continuous problems, based on the paper
% "Socha, K., & Dorigo, M. (2008). Ant colony optimization for continuous domains. 
% European journal of operational research, 185(3), 1155-1173."
% Copyright reserved 
% Contact: (jing.liu5@student.adfa.edu.au; liujing2605@gmail.com)
function [Gbest, GbestValue, ConvergenceData]=ACO(Path, PathLength, Dimension, UpperBound, LowerBound, InitPos, SafetyDistance, SafetyTime, SwarmSize, MaximumFEs, Num_WayPoints)

%-------------------- Initialization--------------------------
zeta=1;   % a parameter>0, is similar to the pheromone evaporation rate
q=0.5;         % a parameter to influence the distribution of solution weights
Ant=InitPos; 
FitnessValue=MultiCoordinationCost(Path, PathLength, Ant, Dimension, SafetyDistance, SafetyTime, Num_WayPoints);

[~,SortIndex]=sort(FitnessValue);
Ant=Ant(:,SortIndex);
FitnessValue=FitnessValue(SortIndex);
GbestValue=FitnessValue(1);
ConvergenceData=zeros(1,MaximumFEs+1);
ConvergenceData(1)=GbestValue;
CurrentFEs=1;
[StandardDeviation, NewAnt]=deal (zeros(Dimension, SwarmSize));

SolutionWeights=1/(q*SwarmSize*sqrt(2*pi))*exp(-0.5*(((1:SwarmSize)-1)/(q*SwarmSize)).^2);
Probability=SolutionWeights./sum(SolutionWeights);

while CurrentFEs<=MaximumFEs 
      
%---------------------Update individuals------------------------------
    % standardDeviation
    for i=1:SwarmSize
        StandardDeviation(:,i)=zeta.*sum(abs(Ant(:,i)-Ant),2)./(SwarmSize-1);  % Dimension *SwarmSize
    end
    % generate new population
    for i=1:SwarmSize
        for j=1:Dimension
            ell=RouletteWheelSelection(Probability);
            NewAnt(j,i)=Ant(j,ell)+StandardDeviation(j,ell)*randn;
        end
    end
    
    NewAnt=BoundLimits(NewAnt,LowerBound,UpperBound); 
%---------------------Evaluation----------------------------------------
    NewFitnessValue=MultiCoordinationCost(Path, PathLength, NewAnt, Dimension, SafetyDistance, SafetyTime, Num_WayPoints);
    %feval(Problem, NewAnt, ProblemIndex); 
    allSwarm=[Ant NewAnt];
    allFitnessValue= [FitnessValue NewFitnessValue];
    
    % sort
    [~, SortIndex]=sort(allFitnessValue);
    Ant=allSwarm(:,SortIndex(1:SwarmSize));
    FitnessValue=allFitnessValue(SortIndex(1:SwarmSize));
    
    CurrentFEs=CurrentFEs+SwarmSize;
    ConvergenceData(CurrentFEs-SwarmSize+1:CurrentFEs)=NewFitnessValue;    
end
    Gbest=Ant (:,1);
    %NewFitnessValue=MultiCoordinationCost(Path, PathLength, Gbest, Dimension, SafetyDistance, SafetyTime, Num_WayPoints);

    GbestValue=FitnessValue(1);
end

