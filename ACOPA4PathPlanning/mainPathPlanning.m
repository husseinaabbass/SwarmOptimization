% Version 23/Dec/2021
% The codes for the paper 
% Liu, J., Anavatti, S., Garratt, M., & Abbass, H. A. (2022). Modified continuous Ant Colony Optimisation for multiple Unmanned Ground Vehicle path planning. Expert Systems with Applications, 196, 116605.
% Copyright reserved 
% Please cite this paper if you use this code.
% Contact: (jing.liu5@unsw.edu.au; h.abbass@unsw.edu.au)
function mainPathPlanning (DimensionSet, TaskSet)
if nargin==0
    DimensionSet=20;   % the number of waypoints
    TaskSet=1;
end
%clear;
%clc;
AlgorithmName={'ACOPA'};
TrialTimes=30;
% algorithm parameters 
SwarmSize=40;
pRepair=1;  % probability of repair
flag_uniform=0;
TimeUsed=zeros(1, TrialTimes);

for Dimension=[DimensionSet]  % the number of waypoints 
       
file_result=strcat('results/', 'Size',int2str(SwarmSize),'Dim',int2str(Dimension),'Result.txt'); % record results 
find_file_result=fopen(file_result,'a+');

for TaskIndex=[TaskSet]  %1:12
    [TaskInfor, ThreatInfor, ObstacleInfor, MovingObstacleInfor ]=EnvironmentInfor(TaskIndex);
    ModelInfor=ModelSetup(TaskInfor, ThreatInfor, ObstacleInfor,MovingObstacleInfor, Dimension); 
    TaskNumber=ModelInfor.TaskNumber;
    MaximumFEs=(50000+(Dimension-20)*2500)*TaskNumber;
    Penalty=3;
    % Initialise positions
    BestPathAndCost=zeros(Dimension*TaskNumber+TaskNumber+1, TrialTimes);
    
    for AlgorithmIndex=[1 ]  %
        ConvergenceData=zeros(TrialTimes,MaximumFEs+1);  %record the convergence data for each run 
        file_convergence_data=strcat('results/', char(AlgorithmName(AlgorithmIndex)),'Prob',int2str(TaskIndex),'Dim',int2str(Dimension),'Data.txt'); %record the convergence data
        find_file_convergence_data=fopen(file_convergence_data,'w');
        file_eachTrialResult_data=strcat('results/',char(AlgorithmName(AlgorithmIndex)),'Prob',int2str(TaskIndex),'Dim',int2str(Dimension),'Result.txt'); %record the best position and fitness values found by each trial
        find_file_eachTrialResult_data=fopen( file_eachTrialResult_data,'w');
        file_bestpath=strcat('results/',char(AlgorithmName(AlgorithmIndex)), 'Prob',int2str(TaskIndex), 'Dim',int2str(Dimension),'Path.txt'); % record the best path among all the trials by all algorithms
        find_file_bestpath=fopen(file_bestpath,'w');
        
        Algorithm=str2func(char(AlgorithmName(AlgorithmIndex)));
        
        for TrialIndex=1:TrialTimes
            TimeStart=tic; % record the running time
            
            TModelInfor=CordinateTransformation(ModelInfor, 1);
            Bound=TModelInfor.Bound;
            InitPos=Bound(:,1)+rand(Dimension, SwarmSize).*(Bound(:,2)-Bound(:,1));
            InitPos(:,SwarmSize/2+1:end)=repair(InitPos(:,SwarmSize/2+1:end),TModelInfor,1, pRepair, flag_uniform);
            [Gbest, GbestValue, GbestHistory]=feval(Algorithm, MaximumFEs, SwarmSize, InitPos, ModelInfor,pRepair);
            
            BestPathAndCost(1:TaskNumber+1, TrialIndex)=GbestValue; % Optimal cost
            BestPathAndCost(TaskNumber+2:(Dimension+2)*TaskNumber*2+TaskNumber+1, TrialIndex)= Gbest; 
            ConvergenceData(TrialIndex,:)=GbestHistory(1:MaximumFEs+1);
            
            TimeUsed(TrialIndex)=toc(TimeStart);
        end
        AverageTime=mean(TimeUsed);
        StdTime=std(TimeUsed);
        MeanOfBestCost=mean(BestPathAndCost(1,:));
        StdOfBestCost=std(BestPathAndCost(1,:));
        MaxOfBestCost=max(BestPathAndCost(1,:));
        [MinOfBestCost, BestIndex]=min(BestPathAndCost(1,:));
        BBestPath=BestPathAndCost(TaskNumber+2:TaskNumber+(Dimension+2)*TaskNumber*2+1, BestIndex);
        
        A=sum(BestPathAndCost(2:1+TaskNumber,:)<=Penalty,1);
        FeasibleCostIndex=find(A==TaskNumber);
        FeasibleCost=BestPathAndCost(1, FeasibleCostIndex);
        SuccessfulRate=length(FeasibleCostIndex)/TrialTimes;
        MeanOfFeasible=mean(FeasibleCost);
        StdOfFeasible=std(FeasibleCost);
        
        
        MedianOfBestCost=median(BestPathAndCost(1,:));
        %MedianIndex = find(BestPathAndCost(1,:) == MedianOfBestCost);
        %MedianY=BestPathAndCost(2:Dimension+1, MedianIndex);
        %MedianBestPath=CordinatesRecover(ModelInfor, MedianY, TaskInfor);
        
        AverageConvergenceData=sum(ConvergenceData(:,1:end),1)/TrialTimes; % for the convergence graph
        
        fprintf('%i  %s  %.2f  %.2f  %.2f  %.2f  %.2f  %.2f  %.2f  %.2f  %.2f  %.2f\r\n', TaskIndex, char(AlgorithmName(AlgorithmIndex)), SuccessfulRate, MinOfBestCost, MaxOfBestCost, MedianOfBestCost, MeanOfBestCost, StdOfBestCost, MeanOfFeasible, StdOfFeasible, AverageTime, StdTime);
        fprintf(find_file_result, '%i  %s  %.4f  %.4f  %.4f  %.4f  %.4f  %.4f  %.4f %.4f  %.4f  %.4f\r\n', TaskIndex, char(AlgorithmName(AlgorithmIndex)), SuccessfulRate, MinOfBestCost, MaxOfBestCost, MedianOfBestCost, MeanOfBestCost, StdOfBestCost, MeanOfFeasible, StdOfFeasible, AverageTime, StdTime);
        fprintf(find_file_bestpath, '%.4f\t', BBestPath);
        
        fprintf(find_file_convergence_data, '%12.4e', AverageConvergenceData); %record the convergence data
        fprintf(find_file_eachTrialResult_data, '%12.4e', BestPathAndCost);
        fclose(find_file_convergence_data);
        fclose(find_file_eachTrialResult_data);
        fclose(find_file_bestpath);
    end
    
end
end
fclose ('all');
end
 