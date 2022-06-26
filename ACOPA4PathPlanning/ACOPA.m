% Copyright reserved 
% ACOPA for path planning
% Contact: (jing.liu5@student.adfa.edu.au)
function [TGbest, TGbestValue, FEvBestFitness]= ACOPA (MaximumFEs, SwarmSize, InitPos, ModelInfor, pRepair)
flag_restart=1;  
zeta=0.6;
q=0.2;
win=10;
AgentIndex=1;
eval_agent(AgentIndex)=1;  % fitness values evaluations (FEs) count for each robot(agent)
TModelInfor=CordinateTransformation(ModelInfor, AgentIndex); % thansfer the coordinate system

TaskNumber=TModelInfor.TaskNumber;  % number of tasks/robots
Dimension=TModelInfor.Num_WayPoints; 
Bound=TModelInfor.Bound;
xmin=Bound(:,1);
xmax=Bound(:,2);
Penalty=3 ; %ModelInfor.Task(1,5)*(Dimension+1)*2;

y=zeros(Dimension, TaskNumber);  % y coordinates
SMaximumFEs=ceil(MaximumFEs/TaskNumber);  % maximum FEs for each task
TGbest=[];
TGbestValue=[];

flag_agent=0;  % if the optimisation of one agent'path is finished

%% parameters for Adaptive waypoints repair method
n_Oper= 3; % number of moving directions
win_size=3*win; % the window size before probabilities are updated
n_win=2;  % none of the strategies improved the solutions in the m previous W windows, reinitialise
n_Ants=SwarmSize; % number of new solutions
Experience_Oper=ones(1, n_Oper);

ConvergenceData = ones(1, MaximumFEs+1)*10^5;    % best fitness found

TrialIndex=1;
current_eval=1; %%% fitness function evaluations counter
%previous_eval=0;
iter=0;

%% Start initialization in the archive (PopSize, Dimension)
xant=InitPos';
fitx=SingleCostFunction(xant', TModelInfor, AgentIndex);

%% Sort the population based on fitx
[fitx, indecies ] = sort( fitx );
xant = xant( indecies, : );
ConvergenceData(1)=fitx(1);

%StandardDeviation=zeros(PopSize, Dimension);
NewAnt= zeros(n_Ants, Dimension);

NoImprove=0;
SolutionWeights=1/(q*SwarmSize*sqrt(2*pi))*exp(-0.5*(((1:SwarmSize)-1)/(q*SwarmSize)).^2);
Probability=SolutionWeights./sum(SolutionWeights);

t=SwarmSize:-1:1;                                                    
Pci=0.5+0.4*(exp(10*(t-1)/(SwarmSize-1))-1)/(exp(10)-1);

while current_eval<MaximumFEs
    if flag_agent==1
        xant(SwarmSize/2+1:end,:)=xmin'+rand(SwarmSize/2, Dimension).*(xmax'-xmin');

        pp=repair1(xant(SwarmSize/2+1:end,:)',TModelInfor,AgentIndex, pRepair, flag_uniform);
        xant(SwarmSize/2+1:end,:)=pp';
        fitx=SingleCostFunction(xant', TModelInfor,AgentIndex);
        [fitx, indecies ] = sort( fitx );
        xant = xant( indecies, : );
        eval_agent(AgentIndex)=current_eval+1;
        current_eval=current_eval+n_Ants;

        ConvergenceData(current_eval-n_Ants+1:current_eval)=fitx(1); 

        flag_agent=0;
    else
        iter=iter+1;

%% ---------------------Update individuals------------------------------
        if mod(iter,win_size)==1
            Prob_Oper=Experience_Oper/(sum(Experience_Oper)+realmin);
            Experience_Oper=ones(1, n_Oper);
        end

        % generate new population
        % Prob_Oper
        flag_moving=RouletteWheelSelection(Prob_Oper);
        flag_uniform=[0 flag_moving];
        old_fitx=fitx(1);
        ttemp=rand(SwarmSize,Dimension);
        flag=(ttemp>Pci')*1;

        for i=1:n_Ants

            [NewAnt(i,:), Nfitx(i)]= NewSolConst(TModelInfor, xant, SwarmSize,Dimension, Probability, flag,zeta, AgentIndex, pRepair, flag_uniform);

        end
%% ---------------------Evaluation----------------------------------------      

        allSwarm=[xant; NewAnt];
        allFitnessValue= [fitx Nfitx];

        % sort
        [allFitnessValue, SortIndex]=sort(allFitnessValue);
        allSwarm=allSwarm(SortIndex,:);

        current_eval=current_eval+n_Ants;
        ConvergenceData(current_eval-n_Ants+1:current_eval)=allFitnessValue(1); 

        % record the number without impovement
        if fitx(1)<=allFitnessValue(1)
            NoImprove=NoImprove+1;
        else
            NoImprove=0;
        end

        %remove duplicates
        old_current_eval=current_eval;
        [allSwarm,allFitnessValue,current_eval] = hanDuplicate(allSwarm,allFitnessValue,xmin, xmax, TModelInfor,current_eval, AgentIndex);
        ConvergenceData(old_current_eval+1:current_eval)=allFitnessValue(1);

        xant=allSwarm(1:SwarmSize,:);
        fitx=allFitnessValue(1:SwarmSize);
        if NoImprove<=n_win*win_size
            Experience_Oper(flag_moving)=Experience_Oper(flag_moving)+abs((old_fitx-fitx(1)));
        else
            Experience_Oper=ones(1, n_Oper);
            Prob_Oper=Experience_Oper/(sum(Experience_Oper)+realmin);
        end

        Gbest=xant(1,:);
        GbestValue=fitx(1);
    end

    if flag_restart==1
        if current_eval>=SMaximumFEs*(AgentIndex-1)+ TrialIndex*SMaximumFEs/10 && GbestValue>=Penalty  % restart
            TrialIndex=TrialIndex+1;
            xant=xmin'+rand(SwarmSize, Dimension).*(xmax'-xmin');
            pp=repair1(xant',TModelInfor,AgentIndex, pRepair, flag_uniform);
            xant=pp';
            fitx=SingleCostFunction(xant', TModelInfor,AgentIndex);
            [fitx, indecies ] = sort( fitx );
            xant = xant( indecies, : );
            current_eval=current_eval+n_Ants;
            ConvergenceData(current_eval-n_Ants+1:current_eval)=fitx(1); 
        end
    end

    if current_eval>=SMaximumFEs*(AgentIndex)   % the next agent
        TrialIndex=1;
        fitx=SingleCostFunction(Gbest', TModelInfor,AgentIndex);
        [path, Gbest] = CordinatesRecover(TModelInfor, Gbest',ModelInfor, AgentIndex); % transfer it to the original coordinate system
        Gbest=Gbest';

        y(:, AgentIndex)=Gbest';
        ModelInfor.y=y;
        TGbestValue=[TGbestValue ; GbestValue]; % the total cost of all tasks
        TGbest=[TGbest ;  path(:,1); path(:,2)];   % the best solutions of all tasks
        flag_agent=1;
        AgentIndex=AgentIndex+1;
        if AgentIndex<= TaskNumber
          TModelInfor=CordinateTransformation(ModelInfor, AgentIndex);
          Bound=TModelInfor.Bound;
          xmin=Bound(:,1);
          xmax=Bound(:,2);
        end
    end

end
        
FEvBestFitness=ConvergenceData(1:MaximumFEs+1);
for l=1:MaximumFEs
    if isnan(FEvBestFitness( l))
        FEvBestFitness(l)=10^10;
    end

    if  ismember(l, eval_agent)% l==MaximumFEs/TaskNumber+1
        % do nothing
    else
        if FEvBestFitness(l)<FEvBestFitness(l+1)
            FEvBestFitness(l+1)=FEvBestFitness(l);
        end   
    end
end

TotalCost=sum(TGbestValue);
TGbestValue=[TotalCost; TGbestValue];
disp([ ' best fitness = ', num2str(TGbestValue')]);                              

end