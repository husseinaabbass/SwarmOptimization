% code for improved continuous ant colony optimization algorithms for
% real-world engineering optimization problems
% Jing Liu
% select different strategies by probability, MultiOperator7
function data_performance= MACOr (I_fno, reps, max_eval, swarm_size, ~, zeta, q)
    tic
    xmax=[];
    xmin=[];
    Dimension=0;  % number of the decision variables
    
    [Dimension, xmax, xmin] = define_boundaries (Dimension, xmax, xmin, I_fno); %%%% Initialize the boundaries
    swarm_size=max(swarm_size, Dimension); % archive size
    
    n_Oper=4;
    Experience_Oper=ones(1, n_Oper);
    win_size=n_Oper*5; % the window size before probabilities are updated
    n_win=2;  % none of the strategies improved the solutions in the m previous W windows, reinitialise
    n_Ants=swarm_size-mod(swarm_size,3); % number of new solutions
    
    bestInd_data =zeros(reps,Dimension);
    bestFitness=zeros(reps,1);
    EvBestFitness = ones(reps, max_eval)*10^15;    % best fitness found
    x=zeros(swarm_size,Dimension);
    fitx=zeros(1, swarm_size);
    for time=1:reps
      current_eval=0; % fitness function evaluations counter
      iter=0;

      % Start initialization in the archive (swarm_size, Dimension)
       for i=1:swarm_size
            for j=1:Dimension
                x(i,j)=xmin (j) +rand*(xmax(j)-xmin(j));
            end
            fitx(i)=fnceval(x(i,:), I_fno);
       end % End initialization
       
       % assign a random-walk strategy (Gaussian or levy) randomly to each
       % solution, 0 means Gaussian, 1 means levy
       flag=randi([0,1], swarm_size, 1);
   
       current_eval = current_eval+ swarm_size;
       EvBestFitness(time, current_eval-swarm_size+1 : current_eval)=fitx;
       
       % Sort the population based on fitx
       [fitx, indecies ] = sort( fitx );
        x = x( indecies, : );
    
        %StandardDeviation=zeros(swarm_size, Dimension);
        NewAnt= zeros(n_Ants, Dimension);

        NoImprove=0;
        SolutionWeights=1/(q*swarm_size*sqrt(2*pi))*exp(-0.5*(((1:swarm_size)-1)/(q*swarm_size)).^2);
        Probability=SolutionWeights./sum(SolutionWeights);

        while current_eval<max_eval
            iter=iter+1;
          
        %% ---------------------Update individuals------------------------------
            if mod(iter,win_size)==1
                Prob_Oper=Experience_Oper/(sum(Experience_Oper)+realmin);
            end

            % generate new populationx
            index=RouletteWheelSelection(Prob_Oper);
            old_fitx=fitx(1);
            if index<=2
                for i=1:3:n_Ants
                    [ell, NewAnt(i:i+2,:), Nfitx(i:i+2)]= MultiOperator(I_fno, x, fitx, swarm_size,Dimension, Probability, index, flag,zeta,xmax, xmin);
                    if index==1
                        for ii=1:3
                            % success-based random-walk selection, if not better,
                             % switch the random-walk strategy
                            if Nfitx(i+ii-1)>fitx(ell(ii)) 
                                flag(ell(ii))=1-flag(ell(ii));
                            end
                        end 
                    end
                end
            else  
                for i=1:n_Ants
                    [ell, NewAnt(i,:), Nfitx(i)]= MultiOperator(I_fno, x, fitx, swarm_size,Dimension, Probability, index, flag,zeta,xmax, xmin);
                    % success-based random-walk selection, if not better,
                    % switch the random-walk strategy
                    if Nfitx(i)>fitx(ell) 
                        flag(ell)=1-flag(ell);
                    end
                end
            end
           
        %% ---------------------Evaluation----------------------------------------      
            allSwarm=[x; NewAnt];
            allFitnessValue= [fitx Nfitx];

            % sort
            [allFitnessValue, SortIndex]=sort(allFitnessValue);
            allSwarm=allSwarm(SortIndex,:);
            
            current_eval=current_eval+n_Ants;
            EvBestFitness(time, current_eval-n_Ants+1:current_eval)=allFitnessValue(1); 
            
            % record the number without impovement
            if fitx(1)<=allFitnessValue(1)
                NoImprove=NoImprove+1;
            else
                NoImprove=0;
            end
            
            %remove duplicates
            old_current_eval=current_eval;
            [allSwarm,allFitnessValue,current_eval] = hanDuplicate(allSwarm,allFitnessValue,xmin, xmax, I_fno,current_eval);
            EvBestFitness(time, old_current_eval+1:current_eval)=allFitnessValue(1);
            
            x=allSwarm(1:swarm_size,:);
            fitx=allFitnessValue(1:swarm_size);
            if NoImprove<=n_win*win_size
                Experience_Oper(index)=Experience_Oper(index)+abs((old_fitx-fitx(1)))/(abs(old_fitx)+realmin);
            else
                Experience_Oper=ones(1, n_Oper);
                Prob_Oper=Experience_Oper/(sum(Experience_Oper)+realmin);
            end
 
            % OPTIONAL: Local search for a randomly-chosen archive solution
            if rand < 0.1 
                [x, fitx, EvBestFitness(time,:), current_eval] = LocalSearch(Probability, x, fitx, max_eval, current_eval, swarm_size, Dimension,  EvBestFitness(time,:),I_fno,xmin,xmax);
            end
        end
        bestInd_data(time,:)=x(1,:);
        bestFitness(time)=fitx(1);
        FEvBestFitness(time,:)=EvBestFitness(time, 1:max_eval);
        for l=1:max_eval-1
            if isnan(FEvBestFitness(time, l))
                FEvBestFitness(time, l)=10^10;
            end
            if FEvBestFitness(time, l)<FEvBestFitness(time, l+1)
                FEvBestFitness(time, l+1)=FEvBestFitness(time, l);
            end
        end
        
        disp(['iteration = ', num2str(iter), ' best fitness = ', num2str(fitx(1))]);                              
    end
    data_performance.wall_clock_time=toc;
    data_performance.EvBestFitness=FEvBestFitness;
    data_performance.bestInd_data=bestInd_data;
    data_performance.bestFitness=bestFitness;
end




