function [S, f, run, nfe] = LocalSearch(Probability, S,f, FEmax, nfe, k, n, run,I_fno, xMin, xMax)
   % Choose a solution according to probability of selection
    
    rand_ind =RouletteWheelSelection(Probability);
    %rand_ind = RW(option,f, nfe, FEmax);

    % Interior point method (Matlab Optimization toolbox)
    % See http://fr.mathworks.com/help/optim/ug/constrained-nonlinear-optimization-algorithms.html


   % MaxFunEvals=min(FEmax - nfe, round(2*FEmax/k)); % Empirical used in APS9
   MaxFunEvals=min(FEmax-nfe,3*k*n); % Empirical used in APS11

    if MaxFunEvals > 0

      %  options = optimoptions(@fmincon,'MaxFunEvals',MaxFunEvals,'Display','off');

        TolCon=0;  TolFun=0;
        options = optimoptions(@fmincon,'MaxFunEvals',MaxFunEvals,...
        'Display','off','Algorithm', 'interior-point', ...
        'TolCon',TolCon, 'TolFun',TolFun);

        ff = @(x)fnceval(x, I_fno); 

        %old_nfe = nfe;
        oldf=f;
        [S(rand_ind,:),f(rand_ind),~,output] = fmincon(ff,S(rand_ind,:),[],[],[],[],xMin,xMax,[],options);
        if output.funcCount>MaxFunEvals
           output.funcCount=MaxFunEvals;
        end
        run(nfe+1:nfe+output.funcCount) = min(f);

        nfe = nfe + output.funcCount;	

        % Sort the archive based on f
        [f, indecies] = sort(f);
        S = S(indecies,:);

        % Determine the diversity of the archive    
        %prum = mean(S);
        %prum_mat=repmat(prum,k,1);
        %diam2 = (sum( sum( (S-prum_mat).*(S-prum_mat) ) ) / k);
        %diversity = sqrt(diam2);
        %d_run(old_nfe+1:nfe) = diversity;   
    end
    
end










