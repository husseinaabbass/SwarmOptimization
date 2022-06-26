function [ell,NewAnt,Nfitx]= MultiOperator(I_fno, x, fitx, PopSize, Dimension, Probability, index, flag,zeta,xmax, xmin)
switch index
    case 1
        if I_fno > 16 && I_fno <= 20
           beta = normrnd(0.5,0.3);  %%% generate beta = Gaussian number, with mean=0.5 and standard deviation=0.3.%%%%
        else
           beta = normrnd(0.7,0.1);  %%% generate beta = Gaussian number, with mean=0.7 and standard deviation=0.1.%%%%
        end

        ell(1)=RouletteWheelSelection(Probability);
        ell(2)=RouletteWheelSelection(Probability);
        ell(3)=RouletteWheelSelection(Probability);

        ell=sort(ell);
        %%% Check the similarity between all selected individuals
        if (ell(1)== ell(2))
            while (ell(2)== ell(1)) ||(ell(2)== ell(3))
                ell(2)=randi(PopSize);
            end
            ell= sort(ell);
        end
        if (ell(1)== ell(3)) 
            while (ell(3)== ell(1)) ||(ell(3)== ell(2))
                ell(3)= randi(PopSize);%
            end
            ell= sort(ell);
        end
        if (ell(2)== ell(3))
            while (ell(3)== ell(1)) ||(ell(3)== ell(2))
                ell(3)= randi(PopSize);
            end
            ell= sort(ell);
        end
        
        Nsolution(1,:)=x(ell(1),:)+beta.*(x(ell(2),:)-x(ell(3),:));
        Nsolution(2,:)=x(ell(2),:)+beta.*(x(ell(3),:)-x(ell(1),:));
        Nsolution(3,:)=x(ell(3),:)+beta.*(x(ell(1),:)-x(ell(2),:));
        for ii=1:3
            StandardDeviation(ii,:)=zeta.*sum(abs(Nsolution(ii,:)-x))./(PopSize-1);  % Dimension *PopSize
            Step=StepSelection(ell(ii),flag, Dimension);%levy(1, Dimension);  % (1, Dimension)  
            NewAnt(ii,:)=Nsolution(ii,:)+StandardDeviation(ii,:).*Step;
            [NewAnt(ii,:), Dimension, xmax, xmin] = han_boun (NewAnt(ii,:), Dimension, xmax, xmin, I_fno,1);
            Nfitx(ii)=fnceval(NewAnt(ii,:), I_fno);
        end

    case 2  % GAMPC
        if I_fno > 16 && I_fno <= 20
           beta = normrnd(0.5,0.3);  %%% generate beta = Gaussian number, with mean=0.5 and standard deviation=0.3.%%%%
        else
           beta = normrnd(0.7,0.1);  %%% generate beta = Gaussian number, with mean=0.7 and standard deviation=0.1.%%%%
        end

        ell(1)=TournamentSelection(fitx, 2);
        ell(2)=TournamentSelection(fitx, 2);
        ell(3)=TournamentSelection(fitx, 2);

        ell=sort(ell);
        %%% Check the similarity between all selected individuals
        if (ell(1)== ell(2))
            while (ell(2)== ell(1)) ||(ell(2)== ell(3))
                ell(2)=randi(PopSize);
            end
            ell= sort(ell);
        end
        if (ell(1)== ell(3)) 
            while (ell(3)== ell(1)) ||(ell(3)== ell(2))
                ell(3)= randi(PopSize);%
            end
            ell= sort(ell);
        end
        if (ell(2)== ell(3))
            while (ell(3)== ell(1)) ||(ell(3)== ell(2))
                ell(3)= randi(PopSize);
            end
            ell= sort(ell);
        end
        
        Nsolution(1,:)=x(ell(1),:)+beta.*(x(ell(2),:)-x(ell(3),:));
        Nsolution(2,:)=x(ell(2),:)+beta.*(x(ell(3),:)-x(ell(1),:));
        Nsolution(3,:)=x(ell(3),:)+beta.*(x(ell(1),:)-x(ell(2),:));
        for ii=1:3
            NewAnt(ii,:)=Nsolution(ii,:);
            for jj=1:Dimension
                if rand<0.1
                    pos=randi(ceil(PopSize/2));
                    NewAnt(ii,jj)=x(pos,jj);
                end
            end
            
           [NewAnt(ii,:), Dimension, xmax, xmin] = han_boun (NewAnt(ii,:), Dimension, xmax, xmin, I_fno,1);
            Nfitx(ii)=fnceval(NewAnt(ii,:), I_fno);
        end

    case 3   % DE operator (ACO6)
            ell=RouletteWheelSelection(Probability);
            ell1=RouletteWheelSelection(Probability);
            Nsolution=x(ell,:)+rand.*(x(ell,:)-x(ell1,:));
            StandardDeviation=zeta.*sum(abs(Nsolution-x))./(PopSize-1);  % Dimension *PopSize
            Step=StepSelection(ell,flag, Dimension);
            NewAnt=Nsolution+StandardDeviation.*Step;
            [NewAnt, Dimension, xmax, xmin] = han_boun (NewAnt, Dimension, xmax, xmin, I_fno,1);
            Nfitx=fnceval(NewAnt, I_fno);
        
     case 4   % average ell (ACO1)
        ell=RouletteWheelSelection(Probability);
        ell1=RouletteWheelSelection(Probability);
        Nsolution=(x(ell, :)+x(ell1, :))/2;
        StandardDeviation=zeta.*sum(abs(Nsolution-x))./(PopSize-1);  % Dimension *PopSize
        Step=StepSelection(ell,flag, Dimension);
        NewAnt=Nsolution+StandardDeviation.*Step;    
       [NewAnt, Dimension, xmax, xmin] = han_boun (NewAnt, Dimension, xmax, xmin, I_fno,1);
        Nfitx=fnceval(NewAnt, I_fno);    
        
end
end

function Step=StepSelection(ell,flag, Dimension)
     if flag(ell)==0   % Gaussian
        Step=randn(1, Dimension);    
     else    % levy flight  
        Step=levy(1, Dimension);  % (1, Dimension)       
    end
end

