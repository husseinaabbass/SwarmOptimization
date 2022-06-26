function [n xmax xmin] = define_boundaries (n, xmax, xmin, I_fno)
switch I_fno;
    %%%%%%%%%%%%%%%%% Problem 1 (FM)%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    case 1
        n = 6; 
        
        %%%%%% Define the Boundaries
        for j=1 : n
            xmax (j) =6.35;
            xmin(j) =-6.4;
        end
        
          %%%%%%%%%%%%%%%%% Problem 2 (Lennard-Jones)%%%%%%%%%%%%%%%%%%%%%
    case 2
        n = 3 * 10;
        for j=1 : n
            if (j==1)
                xmax (j) =4;
                xmin(j) =0;
            else if (j==2)
                    xmax (j) =4;
                    xmin(j) =0;
                else if (j==3)
                        xmax (j) = pi;
                        xmin(j) =0;
                    else
                        xmax (j) =(4 + (1/4) * floor((j-4)/3));
                        xmin(j) =(-4 - (1/4) * floor((j-4)/3));
                        
                    end
                end
            end
        end
        
             %%%%%%%%%%%%%%%%% Problem 3 (The Bifunctional Catalyst Blend Optimal Control)%%%%%%%%%%%%%%%%%%%%%
    case 3
        n = 1;
        for j=1 : n
            
            xmax (j) =0.9;
            xmin(j) =0.6;
            
        end
        
        
                 %%%%%%%%%%%%%%%%% Problem 3 (Stirred Tank Reactor)%%%%%%%%%%%%%%%%%%%%%
    case 4
        n = 1;
        for j=1 : n
            
            xmax (j) =5;
            xmin(j) =0;
            
        end
        
                     %%%%%%%%%%%%%%%%% Problem 5 (Si (B))%%%%%%%%%%%%%%%%%%%%%
    case 5
        n = 3 * 10;
        for j=1 : n
            if (j==1)
                xmax (j) =4;
                xmin(j) =0;
            else if (j==2)
                    xmax (j) =4;
                    xmin(j) =0;
                else if (j==3)
                        xmax (j) = pi;
                        xmin(j) =0;
                    else
                        xmax (j) =(4 + (1/4) * floor((j-4)/3));
                        xmin(j) =(-4 - (1/4) * floor((j-4)/3));
                        
                    end
                end
            end
        end
        
                        %%%%%%%%%%%%%%%%% Problem 6 (Si(C))%%%%%%%%%%%%%%%%%%%%%
    case 6
        n = 3 * 10;
        for j=1 : n
            if (j==1)
                xmax (j) =4;
                xmin(j) =0;
            else if (j==2)
                    xmax (j) =4;
                    xmin(j) =0;
                else if (j==3)
                        xmax (j) = pi;
                        xmin(j) =0;
                    else
                        xmax (j) =(4 + (1/4) * floor((j-4)/3));
                        xmin(j) =(-4 - (1/4) * floor((j-4)/3));
                        
                    end
                end
            end
        end
        
        
       %%%%%%%%%%%%%%%%% Problem 7 (Spread Spectrum)%%%%%%%%%%%%%%%%%%%%%
    case 7
        n = 20;
        for j=1 : n
            xmax (j) =2*pi;
            xmin(j) =0;
            
        end
        
       %%%%%%%%%%%%%%%%% Problem 8 (TNEP)%%%%%%%%%%%%%%%%%%%%%
    case 8
        n = 7;
        for j=1 : n
            
            xmax (j) =15;
            xmin(j) =0;
            
        end
        
              
       %%%%%%%%%%%%%%%%% Problem 8 (TNEP)%%%%%%%%%%%%%%%%%%%%%
    case 9
        initial_flag=0;
        EBEinputfile;

        Pg=(bus_spec(:,7))/100;
        Pd=(bus_spec(:,5))/100;
        
        g=find(Pg>0);
        d=find(Pd>0);
        
        %%%%%%%%% define BT   %%%%%%%%%%%%%%%%%%%%%
        BT=zeros(length(g),length(d));
        
        BT(1,4)=5;BT(1,5)=10;BT(1,6)=5;
        BT(2,3)=5;
        BT(3,21)=2.5;
        BT(4,21)=2.5;BT(4,16)=15;
        BT(5,12)=2.5;BT(6,8)=2.5;
        
        BT=BT/100;
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        GD_max=zeros(length(g),length(d));
        for i=1:length(g)
            for j=1:length(d)
                GD_max(i,j)=min(Pg(g(i))-BT(i,j),Pd(d(j))-BT(i,j));
                GD_min (i,j) = 0 ;
            end
        end
        
        n = 6*21; 
        
        %%% Define Boundaries
        c=0;
        
        for i=1:length(g)
            for j=1:length(d)
                c=c+1;
                xmin(c)=0;
                xmax(c)=GD_max(i,j);
            end
        end
        
          %%%%%%%%%%%%%%%%% Problem 10%%%%%%%%%%%%%%%%%%%%%
    case 10
        n = 12;
        
        for j=1 : n
            if (j<=6)
                xmin(j) = 0.2;
                xmax (j) =1;
            else
                xmin(j) = -180;
                xmax (j) =180;
            end
        end

            %%%%%%%%%%%%%%%%% Problem 10_1 %%%%%%%%%%%%%%%%%%%%%
    case 11
        n = 5 * 24; %%%% Number of decision variables
        
        Pmin = [10,20,30,40,50;];
        Pmax = [75,125,175,250,300;];
        Lower_Limit = repmat(Pmin,1,24);
        Upper_Limit= repmat(Pmax,1,24);
        
        for j=1:n
            xmax(j)=Upper_Limit (j);
            xmin(j)=Lower_Limit(j);
        end
        
               %%%%%%%%%%%%%%%%% Problem 10_2 %%%%%%%%%%%%%%%%%%%%%
    case 12
        n = 10 * 24; %%%% Number of decision variables
        
        Pmin = [150,135,73,60,73,57,20,47,20,55];
        Pmax = [470,460,340,300,243,160,130,120,80,55];
        Lower_Limit = repmat(Pmin,1,24);
        Upper_Limit= repmat(Pmax,1,24);
        
        for j=1:n
            xmax(j)=Upper_Limit (j);
            xmin(j)=Lower_Limit(j);
        end
            %%%%%%%%%%%%%%%%% Problem 11_1 %%%%%%%%%%%%%%%%%%%%%
    case 13
        n = 6; %%%% Number of decision variables
        bounds=[100 500;50 200;80 300;50 150;50 200;50 120;];
        
        for i=1:n
            xmin(i)= bounds(i,1);
            xmax(i)= bounds(i,2);
        end
             %%%%%%%%%%%%%%%%% Problem 11_2 %%%%%%%%%%%%%%%%%%%%%
    case 14
        n = 13; %%%% Number of decision variables
        bounds=[0,680;0,360;0,360;60,180;60,180;60,180;60,180;60,180;60,180;40,120;40,120;55,120;55,120;];
        
        for i=1:n
            xmin(i)= bounds(i,1);
            xmax(i)= bounds(i,2);
        end
             %%%%%%%%%%%%%%%%% Problem 11_3 %%%%%%%%%%%%%%%%%%%%%
    case 15
        n = 15; %%%% Number of decision variables
        bounds=[150,455;150,455;20,130;20,130;150,470;135,460;135,465;60,300;25,162;25,160;20,80;20,80;25,85;15,55;15,55;];
        
        for i=1:n
            xmin(i)= bounds(i,1);
            xmax(i)= bounds(i,2);
        end
             %%%%%%%%%%%%%%%%% Problem 11_4 %%%%%%%%%%%%%%%%%%%%%
    case 16
        n = 40; %%%% Number of decision variables
        bounds=[36,114;36,114;60,120;80,190;47,97;68,140;110,300;135,300;135,300;130,300;94,375;94,375;125,500;125,500;125,500;125,500;220,500;220,500;242,550;242,550;254,550;254,550;254,550;254,550;254,550;254,550;10,150;10,150;10,150;47,97;60,190;60,190;60,190;90,200;90,200;90,200;25,110;25,110;25,110;242,550;]; 

        
        for i=1:n
            xmin(i)= bounds(i,1);
            xmax(i)= bounds(i,2);
        end
             %%%%%%%%%%%%%%%%% Problem 11_5 %%%%%%%%%%%%%%%%%%%%%
    case 17
        n = 140; %%%% Number of decision variables
        bounds=[71,119;120,189;125,190;125,190;90,190;90,190;280,490;280,490;260,496;260,496;260,496;260,496;260,506;260,509;260,506;260,505;260,506;260,506;260,505;260,505;260,505;260,505;260,505;260,505;280,537;280,537;280,549;280,549;260,501;260,501;260,506;260,506;260,506;260,506;260,500;260,500;120,241;120,241;423,774;423,769;3,19;3,28;160,250;160,250;160,250;160,250;160,250;160,250;160,250;160,250;165,504;165,504;165,504;165,504;180,471;180,561;103,341;198,617;100,312;153,471;163,500;95,302;160,511;160,511;196,490;196,490;196,490;196,490;130,432;130,432;137,455;137,455;195,541;175,536;175,540;175,538;175,540;330,574;160,531;160,531;200,542;56,132;115,245;115,245;115,245;207,307;207,307;175,345;175,345;175,345;175,345;360,580;415,645;795,984;795,978;578,682;615,720;612,718;612,720;758,964;755,958;750,1007;750,1006;713,1013;718,1020;791,954;786,952;795,1006;795,1013;795,1021;795,1015;94,203;94,203;94,203;244,379;244,379;244,379;95,190;95,189;116,194;175,321;2,19;4,59;15,83;9,53;12,37;10,34;112,373;4,20;5,38;5,19;50,98;5,10;42,74;42,74;41,105;17,51;7,19;7,19;26,40;];
        
        for i=1:n
            xmin(i)= bounds(i,1);
            xmax(i)= bounds(i,2);
        end
        
                  %%%%%%%%%%%%%%%%% Problem 12_1 %%%%%%%%%%%%%%%%%%%%%
    case 18
        n = 96; %%%% Number of decision variables
        
        Qmin = [5 6 10 13]; Qmax = [15 15 30 25];
        Lower_Limit = repmat(Qmin,1,24);
        Upper_Limit = repmat(Qmax,1,24);
        for i=1:n
            xmin (i)= Lower_Limit(i);
            xmax (i)= Upper_Limit(i);
        end
        
                   %%%%%%%%%%%%%%%%% Problem 12_2 %%%%%%%%%%%%%%%%%%%%%
    case 19
        n = 96; %%%% Number of decision variables
        
        Qmin = [5 6 10 13]; Qmax = [15 15 30 25];
        Lower_Limit = repmat(Qmin,1,24);
        Upper_Limit = repmat(Qmax,1,24);
        for i=1:n
            xmin (i)= Lower_Limit(i);
            xmax (i)= Upper_Limit(i);
        end
        
                   %%%%%%%%%%%%%%%%% Problem 12_3 %%%%%%%%%%%%%%%%%%%%%
    case 20
        n = 96; %%%% Number of decision variables
        
        Qmin = [5 6 10 13]; Qmax = [15 15 30 25];
        Lower_Limit = repmat(Qmin,1,24);
        Upper_Limit = repmat(Qmax,1,24);
        for i=1:n
            xmin (i)= Lower_Limit(i);
            xmax (i)= Upper_Limit(i);
        end
        
                       %%%%%%%%%%%%%%%%% Problem  (Messenger Full) %%%%%%%%%%%%%%%%%%%%%
    case 21
        n = 26; %%%% Number of decision variables
        
        %%% Define Boundaries
        xmin =[1900 2.5  0 0 100 100 100 100 100 100 0.01 0.01 0.01 0.01  0.01 0.01  1.1 1.1 1.05 1.05 1.05 -pi -pi -pi -pi -pi];
        xmax= [2300 4.05 1 1 500 500 500 500 500 600 0.99 0.99 0.99 0.99  0.99 0.99 6 6 6 6 6 pi pi pi pi pi];


        
        
                       %%%%%%%%%%%%%%%%% Problem Cassini2 %%%%%%%%%%%%%%%%%%%%%
    case 22
        n = 22; %%%% Number of decision variables
        
        %%% Define Boundaries
        xmin = [-1000 3 0 0 100 100 30 400 400 0.01 0.01 0.01 0.01 0.01 1.05 1.05 1.15 1.7 -pi -pi -pi -pi];
        xmax = [ 0 5 1 1 400 500 300 1600 2200 0.9 0.9 0.9 0.9 0.9 6 6 6.5 291 pi pi pi pi];


end
end