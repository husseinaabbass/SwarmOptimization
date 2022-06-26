% Based on the waypoints
% [Gbest, ETC]=SpeedControl(Gbest, ModelInfor, Num_WayPoints)
% Version 23/Dec/2021
% The codes for the paper 
% Liu, J., Anavatti, S., Garratt, M., & Abbass, H. A. (2022). Modified continuous Ant Colony Optimisation for multiple Unmanned Ground Vehicle path planning. Expert Systems with Applications, 196, 116605.
% Copyright reserved 
% Please cite this paper if you use this code.
% Contact: (jing.liu5@unsw.edu.au; h.abbass@unsw.edu.au)

function mainMultiagentCoordination (DimensionSet, TaskSet)
if nargin==0
    DimensionSet=20;   % the number of waypoints
    TaskSet=1;
end
%clc;
%clear;
SafetyDistance=1;
SafetyTime=1;
MaxSpeed=3;
MinSpeed=0.1;
TrialTimes=10;
%rng(123456789);
SwarmSize=20;    
MaximumFEs=4000;


AlgorithmName={'ACOPA'};
for Dimension=[ DimensionSet]  % 20 30 40 60 
    
    file_ETA=strcat('results/', 'NewSize',int2str(SwarmSize),'Dim',int2str(Dimension),'ETA.txt'); % record results 
    find_file_ETA=fopen(file_ETA,'a+');
    for ProblemIndex=[TaskSet]  % 1:12
        Num_WayPoints=Dimension;
        [Task, ~, ~,  ~ ]=EnvironmentInfor(ProblemIndex);
        [NTask,~]=size(Task);
        Num_Agent=NTask; % Problem dimension
        for AlgorithmIndex=[1]
            time=[];
            PathLength=[];
            %Path=[];
            FileName=strcat('results/', char(AlgorithmName(AlgorithmIndex)), 'Prob',int2str(ProblemIndex), 'Dim',int2str(Dimension),'Path.txt');   
            FindFile=fopen(FileName, 'r');
            Path=fscanf(FindFile,'%50f',[(Dimension+2),inf]);
            
            %insert waypoints  
            if Dimension ==20 ||  Dimension==30 
                N_InsertPoint=60/Dimension-1;
                
                NewPath=zeros((Dimension+2)+(Dimension+1)*N_InsertPoint, Num_Agent*2);
                for jj=1:Num_Agent
                    SNewPath=[];
                    for ii=1:Num_WayPoints+1
                        interval=(Path(ii+1,(jj-1)*2+1:jj*2)-Path(ii,(jj-1)*2+1:jj*2))./(N_InsertPoint+1);
                        SNewPath=[SNewPath; Path(ii,(jj-1)*2+1:jj*2)];
                        for tt=1:N_InsertPoint
                            SNewPath=[SNewPath; Path(ii,(jj-1)*2+1:jj*2)+interval*tt];
                        end
                    end
                    SNewPath=[SNewPath; Path(Num_WayPoints+2,(jj-1)*2+1:jj*2)];
                    NewPath(:,(jj-1)*2+1:jj*2)=SNewPath;
                end
                Path=NewPath;
                Num_WayPoints=(Dimension)+(Dimension+1)*N_InsertPoint;
            end
            
            if Dimension ==40
                N_InsertPoint=1;
                NewPath=zeros(63, Num_Agent*2);
                for jj=1:Num_Agent
                    SNewPath=[];
                    for ii=1:Num_WayPoints+1
                        interval=(Path(ii+1,(jj-1)*2+1:jj*2)-Path(ii,(jj-1)*2+1:jj*2))./(N_InsertPoint+1);
                        SNewPath=[SNewPath; Path(ii,(jj-1)*2+1:jj*2)];
                        if mod(ii,2)==1
                        SNewPath=[SNewPath; Path(ii,(jj-1)*2+1:jj*2)+interval];
                        end
                    end
                    SNewPath=[SNewPath; Path(Num_WayPoints+2,(jj-1)*2+1:jj*2)];
                    NewPath(:,(jj-1)*2+1:jj*2)=SNewPath;
                end
                Path=NewPath;
                Num_WayPoints=61;
            end
            
            dd=Path(2:Num_WayPoints+2,:)-Path(1:Num_WayPoints+1,:);   % Num_WayPoints+1
            for i=1:Num_Agent
                PathLength(:,i)=sum(dd(:,(i-1)*2+1:i*2).*dd(:,(i-1)*2+1:i*2),2).^0.5;
                %time(:,i)=PathLength(:,i)/Speed;  % record the time spending on each line segment beween two waypoints; Num_WayPoints+1
            end
            fclose(FindFile);    

           % optimisation
           for TrialIndex=1:TrialTimes
               InitPos=MinSpeed+rand(Num_Agent, SwarmSize).*(MaxSpeed-MinSpeed);
               [Gbest,GbestValue, GbestHistory]=ACO(Path, PathLength, Num_Agent, MaxSpeed, MinSpeed, InitPos, SafetyDistance, SafetyTime, SwarmSize, MaximumFEs, Num_WayPoints);
               %GbestValue
               %Gbest;
               ETA(1,TrialIndex)=GbestValue;
           end
           MeanETA=mean(ETA);
           StdETA=std(ETA);
           fprintf('%i  %s   %.4f  %.4f\r\n', ProblemIndex, char(AlgorithmName(AlgorithmIndex)), MeanETA, StdETA );
           fprintf(find_file_ETA, '%i  %s   %.4f  %.4f\r\n', ProblemIndex, char(AlgorithmName(AlgorithmIndex)), MeanETA, StdETA );
        end
    end
end
fclose ('all');
end