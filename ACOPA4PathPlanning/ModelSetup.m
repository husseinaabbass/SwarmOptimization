function  Infor= ModelSetup(TaskInfor, ThreatInfor,ObstacleInfor,MovingObstacleInfor, Num_WayPoints)
      
      [TaskNumber,~] =size(TaskInfor); 
      BoundInfor=TaskInfor(1,6);
      Bound=repmat([0, BoundInfor],Num_WayPoints,1);
      
      StartPoint=TaskInfor(:,1:2);
      TargetPoint=TaskInfor(:,3:4);
      %d=dist(StartPoint, TargetPoint');
      
      k=1:Num_WayPoints;
      x=StartPoint(:,1)+k.*(TargetPoint(:,1)-StartPoint(:,1))/(1+Num_WayPoints);
      
      % calculate the itersection with obstacles of each verticle line
      [ObsNum,~]=size(ObstacleInfor);
      obsx=x; %[StartPoint(:,1) x TargetPoint(:, 1)];
      Tfeasible=TaskInfor(1,6)*ones(TaskNumber,Num_WayPoints, ObsNum+1, 2);
      feasNum=zeros(TaskNumber, Num_WayPoints);
      %TInfeasible=TaskInfor(1,6)*ones(TaskNumber,Num_WayPoints, ObsNum, 2);
      maxgap=ones(TaskNumber,Num_WayPoints+2, 1, 2);
      for ii=1:TaskNumber
      for i=1:Num_WayPoints % for each bin in x direction
         % intersectionNum=zeros(1,Num_WayPoints);
          infeasible=[];
          feasible=[];
          for j=1:ObsNum
              %syms yy
              a=ObstacleInfor(j,1);
              b=ObstacleInfor(j,2);
              r=ObstacleInfor(j,3);
              if abs(obsx(ii,i)-a)<r
                   obsy1=(r^2-(obsx(ii,i)-a)^2)^0.5+b;
                   obsy2=-(r^2-(obsx(ii,i)-a)^2)^0.5+b;
                   %eq=(obsx(i)-a)^2+(yy-b)^2==r^2;
                   %s=solve(eq,yy);
                   infeasible=[infeasible; obsy2 obsy1];
                   %intersectionNum(i)=intersectionNum(i)+1; 
              end
          end
          if ~isempty(infeasible)
              [~,sortindex]=sort(infeasible(:,1));
              sinfeasible=infeasible(sortindex,:);
              ssinfeasible=[0 reshape(sinfeasible',1,[]) TaskInfor(1,6)];

              %if sinfeasible(1,1)>0
              %   feasible=[feasible; 0 sinfeasible(1,1)];
              %end
              ifNum=length(ssinfeasible);
              for k=2:2:ifNum
                  if ssinfeasible(k)>ssinfeasible(k-1)
                     feasible=[feasible; ssinfeasible(k-1) ssinfeasible(k)];
                  end
              end
              gap=feasible(:,2)-feasible(:,1);
              [~, maxindex]=max(gap);
              maxgap(ii,i,1,:)=feasible(maxindex,:);
              [fNum,~]=size(feasible);
              Tfeasible(ii, i, 1:fNum,:)=feasible;
              feasNum(ii,i)=fNum;
              %[iNum,~]=size(sinfeasible);
              %TInfeasible(ii, i, 1:iNum,:)=sinfeasible;
          else
              Tfeasible(ii, i, 1,:)=[0 TaskInfor(1,6)];
              maxgap(ii,i,1,:)=[0 TaskInfor(1,6)];
              feasNum(ii,i)=1;
          end
      end
      end
      
      Infor.maxgap=maxgap;
      %Infor.Tinfeasible=TInfeasible;
      Infor.feasible=Tfeasible;
      Infor.feasibleNum=feasNum;
      
      Infor.Task=TaskInfor;
      Infor.Threat=ThreatInfor;
      Infor.Obstacle=ObstacleInfor;
      Infor.MObstacle=MovingObstacleInfor;
      Infor.x=x';   %Num_WayPoints,TaskNumber
      Infor.Bound=Bound;
      Infor.Num_WayPoints=Num_WayPoints;
      Infor.TaskNumber=TaskNumber;
      
      Infor.y=[];  % the variables (Num_WayPoints,TaskNumber)
      
end