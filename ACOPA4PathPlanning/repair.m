function pos=repair(pos,ModelInfor,AgentIndex,prob, flag_uniform)
      flag_uniform=0;
      AgentIndex=1;
      [Dimension, SwarmSize]=size(pos);
      %maxGap=ModelInfor.maxgap;
      feasNum=ModelInfor.feasibleNum; %(TaskNumber, Num_WayPoints);
      feasibleGap=ModelInfor.feasible;  %(TaskNumber,Num_WayPoints, ObsNum+1, 2);
      %flag=0;   %the position is infeasible
      %Infeasible=ModelInfor.Tinfeasible;
      Bound=ModelInfor.Bound;
      minrange=Bound(:,1);
      maxrange=Bound(:,2);
     
      
      for i=1:SwarmSize
          if rand<prob
              if flag_uniform==1
                 pos(:,i)=pos(:,i).*(maxrange-minrange)+minrange; % decoding
              end
              for j=1:Dimension
                  %num
                  flag=0;   %the position is infeasible
                  gap=abs(feasibleGap(AgentIndex, j, 1:feasNum(AgentIndex, j), 2)-feasibleGap(AgentIndex, j, 1:feasNum(AgentIndex, j), 1));
                  %p=gap./sum(gap);
                  ggap=zeros(1,feasNum(AgentIndex, j));
                  for k=1:feasNum(AgentIndex, j)
                      if pos(j,i)<=feasibleGap(AgentIndex, j, k, 2) && feasibleGap(AgentIndex, j, k, 1)<=pos(j,i)
                          flag=1;    % feasible                   
                      end
                      ggap(k)=gap(1,1,k);
                  end
                  if flag==0
                      index=RouletteWheelSelection(ggap);
                      pos(j,i)=feasibleGap(AgentIndex, j, index, 1)+(feasibleGap(AgentIndex, j, index, 2)-feasibleGap(AgentIndex, j, index, 1))*rand;
                      %pos(j,i)=maxGap(1)+(maxGap(2)-maxGap(1))*rand;
                  end
              end
              if flag_uniform==1
                 pos(:,i)=(pos(:,i)-minrange)./(maxrange-minrange) ;
              end
          end
      end
end