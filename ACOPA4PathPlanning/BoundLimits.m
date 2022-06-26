% revised at 4.9.2018
% this is to control the bounds of variables
% Copyright reserved 
% Contact: (jing.liu5@student.adfa.edu.au)  

function X=BoundLimits(X,lower,upper)
   [D, S]=size(X);
   if length(lower)>1
      for i=1:S
           TX=X(:,i);
           TX(TX>upper)=upper(TX>upper);
           TX(TX<lower)=lower(TX<lower);
           X(:,i)=TX;
      end
   else
       X(X>upper)=upper;
       X(X<lower)=lower;
   end
end