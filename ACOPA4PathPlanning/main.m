
% Version 23/Dec/2021
% The codes for the paper 
% Liu, J., Anavatti, S., Garratt, M., & Abbass, H. A. (2022). Modified continuous Ant Colony Optimisation for multiple Unmanned Ground Vehicle path planning. Expert Systems with Applications, 196, 116605.
% Copyright reserved 
% Please cite this paper if you use this code.
% Contact: (jing.liu5@unsw.edu.au; h.abbass@unsw.edu.au)

for Dimension=20   % the number of waypoints  20 30 40 60
    for Task=1:4   % 1:12
        %% Path plannning
        mainPathPlanning (Dimension, Task)

        %% Multi-agent coordination
        mainMultiagentCoordination (Dimension, Task)

        %% plot
        pathplot (Dimension, Task)
    end
end