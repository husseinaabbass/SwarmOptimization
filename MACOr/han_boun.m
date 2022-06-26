function [x n xmax xmin] = han_boun (x, n, xmax, xmin, ~, i)
   % 11 12 14 15 16 20 different in the GAMPC coedes
   x(isnan(x))=Inf;

   for j=1: n
        if( x(i,j) <xmin (j))
            x(i,j)=   xmin (j) +rand*(xmax(j)-xmin(j));
        else
            if ( x(i,j)>xmax (j))
                x(i,j)=   xmin (j) +rand*(xmax(j)-xmin(j));
            end
        end
    end

end