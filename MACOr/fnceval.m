function fitness = fnceval (x, index)
        if index <9
            fitness=bench_func(x,index);            
        end
        if index == 9
            [fitness penalty rate_d]= cost_fn(x);
        end
        if index == 10
            null = [50, 120];
            phi_desired= 180 ;
            distance= 0.5;
            [y sllreturn bwfn]= antennafunccircular(x, null, phi_desired, distance);
            fitness=y;
        end
        if index==11
            [fitness, funccount]= MY_FUNCTION_10_1(x);
        end
        if index==12
            [fitness, funccount]= MY_FUNCTION_10_2(x);
        end
        if index==13
            [fitness, funccount ]= MY_FUNCTION_11_1(x);
        end
        if index==14
            [fitness, funccount]= MY_FUNCTION_11_2(x);
        end
        if index==15
            [fitness, funccount]= MY_FUNCTION_11_3(x);
        end
        if index==16
            [fitness, funccount]= MY_FUNCTION_11_4(x);
        end
        if index==17
            [fitness, funccount ]= MY_FUNCTION_11_5(x);
        end
        if index==18
            [fitness, funccount]= MY_FUNCTION_12_1(x);
        end
        if index==19
            [fitness, funccount]= MY_FUNCTION_12_2(x);
        end
        if index==20
            [fitness, funccount ]= MY_FUNCTION_12_3(x);
        end
        if index == 21
            a=load('messengerfull.mat');
            fitness =messengerfull(x,a.MGADSMproblem);
        end
        if index == 22
            a=load('cassini2.mat');
            fitness =cassini2(x,a.MGADSMproblem);
        end
        
        if isnan(fitness) 
            fitness=10^10;
        end
end