 function [i,j] = find_grid(x,z)

    % x can be between 1 and 1242 = resolution of x
    % z can be between 1 and 240 = the depth of z
    % a = {1,2....124}
    % b = {1,2,3,4....48}
    res_x = 10;
    res_z = 5;
    i = ceil(x/res_x);
    if(i == 125)
        i = 124;
    end
    j = ceil(z/res_z);
    if(j<1)
        j=1;
    end
 
 end
