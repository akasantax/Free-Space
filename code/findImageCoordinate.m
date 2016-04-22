function [X,Y] = findImageCoordinate(disparityMappp,x,z)
    
    d_t = floor(960*0.54/z);
    pos=1;
    xt = 10*x;
    for i=1:375
        
        if(disparityMappp(i,xt) == d_t || disparityMappp(i,xt-1) == d_t || disparityMappp(i,xt-2) == d_t)
            pos = i;
            break;
        end
    end
    X = xt;
    Y = pos;    
end
