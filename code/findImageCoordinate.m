function [X,Y] = findImageCoordinate(disparityMappp,x,z)
    
    d_t = floor(960*0.54/z);
    pos=1;
    xt = 10*x;
    for i=2:374
        
        if(disparityMappp(375-i,xt) == (d_t) || disparityMappp(375-i,xt) == (d_t-1)...
                || disparityMappp(375-i,xt) == (d_t-2) || disparityMappp(375-i,xt) == (d_t-3)...
                || disparityMappp(375-i,xt) == (d_t-4) || disparityMappp(375-i,xt) == (d_t-5)...
                || disparityMappp(375-i,xt) == (d_t+1) || disparityMappp(375-i,xt) == (d_t+2)...
                || disparityMappp(375-i,xt) == (d_t+3) || disparityMappp(375-i,xt) == (d_t+4)...
                || disparityMappp(375-i,xt) == (d_t+5) || disparityMappp(375-i,xt) == (d_t+6)...
                )
            pos = i;
            break;
        end
    end
    X = xt;
    Y = pos;    
end
