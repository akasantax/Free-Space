function [X,Y] = findImageCoordinate(disparityMappp,x,z)
    
    dt = floor(960*0.54/z);
    pos=1;
    xt = 10*x;
    
    vals1 = [dt-1 dt-2 dt-3 dt-4 dt-5 dt-6 dt-7 dt-8 dt-9 ];
    vals2 = [dt+1 dt+2 dt+3 dt+4 dt+5 dt+6 dt+7 dt+8 dt+9 ];
     min_d = 10;
     size_val = size(vals1);
     
     for i=1:size_val(1,2)
         if(vals1(1,i) == min_d)
             vals1(1,i) = 20;
         end
         if(vals2(1,i) == min_d)
             vals2(1,i) = 20;
         end
     end
    
    
    for i=2:374
        disp_val = disparityMappp(375-i,xt);
        disp_val2 = disparityMappp(375-i,xt-1);
        disp_val3 = disparityMappp(375-i,xt+1);
        % one could use a loop here, I don't :P
        if(disp_val == vals1(1,1) || disp_val == vals1(1,2) || disp_val == vals1(1,3)...
                || disp_val == vals1(1,1) || disp_val == vals1(1,2) || disp_val == vals1(1,3)...
                || disp_val == vals1(1,4) || disp_val == vals1(1,5) || disp_val == vals1(1,6)...
                || disp_val == vals1(1,7) || disp_val == vals1(1,8) || disp_val == vals1(1,9)...
                || disp_val == vals2(1,1) || disp_val == vals2(1,2) || disp_val == vals2(1,3)...
                || disp_val == vals2(1,4) || disp_val == vals2(1,5) || disp_val == vals2(1,6)...
                || disp_val == vals2(1,7) || disp_val == vals2(1,8) || disp_val == vals2(1,9)...
               ) 
            pos = i;
            break;
        end
    end
    X = xt;
    Y = pos;    
end
