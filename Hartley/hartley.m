clear;

% 1 dimensional discrete hartley transform
% for wav files
function retVal = fht_shader (data)
    retVal = zeros( size(data));
    [n sz] = size(data);
    for k = 1:sz
        ret = 0 ;
        start = k-1;
        for n = 1:sz
            val = data( n );
            angle = (2 * 3.141592654 / sz) * (n-1) * start;
            mcos = cos(angle);
            msin = sin(angle);
            ret += (mcos + msin) * val;
        end
        retVal(k)=ret;
    end
    retVal /= sqrt(sz);
endfunction

% 2 dimensional discrete hartley transform
% for images
function retVal = fht_shader2 (data)
    retVal = zeros( size(data) );
    [w h] = size(data);
    for x = 1:w
        retVal(x,:) = dht_shader(data(x,:));
    end
    retVal = retVal';
    for y = 1:h
        retVal(y,:) = dht_shader(retVal(y,:));
    end
    retVal = retVal';
endfunction

% 1 dimentional fast hartley transform
% fht( data )

% 2 dimensional fast hartley transform
function retVal = fht2 (data)
    [w h] = size(data);
    retVal = (fht( data ))';
    retVal = (fht(retVal))';
    retVal /= sqrt(w*h);
endfunction


% ---- example program: ----

data = wavread("sinus.wav");

data_transformed = fht(data);

subplot(1,2,1);
plot(data(1:200));
title ("1. Original");

subplot(1,2,2);
plot( data_transformed(850:900) );
title ("2. Transformed");

% ---- example program: ----