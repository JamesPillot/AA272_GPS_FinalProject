function [R1,R2,R3,R4] = getCN0var(entry_index)
%GETCN0 Summary of this function goes here
%   Detailed explanation goes here
Sat1CN0 = sqrt(4)*randn(1,10001) + 35;
Sat1CN0(end-100:end) = sqrt(7)*randn(1,101) + 23;
Sat2CN0 = sqrt(7)*randn(1,10001) + 23;
Sat2CN0(6500:7500) = sqrt(10)*randn(1,1001) + 15;
Sat3CN0 = sqrt(4)*randn(1,10001) + 35;
Sat4CN0 = sqrt(7)*randn(1,10001) + 23;
Sat4CN0(7000:8000) = sqrt(10)*randn(1,1001) + 15;


CN01 = Sat1CN0(entry_index);
CN02 = Sat2CN0(entry_index);
CN03 = Sat3CN0(entry_index);
CN04 = Sat4CN0(entry_index);

R1 = CN02VAR(CN01);
R2 = CN02VAR(CN02);
R3 = CN02VAR(CN03);
R4 = CN02VAR(CN04);


    function [VAR] = CN02VAR(CN0)
        dBpoints = [6 40];
        VARpoints = [10 .1];
        VAR = interp1(dBpoints,VARpoints,CN0);
    end

end

