function [muEst,mu] = EKFmod(x0,u,time,sat_positions,Q,R,params,fixed_variance)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    dt = params.delta_t;
    %Overview
    P0 = .01*eye(4);
    x0bar = x0 + chol(P0)*randn(length(x0),1); %
    timeSteps = floor(time/dt);
    mu_cur = x0bar;
    P_cur = P0;
    mu_true = x0;
    
    
    muEst = zeros(4,10001);
    mu = zeros(4,10001);
    
    muEst(:,1) = mu_cur;
    mu(:,1) = mu_true;
    
    for i = 2:timeSteps+1
        %Propagate True State
        fx = u(i, 1); 
        fy = u(i, 2); 
        x_dot = [mu_true(3); mu_true(4); fx/params.m; fy/params.m];
        process_noise = sqrt(Q)*randn(4,1);
        mu_true = mu_true + dt*x_dot + process_noise;
         %Iterate the filter
         
         if fixed_variance == 1
             [r1, r2, r3 ,r4] = getCN0var(i);
             [~,drop] = max([r1, r2, r3 ,r4]);
             R = diag([3 3 3]);
         else
             [r1, r2, r3 ,r4] = getCN0var(i);
             R = diag([r1,r2,r3,r4]);
            [~,drop] = max([r1, r2, r3 ,r4]);
            if drop == 1
               R = diag([r2,r3,r4]);
            elseif drop == 2
                R = diag([r1,r3,r4]);
            elseif drop == 3
                R = diag([r1,r2,r4]);
            elseif drop == 4
                R = diag([r1,r2,r3]);
            end
         end
        [mu_pred,P_pred] = ekf_predict(mu_cur,u(i,:),P_cur,Q,dt,params.m);
        [mu_update,P_update] = ekf_update(mu_pred,P_pred,mu_true,R,sat_positions,i,fixed_variance,drop);
        %Update and Store Results
        mu_cur = mu_update;
        P_cur = P_update;
        muEst(:,i) = mu_update;
        mu(:,i) = mu_true;
        
    end




    %Problme 3f
    function [Ht] = get_H(bec,mu_cur,drop)
        bec1 = bec(1,:);
        bec2 = bec(2,:);
        bec3 = bec(3,:);
        bec4 = bec(4,:);
        
        if drop == 1
            Ht = ...
            [-(bec2(1)-mu_cur(1))/sqrt((bec2(1)-mu_cur(1))^2 + (bec2(2)-mu_cur(2))^2), -(bec2(2)-mu_cur(2))/sqrt((bec2(1)-mu_cur(1))^2 + (bec2(2)-mu_cur(2))^2),0,0;...
            -(bec3(1)-mu_cur(1))/sqrt((bec3(1)-mu_cur(1))^2 + (bec3(2)-mu_cur(2))^2), -(bec3(2)-mu_cur(2))/sqrt((bec3(1)-mu_cur(1))^2 + (bec3(2)-mu_cur(2))^2),0,0;...
            -(bec4(1)-mu_cur(1))/sqrt((bec4(1)-mu_cur(1))^2 + (bec4(2)-mu_cur(2))^2), -(bec4(2)-mu_cur(2))/sqrt((bec4(1)-mu_cur(1))^2 + (bec4(2)-mu_cur(2))^2),0,0];
        elseif drop == 2
            Ht = ...
            [-(bec1(1)-mu_cur(1))/sqrt((bec1(1)-mu_cur(1))^2 + (bec1(2)-mu_cur(2))^2), -(bec1(2)-mu_cur(2))/sqrt((bec1(1)-mu_cur(1))^2 + (bec1(2)-mu_cur(2))^2),0,0;...
            -(bec3(1)-mu_cur(1))/sqrt((bec3(1)-mu_cur(1))^2 + (bec3(2)-mu_cur(2))^2), -(bec3(2)-mu_cur(2))/sqrt((bec3(1)-mu_cur(1))^2 + (bec3(2)-mu_cur(2))^2),0,0;...
            -(bec4(1)-mu_cur(1))/sqrt((bec4(1)-mu_cur(1))^2 + (bec4(2)-mu_cur(2))^2), -(bec4(2)-mu_cur(2))/sqrt((bec4(1)-mu_cur(1))^2 + (bec4(2)-mu_cur(2))^2),0,0];
           
        elseif drop == 3
            Ht = ...
            [-(bec1(1)-mu_cur(1))/sqrt((bec1(1)-mu_cur(1))^2 + (bec1(2)-mu_cur(2))^2), -(bec1(2)-mu_cur(2))/sqrt((bec1(1)-mu_cur(1))^2 + (bec1(2)-mu_cur(2))^2),0,0;...
            -(bec2(1)-mu_cur(1))/sqrt((bec2(1)-mu_cur(1))^2 + (bec2(2)-mu_cur(2))^2), -(bec2(2)-mu_cur(2))/sqrt((bec2(1)-mu_cur(1))^2 + (bec2(2)-mu_cur(2))^2),0,0;...
            -(bec4(1)-mu_cur(1))/sqrt((bec4(1)-mu_cur(1))^2 + (bec4(2)-mu_cur(2))^2), -(bec4(2)-mu_cur(2))/sqrt((bec4(1)-mu_cur(1))^2 + (bec4(2)-mu_cur(2))^2),0,0];
        elseif drop == 4
            Ht = ...
            [-(bec1(1)-mu_cur(1))/sqrt((bec1(1)-mu_cur(1))^2 + (bec1(2)-mu_cur(2))^2), -(bec1(2)-mu_cur(2))/sqrt((bec1(1)-mu_cur(1))^2 + (bec1(2)-mu_cur(2))^2),0,0;...
            -(bec2(1)-mu_cur(1))/sqrt((bec2(1)-mu_cur(1))^2 + (bec2(2)-mu_cur(2))^2), -(bec2(2)-mu_cur(2))/sqrt((bec2(1)-mu_cur(1))^2 + (bec2(2)-mu_cur(2))^2),0,0;...
            -(bec3(1)-mu_cur(1))/sqrt((bec3(1)-mu_cur(1))^2 + (bec3(2)-mu_cur(2))^2), -(bec3(2)-mu_cur(2))/sqrt((bec3(1)-mu_cur(1))^2 + (bec3(2)-mu_cur(2))^2),0,0];
       end

    end


    %EKF Predict Step
    function [mu_pred,P_pred] = ekf_predict(mu_cur,u_cur,P_cur,Q,dt,m)
        xDOT = [mu_cur(3); mu_cur(4);  u_cur(1)/m; u_cur(2)/m];
        mu_pred = mu_cur + dt*xDOT;
        Ft = [0 0 1 0;0 0 0 1;0 0 0 0;0 0 0 0];
        P_pred = Ft*P_cur*Ft' + Q;
    end

    %EKF Update Step
    function [mu_update,P_update] = ekf_update(mu_pred,P_pred,mu_cur,R,sat_pos,entry_index,fixed_variance,drop)
        Ht = get_H(sat_pos,mu_pred,drop); %Get H for predicted mu
        zt = simulate_noisy_meas(mu_cur,sat_pos,R,drop,fixed_variance); %Simulate a noisy measurement
        ytbar = zt -  meas_mdl(mu_pred,sat_pos,drop); %Get different between meas. and predict
        Kt = P_pred*Ht'*inv(R+Ht*P_pred*Ht'); %Kalman Gain
        mu_update = mu_pred + Kt*ytbar; %Perform update
        P_update = (eye(4) - Kt*Ht)*P_pred;
    end

    function [curr_meas] = meas_mdl(mu_cur,sat_positions,drop)
            if drop == 1
                pseudorange2_fixed = calc_pseudorange(sat_positions(2,:), [mu_cur(1), mu_cur(2)]);
                pseudorange3_fixed = calc_pseudorange(sat_positions(3,:), [mu_cur(1), mu_cur(2)]);
                pseudorange4_fixed = calc_pseudorange(sat_positions(4,:), [mu_cur(1), mu_cur(2)]);
                curr_meas = [pseudorange2_fixed; pseudorange3_fixed; pseudorange4_fixed];  
            elseif drop == 2
                pseudorange1_fixed = calc_pseudorange(sat_positions(1,:), [mu_cur(1), mu_cur(2)]);
                pseudorange3_fixed = calc_pseudorange(sat_positions(3,:), [mu_cur(1), mu_cur(2)]);
                pseudorange4_fixed = calc_pseudorange(sat_positions(4,:), [mu_cur(1), mu_cur(2)]);
                curr_meas = [pseudorange1_fixed; pseudorange3_fixed; pseudorange4_fixed];
           
            elseif drop == 3
                pseudorange1_fixed = calc_pseudorange(sat_positions(1,:), [mu_cur(1), mu_cur(2)]) ;
                pseudorange2_fixed = calc_pseudorange(sat_positions(2,:), [mu_cur(1), mu_cur(2)]) ;
                pseudorange4_fixed = calc_pseudorange(sat_positions(4,:), [mu_cur(1), mu_cur(2)]) ;
                curr_meas = [pseudorange1_fixed; pseudorange2_fixed; pseudorange4_fixed];
            elseif drop == 4
                pseudorange1_fixed = calc_pseudorange(sat_positions(1,:), [mu_cur(1), mu_cur(2)]);
                pseudorange2_fixed = calc_pseudorange(sat_positions(2,:), [mu_cur(1), mu_cur(2)]);
                pseudorange3_fixed = calc_pseudorange(sat_positions(3,:), [mu_cur(1), mu_cur(2)]);
                curr_meas = [pseudorange1_fixed; pseudorange2_fixed; pseudorange3_fixed];
            end
        
    end

    function [curr_meas] = simulate_noisy_meas(mu_cur,sat_positions,R,drop,fixed_variance)
        if fixed_variance == 1
            sensor_noise = sqrt(R)*randn(3,1);
            if drop == 1
                pseudorange2_fixed = calc_pseudorange(sat_positions(2,:), [mu_cur(1), mu_cur(2)]) + sensor_noise(1);
                pseudorange3_fixed = calc_pseudorange(sat_positions(3,:), [mu_cur(1), mu_cur(2)]) + sensor_noise(2);
                pseudorange4_fixed = calc_pseudorange(sat_positions(4,:), [mu_cur(1), mu_cur(2)]) + sensor_noise(3);
                curr_meas = [pseudorange2_fixed; pseudorange3_fixed; pseudorange4_fixed];  
            elseif drop == 2
                pseudorange1_fixed = calc_pseudorange(sat_positions(1,:), [mu_cur(1), mu_cur(2)]) + sensor_noise(1);
                pseudorange3_fixed = calc_pseudorange(sat_positions(3,:), [mu_cur(1), mu_cur(2)]) + sensor_noise(2);
                pseudorange4_fixed = calc_pseudorange(sat_positions(4,:), [mu_cur(1), mu_cur(2)]) + sensor_noise(3);
                curr_meas = [pseudorange1_fixed; pseudorange3_fixed; pseudorange4_fixed];
           
            elseif drop == 3
                pseudorange1_fixed = calc_pseudorange(sat_positions(1,:), [mu_cur(1), mu_cur(2)]) + sensor_noise(1);
                pseudorange2_fixed = calc_pseudorange(sat_positions(2,:), [mu_cur(1), mu_cur(2)]) + sensor_noise(2);
                pseudorange4_fixed = calc_pseudorange(sat_positions(4,:), [mu_cur(1), mu_cur(2)]) + sensor_noise(3);
                curr_meas = [pseudorange1_fixed; pseudorange2_fixed; pseudorange4_fixed];
            elseif drop == 4
                pseudorange1_fixed = calc_pseudorange(sat_positions(1,:), [mu_cur(1), mu_cur(2)]) + sensor_noise(1);
                pseudorange2_fixed = calc_pseudorange(sat_positions(2,:), [mu_cur(1), mu_cur(2)]) + sensor_noise(2);
                pseudorange3_fixed = calc_pseudorange(sat_positions(3,:), [mu_cur(1), mu_cur(2)]) + sensor_noise(3);
                curr_meas = [pseudorange1_fixed; pseudorange2_fixed; pseudorange3_fixed];
            end
        else
            
            
            if drop == 1
                R2 = R(1,1); R3 = R(2,2);R4 = R(3,3);

        
                sensor_noise2 = sqrt(R2)*randn(1,1);
                sensor_noise3 = sqrt(R3)*randn(1,1);
                sensor_noise4 = sqrt(R4)*randn(1,1);


                pseudorange2_fixed = calc_pseudorange(sat_positions(2,:), [mu_cur(1), mu_cur(2)]) + sensor_noise2;
                pseudorange3_fixed = calc_pseudorange(sat_positions(3,:), [mu_cur(1), mu_cur(2)]) + sensor_noise3;
                pseudorange4_fixed = calc_pseudorange(sat_positions(4,:), [mu_cur(1), mu_cur(2)]) + sensor_noise4;
                curr_meas = [pseudorange2_fixed; pseudorange3_fixed; pseudorange4_fixed];  
            elseif drop == 2
                R1 = R(1,1); R3 = R(2,2);R4 = R(3,3);

                sensor_noise1 = sqrt(R1)*randn(1,1);
                sensor_noise3 = sqrt(R3)*randn(1,1);
                sensor_noise4 = sqrt(R4)*randn(1,1);

                pseudorange1_fixed = calc_pseudorange(sat_positions(1,:), [mu_cur(1), mu_cur(2)]) + sensor_noise1;
                pseudorange3_fixed = calc_pseudorange(sat_positions(3,:), [mu_cur(1), mu_cur(2)]) + sensor_noise3;
                pseudorange4_fixed = calc_pseudorange(sat_positions(4,:), [mu_cur(1), mu_cur(2)]) + sensor_noise4;
                curr_meas = [pseudorange1_fixed; pseudorange3_fixed; pseudorange4_fixed];
           
            elseif drop == 3
                R1 = R(1,1); R2 = R(2,2);R4 = R(3,3);

                sensor_noise1 = sqrt(R1)*randn(1,1);
                sensor_noise2 = sqrt(R2)*randn(1,1);
                sensor_noise4 = sqrt(R4)*randn(1,1);

                pseudorange1_fixed = calc_pseudorange(sat_positions(1,:), [mu_cur(1), mu_cur(2)]) + sensor_noise1;
                pseudorange2_fixed = calc_pseudorange(sat_positions(2,:), [mu_cur(1), mu_cur(2)]) + sensor_noise2;
                pseudorange4_fixed = calc_pseudorange(sat_positions(4,:), [mu_cur(1), mu_cur(2)]) + sensor_noise4;
                curr_meas = [pseudorange1_fixed; pseudorange2_fixed; pseudorange4_fixed];
            elseif drop == 4
                R1 = R(1,1); R2 = R(2,2);R3 = R(3,3);

                sensor_noise1 = sqrt(R1)*randn(1,1);
                sensor_noise2 = sqrt(R2)*randn(1,1);
                sensor_noise3 = sqrt(R3)*randn(1,1);

                pseudorange1_fixed = calc_pseudorange(sat_positions(1,:), [mu_cur(1), mu_cur(2)]) + sensor_noise1;
                pseudorange2_fixed = calc_pseudorange(sat_positions(2,:), [mu_cur(1), mu_cur(2)]) + sensor_noise2;
                pseudorange3_fixed = calc_pseudorange(sat_positions(3,:), [mu_cur(1), mu_cur(2)]) + sensor_noise3;
                curr_meas = [pseudorange1_fixed; pseudorange2_fixed; pseudorange3_fixed];
            end
        end
    end

end

