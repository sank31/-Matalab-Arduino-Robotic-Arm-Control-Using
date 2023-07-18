function [ fwdkin_result ] = Calc_Fwd_Kinematics(dh_params)
    % Columns: 1 = theta, 2 = d, 3 = a (o r), 4 = alpha
    T12 = Make_DH_Matrix(dh_params(1,1), dh_params(2,1), dh_params(3,1), dh_params(4,1));
    T23 = Make_DH_Matrix(dh_params(1,2), dh_params(2,2), dh_params(3,2), dh_params(4,2));
    T34 = Make_DH_Matrix(dh_params(1,3), dh_params(2,3), dh_params(3,3), dh_params(4,3));
    T45 = Make_DH_Matrix(dh_params(1,4), dh_params(2,4), dh_params(3,4), dh_params(4,4));

    T13 = T12 * T23;
    T14 = T13 * T34;
    T15 = T14 * T45;
    
    % Pose of each link w.r.t. origin 1
    fwdkin_result = [T12 T13 T14 T15]; 
end

