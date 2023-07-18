function [ delta ] = Calc_Pos_Delta(init_pos, final_pos)
    % Compute the difference (delta) between init pose and final pose
    delta_xyz = final_pos - init_pos; 
    
    % Get delta in each coordinate
    delta_x = delta_xyz(1, 1);
    delta_y = delta_xyz(2, 1);
    delta_z = delta_xyz(3, 1);
    
    % Compute the Euclidean distance projection between init_pos and 
    % final_pos in [x, y] plane (handy for drawings on a table)
    euc_xy = sqrt(delta_x^2 + delta_y^2);
    
    % Compute the Euclidean distance between init_pos and final_pos
    euc_xyz = sqrt(euc_xy^2 + delta_z^2);
    
    % Vector de diferenciales, distancias euclidianas y orientacion del
    % extremo del brazo
    delta = [delta_x; delta_y; delta_z; euc_xy; euc_xyz];
    
end