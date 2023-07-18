function [Q] = Get_Link_Coordinates(fwdkin_res)

% Get coordinates for all links, from the forward kinematics result
Q1 = [0 fwdkin_res(1,4) fwdkin_res(1,8) fwdkin_res(1,12) fwdkin_res(1,16)]; % X coordinates for all links
Q2 = [0 fwdkin_res(2,4) fwdkin_res(2,8) fwdkin_res(2,12) fwdkin_res(2,16)]; % Y coordinates for all links
Q3 = [0 fwdkin_res(3,4) fwdkin_res(3,8) fwdkin_res(3,12) fwdkin_res(3,16)]; % Z coordinates for all links

Q  =[Q1; Q2; Q3];
end

