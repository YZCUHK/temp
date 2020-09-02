%----------------------------------------------------
% new Forward kinematics, for simplify calculation
% model: DH, q: joint position,
% T is transformation matrix: 4*4*7
% tip: tip position 3x1 & orientation 3x3; Jacob: 6x7x8
% mcenters: position of centers of mass 4x7
%----------------------------------------------------
function [ tip, Jacob, mcenters,T] = fkinew( model, q )
    T= [];                                    
    joint_num=7;
    % Update DH parameter 
    for i = 1:joint_num                                               
        if model.DH(i,1) == 1
            model.DH(i,5) = model.DH(i,5) + q(i);
        else
            model.DH(i,4) = model.DH(i,4) + q(i);
        end
    end
    % Calculate transformation matrices
    for i = 1:joint_num                                               
        V2=model.DH(i,2);V3=model.DH(i,3);V4=model.DH(i,4);V5=model.DH(i,5);V6=model.DH(i,6);V7=model.DH(i,7);
            if i==1
                T(:,:,i) = [ cos(V5)*cos(V7) - cos(V2)*sin(V5)*sin(V7), - cos(V7)*sin(V5) - cos(V2)*cos(V5)*sin(V7),  sin(V2)*sin(V7), V3*cos(V7) + V4*sin(V2)*sin(V7);
	cos(V5)*sin(V7) + cos(V2)*cos(V7)*sin(V5),   cos(V2)*cos(V5)*cos(V7) - sin(V5)*sin(V7), -cos(V7)*sin(V2), V3*sin(V7) - V4*cos(V7)*sin(V2);
	                         sin(V2)*sin(V5),                             cos(V5)*sin(V2),          cos(V2),                 V6 + V4*cos(V2);
	                                      0,                                           0,                0,                               1];
            else
                T(:,:,i) = [         cos(V5),        -sin(V5),        0,          V3;
	 cos(V2)*sin(V5), cos(V2)*cos(V5), -sin(V2), -V4*sin(V2);
	sin(V2)*sin(V5), cos(V5)*sin(V2),  cos(V2),  V4*cos(V2);
	              0,               0,        0,           1];
            end
    end  
    % tip location in Cartesian space
    T_base2tip = homotrans (T, joint_num, 0);      
    tip.pos = T_base2tip(1:3,4);
    tip.ori = T_base2tip(1:3,1:3);
    [Jacob,mcenters] = Jacobian(T, model, T_base2tip);
end

%homo transformation from base frame to finalframe.
function  Trans = homotrans (T, finalframe, initframe)

     Trans = eye(4,4);
     for i = (initframe+1):finalframe
         Trans = Trans*T(:,:,i);
     end
end

% Obtain Jacobian
function [Jacob, mcenters]  = Jacobian(T, model, T_base2tip )
    Trans_realtip_pos = T_base2tip(1:3,4);

    J             = [];
    Jacob      = zeros(6,7,8);
    [Joint_num,~] = size(model.DH);
    mvector=model.mcenter;
    mcenters=[];
 for j=1: Joint_num
     
        % position of center of mass
        mcenter=homotrans (T, j, 0)*mvector(j,:)';     
        mcenters=[mcenters mcenter];
        for i=1: j
        Trans_temp = homotrans (T, i, 0);
        Pos_vector = Trans_temp(1:3,4);
        Z_vector   = Trans_temp(1:3,3);
        
        if model.DH(i,1) == 1
            J_add = [cross(Z_vector,(mcenter(1:3,:)-Pos_vector));Z_vector];
            J             = [J J_add];
        end
        if model.DH(i,1) == 2
            J_add = [Z_vector; zeros(3,1)];
            J             = [J J_add];
        end
        end
       [Jx,Jy]=size(J);
    Jacob(1:Jx,1:Jy,j)=J;
    J=[];
 end
 
    for i=1:Joint_num
        
        Trans_temp    = homotrans (T, i, 0);
        Pos_vector = Trans_temp(1:3,4);
        Z_vector   = Trans_temp(1:3,3);
        
        if model.DH(i,1) == 1
            J_add = [cross(Z_vector,(Trans_realtip_pos-Pos_vector));Z_vector];
            J             = [J J_add];
        end
        if model.DH(i,1) == 2
            J_add = [Z_vector; zeros(3,1)];
            J             = [J J_add];
        end
    end
    Jacob(:,:,8)=J;
end
