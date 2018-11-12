function [spatial_jac] = comp_spatial_jac(type_joint, q_axes, joint_axes, transform_upto_joint)
    dim = 3;
    num_of_joints = length(type_joint);
    spatial_jac = zeros(dim+3, num_of_joints);
    
    for i = 1:num_of_joints
        if i > 1
        g = transform_upto_joint(:,:,i);
        R = g(1:3,1:3);
        p = g(1:3,4);
        p_hat = [0 -p(3) p(2);
                     p(3) 0 -p(1);
                    -p(2) p(1) 0];
        temp1 = p_hat*R;
        Ad_g = [R temp1];
        temp2 = [zeros(3,3) R];
        Ad_g = [Ad_g; temp2];
        end
        if strcmp(type_joint(i), 'R')
            omega = joint_axes(:,i);
            q = q_axes(:,i);
            xi = [cross(-omega, q); omega];
        end
        if strcmp(type_joint,'P')
            omega = zeros(3,1);
            v = joint_axes(:,i);
            xi = [v; omega];
        end
        if i > 1
            xi_prime = Ad_g*xi;
        else
            xi_prime = xi;
        end
        spatial_jac(:,i) =  xi_prime;
    end
end