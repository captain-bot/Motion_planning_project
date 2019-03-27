function [contact_wrench,dista,contact_jacobian] = bax_colli_info(joint_th)
    global cir; global num_dof; global num_contacts;
    
    % Number of Obstacles
    num_obs = size(cir,1);
    
    % Initialize contact wrench and contact jacobian arrays
    contact_wrench = zeros(6,num_contacts);
    contact_jacobian = zeros(6,num_dof,num_contacts);
    dista = zeros(1,num_contacts);
    
    % Get edge points of each segments of the arm
    [segs, spatial_jac] = get_edges_of_segments(joint_th);
    count = 1;    
    % Compute contact jacobians and contact wrenches for each segment
    for i = 1:size(segs,1)
        if i == 1 || i == 3 || i == 5 || i == 7
            link_s_cp_array = [];
            dist_link_s_array = [];
            cw_array = [];
            val_count = 1;
        end
        for j = 1:num_obs
            [link_s_cp, dist_link_s, cw] = sphere_line_distance(cir(j,2:end)', cir(j,1), reshape(segs(i,:),[3,2]));
            link_s_cp_array(:,:,val_count) = link_s_cp;
            dist_link_s_array(1,val_count) = dist_link_s;
            cw_array(:,val_count) = cw;
            val_count = val_count + 1;
        end
        % compute minimum of dist_link_s and corresponding contact jacobian
        if i == 2 || i == 4 || i == 6 || i == 7
            [min_dist, min_idx] = min(dist_link_s_array);
            dista(1,count) = min_dist;
            contact_wrench(1:3,count) = cw_array(:,min_idx);
            contact_pt = link_s_cp_array(:,2,min_idx);
            contact_jacobian(:,:,count) = compute_conjac(contact_pt,count,spatial_jac);
            count = count + 1;
        end
    end
end

% //////////////////////////////////////////////////
%    Find terminal points of each segments        //
% //////////////////////////////////////////////////
function [seg_vec, spatial_jac] = get_edges_of_segments(theta)
    global g_st0;       global gst0_art;     global wr_base;
    global wr;          global qr;           global qr_base;
    global base_mat;    global type_joint;   

    % Solve direct position kinematics
    [gst, transform_upto_joint] = direct_kin(g_st0, type_joint, wr, qr, theta);

    % Transformation of the end frame
    gend = base_mat*gst;
    
    % Find transformation of intermediate points
    inter_transform = zeros(4,4,size(transform_upto_joint,3));
    transf_upto_joint_wrt_base = zeros(4,4,size(transform_upto_joint,3));
    for i = 1:size(transform_upto_joint,3)-1
        inter_transform(:,:,i) = base_mat*transform_upto_joint(:,:,i)*gst0_art(:,:,i);
        transf_upto_joint_wrt_base(:,:,i) = base_mat*transform_upto_joint(:,:,i);
    end
    inter_transform(:,:, end) = gend;
    transf_upto_joint_wrt_base(:,:,end) = gend;   % transformation_upto_joint with respect to base frame
   
    % Find the terminal coordinates of each segment
    seg_vec = zeros(size(inter_transform,3)-1,6);
    for i = 1:size(transform_upto_joint,3)-1
        seg_vec(i,1:3) = inter_transform(1:3,4,i)';
        seg_vec(i,4:6) = inter_transform(1:3,4,i+1)';
    end
    
    % Compute spatial jacobian wrt base frame
    spatial_jac = comp_spatial_jac(type_joint, qr_base, wr_base, transf_upto_joint_wrt_base);
end