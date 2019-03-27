function [conjac] = compute_conjac(contact_pt,contact_indx,spt_jac)
    global num_dof;    
    conjac = zeros(6,num_dof);
    if contact_indx == 1
        conjac(:,1) = spt_jac(:,1);        
    else
        conjac(:,1:2*contact_indx-1) = spt_jac(:,1:2*contact_indx-1);
    end
    conjac = [eye(3,3) -skew_sym(contact_pt); zeros(3,3) zeros(3,3)]*conjac; 
end

function [skew_sym_mat] = skew_sym(vec)
    skew_sym_mat = [0 -vec(3) vec(2);
                    vec(3) 0 -vec(1);
                    -vec(2) vec(1) 0];
end