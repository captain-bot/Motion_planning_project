function [f,J,domerr]= mcpfuncjacEval(z,jacflag)
    global num_dof;         global num_contacts;
    global contact_wrench;  global contact_jacobian;
    global h;               global safe_dist;           
    global dista;           global q_o;                 global v;
    
    z = z(:);
    f = [];
    J = [];
    domerr = 0;
    % Unknowns in position and velocity variables
    q_n = z(1:num_dof);
    v_n = z(num_dof+1:end);

    % Compute inverse of contact jacobians J^T(JJ^T)^-1
    invjcon = zeros(size(contact_jacobian,2),size(contact_jacobian,1),size(contact_jacobian,3));
    for i = 1:num_contacts
        if 2*i-1 < 7
            invjcon(1:2*i-1,:,i) = pinv(contact_jacobian(:,1:2*i-1,i));
        else
%             invjcon(:,:,i) = contact_jacobian(:,:,i)'*inv(contact_jacobian(:,:,i)*contact_jacobian(:,:,i)');
            invjcon(:,:,i) = pinv(contact_jacobian(:,:,i));
        end
    end
    
    % The Equations of Motion
    temp_sum = 0;
    temp_prod1 = zeros(num_dof,num_contacts);
    temp_prod2 = zeros(num_contacts, num_dof);
    temp = zeros(num_contacts,1);
    for i = 1:num_contacts
        temp_prod1(:,i) = invjcon(:,:,i)*contact_wrench(:,i);
        temp_sum = temp_sum + temp_prod1(:,i)*v_n(i);
        temp_prod2(i,:) = contact_wrench(:,i)'*contact_jacobian(:,:,i);
        temp(i,1) = temp_prod2(i,:)*(q_n-q_o);
    end
    eom1 = (q_o-q_n) + h*v + h*temp_sum;
    eom2 = temp + (dista - safe_dist*ones(1,num_contacts))';
    f = [eom1;eom2];

    % Construct the jacobian matrix of EOM
    if (jacflag)
        J=zeros(num_dof+num_contacts,num_dof+num_contacts);
        J(1:num_dof,1:num_dof) = -eye(num_dof,num_dof);
        J(1:num_dof,num_dof+1:end) = h*temp_prod1;
        J(num_dof+1:end,1:num_dof) = temp_prod2;
        J=sparse(J);   % Take advantage of sparsity properties of a matrix
    end
end