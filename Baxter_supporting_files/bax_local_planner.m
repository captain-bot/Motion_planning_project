function[q_new] = bax_local_planner(qe,qg,vel,tm,tc,err_tol)  
    global tf;                global num_dof;
    global num_contacts;      global h; 
    global dista;             global v;
    global array_qo;          global q_o;
    global contact_jacobian;  global contact_wrench;                 
    
    % Will be used for plotting
    array_qo =  [];

    % My Local Planner function
    v = vel';
    tf = tm;
    h = tc;
    q_o = qe';
    
    % vector to hold solutions for unknowns
    z = zeros(num_dof+num_contacts, 1);
    
    % Fix the lower and upper bound of unknowns
    for j = 1:num_dof+num_contacts
        l(j) = -Inf;
        u(j) = Inf;
    end
    
    % Modify lower bound of complementarity variables to ZERO
    for j = num_dof+1:num_dof+num_contacts
        l(j) = 0;
    end
    
    % Get collission info and call pathsolver
    q_new = qe';
    t = 0;
    while (t < tm)
        [contact_wrench,dista,contact_jacobian] = bax_colli_info(q_o);
        [z,f,J] = pathmcp(z,l,u,'mcpfuncjacEval');
        q_o = z(1:num_dof);
        array_qo = [array_qo, q_o];
        q_new = z(1:num_dof);
        
        % Check if reached goal. If yes! then exit.      
        if (norm(q_new-qg')<=err_tol)
            t = tm;
        end       
        
        % Increament of local time
        t = t + h;
    end
    q_new = q_new';
end