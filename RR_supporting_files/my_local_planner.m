function[q_new] = my_local_planner(qe,qg,vel,tm,tc,err_tol,dof)
    global dimen;           global tf;
    global num_of_contacts; global h; 
    global safe_dist;       global contact_wrench;           
    global dista;           global l1contact; 
    global array_qo;        global l2contact;
    global q_o;             global v;
    
    % Will be used for plotting
    array_qo =  [];

    % My Local Planner function
    v = vel';
    dimen = dof;
    num_of_contacts = dof;
    safe_dist = 0.01;
    tf = tm;
    h = tc;
    q_o = qe';
    
    % vector to hold solution of unknowns
    z = zeros(2*dof, 1);
    
    % Fix the lower and upper bound of unknowns
    for j = 1:2*dof
        l(j) = -Inf;
        u(j) = Inf;
    end
    
    % Modify lower bound of complementarity variables
    for j = dof+1:2*dof
        l(j) = 0;
    end
    
    % Get collission info and call pathsolver
    q_new = qe';
    t = 0;
    while (t < tm)
        [contact_wrench,dista,l1contact,l2contact] = get_colli_info(q_new);
        %[z,f,J] = pathmcp(z,l,u,'mcpfuncjacEval');
        [z,~,~] = pathmcp(z,l,u,'mcpfuncjacEval');
        q_o = z(1:dof);
        array_qo = [array_qo, q_o];
        q_new = z(1:dof);
        
        % Check if reached goal. If yes! then exit.      
        if (norm(q_new-qg')<=err_tol)
            t = tm;
        end       
        
        % Increament of local time
        t = t + h;
    end
    q_new = q_new';
end