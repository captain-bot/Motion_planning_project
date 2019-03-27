function[q_new] = my_local_planner(qe,qg,vel,tm,tc,err_tol,dof)
    global dimen;           global v;
    global num_of_contacts; global h; 
    global safe_dist;       global contact_wrench;           
    global dista;           global l1contact; 
    global array_qo;        global l2contact;
    global q_o;             global array_vin;
    global array_compensating_vel;
    
    % Will be used for plotting
    array_qo =  [];
    array_vin = [];
    array_compensating_vel = [];

    % My Local Planner function
    v = vel';
    dimen = dof;
    num_of_contacts = dof;
    safe_dist = 0.01;
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
        array_vin = [array_vin, v];
        array_compensating_vel = [array_compensating_vel, z(dof+1:end)];
        q_new = z(1:dof);
        % Check if reached goal. If yes! then exit.      
        if (norm(q_new-qg')<=err_tol)
            t = tm;  % break
        end       
        
        % Increament of local time
        t = t + h;
    end
    q_new = q_new';
end