function [f,J, domerr]= mcpfuncjacEval(z,jacflag)
    global dimen; global links; global h; global safe_dist;
    global contact_wrench; global dista; global l1contact; global l2contact;
    global q_o; global v;

    f = [];
    J = [];
    domerr = 0;

    q_n = [z(1);z(2)];

    p_1 = z(3);
    p_2 = z(4);

    theta_1o = q_o(1);
    theta_2o = q_o(2);

    % jacobian for end_effector at beginning of current step
    Je11_o = -links(1)*sin(theta_1o)-links(2)*sin(theta_1o + theta_2o);
    Je12_o =-links(2)*sin(theta_1o + theta_2o);
    Je21_o = links(1)*cos(theta_1o)+links(2)*cos(theta_1o + theta_2o);
    Je22_o = links(2)*cos(theta_1o + theta_2o);

    Je_o=[Je11_o,Je12_o;Je21_o,Je22_o];

    % jacobian for contact point of bar 1 at beginning of current step
    Jc11_1 = -l1contact*sin(theta_1o);
    Jc12_1 =0;
    Jc21_1 = l1contact*cos(theta_1o);
    Jc22_1 = 0;
    % jacobian for contact point of bar 2 at beginning of current step
    Jc11_2 = -links(1)*sin(theta_1o)-l2contact*sin(theta_1o + theta_2o);
    Jc12_2 =-l2contact*sin(theta_1o + theta_2o);
    Jc21_2 = links(1)*cos(theta_1o)+l2contact*cos(theta_1o + theta_2o);
    Jc22_2 = l2contact*cos(theta_1o + theta_2o);

    jcon1=[Jc11_1,0;Jc21_1,0];
    invjcon1= pinv(jcon1);
    jcon2=[Jc11_2,Jc12_2;Jc21_2,Jc22_2];
    invjcon2= pinv(jcon2);

    eq_of_motion = (q_o-q_n) + (h*v) + h*(invjcon1*contact_wrench(:,1)*p_1) + h*(invjcon2*contact_wrench(:,2)*p_2);

    % plotp(:,count+1)=[p_1;p_2];

    for i = 1:dimen
        f(i) = eq_of_motion(i,1);
    end
    temp(1,1) = (contact_wrench(:,1))'*jcon1*(q_n-q_o);
    temp(2,1) = (contact_wrench(:,2))'*jcon2*(q_n-q_o);
    f(3) = temp(1,1) + (dista(1) - safe_dist);
    f(4) = temp(2,1) + (dista(2) - safe_dist);

    % plotjointrate(:,count+1)=((q_n-q_o))/h;

    if (jacflag)
        J=zeros(4,4);
        J(1,1) = -1; J(1,2) = 0; J(1,3) = h*((invjcon1(1,1)*contact_wrench(1,1))+(invjcon1(1,2)*contact_wrench(2,1))); J(1,4) =h*((invjcon2(1,1)*contact_wrench(1,2))+(invjcon2(1,2)*contact_wrench(2,2)));
        J(2,1) = 0; J(2,2) = -1; J(2,3) = h*((invjcon1(2,1)*contact_wrench(1,1))+(invjcon1(2,2)*contact_wrench(2,1))); J(2,4) =h*((invjcon2(2,1)*contact_wrench(1,2))+(invjcon2(2,2)*contact_wrench(2,2)));

        J(3,1) = (contact_wrench(1,1)*jcon1(1,1))+(contact_wrench(2,1)*jcon1(2,1)); J(3,2) = (contact_wrench(1,1)*jcon1(1,2))+(contact_wrench(2,1)*jcon1(2,2));
        J(4,1) = (contact_wrench(1,2)*jcon2(1,1))+(contact_wrench(2,2)*jcon2(2,1)); J(4,2) = (contact_wrench(1,2)*jcon2(1,2))+(contact_wrench(2,2)*jcon2(2,2));

        J=sparse(J);
    end

end