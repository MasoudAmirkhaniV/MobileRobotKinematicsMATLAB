function J = Jacob(q)
    L1 = 1;
    L2 = 1;
    L3 = 1;

    t1 = q(1);
    t2 = q(2);
    t3 = q(3);

    z0 = [0; 0; 1];
    z1 = z0;
    z2 = z1;

    p0 = [0; 0; 0];
    p1 = [L1*cos(t1); L1*sin(t1); 0];
    p2 = [L2*cos(t1+t2) + L1*cos(t1); L2*sin(t1+t2) + L1*sin(t1); 0];
    p3 = [L3*cos(t1+t2+t3) + L2*cos(t1+t2) + L1*cos(t1); L3*sin(t1+t2+t3) + L2*sin(t1+t2) + L1*sin(t1); 0];

    Ja = [cross(z0, (p3-p0)) cross(z1, (p3-p1)) cross(z2, (p3-p2))];
    Jw = [z0 z1 z2];

    J = Ja;

end