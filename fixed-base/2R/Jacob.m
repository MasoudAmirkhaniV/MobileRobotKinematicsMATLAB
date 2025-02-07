function J = Jacob(q)
    global L1 L2

    t1 = q(1);
    t2 = q(2);

    z0 = [0; 0; 1];
    z1 = z0;

    p0 = [0; 0; 0];
    p1 = [L1*cos(t1); L1*sin(t1); 0];
    p2 = [L2*cos(t1+t2) + L1*cos(t1); L2*sin(t1+t2) + L1*sin(t1); 0];

    Ja = [cross(z0, (p2-p0)) cross(z1, (p2-p1))];
    Jw = [z0 z1];

    J = Ja;

end