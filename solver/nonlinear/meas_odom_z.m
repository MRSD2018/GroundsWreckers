function [z] = meas_odom_z(vl, vr, rt)
    L = .508;
    z = zeros(3, 1);
    dt = 1/10.0;
    %%%%%%%% This is a grudsby related bamboozle
    scale = 2;
    vr = vr / scale;
    vl = vl / scale;
    %%%/ bamboozle %%%%%

    z(1) = dt * ( (vr + vl)/2 ) * cos(rt);
    z(2) = dt * ( (vr + vl)/2 ) * sin(rt);
    z(3) = dt * (vr - vl)/L;
end

