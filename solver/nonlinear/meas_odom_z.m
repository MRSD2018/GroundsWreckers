function [z] = meas_odom_z(vl, vr, rt)
    L = .508;
    z = zeros(3, 1);
    dt = 1/4.2;

    z(1) = dt * ( (vr + vl)/2 ) * cos(rt);
    z(2) = dt * ( (vr + vl)/2 ) * sin(rt);
    z(3) = dt * (vr - vl)/L;
end

