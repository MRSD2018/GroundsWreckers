function [z] = meas_odom_z(vl, vr, rt)
    L = .508;
    z = zeros(3, 1);
    z(1) = (vr + vl)/2 * cos(rt);
    z(2) = (vr + vl)/2 * sin(rt);
    z(3) = (vr + vl)/L;
end

