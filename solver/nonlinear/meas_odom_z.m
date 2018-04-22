function [z] = meas_odom_z(vr, vl, rt, L)
    z(1) = (vr + vl)/2 * cos(rt);
    z(2) = (vr + vl)/2 * sin(rt);
    z(3) = (vr + vl)/L;
end

