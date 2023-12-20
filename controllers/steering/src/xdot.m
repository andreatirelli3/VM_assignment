function xdot = xdot(x, A, B, K, B_d, d)
    xdot = (A - B*K)*x + B_d*d;
end