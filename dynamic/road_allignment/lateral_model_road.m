function e_dot = lateral_model_road(x,u,C_f,C_r,l_f,l_r,inertia,V, v_mass,d)

A = [0 1 0 0; 
    0 -((2*C_f + 2*C_r)/(v_mass*V)) ((2*C_f + 2*C_r)/(v_mass)) (((-2*C_f*l_f) + 2*C_r*l_r)/(v_mass*V));
    0 0 0 1;
    0 -((2*C_f*l_f-2*C_r*l_r)/(inertia*V)) ((2*C_f*l_f-2*C_r*l_r)/inertia) -((2*C_f*(l_f^2)+(2*C_r*(l_r^2)))/(inertia*V))];

B = [0;(2*C_f)/v_mass; 0; (2*l_f*C_f)/inertia];

Bd = [0; ((-((2*C_f*l_f - 2*C_r*l_r)/(v_mass*V)))-V); 0; -((2*C_f*(l_f^2)+2*C_r*(l_r^2))/(inertia*V))];

e_dot = A*x + B*u + Bd*d;
end