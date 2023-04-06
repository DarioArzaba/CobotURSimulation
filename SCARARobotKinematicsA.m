clc
clear
close all

syms t1 t2 t3 t4 d1 d2 d3 d4 a1 a2

a1 = [cos(t1) -sin(t1) 0 a1*cos(t1); 
      sin(t1) cos(t1) 0 a1*sin(t1);
      0 0 1 d1;
      0 0 0 1];

a2 = [cos(t2) sin(t2) 0 a2*cos(t2); 
      sin(t2) -cos(t2) 0 a2*sin(t2);
      0 0 -1 0;
      0 0 0 1];

a3 = [1 0 0 0; 
      0 1 0 0;
      0 0 1 d3;
      0 0 0 1];

a4 = [cos(t4) -sin(t4) 0 0; 
      sin(t4) cos(t4) 0 0;
      0 0 1 d4;
      0 0 0 1];

T0 = simplify(expand(a1*a2*a3*a4));
pretty(T0)