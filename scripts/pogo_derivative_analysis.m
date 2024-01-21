clear; clc;
pkg load symbolic
syms x_j y_j theta_j
syms x_i y_i theta_i
syms x_z y_z theta_z
syms z_i_j t_i_j
syms psi gamma theta

psi = theta_z - theta_i
gamma = theta_j - theta_i
z_i_j = [[cos(psi), -sin(psi), x_z - x_i] ;
         [sin(psi),  cos(psi), y_z - y_i] ;
                [0,         0,         1]];
t_i_j = [[cos(gamma), -sin(gamma), x_j - x_i] ;
         [sin(gamma),  cos(gamma), y_j - y_i] ;
                [0,         0,         1]];
z_j_i = simplify(inv(z_i_j));
before_t2v = simplify(z_j_i * t_i_j);
e_i_j = [[before_t2v(1,3)];
         [before_t2v(2,3)];
         [acos(before_t2v(1,1))]]

variables = [x_i, y_i, theta_i]

A =simplify([[diff(e_i_j(1), x_i), diff(e_i_j(1), y_j), diff(e_i_j(1), theta_i)] ;
            [diff(e_i_j(2), x_i), diff(e_i_j(2), y_i), diff(e_i_j(2), theta_i)] ;
            [diff(e_i_j(3), x_i), diff(e_i_j(3), y_i), diff(e_i_j(3), theta_i)]])

B =simplify([[diff(e_i_j(1), x_j), diff(e_i_j(1), y_j), diff(e_i_j(1), theta_j)] ;
            [diff(e_i_j(2), x_j), diff(e_i_j(2), y_j), diff(e_i_j(2), theta_j)] ;
            [diff(e_i_j(3), x_j), diff(e_i_j(3), y_j), diff(e_i_j(3), theta_j)]])
