clc
clear
close all

startup_rvc
L(1) = Link([0 0.5 0.25 0])
L(2) = Link([0 0 0.15 pi])
L(3) = Link([0 0.3 0 0])
L(4) = Link([0 0.15 0 0])

R = SerialLink(L)

q0 = [0 0 0 0]
q1 = [-pi/2 pi/4 0.2 pi/3]

T0 = R.fkine(q0)

T1 = R.fkine(q1)

R.plot(q1)