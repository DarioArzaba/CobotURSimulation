clear 
close all
clc

%Links
a1=0.25; a2=0.15; d1=0.5; d4=0.15;
L1=Link('revolute','d',d1,'a',a1,'alpha',0);
L2=Link ('revolute','d',0,'a',a2,'alpha',pi);
L3=Link ('prismatic','theta',0,'a',0,'alpha',0);
L4=Link('revolute','d',d4,'a',0,'alpha',0);

%Definir limites de articulación
L1.qlim=[deg2rad(-180) deg2rad(180)];
L2.qlim=[deg2rad(-120) deg2rad(120)];
L3.qlim=[0 5];
L4.qlim=[deg2rad(-160) deg2rad(160)];

%Se enlaza la cadena cinemática
scara=SerialLink ([L1 L2 L3 L4],'name','Scara');

%LETRAS
%R
load hershey
R = hershey{'R'}; %cargar la letra mayúscula R
R.stroke;
path1 = [ 0.2*R.stroke; zeros(1,numcols(R.stroke))];
k1 = find(isnan(path1(1,:)));
path1(:,k1) = path1(:,k1-1); 
path1(3,k1) = 0.1;
path1(1,:)=path1(1,:)+0.05;
%O
O = hershey{'O'}; %cargar la letra mayúscula O
O.stroke;
path2 = [ 0.2*O.stroke; zeros(1,numcols(O.stroke))];
k2 = find(isnan(path2(1,:)));
path2(:,k2) = path2(:,k2-1); 
path2(3,k2) = 0.1;
path2(1,:)=path2(1,:)+0.25;
%B
B = hershey{'B'}; %cargar la letra mayúscula B
B.stroke;
path3 = [ 0.2*B.stroke; zeros(1,numcols(B.stroke))];
k3 = find(isnan(path3(1,:)));
path3(:,k3) = path3(:,k3-1); 
path3(3,k3) = 0.1;
path3(1,:)=path3(1,:)+0.45;

path=([path1,path2,path3]);
traj = mstraj(path(:,2:end)', [0.8 0.8 0.8], [], path(:,1)',0.02, 0.1);
clf
plot3(traj(:,1), traj(:,2), traj(:,3))
Tp1 = SE3(traj) * SE3.oa([0 1 0], [0 0 -1]);
q1 = words_SCARA(Tp1);
scara.plot(q1,'reach',1)