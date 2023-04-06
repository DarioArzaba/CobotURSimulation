%Actividad clase cinemática inversa para robot 3R en el plano.
%Solución geométrica
%Función invk_rob3R plano; salida = thetas, entradas=Tp matriz homogénea 
%defininedo posición y orientación de pinza, a1,a2,a3 dimensiones de
%eslabones del robot
function [th1,th2,th3] = invk_rob3R_2d_geom(Tp,a1, a2, a3)
phi = acosd(Tp(1,1));
pwx = Tp(1,4)-a3*cosd(phi)
pwy = Tp(2,4)-a3*sind(phi)

%Calcular theta 2
c2 = (pwx^2 + pwy^2 - a1^2 - a2^2) / (2*a1*a2)
s2 = -sqrt(1-c2^2);
th2 = atan2d(s2,c2);
%th2 = acosd(c2) %solución con theta 2 positiva (codo abajo)

%Calcular alpha y beta
alpha = atan2d(pwy, pwx)
%beta = acosd((a1^2 - a2^2 + pwx^2 + pwy^2)/(2*a1*sqrt(pwx^2 + pwy^2)))
cb = (a1^2 - a2^2 + pwx^2 + pwy^2)/(2*a1*sqrt(pwx^2 + pwy^2));
sb = sqrt(1-cb^2);
beta = atan2d(sb,cb)

% Calcular theta 1
th1 = alpha + beta; %beta negativa para theta2 positiva

%Calcular theta 3
th3 = phi - th1 - th2

%definimos estructura del robot
L1=Link('revolute','d',0,'a',a1,'alpha',0);
L2=Link ('revolute','d',0,'a',a2,'alpha',0);
L3=Link ('revolute','d',0,'a',a3,'alpha',0);
rob3R = SerialLink ([L1 L2 L3],'name','Rob 3R');
rob3R.plot([deg2rad(th1) deg2rad(th2) deg2rad(th3)],'top');
end