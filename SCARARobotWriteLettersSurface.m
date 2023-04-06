%Función Cinemática Inversa SCARA
function qq = words_SCARA(Tp)
%Datos
a1 = 0.25;
a2 = 0.25;
d1 = 0.5;
d4 = 0.15;
qq=[];
n = size(Tp);

    for i=1:n(2)
        tp = Tp(i);
        tpR = tp.R;
        tpx = tp.t(1);
        tpy = tp.t(2);
        tpz = tp.t(3);
        r11 = tpR(1,1);
        r12 = tpR(1,2);
        %Theta2
        D = (tpx^2+tpy^2-(a1^2)-(a2^2))/(2*a1*a2);
        D1 = real(sqrt(1-D^2));
        ang2 = atan2(D1,D);
        %th2 = rad2deg(ang2);
        %Theta1
        ang1 = atan2(tpy,tpx)-atan2((a2*sin(ang2)),(a1+a2*cos(ang2)));
        %th1 = rad2deg(ang1)
        %Desplazamiento3
        d3 = d1-(tpz+d4);
        %Theta4
        phi = atan2(r12,r11);
        %th4 = rad2deg(th1+th2-phi);
        ang4 = ang1+ang2-phi;
        %Q
        %q=[th1, th2, d3, th4];
        q = [ang1 ang2 d3 ang4];
        qq = vertcat(qq,q);
    end

end
