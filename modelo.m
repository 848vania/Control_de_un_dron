%Constantes 
g = 9.78; 
m = 1.1;  %masa del dron en kg 
l = 0.184;   %distancia de cada brazo al centro de masa 
d = (sqrt(2)/2)*l;
%Inercias para cada eje 
Ixx = 1694.39e-6; 
Iyy = 1708.46e-6;
Izz = 2948.248e-6;
k = 7.7e-7; % 
bm = 6.1e-8; % 
syms x y z a b c xp yp zp ap bp cp xs ys zs as bs cs w1 w2 w3 w4 A B;
%Matriz de inercia 
M = [m 0 0 0 0 0;
     0 m 0 0 0 0;
     0 0 m 0 0 0;
     0 0 0 Ixx 0 -Ixx*sin(b);
     0 0 0 0 Iyy*(cos(a))^2+Izz*(sin(a))^2 (Izz-Iyy)*cos(b)*cos(a)*sin(a);
     0 0 0 -Ixx*sin(b) (Iyy-Izz)*cos(b)*cos(a)*sin(a) (Ixx+Iyy*(cos(b))^2)*(sin(a))^2+Izz*(cos(a))^2*(cos(b))^2];
%Matriz de Coriolis 
Co = [0 0 0 0 0 0;
      0 0 0 0 0 0;
      0 0 0 0 0 0;
      0 0 0 0 (Iyy-Izz)*(cos(a)*sin(a)*bp+cos(b)*(sin(a))^2*cp)+((Izz-Iyy)*cos(b)*(cos(a))^2-Ixx*cos(b))*cp (Izz-Iyy)*(cos(b))^2*cos(a)*sin(a)*cp;
      0 0 0 (Izz-Iyy)*(cos(a)*sin(a)*bp+cos(b)*(sin(a))^2*cp)+(Ixx+(Iyy-Izz)*(cos(a))^2)*cos(b)*cp (Izz-Iyy)*cos(a)*sin(a)*ap (Izz*(cos(a))^2+Iyy*(sin(a))^2-Ixx)*cos(b)*sin(b)*cp;
      0 0 0 (Iyy-Izz)*(cos(b))^2*sin(a)*cos(a)*cp-Ixx*cos(b)*bp (Izz-Iyy)*(cos(a)*sin(a)*sin(b)*bp+cos(b)*(sin(a))^2*ap)+(Iyy-Izz)*(cos(a))^2*cos(b)*ap+(Ixx-Izz*(cos(a))^2-Iyy*(sin(a))^2)*sin(b)*cos(b)*cp (Ixx-Izz*(cos(a))^2-Iyy*(sin(a))^2)*sin(b)*cos(b)*bp+(Iyy-Izz)*cos(a)*sin(a)*(cos(b))^2*ap];
% Vector de termino gravitacionales 
G = [0;
     0;
     m*g;
     0;
     0;
     0];
% x,y,z son los desplazamientos en cada eje 
% a,b,c son los angulos que se forman con cada eje 
q = [x;
     y;
     z;
     a;
     b;
     c];
% primera derivada 
qp = [xp;
      yp;
      zp;
      ap;
      bp;
      cp];
%Vector de fuerzas generalizadas externas 
%Tres primeras filas fuerzas lineales, las ultimas fuerzas angulares 
qt = [sin(b)*(k*w1^2+k*w2^2+k*w3^2+k*w4^2);
      -cos(b)*sin(a)*(k*w1^2+k*w2^2+k*w3^2+k*w4^2);
      cos(b)*cos(a)*(k*w1^2+k*w2^2+k*w3^2+k*w4^2);
      d*(k*w2^2+k*w3^2-k*w1^2-k*w4^2);
      d*(k*w1^2+k*w2^2-k*w3^2-k*w4^2);
      bm*w2^2+bm*w4^2-bm*w1^2-bm*w3^2];
f2 = inv(M);
f3 = f2*(qt-Co*qp-G);
%Segunda derivada despejada 
xpf = [qp;
       f3];
Ad = jacobian(xpf, [x,y,z,a,b,c,xp,yp,zp,ap,bp,cp]);   
Bd = jacobian(xpf, [w1,w2,w3,w4]);

%Substituyendo en el punto de equilibrio 
%Donde los tres angulos y las tres velocidades tienen que ser igual a cero.
%Matriz A 
A = subs(Ad,{a,b,c,ap,bp,cp,w1,w2,w3,w4},{0,0,0,0,0,0,1868.91,1868.91,1868.91,1868.91});
%Matriz B 
B = subs(Bd,{a,b,c,ap,bp,cp,w1,w2,w3,w4},{0,0,0,0,0,0,1868.91,1868.91,1868.91,1868.91});
  
CC = ctrb(A,B);        % matriz de controlabilidad 
n = rank(CC);   % Ver si es controlable

%Matriz C 
C = [0 0 0 0 0 0 0 0 0 0 0 0;
     0 0 0 0 0 0 0 0 0 0 0 0;
     0 0 0 0 0 0 0 0 0 0 0 0;
     0 0 0 1 0 0 0 0 0 0 0 0;
     0 0 0 0 1 0 0 0 0 0 0 0;
     0 0 0 0 0 1 0 0 0 0 0 0;
     0 0 0 0 0 0 0 0 0 0 0 0;
     0 0 0 0 0 0 0 0 0 0 0 0;
     0 0 0 0 0 0 0 0 0 0 0 0;
     0 0 0 0 0 0 0 0 0 1 0 0;
     0 0 0 0 0 0 0 0 0 0 1 0;
     0 0 0 0 0 0 0 0 0 0 0 1];

%Matriz D 
D = zeros(size(C,1),size(B,2));