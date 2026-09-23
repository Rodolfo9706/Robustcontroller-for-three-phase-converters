%system parameters
R = 0.1 ; %Resistencìa
w = 60 ; % frecuencìa ------------------
L = 10*10^-3 ;  % inductancia
C = 22000*10^-6 ; % Capacitancìa 
%r = 1; %voltaje deseado
%Rl = 33.33; %cargas
E_m = 80 ;


%control parameters

x2d = 0;
x3d = 48 ;
x1d_1 = (E_m/(2*R)) + (sqrt((E_m)^2 - (16*R*x3d^2)))/(2*R);
x1d_2 = (E_m/(2*R)) - (sqrt((E_m)^2 - (16*R*x3d^2)))/(2*R);
%tomamos el menor
x1d = x1d_2;
%Ganancìas 

k1 = 5;
k2 = 10000; %50
k3 = 5000; %60
km = 10000;
epsilon = 0.1;

%Modelo propuesto 
am = 15;
bm = 15;
%um= x3d;

gamma = 1;
alpha = 1 ;
th = 0;
