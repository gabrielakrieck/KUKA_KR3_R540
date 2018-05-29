clc, clear all, close all

%Define Rob�:
MeuRobo=KUKA_KR3_R540

%Plota Rob� na posi��o master:
plotar(MeuRobo)

%Eqn cinem�tica direta simb�licas:
[A1 A2 A3 A4 A5 A6]=CineD_simb(MeuRobo);

%Eqn cinem�tica direta num�rica:

%Q = [0 -pi/2 pi/2 80*pi/180 0 0] %Vetor Q 1x6 qualquer
Q=pi/180*[170 -50 10 1 1 1]
plotar(MeuRobo,Q)
M=CineD_num(MeuRobo, Q);

% Eqn cinem�tica inversa:
Q = [pi/4 pi/18 pi/8 pi/2 pi pi/3] %Vetor Q 1x6 qualquer
M=CineD_num(MeuRobo, Q);
Qi=CineI(M)
