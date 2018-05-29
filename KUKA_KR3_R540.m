%KUKA_KR3_R540 cria um modelo do manipulador KUKA AGILUS KR3 R540
%
%
% KUKA_KR3_R540 é um script que cria um modelo para descrever as caracterís-
% ticas cinemáticas e dinâmicas do manipulador KUKA AGILUS KR3 R540 consi-
% derando como efetuador uma garra elétrica paralela FESTO HGPLE-14-30-3,
% 1-DC-VCSC-G96.
%
%
% O modelo do robô é descrito pelos parâmetros de Denavit-Hartenberg dados
% por:
%
%       |    qi    |  di[mm] | ai[mm] | alphai |
% elo 1 |    q1    |  -345   |   20   |  pi/2  |
% elo 2 |    q2    |   0     |  260   |    0   |
% elo 3 |  q3-pi/2 |   0     |   20   |  pi/2  |
% elo 4 | q4-4*pi/9|  -260   |    0   | -pi/2  |
% elo 5 |    q5    |   0     |    0   |  pi/2  |
% elo 6 |   q6+pi  |  -195   |    0   |   pi   |
%
%
% Esse arquivo foi desenvolvido utilizando a toolbox de Robótica para 
%MATLAB (RVC toolbox).
%
%
% Referências:
%  - KUKA KR3 AGILUS R540:  https://www.kuka.com/pt-br/produtos-servi%C3%A7os/sistemas-de-rob%C3%B4/rob%C3%B4s-industriais/kr-3-agilus
%  - RVC Toolbox: http://www.petercorke.com
% 
% Por Gabriela Krieck.

classdef KUKA_KR3_R540  
    properties 
      KR3_R540
      Qm
    end
    
    methods
        
      %Construtor  
      function obj=KUKA_KR3_R540
      
         obj.KR3_R540 = SerialLink([Link('offset', 0,'d', -345,'a', 20,'alpha', pi/2,'qlim', [-170*pi/180 170*pi/180]);
         Link('offset', 0,'d', 0,'a', 260,'alpha', 0,'qlim', [-170*pi/180 50*pi/180]);
         Link('offset', -pi/2,'d', 0,'a', 20,'alpha', pi/2,'qlim',[-110*pi/180 155*pi/180]);
         Link('offset', -80*pi/180,'d', -260,'a', 0,'alpha', -pi/2,'qlim', [-175*pi/180 175*pi/180]);
         Link('offset', 0,'d', 0,'a', 0,'alpha', pi/2,'qlim',[-120*pi/180 120*pi/180]);
         Link('offset', pi,'d', -195.32,'a', 0,'alpha', pi,'qlim',[-350*pi/180 350*pi/180])], 'name', 'KUKA KR3 R540');
         
         obj.Qm = [0 -pi/2 pi/2 80*pi/180 0 0]; %Posição Master
      end
      
      function plotar(obj, Q)
      % PLOTAR apresenta uma representação gráfica do manipulador   
      %
      %     Parâmetros: 
      %     obj -> Parâmetro do tipo KUKA_KR3_R540.
      %     Q -> Vetor 1x6 contendo as posições desejadas para as 6 juntas
      %     do manipulador em radianos.
      %
      %
      %   PLOTAR(obj) plota o robô na posição 'master' definida pelo manual do manipulador. 
      %
      %   PLOTAR(obj, Q) plota o robô na posição Q, em radianos, definida pelo usuário.

        switch nargin
            case 2
                Qp=Q;
            case 1
                Qp=obj.Qm;
          otherwise
        	return
        end
            set(gca,'Zdir','reverse','Xdir','reverse')
            obj.KR3_R540.teach(Qp, 'notiles', 'floorlevel', 156, 'lightpos',[0 0 -20])
      end
      
      function [A1,A2,A3,A4,A5,A6] = CineD_simb(obj)
      % CineD_simb  retorna as matrizes de cinemática direta simbólicas do manipulador   
      %
      % 
      %     Parâmetros: 
      %     obj -> Parâmetro do tipo KUKA_KR3_R540.
      %     q1-6 -> Posição das juntas 1 a 6.
      %     A1-6 -> Matrizes 4x4 contendo as equações de cinemática direta
      %     do manipulador.
      %
      %   [A1,A2,A3,A4,A5,A6] = CineD_simb(obj) retorna as matrizes de 
      % cinemática direta simbólicas do manipulador
        
      syms q1 q2 q3 q4 q5 q6;
      
      %Param DH:
%       DH=[q1 -345 20 pi/2; 
%           q2 0 260 0; 
%           q3-pi/2 0 20 pi/2;
%           q4-80*pi/180 -260 0 -pi/2;
%           q5 0 0 pi/2;
%           q5+pi -195.32 0 pi];

    L1 = Link('offset', 0,      'd', -345,      'a', 20,   'alpha', pi/2, 'qlim', [-170*pi/180 170*pi/180]);
    L2 = Link('offset', 0,  'd', 0,         'a', 260,   'alpha', 0,     'qlim', [-170*pi/180 50*pi/180]);
    L3 = Link('offset', -pi/2,      'd', 0,         'a', 20,    'alpha', pi/2,  'qlim', [-110*pi/180 155*pi/180]);
    L4 = Link('offset', -80*pi/180,      'd', -260,      'a', 0,     'alpha', -pi/2, 'qlim', [-175*pi/180 175*pi/180]);
    L5 = Link('offset', 0,      'd', 0,         'a', 0,     'alpha', pi/2,  'qlim', [-120*pi/180 120*pi/180]);
    L6 = Link('offset', pi,      'd', -195,   'a', 0,     'alpha', pi,     'qlim', [-350*pi/180 350*pi/180]);
      
    A1=L1.A(q1);
    A1=A1.T;
    A1(1,2)=0;
    A1(2,2)=0;
    A1(3,3)=0;

    A2=L2.A(q2);
    A2=A2.T;
    A2(1,1)=sin(q2);
    A2(2,1)=-cos(q2);
    A2(1,2)=cos(q2);
    A2(2,2)=sin(q2);
    A2(1,4)=260*sin(q2);
    A2(2,4)=-260*cos(q2);

    A3=L3.A(q3);
    A3=A3.T;
    A3(1,2)=0;
    A3(2,2)=0;
    A3(3,3)=0;

    A4=L4.A(q4);
    A4=A4.T;
    A4(1,2)=0;
    A4(2,2)=0;
    A4(3,3)=0;

    A5=L5.A(q5);
    A5=A5.T;
    A5(1,2)=0;
    A5(2,2)=0;
    A5(3,3)=0;

    A6=L6.A(q6);
    A6=A6.T;
    A6(3,2)=0;
    A6(1,3)=0;
    A6(2,3)=0;
      end
    
      
      function M=CineD_num(obj,Q)
      % CineD_num retorna a matriz resultante de cinemática direta do manipulador
      %dado um vetor Q contendo as posições das juntas.
      %
      % 
      %     Parâmetros: 
      %     obj -> Parâmetro do tipo KUKA_KR3_R540.
      %     Q -> Vetor 1x6 contendo a posição das juntas 1 a 6 em radianos.
      %     M -> Matriz 4x4 que contém o resultado da cinemática direta
      %     do manipulador.
      %
      %   M=CineD_num(obj,Q) retorna a matriz resultante de cinemática 
      %direta do manipulador considerando um vetor Q contendo as posições 
      %das juntas.


        L1 = Link('offset', 0,      'd', -345,      'a', 20,   'alpha', pi/2, 'qlim', [-170*pi/180 170*pi/180]);
        L2 = Link('offset', 0,  'd', 0,         'a', 260,   'alpha', 0,     'qlim', [-170*pi/180 50*pi/180]);
        L3 = Link('offset', -pi/2,      'd', 0,         'a', 20,    'alpha', pi/2,  'qlim', [-110*pi/180 155*pi/180]);
        L4 = Link('offset', -80*pi/180,      'd', -260,      'a', 0,     'alpha', -pi/2, 'qlim', [-175*pi/180 175*pi/180]);
        L5 = Link('offset', 0,      'd', 0,         'a', 0,     'alpha', pi/2,  'qlim', [-120*pi/180 120*pi/180]);
        L6 = Link('offset', pi,      'd', -195.32,   'a', 0,     'alpha', pi,     'qlim', [-350*pi/180 350*pi/180]);
      
        A1=L1.A(Q(1,1));
        A1=A1.T;
        A1(1,2)=0;
        A1(2,2)=0;
        A1(3,3)=0;

        A2=L2.A(Q(1,2));
        A2=A2.T;
        A2(1,1)=sin(Q(1,2));
        A2(2,1)=-cos(Q(1,2));
        A2(1,2)=cos(Q(1,2));
        A2(2,2)=sin(Q(1,2));
        A2(1,4)=260*sin(Q(1,2));
        A2(2,4)=-260*cos(Q(1,2));

        A3=L3.A(Q(1,3));
        A3=A3.T;
        A3(1,2)=0;
        A3(2,2)=0;
        A3(3,3)=0;

        A4=L4.A(Q(1,4));
        A4=A4.T;
        A4(1,2)=0;
        A4(2,2)=0;
        A4(3,3)=0;

        A5=L5.A(Q(1,5));
        A5=A5.T;
        A5(1,2)=0;
        A5(2,2)=0;
        A5(3,3)=0;

        A6=L6.A(Q(1,6));
        A6=A6.T;
        A6(3,2)=0;
        A6(1,3)=0;
        A6(2,3)=0;
        
        M=A1*A2*A3*A4*A5*A6;
      end
      
      function Q=CineI(M)
      % CineI retorna o vetor Q contendo as posições das juntas dada a 
      % matriz resultante do manipulador
      %
      % 
      %     Parâmetros: 
      %     obj -> Parâmetro do tipo KUKA_KR3_R540.
      %     Q -> Vetor 1x6 contendo a posição das juntas 1 a 6 em radianos.
      %     M -> Matriz 4x4 que contém o resultado da cinemática direta
      %     do manipulador.   
          
        %Verifica se a matriz M é 4x4:
            if(size(M) ~= [4 4])
                error('Informe uma matriz de tamanho 4x4')
                return
            end
          
        %Links:
            L1 = Link('offset', 0,      'd', -345,      'a', 20,   'alpha', pi/2, 'qlim', [-170*pi/180 170*pi/180]);
            L2 = Link('offset', 0,  'd', 0,         'a', 260,   'alpha', 0,     'qlim', [-170*pi/180 50*pi/180]);
            L3 = Link('offset', -pi/2,      'd', 0,         'a', 20,    'alpha', pi/2,  'qlim', [-110*pi/180 155*pi/180]);
            
            L=[L1; L2; L3];
            
        %Parâmetros de interesse:  
            d6=[0 0 195]'; %Distância do centro do punho ao efetuador
            tM = M(1:3,4); %Submatriz de transição
            RM = M(1:3,1:3); %Submatriz de rotação
        
        %Primeira parte: Determinar a posiçao do centro do punho
        
         %Posição do centro do punho em termos das componentes em x, y e z:
            p0_aux = tM+RM*d6; 
        
         %Módulo da posição do centro do punho:
            p0 = sqrt(p0_aux(1,1)^2+p0_aux(2,1)^2+p0_aux(3,1)^2);  
            
        %Segunda parte: Determinar q1 q2 e q3 
        
            %Cálculo de q1:
             q1 = atan2(p0_aux(2,1),p0_aux(1,1))+pi;
 
            %Cálculo de q2:
            
            %Matriz de transformação da base pra junta 1:
                T01_aux = L1.A(q1);
            
            %Parâmetros de interesse:  
            
                %Determinar P01:
                T01_aux = [T01_aux.t(1,1); T01_aux.t(2,1); T01_aux.t(3,1)]; 
                p01 = sqrt(T01_aux(1,1)^2+T01_aux(2,1)^2+T01_aux(3,1)^2); %Módulo
            
                %Determinar P1W:
                p1w_aux = p0_aux-T01_aux;
                p1w = sqrt(p1w_aux(1,1)^2+p1w_aux(2,1)^2+p1w_aux(3,1)^2);
                
                %Determinar P2W:
                p2w = sqrt(260^2+20^2);
 
                %Determinar beta, gama e omega:
                beta = atan(345/20);
                psi = acos((p0^2-p01^2-p1w^2)/(-2*p01*p1w));
                omega = acos((-p1w^2-260^2+p2w^2)/(-2*260*p1w));
            
            %q2 será dado por:
                q2 = -(2*pi-(beta+psi+omega));
 
            %Cálculo de q3
                epsolon = acos((p1w^2-260^2-p2w^2)/(-2*260*p2w));
                varphi = atan(260/20);
                q3 = pi-epsolon-varphi;

           % Terceira Parte: Determinar q4, q5 e q6
                        
            T1 = L1.A(q1).T;
            T2 = L2.A(q2).T;
            T3 = L3.A(q3).T;
        
            T03 = (T1*T2*T3);
            t_T03 = T03(1:3,4); %Submatriz de transição
            R_T03 = T03(1:3,1:3); %Submatriz de rotação
            T36=[R_T03' -R_T03'*t_T03; 0 0 0 1];
             
            q4 = atan2(-T36(1,3),T36(2,3));
            q6 = atan2(T36(3,2),T36(3,1));
            q5 = atan2(M(3,2)/sin(q6),-T36(3,3));
 
            Q=[q1 q2 q3 q4 q5 q6];

      end
    
      
  end
end




