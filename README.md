# KUKA_KR3_R540

**KUKA_KR3_R540** é um script desenvolvido no software MATLAB  que cria um modelo para descrever as características cinemáticas  do manipulador KUKA AGILUS KR3 R540 considerando como efetuador uma garra elétrica paralela FESTO HGPLE-14-30-3, 1-DC-VCSC-G96.

O modelo foi desenvolvido com auxílio da **toolbox de Robótica para  MATLAB** (RVC toolbox) e conta com a implementação das cinemáticas direta e inversa do manipulador bem como com um protótipo do modelo.


## Modelo
O modelo do manipulador é descrito pelos parâmetros de Denavit-Hartenberg dados  por

Elo | q_i  | d_i [mm] | a_i [mm] | alpha_i |
:---------: | :---------:  | :------: | :------: | :------: 
1 | q_1 | -345   |   20   |  pi/2  |
2 | q_2 |   0     |  260   |    0   |
3 | q_3-pi/2  |   0     |   20   |  pi/2  |
4 | q_4-4pi/9|  -260   |    0   | -pi/2   |
5 | q_5 |   0     |    0   |  pi/2  |
6 | q_6+pi |  -195   |    0   |   pi  |


## Métodos

Foram implementados métodos que realizam a representação gráfica do manipulador, bem como as cinemáticas direta (simbólica e numérica) e inversa do manipulador. O arquivo *main.m* contém exemplos de utilização dos métodos implementados  na classe **KUKA_KR3_R540**

### KUKA_KR3_R540 - Construtor do modelo do manipulador

O construtor cria um novo objeto do tipo *KUKA_KR3_R540* de acordo com os parâmetros de Denavit-Hartenberg já definidos. Para criar um novo objeto do tipo *KUKA_KR3_R540* faça:

``` matlab
MeuRobo=KUKA_KR3_R540
```

o construtor incializa uma variável *Qm* contendo as posições das juntas do manipulador quando o mesmo está na *configuração master*, definida pelo fabricante e também a variável *KR3_R540* do tipo *SerialLink* que contém o modelo do robô.

### plotar(obj, Q) - Representação gráfica do manipulador

**PLOTAR** apresenta uma representação gráfica do manipulador, realizando a configuração do ambiente de simulação.


 Parâmetros: 
 - **obj**: Parâmetro do tipo KUKA_KR3_R540.
 - **Q**: Vetor 1x6 contendo as posições desejadas para as 6 juntas do manipulador em radianos.



**PLOTAR(obj)** plota o robô na posição 'master' definida pelo manual do manipulador. 

**PLOTAR(obj, Q)** plota o robô na posição Q, em radianos, definida pelo usuário.

Para plotar o robô na posição master faça:
``` matlab
%Plota Robô na posição master:
plotar(MeuRobo)
```

Para plotar o robô na posição Q faça:
``` matlab
 %Plota Robô na posição Q:
 plotar(MeuRobo, Q)
```
### CineD_simb(obj) - Equações de cinemática direta simbólicas do manipulador
 
 **CineD_simb**  retorna as matrizes de cinemática direta simbólicas do manipulador   

Parâmetros: 
- obj -> Parâmetro do tipo KUKA_KR3_R540.
- q1-6 -> Posição das juntas 1 a 6.
- A1-6 -> Matrizes 4x4 contendo as equações de cinemática direta do manipulador.

Utilização:

Para encontrar as matrizes transição de estados que descrevem cada uma das juntas do manipulador faça:

``` matlab
[A1 A2 A3 A4 A5 A6]=CineD_simb(MeuRobo);
```

Se, por exemplo, desejar encontrar a matriz transição de estados **T** que descreve as equações do sistema de referência da base ao efetuador, faça:

``` matlab
T=simplify(A1*A2*A3*A4*A5*A6);
```

### CineD_simb(obj) - Equações de cinemática direta numéricas do manipulador
 **CineD_num** retorna a matriz resultante da cinemática direta do manipulador dado um vetor Q contendo as posições das juntas.

Parâmetros: 
- obj -> Parâmetro do tipo KUKA_KR3_R540.
- Q -> Vetor 1x6 contendo a posição das juntas 1 a 6 em radianos.
- M -> Matriz 4x4 que contém o resultado da cinemática direta do manipulador.

**M=CineD_num(obj,Q)** retorna a matriz resultante de cinemática direta do manipulador considerando um vetor Q contendo as posições das juntas.
 
 Utilização:
 
 Para encontrar a matriz resultante da cinemática direta do manipulador faça:
 
 ``` matlab
Q = [0 -pi/2 pi/2 80*pi/180 0 0] %Vetor Q 1x6 qualquer
M=CineD_num(MeuRobo, Q);
```

Onde **Q** representa um vetor 1x6 contendo as posições das 6 juntas do manipulador.


# Requisitos
- Software MATLAB 
- Toolbox de robótica para Matlab 10.2.1 (RVC toolbox) 

# Consulte também
- KUKA KR3 AGILUS R540:  https://www.kuka.com/pt-br/produtos-servi%C3%A7os/sistemas-de-rob%C3%B4/rob%C3%B4s-industriais/kr-3-agilus
 - RVC Toolbox: http://www.petercorke.com

# Autores 
- Artur H. Lauth
- Gabriela A. Krieck
