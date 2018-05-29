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

# Requisitos
- Software MATLAB 
- Toolbox de robótica para Matlab 10.2.1 (RVC toolbox) 

# Consulte também
- KUKA KR3 AGILUS R540:  https://www.kuka.com/pt-br/produtos-servi%C3%A7os/sistemas-de-rob%C3%B4/rob%C3%B4s-industriais/kr-3-agilus
 - RVC Toolbox: http://www.petercorke.com

# Autores 
- Artur H. Lauth
- Gabriela A. Krieck
