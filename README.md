# Manipulador-SOLVER  

Manipulador robótico projetado para a limpeza de painéis solares.  

# Repositório do manipulador SOLVER – Dinâmica de Sistemas Robóticos

Este repositório contém o código e todos os arquivos desenvolvidos para o projeto final da disciplina SEM0590 – Dinâmica de Sistemas Robóticos (EESC-USP).  
Aqui estão incluídos os arquivos de modelagem, simulação, geração de trajetórias e análise dinâmica do manipulador.


---
# Estrutura do repositório

/src  
manipulador_dinamica.py

/cad  
cad.png  
cad1.png  
cano.SLDPRT  
conector_1.SLDPRT  
efetuador.SLDPRT  
elo.SLDPRT  
eloelo.SLDPRT  
base.SLDPRT  
Mont1.SLDASM  

/resultados  
cinemática direta até a posição [20° 20° 0.4].jpeg  
cinemática inversa até a posição [0.4 0.4 -0,4].jpeg  
trajetória de escovação.jpeg  


/documentação  
Relatório_de_Dinâmica (3).pdf    


---
# Descrição das pastas

- /src – Contém o código utilizado na modelagem, implementação das cinemáticas, simulação na Robotics Toolbox e execução de trajetórias.  
- /cad – Imagens e arquivos do projeto mecânico do manipulador.  
- /resultados – Gráficos da análise dinâmica (posição, velocidade, aceleração e torque) da cinemática direta, inversa e trajetória de escovação.  
- /documentação – Relatório final do projeto.


---
# Instruções para executar os códigos

## Pré-requisitos

- Python instalado  
- Bibliotecas Robotics Toolbox, Numpy e Matplotlib instaladas

## Passo a passo

1. Baixe ou clone este repositório
2. Abra o ambiente de Python e execute o código
3. Escolha no terminal entre cinemática direta, cinemática inversa, trajetória de escovação, limpar ou sair

## Funcionamento do programa

Ao executar o código, o usuário interage com o programa por meio do terminal, podendo selecionar diferentes modos de operação do manipulador robótico. Cada opção aciona uma rotina específica, responsável por calcular, simular e visualizar o comportamento do manipulador conforme descrito a seguir.

- cinemática direta: o usuário define a posição final das três juntas do manipulador. A partir desses valores, o programa calcula a pose do end-effector, gera os gráficos correspondentes e realiza a simulação do movimento  

- cinemática inversa: o usuário informa a posição cartesiana desejada para o end-effector. O programa calcula automaticamente as variáveis articulares necessárias, apresenta os gráficos e executa a simulação do movimento até a posição especificada  

- trajetória de escovação: o programa executa uma trajetória previamente planejada no espaço cartesiano, representativa do movimento de limpeza. São gerados os gráficos das grandezas articulares e a simulação completa do manipulador ao longo da trajetória  

- limpar: ao final de cada simulação, a posição final do manipulador é armazenada como condição inicial para a próxima execução. Essa opção redefine a posição inicial para o valor padrão q = [0° 0° 0], permitindo reiniciar as simulações a partir da configuração inicial  

- sair: encerra o loop de execução, finalizando o programa  

Observação: recomenda-se utilizar a opção sair após cada simulação, pois a execução de simulações de forma concatenada compromete o desempenho gráfico da animação.

---
# Integrantes do grupo do projeto

Amanda Hellen Venâncio Reis  
Gabriel Calori Badini  
Gabrielle de Moura Pereira  
João Augusto Costa Perino  

