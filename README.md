# Robotica Computacional - Projeto 1
Repositório para o Projeto 1 da disciplina de Robótica Computacional.

Alunos:
* Henry Rocha Furquim
* Vitor Grando Eller

Nosso objetivo é elaborar um código que controle um robo, e atinja os seguintes objetivos:

* Reconhecer um objeto através do uso de uma rede neural (no caso a escolhida foi a `MobileNet`) e reagir à isso.
  * O objeto escolhido foi uma garrafa qualquer.
  * A reação escolhida foi a de centralizar o objeto, e, quando centralizado, avançar em sua direção.
* Reconhecer um objeto pela sua cor e reagir à isso.
  * A reação escolhida foi fazer com que o robo entre em estado de "choque", girando loucamente.
* Sobreviver
  * Para isso, implementamos leituras do laser e dos bumpers do robo, reagindo a cada estímulo de uma maneira específica.

Para um vídeo com a demonstração dessas funcionalidades, [clique aqui](https://drive.google.com/file/d/10rfVnfP8Ng_la8jPCZcYbqJGMaa8yrsl/view?usp=sharing)