# EmbarcaTechProjetoFinal
Projeto inicial de revisão do programa de capacitação EmbarcaTech - TIC 37 - Fase 2

# Vídeo Demonstração

https://youtu.be/mjIRMKBfFYA

# Hardware/Firmware

Projeto desenvolvido em uma placa de desenvolvimento BitDogLab, versão 6.3.<br>
Desenvolvimento de firmware feito através do PicoSDK, versão 2.1.1, com a IDE Visual Studio Code.

# Instruções

O projeto verifica o valor de dois potenciômetros, que correspondem ao Eixo X e Y do Joystick.<br>
Partindo de um dos potenciômetros como referencial (é possível definir qual está sendo utilizado) são emitidos sinais correspondentes.<br>
De acordo a distância do ponto central, o LED vermelho e os buzzers vão emitir um sinal. Quanto maior a distância, maior a intensidade.<br>
Na matriz de LEDs, o sinal do referencial é mostrado. Novamente, a referência de negativo ou positivo é o ponto central do Joystick.<br>
O display possui um quadrado de 8x8 pixels que demonstra a posição atual dos eixos do joystick.