# Seletiva_Embarcados_Robcin

main.cpp = codigo da entrega final 
readGyro.cpp = codigo que le o arquivo csv

Projeto de embarcados para a seletiva do robin

No repositório tem 2 códigos, um que é a versão que lê o log do csv (se for rodar lembra de trocar o path pra onde o arquivo tá) e o outro que testei no próprio sensor no robôcin.

O código, se substituído na main do ssl embedded, funciona (como testado quando fui no robôcin).

Preferi deixar tudo no mesmo código por simplicidade na hora de rodar.

Dependendo se for rodar no mpu do robô ou no externo que testei no dia que fui no robôcin, basta trocar a pinagem dos fios do ic2:

sda = pf_0,
scl = pf_1, isso no robô mesmo
sda = pb_9,
scl = pb_8, no sensor externo.
