# Robotica
Conectar ao robô via UDP através do wifi gerado pelo robô.
Se quiser utilizar internet, conectar via cabo.
Conectar à rede UDP_comm.
Por DHCP ele só disponibiliza 192.168.4.23 e fica como 162.168.4.22.
Funciona no port 8080.

Lançar o ROS pelo o udp_dialog.cpp.
Dar comandos pelo UDP de acordo com a tabela nos anexos do TCC.
Sempre um comando com um número independente de o número ser útil ou não.

Modificar forma como os comandos do cmd_vel são recebidos e reenviados como textos UDP.
