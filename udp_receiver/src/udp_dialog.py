import rospy
import socket
import threading


def receive_messages(sock):
    """
    Função para receber mensagens via socket UDP.
    """
    while not rospy.is_shutdown():
        try:
            # Recebe os dados do socket
            data, sender_address = sock.recvfrom(1024)
            message = data.decode('utf-8')
            print(f"Mensagem recebida de {sender_address}: {message}")
        except Exception as e:
            rospy.logerr(f"Erro ao receber mensagem: {e}")


def main():
    # Inicializa o nó ROS
    rospy.init_node("udp_receiver_sender", anonymous=True)

    # Configura o socket UDP para receber mensagens
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_socket.bind(("0.0.0.0", 8080))
    rospy.loginfo("Socket UDP vinculado à porta 8080 e aguardando mensagens...")

    # Inicia a thread para receber mensagens
    receiver_thread = threading.Thread(target=receive_messages, args=(udp_socket,))
    receiver_thread.daemon = True
    receiver_thread.start()

    # Configura o socket e o endereço remoto para enviar mensagens
    remote_ip = "192.168.4.22"
    remote_port = 8080

    while not rospy.is_shutdown():
        try:
            # Solicita ao usuário que digite uma mensagem
            message = input("Digite uma mensagem para enviar (ou 'exit' para sair): ")

            if message.lower() == "exit":
                break  # Encerra o loop se o usuário digitar "exit"

            # Envia a mensagem para o endereço remoto
            udp_socket.sendto(message.encode('utf-8'), (remote_ip, remote_port))
            rospy.loginfo(f"Mensagem enviada: {message}")
        except Exception as e:
            rospy.logerr(f"Erro ao enviar mensagem: {e}")

    # Fecha o socket antes de encerrar
    udp_socket.close()
    rospy.loginfo("Socket fechado. Programa encerrado.")


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

