# Importa o publish do paho-mqtt
import paho.mqtt.publish as publish
ip="192.168.1.113"
# Publica
#publish.single("topico/teste", "Oi, aqui é um teste", "192.168.1.113", port=1883)
publish.single("topic/","teste" , hostname="localhost", port=1883)
