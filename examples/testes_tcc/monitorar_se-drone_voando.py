##########DEPENDENCIES#############

from dronekit import connect,VehicleMode,LocationGlobalRelative,APIException
#import dronekit
import time
import socket
import math
import argparse
from pymavlink import mavutil
import dronekit_sitl
################Outros imports############
import threading
from click import exceptions
vehicle = None
toTakeOff = None
monTakeOff = None
#########FUNCTIONS#################

def connectMyCopter():
    global vehicle
    try:

        parser = argparse.ArgumentParser(description='commands')
        parser.add_argument('--connect')
        args = parser.parse_args()

        connection_string = args.connect

        if not connection_string:
            import dronekit_sitl
            sitl = dronekit_sitl.start_default()
            connection_string = sitl.connection_string()

        vehicle = connect(connection_string,wait_ready=True, timeout=5)

    except socket.error:
        print ('SEM SERVIDOR SITL EXISTENTE!')
    except exceptions.OSError as e:
        print ('SEM COMINICAÇÃO SERIAL EXISTENTE!')
    except dronekit.APIException:
        print ('TEMPO DE CONEXAO EXCEDIDO!')
    except:
        print ('ACONTEREAM OUTROS ERROS DE CONEXAO!')
    if vehicle == None:
        return {vehicle, False}
        #return{'vehicle':vehicle,'state': False}
    else:
        return {vehicle, True}
        #return{'vehicle':vehicle,'state': True}


def detect_face():
    arming = vehicle._armed
    print("ARMADO = ",arming)
    return None


##################MAIN#################################
if __name__ == "__main__":

    a, b= connectMyCopter()
    print(a)
    print(b)
    #print(c)
    # print("Veiculo ", vehicle)
    # print("CONECTADO= ",state)

    # param_veic= "{s}".format (s= vehicle['state'])
    # state_veic= "vehicle = {v}".format (v=vehicle['vehicle'])
    #stringSaida= "Quadrado = {q} , Cubo = {c} ".format (q=resultado['quadrado'], c= resultado['cubo'])
    #a = vehicle[0]
    #print(vehicle)

    #print("CONEXAO: ",param_veic)
    #print(state_veic)
    # cmds = vehicle.commands
    # cmds.download()
    # cmds.wait_ready()
    # print(cmds)
    # print(vehicle)

    # arming = vehicle._armed
    # toTakeOff = vehicle.location.global_relative_frame.alt
    # monTakeOff = vehicle.location.global_relative_frame.alt > toTakeOff
    # print("ARMADO = ",arming)
    # print("ALTITUDE ATUAL ",vehicle.location.global_relative_frame.alt)
    # print("ALTITUDE INICIAL ",toTakeOff)
    # print(monTakeOff)


    count = 5
    while True:

        if a == True:
            startSystem = threading.Thread(target=detect_face)
            startSystem.start()
            print("conectou")
        elif a == False:
            print("O VEICULO NAO ESTA PRONTO")
        time.sleep(1)
        # count+=1




    # if arming == True :
    #     print("ARMADO")
    #     pass
    # else:
    #     print("NAO ESTA ARMADO")
    #     pass

