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


        
def altitudeFaceDetect():
    print("TESTE")
    return None



def ifArming():
    targetHeight = 3.000
    arming = vehicle._armed
    #stateTakeOff = vehicle.location.global_relative_frame.alt > toTakeOff
    print("INICIO ARMADO = ",arming)
    while arming == True:
        arming = vehicle._armed
        print(vehicle.location.global_relative_frame.alt)
        print(.92*targetHeight)
        if vehicle.location.global_relative_frame.alt>=.92*targetHeight:
            altitudeFaceDetect()
        print("JA ARMADO = ",arming)

        time.sleep(1)



##################MAIN#################################
if __name__ == "__main__":

    state_connection,vehicle= connectMyCopter()
    print(state_connection)
    print(vehicle)
    #print(c)
    # print("Veiculo ", vehicle)
    # print("CONECTADO= ",state)

    # param_veic= "{s}".format (s= vehicle['state'])
    # state_veic= "vehicle = {v}".format (v=vehicle['vehicle'])
    #stringSaida= "Quadrado = {q} , Cubo = {c} ".format (q=resultado['quadrado'], c= resultado['cubo'])
    #a = vehicle[0]
    #print(vehicle)
    #print(state_veic)
    # cmds = vehicle.commands
    # cmds.download()
    # cmds.wait_ready()
    # print(cmds)
    # print(vehicle)
    startAlt = vehicle.location.global_relative_frame.alt
    print(vehicle.location.global_relative_frame.alt)
    arming = vehicle._armed
    # toTakeOff = vehicle.location.global_relative_frame.alt
    # monTakeOff = vehicle.location.global_relative_frame.alt > toTakeOff
    print("VAI ARMAR = ",arming)
    # print("ALTITUDE ATUAL ",vehicle.location.global_relative_frame.alt)
    # print("ALTITUDE INICIAL ",toTakeOff)
    # print(monTakeOff)
    print(state_connection)
    if state_connection == True:
        if arming == True:
            ifArming()
        print("conectou")
    elif state_connection == False:
        print("O VEICULO NAO ESTA PRONTO")
        time.sleep(1)





