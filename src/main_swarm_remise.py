import math
import time
import matplotlib.pyplot as plt
import numpy as np

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm


#nombre de drones
nb_cf=2

# URI to the Crazyflie to connect to
uri_1 = 'radio://0/80/2M/E7E7E7E701'
uri_2='radio://0/80/2M/E7E7E7E702'

uris={uri_1, uri_2}

def wait_for_position_estimator(scf):
    print('Waiting for estimator to find position...')

    log_config = LogConfig(name='Kalman Variance', period_in_ms=100)
    log_config.add_variable('kalman.varPX', 'float')
    log_config.add_variable('kalman.varPY', 'float')
    log_config.add_variable('kalman.varPZ', 'float')

    var_y_history = [1000] * 10
    var_x_history = [1000] * 10
    var_z_history = [1000] * 10

    threshold = 0.001

    with SyncLogger(scf, log_config) as logger:
        for log_entry in logger:
            data = log_entry[1]

            var_x_history.append(data['kalman.varPX'])
            var_x_history.pop(0)
            var_y_history.append(data['kalman.varPY'])
            var_y_history.pop(0)
            var_z_history.append(data['kalman.varPZ'])
            var_z_history.pop(0)

            min_x = min(var_x_history)
            max_x = max(var_x_history)
            min_y = min(var_y_history)
            max_y = max(var_y_history)
            min_z = min(var_z_history)
            max_z = max(var_z_history)


            if (max_x - min_x) < threshold and (
                    max_y - min_y) < threshold and (
                    max_z - min_z) < threshold:
                break


def reset_estimator(scf):
    cf = scf.cf
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')

    wait_for_position_estimator(cf)

def flight_data_callback(timestamp, data, logconf):
    global roll
    global pitch
    global yaw
    global thrust

    roll_d=data['stabilizer.roll']
    pitch_d = data['stabilizer.pitch']
    yaw_d = data['stabilizer.yaw']
    thrust_d=data['stabilizer.thrust']

    roll.append(roll_d)
    pitch.append(pitch_d)
    yaw.append(yaw_d)
    thrust.append(thrust_d)

    print('pos:_({},_{},_{})'.format(roll_d,pitch_d,yaw_d,thrust_d))

def print_flight_data(scf):
    lg_stab = LogConfig(name='Stabilizer', period_in_ms=100)
    lg_stab.add_variable('stabilizer.roll', 'float')
    lg_stab.add_variable('stabilizer.pitch', 'float')
    lg_stab.add_variable('stabilizer.yaw', 'float')
    lg_stab.add_variable('stabilizer.thrust', 'float')

    cf = Crazyflie(rw_cache='./cache')

    scf.cf.log.add_config(lg_stab)
    lg_stab.data_received_cb.add_callback(flight_data_callback)
    lg_stab.start()

def position_callback(timestamp, data, logconf):
    global x_pos
    global y_pos
    global z_pos


    x=data['kalman.stateX']
    y = data['kalman.stateY']
    z = data['kalman.stateZ']

    x_pos=np.append(x_pos,x)
    y_pos = np.append(y_pos, y)
    z_pos = np.append(z_pos, z)
    print('pos:_({},_{},_{})'.format(x,y,z))

def start_position_printing(scf):
    log_conf=LogConfig(name='Position', period_in_ms=100)
    log_conf.add_variable('kalman.stateX','float')
    log_conf.add_variable('kalman.stateY', 'float')
    log_conf.add_variable('kalman.stateZ', 'float')

    scf.cf.log.add_config(log_conf)
    log_conf.data_received_cb.add_callback(position_callback)
    log_conf.start()

def landing(cf,last_pos):
    print("Landing!")
    time.sleep(1)
    vz=0.2
    cf.commander.send_velocity_world_setpoint(0.0, 0.0, -vz, 0.0)

    if (last_pos[2]*10/vz)<50:
        for i in range(50):
            cf.commander.send_hover_setpoint(0.0, 0.0, 0.0, 0)
            time.sleep(0.1)
    else:
        for i in range(last_pos[2]*10/vz):
            cf.commander.send_hover_setpoint(0.0, 0.0, 0.0, 0)
            time.sleep(0.1)

def take0ff(cf,pos_init):
    print("Take off")
    time.sleep(1)
    take_off_time=1
    take_off_height=0.5
    vz = take_off_height/take_off_time

    for i in range(int(take_off_time/0.1)):
        cf.commander.send_velocity_world_setpoint(0.0, 0.0, vz, 0.0)
        time.sleep(0.1)

def run_sequence_hover(scf, hover_pos,pos_initiale):
    print("starting hover sequence")
    cf = scf.cf

    #Hover sequence
    for position in hover_pos:
        print('Setting position {}'.format(position))
        time.sleep(2)

        for i in range(100):
            cf.commander.send_hover_setpoint(0.0, 0.0, 0.0, pos_initiale[2]+position[2])
            time.sleep(0.1)

    #Landing
    last_pos=(pos_initiale[0], pos_initiale[1], pos_initiale[2]+hover_pos[-1][2])
    land_time=1
    vzl=last_pos[2]/land_time
    time.sleep(0.5)

    for i in range(int(land_time / 0.1)):
        cf.commander.send_velocity_world_setpoint(0.0, 0.0, -vzl, 0.0)
        time.sleep(0.1)

    cf.commander.send_setpoint(0, 0, 0, 0)
    time.sleep(0.5)

    cf.commander.send_stop_setpoint()
    time.sleep(0.5)


if __name__ == '__main__':
    cflib.crtp.init_drivers(enable_debug_driver=False)

    x_pos=[]
    y_pos=[]
    z_pos=[]
    roll=[]
    pitch=[]
    yaw=[]
    thrust=[]
    position_initiale=[]

    #Calcul de position intiale
    for i in uris:
        with SyncCrazyflie(i, cf=Crazyflie(rw_cache='./cache')) as scf:
            reset_estimator(scf)
            start_position_printing(scf)
            time.sleep(6)
            scf.close_link()
            initial_x = np.mean(x_pos)
            initial_y = np.mean(y_pos)
            initial_z = np.mean(z_pos)
            position_initiale.append((initial_x,initial_y,initial_z))
            x_pos = []
            y_pos = []
            z_pos = []

    print('got initial position')
    time.sleep(2)

    # Paramètre séquence vol stationnaire
    hover_sequence = [(0.0, 0.0, 0.5)]
    params_hover = {uri_1: [hover_sequence, position_initiale[0]], uri_2: [hover_sequence, position_initiale[1]]}

    #Run Sequence
    factory= CachedCfFactory(rw_cache='./cache')
    with Swarm(uris, factory=factory) as swarm:

        #get position
        swarm.parallel(reset_estimator)
        time.sleep(1)

        #flight data printing
        print_flight_data(scf)

        #position printing
        swarm.parallel(start_position_printing)
        time.sleep(5)

        #Hover sequence
        swarm.parallel(run_sequence_hover, args_dict=params_hover)
        time.sleep(1)
        swarm.close_links()

    #Visualisation de la position
    #Arranger positions des nb_cf drones
    x_fin = np.zeros((nb_cf,int(len(x_pos)/nb_cf)))
    y_fin = np.zeros((nb_cf,int(len(y_pos)/nb_cf)))
    z_fin = np.zeros((nb_cf,int(len(z_pos)/nb_cf)))

    for i in range(0,nb_cf):
        x_fin[i,:]=x_pos[i::2]
        y_fin[i, :] = y_pos[i::2]
        z_fin[i, :] = z_pos[i::2]

    print("link closed")
    time.sleep(1)

    #Print position
    for j in range(0,nb_cf):
        a = np.arange(0, ((len(x_fin[0])) / 2), 0.5)
        plt.plot(a, x_fin[j,:], label="coordonnée x cf %d" %(i+1))
        plt.plot(a, y_fin[j,:], label="coordonnée y cf %d" %(i+1))
        plt.plot(a, z_fin[j,:], label="coordonnée z cf %d" %(i+1))

        plt.xlabel('temps')
        plt.ylabel('Position')
        plt.title('Coordonnées dans le temps')
        plt.legend()
        plt.grid()
        plt.show()

