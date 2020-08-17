#!/usr/bin/env python

# author Eleni Zapridou

import glob
import os
import sys

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

import random
import time
import rtamt
import math

# rtamt Monitor for a pair of vehicles
class Monitor:

    REQUIREMENT = "R" # set the requirement to be checked by the monitor

    # initializer
    def __init__(self, rearVehicle, frontVehicle):
        self.rearVehicle = rearVehicle # rear vehicle
        self.frontVehicle = frontVehicle # front vehicle
        self.lastChecked = -1 # keep track of the last event checked
        self.events = [] # events' list

        # store signals to write in files
        self.distances = []
        self.SDs = []
        self.robs = []

        # Discrete time monitor with input and output variables
        self.spec = rtamt.STLIOSpecification(1)
        self.spec.name = 'AccMonitor'

        # F: front car
        # R: rear car
        self.spec.declare_var('dist', 'float')
        self.spec.declare_var('SD', 'float')
        self.spec.declare_var('rob', 'float')
        self.spec.declare_var('accF', 'float')
        self.spec.declare_var('accR', 'float')
        self.spec.declare_var('vF', 'float')
        self.spec.declare_var('vR', 'float')
        self.spec.declare_var('trafficLightR', 'int')

        self.spec.set_var_io_type('SD', rtamt.StlIOType.IN)
        self.spec.set_var_io_type('rob', rtamt.StlIOType.OUT)
        self.spec.set_var_io_type('accF', rtamt.StlIOType.IN)
        self.spec.set_var_io_type('accR', rtamt.StlIOType.OUT)
        self.spec.set_var_io_type('vF', rtamt.StlIOType.IN)
        self.spec.set_var_io_type('vR', rtamt.StlIOType.OUT)
        self.spec.set_var_io_type('trafficLightR', rtamt.StlIOType.IN)

        # set the requirement the monitor will check
        if (REQUIREMENT == "R"):
            self.spec.set_var_io_type('dist', rtamt.StlIOType.OUT)
            self.spec.spec = 'rob = (dist - SD >= 0)'
        elif (REQUIREMENT == "R1"):
            self.spec.set_var_io_type('dist', rtamt.StlIOType.IN)
            self.spec.spec = 'rob = (vF - vR < 1.5) or (dist > 100) or trafficLightR==1'
        else:
            self.spec.set_var_io_type('dist', rtamt.StlIOType.IN)
            self.spec.spec = 'rob = (accF <= -5.2) or (always[0:3] (accR > -5.2 or trafficLightR==1))'


        try:
            self.spec.parse()
        except rtamt.STLParseException as err:
            print('STL Parse Exception: {}'.format(err))
            sys.exit()


    # calculate the safety distance
    def calculateSD(self, vF, vR):
        max_brake = 9.8
        min_brake = 2.9
        max_accel = 5.4
        SD = vR*0.05 + max_accel * 0.00125 - vF**2 / (2*max_brake) + (vR + 0.05*max_accel)**2 / (2*min_brake)
        if (SD < 3):
            SD = 3 # minimum distance between two cars
        return SD


    # update the monitor with the new events
    def check(self):
        toCheckFrom = self.lastChecked + 1
        toCheckTo = len(self.events)
        for i in range(toCheckFrom, toCheckTo): # for each one of the new events
            event = self.events[i]
            vehicleF = self.frontVehicle
            vehicleR = self.rearVehicle
            dist = event['distance']
            vF = vehicleF.get_velocity()
            speedF_m_s = event['vF'] #in m/s
            speedR_m_s = event['vR'] #in m/s
            if (i == 0) :
                accR = 0
                accF = 0
            else:
                speedR_prev = self.events[i-1]['vR']
                accR = (speedR_m_s - speedR_prev) / 0.05
                speedF_prev = self.events[i-1]['vF']
                accF = (speedF_m_s - speedF_prev) / 0.05

            SD = self.calculateSD(speedF_m_s, speedR_m_s)

            timestamp = event['timestamp']

            trafficR = vehicleR.is_at_traffic_light()

            rob = self.spec.update(timestamp, [('dist', dist), ('SD', SD), ('accF', accF), ('accR', accR), ('vF', speedF_m_s), ('vR', speedR_m_s), ('trafficLightR', trafficR)])

            # store signals
            self.distances.append(dist)
            self.SDs.append(SD)
            self.robs.append(rob)

        self.lastChecked = toCheckTo - 1


    # register a new event
    def register_event(self, distance, timestamp):
        vR = self.rearVehicle.get_velocity()
        speedR_m_s = math.sqrt(vR.x**2 + vR.y**2 + vR.z**2)
        vF = self.frontVehicle.get_velocity()
        speedF_m_s = math.sqrt(vF.x**2 + vF.y**2 + vF.z**2)
        event = {
            'distance' : distance,
            'timestamp' : timestamp,
            'vR': speedR_m_s,
            'vF': speedF_m_s
        }
        self.events.append(event)


# controls all the monitors and creates new ones if needed
class MonitorsController:

    def __init__(self):
        self.sensorDic = {} # dictionary that returns the vehicle a sensor is attached to (sensor : vehicle)
        self.follow = {} # dictionary that stores all the monitors (rearVehicle : {'frontVehicle': frontVehcileId, 'monitor': monitor})


    def setSensorDictionary(self, sensorDic):
        self.sensorDic = sensorDic


    # register a new obstacle event
    def register_obstacle(self, obstacle_detect):
        frontVehicle = obstacle_detect.other_actor
        # only interested if the obstacle is a vehicle
        if not (frontVehicle.type_id.startswith('vehicle.')):
            return
        rearVehicle = self.sensorDic.get(obstacle_detect.actor.id)

        # if rearVehicle already exists in the follow dictionary then register the event to the according monitor
        if rearVehicle.id in self.follow and self.follow.get(rearVehicle.id)['frontVehicle'] == frontVehicle.id:
            self.follow.get(rearVehicle.id)['monitorR1'].register_event(obstacle_detect.distance, obstacle_detect.timestamp)
        else:
            # else create new monitor and add it to follow
            monitorR1 = Monitor(rearVehicle, frontVehicle)
            self.follow[rearVehicle.id] = {'frontVehicle': frontVehicle.id, 'monitorR1': monitorR1}
            monitorR1.register_event(obstacle_detect.distance, obstacle_detect.timestamp)


    # check all the new events of every monitor
    # called n every step of the simulation
    def check_monitors(self):
        for vehicleId in self.follow:
            self.follow.get(vehicleId)['monitorR1'].check()


    # write the stored lists into files
    def writeFiles(self):
        for vehicleId in self.follow:
            frontVehicleId = self.follow.get(vehicleId)['frontVehicle']
            fileString = "R1_" + str(vehicleId) + "_" + str(frontVehicleId) + ".csv"

            monitor = self.follow.get(vehicleId)['monitorR1']
            distances = monitor.distances
            SDs = monitor.SDs
            robs = monitor.robs

            with open(fileString, "w") as f:
                f.write("Distance,SD,RobustnessR1\n")
                for i in range(len(accFs)):
                    f.write("%f,%f,%f\n" % (distances[i],SDs[i],robs[i]))



def main():
    actor_list = []
    sensorDic = {} #dictionary {sensor: vehicle}

    try:
        # Set the client
        client = carla.Client('localhost', 2000)
        client.set_timeout(2.0)

        # Set the synchronous mode
        world = client.get_world()

        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05
        world.apply_settings(settings)

        # Set weather
        blueprint_library = world.get_blueprint_library()

        weather = carla.WeatherParameters(
            cloudyness=1.0,
            sun_azimuth_angle=180.0,
            sun_altitude_angle=60.0
            )

        world.set_weather(weather)

        monitorsCtrl = MonitorsController()

        # Spawn the cars
        transform = world.get_map().get_spawn_points()[0]

        for i in range(0, 2):

            vehicle_types = ['vehicle.audi.tt', 'vehicle.seat.leon', 'vehicle.seat.leon', 'vehicle.seat.leon']
            # choose a vehicle
            bp = blueprint_library.find(vehicle_types[i])

            if (i>0):
                transform.location.x -= 20.0
            vehicle = world.try_spawn_actor(bp, transform)

            if vehicle is not None:
                actor_list.append(vehicle)
                print('created {} {}'.format(vehicle.type_id, vehicle.id))
                vehicle.set_autopilot(True)
                vehicle.set_acc(True)

                # Add sensor.other.collision
                sensor_bp = blueprint_library.find('sensor.other.obstacle')
                sensor_bp.set_attribute('distance', '200')
                sensor_bp.set_attribute('only_dynamics', 'true')
                # if it equals fixed_delta_seconds then sensor ticks every tick of the world. Increase to sample
                sensor_bp.set_attribute('sensor_tick', '0.05')
                sensor_transform = carla.Transform(carla.Location(x=1.0, z=1.0))
                sensor = world.spawn_actor(sensor_bp, sensor_transform, attach_to=vehicle)
                actor_list.append(sensor)
                print('created {} {}'.format(sensor.type_id, sensor.id))
                sensorDic.update( {sensor.id : vehicle} )

                # Register the listener that will be called for each sensor tick
                sensor.listen(lambda obstacle_detect: monitorsCtrl.register_obstacle(obstacle_detect))

        monitorsCtrl.setSensorDictionary(sensorDic)
        print('vehicles spawned, press Ctrl+C to exit.')

        while True:
            world.tick()
            monitorsCtrl.check_monitors()

    finally:

        print('destroying actors')
        for actor in actor_list:
            actor.destroy()

        monitorsCtrl.writeFiles()
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        world.apply_settings(settings)
        print('done.')


if __name__ == '__main__':

    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('\ndone.')
