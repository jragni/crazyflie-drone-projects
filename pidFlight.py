import cflib
from cflib.crazyflie import Crazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils.multiranger import Multiranger
import time

################# Purpose ###################
# pidFlight.py will control the flight of the crazyflie such that it will be within 20 cm +/- 0.05 of the object in front of it
# and then land.



URI = 'radio://0/80/2M'



class forwardControlFlight():


    def __init__(self):
        #PID constants  --- needs to be tuned 
        self.Kp = .8  # Proportion control constant
        self.Ki = 0.001  # Integral control constant
        self.Kd = 300  # Derivative control constant
        self.distance_to_rotor = 0.05  # distance of the range sensor to the tip of the rotor

        # initialize the crazyflie API
        cflib.crtp.init_drivers(enable_debug_driver=False)
        self.cf = Crazyflie(ro_cache=None, rw_cache='cache')

        # connect to crazyfle
        with SyncCrazyflie(URI, cf=self.cf) as self.scf:
            print('Crazyflie is connected')

            # initialize the Multiranger
            with Multiranger(self.cf) as self.MR:
                # take off
                time.sleep(.01)

                with MotionCommander(self.scf) as self.MC:
                    # Allow crazyflie time to take-off and stabilize
                    time.sleep(1)
                    self.move()



    def frontSensorHandler(self):
        front = self.MR._front_distance
        while(front == None):
            time.sleep(.01)  # wait until the ranger is responsive
            front = self.MR._front_distance
        return front



    def move(self):

        errorTolerance = 0.05  # precision of distance to wall
        x_desired = 0.2  # [m] distance from wall
        currentDistance = self.frontSensorHandler()
        error = currentDistance - x_desired
        previousError = 0  # initialize previous error
        I = 0  # initialize integral control
        count = 0  # used to limit print
        arrive = False
        while(abs(error) >= errorTolerance):

            error = self.frontSensorHandler() - x_desired -self.distance_to_rotor

            P = self.Kp * error  # proportional control
            I += (self.Ki * error)

            D = self.Kd * (error - previousError)
            PID = P + I + D
            previousError = error
            if (count == 5):
                print( "P:",P," I:", I," D:",  D,"V:", PID,"Error: ", error)

                count = 0
            if PID > 1:
                PID = 1  # set speed limit to .8 [m/s]


            self.MC.start_linear_motion(PID, 0, 0)
            count +=1
            time.sleep(.005)
        print('final error: ',error)
        print('Arrived')
        self.MC.land()




forwardControlFlight()