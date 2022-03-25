from asyncio import selector_events
import pickle

#Reference: https://github.com/f1tenth/f1tenth_simulator/blob/master/node/simulator.cpp

class Controller:
    def __init__(self) -> None:
        
        #constants
        self.max_accel = 7.51
        self.max_decel = 8.26
        self.max_speed = 7.0
        self.max_steering_vel = 3.2 
        self.max_steering_angle = 0.4189

        #parameters
        self.desired_velocity = 0.0
        self.desired_angle = 0.0

        self.kp = 0.0       #proportional gain 

        #inputs 
        self.velocity = 0.0
        self.steer_angle = 0.0 

        #other variables
        self.previous_time = 0.0 
        self.actual_ang = 0.0      


        #reference to xml file 
        self.reference_to_attribute = {
            0: "desired_velocity",
            1: "desired_angle", 
            2: "kp",
            3: "velocity",
            4: "steer_angle",
            5: "output_acceleration",
            6: "output_steer_angle_vel",
        }

    
    #------progress in time------ 
    def fmi2DoStep(self, current_time, step_size, no_step_prior):

        self.control_speed_and_angle()
        
        return Fmi2Status.ok
    

    #------controller logic------
    def control_speed_and_angle(self):
        
        #calculating and setting the new acceleration 
        self.compute_accel()

        #compute and setting new steer angular velocity 
        if self.actual_ang != 0.0:
            self.actual_ang = 0.0
        else:
            self.actual_ang = self.desired_angle
        
        self.compute_steer_vel()


        return Fmi2Status.ok


    #------setters and getters------
    def fmi2SetReal(self, references, values):
        return self._set_value(references, values)


    

    #------acceleration ------ 
    def compute_accel(self):
        #get difference of velocity
        diff = self.desired_velocity - self.velocity

        #determine acceleration or braking based on difference between velocities
        if self.velocity > 0:
            if diff > 0:
                #accelerate
                self.set_accel(self.kp * diff)

            else:
                #brake
                self.output_acceleration = -self.max_decel
        
        elif self.velocity < 0:
            if diff > 0:
                #brake
                self.output_acceleration = self.max_decel
            
            else:
                #acceleration
                self.set_accel(self.kp * diff)
        
        else:
            #acceleration 
            self.set_accel(self.kp * diff)


    def set_accel(self, accel):
        self.output_acceleration = min(max(accel, -self.max_accel), self.max_accel)



    #------angle velocity------
    def compute_steer_vel(self):
        #difference between angles
        diff_angle = self.desired_angle - self.steer_angle
        
        # calculate velocity
        if (abs(diff_angle) > .0001):  # if the difference is not trivial
            self.steer_vel = diff_angle / abs(diff_angle) * self.max_steering_vel
        else:
            self.steer_vel = 0

        self.set_steer_angle_vel(self.steer_vel)


    def set_steer_angle_vel(self, steer_angle_vel):
        self.output_steer_angle_vel = min(max(steer_angle_vel, -self.max_steering_vel), self.max_steering_vel)



    #------ serialize/deserialize????------
    """def fmi2ExtSerialize(self):

        bytes = pickle.dumps(
            (
                self.controller_input,
            )
        )
        return Fmi2Status.ok, bytes


    def fmi2ExtDeserialize(self, bytes) -> int:
        (
            controller_input,
        ) = pickle.loads(bytes)

        self.controller_input = controller_input

        return Fmi2Status.ok
    """



class Fmi2Status:
    """Represents the status of the FMU or the results of function calls.

    Values:
        * ok: all well
        * warning: an issue has arisen, but the computation can continue.
        * discard: an operation has resulted in invalid output, which must be discarded
        * error: an error has ocurred for this specific FMU instance.
        * fatal: an fatal error has ocurred which has corrupted ALL FMU instances.
        * pending: indicates that the FMu is doing work asynchronously, which can be retrived later.

    Notes:
        FMI section 2.1.3

    """

    ok = 0
    warning = 1
    discard = 2
    error = 3
    fatal = 4
    pending = 5


if __name__ == "__main__":
    m = Controller()

    #parameters
    m.desired_velocity = 4.5
    m.desired_angle = 0.2

    #inputs
    m.velocity = 2.0
    m.steer_angle = 0.1

    #do steo
    m.fmi2DoStep(0.0, 1.0, False)


    #outputs
    print(m.output_acceleration)
    print(m.output_steer_angle_vel)
    print(m.previous_time)

    