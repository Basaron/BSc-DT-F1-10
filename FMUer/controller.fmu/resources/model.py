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

        self.kp_speed = 0.5       #speed proportional gain
        self.kp_angle = 0.5       #angle proportional gain 

        #inputs 
        self.velocity = 0.0
        self.steer_angle = 0.0 
        self.distance = 0.0
        self.angle = 0.0

        #outputs 
        self.acceleration = 0.0 
        self.steer_angle_vel = 0.0

        #reference to xml file 
        self.reference_to_attribute = {
            0: "desired_velocity",
            1: "desired_angle", 
            2: "velocity",
            3: "steer_angle",
            4: "acceleration",
            5: "steer_angle_vel",
            6: "kp_speed",
            7: "kp_angle",
            8: "distance",
            9: "angle",
        }

        #references to the inputs and outputs 
        self.references_input = [2, 3, 8, 9]
        self.references_output = [4, 5]

        #getting the values 
        input = self.fmi2GetReal(self.references_input)

        self.velocity = input[1][0]
        self.steer_angle = input[1][1]
        self.distance = input[1][2]
        self.angle = input[1][3]
       

    
    #------PROCESS IN TIME------ 
    def fmi2DoStep(self, current_time, step_size, no_step_prior):
        self.control_speed_and_angle()
        return Fmi2Status.ok


    #------INITIALIZATION------ 
    def fmi2EnterInitializationMode(self):
        return Fmi2Status.ok

    def fmi2ExitInitializationMode(self):
        return Fmi2Status.ok


    #------SETUP EXPERIMENT------ 
    def fmi2SetupExperiment(self, start_time, stop_time, tolerance):
        return Fmi2Status.ok
    
    def fmi2Reset(self):
        return Fmi2Status.ok

    def fmi2Terminate(self):
        return Fmi2Status.ok


    #------SETTERS------ 
    def fmi2SetReal(self, references, values):
        return self._set_value(references, values)

    def fmi2SetInteger(self, references, values):
        return self._set_value(references, values)

    def fmi2SetBoolean(self, references, values):
        return self._set_value(references, values)

    def fmi2SetString(self, references, values):
        return self._set_value(references, values)
    
    def _set_value(self, references, values):

        for r, v in zip(references, values):
            setattr(self, self.reference_to_attribute[r], v)

        return Fmi2Status.ok
    

    #------GETTERS------ 
    def fmi2GetReal(self, references):
        return self._get_value(references)

    def fmi2GetInteger(self, references):
        return self._get_value(references)

    def fmi2GetBoolean(self, references):
        return self._get_value(references)

    def fmi2GetString(self, references):
        return self._get_value(references)
    
    def _get_value(self, references):

        values = []

        for r in references:
            values.append(getattr(self, self.reference_to_attribute[r]))

        return Fmi2Status.ok, values


    #------CONTROLLER LOGIC------
    def control_speed_and_angle(self):
        #calculating and setting the new acceleration 
        self.compute_acceleration()
        self.compute_steer_angle_vel()

        print("distance = ", self.distance, "angle = ", self.angle)
        
        #setting the output values 
        self.fmi2SetReal(self.references_output, (self.acceleration, self.steer_angle_vel))

        return Fmi2Status.ok
    

    #------acceleration ------ 
    def compute_acceleration(self):
        #get difference of velocity
        diff = self.desired_velocity - self.velocity

        #determine acceleration or braking based on difference between velocities
        if self.velocity > 0:
            if diff > 0:
                #accelerate
                self.set_accel(self.kp_speed * diff)

            else:
                #brake
                self.acceleration = -self.max_decel
        
        elif self.velocity < 0:
            if diff > 0:
                #brake
                self.acceleration = self.max_decel
            
            else:
                #acceleration
                self.set_accel(self.kp_speed * diff)
        
        else:
            #acceleration 
            self.set_accel(self.kp_speed * diff)
        
        #inserting the acceleration into output list
        #self.outputs.append(self.acceleration)


    def set_accel(self, accel):
        self.acceleration = min(max(accel, -self.max_accel), self.max_accel)
    

    #------angle velocity------
    def compute_steer_angle_vel(self):
        #difference between angles
        diff_angle = self.desired_angle - self.steer_angle
        
        # calculate velocity
        if (abs(diff_angle) > .0001):  # if the difference is not trivial
            steer_vel = diff_angle * self.kp_angle
        else:
            steer_vel = 0

        self.set_steer_angle_vel(steer_vel)

        #inserting the angle velocity into output list
        #self.outputs.append(self.steer_angle_vel)


    def set_steer_angle_vel(self, steer_angle_vel):
        self.steer_angle_vel = min(max(steer_angle_vel, -self.max_steering_vel), self.max_steering_vel)






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


#------TEST CASES------
"""
if __name__ == "__main__":
    m = Controller()
    #parameters
    m.desired_velocity = 4.5
    m.desired_angle = 0.2
    #inputs
    m.velocity = 2.0
    m.steer_angle = 0.1
    #do step
    m.fmi2DoStep(0.0, 1.0, False)
    #outputs
    print(m.output_acceleration)
    print(m.output_steer_angle_vel)
"""