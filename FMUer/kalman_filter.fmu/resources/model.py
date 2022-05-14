from os import access
import pickle
import math
from xml.sax.handler import DTDHandler
import numpy as np
from scipy.integrate import odeint

#Reference: https://github.com/f1tenth/f1tenth_simulator/blob/master/src/st_kinematics.cpp

class Model:
    def __init__(self) -> None:

        self.x_s = 0.
        self.y_s = 0.
        self.steer_angle_s = 0.
        self.velocity_s = 0.
        self.theta_s = 0.
        self.u = [0., 0.]
        self.acceleration = 0.
        self.steer_angel_velocity = 0.

        self.reference_to_attribute = {
            0: "x_s",           
            1: "y_s",
            2: "steer_angle_s",
            3: "velocity_s",
            4: "theta_s",            
            5: "acceleration",
            6: "steer_angel_velocity",
            7: "x_p",
            8: "y_p",
        }

        #references to the inputs and outputs 
        self.references_input = [0, 1, 2, 3, 4, 5, 6]
        self.references_output = [7, 8]

        #getting the values 
        input = self.fmi2GetReal(self.references_input)

        self.x_s = input[1][0]
        self.y_s = input[1][1]
        self.steer_angle_s = input[1][2]
        self.velocity_s = input[1][3]
        self.theta_s = input[1][4]
        self.meas = [self.x_s, self.y_s, self.steer_angle_s, self.velocity_s, self.theta_s]

        self.acceleration = input[1][5]
        self.steer_angel_velocity = input[1][6]
        self.u = [self.acceleration, self.steer_angel_velocity]

        f = lambda x, t, u: np.array([x[3]*math.cos(x[6] + x[4]), 
            x[3]*math.sin(x[6] + x[4]), 
            u[0], 
            u[1], 
            x[5], 
            0, 
            0])

        h = lambda x: x[0:5]

        n = 7
        G = np.eye(n)
        Q = np.diag([0.1*0.1, 0.1*0.1, 0.01*0.01, 0.1*0.1, 0.1*0.1, 0.01*0.01, 0.1*0.1])
        R = np.diag([0.04*0.04, 0.04*0.04, 0.04*0.04, 0.04*0.04, 0.04*0.04])
        x0 = np.array([0, 0, 0, 0, 0, 0, 0 ])
        x0_est = np.array([0, 0, 0, 0, 0, 0, 0])
        P0 = np.diag([0.1*0.1, 0.1*0.1, 0.01*0.01, 0.1*0.1, 0.1*0.1, 0.01*0.01, 0.1*0.1])
        Q = Q + np.diag([10**-4, 10**-4, 10**-9, 10**-9, 10**-9, 10**-9, 10**-9])

        self.x_p = None
        self.P = None
    

        # Sigma Point Kalman Filterinput[1][0]
        alpha = 0.0001
        kappa = 0.0
        beta = 2.0
        sut = SUT(alpha, beta, kappa, n)
        self.filter = SPKF(f,h,G,Q,R,x0_est,P0,sut,variant=0)  # 0 - normal UKF, 1 - IUKF, 2 - UKFz

    
    #------PROCESS IN TIME------ 
    def fmi2DoStep(self, current_time, step_size, no_step_prior):

        self.x_p, self.P = self.filter.predict(self.u, step_size,1)
        self.filter.update(self.meas,1)
        
        #updating the output values
        self.fmi2SetReal(self.references_output, (self.x_p[0], self.x_p[1]))

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


    
    #------UPDATING THE STATE------
class SUT: 
    """ Scaled Unscented Transform """

    def __init__(self, alpha, beta, kappa, n):
        
        self.alpha = alpha
        self.beta = beta
        self.kappa = kappa
        self.n = n

        self.l = (self.alpha**2)*(self.n+self.kappa) - self.n # lambda parameter 

        # Init weights (constants) 
        self.W0m = self.l/(self.n + self.l)
        self.W0c = self.W0m + 1 - self.alpha**2 + self.beta
        self.Wi = 0.5/(self.n + self.l)
        """ sigma points weights """
    
    def create_points(self, x, P):
        sr_P = np.linalg.cholesky((self.n + self.l)*P)
        S = [x]
        for i in range(self.n):
            S.append(x+sr_P[:,i])
            S.append(x-sr_P[:,i])
        return S
##########################################################

#class CDT:       
#  """ Central Difference Transform """
##########################################################

class SPKF:
    """ Sigma Point Kalman Filter """

    def __init__(self,f,h,G,Q,R,x0,P0,spt,variant=0):

        self.f = f
        """ Continous state dynamics; dot(x) =  f(x,u) """

        self.h = h
        """ Measurement function y = h(x) """

        self.G = G
        """ Noise input matrix """

        self.x = x0
        """ Initial State """

        self.P = P0
        """ Initial Covariance """

        self.Q = Q
        """ Propagation noise matrix """

        self.R = R
        """ Measurement noise matrix """

        self.spt = spt
        """ Sigma point transform and parameters """

        self.n = x0.size
        """ State dimensionality """

        self.variant = variant # 0 - normal UKF, 1 - IUKF, 2 - UKFz

    def predict(self,u,dt,simple = 1):

        S = self.spt.create_points(self.x, self.P)
        # TODO: pass f1 and f2 to swich based of off velocity state
        # propagate the points 
        Sp = [ ]
        for i in range(len(S)):
            if (simple):
                # Euler faster
                Sp.append(S[i]+self.f(S[i],0,u)*dt)
            else:
                # ODE Int integration to make the state tranzition
                Y = odeint(self.f,S[i],np.array([0, dt]),args=(u,))
                Sp.append(Y[1]) # Y[0]=x(t=t0)
     
        # calculate the mean, covariance and cross-covariance of the set
        Xm = self.spt.W0m*Sp[0]
        for i in range(2*self.n):
            Xm = Xm + self.spt.Wi*Sp[i+1]

        Cx = self.spt.W0c*np.outer(Sp[0]-Xm,Sp[0]-Xm)+dt*self.Q
        for i in range(2*self.n):
            Cx = Cx + self.spt.Wi*np.outer(Sp[i+1]-Xm,Sp[i+1]-Xm)

        self.x = Xm
        self.P = Cx
        return self.x, self.P

    def update(self,x_s,meas,var=0):

        X = self.spt.create_points(x_s, self.P)
        
        Y = [ ]
        for i in range(2*self.n+1):
            Y.append(self.h(X[i]))

        # calculate the mean and covariance of the set
        if (self.variant == 2): # UKFz
            Ym = self.h(self.x)
        else:  
            Ym = self.spt.W0m*Y[0]
            for i in range(2*self.n):
                Ym = Ym + self.spt.Wi*Y[i+1]
        
        Cy = self.spt.W0c*np.outer(Y[0]-Ym,Y[0]-Ym)
        Cxy = self.spt.W0c*np.outer(X[0]-self.x,Y[0]-Ym)
        for i in range(2*self.n):
            Cy = Cy + self.spt.Wi*np.outer(Y[i+1]-Ym,Y[i+1]-Ym)
            Cxy = Cxy + self.spt.Wi*np.outer(X[i+1]-self.x,Y[i+1]-Ym)

        # Kalman Gain
        K = Cxy@np.linalg.inv(Cy + self.R)

        # Update mean and covarince
        if (self.variant == 1): # IUKF
            inn = meas - self.h(self.x)
        else:
            inn = meas - Ym

        self.x = self.x + K@inn
        
        if (var == 0):
            # Simple Covariance Update
            self.P = self.P - K@(Cy+self.R)@np.transpose(K)
        else:
            # -> Joseph Form Covariance Update
            # H = stochastic linearization derived from the fact that Cxy = PH' in the linear case [Skoglund, Gustafsson, Hendeby - 2019]
            self.H = Cxy.transpose()@np.linalg.inv(self.P)
            IKH =  np.eye(self.n) - K@self.H
            self.P = IKH@self.P@IKH.transpose()+K@self.R@K.transpose() # Joseph Form
##########################################################


#------Represents the status of the FMU or the results of function calls------
class Fmi2Status:
    """
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
    m = Model()
    m.acceleration = 3.0
    m.steer_angle_vel = 0.0
    m.fmi2DoStep(0.0, 4.0, False)
    print(m.x)
    print(m.y)
    print(m.theta)
    print(m.angular_velocity)
    print(m.slip_angle)
    print(m.velocity)
    print(m.steer_angle)
"""