import numpy as np
from math import sin, cos

class Controller3D():
    """
    This class computes the commanded thrusts (N) to be applied to the quadrotor plant.

    You are to implement the "compute_commands" method.
    """
    def __init__(self, cfparams, pid_gains, dt):
        """
        Inputs:
        - cfparams (CrazyflieParams dataclass):     model parameter class for the crazyflie
        - pid_gains (dict):                         pid gains

        N.B. pid_gains is a dictionary structure where the keys are 'kp_x', 'kd_z', etc.
        """
        self.params = cfparams
        
        # PID gains

        self.kp_x = pid_gains["kp_x"]
        self.kp_y = pid_gains["kp_y"]
        self.kp_z = pid_gains["kp_z"]
        #-----
        self.ki_x = pid_gains["ki_x"]
        self.ki_y = pid_gains["ki_y"]
        self.ki_z = pid_gains["ki_z"]
        #-----
        self.kd_x = pid_gains["kd_x"]
        self.kd_y = pid_gains["kd_y"]
        self.kd_z = pid_gains["kd_z"]
        #-----

        # rotational 
        self.kp_phi = pid_gains["kp_phi"]
        self.kp_theta = pid_gains["kp_theta"]
        self.kp_psi = pid_gains["kp_psi"]
        #-----
        self.ki_phi = pid_gains["ki_phi"]
        self.ki_theta = pid_gains["ki_theta"]
        self.ki_psi = pid_gains["ki_psi"]
        #-----
        self.kd_p = pid_gains["kd_p"]
        self.kd_q = pid_gains["kd_q"]
        self.kd_r = pid_gains["kd_r"]

        self.E_last_phi   = 0
        self.E_last_theta = 0
        self.E_last_psi   = 0

        self.E_accum_x = 0
        self.E_accum_y = 0
        self.E_accum_z = 0

    def compute_commands(self, setpoint, state, time_delta):
        """
        Inputs:
        - setpoint (TrajPoint dataclass):   the desired control setpoint
        - state (State dataclass):          the current state of the system
        Returns:
        - U (np.array):     array of control inputs {u1-u4}

        N.B. TrajPoint is a new dataclass. Please check it out from the utils.py script
        """
        U = np.array([0.,0.,0.,0.])

        # P - Errors: E
        E_x = setpoint.x_pos - state.x_pos
        E_y = setpoint.y_pos - state.y_pos
        E_z = setpoint.z_pos - state.z_pos

        # I - Acculmulated E
        self.E_accum_x += E_x
        self.E_accum_y += E_y
        self.E_accum_z += E_z

        # D - Error derivative: E_dot
        E_dot_x = setpoint.x_vel - state.x_vel
        E_dot_y = setpoint.y_vel - state.y_vel
        E_dot_z = setpoint.z_vel - state.z_vel


        # PIDs
        setpoint.x_acc = self.kd_x*E_dot_x + self.kp_x*E_x + self.ki_x*self.E_accum_x
        setpoint.y_acc = self.kd_y*E_dot_y + self.kp_y*E_y + self.ki_y*self.E_accum_y
        setpoint.z_acc = self.kd_z*E_dot_z + self.kp_z*E_z + self.ki_z*self.E_accum_z

        # Angles
        phi_d = (1/self.params.g)*(setpoint.x_acc*sin(state.psi) - setpoint.y_acc*cos(state.psi))
        theta_d = (1/self.params.g)*(setpoint.x_acc*cos(state.psi) + setpoint.y_acc*sin(state.psi))
        psi_d = state.psi

        # Angle errors
        E_phi = float(phi_d - state.phi)
        E_theta = float(theta_d - state.theta)
        E_psi = float(psi_d - state.psi)

        # Angle errors dot (angular velocity) ?????
        E_dot_phi   = float((E_phi - self.E_last_phi)/time_delta)
        E_dot_theta = float((E_theta - self.E_last_theta)/time_delta)
        E_dot_psi   = float((E_psi - self.E_last_psi)/time_delta)

        # Control inputs
        U[0] = self.params.mass*(setpoint.z_acc+self.params.g)
        U[1] = self.kd_p*E_dot_phi + self.kp_phi*E_phi
        U[2] = self.kd_q*E_dot_theta + self.kp_theta*E_theta
        U[3] = self.kd_r*E_dot_psi + self.kp_psi*E_psi

        return U
