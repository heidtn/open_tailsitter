import numpy as np
from collections import namedtuple


TailsitterConfig = namedtuple('TailsitterConfig', ['fmin', 'fmax',
                                                   'dmin', 'dmax',
                                                   'm', 'Jxx', 'Jyy', 'Jzz',
                                                   'p', 'd',
                                                   'bx', 'by', 'bz', 'cx', 'cz',
                                                   'l', 'k', 'tp', 'ksip', 'ta', 'tw',
                                                   'kp', 'kl', 'kd'])

# fr, fl are right and left motor thrust values (Newtons)
# and dl, dr are left and right flap angles (radians)
TailsitterActuators = namedtuple('TailsitterActuators', ['dl', 'dr', 'fl', 'fr'])

def load_config(filename):
    # Accepts a path to a json file and returns a tailsitter_config
    pass


class TailsitterDynamics:
    """
    VTOL tailsitter dynamics.  Based on the work done by IDSC.

    NOTES:
    - suffixes B and I signify body frame and inertial frame.  Velocity/position is in inertial frame
      unless otherwise specified and rotation/omega is in body frame unless otherwise specified.
    """
    def __init__(self, config, initial_pos=None, initial_rot=None, g=9.8):
        self.cfg = config
        self.g = np.array([0, 0, -g])
        self.Jinv = np.diag([1.0 / config.Jxx, 1.0 / config.Jyy, 1.0 / config.Jzz])
        self.J = np.diag([config.Jxx, config.Jyy, config.Jzz])

        self.p = 
        self.v = 
        self.R = 
        self.omega = 

    def get_dynamics(self, u):
        # u is a TailsitterActuators
        # dt is the time since the last update

        fB_tot = self._get_fB_tot(u)
        mB_tot = self._get_mB_tot(u)

        # v is in inertial frame
        v_dot = 1.0 / self.cfg.m * (np.dot(self.R.T, fB_tot) + g)
        # omega is in body frame
        omega_dot = np.dot(self.Jinv, mB_tot - np.cross(self.omega, np.dot(self.J, self.omega)))

        return v_dot, omega_dot

    def _get_mB_tot(self, u):
        mB_tot = self._get_mB_air(u) + self._get_mB_prop(u)
        return mB_tot
    
    def _get_mB_air(self, u):
        vB = np.dot(self.R, self.v)
        alpha = np.arctan2(vB[1], vB[2])
        v = np.sqrt(vB[1]**2.0 + vB[2]**2.0)

        vB_lz = self._get_vB_sz(u.fl, vB[2])
        vB_rz = self._get_vB_sz(u.fr, vB[2])

        v_l = np.sqrt((vB[1]**2.0 + vB_lz**2.0))
        v_r = np.sqrt((vB[1]**2.0 + vB_rz**2.0))
        bx, by, bz, cx, cz = self.cfg.bx, self.cfg.by, self.cfg.bz, self.cfg.cx, self.cfg.cz
        mB_airx = (bx + cx * u.dl) * v_l**2.0 + (bx + cx * u.dr) * v_r**2.0 + self._get_m_pitch(alpha, v, u)
        mB_airy = by * v_l**2.0 - by * v_r**2.0
        mB_airz = (bz + cz * u.dl) * v_l**2.0 - (bz + cz * u.dr) * v_r**2.0
        mB_air = np.array([mB_airx, mB_airy, mB_airz])
        return mB_air

    def _get_m_pitch(self, alpha, v, u):
        return self.cfg.kp * np.sin(alpha) * v**2.0

    def _get_vB_sz(self, fs, vB_z):
        return np.sqrt((2 * fs) / (self.cfg.p * self.cfg.d) + np.max([0, vB_z])**2.0)

    def _get_mB_prop(self, u):
        mB_prop = np.array([0, l * (u.fr - u.fl), k * (u.fr - u.fl)])
        return mB_prop

    def _get_fB_tot(self, u):
        fB_tot = self._get_fB_air(u) + self._get_fB_prop(u)
        return fB_tot

    def _get_fB_air(self, u):
        vB = np.dot(self.R, self.v)
        alpha = np.arctan2(vB[1], vB[2])
        v = np.sqrt(vB[1]**2.0 + vB[2]**2.0)
        kl = self.cfg.kl
        kd = self.cfg.kd
        f_a = (u.fr + u.fl) / 2.0
        f_lift = (kl[0] * np.sin(alpha) * np.cos(alpha)**2.0 + kl[1]*np.sin(alpha)**3.0) * v**2.0 + kl[2] * f_a 
        f_drag = (kd[0] * np.sin(alpha)**2.0 * np.cos(alpha) + kd[1]*np.cos(alpha)) * v**2.0 + kd[2] * f_a
        fB_air = np.array([0, f_lift, -f_drag])
        retrun fB_air

    def _get_fB_prop(self, u):
        f_a = (u.fr + u.fl) / 2.0
        return np.array([0, 0, 2 * f_a])

