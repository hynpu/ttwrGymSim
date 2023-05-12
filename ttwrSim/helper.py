import numpy as np

# wrape the angle to [-pi, pi]
def wrapToPi(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi
