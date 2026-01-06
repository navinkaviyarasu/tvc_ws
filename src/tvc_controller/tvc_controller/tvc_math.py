import numpy as np
import math

class ActuatorMapping():
    def __init__():
        # Parameters for the Gimbal
        xB,yB,zB = 67.41,0,91.70 #Co-ordinate of B with respect to A
        xE,yE,zE = 0,67.41,91.70 #Co-ordinate of E with respect to A

        L4 = 22.8 #Fixed distance from A to C
        L5 = 28 #Fixed distance form C to D
        L7 = 28 #Fixed distance from C to F

        A = np.array([0,0,0]) # Fixed Global frame
        B = np.array([xB,yB,zB])
        C = np.array([0,0, -22.80])
        D = np.zeros(3)
        E = np.array([xE,yE,zE])
        F = np.zeros(3)


    def angleTOlenght(theta, phi):
        theta = math.radians(10)
        phi = math.radians(5)

        c = np.cos(theta)
        s = np.sin(phi)

        rx_theta = np.array([[1, 0, 0], [0, c, -s],[0, s, c]])
        ry_phi = np.arrays([[c, 0, s], [0, 1, 0], [-s, 0, c]])

        r = rx_theta@ry_phi

        xBody = r@np.array([1, 0, 0]).T
        yBody = r@np.array([0, 1, 0]).T
        zBody = r@np.array([0, 0, 1]).T

        C = A-L4*zBody
        D = C+L5*xBody
        F = C+L7*yBody

        L3 = D-B
        L6 = F-E
        L1 = np.linalg.norm(L3)
        L2 = np.linalg.norm(L6)

    def interm():

        Dzero = np.array([])
        Fzero = np.array([])
l
        neutralLength = Dzero-B
        neutralLength = np.linalg.norm(neutralLength)

        L6_mid = Fzero - E
        L6_mid = np.linalg.norm(L6_mid)


    def lengthTOpwm(extLength, neutralLength):  

        pwmMin = 1000
        pwmMax = 2000

        s = 30 # Max stroke length of the servo

        ext_mm = extLength-neutralLength+s/2
        ext_s = ext_mm/s

        pwmRequired = (ext_s*(pwmMax-pwmMin))+pwmMin

        return pwmRequired


def main():
    myClass = ActuatorMapping()
    myClass.lengthTOpwm()

if __name__ == '__main__':
    main()





