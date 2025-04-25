from math import degrees, radians, cos, sin, sqrt, acos, atan, asin

class SpiderLeg:
    def __init__(self, name, COXA, FEMUR, TIBIA):
        self.name = name
        self.COXA = COXA
        self.FEMUR = FEMUR
        self.TIBIA = TIBIA
        self.theta1 = 0.  # Initially, set the angle to zero
        self.theta2 = 0.
        self.theta3 = 0.

        # Forward kinematics calculates the target position (x, y, z) based on the joint angles
        self.joints = self.forwardKinematics()

    def setAngles(self, angles):
        angles = self.normalizeAngles(angles)
        self.theta1, self.theta2, self.theta3 = angles
        return self.getAngles()

    def getAngles(self):
        return [self.theta1, self.theta2, self.theta3]

    def normalizeAngles(self, angles):
        for idx, ang in enumerate(angles):
            sign = 1
            if ang < 0:
                sign = -1
            angles[idx] = sign * (abs(ang) % 360)
            if abs(ang) > 180:
                angles[idx] = ang - (360 * sign)
        return angles

    def getTarget(self):
        return self.joints[3]

    def inverseKinematics(self, target=None):
        if target is None:
            target = self.joints[3]
        x, y, z = target[0], target[1], target[2]

        # Ensure that the movement is constrained along the y-axis
        x = 0  # Fix the x-coordinate to 0
        z = 0  # Fix the z-coordinate to 0

        # Now calculate theta1 based on y-coordinate
        theta1 = atan(y / 0.001)  # Small value to avoid division by zero, since x=0

        Xa = self.COXA * cos(theta1)
        Ya = self.COXA * sin(theta1)

        # Calculate intermediate values for theta2 and theta3
        P = sqrt((y - Ya) ** 2)  # Calculate distance along the y-axis
        G = abs(z)

        H = sqrt(P ** 2 + G ** 2)

        phi3 = asin(G / H)
        phi2Acos = ((self.TIBIA ** 2) + (H ** 2) - (self.FEMUR ** 2)) / (2 * self.TIBIA * H)
        phi2 = acos(phi2Acos)

        phi1 = acos((self.FEMUR ** 2 + H ** 2 - self.TIBIA ** 2) / (2 * self.FEMUR * H))

        if y > 0:
            theta2 = phi1 + phi3
        else:
            theta2 = phi1 - phi3

        theta3 = phi1 + phi2

        ang = [degrees(theta1), degrees(theta2), degrees(theta3)]
        self.setAngles(ang)
        self.forwardKinematics()
        return ang

    def forwardKinematics(self, angles=None):
        if angles is None:
            angles = [radians(self.theta1), radians(self.theta2), radians(self.theta3)]
        theta1, theta2, theta3 = angles

        Xa = self.COXA * cos((theta1))
        Ya = self.COXA * sin((theta1))

        G2 = sin(theta2) * self.FEMUR
        P1 = cos(theta2) * self.FEMUR
        Xc = cos(theta1) * P1
        Yc = sin(theta1) * P1

        H = sqrt(self.TIBIA ** 2 + self.FEMUR ** 2 - (2 * self.TIBIA * self.FEMUR * cos(radians(180) - theta3)))
        phi1 = acos((self.FEMUR ** 2 + H ** 2 - self.TIBIA ** 2) / (2 * self.FEMUR * H))
        phi2 = radians(180) - (radians(180) - theta3) - phi1
        phi3 = (phi1 - theta2)
        Pp = cos(phi3) * H
        P2 = Pp - P1
        Yb = sin(theta1) * Pp
        Xb = cos(theta1) * Pp
        G1 = sin(phi3) * H
        G1 = -G1

        # Keep the x and z coordinates fixed
        jointLocation = [
            [0, 0, 0],  # Fixed origin
            [Xa, Ya, 0],  # coxa-femur joint
            [Xa + Xc, Ya + Yc, G2],  # femur-tibia joint
            [Xa + Xb, Ya + Yb, G1]  # tip of the leg
        ]

        self.joints = jointLocation
        return jointLocation
