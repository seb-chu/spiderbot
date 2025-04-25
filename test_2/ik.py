import math

# Constants (should be defined based on your robot's geometry)
PI = math.pi
Y_Rest = 10  # Example value for Y rest position offset
Z_Rest = 5   # Example value for Z rest position offset
J2L = 100    # Length of joint 2 (for example)
J3L = 100    # Length of joint 3 (for example)
J3_LegAngle = 90  # Example value for leg angle offset

# Function to perform Cartesian move based on X, Y, and Z input positions
def CartesianMove(X, Y, Z):
    # OFFSET TO REST POSITION
    Y += Y_Rest
    Z += Z_Rest

    # CALCULATE INVERSE KINEMATIC SOLUTION
    J1 = math.atan(X / Y) * (180 / PI)
    
    # Calculate H and L for further joint angles
    H = math.sqrt((Y * Y) + (X * X))
    L = math.sqrt((H * H) + (Z * Z))

    # Solve for J3 using the inverse cosine function
    J3 = math.acos(((J2L * J2L) + (J3L * J3L) - (L * L)) / (2 * J2L * J3L)) * (180 / PI)
    
    # Calculate angle B for J2
    B = math.acos(((L * L) + (J2L * J2L) - (J3L * J3L)) / (2 * L * J2L)) * (180 / PI)
    
    # Angle A is calculated from Z and H (vertical component)
    A = math.atan(Z / H) * (180 / PI)

    # Calculate J2
    J2 = (B + A)  # Because 'A' is negative at rest, we add it to B to get J2

    print(f"inital position (x, y, z): {J1}, {J2}, {J3}")

    # Call the UpdatePosition function with the calculated joint angles
    UpdatePosition(J1, J2, J3)

# Function to update the position of the joints (you can replace with hardware interface code)
def UpdatePosition(J1, J2, J3):
    # Here, we simulate the movement of the joints (replace with actual code to control your joints)
    print(f"Moving to Position: Joint1: {90 - J1}, Joint2: {90 - J2}, Joint3: {J3 + J3_LegAngle - 90}")
    # Replace with your hardware-specific commands for controlling the servos or motors

    # Uncomment to debug
    # print(f"J1: {J1}, J2: {J2}, J3: {J3}")

# Example usage
X = 50  # Example X coordinate
Y = 75  # Example Y coordinate
Z = 100  # Example Z coordinate

CartesianMove(X, Y, Z)
