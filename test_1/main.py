from spider_leg import SpiderLeg
from plotting import plot_leg

# Initialize a spider leg with given name and segment lengths
leg = SpiderLeg("Leg1", COXA=50, FEMUR=100, TIBIA=100)

# Set the joint angles (in degrees) for the leg
leg.setAngles([0, 45, 60])

# Get the current joint angles
currentAngles = leg.getAngles()

# Get the current target position (x, y, z) of the leg tip
currentTarget = leg.getTarget()

# Define the new target
newTarget = [70, 70, -10]

# Calculate the joint angles required to reach a new target position using inverse kinematics
new_angles = leg.inverseKinematics(target=newTarget)

# Calculate the joint positions based on the joint angles using forward kinematics
joint_positions = leg.forwardKinematics()

# Print results
print("Current Joint Angles:", currentAngles)
print("Current Target Position:", currentTarget)
print("New Joint Angles:", new_angles)
print("New Joint Positions:", joint_positions)
print("Desired Target", newTarget, "Forward kinematics test", joint_positions[3])
print("Values in above arrays should be very close to each other")

plot_leg(joint_positions)
