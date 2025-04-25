from spider_leg import SpiderLeg
import matplotlib.pyplot as plt
from real_time_plotting import animate_leg

# Initialize a spider leg with given name and segment lengths
leg = SpiderLeg("Leg1", COXA=50, FEMUR=100, TIBIA=100)

# Set the joint angles (in degrees) for the leg
leg.setAngles([30, 45, 60])

# Define the new target
newTarget = [70, 70, -10]

# Create the figure for plotting
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Animate the movement of the leg from the starting position to the new target
animate_leg(leg, ax, fig, target=newTarget, steps=100)
