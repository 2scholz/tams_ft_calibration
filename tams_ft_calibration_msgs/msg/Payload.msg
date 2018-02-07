# one mass-point as known sensor (tool) payload: this
# contains the mass [kg] and the offset from the sensor
# frame [m] to calculate effective torque.
# and torques by subtracting the known tool weight from 
# the measured values.
#
float64 mass
geometry_msgs/Vector3 offset
