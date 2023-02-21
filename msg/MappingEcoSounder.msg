# Command an absolute yaw and velocity.

#Header header
# Yaw is specified in radians counter-clockwise from true east.
#float32 yaw

# Velocity is specified in meters/s. Negative values correspond to Heron
# reversing.
#float32 speed

uint32 iTOW		# Time of week reportado por el GPS
float64 posX		# Posición ECEF del GPS, presición RTK
float64 posY
float64 posZ
float32 heading		# Àngulo de Heading reportado por el GPS
float32 alfa		# Àngulo de Roll de la ecosonda. 0º es Vertical
float32 rango		# Distancia medida por la ecosonda