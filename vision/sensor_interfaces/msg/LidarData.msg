# Topic name:
# /sensor/lidardata
Header header
# Measured distance in meters.
# If distance == 0, the measured distance was below the range
# If distance == MAX_FLOAT, the measured distance was above the range
# For now, it is assumed that this includes a full 360° measurement.
float32[] distances
# Angles at which the measurements were made.
# An angle corresponds to the distance measurement made at the same index.
# Angles are in radians.
float32[] measurement_angles
