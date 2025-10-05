# This is a message that holds commanded angular joint velocity
std_msgs/Header header

# Use an array of type float32 for the two wheel joint velocities.
# float32 is used instead of float64 because it is not supporte by Arduino/Teensy.
WheelsCmd wheels_cmd