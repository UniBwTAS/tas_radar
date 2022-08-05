# A single measurement with scalar uncertainty

uint8   available           # >0, if this measurement is actually available
float64 value               # the mean of the measurement
float32 uncertainty         # the standard deviation of the measurement (only available if uncertainty is explicitly provided by the sensor data)

uint8 AVAILABLE_NOTHING = 0
uint8 AVAILABLE_VALUE = 1
uint8 AVAILABLE_VALUE_AND_UNCERTAINTY = 2
