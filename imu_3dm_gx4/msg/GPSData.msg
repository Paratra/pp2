# ECEF Position
float64 position_x
float64 position_y
float64 position_z
float32 position_accuracy

# ECEF Velocity
float64 velocity_x
float64 velocity_y
float64 velocity_z
float32 velocity_accuracy

# GPS Time
float64 time
uint16 time_week_number

# Clock Information
float64 clock_bias
float64 clock_drift
float64 clock_accuracy

# GNSS Information (Fix?)
uint8 info_fix_type
uint8 info_fix_sv_count
uint16 info_fix_flag

# Space-Vehilce Information (SVI)
# Separate MIP packet!
# TODO: combined into array
uint8[] svi_channels
uint8[] svi_ids
uint16[] svi_noises
uint16[] svi_azimuths
uint16[] svi_elevations
uint16[] svi_flags

# Hardware status
uint8 status_sensor_state
uint8 status_antenna_state
uint8 status_antenna_power


#GPSLLHPosition position_llh
#GPSNEDVelocity velocity_ned
#GPSVec3A position_ecef
#GPSVec3A velocity_ecef

# LLH Position
#float64 latitude
#float64 longitude
#float64 height_above_ellipsoid
#float64 height_above_msl
#float32 accuracy_horizontal
#float32 accuracy_vertical

# NED Velocity
#float32 north
#float32 east
#float32 down
#float32 speed
#float32 ground_speed
#float32 heading
#float32 accuracy_speed
#float32 accuracy_heading
