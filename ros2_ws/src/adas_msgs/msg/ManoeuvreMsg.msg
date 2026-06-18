# Manoeuvre message — agent sends this to the simulator every 50ms
  

# Cycle info
int32   id
int32   version
int32   cycle_number
float64 ecu_up_time
int32   status

# Planned trajectory (max 23 points)
int32       n_trajectory_points
float64[23] trajectory_time   # timestamp for each point [s]
float64[23] trajectory_x      # X position in vehicle frame [m]
float64[23] trajectory_y      # Y position in vehicle frame [m]

# Control outputs
float64 target_speed           # desired speed at end of manoeuvre [m/s]
float64 requested_acc          # acceleration demand [m/s²]
int32   manoeuvre_type         # 0=free flow 1=stopping 2=following
float64 requested_steer_whl_ag # steering wheel angle demand [rad]