# Scenario message — simulator sends this to the agent every 50ms

# Cycle info
int32   id
int32   version
int32   cycle_number
float64 ecu_up_time
int32   status

# Vehicle state
float64 v_lgt_fild               # longitudinal velocity [m/s]
float64 a_lgt_fild               # longitudinal acceleration [m/s²]
float64 yaw_rate_fild            # yaw rate [rad/s]
float64 steer_whl_ag             # steering wheel angle [rad], positive = left
float64 vehicle_len              # vehicle length [m]
float64 vehicle_width            # vehicle width [m]
float64 requested_cruising_speed # cruise control target [m/s]
int32   automation_level         # SAE level: 0=none 1=assisted ... 5=full
int32   current_lane             # 0=unknown 1=emergency 2=single 3=leftmost 4=rightmost

# Detected objects (max 20)
int32      nr_objs
int32[20]  obj_id
float64[20] obj_x      # object X position in vehicle frame [m]
float64[20] obj_y      # object Y position in vehicle frame [m]
float64[20] obj_len    # object length [m]
float64[20] obj_width  # object width [m]
float64[20] obj_vel    # object speed [m/s]
float64[20] obj_course # object heading in vehicle frame [rad]

# Lane information
float64 lane_width
float64 lat_offs_line_r  # lateral offset to right lane line [m]
float64 lat_offs_line_l  # lateral offset to left lane line [m]
float64 lane_heading     # [rad], positive counter-clockwise
float64 lane_crvt        # lane curvature [1/m], positive = left curve

# ADASIS curvature horizon (map data, up to 100 points ahead)
int32       adasis_curvature_nr
float64[100] adasis_curvature_dist   # distance to each point [m]
float64[100] adasis_curvature_values # curvature value [1/m]

# ADASIS speed limits ahead (up to 10 points)
int32      adasis_speed_limit_nr
float64[10] adasis_speed_limit_dist
int32[10]  adasis_speed_limit_values # [km/h], 0=unknown

# Traffic light
int32   nr_trf_lights
float64 trf_light_dist                   # distance to nearest light [m]
int32   trf_light_curr_state             # 1=Green 2=Yellow 3=Red 0=Flashing
float64 trf_light_first_time_to_change   # time until next state change [s]
int32   trf_light_first_next_state
float64 trf_light_second_time_to_change
int32   trf_light_second_next_state
float64 trf_light_third_time_to_change
