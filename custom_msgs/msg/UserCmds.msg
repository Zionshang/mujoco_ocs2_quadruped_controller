# user control input message, include gait type, desired vx, vy, w

float64 linear_x_input       # velocity or position in linear x direction
float64 linear_y_input       # velocity or position in linear y direction
float64 angular_y_input      # velocity or position in angular y direction
float64 angular_z_input      # velocity or position in angular z direction
float64 height_ratio         # the ratio of target height to the nominal height

string gait_name             # gaint name [stance, trot, standing_trot, flying_trot]
bool passive_enable          # enable passive state