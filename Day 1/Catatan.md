# Day one

## INITIALIZATION

### DIRECT CONTROL

    rostopic pub /robotis/enable_ctrl_module std_msgs/String "data: 'direct_control_module'"

### ONLINE WALKING

    rostopic pub /robotis/enable_ctrl_module std_msgs/String "data: 'online_walking_module'"

### INI POSE(SIT/DEFAULT POSE)

    rostopic pub /robotis/base/ini_pose std_msgs/String "data: 'ini_pose'"

### WALKING(STANDING)

    rostopic pub /robotis/enable_ctrl_module std_msgs/String "data: 'walking_module'"
    
## CONTROL

### DIRECT SERVO CONTROL

    rostopic pub /robotis/direct_control/set_joint_states <messages data(TAB)>

### JOINT NAMES

#### AXES ABBREVIATIONS

- Z AXES(TOP/BOTTOM)  : YAW/PAN(Z AXES-WISE ANGLE)
- Y AXES(FRONT/BACK)  : ROLL(Y AXES-WISE ANGLE)
- X AXES(LEFT/RIGHT)  : PITCH(X AXES-WISE ANGLE)
- TILT(PAN AXES-WISE ANGLE)

#### TOP SIDE
- HEAD ROTATION(LEFT POSITIVE)    : head_pan
- HEAD UP/DOWN ANGLE(UP POSITIVE) : head_tilt

#### LEFT SIDE
- LEFT SHOULDER ROTATION(INWARD POSITIVE)         : l_sho_pitch
- LEFT SHOULDER ANGLE(INWARD POSITIVE)            : l_sho_roll
- LEFT ELBOW ANGLE(INWARD NEGATIVE)               : l_el
- LEFT HIP ROTATION(INWARD POSITIVE)              : l_hip_yaw
- LEFT HIP FRONT/BACK ANGLE(INWARD POSITIVE)      : l_hip_pitch
- LEFT HIP LEFT/RIGHT ANGLE(INWARD POSITIVE)      : l_hip_roll
- LEFT KNEE ANGLE(INWARD POSITIVE)                : l_knee
- LEFT ANKLE LEFT/RIGHT ANGLE(INWARD NEGATIVE)    : l_ank_roll
- LEFT ANKLE FRONT/BACK ANGLE(INWARD NEGATIVE)    : l_ank_pitch

#### RIGHT SIDE
- RIGHT SHOULDER ROTATION(INWARD NEGATIVE)        : r_sho_pitch
- RIGHT SHOULDER ANGLE(INWARD NEGATIVE)           : r_sho_roll
- RIGHT ELBOW ANGLE(INWARD POSITIVE)              : r_el
- RIGHT HIP ROTATION(INWARD NEGATIVE)             : r_hip_yaw
- RIGHT HIP FRONT/BACK ANGLE(INWARD NEGATIVE)     : r_hip_pitch
- RIGHT HIP LEFT/RIGHT ANGLE(INWARD NEGATIVE)     : r_hip_roll
- RIGHT KNEE ANGLE(INWARD NEGATIVE)               : r_knee
- RIGHT ANKLE LEFT/RIGHT ANGLE(INWARD POSITIVE)   : r_ank_roll
- RIGHT ANKLE FRONT/BACK ANGLE(INWARD POSITIVE)   : r_ank_pitch

### Langkah

- masukkan ip address di vnc nya sesuaikan dengan robotis op3 nya 

### Batas

1.5 = head_pan maximum (kiri)
-1.5 = head_pan minimum (kanan)

1 = head_tilt maximum (atas)
-1 = head_tilt minimum (bawah)

1 = l_sho_pitch maximum (belakang)
-1 = l_sho_pitch minimum (depan)

1 = r_sho_pitch maximum (depan)
-1 = r_ho_pitch minimum (belakang)

-1.5 = l_sho_roll maximum (atas)
1.5 = l_sho_roll minimum (bawah)
