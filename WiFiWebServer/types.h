
#define false ((bool) 0)
#define true ((bool) 1)


typedef enum {
    THROTTLE=0,
    SPEED,
//    DUMMY=1<<31
} tcs_mode_t;


//typedef uint8_t bool;

typedef struct {
  uint32_t tc3_ovf_count;
  uint32_t current_tach;
  uint32_t previous_tach;
  int32_t speed;
  int32_t previous_speed;
  int32_t previous_error;
  int32_t throttle_slew_rate;
  int32_t speed_cmd;
  int32_t throttle_cmd;
  int32_t throttle;
  int32_t previous_throttle;
  int32_t vac;
  int32_t vcc;
  int32_t amps;
  float Kp;
  float Ki;
  float Kd;
  float Ko;
  tcs_mode_t mode;

//    int16_t speed_error;
//    int16_t speed_delta;
//    uint16_t tach_delta;
//    uint16_t previous_current_tach;
//    int32_t current_speed;
//    int32_t previous_speed;
//    int32_t cmd;
//    int16_t requested_speed;
//    int16_t max_speed;
//    uint8_t max_throttle;
//    int16_t previous_error;
//    int16_t integral;
//    uint8_t throttle;
//    uint8_t cruise;
} params_t;

