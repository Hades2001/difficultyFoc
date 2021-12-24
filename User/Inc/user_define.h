#ifndef _USER_DEFINE_H_
#define _USER_DEFINE_H_

#define nullptr NULL
#define _PI 3.141592653f
#define _2PI 6.283185307f
#define _PI_3 1.047197551f
#define _3PI_2 4.712388980f
#define _SQRT3 1.732050808f

#define _R2D(_R) (_R * 57.295779f)
#define _D2R(_D) (_D * _PI / 180 )

#define _CONSTRAIN(val,down,up) ((val) < (down) ? (down) : ((val) > (up) ? (up) : (val)))

#define LOG_INFO    0
#define LOG_WARN    1
#define LOG_ERROR   2

#define LOG_PRINTF(t,c,...) serial.log(0,c,##__VA_ARGS__)


typedef enum{
	CMD_READ_ALL            = 0x25,

	CMD_SET_IDLE            = 0x90,
	CMD_SET_VELOCITY        = 0x91,
	CMD_SET_ANGLE           = 0x92,

	CMD_SET_ALL             = 0xA0,
	CMD_SET_SENSOR_OFFSET   = 0xA1,
	CMD_SET_DC              = 0xA2,
	CMD_SET_VOUT_LIMIT      = 0xA3,

	CMD_SET_VELOCITY_PID    = 0xB1,
	CMD_SET_ANGLE_PID       = 0xB2,
}pack_cmd_t;


typedef enum{
    STATE_IDLE = 0,
    STATE_ANGLE,
    STATE_VELOCITY,
    STATE_WRITE_BUSY,
}system_state_t;




#endif
