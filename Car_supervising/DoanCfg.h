/*************************************************
DEFINE PROTOCOL
 *************************************************/
#define REQUEST_TIME 1000
#define RETURN_OFFSET 0x40
#define SID_POSITION 0// mode 
#define PID_POSITION 1 
#define DATA_A_POSITION 2
#define DATA_B_POSITION 3
#define DATA_C_POSITION 4
#define DATA_D_POSITION 5
// #define SL_POSITION 1 // SỐ LƯỢNG MÃ LỖI

/*************************************************
DEFINE OBD MODE VALUE
 *************************************************/
#define OBD_MODE_1 0x01
#define OBD_MODE_2 0x02
#define OBD_MODE_3 0x03
#define OBD_MODE_4 0x04
#define OBD_MODE_5 0x05
#define OBD_MODE_6 0x06
#define OBD_MODE_7 0x07
#define OBD_MODE_8 0x08
#define OBD_MODE_9 0x09
#define OBD_MODE_A 0x0A

/*************************************************
DEFINE OBD MODE 1 PARAMETER ID
 *************************************************/
#define MODE_1_VEHICAL_SPEED      13
#define MODE_1_ENGINE_RPM         0x0C
#define MODE_1_ENGINE_COOL_TEMP   0x05
#define MODE_1_AIR_FLOW           0x10
#define MODE_1_THROTTLE_POSITION  0x11
#define MODE_1_FUEL_LEVEL         0x2F
#define MODE_1_ENGINE_LOAD        0x04
#define MODE_1_PERCENT_TORQUE     0x62 