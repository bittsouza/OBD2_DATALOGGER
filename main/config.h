/* --------------------- Definitions and static variables ------------------ */
// https://threepixels.com.br/octsat/android_app/upload/hello.csv
//System Configuration
#define EXAMPLE_TAG                     "CAN Listen Only"
#define NO_OF_ITERS                     500          //Number of interations
#define RX_TASK_PRIO                    9           //Priority of the main task
#define TX_GPIO_NUM                     22          //Tx pin ESP32
#define RX_GPIO_NUM                     21          //Rx pin ESP32
#define CAN_MAX_DATA_LEN                8           //< Maximum number of data bytes in a CAN2.0B frame

//Can configuration
#define ID_HELLO_ECU                    0x7DF       //ID to request data for ECU
#define ID_ANSWER_ECU                   0x7E8       //ID answer from ECU with the data

#define SERVICE_MODE_CURRENT            01        //Current information from ECU
#define SERVICE_MODE_FREEZE             02        //Freze frame data information from ECU
#define SERVICE_MODE_ERR_CODES          03        //Request trouble codes