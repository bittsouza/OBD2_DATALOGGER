// Data acquired and processed.
uint16_t RPM;
uint16_t SPD;
uint16_t INT;
uint16_t TPS;
uint16_t FUL;
uint16_t ODO;
uint16_t LBD_A;
uint16_t LBD_B;
uint16_t LBD;
uint16_t RTM;
uint16_t ETH;

//RPM
static const can_message_t start_message_RPM = {.identifier = ID_HELLO_ECU, .data_length_code = 8,
                                            .flags = CAN_MSG_FLAG_NONE, .data = {0x02,SERVICE_MODE_CURRENT,ID_ENGINE_RPM,55,55,55,55,55}};                                              
//speed
static const can_message_t start_message_SPD = {.identifier = ID_HELLO_ECU, .data_length_code = 8,
                                            .flags = CAN_MSG_FLAG_NONE, .data = {0x02,SERVICE_MODE_CURRENT,ID_ENGINE_SPD,01,55,55,55,55}};                                              
//Intake Temp
static const can_message_t start_message_INT = {.identifier = ID_HELLO_ECU, .data_length_code = 8,
                                            .flags = CAN_MSG_FLAG_NONE, .data = {0x02,SERVICE_MODE_CURRENT,ID_ENGINE_INT,01,55,55,55,55}};                                              
//Throttle position
static const can_message_t start_message_TPS = {.identifier = ID_HELLO_ECU, .data_length_code = 8,
                                            .flags = CAN_MSG_FLAG_NONE, .data = {0x02,SERVICE_MODE_CURRENT,ID_ENGINE_TPS,01,55,55,55,55}};                                              
//Fuel Level
static const can_message_t start_message_FUL = {.identifier = ID_HELLO_ECU, .data_length_code = 8,
                                            .flags = CAN_MSG_FLAG_NONE, .data = {0x02,SERVICE_MODE_CURRENT,ID_ENGINE_FUL,01,55,55,55,55}};                                              
//Odometer
static const can_message_t start_message_ODO = {.identifier = ID_HELLO_ECU, .data_length_code = 8,
                                            .flags = CAN_MSG_FLAG_NONE, .data = {0x02,SERVICE_MODE_CURRENT,ID_ENGINE_ODO,55,55,55,55,55}};                                              
//Lambda sensor
static const can_message_t start_message_LBD = {.identifier = ID_HELLO_ECU, .data_length_code = 8,
                                           .flags = CAN_MSG_FLAG_NONE, .data = {0x02,SERVICE_MODE_CURRENT,ID_ENGINE_LBD,01,55,55,55,55}};                                              
// //Run time engine
static const can_message_t start_message_RTM = {.identifier = ID_HELLO_ECU, .data_length_code = 8,
                                            .flags = CAN_MSG_FLAG_NONE, .data = {0x02,SERVICE_MODE_CURRENT,ID_ENGINE_RTM,55,55,55,55,55}};                                              
//Ethanol %
static const can_message_t start_message_ETH = {.identifier = ID_HELLO_ECU, .data_length_code = 8,
                                            .flags = CAN_MSG_FLAG_NONE, .data = {0x02,SERVICE_MODE_CURRENT,ID_ENGINE_ETH,55,55,55,55,55}};                                              

