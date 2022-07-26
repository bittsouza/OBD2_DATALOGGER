#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
// #include "esp_err.h"
#include "nvs_flash.h"
#include "esp_vfs.h"
#include "esp_spiffs.h"

#include <stdio.h>
#include <sys/unistd.h>
#include <sys/stat.h>

// #include "protocol_examples_common.h"

#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "lwip/dns.h"
#include "lwip/err.h"
#include "lwip/sys.h"

#include "cmd.h"

#include "driver/can.h"
#include "config.h"
/* The examples use WiFi configuration that you can set via project configuration menu

   If you'd rather not, just change the below entries to strings with
   the config you want - ie #define EXAMPLE_WIFI_SSID "mywifissid"
*/
// #define EXAMPLE_ESP_WIFI_SSID      "Nathalia.2G"
// #define EXAMPLE_ESP_WIFI_PASS      "nati4792"

#define EXAMPLE_ESP_WIFI_SSID      "WIFI_NAME"
#define EXAMPLE_ESP_WIFI_PASS      "WIFI_PASSWORD"
#define EXAMPLE_ESP_MAXIMUM_RETRY  3

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

#define WEB_SERVER "123.123.123.123" // WEB SERVER IP
#define WEB_PORT "80" // WEB SERVER PORT 
#define WEB_PATH "/webserver/path"

/* --------------------- Definitions and static variables ------------------ */

//PIDs requested
#define ID_ENGINE_RPM                   0x0C // RPM - Revolutions per minute from the motor
#define ID_ENGINE_SPD                   0x0D // SPD - Intake temperature in ºC
#define ID_ENGINE_INT                   0x0F // INT - Intake temperature in ºC
#define ID_ENGINE_TPS                   0x45 // TPS - Throttle position in %
#define ID_ENGINE_FUL                   0x2F // FUL - Fuel in the tank in %
#define ID_ENGINE_ODO                   0x31 // ODO - Total distance traveled in Km
#define ID_ENGINE_LBD                   0x44 // LBD - Lambda sensor
#define ID_ENGINE_RTM                   0x1F // RTM - Run time since the engine start

void obd_id_class(can_message_t rx_msg);

int TIMING_CONFIG = 0;
int Contador_TX = 0;

int endlog = 0;

//Config to no expect AKC
static const can_filter_config_t f_config = CAN_FILTER_CONFIG_ACCEPT_ALL();

//Config bit timing to 500Kbits of rate
static const can_timing_config_t t_config = CAN_TIMING_CONFIG_500KBITS();

//Set TX queue length to 0 due to listen only mode
static const can_general_config_t g_config = {.mode = CAN_MODE_NO_ACK,
                                              .tx_io = TX_GPIO_NUM, .rx_io = RX_GPIO_NUM,
                                              .clkout_io = CAN_IO_UNUSED, .bus_off_io = CAN_IO_UNUSED,
                                              .tx_queue_len = 5, .rx_queue_len = 5,
                                              .alerts_enabled = CAN_ALERT_NONE,
                                              .clkout_divider = 0};


//RPM
static const can_message_t start_message_RPM = {.identifier = ID_HELLO_ECU, .data_length_code = 8,
                                            .flags = CAN_MSG_FLAG_NONE, .data = {0x02,SERVICE_MODE_CURRENT,ID_ENGINE_RPM,55,55,55,55,55}};
//speed
static const can_message_t start_message_SPD = {.identifier = ID_HELLO_ECU, .data_length_code = 8,
                                            .flags = CAN_MSG_FLAG_NONE, .data = {0x02,SERVICE_MODE_CURRENT,ID_ENGINE_SPD,01,55,55,55,55}};
// //Intake Temp
// static const can_message_t start_message_INT = {.identifier = ID_HELLO_ECU, .data_length_code = 8,
//                                             .flags = CAN_MSG_FLAG_NONE, .data = {0x02,SERVICE_MODE_CURRENT,ID_ENGINE_INT,01,55,55,55,55}};
//Throttle position
static const can_message_t start_message_TPS = {.identifier = ID_HELLO_ECU, .data_length_code = 8,
                                            .flags = CAN_MSG_FLAG_NONE, .data = {0x02,SERVICE_MODE_CURRENT,ID_ENGINE_TPS,01,55,55,55,55}};
// //Fuel Level
// static const can_message_t start_message_FUL = {.identifier = ID_HELLO_ECU, .data_length_code = 8,
//                                             .flags = CAN_MSG_FLAG_NONE, .data = {0x02,SERVICE_MODE_CURRENT,ID_ENGINE_FUL,01,55,55,55,55}};

// //Odometer
// static const can_message_t start_message_ODO = {.identifier = ID_HELLO_ECU, .data_length_code = 8,
//                                             .flags = CAN_MSG_FLAG_NONE, .data = {0x02,SERVICE_MODE_CURRENT,ID_ENGINE_ODO,55,55,55,55,55}};
//Lambda sensor
static const can_message_t start_message_LBD = {.identifier = ID_HELLO_ECU, .data_length_code = 8,
                                            .flags = CAN_MSG_FLAG_NONE, .data = {0x02,SERVICE_MODE_CURRENT,ID_ENGINE_LBD,55,55,55,55,55}};
//Run time engine
static const can_message_t start_message_RTM = {.identifier = ID_HELLO_ECU, .data_length_code = 8,
                                            .flags = CAN_MSG_FLAG_NONE, .data = {0x02,SERVICE_MODE_CURRENT,ID_ENGINE_RTM,55,55,55,55,55}};




static SemaphoreHandle_t rx_sem;
esp_err_t error_rx_can;
esp_err_t error_tx_can;

can_message_t rx_msg;

// Data acquired and processed.
uint16_t RPM;
uint16_t SPD;
uint16_t INT;
uint16_t TPS;
uint16_t FUL;
uint16_t ODO;
double LBD;
uint16_t RTM;

QueueHandle_t xQueueRequest;
QueueHandle_t xQueueResponse;

struct Data_received
{
    int RPM;
    int SPD;
    int TPS;
    int INT;
    int LBD;
    int RTM;
};
struct Data_received dataobd2;

static const char *TAG = "wifi station";

static const char *REQUEST = "POST " WEB_PATH " HTTP/1.0\r\n"
    "Host: "WEB_SERVER":"WEB_PORT"\r\n"
    "User-Agent: esp-idf/1.0 esp32\r\n"
    "\r\n";

static int s_retry_num = 0;

void read_log();
void wifi_init_stat(void);
void http_post_task(void *pvParameters);

uint32_t iterations = 0;
// static void can_receive_task(void *arg)
void can_receive_task(void *arg)
{



    //uint32_t cont = 0;

    while (iterations < NO_OF_ITERS) {

        
        

        
        // vTaskDelay(5/portTICK_PERIOD_MS);
        // vTaskDelay(5/portTICK_PERIOD_MS);
        // error_tx_can =  can_transmit(&start_message_INT, portMAX_DELAY);
        // obd_id_class(rx_msg);
        // vTaskDelay(5/portTICK_PERIOD_MS);

        // error_tx_can =  can_transmit(&start_message_FUL, portMAX_DELAY);
        // obd_id_class(rx_msg);
        

        error_tx_can =  can_transmit(&start_message_RPM, portMAX_DELAY);
        obd_id_class(rx_msg);
        error_tx_can =  can_transmit(&start_message_SPD, portMAX_DELAY);
        obd_id_class(rx_msg);
        error_tx_can =  can_transmit(&start_message_TPS, portMAX_DELAY);
        obd_id_class(rx_msg);
        error_tx_can =  can_transmit(&start_message_LBD, portMAX_DELAY);
        obd_id_class(rx_msg);
        error_tx_can =  can_transmit(&start_message_RTM, portMAX_DELAY);
        obd_id_class(rx_msg);
        // vTaskDelay(5/portTICK_PERIOD_MS);
        // vTaskDelay(5/portTICK_PERIOD_MS);
        // error_tx_can =  can_transmit(&start_message_ODO, portMAX_DELAY);
        // obd_id_class(rx_msg);
        // vTaskDelay(5/portTICK_PERIOD_MS);
        // vTaskDelay(5/portTICK_PERIOD_MS);
        // vTaskDelay(5/portTICK_PERIOD_MS);

        if(error_tx_can == ESP_OK){
            //ESP_LOGI(EXAMPLE_TAG, "Transmitted start command");
            Contador_TX=1;
        }
        else{
            ESP_LOGI(EXAMPLE_TAG, "ERRO_TX");
        }
        iterations++;
        ESP_LOGI(EXAMPLE_TAG, "iteration n:%i", iterations);
    }


    ESP_LOGI(EXAMPLE_TAG, "OPPENING LOG ...");
    // read_log();

     // Create Queue
	xQueueRequest = xQueueCreate( 1, sizeof(REQUEST_t) );
	xQueueResponse = xQueueCreate( 1, sizeof(RESPONSE_t) );
	configASSERT( xQueueRequest );
	configASSERT( xQueueResponse );

	// Create Task
	xTaskCreate(&http_post_task, "POST", 4096, NULL, 5, NULL);

    // Search Directory
	DIR* dir = opendir("/spiffs/");
	assert(dir != NULL);

	REQUEST_t requestBuf;
	requestBuf.command = CMD_SEND;
	requestBuf.taskHandle = xTaskGetCurrentTaskHandle();
	RESPONSE_t responseBuf;
	while (true) {
		struct dirent*pe = readdir(dir);
		if (!pe) break;
		ESP_LOGI(TAG, "d_name=%s d_ino=%d d_type=%x", pe->d_name,pe->d_ino, pe->d_type);
		strcpy(requestBuf.remoteFileName, pe->d_name);
		strcpy(requestBuf.localFileName, "/spiffs/");
		strcat(requestBuf.localFileName, pe->d_name);
		if (xQueueSend(xQueueRequest, &requestBuf, 10) != pdPASS) {
			ESP_LOGE(TAG, "xQueueSend fail");
		} else {
			xQueueReceive(xQueueResponse, &responseBuf, portMAX_DELAY);
			ESP_LOGI(TAG, "\n%s", responseBuf.response);
#if 0
			for(int i = 0; i < strlen(responseBuf.response); i++) {
				putchar(responseBuf.response[i]);
			}
			printf("\n");
#endif
		}
	} // end while
	closedir(dir);
	ESP_LOGI(TAG, "All file uploded");


    vTaskDelete(NULL);
}


static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void wifi_init_sta(void)
{
    // end_log();
    // while (logstatus != 1)
    // {
    //    ESP_LOGI(EXAMPLE_TAG, "WAITING FOR LOG END...");
    //    vTaskDelay(2000/portTICK_PERIOD_MS);
    // }

    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .password = EXAMPLE_ESP_WIFI_PASS,
            /* Setting a password implies station will connect to all security modes including WEP/WPA.
             * However these modes are deprecated and not advisable to be used. Incase your Access point
             * doesn't support WPA2, these mode can be enabled by commenting below line */
	     .threshold.authmode = WIFI_AUTH_WPA2_PSK,

            .pmf_cfg = {
                .capable = true,
                .required = false
            },
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }

    /* The event will not be processed after unregister */
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip));
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id));
    vEventGroupDelete(s_wifi_event_group);

    return;
}

static void http_get_task(void *pvParameters)
{
    const struct addrinfo hints = {
        .ai_family = AF_INET,
        .ai_socktype = SOCK_STREAM,
    };
    struct addrinfo *res;
    struct in_addr *addr;
    int s, r;
    char recv_buf[64];

    while(1) {

        int err = getaddrinfo("187.63.222.127", "80", &hints, &res);

        if(err != 0 || res == NULL) {
            ESP_LOGE(TAG, "DNS lookup failed err=%d res=%p", err, res);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            continue;
        }

        /* Code to print the resolved IP.

           Note: inet_ntoa is non-reentrant, look at ipaddr_ntoa_r for "real" code */
        addr = &((struct sockaddr_in *)res->ai_addr)->sin_addr;
        ESP_LOGI(TAG, "DNS lookup succeeded. IP=%s", inet_ntoa(*addr));

        s = socket(res->ai_family, res->ai_socktype, 0);
        if(s < 0) {
            ESP_LOGE(TAG, "... Failed to allocate socket.");
            freeaddrinfo(res);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            continue;
        }
        ESP_LOGI(TAG, "... allocated socket");

        if(connect(s, res->ai_addr, res->ai_addrlen) != 0) {
            ESP_LOGE(TAG, "... socket connect failed errno=%d", errno);
            close(s);
            freeaddrinfo(res);
            vTaskDelay(4000 / portTICK_PERIOD_MS);
            continue;
        }

        ESP_LOGI(TAG, "... connected");
        freeaddrinfo(res);

        if (write(s, REQUEST, strlen(REQUEST)) < 0) {
            ESP_LOGE(TAG, "... socket send failed");
            close(s);
            vTaskDelay(4000 / portTICK_PERIOD_MS);
            continue;
        }
        ESP_LOGI(TAG, "... socket send success");

        struct timeval receiving_timeout;
        receiving_timeout.tv_sec = 5;
        receiving_timeout.tv_usec = 0;
        if (setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &receiving_timeout,
                sizeof(receiving_timeout)) < 0) {
            ESP_LOGE(TAG, "... failed to set socket receiving timeout");
            close(s);
            vTaskDelay(4000 / portTICK_PERIOD_MS);
            continue;
        }
        ESP_LOGI(TAG, "... set socket receiving timeout success");

        /* Read HTTP response */
        do {
            bzero(recv_buf, sizeof(recv_buf));
            r = read(s, recv_buf, sizeof(recv_buf)-1);
            for(int i = 0; i < r; i++) {
                putchar(recv_buf[i]);
            }
        } while(r > 0);

        ESP_LOGI(TAG, "... done reading from socket. Last read return=%d errno=%d.", r, errno);
        close(s);
        for(int countdown = 10; countdown >= 0; countdown--) {
            ESP_LOGI(TAG, "%d... ", countdown);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
        ESP_LOGI(TAG, "Starting again!");
    }
    vTaskDelete(NULL);
}



void setup_spiffs(){
    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

   esp_vfs_spiffs_conf_t conf = {
      .base_path = "/spiffs",
      .partition_label = NULL,
      .max_files = 5,
      .format_if_mount_failed = true
    };

    // Use settings defined above to initialize and mount SPIFFS filesystem.
    // Note: esp_vfs_spiffs_register is an all-in-one convenience function.
    ret = esp_vfs_spiffs_register(&conf);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount or format filesystem");
        } else if (ret == ESP_ERR_NOT_FOUND) {
            ESP_LOGE(TAG, "Failed to find SPIFFS partition");
        } else {
            ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
        }
        return;
    }

    size_t total = 0, used = 0;
    ret = esp_spiffs_info(conf.partition_label, &total, &used);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get SPIFFS partition information (%s)", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Partition size: total: %d, used: %d", total, used);
    }


    //DELETING THE LAST LOG
    unlink("/spiffs/hello.csv");
    ESP_LOGI(TAG, "Opening file");
    FILE* f = fopen("/spiffs/hello.csv", "a");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for writing");
        return;
    }
    //WRITING THE HEADER OF THE LOG
    fprintf(f, "KPH,RPM,LBD,RTM(S),TPS(%%)\r\n");
    fclose(f);
    ESP_LOGI(TAG, "File written");
}

void obd_id_class(can_message_t rx_msg){

    //CREATING FILE
    //ESP_LOGI(TAG, "Opening file");
    FILE* f = fopen("/spiffs/hello.csv", "a");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for writing");
        return;
    }

    error_rx_can = can_receive(&rx_msg, (100/portTICK_PERIOD_MS));
    // ESP_LOGI(EXAMPLE_TAG, "STOAQU");
        if(error_rx_can == ESP_OK){
            // ESP_LOGI(EXAMPLE_TAG, "STOAQU1");
            if(rx_msg.identifier == 401604624) {
                //cont ++;
                // ESP_LOGI(EXAMPLE_TAG, "STOAQU2");
            } else{


                switch (rx_msg.data[2])
                {

                    case ID_ENGINE_RPM:
                        RPM = (((rx_msg.data[3]*256)+rx_msg.data[4])/4); //RPM=(((A*256)+B)/4)
                        ESP_LOGI(EXAMPLE_TAG, "RPM: %d", RPM);
                        //dataobd2.RPM=RPM;
                        fprintf(f, "%i,", RPM);

                    break;

                    case ID_ENGINE_SPD:
                        SPD = (rx_msg.data[3]);
                        ESP_LOGI(EXAMPLE_TAG, "SPEED: %d KPH", SPD);
                        // dataobd2.SPD=SPD;
                        fprintf(f, "%i,", SPD);
                    break;


                    case ID_ENGINE_TPS:
                        TPS = ((rx_msg.data[3]*100)/255);
                        ESP_LOGI(EXAMPLE_TAG, "TPS: %d %%", TPS);
                        // dataobd2.TPS=TPS;
                        fprintf(f, "%i,", TPS);
                    break;

                    case ID_ENGINE_LBD:
                        LBD = (((2/65536)*(256*rx_msg.data[3])+rx_msg.data[4]));
                        ESP_LOGI(EXAMPLE_TAG, "LBD: %f", LBD);
                        // dataobd2.LBD=LBD;
                        fprintf(f, "%f,", LBD);
                    break;

                    case ID_ENGINE_RTM:
                        RTM = ((rx_msg.data[3]*256)+rx_msg.data[4]);
                        ESP_LOGI(EXAMPLE_TAG, "RUNTIME: %d", RTM);
                        // dataobd2.RTM=RTM;
                        fprintf(f, "%i\n", RTM);
                    break;
                    
                    // case ID_ENGINE_INT:
                    //     //INT = ((rx_msg.data[3])-40);
                    //     // ESP_LOGI(EXAMPLE_TAG, "INTAKE: %d C", INT);
                    //     // dataobd2.INT=INT;
                    //     // fprintf(f, "%i,", dataobd2.INT);
                    // break;

                    // case ID_ENGINE_FUL:
                    //     //FUL = ((rx_msg.data[3]*100)/255);
                    //     // ESP_LOGI(EXAMPLE_TAG, "FUEL LEVEL: %d %%", FUL);
                    // break;

                    // case ID_ENGINE_ODO:
                    //     //ODO = (3660+((256*rx_msg.data[3])+rx_msg.data[4]));
                    //     // ESP_LOGI(EXAMPLE_TAG, "ODO: %d Km", ODO);
                    // break;

                }
            }
        }
		else {

            ESP_LOGI(EXAMPLE_TAG, "ERRO");

		}

    //END OF RECEIVED DATA

    //fprintf(f, "RPM ,KPH ,TPS\r\n");

    fclose(f);
    //ESP_LOGI(TAG, "File written");


    // ESP_LOGI(TAG, "Opening file");
    // f = fopen("/spiffs/hello.csv", "a");
    // if (f == NULL) {
    //     ESP_LOGE(TAG, "Failed to open file for writing");
    //     return;
    // }

    //fprintf(f, "%i;%i;%i", dataobd2.RPM,dataobd2.INT,dataobd2.TPS);
    // fprintf(f, "1000,20,14\r\n");
    // fprintf(f, "1100,22,16\r\n");
    // fprintf(f, "1200,24,18\r\n");
    //fclose(f);

    //ESP_LOGI(TAG, "File written");
    

}

void read_log(){
    // Open renamed file for reading
    ESP_LOGI(TAG, "Reading file");
    FILE* f = fopen("/spiffs/hello.csv", "r");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for reading");
        return;
    }
    char line[128];

    //ESP_LOGI(TAG, "Read from file: '%s'", line);
    ESP_LOGI(TAG, "Read from file: ");

    // lendo um arquivo até o final
    while(!feof(f)){
        if(fgets(line, 128, f))
        printf("\n--> %s", line);
    }
    fclose(f);
   return;
}

void app_main(void)
{
    setup_spiffs();

    rx_sem = xSemaphoreCreateBinary();
    xTaskCreatePinnedToCore(can_receive_task, "CAN_rx", 4096, NULL, RX_TASK_PRIO, NULL, tskNO_AFFINITY);
    

    //Install and start CAN driver
    ESP_ERROR_CHECK(can_driver_install(&g_config, &t_config, &f_config));
    ESP_LOGI(EXAMPLE_TAG, "Driver installed");
    ESP_ERROR_CHECK(can_start());
    ESP_LOGI(EXAMPLE_TAG, "Driver started");

    vTaskDelay(2000 / portTICK_PERIOD_MS);

    //Wifi init
    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
    wifi_init_sta();

    vTaskDelay(2000 / portTICK_PERIOD_MS);

    //Stop and uninstall CAN driver
    // ESP_ERROR_CHECK(can_stop());
    // ESP_LOGI(EXAMPLE_TAG, "Driver stopped");
    // ESP_ERROR_CHECK(can_driver_uninstall());
    // ESP_LOGI(EXAMPLE_TAG, "Driver uninstalled");

    //Cleanup
    // vSemaphoreDelete(rx_sem);


}