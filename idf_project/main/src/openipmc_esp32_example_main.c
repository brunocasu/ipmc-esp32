//#include "printf/printf.h"
// ESP32 FreeRTOS includes
#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
//#include "freetos/timers.h"

// ESP32 includes
#include "esp_err.h"
#include "esp_log.h"
#include "sdkconfig.h"

#include <stdbool.h>

//OpenIPMC includes
#include <openipmc/src/ipmc_ios.h>
#include <openipmc/src/ipmb_0.h> 
#include <openipmc/src/ipmi_msg_manager.h> 
#include <openipmc/src/fru_state_machine.h>
#include <openipmc/src/ipmc_tasks.h>

#include "soc/i2c_periph.h"

#define TIMER_ID	1
#define DELAY_60_SECONDS	60000UL // lucas
#define DELAY_30_SECONDS	30000UL // lucas
#define DELAY_10_SECONDS	10000UL
#define DELAY_1_SECOND		1000UL
#define TIMER_CHECK_THRESHOLD	9

// tasks for i2c channels receiving in slave mode


int periphs_init();
// Task handlers for I2C receiver
TaskHandle_t i2c_0_receiver_manager_task_ptr;
TaskHandle_t i2c_1_receiver_manager_task_ptr;
void i2c_0_receiver_manager_task( void *pvParameters );
void i2c_1_receiver_manager_task( void *pvParameters );
  
void app_main( void )
{
    //Example for testing the configuration of master and slave modes in ESP32
    int init_status = periphs_init();
    if (init_status == 1)
        printf("Peripherals Initialization OK\n\n");
    else 
    {printf("Peripherals Initialization FAIL -- -- -- status = %d\n",init_status);}
    

    xTaskCreate( ipmb_0_msg_receiver_task, 				
	             ( const char * ) "MNG", 	
	             8000, 	
	             NULL, 						
	             tskIDLE_PRIORITY+3,		    
	             &ipmb_0_msg_receiver_task_ptr );

	xTaskCreate( ipmb_0_msg_sender_task, 				
	             ( const char * ) "MNGO", 	
	             8000, 	
	             NULL, 						
	             tskIDLE_PRIORITY+1,		    
	             &ipmb_0_msg_sender_task_ptr );
/*    
    xTaskCreate( fru_state_machine_task, 	
	             ( const char * ) "SMM", 	
	             8000, 	
	             NULL, 						
	             tskIDLE_PRIORITY+1,		
	             &fru_state_machine_task_ptr );
  
	xTaskCreate( ipmi_income_requests_manager_task, 
	             ( const char * ) "IPMI_MSG_MGMT", 	
	             8000, 	
	             NULL, 						
	             tskIDLE_PRIORITY+2,		
	             &ipmi_income_requests_manager_task_ptr );

	xTaskCreate( ipmc_handle_switch_task, 				
	             ( const char * ) "IPMC_HANDLE_TRANS", 	
	             8000, 	
	             NULL, 						
	             tskIDLE_PRIORITY+2,		    
	             &ipmc_handle_switch_task_ptr );
	
	xTaskCreate( ipmc_blue_led_blink_task, 		
	             ( const char * ) "BlueLed", 	
	             8000, 	
	             NULL, 						
	             tskIDLE_PRIORITY+2,		
	             &ipmc_blue_led_blink_task_ptr );
*/    
    data_ipmb ipmi_req;
    ipmi_req.channel = 'A';
    ipmi_req.length  = 8;
    ipmi_req.data[0] = 0x86;
    ipmi_req.data[1] = 0xff;
    ipmi_req.data[2] = 0x7b; // checksum of bytes [0] and [1]
    ipmi_req.data[3] = 0xff;
    ipmi_req.data[4] = 0xff;
    ipmi_req.data[5] = 0xff;
    ipmi_req.data[6] = 0xff;
    ipmi_req.data[7] = 0xff;
    // int n=0;
    //vTaskDelay (pdMS_TO_TICKS(5500));
    
    while(1){
    vTaskDelay (pdMS_TO_TICKS(4000));
    xQueueSendToBack(queue_ipmb0_out, &ipmi_req, portMAX_DELAY); // send message
    vTaskDelay (pdMS_TO_TICKS(10));
    xQueueSendToBack(queue_ipmb0_out, &ipmi_req, portMAX_DELAY);
    vTaskDelay (pdMS_TO_TICKS(10));
    xQueueSendToBack(queue_ipmb0_out, &ipmi_req, portMAX_DELAY);
    vTaskDelay (pdMS_TO_TICKS(10));
    xQueueSendToBack(queue_ipmb0_out, &ipmi_req, portMAX_DELAY);
    }    
    while (1){
        vTaskDelay (pdMS_TO_TICKS(100));
    }
}

// example for testing
/*
    data_ipmb ipmi_req;
    ipmi_req.channel = 'A';
    ipmi_req.lenght  = 8;
    ipmi_req.data[0] = 0x86;
    ipmi_req.data[1] = 0xff;
    ipmi_req.data[2] = 0x7b; // checksum of bytes [0] and [1]
    ipmi_req.data[3] = 0xff;
    ipmi_req.data[4] = 0xff;
    ipmi_req.data[5] = 0xff;
    ipmi_req.data[6] = 0xff;
    ipmi_req.data[7] = 0xff;
    int n=0;
    //vTaskDelay (pdMS_TO_TICKS(5500));
    
    while(1){
    vTaskDelay (pdMS_TO_TICKS(4000));
    xQueueSendToBack(queue_ipmb0_out, &ipmi_req, portMAX_DELAY); // send message
    vTaskDelay (pdMS_TO_TICKS(10));
    xQueueSendToBack(queue_ipmb0_out, &ipmi_req, portMAX_DELAY);
    vTaskDelay (pdMS_TO_TICKS(10));
    xQueueSendToBack(queue_ipmb0_out, &ipmi_req, portMAX_DELAY);
    vTaskDelay (pdMS_TO_TICKS(10));
    xQueueSendToBack(queue_ipmb0_out, &ipmi_req, portMAX_DELAY);
    }
    */
