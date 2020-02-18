/****************************************************************************/
/*                                                                          */
/* This Source Code Form is subject to the terms of the Mozilla Public      */
/* License, v. 2.0. If a copy of the MPL was not distributed with this      */
/* file, You can obtain one at http://mozilla.org/MPL/2.0/.                 */
/*                                                                          */
/****************************************************************************/


/** 
 * @file esp32_ipmc_ios.c
 * 
 * @author Bruno Agusto Casu
 * 
 * This file contains the Hardware interface for the OpenIPMC implementation in the ESP32 microcontroller.
 * It's objective is to serve as an example of usage of the IPMController, and provide the correct implementation of 
 * the IPMB using the I2C peripheral available in the ESP32, in addition to the i2c driver functions, provided in the IDF platform.
 * 
 * Is important to mention that the I2C driver avilable in the IDF does not complies with the requirements of 
 * the I2C multi-master configuration used in the IPMI protocol. Therefore some modifications in the interrupt
 * handler of the driver are necessary to successfully implement the OpenIPMC.
 * 
 */ 

// ESP32 FreeRTOS includes
#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

// ESP32 includes
#include "esp_err.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "sdkconfig.h"
#include <stdbool.h>
#include "soc/i2c_periph.h"

//OpenIPMC includes
#include "openipmc/src/ipmc_ios.h"
#include "openipmc/src/ipmb_0.h"
#include "openipmc/src/ipmi_msg_manager.h"
#include "openipmc/src/fru_state_machine.h"
#include "openipmc/src/ipmc_tasks.h"

#define ACK_CHECK_EN 0x1       /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0      /*!< I2C master will not check ack from slave */

#define I2C_SCLK_RATE   100000 // 100 kHz I2C bus
#define IIC_MODE_MASTER 1
#define IIC_MODE_SLAVE  0
#define IPMB_BUFF_SIZE  256 


static _Bool ipmc_ios_ready_flag = pdFALSE;

void i2c_0_receiver_manager_task( void *pvParameters );
void i2c_1_receiver_manager_task( void *pvParameters );

TaskHandle_t i2c_0_receiver_manager_task_ptr;
TaskHandle_t i2c_1_receiver_manager_task_ptr;

//I2C state variables
static int i2c0_mode;    // Master or Slave
static int i2c1_mode;
static uint32_t iic0_recv_len=0;  // Length of the the received message.
static uint32_t iic1_recv_len=0;
static uint8_t  ipmb_addr;

//Buffers for IPMB receiving
uint8_t ipmba_input_buffer[IPMB_BUFF_SIZE];
uint8_t ipmbb_input_buffer[IPMB_BUFF_SIZE];

// Semaphores to synchronize the IPMB operations with the I2C transmission
static SemaphoreHandle_t ipmb_rec_semphr = NULL;

// Mutex to avoid printf superpositioning.
// static SemaphoreHandle_t printf_mutex = NULL;

// config i2c mode functions
esp_err_t set_ipmba_channel_as_master(void);
esp_err_t set_ipmbb_channel_as_master(void);
esp_err_t set_ipmba_channel_as_slave (void);
esp_err_t set_ipmbb_channel_as_slave (void);

 // uint8_t ipmc_ios_read_haddress(void);

/*
 * Configuration functions for i2c modes in ESP32
 */
// functions for IPMB channel A
esp_err_t set_ipmba_channel_as_master (void)
{
    static int i2c_ipmba_port = I2C_NUM_0;
    static i2c_config_t conf_ipmba_master;
    static esp_err_t ipmba_status;
    
    if (i2c0_mode == IIC_MODE_SLAVE)
    {
        // The task that holds until a mssage arrives is kept in a loop.
        // For this reason when the master driver must be installed to transmit
        // the receiver loop and current environment of the slave driver must be deleted.
        // This must be done first exiting the loop (deleting the task)
        // and then deleting the i2c driver, using the provided function.
        vTaskSuspendAll();
        vTaskDelete(i2c_0_receiver_manager_task_ptr);
        xTaskResumeAll();
        
        i2c_driver_delete(i2c_ipmba_port);
        i2c0_mode = IIC_MODE_MASTER;
        conf_ipmba_master.sda_io_num = GPIO_NUM_22;
        conf_ipmba_master.sda_pullup_en = GPIO_PULLUP_ENABLE;
        conf_ipmba_master.scl_io_num = GPIO_NUM_23;
        conf_ipmba_master.scl_pullup_en = GPIO_PULLUP_ENABLE;
        conf_ipmba_master.mode = I2C_MODE_MASTER;
        conf_ipmba_master.master.clk_speed = I2C_SCLK_RATE;
        i2c_param_config(i2c_ipmba_port, &conf_ipmba_master);
        ipmba_status = i2c_driver_install(i2c_ipmba_port, conf_ipmba_master.mode, IPMB_BUFF_SIZE, IPMB_BUFF_SIZE, ESP_INTR_FLAG_IRAM);
        
        return ipmba_status;
    }
    else // mode already configured
        return ESP_OK; 
}

esp_err_t set_ipmba_channel_as_slave (void)
{
    static int i2c_ipmba_port = I2C_NUM_0;
    static i2c_config_t conf_ipmba_slave;
    static esp_err_t ipmba_status;    
    
    if (i2c0_mode == IIC_MODE_MASTER)
    {    
        i2c_driver_delete(i2c_ipmba_port);
    
        conf_ipmba_slave.sda_io_num = GPIO_NUM_22;
        conf_ipmba_slave.sda_pullup_en = GPIO_PULLUP_ENABLE;
        conf_ipmba_slave.scl_io_num = GPIO_NUM_23;
        conf_ipmba_slave.scl_pullup_en = GPIO_PULLUP_ENABLE;
        conf_ipmba_slave.mode = I2C_MODE_SLAVE;
        conf_ipmba_slave.slave.addr_10bit_en = 0;
        conf_ipmba_slave.slave.slave_addr = ipmc_ios_read_haddress();// 0x43h
        i2c_param_config(i2c_ipmba_port, &conf_ipmba_slave);
        ipmba_status = i2c_driver_install(i2c_ipmba_port, conf_ipmba_slave.mode, IPMB_BUFF_SIZE, IPMB_BUFF_SIZE, ESP_INTR_FLAG_LEVEL1);    
        i2c0_mode = IIC_MODE_SLAVE;
        
        // When returning to the slave mode, the task that holds until a message arrives must be relaunched
        vTaskSuspendAll();
        xTaskCreate(i2c_0_receiver_manager_task, ( const char * ) "I2C0", 8000, NULL, tskIDLE_PRIORITY, &i2c_0_receiver_manager_task_ptr );
        xTaskResumeAll();
        
        return ipmba_status;
    }
    else // mode already configured
        return ESP_OK; 
}

// functions for IPMB channel B
esp_err_t set_ipmbb_channel_as_master(void)
{
    static int i2c_ipmbb_port = I2C_NUM_1;
    static i2c_config_t conf_ipmbb_master;
    static esp_err_t ipmbb_status;

    if (i2c1_mode == IIC_MODE_SLAVE)    
    {
        vTaskSuspendAll();
        vTaskDelete(i2c_1_receiver_manager_task_ptr);
        xTaskResumeAll();
        
        i2c_driver_delete(i2c_ipmbb_port);
        i2c1_mode = IIC_MODE_MASTER;
        conf_ipmbb_master.sda_io_num = GPIO_NUM_18;
        conf_ipmbb_master.sda_pullup_en = GPIO_PULLUP_ENABLE;
        conf_ipmbb_master.scl_io_num = GPIO_NUM_19;
        conf_ipmbb_master.scl_pullup_en = GPIO_PULLUP_ENABLE;
        conf_ipmbb_master.master.clk_speed = I2C_SCLK_RATE;
        conf_ipmbb_master.mode = I2C_MODE_MASTER;
        i2c_param_config(i2c_ipmbb_port, &conf_ipmbb_master);
        ipmbb_status = i2c_driver_install(i2c_ipmbb_port, conf_ipmbb_master.mode, IPMB_BUFF_SIZE, IPMB_BUFF_SIZE, ESP_INTR_FLAG_IRAM);
        
        return ipmbb_status; 
    }
    else // mode already configured
        return ESP_OK; 
}

esp_err_t set_ipmbb_channel_as_slave(void)
{
    static int i2c_ipmbb_port = I2C_NUM_1;
    static i2c_config_t conf_ipmbb_slave;
    static esp_err_t ipmbb_status;
    if (i2c1_mode == IIC_MODE_MASTER)        
    {
        i2c_driver_delete(i2c_ipmbb_port);

        conf_ipmbb_slave.sda_io_num = GPIO_NUM_18;
        conf_ipmbb_slave.sda_pullup_en = GPIO_PULLUP_ENABLE;
        conf_ipmbb_slave.scl_io_num = GPIO_NUM_19;
        conf_ipmbb_slave.scl_pullup_en = GPIO_PULLUP_ENABLE;
        conf_ipmbb_slave.mode = I2C_MODE_SLAVE;
        conf_ipmbb_slave.slave.addr_10bit_en = 0;
        conf_ipmbb_slave.slave.slave_addr = ipmc_ios_read_haddress(); // 0x43h
        i2c_param_config(i2c_ipmbb_port, &conf_ipmbb_slave);
        ipmbb_status = i2c_driver_install(i2c_ipmbb_port, conf_ipmbb_slave.mode, IPMB_BUFF_SIZE, IPMB_BUFF_SIZE, ESP_INTR_FLAG_LEVEL1);    
        i2c1_mode = IIC_MODE_SLAVE;

        vTaskSuspendAll();
        xTaskCreate(i2c_1_receiver_manager_task, ( const char * ) "I2C1", 8000, NULL, tskIDLE_PRIORITY, &i2c_1_receiver_manager_task_ptr );
        xTaskResumeAll();
        return ipmbb_status; 
    }
    else // mode already configured
        return ESP_OK; 
}


/*
 * Initialization of all IPMC peripherals in the context of ESP32 setup
 */
int periphs_init(void)
{
  int status_global = 1;

    /* Pin Direction */
    gpio_set_direction(GPIO_NUM_14,  GPIO_MODE_INPUT); // HA0
    gpio_set_direction(GPIO_NUM_27,  GPIO_MODE_INPUT); // HA1
    gpio_set_direction(GPIO_NUM_26,  GPIO_MODE_INPUT); // HA2
    gpio_set_direction(GPIO_NUM_25,  GPIO_MODE_INPUT); // HA3
    gpio_set_direction(GPIO_NUM_33,  GPIO_MODE_INPUT); // HA4
    gpio_set_direction(GPIO_NUM_32,  GPIO_MODE_INPUT); // HA5
    gpio_set_direction(GPIO_NUM_35,  GPIO_MODE_INPUT); // HA6
    gpio_set_direction(GPIO_NUM_34,  GPIO_MODE_INPUT); // HA7
    
    gpio_set_direction(GPIO_NUM_16,  GPIO_MODE_INPUT); // Handle Switch
    
    gpio_set_direction(GPIO_NUM_17,  GPIO_MODE_OUTPUT);// LED on DIMM adapter
    gpio_set_direction(GPIO_NUM_4,   GPIO_MODE_OUTPUT);// EXT_RST
    //gpio_set_direction(GPIO_NUM_12,  GPIO_MODE_OUTPUT);// PIN 12 OUT - 12V_Enable COMMENTED FOR SAFETY.
    
    // enable GPIO pull up resistor
    esp_err_t retval_pullup;
    retval_pullup = gpio_pullup_en(GPIO_NUM_16);
    if (retval_pullup != ESP_OK) {status_global = 0;} // config fail
    
    retval_pullup = gpio_pullup_en(GPIO_NUM_14);
    if (retval_pullup != ESP_OK) {status_global = 0;} // config fail
    
    retval_pullup = gpio_pullup_en(GPIO_NUM_27);
    if (retval_pullup != ESP_OK) {status_global = 0;} // config fail
    
    retval_pullup = gpio_pullup_en(GPIO_NUM_26);
    if (retval_pullup != ESP_OK) {status_global = 0;} // config fail
    
    retval_pullup = gpio_pullup_en(GPIO_NUM_25);
    if (retval_pullup != ESP_OK) {status_global = 0;} // config fail
    
    retval_pullup = gpio_pullup_en(GPIO_NUM_33);
    if (retval_pullup != ESP_OK) {status_global = 0;} // config fail
    
    retval_pullup = gpio_pullup_en(GPIO_NUM_32);
    if (retval_pullup != ESP_OK) {status_global = 0;} // config fail
    
    retval_pullup = gpio_pullup_en(GPIO_NUM_35);
    if (retval_pullup != ESP_OK) {status_global = 0;} // config fail
    
    retval_pullup = gpio_pullup_en(GPIO_NUM_34);
    if (retval_pullup != ESP_OK) {status_global = 0;} // config fail
        
        
     
  
  //printf_mutex = xSemaphoreCreateMutex();
    ipmb_rec_semphr = xSemaphoreCreateBinary();
  
   /*
    * Initializes IPMB-A
    *     pins used:
    *     G23 - SCL
    *     G22 - SDA
    */
    int i2c_ipmba_port = I2C_NUM_0; // IPMB-A is configured in I2C0 of ESP32
    
    i2c_config_t conf_ipmba_master;
    conf_ipmba_master.sda_io_num = GPIO_NUM_22;
    conf_ipmba_master.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf_ipmba_master.scl_io_num = GPIO_NUM_23;
    conf_ipmba_master.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf_ipmba_master.mode = I2C_MODE_MASTER;
    conf_ipmba_master.master.clk_speed = I2C_SCLK_RATE;
    
    i2c_param_config(i2c_ipmba_port, &conf_ipmba_master);
    esp_err_t ipmba_status = i2c_driver_install(i2c_ipmba_port, conf_ipmba_master.mode, IPMB_BUFF_SIZE, IPMB_BUFF_SIZE, ESP_INTR_FLAG_IRAM);
    if ( ipmba_status!=ESP_OK )
        status_global = 0; // config fail
        
    i2c0_mode = IIC_MODE_MASTER;
    
    i2c_driver_delete(i2c_ipmba_port);
    
    i2c_config_t conf_ipmba_slave;
    conf_ipmba_slave.sda_io_num = GPIO_NUM_22;
    conf_ipmba_slave.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf_ipmba_slave.scl_io_num = GPIO_NUM_23;
    conf_ipmba_slave.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf_ipmba_slave.mode = I2C_MODE_SLAVE;
    conf_ipmba_slave.slave.addr_10bit_en = 0;
    conf_ipmba_slave.slave.slave_addr = ipmc_ios_read_haddress();// 0x43h
    i2c_param_config(i2c_ipmba_port, &conf_ipmba_slave);
    ipmba_status = i2c_driver_install(i2c_ipmba_port, conf_ipmba_slave.mode, IPMB_BUFF_SIZE, IPMB_BUFF_SIZE, ESP_INTR_FLAG_LEVEL1);    
    i2c0_mode = IIC_MODE_SLAVE;
    if ( ipmba_status!=ESP_OK )
        status_global = 0; // config fail

   /*
    * Initializes IPMB-B
    * 
    *     pins used:
    *     G19 - SCL
    *     G18 - SDA
    */
    int i2c_ipmbb_port = I2C_NUM_1; // IPMB-B is configured in I2C1 of ESP32
    
    i2c_config_t conf_ipmbb_master;
    conf_ipmbb_master.sda_io_num = GPIO_NUM_18;
    conf_ipmbb_master.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf_ipmbb_master.scl_io_num = GPIO_NUM_19;
    conf_ipmbb_master.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf_ipmbb_master.mode = I2C_MODE_MASTER;
    conf_ipmbb_master.master.clk_speed = I2C_SCLK_RATE;
    i2c_param_config(i2c_ipmbb_port, &conf_ipmbb_master);
    esp_err_t ipmbb_status = i2c_driver_install(i2c_ipmbb_port, conf_ipmbb_master.mode, IPMB_BUFF_SIZE, IPMB_BUFF_SIZE, ESP_INTR_FLAG_IRAM);
    if ( ipmbb_status!=ESP_OK )
        status_global = 0; // config fail
        
    i2c1_mode = IIC_MODE_MASTER;
    
    i2c_driver_delete(i2c_ipmbb_port);
    
    i2c_config_t conf_ipmbb_slave;
    conf_ipmbb_slave.sda_io_num = GPIO_NUM_18;
    conf_ipmbb_slave.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf_ipmbb_slave.scl_io_num = GPIO_NUM_19;
    conf_ipmbb_slave.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf_ipmbb_slave.mode = I2C_MODE_SLAVE;
    conf_ipmbb_slave.slave.addr_10bit_en = 0;
    conf_ipmbb_slave.slave.slave_addr = ipmc_ios_read_haddress(); // 0x43h
    i2c_param_config(i2c_ipmbb_port, &conf_ipmbb_slave);
    ipmbb_status = i2c_driver_install(i2c_ipmbb_port, conf_ipmbb_slave.mode, IPMB_BUFF_SIZE, IPMB_BUFF_SIZE, ESP_INTR_FLAG_LEVEL1);    
    i2c1_mode = IIC_MODE_SLAVE;
    if ( ipmbb_status!=ESP_OK )
        status_global = 0; // config fail

    // Create the receiver tasks
    xTaskCreate(i2c_0_receiver_manager_task, 				
                ( const char * ) "I2C0", 	         
                8000, 	         
                NULL, 						         
                tskIDLE_PRIORITY,		         
                &i2c_0_receiver_manager_task_ptr );

    xTaskCreate(i2c_1_receiver_manager_task,
                ( const char * ) "I2C1",
                8000,
                NULL,
                tskIDLE_PRIORITY,
                &i2c_1_receiver_manager_task_ptr );
        
    // initialization status for the IOs
    if (status_global == 1)
      ipmc_ios_ready_flag = pdTRUE;
  /*
   * If any of the previous initialization fail, return 0.
   * Otherwise return 1 (all initialization successful)
   */
  return status_global;
  
}

/*
 * Check if the IOs are initialized.
 * 
 * Returns TRUE if ipmc_ios_init() ran all the initializations successfully.
 */
_Bool ipmc_ios_ready(void)
{
	return ipmc_ios_ready_flag;
}

/*
 * Read the state of the ATCA handle
 * 
 * Mechanically, when the handle is CLOSED, the pin is grounded, giving a LOW.
 * When the handle is OPEN, the micro-switch is also open, and a pull-up 
 * resistor imposes a HIGH.
 * 
 * return value:
 *   1: handle is OPEN
 *   0: handle is CLOSED
 */
int ipmc_ios_read_handle(void)
{
  
  if( gpio_get_level(GPIO_NUM_16) > 0 )
    return 1; // OPEN
  else
    return 0; // CLOSED

}


/*
 * The Hardware Address of the IPMC (slave addr) is read from the ATCA backplane pins
 * A party check is considered for the address (parity must be ood)
 */
uint8_t ipmc_ios_read_haddress(void)
{
  
  int i;
  uint8_t  HA_bit[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  uint8_t  HA_num;
  int parity_odd;
  
  /* Get the HA0 from the bit 1 in PL GPIO */
  if( gpio_get_level(GPIO_NUM_14) > 0)
    HA_bit[0] = 1;
  
  /* Get HA1 to HA7 from PS GPIO1 */ 
  if( gpio_get_level(GPIO_NUM_27) > 0 ) // HA1
    HA_bit[1] = 1;
  if( gpio_get_level(GPIO_NUM_26) > 0 ) // HA2
    HA_bit[2] = 1;
  if( gpio_get_level(GPIO_NUM_25) > 0 ) // HA3
    HA_bit[3] = 1;
  if( gpio_get_level(GPIO_NUM_33) > 0 ) // HA4
    HA_bit[4] = 1;
  if( gpio_get_level(GPIO_NUM_32) > 0 ) // HA5
    HA_bit[5] = 1;
  if( gpio_get_level(GPIO_NUM_35) > 0 ) // HA6
    HA_bit[6] = 1;
  if( gpio_get_level(GPIO_NUM_34) > 0 ) // HA7
    HA_bit[7] = 1;
  
  return 0x43;  // THIS IS ONLY FOR TESTING
  
  /* Calculate parity */
  parity_odd = 0; // initialize as EVEN
  for(i=0; i<8; i++)
    if(HA_bit[i] == 1)
      parity_odd = ~parity_odd; // flip parity
  
  /* Result */
  HA_num = 0;
  if( parity_odd )
  {
    for(i=0; i<=6; i++)
      HA_num |= (HA_bit[i]<<i);
  
      return HA_num; // 7bit addr
  }
  else
    return HA_PARITY_FAIL; //parity fail (must be ODD)
}


/*
 * Set the IPMB address.
 */
void ipmc_ios_ipmb_set_addr(uint8_t addr)
 {
    // Convert into 7-bit format to be used by the I2C driver
    ipmb_addr = addr >> 1;
   
    set_ipmba_channel_as_slave(); 
    set_ipmbb_channel_as_slave();
    iic0_recv_len = 0;
    iic1_recv_len = 0;
 }


/*
 * Sending functions for IPMB-A and IPMB-B lines
 * 
 * The send function follows the procedures to write in the I2C line using the provided driver from IDF platform.
 * 
 */

int ipmc_ios_ipmba_send(uint8_t *MsgPtr, int ByteCount)
{
    uint16_t dest_addr;
    esp_err_t ret;
    
    // convert port into master
    ret = set_ipmba_channel_as_master(); 
    if (ret != ESP_OK)
    {
        printf("Master config failed in I2C0\n");
        return IPMB_SEND_FAIL;
    }
    
    if (i2c0_mode == I2C_MODE_MASTER)
    {
        // Gets destination address from the message header
        // It needs to be in 7bit format (no, it does not in the ESP32 i2c driver - I think...)
        dest_addr = (uint16_t)MsgPtr[0] >> 1;
    
        // for esp "master_write_slave" function a command queue must be created as it follows
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd); // start command queue
        i2c_master_write_byte(cmd,  (dest_addr << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN); // send the slave address
        i2c_master_write(cmd, &MsgPtr[1], ByteCount-1, ACK_CHECK_EN); // send the payload data
        i2c_master_stop(cmd); // finsh command queue
        // execute the defined commands
        ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(100));
        i2c_cmd_link_delete(cmd);
            
        // Convert the port back into Slave
        esp_err_t ret_cfg = set_ipmba_channel_as_slave ();
        if (ret_cfg != ESP_OK)
        {
            printf("Slave config failed in I2C0\n");
            return IPMB_SEND_FAIL;
        }
        
        iic0_recv_len = 0;
        
        // check for transmission return satus 
        if      (ret == ESP_ERR_INVALID_ARG) {printf("IPMBa write ERROR: ESP_ERR_INVALID_ARG\r\n"); return IPMB_SEND_FAIL; }// for debug purposes
        else if (ret == ESP_FAIL) {printf("IPMBa write ERROR: ESP_FAIL\r\n"); return IPMB_SEND_FAIL; }// for debug purposes
        else if (ret == ESP_ERR_INVALID_STATE) {printf("IPMBa write ERROR: ESP_ERR_INVALID_STATE\r\n"); return IPMB_SEND_FAIL; }// for debug purposes
        else if (ret == ESP_ERR_TIMEOUT) {printf("IPMBa write ERROR: ESP_ERR_TIMEOUT\r\n"); return IPMB_SEND_FAIL; }// abort if bus is busy or transmission timeout
        
        return IPMB_SEND_DONE;
    }
    else 
        return IPMB_SEND_FAIL;
}


int ipmc_ios_ipmbb_send(uint8_t *MsgPtr, int ByteCount)
{
    uint16_t dest_addr;
    esp_err_t ret;
    
    // convert port into master
    ret = set_ipmbb_channel_as_master();
    if (ret != ESP_OK)
    {
        printf("Master config failed in I2C1\n");
        return IPMB_SEND_FAIL;
    }
    if (i2c1_mode == I2C_MODE_MASTER)
    {
        // Gets destination address from the message header
        dest_addr = (uint16_t)MsgPtr[0] >> 1;
        
        // for esp "master_write_slave" function a command queue must be created as it follows
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd); // start command queue
        i2c_master_write_byte(cmd,  (dest_addr << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN); // send slave address
        i2c_master_write(cmd, &MsgPtr[1], ByteCount-1, ACK_CHECK_EN); // send the payload data
        i2c_master_stop(cmd); // finsh command queue
        // execute commands
        ret = i2c_master_cmd_begin(I2C_NUM_1, cmd, pdMS_TO_TICKS(100));
        i2c_cmd_link_delete(cmd);
        
        // Convert the port back into Slave
        esp_err_t ret_cfg = set_ipmbb_channel_as_slave();
        if (ret_cfg != ESP_OK)
        {
            printf("Slave config failed in I2C1\n");
            return IPMB_SEND_FAIL;
        }
        
        iic1_recv_len = 0;
        
        // check for transmission return satus 
        if      (ret == ESP_ERR_INVALID_ARG) {printf("IPMBb write ERROR: ESP_ERR_INVALID_ARG\r\n"); return IPMB_SEND_FAIL; }// for debug purposes
        else if (ret == ESP_FAIL) {printf("IPMBb write ERROR: ESP_FAIL\r\n"); return IPMB_SEND_FAIL; }// for debug purposes
        else if (ret == ESP_ERR_INVALID_STATE) {printf("IPMBb write ERROR: ESP_ERR_INVALID_STATE\r\n"); return IPMB_SEND_FAIL; }// for debug purposes
        else if (ret == ESP_ERR_TIMEOUT) {printf("IPMBb write ERROR: ESP_ERR_TIMEOUT\r\n"); return IPMB_SEND_FAIL; }// abort if bus is busy or transmission timeout
        
        return IPMB_SEND_DONE;
    }
    else
        return IPMB_SEND_FAIL;
}


int ipmc_ios_ipmba_read(uint8_t *MsgPtr)
{
    int i;
    // Length zero means no message received
    if(iic0_recv_len <= 0)
        return 0;
    
    // Copy message to the user provided buffer.
    // The driver doesn't get the slave address in the message. It need to be
    // included artificially into the user buffer in 8bit format.
    MsgPtr[0] = ipmb_addr << 1;
    for(i=0; (i < iic0_recv_len); i++) 
        MsgPtr[i+1] = ipmba_input_buffer[i];
    
    // Restart Receiving...
    iic0_recv_len = 0;

    // Returns the amount of copied bytes
    return i+1; 

}

int ipmc_ios_ipmbb_read(uint8_t *MsgPtr )
{
    int i;
    
    // Length zero means no message received
    if(iic1_recv_len <= 0)
        return 0;
    
    // Copy message to the user provided buffer.
    // The driver doesn't get the slave address in the message. It need to be
    // included artifitially into the user buffer in 8bit format.
    MsgPtr[0] = ipmb_addr << 1;
    for(i=0; (i < iic1_recv_len); i++) 
        MsgPtr[i+1] = ipmbb_input_buffer[i];
    
    // Restart Receiving...
    iic1_recv_len = 0;

    // Returns the amount of copied bytes
    return i+1; 
}

void ipmc_ios_ipmb_wait_input_msg(void)
{
    xSemaphoreTake (ipmb_rec_semphr, portMAX_DELAY);
}


//// Task for receiving the messages through I2C channel 0
//void i2c_0_receiver_manager_task( void *pvParameters )
//{
//    int status;
//    for(;;)
//    {
//        if ( i2c0_mode == IIC_MODE_SLAVE )
//        {  
//            size_t buffer_free_size = IPMB_BUFF_SIZE;
//            printf("WAITING... (0)\n");
//            status = i2c_slave_receive_message(I2C_NUM_0, ipmba_input_buffer, &buffer_free_size, portMAX_DELAY);
//            printf("EXITING LOOP (0)\n");
//            if (status == I2C_SLAVE_RECEIVE_MESSAGE_RETVAL_SUCCESS)
//            {    
//                iic0_recv_len = IPMB_BUFF_SIZE - buffer_free_size;
//                if (iic0_recv_len > 0)
//                    xSemaphoreGive(ipmb_rec_semphr); // message has arrived at i2c0
//            }        
//        }
//    }
//}
//
//// Task for receiving the messages through I2C channel 1
//void i2c_1_receiver_manager_task( void *pvParameters )
//{
//    int status;
//    for(;;)
//    {
//        if ( i2c1_mode == IIC_MODE_SLAVE )   
//        {
//            size_t buffer_free_size = IPMB_BUFF_SIZE;
//            printf("WAITING... (1)\n");
//            status = i2c_slave_receive_message(I2C_NUM_1, ipmbb_input_buffer, &buffer_free_size, portMAX_DELAY);
//            printf("EXITING LOOP (1)\n");
//            if (status == I2C_SLAVE_RECEIVE_MESSAGE_RETVAL_SUCCESS)
//            {    
//                iic1_recv_len = IPMB_BUFF_SIZE - buffer_free_size;
//                if (iic1_recv_len > 0)
//                    xSemaphoreGive(ipmb_rec_semphr); // message has arrived at i2c1
//            }
//        }
//    }
//}

// Task for receiving the messages through I2C channel 0
void i2c_0_receiver_manager_task( void *pvParameters )
{
    for(;;)
    {
        if ( i2c0_mode == IIC_MODE_SLAVE )
        {  
            // int i2c_slave_receive_message(i2c_port_t i2c_num, uint8_t* user_buffer, size_t* user_buffer_free_size, TickType_t const ticks_timeout)
            size_t ipmba_input_buffer_free_space = IPMB_BUFF_SIZE;
            int const retcode = i2c_slave_receive_message(I2C_NUM_0, ipmba_input_buffer, &ipmba_input_buffer_free_space, portMAX_DELAY);
            iic0_recv_len = IPMB_BUFF_SIZE - ipmba_input_buffer_free_space;
            
            if (iic0_recv_len > 0 && retcode == 0)
                xSemaphoreGive(ipmb_rec_semphr); // message has arrived at i2c0 
        }
    }
}

// Task for receiving the messages through I2C channel 1
void i2c_1_receiver_manager_task( void *pvParameters )
{
    for(;;)
    {
        if ( i2c1_mode == IIC_MODE_SLAVE )
        {  
            // int i2c_slave_receive_message(i2c_port_t i2c_num, uint8_t* user_buffer, size_t* user_buffer_free_size, TickType_t const ticks_timeout)
            size_t ipmbb_input_buffer_free_space = IPMB_BUFF_SIZE;
            int const retcode = i2c_slave_receive_message(I2C_NUM_1, ipmbb_input_buffer, &ipmbb_input_buffer_free_space, portMAX_DELAY);
            iic1_recv_len = IPMB_BUFF_SIZE - ipmbb_input_buffer_free_space;
            
            if (iic1_recv_len > 0 && retcode == 0)
                xSemaphoreGive(ipmb_rec_semphr); // message has arrived at i2c0
        }
    }
}

/*
 * Control the Blue Led
 */
void ipmc_ios_blue_led(int blue_led_state)
{
	if (blue_led_state == 0)
		gpio_set_level(GPIO_NUM_17, 0); // Output level 0: low - OFF
	else
		gpio_set_level(GPIO_NUM_17, 1); // Output level 1: high - ON
}


