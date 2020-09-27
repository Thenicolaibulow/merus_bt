/* Merus Bluetooth Audio Streaming

   Written by Jørgen Kragh Jakobsen
   Infineon Merus Digital Audio
   Aug 2020

   This code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/


#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"

#include "driver/gpio.h"
#include "sdkconfig.h"

#include "MerusAudio.h"
#include "ma120x0.h"

#include "bt_app_core.h"
#include "bt_app_av.h"
#include "esp_spp_api.h"

#include "ma_bt_a2dp.h"
#include "protocol.h"
#include "dsp_processor.h"

static const char *TAG = "Main";
xQueueHandle prot_queue;
xQueueHandle i2s_queue;

extern enum dspFlows dspFlow;
unsigned samplerate = 44100;
extern uint32_t spp_handle; 

void app_main(void)
{   esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
   
    ma_bt_start();

    // Specific to MOUSAI Board. 
      PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO0_U, FUNC_GPIO0_CLK_OUT1);               // MCLK Generator for DSP ~24MHz
      WRITE_PERI_REG(PIN_CTRL, READ_PERI_REG(PIN_CTRL) & 0xFFFFFFF0);     
      setup_ma120x0_0x20();                                                       // Init MA on ADDR 0x20 & 0x21.
      setup_ma120x0_0x21();
    // Specific to MOUSAI Board. 

    dsp_i2s_task_init(samplerate);
    dspFlow = dspfStereo;                           // ^ Set default dspflow to stereo. Can be changed on the fly in the app. 
    
                                            
          //dsp_setup_flow(200.0);                  // Init crossover filters for biAmp dspflow.
          //dsp_setup_hshelf(150.0, 0, 0.707);      // Init highshelf filter.
          //dsp_setup_dynbass(150.0, 0, 0.707);     // Init lowshelf filter.

    dsp_init_filter(0, 120);  // Inits a l-shelf @ 150Hz, gain = 0.
    dsp_init_filter(1, 2000); // Inits a h-shelf @ 3KHz, gain = 0.

    // Gain set to 0, such that the system doesn't spin out off control straigth away. Adjust it in the APP (Be warned, Ear Rape!!) 
    // Lack of headroom in the main mix, when adjusted, is the current hypotesis to this behavior.
    // Also, these calls should be moved to dsp_processor/*new func*: Init DSP FLow

    prot_queue = xQueueCreate(10, sizeof(uint8_t *) );
    xTaskCreatePinnedToCore(protocolHandlerTask, "prot_handler_task", 2*1024, NULL, 5, NULL,0);
    
    while(1)
    { uint8_t amp_state[8];
      for (;;)   // read back 96 to 102 (7 bytes)
      { ma_read(0x20,1, MA_dcu_mon0__PM_mon__a, amp_state, 7); 
        if (spp_handle != 0)
        {   //printf("BT tx\n");
            esp_spp_write(spp_handle, 7, amp_state);
        }
      
       vTaskDelay(pdMS_TO_TICKS(200));
      }
   }
}
