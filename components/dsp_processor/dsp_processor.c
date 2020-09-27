#include <stdint.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "driver/i2s.h"
#include "freertos/ringbuf.h"
#include "dsps_biquad_gen.h"
#include "dsps_biquad.h"

#include "dsp_processor.h"

static xTaskHandle s_dsp_i2s_task_handle = NULL;
static RingbufHandle_t s_ringbuf_i2s = NULL;
extern xQueueHandle i2s_queue;
extern unsigned samplerate ;  


enum dspFlows dspFlow;
uint32_t freqBT = 1000; 
uint32_t dynbassFreq = 400;  
uint8_t gain = 0;   // gain db /4 
uint8_t muteCH[4] = {0};
ptype_t bq[6];

void setup_dsp_i2s(uint32_t sample_rate)
{
  i2s_config_t i2s_config0 = {
    .mode = I2S_MODE_MASTER | I2S_MODE_TX,                                    // Only TX
    .sample_rate = samplerate,
    .bits_per_sample = 32,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,                             //2-channels
    .communication_format = I2S_COMM_FORMAT_I2S, 
    .dma_buf_count = 8, 
    .dma_buf_len = 512,
    .use_apll = true,
    .fixed_mclk = 0,
    .tx_desc_auto_clear = true                                                //Auto clear tx descriptor on underflow
  };
  
  i2s_driver_install(0, &i2s_config0, 4, &i2s_queue);
  i2s_zero_dma_buffer(0);

  i2s_pin_config_t pin_config0 = {
    .bck_io_num = CONFIG_I2S_BCK_PIN,
    .ws_io_num =  CONFIG_I2S_LRCK_PIN,
    .data_out_num = CONFIG_I2S_DATA_PIN,
    .data_in_num = -1                                                       //Not used
  };
  printf("%d %d %d\n",CONFIG_I2S_BCK_PIN,CONFIG_I2S_LRCK_PIN,CONFIG_I2S_DATA_PIN);

  i2s_set_pin(0, &pin_config0);
  gpio_set_drive_capability(CONFIG_I2S_BCK_PIN,0);
  gpio_set_drive_capability(CONFIG_I2S_LRCK_PIN,0);
  gpio_set_drive_capability(CONFIG_I2S_DATA_PIN,0);
}


static void dsp_i2s_task_handler(void *arg)
{ uint32_t cnt = 0;
  uint8_t *audio = NULL;
  float sbuffer0[1024];
  float sbuffer1[1024];
  float sbuffer2[1024];
  float sbuffer3[1024];
  float sbuffer4[1024];
  // float sbuffer5[1024]; Seems to be causeing memory overflow!
  // float sbuffer6[1024];
  // float sbuffer7[1024];
  // float sbuffer8[1024];
  float sbufout0[1024];
  float sbufout1[1024];
  float sbufout2[1024];
  float sbuftmp0[1024];

  float chmax[2] = {0}; 
  float chmaxpost[2] = {0}; 
  
  uint8_t dsp_audio[4*1024];
  uint8_t dsp_audio1[4*1024];

  size_t chunk_size = 0;
  size_t bytes_written = 0;

  for (;;) {
    cnt++;
    audio = (uint8_t *)xRingbufferReceiveUpTo(s_ringbuf_i2s, &chunk_size, (portTickType)portMAX_DELAY,960);
    if (chunk_size !=0 ){
        int16_t len = chunk_size/4;
         uint8_t *data_ptr = audio;
        float prescale0 = 0.25; //1/sqrtf(pow(10, bq[4].gain/20.0));   
        float prescale1 = 0.25; //1/sqrtf(pow(10, bq[5].gain/20.0));      
        if ((cnt%200 == 0) || (chmaxpost[0] > 0.9) )
        { ESP_LOGI("I2S", "Chunk :%d, %f, %f, %f , %f, %f ",chunk_size,prescale0, chmax[0], chmax[1], chmaxpost[0],chmaxpost[1]);
          chmax[0] = 0;
          chmax[1] = 0;
          chmaxpost[0] = 0; 
          chmaxpost[1] = 0;
        }
       
        for (uint16_t i=0;i<len;i++)
        { 
          sbuffer0[i] = ((float) ((int16_t) (audio[i*4+1]<<8) + audio[i*4+0]))/32768;
          sbuffer1[i] = ((float) ((int16_t) (audio[i*4+3]<<8) + audio[i*4+2]))/32768;
          if (dspFlow == dspfFilters)
          { sbuffer0[i] = prescale0 * sbuffer0[i];              // I assume that this causes a lower overall volume, thus adding headspace for filters.
            sbuffer1[i] = prescale1 * sbuffer1[i];
            sbuffer2[i] = prescale1 * sbuffer2[i];
            sbuffer3[i] = prescale1 * sbuffer3[i];
            sbuffer4[i] = prescale1 * sbuffer4[i];
          } 

          sbuffer2[i] = ((sbuffer0[i]/2) +  (sbuffer1[i]/2));
          chmax[0] = (sbuffer0[i]>chmax[0])?sbuffer0[i]:chmax[0];
          chmax[1] = (sbuffer1[i]>chmax[1])?sbuffer1[i]:chmax[1];
        }

        switch (dspFlow) {
          case dspfStereo :
            { 
              for (uint16_t i=0; i<len; i++)
              { audio[i*4+0] = (muteCH[0] == 1)? 0 : audio[i*4+0];
                audio[i*4+1] = (muteCH[0] == 1)? 0 : audio[i*4+1];
                audio[i*4+2] = (muteCH[1] == 1)? 0 : audio[i*4+2];
                audio[i*4+3] = (muteCH[1] == 1)? 0 : audio[i*4+3];
              }
              i2s_write_expand(0, (char*)audio, chunk_size,16,32, &bytes_written, portMAX_DELAY);
            }
            break;

          case dspfFilters : 
            { 

                // Create filter pipeline.
                // Sbuffer2 isn't used, as it's reserved for biamp operation, which is downmixed to mono. : sbuffer2[i] = ((sbuffer0[i]/2) +  (sbuffer1[i]/2));

                dsps_biquad_f32_ae32(sbuffer0, sbuffer3, len, bq[0].coeffs, bq[0].w); 
                dsps_biquad_f32_ae32(sbuffer1, sbuffer4, len, bq[1].coeffs, bq[1].w); 
                dsps_biquad_f32_ae32(sbuffer3, sbufout0, len, bq[2].coeffs, bq[2].w);   
                dsps_biquad_f32_ae32(sbuffer4, sbufout1, len, bq[3].coeffs, bq[3].w);                 

                // dsps_biquad_f32_ae32(sbuffer5, sbuffer7, len, bq[4].coeffs, bq[4].w);  // Causes memory overflow. It seems we need to be able to reuse the sbuffer's
                // dsps_biquad_f32_ae32(sbuffer6, sbuffer8, len, bq[5].coeffs, bq[5].w);  throughout the biquad pipes..
                // dsps_biquad_f32_ae32(sbuffer7, sbufout0, len, bq[6].coeffs, bq[6].w); 
                // dsps_biquad_f32_ae32(sbuffer8, sbufout1, len, bq[7].coeffs, bq[7].w); 

                // Pipeline: L-shelf [bq[0-1]] -> H-shelf [bq[2-3]] -> Peaking [bq[4-5]] -> Notch [bq[6-7]]
                // Even though some biquads might not have any effect (gain), this pipeline should allow us to test each biquad.
                
                int16_t valint[2];

                  for (uint16_t i=0; i<len; i++){ 

                    chmaxpost[0] = (sbufout0[i]>chmax[0])?sbufout0[i]:chmaxpost[0];
                    chmaxpost[1] = (sbufout1[i]>chmax[1])?sbufout1[i]:chmaxpost[1];
          
                    valint[0] = (muteCH[0] == 1) ? (int16_t) 0 : (int16_t) (sbufout0[i]*32768);     // JKJ's 16 -> 32 bit magic.
                    valint[1] = (muteCH[1] == 1) ? (int16_t) 0 : (int16_t) (sbufout1[i]*32768);
                    dsp_audio[i*4+0] = (valint[0] & 0xff);                                          // Populates I2S buffer with filtermodified data.
                    dsp_audio[i*4+1] = ((valint[0] & 0xff00)>>8);
                    dsp_audio[i*4+2] = (valint[1] & 0xff);
                    dsp_audio[i*4+3] = ((valint[1] & 0xff00)>>8);

                }

            i2s_write_expand(0, (char*)dsp_audio, chunk_size,16,32, &bytes_written, portMAX_DELAY); // Writes the modified I2S Buffer.

            }
            break;

          case dspfBiamp :
            { 
              // Process audio ch0 LOW PASS FILTER
              dsps_biquad_f32_ae32(sbuffer0, sbuftmp0, len, bq[0].coeffs, bq[0].w);
              dsps_biquad_f32_ae32(sbuftmp0, sbufout0, len, bq[1].coeffs, bq[1].w);

              // Process audio ch1 HIGH PASS FILTER
              dsps_biquad_f32_ae32(sbuffer0, sbuftmp0, len, bq[2].coeffs, bq[2].w);
              dsps_biquad_f32_ae32(sbuftmp0, sbufout1, len, bq[3].coeffs, bq[3].w);

              int16_t valint[2];

                for (uint16_t i=0; i<len; i++){ 

                  valint[0] = (muteCH[0] == 1) ? (int16_t) 0 : (int16_t) (sbufout0[i]*32768);
                  valint[1] = (muteCH[1] == 1) ? (int16_t) 0 : (int16_t) (sbufout1[i]*32768);
                  dsp_audio[i*4+0] = (valint[0] & 0xff);
                  dsp_audio[i*4+1] = ((valint[0] & 0xff00)>>8);
                  dsp_audio[i*4+2] = (valint[1] & 0xff);
                  dsp_audio[i*4+3] = ((valint[1] & 0xff00)>>8);

                }

              i2s_write_expand(0, (char*)dsp_audio, chunk_size,16,32, &bytes_written, portMAX_DELAY);

            }
            break;
          // Add filter test dsp_flow which bindes all the potential filters. bq[0-7]
          default :
            break;
        }
        vRingbufferReturnItem(s_ringbuf_i2s,(void *)audio);
    }
  }
}

void dsp_i2s_task_init(uint32_t sample_rate)
{ setup_dsp_i2s(sample_rate);
  s_ringbuf_i2s = xRingbufferCreate(8*1024,RINGBUF_TYPE_BYTEBUF);  
  if (s_ringbuf_i2s == NULL) { return; }
  xTaskCreate(dsp_i2s_task_handler, "DSP_I2S", 48*1024, NULL, 7, &s_dsp_i2s_task_handle);
}

void dsp_i2s_task_deninit(void)
{ if (s_dsp_i2s_task_handle) {
    vTaskDelete(s_dsp_i2s_task_handle);
    s_dsp_i2s_task_handle = NULL;
  }
  if (s_ringbuf_i2s) {
      vRingbufferDelete(s_ringbuf_i2s);
      s_ringbuf_i2s = NULL;
  }
}

size_t write_ringbuf(const uint8_t *data, size_t size)
{
   BaseType_t done = xRingbufferSend(s_ringbuf_i2s, (void *)data, size, (portTickType)portMAX_DELAY);
   return (done)?size:0;
}


// DSP processor
//======================================================
// Each time a buffer of audio is passed to the DSP - samples are
// processoed according to a list audio node processings.

// Each audio processor node consist of a data struct holding the
// required weights and states for processing a automomous filter
// function. The high level parameters is maintained in the structre
// as well.

// Release - Prove off concept
// ----------------------------------------
// Fixed 2x2 biquad flow high low seperation for each channel
// Interface for cross over and level
// Additional dynamic bass boost
//

void dsp_setup_flow(double freq) { // Should really be called "Setup x-filter.."

  float f = freq/samplerate;

  bq[0] = (ptype_t) { LPF, f, 0, 0.707, NULL, NULL, {0,0,0,0,0}, {0, 0} } ;
  bq[1] = (ptype_t) { LPF, f, 0, 0.707, NULL, NULL, {0,0,0,0,0}, {0, 0} } ;
  bq[2] = (ptype_t) { HPF, f, 0, 0.707, NULL, NULL, {0,0,0,0,0}, {0, 0} } ;
  bq[3] = (ptype_t) { HPF, f, 0, 0.707, NULL, NULL, {0,0,0,0,0}, {0, 0} } ;

  pnode_t * aflow = NULL;
  aflow = malloc(sizeof(pnode_t));
  if (aflow == NULL){ printf("Could not create node");
  
  for (uint8_t n=0; n<=5; n++){ 
    
    switch (bq[n].filtertype) {
      case LPF: dsps_biquad_gen_lpf_f32( bq[n].coeffs, bq[n].freq, bq[n].q );
              break;
      case HPF: dsps_biquad_gen_hpf_f32( bq[n].coeffs, bq[n].freq, bq[n].q );
              break;
      case LOWSHELF: dsps_biquad_gen_lowShelf_f32(bq[n].coeffs, bq[n].freq, bq[n].gain ,bq[n].q );
              break;
      case HIGHSHELF: dsps_biquad_gen_highShelf_f32(bq[n].coeffs, bq[n].freq, bq[n].gain, bq[n].q);
              break;

      default : break;
    }

      for (uint8_t i = 0;i <=4 ;i++ ){  
        printf("%.6f ",bq[n].coeffs[i]);
      }

    printf("\n");
    }
  }
}

void dsp_set_xoverfreq(uint8_t freqh, uint8_t freql) {
  float freq =  (freqh*256 + freql)/4;
  ESP_LOGI("X-filter","Freq %.0f",freq);
  float f = freq/samplerate;

  for ( int8_t n=0; n<=5; n++){ 
    bq[n].freq = f ;
    switch (bq[n].filtertype) {
      case LPF:
        dsps_biquad_gen_lpf_f32( bq[n].coeffs, bq[n].freq, bq[n].q );
        break;
      case HPF:
        dsps_biquad_gen_hpf_f32( bq[n].coeffs, bq[n].freq, bq[n].q );
        break;
      default : break;
    }
  }
}

void dsp_set_filter_freq(double freq, uint8_t filterType){

    // FilterType: 
    //    0: L-Shelf
    //    1: H-Shelf
    //    2: Peaking
    //    3: Notch
    //    .....

  int8_t nrOfbq = 7;        // How many biquads are registered in the pipeline?.. upto 7 currently.
  float f = freq/44100;     // Filter frequency normalized to samplerate.              

  switch(filterType){
      case 0:                                   

            for(int8_t n = 0; n <= nrOfbq; n++){
                  switch(bq[n].filtertype){

                      case LOWSHELF:
                          bq[n].freq = f;
                          dsps_biquad_gen_lowShelf_f32(bq[n].coeffs, bq[n].freq, bq[n].gain, bq[n].q);
                          break; 
                      default : break;
                  }
            }
            break;
      
      case 1:

            for(int8_t n = 0; n <= nrOfbq; n++){
                  switch(bq[n].filtertype){

                      case HIGHSHELF:
                          bq[n].freq = f;
                          dsps_biquad_gen_highShelf_f32(bq[n].coeffs, bq[n].freq, bq[n].gain, bq[n].q);
                          break; 
                      default : break;
                  }
            }
            break;
            
      case 2:

            for(int8_t n = 0; n <= nrOfbq; n++){
                  switch(bq[n].filtertype){

                      case PEAKINGEQ:
                          bq[n].freq = f;
                          dsps_biquad_gen_peakingEQ_f32(bq[n].coeffs, bq[n].freq, bq[n].q);
                          break; 
                      default : break;
                  }
            }
            break;
      
      case 3:

            for(int8_t n = 0; n <= nrOfbq; n++){
                  switch(bq[n].filtertype){

                      case NOTCH:
                          bq[n].freq = f;
                          dsps_biquad_gen_notch_f32(bq[n].coeffs, bq[n].freq, bq[n].gain, bq[n].q);
                          break; 
                      default : break;
                  }
            }
            break;
      
      default : break;
  }   
}

void dsp_set_filter_gain(double gain, uint8_t filterType){

    // FilterType: 
    //    0: L-Shelf
    //    1: H-Shelf
    //    2: Peaking
    //    3: Notch
    //    .....

  float g = gain;                   // Create float gain.

  switch(filterType){
      case 0:                                   

            for(int8_t n = 0; n <= 5; n++){

                  switch(bq[n].filtertype){

                      case LOWSHELF:
                          bq[n].gain = g;
                          dsps_biquad_gen_lowShelf_f32(bq[n].coeffs, bq[n].freq, bq[n].gain, bq[n].q);
                          break; 
                      default : break;
                  }
            }
            break;
      
      case 1:

            for(int8_t n = 0; n <= 5; n++){

                  switch(bq[n].filtertype){

                      case HIGHSHELF:
                          bq[n].gain = g;
                          dsps_biquad_gen_highShelf_f32(bq[n].coeffs, bq[n].freq, bq[n].gain, bq[n].q);
                          break; 
                      default : break;
                  }
            }          
            break;
      
      case 2:
            
            for(int8_t n = 0; n <= 5; n++){

                  switch(bq[n].filtertype){

                      case PEAKINGEQ:
                          bq[n].gain = g; // So.. apparently no gain adjustment for peaking filter? 
                          dsps_biquad_gen_peakingEQ_f32(bq[n].coeffs, bq[n].freq, bq[n].q);
                          break; 
                      default : break;
                  }
            }          
            break;  
      
      case 3:
            
            for(int8_t n = 0; n <= 5; n++){

                  switch(bq[n].filtertype){

                      case NOTCH:
                          bq[n].gain = g;
                          dsps_biquad_gen_notch_f32(bq[n].coeffs, bq[n].freq, bq[n].gain, bq[n].q);
                          break; 
                      default : break;
                  }
            }          
            break;              
      
      default : break;
  }   
}

void dsp_setup_filter(double freq, double gain, double q_filter, uint8_t filterType){

    // FilterType: 
    //    0: L-Shelf
    //    1: H-Shelf
    //    2: Peaking
    //    3: Notch
    //    .....

  float f = freq/44100;                   // Filter frequency 'normalized to sample rate'
  float g = gain/4; 
  float q = q_filter/64;

    switch(filterType){
      case 0:

            bq[0] = (ptype_t) { LOWSHELF, f, 0, 0.707, NULL, NULL, {0,0,0,0,0}, {0, 0} } ;  // Register Filter Biquads
            bq[1] = (ptype_t) { LOWSHELF, f, 0, 0.707, NULL, NULL, {0,0,0,0,0}, {0, 0} } ;

            dsps_biquad_gen_lowShelf_f32(bq[0].coeffs, bq[0].freq, bq[0].gain, bq[0].q);    // Generate biquad coefficients.
            dsps_biquad_gen_lowShelf_f32(bq[1].coeffs, bq[1].freq, bq[1].gain, bq[1].q);

            break;
      
      case 1:

            bq[2] = (ptype_t) { HIGHSHELF, f, 0, 0.707, NULL, NULL, {0,0,0,0,0}, {0, 0} } ; // Register Filter Biquads
            bq[3] = (ptype_t) { HIGHSHELF, f, 0, 0.707, NULL, NULL, {0,0,0,0,0}, {0, 0} } ;      
            
            dsps_biquad_gen_highShelf_f32(bq[2].coeffs, bq[2].freq, bq[2].gain, bq[2].q);    // Generate biquad coefficients.
            dsps_biquad_gen_highShelf_f32(bq[3].coeffs, bq[3].freq, bq[3].gain, bq[3].q);

            break;
      
      case 2:

            bq[4] = (ptype_t) { PEAKINGEQ, f, 0, 0.707, NULL, NULL, {0,0,0,0,0}, {0, 0} } ; // Register Filter Biquads
            bq[5] = (ptype_t) { PEAKINGEQ, f, 0, 0.707, NULL, NULL, {0,0,0,0,0}, {0, 0} } ;      
            
            dsps_biquad_gen_peakingEQ_f32(bq[4].coeffs, bq[4].freq, bq[4].q);    // Generate biquad coefficients.
            dsps_biquad_gen_peakingEQ_f32(bq[5].coeffs, bq[5].freq, bq[5].q);     

            break;  
      
      case 3:

            bq[6] = (ptype_t) { NOTCH, f, 0, 0.707, NULL, NULL, {0,0,0,0,0}, {0, 0} } ; // Register Filter Biquads
            bq[7] = (ptype_t) { NOTCH, f, 0, 0.707, NULL, NULL, {0,0,0,0,0}, {0, 0} } ;      
            
            dsps_biquad_gen_notch_f32(bq[6].coeffs, bq[6].freq, bq[6].gain, bq[6].q);    // Generate biquad coefficients.
            dsps_biquad_gen_notch_f32(bq[7].coeffs, bq[7].freq, bq[7].gain, bq[7].q);    

            break;              
      
      default : break;
  } 
}

void dsp_init_filter(uint8_t filterType, double freq){
    
    // FilterType: 
    //    0: L-Shelf
    //    1: H-Shelf
    //    2: Peaking
    //    3: Notch
    //    .....

    switch(filterType){
        case 0:
              dsp_setup_filter(freq, 0, 0.707, 0); // Initialize a l-shelf filter with 0 gain, frequency = param freq,  & q = 0.707
              break;
        case 1:
              dsp_setup_filter(freq, 0, 0.707, 1); // Initialize a h-shelf filter with 0 gain, frequency = param freq,  & q = 0.707
              break;              
        case 2:
              dsp_setup_filter(freq, 0, 0.707, 2); // Initialize a peaking filter with 0 gain, frequency = param freq,  & q = 0.707
              break;              
        case 3:
              dsp_setup_filter(freq, 0, 0.707, 3); // Initialize a notch filter with 0 gain, frequency = param freq,  & q = 0.707
              break;              
        default : break;
    }
}