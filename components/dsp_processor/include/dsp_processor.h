#ifndef _DSP_PROCESSOR_H_
#define _DSP_PROCESSOR_H_

enum dspFlows {dspfStereo, dspfBiamp, dspfDynBass};

size_t write_ringbuf(const uint8_t *data, size_t size);

void dsp_i2s_task_init(uint32_t sample_rate);

void dsp_i2s_task_deinit(void);

enum filtertypes { LPF, HPF, BPF, BPF0DB, NOTCH,
                   ALLPASS360, ALLPASS180, PEAKINGEQ,
                   LOWSHELF, HIGHSHELF};

// Process node
typedef struct ptype {
  int filtertype;
  float freq;
  float gain;
  float q;
  float *in,*out;
  float coeffs[5];
  float w[2];
} ptype_t;

// Process flow
typedef struct pnode {
    ptype_t process;
    struct pnode *next;
} pnode_t;

void dsp_setup_flow(double freq);
void dsp_setup_dynbass(double freq,double gain, double quality);
void dsp_set_xoverfreq(uint8_t, uint8_t); 
void dsp_set_gain_lshelf(uint8_t);
void dsp_set_gain_hshelf(uint8_t);
void dsp_set_dynbass(uint8_t, uint8_t, uint8_t, uint8_t);
void dsp_set_dynbassFreq(uint8_t, uint8_t);
void dsp_set_hshelfFreq(uint8_t, uint8_t);
void dsp_setup_hshelf(double freq,double gain, double q_filter);
void dsp_set_filter_gain(double gain, uint8_t filterType);
void dsp_setup_filter(double freq, double gain, double q_filter, uint8_t filterType);
void dsp_set_filter_freq(double freq, uint8_t filterType);
#endif /* _DSP_PROCESSOR_H_  */
