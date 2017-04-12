/*
 * file:  ne_common.h
 * desc:  common definitions and data structures for
 *        `ne_alsa_capture.c`, `ne_spectrum_ladspa.c` and `ne_glprog.c`
 *     See:
 *     > "http://nairobi-embedded.org/alsa_daq_and_rt_fft.html"
 *     > "http://nairobi-embedded.org/alsa_ladspa_demo_dsp_prototyping.html"
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */


#ifndef __NE_COMMON_H__
#define __NE_COMMON_H__

#define prfmt(fmt) "%s:%d:: " fmt, __func__, __LINE__
#define prinfo(fmt, ...) printf(prfmt(fmt), ##__VA_ARGS__)
#define prerr(fmt, ...) fprintf(stderr, prfmt(fmt), ##__VA_ARGS__)
#define prwarn(fmt, ...) fprintf(stderr, prfmt(fmt), ##__VA_ARGS__)
#ifdef DEBUG
#define prdbg(fmt, ...) printf(prfmt(fmt), ##__VA_ARGS__)
#else
#define prdbg(fmt, ...) do{}while(0)
#endif

/* Frequency-band data in POSIX SHM */
#define NE_GLPROG_FBAND_DATA_FILE "ne_glprog_fband_data_file"
struct ne_glprog_fband_data{
	float fband_magn;
};

/* 
 * `ne_glprog.c` is designed to display the audio spectrum of an audio 
 *  stream by either:
 * 
 *	o `ne_alsa_capture.c` (for direct ALSA Capture)
 *  o `ne_spectrum_ladspa.c` (for ALSA/LADSPA - based on "mbeq_1197")
 *
 * For this reason, the values of the frequency bands below
 * were obtained from the `mbeq_1197.c` file of the
 * "https://github.com/swh/ladspa" package by Steve Harris.
 */
#define NE_GLPROG_FBANDS 15
float ne_glprog_fband[NE_GLPROG_FBANDS] =
  { 50.00f, 100.00f, 155.56f, 220.00f, 311.13f,
    440.00f, 622.25f, 880.00f, 1244.51f, 1760.00f, 2489.02f,
    3519.95, 4978.04f, 9956.08f, 19912.16f };

#endif /* __NE_COMMON_H__ */
