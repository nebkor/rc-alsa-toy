/* 
 * file : ne_alsa_capture.c
 * desc : Demo program for ALSA Capture or "Data AcQuisition (DAQ)"
 *        Performs realtime FFT on captured PCM signal  
 *
 * notes:  
 *       The ALSA capture code is based on 'aplay(1)' by Jaroslav Kysela.
 *       See 'aplay.c' in the 'alsa-utils' package.
 *
 *       Some FFT-related ideas taken from 'mbeq_1197.c' of the
 *       "https://github.com/swh/ladspa" package by Steve Harris.
 *
 *       For hints on performing raw FFT data calibration (for "ne_glprog.c"): 
 *       > "http://nairobi-embedded.org/alsa_daq_and_rt_fft_calib.html"
 *
 *       For a general discussion of implementation details, see:
 *       > "http://nairobi-embedded.org/alsa_daq_and_rt_fft.html"
 *
 *       For a general discussion of ALSA capture, check out:
 *       > "http://nairobi-embedded.org/alsa_audio_daq.html"
 *       > "http://nairobi-embedded.org/alsa_daq_and_tuning_capture_volume.html
 *       > "http://nairobi-embedded.org/alsa_pcm_basic_api.html"
 *
 * Build:
 *
 * 	"gcc -Wall -O2 ne_alsa_capture.c -lasound -lrfftw -lfftw -lrt [-ffast-math -funroll-loops]"
 *
 *  NOTE:: Only tested with FFTW2; untested with FFTW3 (although API included)
 *
 * Siro Mugabi, nairobi-embedded.org
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
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <error.h>
#include <unistd.h>
#include <limits.h>
#include <sys/time.h>		/* gettimeofday */
#include <string.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <getopt.h>
#include <alsa/asoundlib.h>
#include <math.h>
#include <time.h>
#include <sched.h>
#include <signal.h>
#include "ne_common.h"

/* =============== FFT related  data ================ */
#ifdef FFTW3
#include <fftw3.h>
typedef fftwf_plan fft_plan;
typedef float fftw_real;
#else
#include <rfftw.h>
typedef rfftw_plan fft_plan;
#endif /* FFTW3 */

fft_plan plan_rc;
fftw_real *cplx = NULL; /* frequency domain signal */
fftw_real *real = NULL; /* time domain signal */

static int *bin_band = NULL;
static float hz_per_bin = 0;
static double *window = NULL;
/**** SHM IPC w/ "ne_glprog.c" ****/
struct ne_glprog_fband_data ddata[NE_GLPROG_FBANDS];
static void *ne_glprog_fband_data_map;

/* ============ ALSA Related Globals =============== */
static char *device = "plughw:0,0";
static snd_pcm_t *handle;
static int verbose = 0;		/* snd_pcm_dump() */

/* hwparams and default settings */
#define HWPARAMS_FORMAT SND_PCM_FORMAT_S16_LE
#define HWPARAMS_CHANNELS 2
#define HWPARAMS_RATE 44100
#define HWPARAMS_PERIOD_FRAMES 1024

static struct {
	snd_pcm_format_t format;
	unsigned int channels;
	unsigned int rate;
	snd_pcm_uframes_t period_frames;
	snd_pcm_uframes_t buffer_frames;
} hwparams = {
.format = HWPARAMS_FORMAT,.channels = HWPARAMS_CHANNELS,.rate =
	    HWPARAMS_RATE,.period_frames = HWPARAMS_PERIOD_FRAMES};
static snd_pcm_stream_t stream = SND_PCM_STREAM_CAPTURE;

/* holds interleaved channel PCM period signal from H/W buffer */
static u_char *audiobuf = NULL;
/* holds deinterleaved channel PCM in separate & contiguous regions */ 
static float *chnldata = NULL;
/* raw capture PCM data for plotting program (e.g. "gnuplot(1)") IPC */
static char *raw_capture_data_file = NULL; /* shm file for raw dump */
static void *raw_capture_data_map = NULL; /* mmap ptr */

/* miscalleneous */
static int quiet_mode = 0;
#ifndef timersub
#define	timersub(a, b, result) \
do { \
	(result)->tv_sec = (a)->tv_sec - (b)->tv_sec; \
	(result)->tv_usec = (a)->tv_usec - (b)->tv_usec; \
	if ((result)->tv_usec < 0) { \
		--(result)->tv_sec; \
		(result)->tv_usec += 1000000; \
	} \
} while (0)
#endif

/* ============================================== *
 *                    RUNTIME                     *
 * ============================================== */

/* I/O error handler */
static void xrun(void)
{
	snd_pcm_status_t *status;
	int res;

	snd_pcm_status_alloca(&status);
	if ((res = snd_pcm_status(handle, status)) < 0) {
		prerr("status error: %s", snd_strerror(res));
		exit(EXIT_FAILURE);
	}
	if (snd_pcm_status_get_state(status) == SND_PCM_STATE_XRUN) {
		struct timeval now, diff, tstamp;
		gettimeofday(&now, 0);
		snd_pcm_status_get_trigger_tstamp(status, &tstamp);
		timersub(&now, &tstamp, &diff);
		prwarn("%s!!! (at least %.3f ms long)\n",
		      stream ==
		      SND_PCM_STREAM_PLAYBACK ? "underrun" : "overrun",
		      diff.tv_sec * 1000 + diff.tv_usec / 1000.0);
		if ((res = snd_pcm_prepare(handle)) < 0) {
			prerr("xrun: prepare error: %s", snd_strerror(res));
			exit(EXIT_FAILURE);
		}
		return;		/* ok, data should be accepted again */
	}
	if (snd_pcm_status_get_state(status) == SND_PCM_STATE_DRAINING) {
		if (stream == SND_PCM_STREAM_CAPTURE) {
			prwarn
			    ("capture stream format change? attempting recover...\n");
			if ((res = snd_pcm_prepare(handle)) < 0) {
				prerr("xrun(DRAINING): prepare error: %s",
				      snd_strerror(res));
				exit(EXIT_FAILURE);
			}
			return;
		}
	}
	prerr("read/write error, state = %s",
	      snd_pcm_state_name(snd_pcm_status_get_state(status)));
	exit(EXIT_FAILURE);
}

/* I/O suspend handler */
static void suspend(void)
{
	int res;

	if (!quiet_mode)
		prwarn("Suspended. Trying resume. ");
	fflush(stderr);
	while ((res = snd_pcm_resume(handle)) == -EAGAIN)
		sleep(1);	/* wait until suspend flag is released */
	if (res < 0) {
		if (!quiet_mode)
			prwarn("Failed. Restarting stream. ");
		fflush(stderr);
		if ((res = snd_pcm_prepare(handle)) < 0) {
			prerr("suspend: prepare error: %s", snd_strerror(res));
			exit(EXIT_FAILURE);
		}
	}
	if (!quiet_mode)
		prinfo("Done.\n");
}

/* *** Acquire ALSA PCM period signal from H/W *** */
static inline ssize_t pcm_read(u_char * data, size_t rcount)
{
	ssize_t r;
	size_t result = 0, count = rcount;
	uint32_t channels = hwparams.channels;
	snd_pcm_uframes_t period_size = hwparams.period_frames;
	snd_pcm_format_t format = hwparams.format;
	uint32_t fmt_phys_width_bits = snd_pcm_format_physical_width(format);
	uint32_t fmt_phys_width_bytes = fmt_phys_width_bits / 8;
	uint32_t fmt_phys_width_bytes_per_frame =
	    fmt_phys_width_bytes * channels;

	assert(count == period_size);

	while (count > 0) {
		r = snd_pcm_readi(handle, data, count);
		if (r == -EAGAIN || (r >= 0 && (size_t) r < count)) {
			snd_pcm_wait(handle, 1000);
		} else if (r == -EPIPE) {
			xrun();
		} else if (r == -ESTRPIPE) {
			suspend();
		} else if (r < 0) {
			prerr("read error: %s", snd_strerror(r));
			exit(EXIT_FAILURE);
		}
		if (r > 0) {
			result += r;
			count -= r;
			data += r * fmt_phys_width_bytes_per_frame;
		}
	}
	return result;
}

/* Deinterleave ALSA frames in PCM period buffer into seperate
	 per-channel buffer regions */
static inline void deinterleave(void)
{
	int i, j, k, chnls = hwparams.channels;
	float *dst = chnldata;
	uint8_t *src = audiobuf, *ptr;
	int32_t psize = hwparams.period_frames;
	snd_pcm_format_t format = hwparams.format;
	int fmt_nominal_width_bits = snd_pcm_format_width(format);
	int fmt_nominal_width_bytes = fmt_nominal_width_bits / 8;
	int fmt_phys_width_bits = snd_pcm_format_physical_width(format);
	int fmt_phys_width_bytes = fmt_phys_width_bits / 8;
	union {
		int32_t i;
		uint32_t u;
	} resln;

	for (i = 0; i < psize; i++) {
		ptr = src + (i * fmt_phys_width_bytes * chnls);
		for (j = 0; j < chnls; j++) {

			/* to support variety of sample formats, perform byte-by-byte
			   extraction for each sample word */
			ptr += j * fmt_phys_width_bytes;
			for (resln.u &= 0x0, k = 0; k < fmt_phys_width_bytes;
			     k++) {
				/* handle endianess of current sample format */
				if (snd_pcm_format_big_endian(format))
					resln.u |=
					    ptr[fmt_phys_width_bytes - 1 -
						k] << k * 8;
				else
					resln.u |= ptr[k] << k * 8;
			}

			/* extend sign of two's complement to fit local storage */
			if (resln.u >= (1U << (fmt_nominal_width_bits - 1))) {
				for (k = fmt_nominal_width_bytes;
				     k < (int)sizeof(resln.u); k++)
					resln.u |= 0xff << k * 8;
			}

			dst[i + (psize * j)] = resln.i;

			/* only dumping channel 0 raw pcm in shm for plotting program */
			if (j == 0 && raw_capture_data_map != NULL)
				((int32_t *) raw_capture_data_map)[i] = resln.i;

		}		/* for(j) */
	}			/* for(i) */
}

/* Top-level capture function: acquire a PCM period from H/W */
static inline void do_capture(void)
{
	size_t ret;
	int channels = hwparams.channels;
	snd_pcm_uframes_t period_size = hwparams.period_frames;

	/* read in an ALSA period from hardware buffer */
	ret = pcm_read(audiobuf, period_size);
	if (ret != period_size)
		prwarn("WARNING: copied %zi instead of %zi\n", ret, period_size);

	/* extract interleaved per-channel data */
	memset(chnldata, 0, (period_size * sizeof(float) * channels));
	deinterleave();
}

/* *** obtain frequency band mangitude *** */
static inline float freq_band_magn(int offset, int count)
{
	int i;
	int length = hwparams.period_frames;

#if 0
	/* return the averge contribution */
	float re, im, total = 0.0f;
	for (i = 0; i < count; i++) {
		re = cplx[i + offset];
		im = cplx[length - offset - i];
		total += sqrt(re * re + im * im);
	}
	return total / count;
#else
	/* return the tallest peak */
	fftw_real re, im, tmp, val = 0.0f;
	for (i = 0; i < count; i++) {
		re = cplx[i + offset];
		im = cplx[length - offset - i];
		tmp = sqrt(re * re + im * im);
		val = tmp > val ? tmp : val;
	}
	return (float)val;
#endif

}

/* func : do_fft()
 * desc : performs fft processing on a given channel 
 * notes: for simultaneous fft processing on stereo signals, see (for example)
 *        "http://nairobi-embedded.org/ne_fft_notes.html"
 */
static inline void do_fft(int channel)
{
	int i, bin, count, offset, n_points = hwparams.period_frames;
	float magn, tmp = 0.0f;
	static float prevtmp[NE_GLPROG_FBANDS];
	void *map = ne_glprog_fband_data_map;

	/* initialize fftw input buffer */
	offset = channel * n_points;
	for (i = 0; i < n_points; i++)
		real[i] = (double)chnldata[offset + i] * window[i];

	/* fftw real->complex transform */
#ifdef FFTW3
	fftwf_execute(plan_rc);
#else
	rfftw_one(plan_rc, real, cplx);
#endif

	/* FFT bins (fftw output) to display's freq-band bars */
	bin = 1;
	memset(ddata, 0, sizeof(ddata));
	for (i = 0; i < NE_GLPROG_FBANDS; i++) {

		count = 0;
		offset = bin;
		while (bin < (n_points / 2) && bin_band[bin] <= i) {
			count++;
			bin++;
		}

		if (count) {

			/* obtain raw freq band bar magnitude */
			magn = freq_band_magn(offset, count);

			/* calibration is maddening */
			tmp = magn > 0.0f ? logf(magn) * 16.7f : 0.0f;
      tmp = tmp > 172.0f ? (tmp - 172.0f) * 3.2f : 0.0f;
			/* clip excessive levels */
			tmp = tmp < 250.0f ? tmp : 250.0f;

			/* control rate of decay */
			/* first establish special case */
			prevtmp[i] = prevtmp[i] > 2.0f ? 
					prevtmp[i] - (2.0f * logf(prevtmp[i])) : 0.0f;
			/* then handle general case */
			if(tmp > prevtmp[i])
				prevtmp[i] = tmp;
			else
				tmp = prevtmp[i];

			ddata[i].fband_magn = tmp;
			prdbg
			    ("FREQ_BAND: %d, bin_count: %d, display_fband_magn: %.2f, raw_fband_magn: %.2f, logf(raw_fband_magn): %.2f\n",
			     i, count, ddata[i].fband_magn, magn, logf(magn));
		}
	}

	/* copy display data to posix shm */
	memcpy(map, ddata, sizeof(ddata));
}

/* ====================================================== *
 *                    INITIALIZATION                      *
 * ====================================================== */

static int fft_init(void)
{
	int i, bin, n_points = hwparams.period_frames;
	float base_freq_ratio;

	/* fftw initialization */
	real = calloc(n_points, sizeof(fftw_real));
	cplx = calloc(n_points, sizeof(fftw_real));
	bin_band = calloc(n_points, sizeof(int));
	window = calloc(n_points, sizeof(double));
	if (!real || !cplx || !bin_band || !window) {
		prerr("calloc(3) failed!\n");
		return -1;
	}
#ifdef FFTW3
	plan_rc =
	    fftwf_plan_r2r_1d(n_points, real, cplx, FFTW_R2HC, FFTW_MEASURE);
#else
	plan_rc =
	    rfftw_create_plan(n_points, FFTW_REAL_TO_COMPLEX, FFTW_ESTIMATE);
#endif

	for (i = 0; i < n_points; i++)
		window[i] = 1.0f;	/* place-holder */

	/* prepare for grouping of fft bins into display freq bars */
	hz_per_bin = (float)hwparams.rate / (float)n_points;
	bin = 1;
	while (bin <= ne_glprog_fband[0] / hz_per_bin)
		bin_band[bin++] = 0;

	for (i = 1;
	     i < NE_GLPROG_FBANDS - 1 && bin < (n_points / 2) - 1
	     && ne_glprog_fband[i + 1] < hwparams.rate / 2; i++) {
		base_freq_ratio = (ne_glprog_fband[i + 1]) / hz_per_bin;
		while (bin <= base_freq_ratio)
			bin_band[bin++] = i;
	}

	for (; bin < (n_points / 2); bin++)
		bin_band[bin] = NE_GLPROG_FBANDS - 1;

	return 0;
}

static void *shm_init(const char *const filename, size_t filesize)
{
	int fd, ret = -1;
	void *map = NULL;

	fd = shm_open(filename, O_RDWR | O_CREAT, (mode_t) 0666);
	if (fd < 0) {
		prerr("%s\n", strerror(errno));
		goto exit;
	}

	ret = ftruncate(fd, filesize);
	if (ret < 0) {
		prerr("%s\n", strerror(errno));
		goto exit;
	}

	map = mmap(0, filesize, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
	if (map == MAP_FAILED) {
		prerr("%s\n", strerror(errno));
		goto exit;
	}

	close(fd);
	return map;
exit:
	return NULL;
}

static int set_hwparams(void)
{
	ssize_t err = -1;
	unsigned int channels = hwparams.channels;
	unsigned int rrate, rate = hwparams.rate;
	snd_pcm_format_t format = hwparams.format;
	snd_pcm_uframes_t *period_size = &hwparams.period_frames;
	snd_pcm_uframes_t buffer_size;

	snd_pcm_hw_params_t *params;
	snd_pcm_hw_params_alloca(&params);

	err = snd_pcm_hw_params_any(handle, params);
	if (err < 0) {
		prerr("%s\n", snd_strerror(err));
		goto exit;
	}

	err = snd_pcm_hw_params_set_access(handle, params,
					   SND_PCM_ACCESS_RW_INTERLEAVED);
	if (err < 0) {
		prerr("%s\n", snd_strerror(err));
		goto exit;
	}

	err = snd_pcm_hw_params_set_format(handle, params, format);
	if (err < 0) {
		prerr("%s\n", snd_strerror(err));
		goto exit;
	}
	err = snd_pcm_hw_params_set_channels(handle, params, channels);
	if (err < 0) {
		prerr("%s\n", snd_strerror(err));
		goto exit;
	}

	rrate = rate;
	err = snd_pcm_hw_params_set_rate_near(handle, params, &rrate, 0);
	if (err < 0) {
		prerr("Rate %iHz not available for playback: %s\n", rate,
		      snd_strerror(err));
		goto exit;
	}
	if (rrate != rate) {
		prerr("Rate doesn't match (requested %iHz, got %iHz)\n", rate,
		      rrate);
		err = -EINVAL;
		goto exit;
	}

	err = snd_pcm_hw_params_set_period_size_near(handle, params,
						     period_size, 0);
	if (err < 0) {
		prerr("%s\n", snd_strerror(err));
		goto exit;
	}
#if 0
	/* we need not concern ourselves with calculating/specifying
	 * a usable h/w ring buffer size since ALSA runtime
	 * is quite capable of determining a suitable value for us.
	 *
	 * Otherwise, do something like...
	 */
	err = snd_pcm_hw_params_set_buffer_size_near(handle, params,
						     &buffer_frames);
	if (err < 0) {
		prerr("%s\n", snd_strerror(err));
		goto exit;
	}
#endif

	err = snd_pcm_hw_params(handle, params);
	if (err < 0) {
		prerr("%s\n", snd_strerror(err));
		goto exit;
	}

	snd_pcm_hw_params_get_buffer_size(params, &buffer_size);
	if (*period_size == buffer_size) {
		prerr("Can't use period equal to buffer size (%lu == %lu)",
		      *(unsigned long *)period_size,
		      (unsigned long)buffer_size);
		err = -1;
		goto exit;
	}

	if (!verbose)
		printf("\n" "Accepted HWPARAMS:\n%*iHz (%s)"
		       "\n%*s (%s)"
		       "\n%*i (%s)"
		       "\n%*lu (%s)"
		       "\n%*lu (%s)"
		       "\n",
		       28, rate, "sampling rate",
		       30, snd_pcm_format_name(format), "sample format",
		       30, channels, "number of channels",
		       30, buffer_size, "h/w ring buffer size in frames",
		       30, *period_size, "period size in frames");

	err = 0;
exit:
	return err;
}

static ssize_t alloc_period_pcm_buf(void)
{
	ssize_t err = -1;
	snd_pcm_uframes_t period_size = hwparams.period_frames;
	uint32_t channels = hwparams.channels;
	snd_pcm_format_t format = hwparams.format;
	unsigned long fmt_phys_width_bits =
	    snd_pcm_format_physical_width(format);
	unsigned long fmt_phys_width_bits_per_frame =
	    fmt_phys_width_bits * channels;
	unsigned long chunk_bytes =
	    period_size * fmt_phys_width_bits_per_frame / 8;

	audiobuf = calloc(chunk_bytes, sizeof(u_char));
	if (audiobuf == NULL) {
		prerr("Insufficient memory");
		goto exit;
	}

	if (!verbose)
		printf("\n" "PCM Data Transfer Stats:"
		       "\n%*lu bits/sample, %lu bits/frame"
		       "\n%*lu period size in bytes (pcm data transfer size)"
		       "\n", 30, fmt_phys_width_bits,
		       fmt_phys_width_bits_per_frame, 30, chunk_bytes);

	err = 0;
exit:
	return err;
}

static ssize_t alloc_chnldata_buf(void)
{
	ssize_t err = -1;
	int channels = hwparams.channels;
	unsigned int period_size = hwparams.period_frames;

	chnldata = calloc(period_size * channels, sizeof(float));
	if (!chnldata) {
		prerr("calloc(3) failed!\n");
		goto exit;
	}

	err = 0;
exit:
	return err;
}

static const char *prog;
static void usage( )
{
	fprintf(stderr, "\n" "Usage: %s [OPTIONS]\n"
			   "\n\n"
				 "OPTIONS:\n"
	       "-h,--help         This menu\n"
	       "-D,--device       Virtual PCM device, e.g. \"plguhw:0,0\", \"default\", etc\n"
	       "-r,--rate         Sample rate in Hz, e.g. 44100\n"
	       "-c,--channels     Channel count, e.g. 2 for stereo\n"
	       "-b,--buffer-size  H/W Ring buffer size in frames (not used)\n"
	       "-p,--period-size  Period size in frames, e.g. 1024\n"
	       "-o,--format       Sample format, e.g. \"S16_LE\", \"U32_BE\", etc\n"
				 "-f,--dumpfile     Raw capture data dump file (posix shm)\n"
	       "-v,--verbose      Display PCM S/W conversions\n" "\n", prog);

	fprintf(stderr, "Recognized sample formats are: "
	"S16_LE S16_BE S24_LE S24_BE S32_LE S32_BE");
	fprintf(stderr, "\n\n");
	exit(EXIT_SUCCESS);
}

static void bad_option(const char *option)
{
	fprintf(stderr, "\n\tBad option \"%s\"\n", option);
	usage();
}

static int set_prio(int prio)
{
	struct sched_param param;
	param.sched_priority = prio;
	if (sched_setscheduler(0, SCHED_FIFO, &param)) {
		prwarn("%s\n", strerror(errno));
		return -1;
	}
	return 0;
}

#define MAX_SAFE_STACK (8 * 1024)
static void stack_prefault(void)
{
	unsigned char dummy[MAX_SAFE_STACK];
	memset(&dummy, 0, MAX_SAFE_STACK);
	return;
}

static int go_rt(void)
{
	int err = -1;

	/* SCHED_FIFO: 1 for min prio; 99 for max prio */
#define SCHED_FIFO_PRIO_VAL 40
	if (set_prio(SCHED_FIFO_PRIO_VAL))
		goto exit;

	/* prevent being paged out during rt execution */
	if (mlockall(MCL_CURRENT | MCL_FUTURE) < 0) {
		prwarn("%s\n", strerror(errno));
		goto exit;
	}

	/* prefault program stack */
	stack_prefault();

	err = 0;
exit:
	return err;
}

static void do_getopt_long(int argc, char *const *argv)
{
	snd_pcm_format_t format;
	struct option long_option[] = {
		{"help", 0, NULL, 'h'},
		{"device", 1, NULL, 'D'},
		{"rate", 1, NULL, 'r'},
		{"channels", 1, NULL, 'c'},
		{"buffer-size", 1, NULL, 'b'},
		{"period-size", 1, NULL, 'p'},
		{"format", 1, NULL, 'o'},
		{"verbose", 0, NULL, 'v'},
		{"dumpfile", 1, NULL, 'f'},
		{NULL, 0, NULL, 0},
	};
	char *eptr;

	for (;;) {
		int c;
		if ((c =
		     getopt_long(argc, argv, "hD:r:c:b:p:o:f:v",
				 long_option, NULL)) < 0)
			break;
		switch (c) {
		case 'D':
			if (!(device = strdup(optarg))) {
				prerr("strdup(3)\n");
				exit(EXIT_FAILURE);
			}
			break;
		case 'r':
			hwparams.rate = strtoul(optarg, &eptr, 0);
			if (*eptr != '\0')
				bad_option("Sample Rate");
			break;
		case 'c':
			hwparams.channels = strtoul(optarg, &eptr, 0);
			if (*eptr != '\0')
				bad_option("Channel Count");
			break;
		case 'b':
			hwparams.buffer_frames = strtoul(optarg, &eptr, 0);
			if (*eptr != '\0')
				bad_option("H/W Buffer Size");
			break;
		case 'p':
			hwparams.period_frames = strtoul(optarg, &eptr, 0);
			if (*eptr != '\0')
				bad_option("Period Size");
			break;
		case 'v':
			verbose = 1;
			break;
		case 'f':
			if (!(raw_capture_data_file = strdup(optarg))) {
				prerr("strdup(3)\n");
				exit(EXIT_FAILURE);
			}
			break;
		case 'o':
			for (format = 0; format < SND_PCM_FORMAT_LAST; format++) {
				const char *format_name =
				    snd_pcm_format_name(format);

				if (format_name)
					if (!strcasecmp(format_name, optarg))
						goto get_format;
			}
			usage( );

get_format:
			if (format == SND_PCM_FORMAT_LAST)
				format = SND_PCM_FORMAT_S16;

			if(snd_pcm_format_unsigned(format)){
				prerr("Unsigned formats not supported\n");
				exit(EXIT_FAILURE);
			}

			if(format == SND_PCM_FORMAT_FLOAT_LE ||
			   format == SND_PCM_FORMAT_FLOAT_BE){
				prerr("FLOAT formats unsupported %s\n", optarg);
				exit(EXIT_FAILURE);
			}

			if (!snd_pcm_format_linear(format)){
				prerr("Invalid non-linear format %s\n",
				       optarg);
				exit(EXIT_FAILURE);
			}
			hwparams.format = format;
			break;

		case 'h':
		default:
			usage( );
		}
	}
}

/* dump PCM Plugin chain of software conversions to stdout */
static int do_snd_pcm_dump(void)
{
	snd_output_t *output = NULL;
	int err = -1;

	err = snd_output_stdio_attach(&output, stdout, 0);
	if (err < 0) {
		prerr("%s\n", snd_strerror(err));
		goto exit;
	}

	snd_pcm_dump(handle, output);
	printf("\n");

	err = 0;
exit:
	return err;
}

#define __print_once_snd_pcm_state( ) \
	do { \
		static int first = 1; \
		if (first) do_snd_pcm_state( ); \
		first = 0; \
	} while (0)

static void do_snd_pcm_state(void)
{
	if (!verbose)
		printf("%*s: %u\n", 30, "PCM Stream State",
		       snd_pcm_state(handle));
}

static int done = 0;
static void sighandler(int sig)
{
	/* printf async-unsafe even with sigaction? */
	fprintf(stderr, "%s:%d:: Received signal %d\n", __func__, __LINE__, sig);
	done = 1;
}

int main(int argc, char *argv[])
{
	int err = -1, filesize;
	struct sigaction sa;
	unsigned int channels;
	snd_pcm_uframes_t period_size;
	snd_pcm_format_t format;

	if (!(prog = strdup(argv[0]))) {
		prerr("strdup(3)\n");
		exit(EXIT_FAILURE);
	}
	do_getopt_long(argc, argv);
	format = hwparams.format;
	channels = hwparams.channels;
	period_size = hwparams.period_frames;

	printf("Capture device is: \"%s\"\n", device);

	/* open device */
	if ((err = snd_pcm_open(&handle, device, stream, 0)) < 0) {
		prerr("pcm open error (%s)\n", snd_strerror(err));
		goto exit;
	}

	do_snd_pcm_state();

	/* setup hwparams */
	if (set_hwparams())
		goto exit;

	do_snd_pcm_state();

	/* alloc buffer to hold PCM period data */
	if (alloc_period_pcm_buf())
		goto exit;

	/* alloc buffer to hold (deinterleaved) per-channel PCM data */
	if (alloc_chnldata_buf())
		goto exit;

	/* shm ipc for "ne_glprog" */
	filesize = sysconf(_SC_PAGE_SIZE);
	ne_glprog_fband_data_map = 
		shm_init(NE_GLPROG_FBAND_DATA_FILE, filesize);
	if(!ne_glprog_fband_data_map)
		goto exit;

	/* shm ipc for a plotting program (e.g. "gnuplot(1)") */
	if(raw_capture_data_file){
		if((snd_pcm_format_physical_width(format) / 8) > (int)sizeof(int32_t)){
			prerr("maximum supported sample format width for plotting is 32bits\n");
			goto exit;
		}
	
		filesize = period_size * channels * sizeof(int32_t);
		raw_capture_data_map = 
			shm_init(raw_capture_data_file, filesize);
		if (!raw_capture_data_map)
			goto exit;
	}

	/* initialize fft engine */
	if (fft_init())
		goto exit;

	if (verbose > 0)
		if (do_snd_pcm_dump())
			goto exit;

#if 0
	/* to perform function tracing with ftrace e.g. in order
	 * to observe CPU affinity - you may include this function
	 * call.
	 * See "http://nairobi-embedded.org/ne_ftrace_rt_proc_affinity.html" 
	 * for an example implementation.
	 */
	if (ftrace_startup())
		goto exit;
#endif

	/* basic signal handling */
	sigemptyset(&sa.sa_mask);
	sa.sa_flags = 0;
	sa.sa_handler = sighandler;
	if (sigaction(SIGINT, &sa, NULL) == -1)
		prwarn("sigaction: %s\n", strerror(errno));

	/* going firm realtime */
	printf("\n");
	if (go_rt())
		prwarn("WARNING: failed to go firm realtime!\n");

	/* perform pcm capture and fft processing of audio stream */
	while (!done) {

		do_capture();
		__print_once_snd_pcm_state( );
		do_fft(0);
	}

	err = 0;
exit:

#if 0
	/* See "http://nairobi-embedded.org/ne_ftrace_rt_proc_affinity.html" 
	 * for an example implementation of the following function.
	 */
	ftrace_cleanup();
#endif

	/* for graceful termination */
	snd_pcm_close(handle);

	if (window)
		free(window);

	if (bin_band)
		free(bin_band);

	if (cplx)
		free(cplx);

	if (real)
		free(real);

	if (chnldata)
		free(chnldata);

	if (audiobuf)
		free(audiobuf);

	return err;
}
