/*
 * Various utilities for command line tools
 * Copyright (c) 2000-2003 Fabrice Bellard
 *
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with FFmpeg; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <errno.h>
#include <math.h>

/* Include only the enabled headers since some compilers (namely, Sun
   Studio) will not omit unused inline functions and create undefined
   references to libraries that are not being built. */

#include "config.h"
#include "compat/va_copy.h"
#include "libavformat/avformat.h"
#include "libavfilter/avfilter.h"
#include "libavdevice/avdevice.h"
#include "libavresample/avresample.h"
#include "libswscale/swscale.h"
#include "libswresample/swresample.h"
#include "libpostproc/postprocess.h"
#include "libavutil/avassert.h"
#include "libavutil/avstring.h"
#include "libavutil/bprint.h"
#include "libavutil/display.h"
#include "libavutil/mathematics.h"
#include "libavutil/imgutils.h"
//#include "libavutil/libm.h"
#include "libavutil/parseutils.h"
#include "libavutil/pixdesc.h"
#include "libavutil/eval.h"
#include "libavutil/dict.h"
#include "libavutil/opt.h"
#include "libavutil/cpu.h"
#include "libavutil/ffversion.h"
#include "libavutil/version.h"
#include "cmdutils.h"
#include <sys/time.h>
#include <sys/resource.h>

AVDictionary *sws_dict;
AVDictionary *swr_opts;
AVDictionary *format_opts, *codec_opts, *resample_opts;

void init_opts(void)
{
	av_dict_set(&sws_dict, "flags", "bicubic", 0);
}

void uninit_opts(void)
{
	av_dict_free(&swr_opts);
	av_dict_free(&sws_dict);
	av_dict_free(&format_opts);
	av_dict_free(&codec_opts);
	av_dict_free(&resample_opts);
}

static void (*program_exit)(int ret);

void register_exit(void (*cb)(int ret))
{
	program_exit = cb;
}

void exit_program(int ret)
{
	if (program_exit)
		program_exit(ret);

	exit(ret);
}

double parse_number_or_die(const char *context, const char *numstr, int type,
                           double min, double max)
{
	char *tail;
	const char *error;
	double d = av_strtod(numstr, &tail);
	if (*tail)
		error = "Expected number for %s but found: %s\n";
	else if (d < min || d > max)
		error = "The value for %s was %s which is not within %f - %f\n";
	else if (type == OPT_INT64 && (int64_t)d != d)
		error = "Expected int64 for %s but found %s\n";
	else if (type == OPT_INT && (int)d != d)
		error = "Expected int for %s but found %s\n";
	else
		return d;
	av_log(NULL, AV_LOG_FATAL, error, context, numstr, min, max);
	exit_program(1);
	return 0;
}

int64_t parse_time_or_die(const char *context, const char *timestr,
                          int is_duration)
{
	int64_t us;
	if (av_parse_time(&us, timestr, is_duration) < 0) {
		av_log(NULL, AV_LOG_FATAL, "Invalid %s specification for %s: %s\n",
		       is_duration ? "duration" : "date", context, timestr);
		exit_program(1);
	}
	return us;
}

static inline void prepare_app_arguments(int *argc_ptr, char ***argv_ptr)
{
	/* nothing to do */
}

void print_error(const char *filename, int err)
{
	char errbuf[128];
	const char *errbuf_ptr = errbuf;

	if (av_strerror(err, errbuf, sizeof(errbuf)) < 0)
		errbuf_ptr = strerror(AVUNERROR(err));
	av_log(NULL, AV_LOG_ERROR, "%s: %s\n", filename, errbuf_ptr);
}

static int warned_cfg = 0;

#define INDENT        1
#define SHOW_VERSION  2
#define SHOW_CONFIG   4
#define SHOW_COPYRIGHT 8

int check_stream_specifier(AVFormatContext *s, AVStream *st, const char *spec)
{
	int ret = avformat_match_stream_specifier(s, st, spec);
	if (ret < 0)
		av_log(s, AV_LOG_ERROR, "Invalid stream specifier: %s.\n", spec);
	return ret;
}

AVDictionary *filter_codec_opts(AVDictionary *opts, enum AVCodecID codec_id,
                                AVFormatContext *s, AVStream *st, AVCodec *codec)
{
	AVDictionary    *ret = NULL;
	AVDictionaryEntry *t = NULL;
	int            flags = s->oformat ? AV_OPT_FLAG_ENCODING_PARAM
	                       : AV_OPT_FLAG_DECODING_PARAM;
	char          prefix = 0;
	const AVClass    *cc = avcodec_get_class();

	if (!codec)
		codec            = s->oformat ? avcodec_find_encoder(codec_id)
		                   : avcodec_find_decoder(codec_id);

	switch (st->codecpar->codec_type) {
	case AVMEDIA_TYPE_VIDEO:
		prefix  = 'v';
		flags  |= AV_OPT_FLAG_VIDEO_PARAM;
		break;
	case AVMEDIA_TYPE_AUDIO:
		prefix  = 'a';
		flags  |= AV_OPT_FLAG_AUDIO_PARAM;
		break;
	case AVMEDIA_TYPE_SUBTITLE:
		prefix  = 's';
		flags  |= AV_OPT_FLAG_SUBTITLE_PARAM;
		break;
	}

	while (t = av_dict_get(opts, "", t, AV_DICT_IGNORE_SUFFIX)) {
		char *p = strchr(t->key, ':');

		/* check stream specification in opt name */
		if (p)
			switch (check_stream_specifier(s, st, p + 1)) {
			case  1: *p = 0; break;
			case  0:         continue;
			default:         exit_program(1);
			}

		if (av_opt_find(&cc, t->key, NULL, flags, AV_OPT_SEARCH_FAKE_OBJ) ||
		    !codec ||
		    (codec->priv_class &&
		     av_opt_find(&codec->priv_class, t->key, NULL, flags,
		                 AV_OPT_SEARCH_FAKE_OBJ)))
			av_dict_set(&ret, t->key, t->value, 0);
		else if (t->key[0] == prefix &&
		         av_opt_find(&cc, t->key + 1, NULL, flags,
		                     AV_OPT_SEARCH_FAKE_OBJ))
			av_dict_set(&ret, t->key + 1, t->value, 0);

		if (p)
			*p = ':';
	}
	return ret;
}

AVDictionary **setup_find_stream_info_opts(AVFormatContext *s,
        AVDictionary *codec_opts)
{
	int i;
	AVDictionary **opts;

	if (!s->nb_streams)
		return NULL;
	opts = av_mallocz_array(s->nb_streams, sizeof(*opts));
	if (!opts) {
		av_log(NULL, AV_LOG_ERROR,
		       "Could not alloc memory for stream options.\n");
		return NULL;
	}
	for (i = 0; i < s->nb_streams; i++)
		opts[i] = filter_codec_opts(codec_opts, s->streams[i]->codecpar->codec_id,
		                            s, s->streams[i], NULL);
	return opts;
}

void *grow_array(void *array, int elem_size, int *size, int new_size)
{
	if (new_size >= INT_MAX / elem_size) {
		av_log(NULL, AV_LOG_ERROR, "Array too big.\n");
		exit_program(1);
	}
	if (*size < new_size) {
		uint8_t *tmp = av_realloc_array(array, new_size, elem_size);
		if (!tmp) {
			av_log(NULL, AV_LOG_ERROR, "Could not alloc buffer.\n");
			exit_program(1);
		}
		memset(tmp + *size * elem_size, 0, (new_size - *size) * elem_size);
		*size = new_size;
		return tmp;
	}
	return array;
}

double get_rotation(AVStream *st)
{
	AVDictionaryEntry *rotate_tag = av_dict_get(st->metadata, "rotate", NULL, 0);
	uint8_t* displaymatrix = av_stream_get_side_data(st,
	                         AV_PKT_DATA_DISPLAYMATRIX, NULL);
	double theta = 0;

	if (rotate_tag && *rotate_tag->value && strcmp(rotate_tag->value, "0")) {
		char *tail;
		theta = av_strtod(rotate_tag->value, &tail);
		if (*tail)
			theta = 0;
	}
	if (displaymatrix && !theta)
		theta = -av_display_rotation_get((int32_t*) displaymatrix);

	theta -= 360 * floor(theta / 360 + 0.9 / 360);

	if (fabs(theta - 90 * round(theta / 90)) > 2)
		av_log(NULL, AV_LOG_WARNING, "Odd rotation angle.\n"
		       "If you want to help, upload a sample "
		       "of this file to ftp://upload.ffmpeg.org/incoming/ "
		       "and contact the ffmpeg-devel mailing list. (ffmpeg-devel@ffmpeg.org)");

	return theta;
}
