/*
 * Copyright (c) 2003 Fabrice Bellard
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

/**
 * @file
 * simple media player based on the FFmpeg libraries
 */

#include <inttypes.h>
#include <math.h>
#include <limits.h>
#include <signal.h>
#include <stdint.h>

#include <libavutil/avstring.h>
#include <libavutil/eval.h>
#include <libavutil/mathematics.h>
#include <libavutil/pixdesc.h>
#include <libavutil/imgutils.h>
#include <libavutil/dict.h>
#include <libavutil/parseutils.h>
#include <libavutil/samplefmt.h>
#include <libavutil/avassert.h>
#include <libavutil/time.h>
#include <libavformat/avformat.h>
#include <libavdevice/avdevice.h>
#include <libswscale/swscale.h>
#include <libavutil/opt.h>
#include <libavcodec/avfft.h>
#include <libswresample/swresample.h>

#include <libavfilter/avfilter.h>
#include <libavfilter/buffersink.h>
#include <libavfilter/buffersrc.h>

#include <SDL2/SDL.h>
#include <SDL2/SDL_thread.h>

#include "cmdutils.h"

const char program_name[] = "ffplay";
const int program_birth_year = 2003;

#define MAX_QUEUE_SIZE (15 * 1024 * 1024)
#define MIN_FRAMES 25

/* Minimum SDL audio buffer size, in samples. */
#define SDL_AUDIO_MIN_BUFFER_SIZE 512
/* Calculate actual buffer size keeping in mind not cause too frequent audio callbacks */
#define SDL_AUDIO_MAX_CALLBACKS_PER_SEC 30

/* Step size for volume control */
#define SDL_VOLUME_STEP (SDL_MIX_MAXVOLUME / 50)

/* no AV sync correction is done if below the minimum AV sync threshold */
#define AV_SYNC_THRESHOLD_MIN 0.04
/* AV sync correction is done if above the maximum AV sync threshold */
#define AV_SYNC_THRESHOLD_MAX 0.1
/* If a frame duration is longer than this, it will not be duplicated to compensate AV sync */
#define AV_SYNC_FRAMEDUP_THRESHOLD 0.1
/* no AV correction is done if too big error */
#define AV_NOSYNC_THRESHOLD 10.0

/* maximum audio speed change to get correct sync */
#define SAMPLE_CORRECTION_PERCENT_MAX 10

/* we use about AUDIO_DIFF_AVG_NB A-V differences to make the average */
#define AUDIO_DIFF_AVG_NB   20

/* polls for possible required screen refresh at least this often, should be less than 1/fps */
#define REFRESH_RATE 0.01

/* NOTE: the size must be big enough to compensate the hardware audio buffersize size */
/* TODO: We assume that a decoded and resampled frame fits into this buffer */
#define SAMPLE_ARRAY_SIZE (8 * 65536)

static unsigned sws_flags = SWS_BICUBIC;

typedef struct MyAVPacketList {
	AVPacket pkt;
	struct MyAVPacketList *next;
	int serial;
} MyAVPacketList;

typedef struct PacketQueue {
	MyAVPacketList *first_pkt, *last_pkt;
	int nb_packets;
	int size;
	int64_t duration;
	int abort_request;
	int serial;
	SDL_mutex *mutex;
	SDL_cond *cond;
} PacketQueue;

#define VIDEO_PICTURE_QUEUE_SIZE 3
#define SUBPICTURE_QUEUE_SIZE 16
#define SAMPLE_QUEUE_SIZE 9
#define FRAME_QUEUE_SIZE FFMAX(SAMPLE_QUEUE_SIZE, FFMAX(VIDEO_PICTURE_QUEUE_SIZE, SUBPICTURE_QUEUE_SIZE))

typedef struct AudioParams {
	int freq;
	int channels;
	int64_t channel_layout;
	enum AVSampleFormat fmt;
	int frame_size;
	int bytes_per_sec;
} AudioParams;

typedef struct Clock {
	double pts;           /* clock base */
	double pts_drift;     /* clock base minus time at which we updated the clock */
	double last_updated;
	double speed;
	int serial;           /* clock is based on a packet with this serial */
	int paused;
	int *queue_serial;    /* pointer to the current packet queue serial, used for obsolete clock detection */
} Clock;

/* Common struct for handling all types of decoded data and allocated render buffers. */
typedef struct Frame {
	AVFrame *frame;
	AVSubtitle sub;
	int serial;
	double pts;           /* presentation timestamp for the frame */
	double duration;      /* estimated duration of the frame */
	int64_t pos;          /* byte position of the frame in the input file */
	SDL_Texture *bmp;
	int allocated;
	int width;
	int height;
	int format;
	AVRational sar;
	int uploaded;
} Frame;

typedef struct FrameQueue {
	Frame queue[FRAME_QUEUE_SIZE];
	int rindex;
	int windex;
	int size;
	int max_size;
	int keep_last;
	int rindex_shown;
	SDL_mutex *mutex;
	SDL_cond *cond;
	PacketQueue *pktq;
} FrameQueue;

enum {
	AV_SYNC_AUDIO_MASTER, /* default choice */
};

typedef struct Decoder {
	AVPacket pkt;
	AVPacket pkt_temp;
	PacketQueue *queue;
	AVCodecContext *avctx;
	int pkt_serial;
	int finished;
	int packet_pending;
	SDL_cond *empty_queue_cond;
	int64_t start_pts;
	AVRational start_pts_tb;
	int64_t next_pts;
	AVRational next_pts_tb;
	SDL_Thread *decoder_tid;
} Decoder;

typedef struct VideoState {
	SDL_Thread *read_tid;
	AVInputFormat *iformat;
	int abort_request;
	int force_refresh;
	int queue_attachments_req;
	int seek_req;
	int seek_flags;
	int64_t seek_pos;
	int64_t seek_rel;
	int read_pause_return;
	AVFormatContext *ic;

	Clock audclk;
	Clock vidclk;
	Clock extclk;

	FrameQueue pictq;
	FrameQueue subpq;
	FrameQueue sampq;

	Decoder auddec;
	Decoder viddec;
	Decoder subdec;

	int audio_stream;

	int av_sync_type;

	double audio_clock;
	int audio_clock_serial;
	double audio_diff_cum; /* used for AV difference average computation */
	double audio_diff_avg_coef;
	double audio_diff_threshold;
	int audio_diff_avg_count;
	AVStream *audio_st;
	PacketQueue audioq;
	int audio_hw_buf_size;
	uint8_t *audio_buf;
	uint8_t *audio_buf1;
	unsigned int audio_buf_size; /* in bytes */
	unsigned int audio_buf1_size;
	int audio_buf_index; /* in bytes */
	int audio_write_buf_size;
	int audio_volume;
	struct AudioParams audio_src;
	struct AudioParams audio_filter_src;
	struct AudioParams audio_tgt;
	struct SwrContext *swr_ctx;
	int frame_drops_early;
	int frame_drops_late;

	enum ShowMode {
		SHOW_MODE_NONE = -1, SHOW_MODE_VIDEO = 0, SHOW_MODE_WAVES, SHOW_MODE_RDFT, SHOW_MODE_NB
	} show_mode;
	int16_t sample_array[SAMPLE_ARRAY_SIZE];
	int sample_array_index;
	int last_i_start;
	RDFTContext *rdft;
	int rdft_bits;
	FFTSample *rdft_data;
	int xpos;
	double last_vis_time;
	SDL_Texture *vis_texture;
	SDL_Texture *sub_texture;

	int subtitle_stream;
	AVStream *subtitle_st;
	PacketQueue subtitleq;

	double frame_timer;
	double frame_last_returned_time;
	double frame_last_filter_delay;
	int video_stream;
	AVStream *video_st;
	PacketQueue videoq;
	double max_frame_duration;      // maximum duration of a frame - above this, we consider the jump a timestamp discontinuity
	struct SwsContext *img_convert_ctx;
	struct SwsContext *sub_convert_ctx;
	int eof;

	char *filename;
	int width, height;

	int vfilter_idx;
	AVFilterContext *in_video_filter;   // the first filter in the video chain
	AVFilterContext *out_video_filter;  // the last filter in the video chain
	AVFilterContext *in_audio_filter;   // the first filter in the audio chain
	AVFilterContext *out_audio_filter;  // the last filter in the audio chain
	AVFilterGraph *agraph;              // audio filter graph

	int last_video_stream, last_audio_stream, last_subtitle_stream;

	SDL_cond *continue_read_thread;
} VideoState;

/* options specified by the user */
static AVInputFormat *file_iformat;
static const char *input_filename;
static const char *window_title;
static int default_width = 640;
static int default_height = 480;
static int screen_width = 0;
static int screen_height = 0;
static const char *wanted_stream_spec[AVMEDIA_TYPE_NB] = {0};
static int seek_by_bytes = -1;
static int av_sync_type = AV_SYNC_AUDIO_MASTER;
static int64_t start_time = AV_NOPTS_VALUE;
static int64_t duration = AV_NOPTS_VALUE;
static int fast = 0;
static int genpts = 0;
static int lowres = 0;
static int decoder_reorder_pts = -1;
static int autoexit;
static int loop = 1;
static int framedrop = -1;
static int infinite_buffer = -1;
static enum ShowMode show_mode = SHOW_MODE_NONE;
static const char *audio_codec_name;
static const char *subtitle_codec_name;
static const char *video_codec_name;
double rdftspeed = 0.02;
static const char **vfilters_list = NULL;
static char *afilters = NULL;

/* current context */
static int64_t audio_callback_time;

static AVPacket flush_pkt;

#define FF_ALLOC_EVENT   (SDL_USEREVENT)
#define FF_QUIT_EVENT    (SDL_USEREVENT + 2)

static SDL_Window *window;
static SDL_Renderer *renderer;

static inline
int cmp_audio_fmts(enum AVSampleFormat fmt1, int64_t channel_count1,
                   enum AVSampleFormat fmt2, int64_t channel_count2)
{
	/* If channel count == 1, planar and non-planar formats are the same */
	if (channel_count1 == 1 && channel_count2 == 1)
		return av_get_packed_sample_fmt(fmt1) != av_get_packed_sample_fmt(fmt2);
	else
		return channel_count1 != channel_count2 || fmt1 != fmt2;
}

static inline
int64_t get_valid_channel_layout(int64_t channel_layout, int channels)
{
	if (channel_layout &&
	    av_get_channel_layout_nb_channels(channel_layout) == channels)
		return channel_layout;
	else
		return 0;
}

static int packet_queue_put_private(PacketQueue *q, AVPacket *pkt)
{
	MyAVPacketList *pkt1;

	if (q->abort_request)
		return -1;

	pkt1 = av_malloc(sizeof(MyAVPacketList));
	if (!pkt1)
		return -1;
	pkt1->pkt = *pkt;
	pkt1->next = NULL;
	if (pkt == &flush_pkt)
		q->serial++;
	pkt1->serial = q->serial;

	if (!q->last_pkt)
		q->first_pkt = pkt1;
	else
		q->last_pkt->next = pkt1;
	q->last_pkt = pkt1;
	q->nb_packets++;
	q->size += pkt1->pkt.size + sizeof(*pkt1);
	q->duration += pkt1->pkt.duration;
	/* XXX: should duplicate packet data in DV case */
	SDL_CondSignal(q->cond);
	return 0;
}

static int packet_queue_put(PacketQueue *q, AVPacket *pkt)
{
	int ret;

	SDL_LockMutex(q->mutex);
	ret = packet_queue_put_private(q, pkt);
	SDL_UnlockMutex(q->mutex);

	if (pkt != &flush_pkt && ret < 0)
		av_packet_unref(pkt);

	return ret;
}

static int packet_queue_put_nullpacket(PacketQueue *q, int stream_index)
{
	AVPacket pkt1, *pkt = &pkt1;
	av_init_packet(pkt);
	pkt->data = NULL;
	pkt->size = 0;
	pkt->stream_index = stream_index;
	return packet_queue_put(q, pkt);
}

/* packet queue handling */
static int packet_queue_init(PacketQueue *q)
{
	memset(q, 0, sizeof(PacketQueue));
	q->mutex = SDL_CreateMutex();
	if (!q->mutex) {
		av_log(NULL, AV_LOG_FATAL, "SDL_CreateMutex(): %s\n", SDL_GetError());
		return AVERROR(ENOMEM);
	}
	q->cond = SDL_CreateCond();
	if (!q->cond) {
		av_log(NULL, AV_LOG_FATAL, "SDL_CreateCond(): %s\n", SDL_GetError());
		return AVERROR(ENOMEM);
	}
	q->abort_request = 1;
	return 0;
}

static void packet_queue_flush(PacketQueue *q)
{
	MyAVPacketList *pkt, *pkt1;

	SDL_LockMutex(q->mutex);
	for (pkt = q->first_pkt; pkt; pkt = pkt1) {
		pkt1 = pkt->next;
		av_packet_unref(&pkt->pkt);
		av_freep(&pkt);
	}
	q->last_pkt = NULL;
	q->first_pkt = NULL;
	q->nb_packets = 0;
	q->size = 0;
	q->duration = 0;
	SDL_UnlockMutex(q->mutex);
}

static void packet_queue_start(PacketQueue *q)
{
	SDL_LockMutex(q->mutex);
	q->abort_request = 0;
	packet_queue_put_private(q, &flush_pkt);
	SDL_UnlockMutex(q->mutex);
}

/* return < 0 if aborted, 0 if no packet and > 0 if packet.  */
static int packet_queue_get(PacketQueue *q, AVPacket *pkt, int block,
                            int *serial)
{
	MyAVPacketList *pkt1;
	int ret;

	SDL_LockMutex(q->mutex);

	while (1) {
		if (q->abort_request) {
			ret = -1;
			break;
		}

		pkt1 = q->first_pkt;
		if (pkt1) {
			q->first_pkt = pkt1->next;
			if (!q->first_pkt)
				q->last_pkt = NULL;
			q->nb_packets--;
			q->size -= pkt1->pkt.size + sizeof(*pkt1);
			q->duration -= pkt1->pkt.duration;
			*pkt = pkt1->pkt;
			if (serial)
				*serial = pkt1->serial;
			av_free(pkt1);
			ret = 1;
			break;
		} else if (!block) {
			ret = 0;
			break;
		} else {
			SDL_CondWait(q->cond, q->mutex);
		}
	}
	SDL_UnlockMutex(q->mutex);
	return ret;
}

static void decoder_init(Decoder *d, AVCodecContext *avctx, PacketQueue *queue,
                         SDL_cond *empty_queue_cond)
{
	memset(d, 0, sizeof(Decoder));
	d->avctx = avctx;
	d->queue = queue;
	d->empty_queue_cond = empty_queue_cond;
	d->start_pts = AV_NOPTS_VALUE;
}

static int decoder_decode_frame(Decoder *d, AVFrame *frame, AVSubtitle *sub)
{
	int got_frame = 0;
	AVPacket pkt;

	do {
		int ret = -1;

		if (d->queue->abort_request)
			return -1;

		if (!d->packet_pending || d->queue->serial != d->pkt_serial) {
			do {
				if (d->queue->nb_packets == 0)
					SDL_CondSignal(d->empty_queue_cond);
				if (packet_queue_get(d->queue, &pkt, 1, &d->pkt_serial) < 0)
					return -1;
				if (pkt.data == flush_pkt.data) {
					avcodec_flush_buffers(d->avctx);
					d->finished = 0;
					d->next_pts = d->start_pts;
					d->next_pts_tb = d->start_pts_tb;
				}
			} while (pkt.data == flush_pkt.data || d->queue->serial != d->pkt_serial);
			av_packet_unref(&d->pkt);
			d->pkt_temp = d->pkt = pkt;
			d->packet_pending = 1;
		}

		switch (d->avctx->codec_type) {
		case AVMEDIA_TYPE_VIDEO:
			ret = avcodec_decode_video2(d->avctx, frame, &got_frame, &d->pkt_temp);
			if (got_frame) {
				if (decoder_reorder_pts == -1) {
					frame->pts = av_frame_get_best_effort_timestamp(frame);
				} else if (!decoder_reorder_pts) {
					frame->pts = frame->pkt_dts;
				}
			}
			break;
		case AVMEDIA_TYPE_AUDIO:
			ret = avcodec_decode_audio4(d->avctx, frame, &got_frame, &d->pkt_temp);
			if (got_frame) {
				AVRational tb = (AVRational) {
					1, frame->sample_rate
				};
				if (frame->pts != AV_NOPTS_VALUE)
					frame->pts = av_rescale_q(frame->pts, av_codec_get_pkt_timebase(d->avctx), tb);
				else if (d->next_pts != AV_NOPTS_VALUE)
					frame->pts = av_rescale_q(d->next_pts, d->next_pts_tb, tb);
				if (frame->pts != AV_NOPTS_VALUE) {
					d->next_pts = frame->pts + frame->nb_samples;
					d->next_pts_tb = tb;
				}
			}
			break;
		case AVMEDIA_TYPE_SUBTITLE:
			ret = avcodec_decode_subtitle2(d->avctx, sub, &got_frame, &d->pkt_temp);
			break;
		default:
			break;
		}

		if (ret < 0) {
			d->packet_pending = 0;
		} else {
			d->pkt_temp.dts = d->pkt_temp.pts = AV_NOPTS_VALUE;
			if (d->pkt_temp.data) {
				if (d->avctx->codec_type != AVMEDIA_TYPE_AUDIO)
					ret = d->pkt_temp.size;
				d->pkt_temp.data += ret;
				d->pkt_temp.size -= ret;
				if (d->pkt_temp.size <= 0)
					d->packet_pending = 0;
			} else {
				if (!got_frame) {
					d->packet_pending = 0;
					d->finished = d->pkt_serial;
				}
			}
		}
	} while (!got_frame && !d->finished);

	return got_frame;
}

static void frame_queue_unref_item(Frame *vp)
{
	av_frame_unref(vp->frame);
	avsubtitle_free(&vp->sub);
}

static int frame_queue_init(FrameQueue *f, PacketQueue *pktq, int max_size,
                            int keep_last)
{
	int i;
	memset(f, 0, sizeof(FrameQueue));
	if (!(f->mutex = SDL_CreateMutex())) {
		av_log(NULL, AV_LOG_FATAL, "SDL_CreateMutex(): %s\n", SDL_GetError());
		return AVERROR(ENOMEM);
	}
	if (!(f->cond = SDL_CreateCond())) {
		av_log(NULL, AV_LOG_FATAL, "SDL_CreateCond(): %s\n", SDL_GetError());
		return AVERROR(ENOMEM);
	}
	f->pktq = pktq;
	f->max_size = FFMIN(max_size, FRAME_QUEUE_SIZE);
	f->keep_last = !!keep_last;
	for (i = 0; i < f->max_size; i++)
		if (!(f->queue[i].frame = av_frame_alloc()))
			return AVERROR(ENOMEM);
	return 0;
}

static Frame *frame_queue_peek(FrameQueue *f)
{
	return &f->queue[(f->rindex + f->rindex_shown) % f->max_size];
}

static Frame *frame_queue_peek_next(FrameQueue *f)
{
	return &f->queue[(f->rindex + f->rindex_shown + 1) % f->max_size];
}

static Frame *frame_queue_peek_last(FrameQueue *f)
{
	return &f->queue[f->rindex];
}

static Frame *frame_queue_peek_writable(FrameQueue *f)
{
	/* wait until we have space to put a new frame */
	SDL_LockMutex(f->mutex);
	while (f->size >= f->max_size &&
	       !f->pktq->abort_request) {
		SDL_CondWait(f->cond, f->mutex);
	}
	SDL_UnlockMutex(f->mutex);

	if (f->pktq->abort_request)
		return NULL;

	return &f->queue[f->windex];
}

static Frame *frame_queue_peek_readable(FrameQueue *f)
{
	/* wait until we have a readable a new frame */
	SDL_LockMutex(f->mutex);
	while (f->size - f->rindex_shown <= 0 &&
	       !f->pktq->abort_request) {
		SDL_CondWait(f->cond, f->mutex);
	}
	SDL_UnlockMutex(f->mutex);

	if (f->pktq->abort_request)
		return NULL;

	return &f->queue[(f->rindex + f->rindex_shown) % f->max_size];
}

static void frame_queue_push(FrameQueue *f)
{
	if (++f->windex == f->max_size)
		f->windex = 0;
	SDL_LockMutex(f->mutex);
	f->size++;
	SDL_CondSignal(f->cond);
	SDL_UnlockMutex(f->mutex);
}

static void frame_queue_next(FrameQueue *f)
{
	if (f->keep_last && !f->rindex_shown) {
		f->rindex_shown = 1;
		return;
	}
	frame_queue_unref_item(&f->queue[f->rindex]);
	if (++f->rindex == f->max_size)
		f->rindex = 0;
	SDL_LockMutex(f->mutex);
	f->size--;
	SDL_CondSignal(f->cond);
	SDL_UnlockMutex(f->mutex);
}

/* return the number of undisplayed frames in the queue */
static int frame_queue_nb_remaining(FrameQueue *f)
{
	return f->size - f->rindex_shown;
}

/* return last shown position */
static int64_t frame_queue_last_pos(FrameQueue *f)
{
	Frame *fp = &f->queue[f->rindex];
	if (f->rindex_shown && fp->serial == f->pktq->serial)
		return fp->pos;
	else
		return -1;
}

static inline void fill_rectangle(int x, int y, int w, int h)
{
	SDL_Rect rect;
	rect.x = x;
	rect.y = y;
	rect.w = w;
	rect.h = h;
	if (w && h)
		SDL_RenderFillRect(renderer, &rect);
}

static int realloc_texture(SDL_Texture **texture, Uint32 new_format,
                           int new_width, int new_height, SDL_BlendMode blendmode, int init_texture)
{
	Uint32 format;
	int access, w, h;
	if (SDL_QueryTexture(*texture, &format, &access, &w, &h) < 0 ||
	    new_width != w || new_height != h || new_format != format) {
		void *pixels;
		int pitch;
		SDL_DestroyTexture(*texture);
		if (!(*texture = SDL_CreateTexture(renderer, new_format,
		                                   SDL_TEXTUREACCESS_STREAMING, new_width, new_height)))
			return -1;
		if (SDL_SetTextureBlendMode(*texture, blendmode) < 0)
			return -1;
		if (init_texture) {
			if (SDL_LockTexture(*texture, NULL, &pixels, &pitch) < 0)
				return -1;
			memset(pixels, 0, pitch * new_height);
			SDL_UnlockTexture(*texture);
		}
	}
	return 0;
}

static void calculate_display_rect(SDL_Rect *rect, int scr_width,
                                   int scr_height, int pic_width, int pic_height, AVRational pic_sar)
{
	float aspect_ratio;
	int width, height, x, y;

	if (pic_sar.num == 0)
		aspect_ratio = 0;
	else
		aspect_ratio = av_q2d(pic_sar);

	if (aspect_ratio <= 0.0)
		aspect_ratio = 1.0;
	aspect_ratio *= (float)pic_width / (float)pic_height;

	/* XXX: we suppose the screen has a 1.0 pixel ratio */
	height = scr_height;
	width = lrint(height * aspect_ratio) & ~1;
	if (width > scr_width) {
		width = scr_width;
		height = lrint(width / aspect_ratio) & ~1;
	}
	x = (scr_width - width) / 2;
	y = (scr_height - height) / 2;
	rect->x = x;
	rect->y = y;
	rect->w = FFMAX(width,  1);
	rect->h = FFMAX(height, 1);
}

static int upload_texture(SDL_Texture *tex, AVFrame *frame,
                          struct SwsContext **img_convert_ctx)
{
	int ret = 0;
	switch (frame->format) {
	case AV_PIX_FMT_YUV420P:
		ret = SDL_UpdateYUVTexture(tex, NULL, frame->data[0], frame->linesize[0],
		                           frame->data[1], frame->linesize[1],
		                           frame->data[2], frame->linesize[2]);
		break;
	case AV_PIX_FMT_BGRA:
		ret = SDL_UpdateTexture(tex, NULL, frame->data[0], frame->linesize[0]);
		break;
	default:
		/* This should only happen if we are not using avfilter... */
		*img_convert_ctx = sws_getCachedContext(*img_convert_ctx,
		                                        frame->width, frame->height, frame->format, frame->width, frame->height,
		                                        AV_PIX_FMT_BGRA, sws_flags, NULL, NULL, NULL);
		if (*img_convert_ctx != NULL) {
			uint8_t *pixels[4];
			int pitch[4];
			if (!SDL_LockTexture(tex, NULL, (void **)pixels, pitch)) {
				sws_scale(*img_convert_ctx, (const uint8_t *const *)frame->data,
				          frame->linesize,
				          0, frame->height, pixels, pitch);
				SDL_UnlockTexture(tex);
			}
		} else {
			av_log(NULL, AV_LOG_FATAL, "Cannot initialize the conversion context\n");
			ret = -1;
		}
		break;
	}
	return ret;
}

static void video_image_display(VideoState *is)
{
	Frame *vp;
	Frame *sp = NULL;
	SDL_Rect rect;

	vp = frame_queue_peek_last(&is->pictq);
	if (vp->bmp) {
		if (is->subtitle_st) {
			if (frame_queue_nb_remaining(&is->subpq) > 0) {
				sp = frame_queue_peek(&is->subpq);

				if (vp->pts >= sp->pts + ((float) sp->sub.start_display_time / 1000)) {
					if (!sp->uploaded) {
						uint8_t *pixels[4];
						int pitch[4];
						int i;
						if (!sp->width || !sp->height) {
							sp->width = vp->width;
							sp->height = vp->height;
						}
						if (realloc_texture(&is->sub_texture, SDL_PIXELFORMAT_ARGB8888, sp->width,
						                    sp->height, SDL_BLENDMODE_BLEND, 1) < 0)
							return;

						for (i = 0; i < sp->sub.num_rects; i++) {
							AVSubtitleRect *sub_rect = sp->sub.rects[i];

							sub_rect->x = av_clip(sub_rect->x, 0, sp->width);
							sub_rect->y = av_clip(sub_rect->y, 0, sp->height);
							sub_rect->w = av_clip(sub_rect->w, 0, sp->width  - sub_rect->x);
							sub_rect->h = av_clip(sub_rect->h, 0, sp->height - sub_rect->y);

							is->sub_convert_ctx = sws_getCachedContext(is->sub_convert_ctx,
							                      sub_rect->w, sub_rect->h, AV_PIX_FMT_PAL8,
							                      sub_rect->w, sub_rect->h, AV_PIX_FMT_BGRA,
							                      0, NULL, NULL, NULL);
							if (!is->sub_convert_ctx) {
								av_log(NULL, AV_LOG_FATAL, "Cannot initialize the conversion context\n");
								return;
							}
							if (!SDL_LockTexture(is->sub_texture, (SDL_Rect *)sub_rect, (void **)pixels,
							                     pitch)) {
								sws_scale(is->sub_convert_ctx, (const uint8_t *const *)sub_rect->data,
								          sub_rect->linesize,
								          0, sub_rect->h, pixels, pitch);
								SDL_UnlockTexture(is->sub_texture);
							}
						}
						sp->uploaded = 1;
					}
				} else
					sp = NULL;
			}
		}

		calculate_display_rect(&rect, is->width, is->height, vp->width, vp->height,
		                       vp->sar);

		if (!vp->uploaded) {
			if (upload_texture(vp->bmp, vp->frame, &is->img_convert_ctx) < 0)
				return;
			vp->uploaded = 1;
		}

		SDL_RenderCopy(renderer, vp->bmp, NULL, &rect);
		if (sp) {
			SDL_RenderCopy(renderer, is->sub_texture, NULL, &rect);
		}
	}
}

static void do_exit(VideoState *is)
{
	av_log(NULL, AV_LOG_QUIET, "%s", "");
	exit(0);
}

static void sigterm_handler(int sig)
{
	exit(123);
}

static void set_default_window_size(int width, int height, AVRational sar)
{
	SDL_Rect rect;
	calculate_display_rect(&rect, INT_MAX, height, width, height, sar);
	default_width = rect.w;
	default_height = rect.h;
}

static int video_open(VideoState *is, Frame *vp)
{
	int w, h;

	if (vp && vp->width)
		set_default_window_size(vp->width, vp->height, vp->sar);

	if (screen_width) {
		w = screen_width;
		h = screen_height;
	} else {
		w = default_width;
		h = default_height;
	}

	if (!window) {
		int flags = SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE;
		if (!window_title)
			window_title = input_filename;
		window = SDL_CreateWindow(window_title, SDL_WINDOWPOS_UNDEFINED,
		                          SDL_WINDOWPOS_UNDEFINED, w, h, flags);
		SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "linear");
		if (window) {
			SDL_RendererInfo info;
			renderer = SDL_CreateRenderer(window, -1,
			                              SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
			if (renderer) {
				if (!SDL_GetRendererInfo(renderer, &info))
					av_log(NULL, AV_LOG_VERBOSE, "Initialized %s renderer.\n", info.name);
			}
		}
	} else {
		SDL_SetWindowSize(window, w, h);
	}

	if (!window || !renderer) {
		av_log(NULL, AV_LOG_FATAL, "SDL: could not set video mode - exiting\n");
		do_exit(is);
	}

	is->width = w;
	is->height = h;

	return 0;
}

/* display the current picture, if any */
static void video_display(VideoState *is)
{
	if (!window)
		video_open(is, NULL);

	SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
	SDL_RenderClear(renderer);
	video_image_display(is);
	SDL_RenderPresent(renderer);
}

static double get_clock(Clock *c)
{
	if (*c->queue_serial != c->serial)
		return NAN;
	if (c->paused) {
		return c->pts;
	} else {
		double time = av_gettime_relative() / 1000000.0;
		return c->pts_drift + time - (time - c->last_updated) * (1.0 - c->speed);
	}
}

static void set_clock_at(Clock *c, double pts, int serial, double time)
{
	c->pts = pts;
	c->last_updated = time;
	c->pts_drift = c->pts - time;
	c->serial = serial;
}

static void set_clock(Clock *c, double pts, int serial)
{
	double time = av_gettime_relative() / 1000000.0;
	set_clock_at(c, pts, serial, time);
}

static void init_clock(Clock *c, int *queue_serial)
{
	c->speed = 1.0;
	c->paused = 0;
	c->queue_serial = queue_serial;
	set_clock(c, NAN, -1);
}

static void sync_clock_to_slave(Clock *c, Clock *slave)
{
	double clock = get_clock(c);
	double slave_clock = get_clock(slave);
	if (!isnan(slave_clock) && (isnan(clock) ||
	                            fabs(clock - slave_clock) > AV_NOSYNC_THRESHOLD))
		set_clock(c, slave_clock, slave->serial);
}

/* get the current master clock value */
static double get_master_clock(VideoState *is)
{
	return get_clock(&is->audclk);
}

/* seek in the stream */
static void stream_seek(VideoState *is, int64_t pos, int64_t rel,
                        int seek_by_bytes)
{
	if (!is->seek_req) {
		is->seek_pos = pos;
		is->seek_rel = rel;
		is->seek_flags &= ~AVSEEK_FLAG_BYTE;
		if (seek_by_bytes)
			is->seek_flags |= AVSEEK_FLAG_BYTE;
		is->seek_req = 1;
		SDL_CondSignal(is->continue_read_thread);
	}
}

static void update_volume(VideoState *is, int sign, int step)
{
	is->audio_volume = av_clip(is->audio_volume + sign * step, 0,
	                           SDL_MIX_MAXVOLUME);
}

static double compute_target_delay(double delay, VideoState *is)
{
	double sync_threshold, diff = 0;

	/* if video is slave, we try to correct big delays by
	   duplicating or deleting a frame */
	diff = get_clock(&is->vidclk) - get_master_clock(is);

	/* skip or repeat frame. We take into account the
	   delay to compute the threshold. I still don't know
	   if it is the best guess */
	sync_threshold = FFMAX(AV_SYNC_THRESHOLD_MIN, FFMIN(AV_SYNC_THRESHOLD_MAX,
	                       delay));
	if (!isnan(diff) && fabs(diff) < is->max_frame_duration) {
		if (diff <= -sync_threshold)
			delay = FFMAX(0, delay + diff);
		else if (diff >= sync_threshold && delay > AV_SYNC_FRAMEDUP_THRESHOLD)
			delay = delay + diff;
		else if (diff >= sync_threshold)
			delay = 2 * delay;
	}

	av_log(NULL, AV_LOG_TRACE, "video: delay=%0.3f A-V=%f\n", delay, -diff);

	return delay;
}

static double vp_duration(VideoState *is, Frame *vp, Frame *nextvp)
{
	if (vp->serial == nextvp->serial) {
		double duration = nextvp->pts - vp->pts;
		if (isnan(duration) || duration <= 0 || duration > is->max_frame_duration)
			return vp->duration;
		else
			return duration;
	} else {
		return 0.0;
	}
}

static void update_video_pts(VideoState *is, double pts, int64_t pos,
                             int serial)
{
	/* update current video pts */
	set_clock(&is->vidclk, pts, serial);
	sync_clock_to_slave(&is->extclk, &is->vidclk);
}

/* called to display each frame */
static void video_refresh(void *opaque, double *remaining_time)
{
	VideoState *is = opaque;
	double time;

	Frame *sp, *sp2;

	if (is->show_mode != SHOW_MODE_VIDEO && is->audio_st) {
		time = av_gettime_relative() / 1000000.0;
		if (is->force_refresh || is->last_vis_time + rdftspeed < time) {
			video_display(is);
			is->last_vis_time = time;
		}
		*remaining_time = FFMIN(*remaining_time, is->last_vis_time + rdftspeed - time);
	}

	if (is->video_st) {
retry:
		if (frame_queue_nb_remaining(&is->pictq) == 0) {
			// nothing to do, no picture to display in the queue
		} else {
			double last_duration, duration, delay;
			Frame *vp, *lastvp;

			/* dequeue the picture */
			lastvp = frame_queue_peek_last(&is->pictq);
			vp = frame_queue_peek(&is->pictq);

			if (vp->serial != is->videoq.serial) {
				frame_queue_next(&is->pictq);
				goto retry;
			}

			if (lastvp->serial != vp->serial)
				is->frame_timer = av_gettime_relative() / 1000000.0;

			/* compute nominal last_duration */
			last_duration = vp_duration(is, lastvp, vp);
			delay = compute_target_delay(last_duration, is);

			time = av_gettime_relative() / 1000000.0;
			if (time < is->frame_timer + delay) {
				*remaining_time = FFMIN(is->frame_timer + delay - time, *remaining_time);
				goto display;
			}

			is->frame_timer += delay;
			if (delay > 0 && time - is->frame_timer > AV_SYNC_THRESHOLD_MAX)
				is->frame_timer = time;

			SDL_LockMutex(is->pictq.mutex);
			if (!isnan(vp->pts))
				update_video_pts(is, vp->pts, vp->pos, vp->serial);
			SDL_UnlockMutex(is->pictq.mutex);

			if (frame_queue_nb_remaining(&is->pictq) > 1) {
				Frame *nextvp = frame_queue_peek_next(&is->pictq);
				duration = vp_duration(is, vp, nextvp);
				if ((framedrop > 0 || framedrop) && time > is->frame_timer + duration) {
					is->frame_drops_late++;
					frame_queue_next(&is->pictq);
					goto retry;
				}
			}

			if (is->subtitle_st) {
				while (frame_queue_nb_remaining(&is->subpq) > 0) {
					sp = frame_queue_peek(&is->subpq);

					if (frame_queue_nb_remaining(&is->subpq) > 1)
						sp2 = frame_queue_peek_next(&is->subpq);
					else
						sp2 = NULL;

					if (sp->serial != is->subtitleq.serial ||
					    (is->vidclk.pts > (sp->pts + ((float) sp->sub.end_display_time / 1000)))
					    || (sp2 &&
					        is->vidclk.pts > (sp2->pts + ((float) sp2->sub.start_display_time / 1000)))) {
						if (sp->uploaded) {
							int i;
							for (i = 0; i < sp->sub.num_rects; i++) {
								AVSubtitleRect *sub_rect = sp->sub.rects[i];
								uint8_t *pixels;
								int pitch, j;

								if (!SDL_LockTexture(is->sub_texture, (SDL_Rect *)sub_rect, (void **)&pixels,
								                     &pitch)) {
									for (j = 0; j < sub_rect->h; j++, pixels += pitch)
										memset(pixels, 0, sub_rect->w << 2);
									SDL_UnlockTexture(is->sub_texture);
								}
							}
						}
						frame_queue_next(&is->subpq);
					} else {
						break;
					}
				}
			}

			frame_queue_next(&is->pictq);
			is->force_refresh = 1;
		}
display:
		/* display picture */
		if (is->force_refresh && is->show_mode == SHOW_MODE_VIDEO &&
		    is->pictq.rindex_shown)
			video_display(is);
	}
	is->force_refresh = 0;
}

/* allocate a picture (needs to do that in main thread to avoid
   potential locking problems */
static void alloc_picture(VideoState *is)
{
	Frame *vp;
	int sdl_format;

	vp = &is->pictq.queue[is->pictq.windex];

	video_open(is, vp);

	if (vp->format == AV_PIX_FMT_YUV420P)
		sdl_format = SDL_PIXELFORMAT_YV12;
	else
		sdl_format = SDL_PIXELFORMAT_ARGB8888;

	if (realloc_texture(&vp->bmp, sdl_format, vp->width, vp->height,
	                    SDL_BLENDMODE_NONE, 0) < 0) {
		/* SDL allocates a buffer smaller than requested if the video
		 * overlay hardware is unable to support the requested size. */
		av_log(NULL, AV_LOG_FATAL,
		       "Error: the video system does not support an image\n"
		       "size of %dx%d pixels. Try using -lowres or -vf \"scale=w:h\"\n"
		       "to reduce the image size.\n", vp->width, vp->height);
		do_exit(is);
	}

	SDL_LockMutex(is->pictq.mutex);
	vp->allocated = 1;
	SDL_CondSignal(is->pictq.cond);
	SDL_UnlockMutex(is->pictq.mutex);
}

static int queue_picture(VideoState *is, AVFrame *src_frame, double pts,
                         double duration, int64_t pos, int serial)
{
	Frame *vp;

	if (!(vp = frame_queue_peek_writable(&is->pictq)))
		return -1;

	vp->sar = src_frame->sample_aspect_ratio;
	vp->uploaded = 0;

	/* alloc or resize hardware picture buffer */
	if (!vp->bmp || !vp->allocated ||
	    vp->width != src_frame->width ||
	    vp->height != src_frame->height ||
	    vp->format != src_frame->format) {
		SDL_Event event;

		vp->allocated = 0;
		vp->width = src_frame->width;
		vp->height = src_frame->height;
		vp->format = src_frame->format;

		/* the allocation must be done in the main thread to avoid
		   locking problems. */
		event.type = FF_ALLOC_EVENT;
		event.user.data1 = is;
		SDL_PushEvent(&event);

		/* wait until the picture is allocated */
		SDL_LockMutex(is->pictq.mutex);
		while (!vp->allocated && !is->videoq.abort_request) {
			SDL_CondWait(is->pictq.cond, is->pictq.mutex);
		}
		/* if the queue is aborted, we have to pop the pending ALLOC event or wait for the allocation to complete */
		if (is->videoq.abort_request &&
		    SDL_PeepEvents(&event, 1, SDL_GETEVENT, FF_ALLOC_EVENT, FF_ALLOC_EVENT) != 1) {
			while (!vp->allocated && !is->abort_request) {
				SDL_CondWait(is->pictq.cond, is->pictq.mutex);
			}
		}
		SDL_UnlockMutex(is->pictq.mutex);

		if (is->videoq.abort_request)
			return -1;
	}

	/* if the frame is not skipped, then display it */
	if (vp->bmp) {
		vp->pts = pts;
		vp->duration = duration;
		vp->pos = pos;
		vp->serial = serial;

		av_frame_move_ref(vp->frame, src_frame);
		frame_queue_push(&is->pictq);
	}
	return 0;
}

static int get_video_frame(VideoState *is, AVFrame *frame)
{
	int got_picture;

	if ((got_picture = decoder_decode_frame(&is->viddec, frame, NULL)) < 0)
		return -1;

	if (got_picture) {
		double dpts = NAN;

		if (frame->pts != AV_NOPTS_VALUE)
			dpts = av_q2d(is->video_st->time_base) * frame->pts;

		frame->sample_aspect_ratio = av_guess_sample_aspect_ratio(is->ic, is->video_st,
		                             frame);

		if (framedrop > 0 || framedrop) {
			if (frame->pts != AV_NOPTS_VALUE) {
				double diff = dpts - get_master_clock(is);
				if (!isnan(diff) && fabs(diff) < AV_NOSYNC_THRESHOLD &&
				    diff - is->frame_last_filter_delay < 0 &&
				    is->viddec.pkt_serial == is->vidclk.serial &&
				    is->videoq.nb_packets) {
					is->frame_drops_early++;
					av_frame_unref(frame);
					got_picture = 0;
				}
			}
		}
	}

	return got_picture;
}

static int configure_filtergraph(AVFilterGraph *graph, const char *filtergraph,
                                 AVFilterContext *source_ctx, AVFilterContext *sink_ctx)
{
	int ret, i;
	int nb_filters = graph->nb_filters;
	AVFilterInOut *outputs = NULL, *inputs = NULL;

	if (filtergraph) {
		outputs = avfilter_inout_alloc();
		inputs = avfilter_inout_alloc();
		if (!outputs || !inputs) {
			ret = AVERROR(ENOMEM);
			goto fail;
		}

		outputs->name = av_strdup("in");
		outputs->filter_ctx = source_ctx;
		outputs->pad_idx = 0;
		outputs->next = NULL;

		inputs->name = av_strdup("out");
		inputs->filter_ctx = sink_ctx;
		inputs->pad_idx = 0;
		inputs->next = NULL;

		if ((ret = avfilter_graph_parse_ptr(graph, filtergraph, &inputs, &outputs,
		                                    NULL)) < 0)
			goto fail;
	} else {
		if ((ret = avfilter_link(source_ctx, 0, sink_ctx, 0)) < 0)
			goto fail;
	}

	/* Reorder the filters to ensure that inputs of the custom filters are merged first */
	for (i = 0; i < graph->nb_filters - nb_filters; i++)
		FFSWAP(AVFilterContext *, graph->filters[i], graph->filters[i + nb_filters]);

	ret = avfilter_graph_config(graph, NULL);
fail:
	avfilter_inout_free(&outputs);
	avfilter_inout_free(&inputs);
	return ret;
}

static int configure_video_filters(AVFilterGraph *graph, VideoState *is,
                                   const char *vfilters, AVFrame *frame)
{
	static const enum AVPixelFormat pix_fmts[] = { AV_PIX_FMT_YUV420P, AV_PIX_FMT_BGRA, AV_PIX_FMT_NONE };
	char sws_flags_str[512] = "";
	char buffersrc_args[256];
	int ret;
	AVFilterContext *filt_src = NULL, *filt_out = NULL, *last_filter = NULL;
	AVCodecParameters *codecpar = is->video_st->codecpar;
	AVRational fr = av_guess_frame_rate(is->ic, is->video_st, NULL);
	AVDictionaryEntry *e = NULL;

	while ((e = av_dict_get(sws_dict, "", e, AV_DICT_IGNORE_SUFFIX))) {
		if (!strcmp(e->key, "sws_flags")) {
			av_strlcatf(sws_flags_str, sizeof(sws_flags_str), "%s=%s:", "flags", e->value);
		} else
			av_strlcatf(sws_flags_str, sizeof(sws_flags_str), "%s=%s:", e->key, e->value);
	}
	if (strlen(sws_flags_str))
		sws_flags_str[strlen(sws_flags_str) - 1] = '\0';

	graph->scale_sws_opts = av_strdup(sws_flags_str);

	snprintf(buffersrc_args, sizeof(buffersrc_args),
	         "video_size=%dx%d:pix_fmt=%d:time_base=%d/%d:pixel_aspect=%d/%d",
	         frame->width, frame->height, frame->format,
	         is->video_st->time_base.num, is->video_st->time_base.den,
	         codecpar->sample_aspect_ratio.num, FFMAX(codecpar->sample_aspect_ratio.den, 1));
	if (fr.num && fr.den)
		av_strlcatf(buffersrc_args, sizeof(buffersrc_args), ":frame_rate=%d/%d", fr.num,
		            fr.den);

	if ((ret = avfilter_graph_create_filter(&filt_src,
	                                        avfilter_get_by_name("buffer"),
	                                        "ffplay_buffer", buffersrc_args, NULL,
	                                        graph)) < 0)
		goto fail;

	ret = avfilter_graph_create_filter(&filt_out,
	                                   avfilter_get_by_name("buffersink"),
	                                   "ffplay_buffersink", NULL, NULL, graph);
	if (ret < 0)
		goto fail;

	if ((ret = av_opt_set_int_list(filt_out, "pix_fmts", pix_fmts,  AV_PIX_FMT_NONE,
	                               AV_OPT_SEARCH_CHILDREN)) < 0)
		goto fail;

	last_filter = filt_out;

	if ((ret = configure_filtergraph(graph, vfilters, filt_src, last_filter)) < 0)
		goto fail;

	is->in_video_filter = filt_src;
	is->out_video_filter = filt_out;

fail:
	return ret;
}

static int configure_audio_filters(VideoState *is, const char *afilters,
                                   int force_output_format)
{
	static const enum AVSampleFormat sample_fmts[] = { AV_SAMPLE_FMT_S16, AV_SAMPLE_FMT_NONE };
	int sample_rates[2] = { 0, -1 };
	int64_t channel_layouts[2] = { 0, -1 };
	int channels[2] = { 0, -1 };
	AVFilterContext *filt_asrc = NULL, *filt_asink = NULL;
	char aresample_swr_opts[512] = "";
	AVDictionaryEntry *e = NULL;
	char asrc_args[256];
	int ret;

	avfilter_graph_free(&is->agraph);
	if (!(is->agraph = avfilter_graph_alloc()))
		return AVERROR(ENOMEM);

	while ((e = av_dict_get(swr_opts, "", e, AV_DICT_IGNORE_SUFFIX)))
		av_strlcatf(aresample_swr_opts, sizeof(aresample_swr_opts), "%s=%s:", e->key,
		            e->value);
	if (strlen(aresample_swr_opts))
		aresample_swr_opts[strlen(aresample_swr_opts) - 1] = '\0';
	av_opt_set(is->agraph, "aresample_swr_opts", aresample_swr_opts, 0);

	ret = snprintf(asrc_args, sizeof(asrc_args),
	               "sample_rate=%d:sample_fmt=%s:channels=%d:time_base=%d/%d",
	               is->audio_filter_src.freq, av_get_sample_fmt_name(is->audio_filter_src.fmt),
	               is->audio_filter_src.channels,
	               1, is->audio_filter_src.freq);
	if (is->audio_filter_src.channel_layout)
		snprintf(asrc_args + ret, sizeof(asrc_args) - ret,
		         ":channel_layout=0x%"PRIx64,  is->audio_filter_src.channel_layout);

	ret = avfilter_graph_create_filter(&filt_asrc,
	                                   avfilter_get_by_name("abuffer"), "ffplay_abuffer",
	                                   asrc_args, NULL, is->agraph);
	if (ret < 0)
		goto end;


	ret = avfilter_graph_create_filter(&filt_asink,
	                                   avfilter_get_by_name("abuffersink"), "ffplay_abuffersink",
	                                   NULL, NULL, is->agraph);
	if (ret < 0)
		goto end;

	if ((ret = av_opt_set_int_list(filt_asink, "sample_fmts", sample_fmts,
	                               AV_SAMPLE_FMT_NONE, AV_OPT_SEARCH_CHILDREN)) < 0)
		goto end;
	if ((ret = av_opt_set_int(filt_asink, "all_channel_counts", 1,
	                          AV_OPT_SEARCH_CHILDREN)) < 0)
		goto end;

	if (force_output_format) {
		channel_layouts[0] = is->audio_tgt.channel_layout;
		channels       [0] = is->audio_tgt.channels;
		sample_rates   [0] = is->audio_tgt.freq;
		if ((ret = av_opt_set_int(filt_asink, "all_channel_counts", 0,
		                          AV_OPT_SEARCH_CHILDREN)) < 0)
			goto end;
		if ((ret = av_opt_set_int_list(filt_asink, "channel_layouts", channel_layouts,
		                               -1, AV_OPT_SEARCH_CHILDREN)) < 0)
			goto end;
		if ((ret = av_opt_set_int_list(filt_asink, "channel_counts" , channels       ,
		                               -1, AV_OPT_SEARCH_CHILDREN)) < 0)
			goto end;
		if ((ret = av_opt_set_int_list(filt_asink, "sample_rates"   , sample_rates   ,
		                               -1, AV_OPT_SEARCH_CHILDREN)) < 0)
			goto end;
	}


	if ((ret = configure_filtergraph(is->agraph, afilters, filt_asrc,
	                                 filt_asink)) < 0)
		goto end;

	is->in_audio_filter = filt_asrc;
	is->out_audio_filter = filt_asink;

end:
	if (ret < 0)
		avfilter_graph_free(&is->agraph);
	return ret;
}

static int audio_thread(void *arg)
{
	VideoState *is = arg;
	AVFrame *frame = av_frame_alloc();
	Frame *af;
	int last_serial = -1;
	int64_t dec_channel_layout;
	int reconfigure;
	int got_frame = 0;
	AVRational tb;
	int ret = 0;

	if (!frame)
		return AVERROR(ENOMEM);

	do {
		if ((got_frame = decoder_decode_frame(&is->auddec, frame, NULL)) < 0)
			goto the_end;

		if (got_frame) {
			tb = (AVRational) {
				1, frame->sample_rate
			};

			dec_channel_layout = get_valid_channel_layout(frame->channel_layout,
			                     av_frame_get_channels(frame));

			reconfigure =
			    cmp_audio_fmts(is->audio_filter_src.fmt, is->audio_filter_src.channels,
			                   frame->format, av_frame_get_channels(frame)) ||
			    is->audio_filter_src.channel_layout != dec_channel_layout ||
			    is->audio_filter_src.freq != frame->sample_rate ||
			    is->auddec.pkt_serial != last_serial;

			if (reconfigure) {
				char buf1[1024], buf2[1024];
				av_get_channel_layout_string(buf1, sizeof(buf1), -1,
				                             is->audio_filter_src.channel_layout);
				av_get_channel_layout_string(buf2, sizeof(buf2), -1, dec_channel_layout);
				av_log(NULL, AV_LOG_DEBUG,
				       "Audio frame changed from rate:%d ch:%d fmt:%s layout:%s serial:%d to rate:%d ch:%d fmt:%s layout:%s serial:%d\n",
				       is->audio_filter_src.freq, is->audio_filter_src.channels,
				       av_get_sample_fmt_name(is->audio_filter_src.fmt), buf1, last_serial,
				       frame->sample_rate, av_frame_get_channels(frame),
				       av_get_sample_fmt_name(frame->format), buf2, is->auddec.pkt_serial);

				is->audio_filter_src.fmt = frame->format;
				is->audio_filter_src.channels = av_frame_get_channels(frame);
				is->audio_filter_src.channel_layout = dec_channel_layout;
				is->audio_filter_src.freq = frame->sample_rate;
				last_serial = is->auddec.pkt_serial;

				if ((ret = configure_audio_filters(is, afilters, 1)) < 0)
					goto the_end;
			}

			if ((ret = av_buffersrc_add_frame(is->in_audio_filter, frame)) < 0)
				goto the_end;

			while ((ret = av_buffersink_get_frame_flags(is->out_audio_filter, frame,
			              0)) >= 0) {
				tb = is->out_audio_filter->inputs[0]->time_base;
				if (!(af = frame_queue_peek_writable(&is->sampq)))
					goto the_end;

				af->pts = (frame->pts == AV_NOPTS_VALUE) ? NAN : frame->pts * av_q2d(tb);
				af->pos = av_frame_get_pkt_pos(frame);
				af->serial = is->auddec.pkt_serial;
				af->duration = av_q2d((AVRational) {
					frame->nb_samples, frame->sample_rate
				});

				av_frame_move_ref(af->frame, frame);
				frame_queue_push(&is->sampq);

				if (is->audioq.serial != is->auddec.pkt_serial)
					break;
			}
			if (ret == AVERROR_EOF)
				is->auddec.finished = is->auddec.pkt_serial;
		}
	} while (ret >= 0 || ret == AVERROR(EAGAIN) || ret == AVERROR_EOF);
the_end:
	avfilter_graph_free(&is->agraph);
	av_frame_free(&frame);
	return ret;
}

static int decoder_start(Decoder *d, int (*fn)(void *), void *arg)
{
	packet_queue_start(d->queue);
	d->decoder_tid = SDL_CreateThread(fn, "decoder", arg);
	if (!d->decoder_tid) {
		av_log(NULL, AV_LOG_ERROR, "SDL_CreateThread(): %s\n", SDL_GetError());
		return AVERROR(ENOMEM);
	}
	return 0;
}

static int video_thread(void *arg)
{
	VideoState *is = arg;
	AVFrame *frame;
	double pts;
	double duration;
	int ret;
	AVRational tb = is->video_st->time_base;
	AVRational frame_rate = av_guess_frame_rate(is->ic, is->video_st, NULL);
	AVFilterGraph *graph = avfilter_graph_alloc();
	AVFilterContext *filt_out = NULL, *filt_in = NULL;
	int last_w = 0;
	int last_h = 0;
	enum AVPixelFormat last_format = -2;
	int last_serial = -1;
	int last_vfilter_idx = 0;
	if (!graph) {
		return AVERROR(ENOMEM);
	}

	frame = av_frame_alloc();
	if (!frame) {
		avfilter_graph_free(&graph);
		return AVERROR(ENOMEM);
	}

	while (1) {
		ret = get_video_frame(is, frame);
		if (ret < 0)
			goto the_end;
		if (!ret)
			continue;

		if (last_w != frame->width || last_h != frame->height ||
		    last_format != frame->format || last_serial != is->viddec.pkt_serial ||
		    last_vfilter_idx != is->vfilter_idx) {
			av_log(NULL, AV_LOG_DEBUG,
			       "Video frame changed from size:%dx%d format:%s serial:%d to size:%dx%d format:%s serial:%d\n",
			       last_w, last_h,
			       (const char *)av_x_if_null(av_get_pix_fmt_name(last_format), "none"),
			       last_serial, frame->width, frame->height,
			       (const char *)av_x_if_null(av_get_pix_fmt_name(frame->format), "none"),
			       is->viddec.pkt_serial);
			avfilter_graph_free(&graph);
			graph = avfilter_graph_alloc();
			if ((ret = configure_video_filters(graph, is,
			                                   vfilters_list ? vfilters_list[is->vfilter_idx] : NULL, frame)) < 0) {
				SDL_Event event;
				event.type = FF_QUIT_EVENT;
				event.user.data1 = is;
				SDL_PushEvent(&event);
				goto the_end;
			}
			filt_in = is->in_video_filter;
			filt_out = is->out_video_filter;
			last_w = frame->width;
			last_h = frame->height;
			last_format = frame->format;
			last_serial = is->viddec.pkt_serial;
			last_vfilter_idx = is->vfilter_idx;
			frame_rate = filt_out->inputs[0]->frame_rate;
		}

		ret = av_buffersrc_add_frame(filt_in, frame);
		if (ret < 0)
			goto the_end;

		while (ret >= 0) {
			is->frame_last_returned_time = av_gettime_relative() / 1000000.0;

			ret = av_buffersink_get_frame_flags(filt_out, frame, 0);
			if (ret < 0) {
				if (ret == AVERROR_EOF)
					is->viddec.finished = is->viddec.pkt_serial;
				ret = 0;
				break;
			}

			is->frame_last_filter_delay = av_gettime_relative() / 1000000.0 -
			                              is->frame_last_returned_time;
			if (fabs(is->frame_last_filter_delay) > AV_NOSYNC_THRESHOLD / 10.0)
				is->frame_last_filter_delay = 0;
			tb = filt_out->inputs[0]->time_base;
			duration = (frame_rate.num && frame_rate.den ? av_q2d((AVRational) {
				frame_rate.den, frame_rate.num
			}) : 0);
			pts = (frame->pts == AV_NOPTS_VALUE) ? NAN : frame->pts * av_q2d(tb);
			ret = queue_picture(is, frame, pts, duration, av_frame_get_pkt_pos(frame),
			                    is->viddec.pkt_serial);
			av_frame_unref(frame);
		}

		if (ret < 0)
			goto the_end;
	}
the_end:
	avfilter_graph_free(&graph);
	av_frame_free(&frame);
	return 0;
}

static int subtitle_thread(void *arg)
{
	VideoState *is = arg;
	Frame *sp;
	int got_subtitle;
	double pts;

	while (1) {
		if (!(sp = frame_queue_peek_writable(&is->subpq)))
			return 0;

		if ((got_subtitle = decoder_decode_frame(&is->subdec, NULL, &sp->sub)) < 0)
			break;

		pts = 0;

		if (got_subtitle && sp->sub.format == 0) {
			if (sp->sub.pts != AV_NOPTS_VALUE)
				pts = sp->sub.pts / (double)AV_TIME_BASE;
			sp->pts = pts;
			sp->serial = is->subdec.pkt_serial;
			sp->width = is->subdec.avctx->width;
			sp->height = is->subdec.avctx->height;
			sp->uploaded = 0;

			/* now we can update the picture count */
			frame_queue_push(&is->subpq);
		} else if (got_subtitle) {
			avsubtitle_free(&sp->sub);
		}
	}
	return 0;
}

/* copy samples for viewing in editor window */
static void update_sample_display(VideoState *is, short *samples,
                                  int samples_size)
{
	int size, len;

	size = samples_size / sizeof(short);
	while (size > 0) {
		len = SAMPLE_ARRAY_SIZE - is->sample_array_index;
		if (len > size)
			len = size;
		memcpy(is->sample_array + is->sample_array_index, samples, len * sizeof(short));
		samples += len;
		is->sample_array_index += len;
		if (is->sample_array_index >= SAMPLE_ARRAY_SIZE)
			is->sample_array_index = 0;
		size -= len;
	}
}

/* return the wanted number of samples to get better sync if sync_type is video
 * or external master clock */
static int synchronize_audio(VideoState *is, int nb_samples)
{
	return nb_samples;
}

/**
 * Decode one audio frame and return its uncompressed size.
 *
 * The processed audio frame is decoded, converted if required, and
 * stored in is->audio_buf, with size in bytes given by the return
 * value.
 */
static int audio_decode_frame(VideoState *is)
{
	int data_size, resampled_data_size;
	int64_t dec_channel_layout;
	av_unused double audio_clock0;
	int wanted_nb_samples;
	Frame *af;

	do {
		if (!(af = frame_queue_peek_readable(&is->sampq)))
			return -1;
		frame_queue_next(&is->sampq);
	} while (af->serial != is->audioq.serial);

	data_size = av_samples_get_buffer_size(NULL, av_frame_get_channels(af->frame),
	                                       af->frame->nb_samples,
	                                       af->frame->format, 1);

	dec_channel_layout =
	    (af->frame->channel_layout &&
	     av_frame_get_channels(af->frame) == av_get_channel_layout_nb_channels(
	         af->frame->channel_layout)) ?
	    af->frame->channel_layout : av_get_default_channel_layout(av_frame_get_channels(
	                af->frame));
	wanted_nb_samples = synchronize_audio(is, af->frame->nb_samples);

	if (af->frame->format != is->audio_src.fmt ||
	    dec_channel_layout != is->audio_src.channel_layout ||
	    af->frame->sample_rate != is->audio_src.freq ||
	    (wanted_nb_samples != af->frame->nb_samples && !is->swr_ctx)) {
		swr_free(&is->swr_ctx);
		is->swr_ctx = swr_alloc_set_opts(NULL,
		                                 is->audio_tgt.channel_layout, is->audio_tgt.fmt, is->audio_tgt.freq,
		                                 dec_channel_layout,           af->frame->format, af->frame->sample_rate,
		                                 0, NULL);
		if (!is->swr_ctx || swr_init(is->swr_ctx) < 0) {
			av_log(NULL, AV_LOG_ERROR,
			       "Cannot create sample rate converter for conversion of %d Hz %s %d channels to %d Hz %s %d channels!\n",
			       af->frame->sample_rate, av_get_sample_fmt_name(af->frame->format),
			       av_frame_get_channels(af->frame),
			       is->audio_tgt.freq, av_get_sample_fmt_name(is->audio_tgt.fmt),
			       is->audio_tgt.channels);
			swr_free(&is->swr_ctx);
			return -1;
		}
		is->audio_src.channel_layout = dec_channel_layout;
		is->audio_src.channels = av_frame_get_channels(af->frame);
		is->audio_src.freq = af->frame->sample_rate;
		is->audio_src.fmt = af->frame->format;
	}

	if (is->swr_ctx) {
		const uint8_t **in = (const uint8_t **)af->frame->extended_data;
		uint8_t **out = &is->audio_buf1;
		int out_count = (int64_t)wanted_nb_samples * is->audio_tgt.freq /
		                af->frame->sample_rate + 256;
		int out_size = av_samples_get_buffer_size(NULL, is->audio_tgt.channels,
		               out_count, is->audio_tgt.fmt, 0);
		int len2;
		if (out_size < 0) {
			av_log(NULL, AV_LOG_ERROR, "av_samples_get_buffer_size() failed\n");
			return -1;
		}
		if (wanted_nb_samples != af->frame->nb_samples) {
			if (swr_set_compensation(is->swr_ctx,
			                         (wanted_nb_samples - af->frame->nb_samples) * is->audio_tgt.freq /
			                         af->frame->sample_rate,
			                         wanted_nb_samples * is->audio_tgt.freq / af->frame->sample_rate) < 0) {
				av_log(NULL, AV_LOG_ERROR, "swr_set_compensation() failed\n");
				return -1;
			}
		}
		av_fast_malloc(&is->audio_buf1, &is->audio_buf1_size, out_size);
		if (!is->audio_buf1)
			return AVERROR(ENOMEM);
		len2 = swr_convert(is->swr_ctx, out, out_count, in, af->frame->nb_samples);
		if (len2 < 0) {
			av_log(NULL, AV_LOG_ERROR, "swr_convert() failed\n");
			return -1;
		}
		if (len2 == out_count) {
			av_log(NULL, AV_LOG_WARNING, "audio buffer is probably too small\n");
			if (swr_init(is->swr_ctx) < 0)
				swr_free(&is->swr_ctx);
		}
		is->audio_buf = is->audio_buf1;
		resampled_data_size = len2 * is->audio_tgt.channels * av_get_bytes_per_sample(
		                          is->audio_tgt.fmt);
	} else {
		is->audio_buf = af->frame->data[0];
		resampled_data_size = data_size;
	}

	audio_clock0 = is->audio_clock;
	/* update the audio clock with the pts */
	if (!isnan(af->pts))
		is->audio_clock = af->pts + (double) af->frame->nb_samples /
		                  af->frame->sample_rate;
	else
		is->audio_clock = NAN;
	is->audio_clock_serial = af->serial;
	return resampled_data_size;
}

/* prepare a new audio buffer */
static void sdl_audio_callback(void *opaque, Uint8 *stream, int len)
{
	VideoState *is = opaque;
	int audio_size, len1;

	audio_callback_time = av_gettime_relative();

	while (len > 0) {
		if (is->audio_buf_index >= is->audio_buf_size) {
			audio_size = audio_decode_frame(is);
			if (audio_size < 0) {
				/* if error, just output silence */
				is->audio_buf = NULL;
				is->audio_buf_size = SDL_AUDIO_MIN_BUFFER_SIZE / is->audio_tgt.frame_size *
				                     is->audio_tgt.frame_size;
			} else {
				if (is->show_mode != SHOW_MODE_VIDEO)
					update_sample_display(is, (int16_t *)is->audio_buf, audio_size);
				is->audio_buf_size = audio_size;
			}
			is->audio_buf_index = 0;
		}
		len1 = is->audio_buf_size - is->audio_buf_index;
		if (len1 > len)
			len1 = len;
		if (is->audio_buf && is->audio_volume == SDL_MIX_MAXVOLUME)
			memcpy(stream, (uint8_t *)is->audio_buf + is->audio_buf_index, len1);
		else {
			memset(stream, 0, len1);
			if (is->audio_buf)
				SDL_MixAudio(stream, (uint8_t *)is->audio_buf + is->audio_buf_index, len1,
				             is->audio_volume);
		}
		len -= len1;
		stream += len1;
		is->audio_buf_index += len1;
	}
	is->audio_write_buf_size = is->audio_buf_size - is->audio_buf_index;
	/* Let's assume the audio driver that is used by SDL has two periods. */
	if (!isnan(is->audio_clock)) {
		set_clock_at(&is->audclk,
		             is->audio_clock - (double)(2 * is->audio_hw_buf_size +
		                                        is->audio_write_buf_size) / is->audio_tgt.bytes_per_sec, is->audio_clock_serial,
		             audio_callback_time / 1000000.0);
		sync_clock_to_slave(&is->extclk, &is->audclk);
	}
}

static int audio_open(void *opaque, int64_t wanted_channel_layout,
                      int wanted_nb_channels, int wanted_sample_rate,
                      struct AudioParams *audio_hw_params)
{
	SDL_AudioSpec wanted_spec, spec;
	const char *env;
	static const int next_nb_channels[] = {0, 0, 1, 6, 2, 6, 4, 6};
	static const int next_sample_rates[] = {0, 44100, 48000, 96000, 192000};
	int next_sample_rate_idx = FF_ARRAY_ELEMS(next_sample_rates) - 1;

	env = SDL_getenv("SDL_AUDIO_CHANNELS");
	if (env) {
		wanted_nb_channels = atoi(env);
		wanted_channel_layout = av_get_default_channel_layout(wanted_nb_channels);
	}
	if (!wanted_channel_layout ||
	    wanted_nb_channels != av_get_channel_layout_nb_channels(
	        wanted_channel_layout)) {
		wanted_channel_layout = av_get_default_channel_layout(wanted_nb_channels);
		wanted_channel_layout &= ~AV_CH_LAYOUT_STEREO_DOWNMIX;
	}
	wanted_nb_channels = av_get_channel_layout_nb_channels(wanted_channel_layout);
	wanted_spec.channels = wanted_nb_channels;
	wanted_spec.freq = wanted_sample_rate;
	if (wanted_spec.freq <= 0 || wanted_spec.channels <= 0) {
		av_log(NULL, AV_LOG_ERROR, "Invalid sample rate or channel count!\n");
		return -1;
	}
	while (next_sample_rate_idx &&
	       next_sample_rates[next_sample_rate_idx] >= wanted_spec.freq)
		next_sample_rate_idx--;
	wanted_spec.format = AUDIO_S16SYS;
	wanted_spec.silence = 0;
	wanted_spec.samples = FFMAX(SDL_AUDIO_MIN_BUFFER_SIZE,
	                            2 << av_log2(wanted_spec.freq / SDL_AUDIO_MAX_CALLBACKS_PER_SEC));
	wanted_spec.callback = sdl_audio_callback;
	wanted_spec.userdata = opaque;
	while (SDL_OpenAudio(&wanted_spec, &spec) < 0) {
		av_log(NULL, AV_LOG_WARNING, "SDL_OpenAudio (%d channels, %d Hz): %s\n",
		       wanted_spec.channels, wanted_spec.freq, SDL_GetError());
		wanted_spec.channels = next_nb_channels[FFMIN(7, wanted_spec.channels)];
		if (!wanted_spec.channels) {
			wanted_spec.freq = next_sample_rates[next_sample_rate_idx--];
			wanted_spec.channels = wanted_nb_channels;
			if (!wanted_spec.freq) {
				av_log(NULL, AV_LOG_ERROR,
				       "No more combinations to try, audio open failed\n");
				return -1;
			}
		}
		wanted_channel_layout = av_get_default_channel_layout(wanted_spec.channels);
	}
	if (spec.format != AUDIO_S16SYS) {
		av_log(NULL, AV_LOG_ERROR,
		       "SDL advised audio format %d is not supported!\n", spec.format);
		return -1;
	}
	if (spec.channels != wanted_spec.channels) {
		wanted_channel_layout = av_get_default_channel_layout(spec.channels);
		if (!wanted_channel_layout) {
			av_log(NULL, AV_LOG_ERROR,
			       "SDL advised channel count %d is not supported!\n", spec.channels);
			return -1;
		}
	}

	audio_hw_params->fmt = AV_SAMPLE_FMT_S16;
	audio_hw_params->freq = spec.freq;
	audio_hw_params->channel_layout = wanted_channel_layout;
	audio_hw_params->channels = spec.channels;
	audio_hw_params->frame_size = av_samples_get_buffer_size(NULL,
	                              audio_hw_params->channels, 1, audio_hw_params->fmt, 1);
	audio_hw_params->bytes_per_sec = av_samples_get_buffer_size(NULL,
	                                 audio_hw_params->channels, audio_hw_params->freq, audio_hw_params->fmt, 1);
	if (audio_hw_params->bytes_per_sec <= 0 || audio_hw_params->frame_size <= 0) {
		av_log(NULL, AV_LOG_ERROR, "av_samples_get_buffer_size failed\n");
		return -1;
	}
	return spec.size;
}

/* open a given stream. Return 0 if OK */
static int stream_component_open(VideoState *is, int stream_index)
{
	AVFormatContext *ic = is->ic;
	AVCodecContext *avctx;
	AVCodec *codec;
	const char *forced_codec_name = NULL;
	AVDictionary *opts = NULL;
	AVDictionaryEntry *t = NULL;
	int sample_rate, nb_channels;
	int64_t channel_layout;
	int ret = 0;
	int stream_lowres = lowres;

	if (stream_index < 0 || stream_index >= ic->nb_streams)
		return -1;

	avctx = avcodec_alloc_context3(NULL);
	if (!avctx)
		return AVERROR(ENOMEM);

	ret = avcodec_parameters_to_context(avctx, ic->streams[stream_index]->codecpar);
	if (ret < 0)
		goto fail;
	av_codec_set_pkt_timebase(avctx, ic->streams[stream_index]->time_base);

	codec = avcodec_find_decoder(avctx->codec_id);

	switch (avctx->codec_type) {
	case AVMEDIA_TYPE_AUDIO:
		is->last_audio_stream = stream_index;
		forced_codec_name = audio_codec_name;
		break;
	case AVMEDIA_TYPE_SUBTITLE:
		is->last_subtitle_stream = stream_index;
		forced_codec_name = subtitle_codec_name;
		break;
	case AVMEDIA_TYPE_VIDEO :
		is->last_video_stream = stream_index;
		forced_codec_name = video_codec_name;
		break;
	default:
		break;
	}
	if (forced_codec_name)
		codec = avcodec_find_decoder_by_name(forced_codec_name);
	if (!codec) {
		if (forced_codec_name)
			av_log(NULL, AV_LOG_WARNING, "No codec could be found with name '%s'\n",
			       forced_codec_name);
		else
			av_log(NULL, AV_LOG_WARNING, "No codec could be found with id %d\n",
			       avctx->codec_id);
		ret = AVERROR(EINVAL);
		goto fail;
	}

	avctx->codec_id = codec->id;
	if (stream_lowres > av_codec_get_max_lowres(codec)) {
		av_log(avctx, AV_LOG_WARNING,
		       "The maximum value for lowres supported by the decoder is %d\n",
		       av_codec_get_max_lowres(codec));
		stream_lowres = av_codec_get_max_lowres(codec);
	}
	av_codec_set_lowres(avctx, stream_lowres);

	if (fast)
		avctx->flags2 |= AV_CODEC_FLAG2_FAST;

	opts = filter_codec_opts(codec_opts, avctx->codec_id, ic,
	                         ic->streams[stream_index], codec);
	if (!av_dict_get(opts, "threads", NULL, 0))
		av_dict_set(&opts, "threads", "auto", 0);
	if (stream_lowres)
		av_dict_set_int(&opts, "lowres", stream_lowres, 0);
	if (avctx->codec_type == AVMEDIA_TYPE_VIDEO ||
	    avctx->codec_type == AVMEDIA_TYPE_AUDIO)
		av_dict_set(&opts, "refcounted_frames", "1", 0);
	if ((ret = avcodec_open2(avctx, codec, &opts)) < 0) {
		goto fail;
	}
	if ((t = av_dict_get(opts, "", NULL, AV_DICT_IGNORE_SUFFIX))) {
		av_log(NULL, AV_LOG_ERROR, "Option %s not found.\n", t->key);
		ret = AVERROR_OPTION_NOT_FOUND;
		goto fail;
	}

	is->eof = 0;
	ic->streams[stream_index]->discard = AVDISCARD_DEFAULT;
	switch (avctx->codec_type) {
	case AVMEDIA_TYPE_AUDIO: {
		AVFilterLink *link;

		is->audio_filter_src.freq = avctx->sample_rate;
		is->audio_filter_src.channels = avctx->channels;
		is->audio_filter_src.channel_layout = get_valid_channel_layout(
		        avctx->channel_layout, avctx->channels);
		is->audio_filter_src.fmt = avctx->sample_fmt;
		if ((ret = configure_audio_filters(is, afilters, 0)) < 0)
			goto fail;
		link = is->out_audio_filter->inputs[0];
		sample_rate = link->sample_rate;
		nb_channels = avfilter_link_get_channels(link);
		channel_layout = link->channel_layout;
	}

		/* prepare audio output */
	if ((ret = audio_open(is, channel_layout, nb_channels, sample_rate,
	                      &is->audio_tgt)) < 0)
		goto fail;
	is->audio_hw_buf_size = ret;
	is->audio_src = is->audio_tgt;
	is->audio_buf_size = 0;
	is->audio_buf_index = 0;

		/* init averaging filter */
	is->audio_diff_avg_coef = exp(log(0.01) / AUDIO_DIFF_AVG_NB);
	is->audio_diff_avg_count = 0;
		/* since we do not have a precise anough audio FIFO fullness,
		   we correct audio sync only if larger than this threshold */
	is->audio_diff_threshold = (double)(is->audio_hw_buf_size) /
	                           is->audio_tgt.bytes_per_sec;

	is->audio_stream = stream_index;
	is->audio_st = ic->streams[stream_index];

	decoder_init(&is->auddec, avctx, &is->audioq, is->continue_read_thread);
	if ((is->ic->iformat->flags & (AVFMT_NOBINSEARCH | AVFMT_NOGENSEARCH |
	                               AVFMT_NO_BYTE_SEEK)) && !is->ic->iformat->read_seek) {
		is->auddec.start_pts = is->audio_st->start_time;
		is->auddec.start_pts_tb = is->audio_st->time_base;
	}
	if ((ret = decoder_start(&is->auddec, audio_thread, is)) < 0)
		goto out;
	SDL_PauseAudio(0);
	break;
	case AVMEDIA_TYPE_VIDEO:
		is->video_stream = stream_index;
		is->video_st = ic->streams[stream_index];

		decoder_init(&is->viddec, avctx, &is->videoq, is->continue_read_thread);
		if ((ret = decoder_start(&is->viddec, video_thread, is)) < 0)
			goto out;
		is->queue_attachments_req = 1;
		break;
	case AVMEDIA_TYPE_SUBTITLE:
		is->subtitle_stream = stream_index;
		is->subtitle_st = ic->streams[stream_index];

		decoder_init(&is->subdec, avctx, &is->subtitleq, is->continue_read_thread);
		if ((ret = decoder_start(&is->subdec, subtitle_thread, is)) < 0)
			goto out;
		break;
	default:
		break;
	}
	goto out;

fail:
	avcodec_free_context(&avctx);
out:
	av_dict_free(&opts);

	return ret;
}

static int decode_interrupt_cb(void *ctx)
{
	VideoState *is = ctx;
	return is->abort_request;
}

static int stream_has_enough_packets(AVStream *st, int stream_id,
                                     PacketQueue *queue)
{
	return stream_id < 0 || queue->abort_request ||
	       (st->disposition & AV_DISPOSITION_ATTACHED_PIC) ||
	       (queue->nb_packets > MIN_FRAMES && (!queue->duration ||
	               av_q2d(st->time_base) * queue->duration > 1.0));
}

/* this thread gets the stream from the disk or the network */
static int read_thread(void *arg)
{
	VideoState *is = arg;
	AVFormatContext *ic = NULL;
	int err, i, ret;
	int st_index[AVMEDIA_TYPE_NB];
	AVPacket pkt1, *pkt = &pkt1;
	int64_t stream_start_time;
	int pkt_in_play_range = 0;
	AVDictionaryEntry *t;
	AVDictionary **opts;
	int orig_nb_streams;
	SDL_mutex *wait_mutex = SDL_CreateMutex();
	int scan_all_pmts_set = 0;
	int64_t pkt_ts;

	if (!wait_mutex) {
		av_log(NULL, AV_LOG_FATAL, "SDL_CreateMutex(): %s\n", SDL_GetError());
		ret = AVERROR(ENOMEM);
		goto fail;
	}

	memset(st_index, -1, sizeof(st_index));
	is->last_video_stream = is->video_stream = -1;
	is->last_audio_stream = is->audio_stream = -1;
	is->last_subtitle_stream = is->subtitle_stream = -1;
	is->eof = 0;

	ic = avformat_alloc_context();
	if (!ic) {
		av_log(NULL, AV_LOG_FATAL, "Could not allocate context.\n");
		ret = AVERROR(ENOMEM);
		goto fail;
	}
	ic->interrupt_callback.callback = decode_interrupt_cb;
	ic->interrupt_callback.opaque = is;
	if (!av_dict_get(format_opts, "scan_all_pmts", NULL, AV_DICT_MATCH_CASE)) {
		av_dict_set(&format_opts, "scan_all_pmts", "1", AV_DICT_DONT_OVERWRITE);
		scan_all_pmts_set = 1;
	}
	err = avformat_open_input(&ic, is->filename, is->iformat, &format_opts);
	if (err < 0) {
		print_error(is->filename, err);
		ret = -1;
		goto fail;
	}
	if (scan_all_pmts_set)
		av_dict_set(&format_opts, "scan_all_pmts", NULL, AV_DICT_MATCH_CASE);

	if ((t = av_dict_get(format_opts, "", NULL, AV_DICT_IGNORE_SUFFIX))) {
		av_log(NULL, AV_LOG_ERROR, "Option %s not found.\n", t->key);
		ret = AVERROR_OPTION_NOT_FOUND;
		goto fail;
	}
	is->ic = ic;

	if (genpts)
		ic->flags |= AVFMT_FLAG_GENPTS;

	av_format_inject_global_side_data(ic);

	opts = setup_find_stream_info_opts(ic, codec_opts);
	orig_nb_streams = ic->nb_streams;

	err = avformat_find_stream_info(ic, opts);

	for (i = 0; i < orig_nb_streams; i++)
		av_dict_free(&opts[i]);
	av_freep(&opts);

	if (err < 0) {
		av_log(NULL, AV_LOG_WARNING, "%s: could not find codec parameters\n",
		       is->filename);
		ret = -1;
		goto fail;
	}

	if (ic->pb)
		// FIXME hack, ffplay maybe should not use avio_feof() to test for the end
		ic->pb->eof_reached = 0;

	if (seek_by_bytes < 0)
		seek_by_bytes = !!(ic->iformat->flags & AVFMT_TS_DISCONT) &&
		                strcmp("ogg", ic->iformat->name);

	is->max_frame_duration = (ic->iformat->flags & AVFMT_TS_DISCONT) ? 10.0 :
	                         3600.0;

	if (!window_title && (t = av_dict_get(ic->metadata, "title", NULL, 0)))
		window_title = av_asprintf("%s - %s", t->value, input_filename);

	/* if seeking requested, we execute it */
	if (start_time != AV_NOPTS_VALUE) {
		int64_t timestamp;

		timestamp = start_time;
		/* add the stream start time */
		if (ic->start_time != AV_NOPTS_VALUE)
			timestamp += ic->start_time;
		ret = avformat_seek_file(ic, -1, INT64_MIN, timestamp, INT64_MAX, 0);
		if (ret < 0) {
			av_log(NULL, AV_LOG_WARNING, "%s: could not seek to position %0.3f\n",
			       is->filename, (double)timestamp / AV_TIME_BASE);
		}
	}

	for (i = 0; i < ic->nb_streams; i++) {
		AVStream *st = ic->streams[i];
		enum AVMediaType type = st->codecpar->codec_type;
		st->discard = AVDISCARD_ALL;
		if (type >= 0 && wanted_stream_spec[type] && st_index[type] == -1)
			if (avformat_match_stream_specifier(ic, st, wanted_stream_spec[type]) > 0)
				st_index[type] = i;
	}
	for (i = 0; i < AVMEDIA_TYPE_NB; i++) {
		if (wanted_stream_spec[i] && st_index[i] == -1) {
			av_log(NULL, AV_LOG_ERROR, "Stream specifier %s does not match any %s stream\n",
			       wanted_stream_spec[i], av_get_media_type_string(i));
			st_index[i] = INT_MAX;
		}
	}

	st_index[AVMEDIA_TYPE_VIDEO] = av_find_best_stream(ic, AVMEDIA_TYPE_VIDEO,
	                               st_index[AVMEDIA_TYPE_VIDEO], -1, NULL, 0);
	st_index[AVMEDIA_TYPE_AUDIO] = av_find_best_stream(ic, AVMEDIA_TYPE_AUDIO,
	                               st_index[AVMEDIA_TYPE_AUDIO], st_index[AVMEDIA_TYPE_VIDEO], NULL, 0);
	st_index[AVMEDIA_TYPE_SUBTITLE] = av_find_best_stream(ic, AVMEDIA_TYPE_SUBTITLE,
	                                  st_index[AVMEDIA_TYPE_SUBTITLE],
	                                  (st_index[AVMEDIA_TYPE_AUDIO] >= 0 ? st_index[AVMEDIA_TYPE_AUDIO] :
	                                   st_index[AVMEDIA_TYPE_VIDEO]), NULL, 0);

	is->show_mode = show_mode;
	if (st_index[AVMEDIA_TYPE_VIDEO] >= 0) {
		AVStream *st = ic->streams[st_index[AVMEDIA_TYPE_VIDEO]];
		AVCodecParameters *codecpar = st->codecpar;
		AVRational sar = av_guess_sample_aspect_ratio(ic, st, NULL);
		if (codecpar->width)
			set_default_window_size(codecpar->width, codecpar->height, sar);
	}

	/* open the streams */
	if (st_index[AVMEDIA_TYPE_AUDIO] >= 0) {
		stream_component_open(is, st_index[AVMEDIA_TYPE_AUDIO]);
	}

	ret = -1;
	if (st_index[AVMEDIA_TYPE_VIDEO] >= 0) {
		ret = stream_component_open(is, st_index[AVMEDIA_TYPE_VIDEO]);
	}
	if (is->show_mode == SHOW_MODE_NONE)
		is->show_mode = ret >= 0 ? SHOW_MODE_VIDEO : SHOW_MODE_RDFT;

	if (st_index[AVMEDIA_TYPE_SUBTITLE] >= 0) {
		stream_component_open(is, st_index[AVMEDIA_TYPE_SUBTITLE]);
	}

	if (is->video_stream < 0 && is->audio_stream < 0) {
		av_log(NULL, AV_LOG_FATAL,
		       "Failed to open file '%s' or configure filtergraph\n",
		       is->filename);
		ret = -1;
		goto fail;
	}

	while (1) {
		if (is->abort_request)
			break;

		if (is->seek_req) {
			int64_t seek_target = is->seek_pos;
			int64_t seek_min = is->seek_rel > 0 ? seek_target - is->seek_rel + 2 :
			                   INT64_MIN;
			int64_t seek_max = is->seek_rel < 0 ? seek_target - is->seek_rel - 2 :
			                   INT64_MAX;
			// FIXME the +-2 is due to rounding being not done in the correct direction in generation
			//      of the seek_pos/seek_rel variables

			ret = avformat_seek_file(is->ic, -1, seek_min, seek_target, seek_max,
			                         is->seek_flags);
			if (ret < 0) {
				av_log(NULL, AV_LOG_ERROR,
				       "%s: error while seeking\n", is->ic->filename);
			} else {
				if (is->audio_stream >= 0) {
					packet_queue_flush(&is->audioq);
					packet_queue_put(&is->audioq, &flush_pkt);
				}
				if (is->subtitle_stream >= 0) {
					packet_queue_flush(&is->subtitleq);
					packet_queue_put(&is->subtitleq, &flush_pkt);
				}
				if (is->video_stream >= 0) {
					packet_queue_flush(&is->videoq);
					packet_queue_put(&is->videoq, &flush_pkt);
				}
				if (is->seek_flags & AVSEEK_FLAG_BYTE) {
					set_clock(&is->extclk, NAN, 0);
				} else {
					set_clock(&is->extclk, seek_target / (double)AV_TIME_BASE, 0);
				}
			}
			is->seek_req = 0;
			is->queue_attachments_req = 1;
			is->eof = 0;
		}
		if (is->queue_attachments_req) {
			if (is->video_st && is->video_st->disposition & AV_DISPOSITION_ATTACHED_PIC) {
				AVPacket copy;
				if ((ret = av_copy_packet(&copy, &is->video_st->attached_pic)) < 0)
					goto fail;
				packet_queue_put(&is->videoq, &copy);
				packet_queue_put_nullpacket(&is->videoq, is->video_stream);
			}
			is->queue_attachments_req = 0;
		}

		/* if the queue are full, no need to read more */
		if (infinite_buffer < 1 &&
		    (is->audioq.size + is->videoq.size + is->subtitleq.size > MAX_QUEUE_SIZE ||
		     (stream_has_enough_packets(is->audio_st, is->audio_stream, &is->audioq) &&
		      stream_has_enough_packets(is->video_st, is->video_stream, &is->videoq) &&
		      stream_has_enough_packets(is->subtitle_st, is->subtitle_stream,
		                                &is->subtitleq)))) {
			/* wait 10 ms */
			SDL_LockMutex(wait_mutex);
			SDL_CondWaitTimeout(is->continue_read_thread, wait_mutex, 10);
			SDL_UnlockMutex(wait_mutex);
			continue;
		}
		if ((!is->audio_st || (is->auddec.finished == is->audioq.serial &&
		                       frame_queue_nb_remaining(&is->sampq) == 0)) &&
		    (!is->video_st || (is->viddec.finished == is->videoq.serial &&
		                       frame_queue_nb_remaining(&is->pictq) == 0))) {
			if (loop != 1 && (!loop || --loop)) {
				stream_seek(is, start_time != AV_NOPTS_VALUE ? start_time : 0, 0, 0);
			} else if (autoexit) {
				ret = AVERROR_EOF;
				goto fail;
			}
		}
		ret = av_read_frame(ic, pkt);
		if (ret < 0) {
			if ((ret == AVERROR_EOF || avio_feof(ic->pb)) && !is->eof) {
				if (is->video_stream >= 0)
					packet_queue_put_nullpacket(&is->videoq, is->video_stream);
				if (is->audio_stream >= 0)
					packet_queue_put_nullpacket(&is->audioq, is->audio_stream);
				if (is->subtitle_stream >= 0)
					packet_queue_put_nullpacket(&is->subtitleq, is->subtitle_stream);
				is->eof = 1;
			}
			if (ic->pb && ic->pb->error)
				break;
			SDL_LockMutex(wait_mutex);
			SDL_CondWaitTimeout(is->continue_read_thread, wait_mutex, 10);
			SDL_UnlockMutex(wait_mutex);
			continue;
		} else {
			is->eof = 0;
		}
		/* check if packet is in play range specified by user, then queue, otherwise discard */
		stream_start_time = ic->streams[pkt->stream_index]->start_time;
		pkt_ts = pkt->pts == AV_NOPTS_VALUE ? pkt->dts : pkt->pts;
		pkt_in_play_range = duration == AV_NOPTS_VALUE ||
		                    (pkt_ts - (stream_start_time != AV_NOPTS_VALUE ? stream_start_time : 0)) *
		                    av_q2d(ic->streams[pkt->stream_index]->time_base) -
		                    (double)(start_time != AV_NOPTS_VALUE ? start_time : 0) / 1000000
		                    <= ((double)duration / 1000000);
		if (pkt->stream_index == is->audio_stream && pkt_in_play_range) {
			packet_queue_put(&is->audioq, pkt);
		} else if (pkt->stream_index == is->video_stream && pkt_in_play_range
		           && !(is->video_st->disposition & AV_DISPOSITION_ATTACHED_PIC)) {
			packet_queue_put(&is->videoq, pkt);
		} else if (pkt->stream_index == is->subtitle_stream && pkt_in_play_range) {
			packet_queue_put(&is->subtitleq, pkt);
		} else {
			av_packet_unref(pkt);
		}
	}

	ret = 0;
fail:
	if (ic && !is->ic)
		avformat_close_input(&ic);

	if (ret != 0) {
		SDL_Event event;

		event.type = FF_QUIT_EVENT;
		event.user.data1 = is;
		SDL_PushEvent(&event);
	}
	SDL_DestroyMutex(wait_mutex);
	return 0;
}

static VideoState *stream_open(const char *filename, AVInputFormat *iformat)
{
	VideoState *is;

	is = av_mallocz(sizeof(VideoState));
	if (!is)
		return NULL;
	is->filename = av_strdup(filename);
	if (!is->filename)
		goto fail;
	is->iformat = iformat;

	/* start video display */
	if (frame_queue_init(&is->pictq, &is->videoq, VIDEO_PICTURE_QUEUE_SIZE, 1) < 0)
		goto fail;
	if (frame_queue_init(&is->subpq, &is->subtitleq, SUBPICTURE_QUEUE_SIZE, 0) < 0)
		goto fail;
	if (frame_queue_init(&is->sampq, &is->audioq, SAMPLE_QUEUE_SIZE, 1) < 0)
		goto fail;

	if (packet_queue_init(&is->videoq) < 0 ||
	    packet_queue_init(&is->audioq) < 0 ||
	    packet_queue_init(&is->subtitleq) < 0)
		goto fail;

	if (!(is->continue_read_thread = SDL_CreateCond())) {
		av_log(NULL, AV_LOG_FATAL, "SDL_CreateCond(): %s\n", SDL_GetError());
		goto fail;
	}

	init_clock(&is->vidclk, &is->videoq.serial);
	init_clock(&is->audclk, &is->audioq.serial);
	init_clock(&is->extclk, &is->extclk.serial);
	is->audio_clock_serial = -1;
	is->audio_volume = SDL_MIX_MAXVOLUME;
	is->av_sync_type = av_sync_type;
	is->read_tid = SDL_CreateThread(read_thread, "read_thread", is);
	if (!is->read_tid) {
		av_log(NULL, AV_LOG_FATAL, "SDL_CreateThread(): %s\n", SDL_GetError());
fail:
		return NULL;
	}
	return is;
}

static void refresh_loop_wait_event(VideoState *is, SDL_Event *event)
{
	double remaining_time = 0.0;
	SDL_PumpEvents();
	while (!SDL_PeepEvents(event, 1, SDL_GETEVENT, SDL_FIRSTEVENT, SDL_LASTEVENT)) {
		if (remaining_time > 0.0)
			av_usleep((int64_t)(remaining_time * 1000000.0));
		remaining_time = REFRESH_RATE;
		if (is->show_mode != SHOW_MODE_NONE)
			video_refresh(is, &remaining_time);
		SDL_PumpEvents();
	}
}

/* handle an event sent by the GUI */
static void event_loop(VideoState *cur_stream)
{
	SDL_Event event;
	double incr, pos;

	while (1) {
		refresh_loop_wait_event(cur_stream, &event);
		switch (event.type) {
		case SDL_KEYDOWN:
			switch (event.key.keysym.sym) {
			case SDLK_UP:
				update_volume(cur_stream, 1, SDL_VOLUME_STEP);
				break;
			case SDLK_DOWN:
				update_volume(cur_stream, -1, SDL_VOLUME_STEP);
				break;
			case SDLK_LEFT:
				incr = -10.0;
				goto do_seek;
			case SDLK_RIGHT:
				incr = 10.0;
				goto do_seek;
do_seek:
				if (seek_by_bytes) {
					pos = -1;
					if (pos < 0 && cur_stream->video_stream >= 0)
						pos = frame_queue_last_pos(&cur_stream->pictq);
					if (pos < 0 && cur_stream->audio_stream >= 0)
						pos = frame_queue_last_pos(&cur_stream->sampq);
					if (pos < 0)
						pos = avio_tell(cur_stream->ic->pb);
					if (cur_stream->ic->bit_rate)
						incr *= cur_stream->ic->bit_rate / 8.0;
					else
						incr *= 180000.0;
					pos += incr;
					stream_seek(cur_stream, pos, incr, 1);
				} else {
					pos = get_master_clock(cur_stream);
					if (isnan(pos))
						pos = (double)cur_stream->seek_pos / AV_TIME_BASE;
					pos += incr;
					if (cur_stream->ic->start_time != AV_NOPTS_VALUE &&
					    pos < cur_stream->ic->start_time / (double)AV_TIME_BASE)
						pos = cur_stream->ic->start_time / (double)AV_TIME_BASE;
					stream_seek(cur_stream, (int64_t)(pos * AV_TIME_BASE),
					            (int64_t)(incr * AV_TIME_BASE), 0);
				}
				break;
			default:
				break;
			}
			break;
		case SDL_WINDOWEVENT:
			switch (event.window.event) {
			case SDL_WINDOWEVENT_RESIZED:
				screen_width = cur_stream->width = event.window.data1;
				screen_height = cur_stream->height = event.window.data2;
				if (cur_stream->vis_texture) {
					SDL_DestroyTexture(cur_stream->vis_texture);
					cur_stream->vis_texture = NULL;
				}
			case SDL_WINDOWEVENT_EXPOSED:
				cur_stream->force_refresh = 1;
			}
			break;
		case SDL_QUIT:
		case FF_QUIT_EVENT:
			do_exit(cur_stream);
			break;
		case FF_ALLOC_EVENT:
			alloc_picture(event.user.data1);
			break;
		default:
			break;
		}
	}
}

static int lockmgr(void **mtx, enum AVLockOp op)
{
	switch (op) {
	case AV_LOCK_CREATE:
		*mtx = SDL_CreateMutex();
		if (!*mtx) {
			av_log(NULL, AV_LOG_FATAL, "SDL_CreateMutex(): %s\n", SDL_GetError());
			return 1;
		}
		return 0;
	case AV_LOCK_OBTAIN:
		return !!SDL_LockMutex(*mtx);
	case AV_LOCK_RELEASE:
		return !!SDL_UnlockMutex(*mtx);
	case AV_LOCK_DESTROY:
		SDL_DestroyMutex(*mtx);
		return 0;
	}
	return 1;
}

/* Called from the main */
int main(int argc, char **argv)
{
	int flags;
	VideoState *is;

	av_log_set_flags(AV_LOG_SKIP_REPEATED);

	/* register all codecs, demux and protocols */
	avdevice_register_all();
	avfilter_register_all();

	av_register_all();
	avformat_network_init();

	init_opts();

	signal(SIGINT , sigterm_handler); /* Interrupt (ANSI).    */
	signal(SIGTERM, sigterm_handler); /* Termination (ANSI).  */

	if (argc < 2) {
		av_log(NULL, AV_LOG_FATAL, "An input file must be specified\n");
		return -1;
	}
	input_filename = argv[1];
	if (!input_filename) {
		av_log(NULL, AV_LOG_FATAL, "An input file must be specified\n");
		exit(1);
	}

	flags = SDL_INIT_VIDEO | SDL_INIT_AUDIO | SDL_INIT_TIMER;

	/* Try to work around an occasional ALSA buffer underflow issue when the
	 * period size is NPOT due to ALSA resampling by forcing the buffer size. */
	if (!SDL_getenv("SDL_AUDIO_ALSA_SET_BUFFER_SIZE"))
		SDL_setenv("SDL_AUDIO_ALSA_SET_BUFFER_SIZE", "1", 1);

	if (SDL_Init(flags)) {
		av_log(NULL, AV_LOG_FATAL, "Could not initialize SDL - %s\n", SDL_GetError());
		av_log(NULL, AV_LOG_FATAL, "(Did you set the DISPLAY variable?)\n");
		exit(1);
	}

	SDL_EventState(SDL_SYSWMEVENT, SDL_IGNORE);
	SDL_EventState(SDL_USEREVENT, SDL_IGNORE);

	if (av_lockmgr_register(lockmgr)) {
		av_log(NULL, AV_LOG_FATAL, "Could not initialize lock manager!\n");
		do_exit(NULL);
	}

	av_init_packet(&flush_pkt);
	flush_pkt.data = (uint8_t *)&flush_pkt;

	is = stream_open(input_filename, file_iformat);
	if (!is) {
		av_log(NULL, AV_LOG_FATAL, "Failed to initialize VideoState!\n");
		do_exit(NULL);
	}

	event_loop(is);

	/* never returns */
	return 0;
}
