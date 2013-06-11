/*
 * QEMU Audio subsystem header
 *
 * Copyright (c) 2003-2005 Vassili Karpov (malc)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
#ifndef QEMU_AUDIO_H
#define QEMU_AUDIO_H

#include "sys-queue.h"

#if defined __STDC_VERSION__ && __STDC_VERSION__ > 199901L
#define FMTZ "z"
#else
#define FMTZ
#endif

typedef void (*audio_callback_fn_t) (void *opaque, int avail);

typedef enum {
    AUD_FMT_U8,
    AUD_FMT_S8,
    AUD_FMT_U16,
    AUD_FMT_S16,
    AUD_FMT_U32,
    AUD_FMT_S32
} audfmt_e;

#define AUDIO_HOST_ENDIANNESS 0

typedef struct {
    int freq;
    int nchannels;
    audfmt_e fmt;
    int endianness;
} audsettings_t;

typedef enum {
    AUD_CNOTIFY_ENABLE,
    AUD_CNOTIFY_DISABLE
} audcnotification_e;

typedef enum
{
    AUD_MIXER_VOLUME,
    AUD_MIXER_PCM,
    AUD_MIXER_LINE_IN
} audmixerctl_t;

typedef enum
{
    AUD_REC_MIC,
    AUD_REC_CD,
    AUD_REC_VIDEO,
    AUD_REC_AUX,
    AUD_REC_LINE_IN,
    AUD_REC_PHONE
} audrecsource_t;

struct audio_capture_ops {
    void (*notify) (void *opaque, audcnotification_e cmd);
    void (*capture) (void *opaque, void *buf, int size);
    void (*destroy) (void *opaque);
};

struct capture_ops {
    void (*info) (void *opaque);
    void (*destroy) (void *opaque);
};

typedef struct CaptureState {
    void *opaque;
    struct capture_ops ops;
    LIST_ENTRY (CaptureState) entries;
} CaptureState;

typedef struct AudioState AudioState;
typedef struct SWVoiceOut SWVoiceOut;
typedef struct CaptureVoiceOut CaptureVoiceOut;
typedef struct SWVoiceIn SWVoiceIn;

typedef struct QEMUSoundCard {
    AudioState *audio;
    char *name;
    LIST_ENTRY (QEMUSoundCard) entries;
} QEMUSoundCard;

typedef struct QEMUAudioTimeStamp {
    uint64_t old_ts;
} QEMUAudioTimeStamp;

void AUD_vlog (const char *cap, const char *fmt, va_list ap);
void AUD_log (const char *cap, const char *fmt, ...)
#if defined (__GNUC__) && !defined (VBOX) /* VBox: oh, please, shut up. */
    __attribute__ ((__format__ (__printf__, 2, 3)))
#endif
    ;

void AUD_register_card (const char *name, QEMUSoundCard *card);
void AUD_remove_card (QEMUSoundCard *card);

CaptureVoiceOut *AUD_add_capture (
    AudioState *s,
    audsettings_t *as,
    struct audio_capture_ops *ops,
    void *opaque
    );
void AUD_del_capture (CaptureVoiceOut *cap, void *cb_opaque);

SWVoiceOut *AUD_open_out (
    QEMUSoundCard *card,
    SWVoiceOut *sw,
    const char *name,
    void *callback_opaque,
    audio_callback_fn_t callback_fn,
    audsettings_t *settings
    );

void AUD_close_out (QEMUSoundCard *card, SWVoiceOut *sw);
int  AUD_write (SWVoiceOut *sw, void *pcm_buf, int size);
int  AUD_get_buffer_size_out (SWVoiceOut *sw);
void AUD_set_active_out (SWVoiceOut *sw, int on);
int  AUD_is_active_out (SWVoiceOut *sw);

void     AUD_init_time_stamp_out (SWVoiceOut *sw, QEMUAudioTimeStamp *ts);
uint64_t AUD_get_elapsed_usec_out (SWVoiceOut *sw, QEMUAudioTimeStamp *ts);

SWVoiceIn *AUD_open_in (
    QEMUSoundCard *card,
    SWVoiceIn *sw,
    const char *name,
    void *callback_opaque,
    audio_callback_fn_t callback_fn,
    audsettings_t *settings
    );

void AUD_close_in (QEMUSoundCard *card, SWVoiceIn *sw);
int  AUD_read (SWVoiceIn *sw, void *pcm_buf, int size);
void AUD_set_active_in (SWVoiceIn *sw, int on);
int  AUD_is_active_in (SWVoiceIn *sw);

void     AUD_init_time_stamp_in (SWVoiceIn *sw, QEMUAudioTimeStamp *ts);
uint64_t AUD_get_elapsed_usec_in (SWVoiceIn *sw, QEMUAudioTimeStamp *ts);

void AUD_set_volume_out (SWVoiceOut *po, int mute, uint8_t lvol, uint8_t rvol);
void AUD_set_volume (audmixerctl_t mt, int *mute, uint8_t *lvol, uint8_t *rvol);
void AUD_set_record_source (audrecsource_t *ars, audrecsource_t *als);

int  AUD_init_null(void);

static inline void *advance (void *p, int incr)
{
#ifndef VBOX
    uint8_t *d = p;
#else
    uint8_t *d = (uint8_t*)p;
#endif
    return (d + incr);
}

uint32_t popcount (uint32_t u);
uint32_t lsbindex (uint32_t u);

#ifdef __GNUC__
#define audio_MIN(a, b) ( __extension__ ({      \
    __typeof (a) ta = a;                        \
    __typeof (b) tb = b;                        \
    ((ta)>(tb)?(tb):(ta));                      \
 }))

#define audio_MAX(a, b) ( __extension__ ({      \
    __typeof (a) ta = a;                        \
    __typeof (b) tb = b;                        \
    ((ta)<(tb)?(tb):(ta));                      \
 }))
#else
#define audio_MIN(a, b) ((a)>(b)?(b):(a))
#define audio_MAX(a, b) ((a)<(b)?(b):(a))
#endif

#ifdef VBOX
const char *audio_get_stream_name(void);
#endif

int AUD_is_host_voice_in_ok(SWVoiceIn *hw);
int AUD_is_host_voice_out_ok(SWVoiceOut *hw);

#endif  /* audio.h */
