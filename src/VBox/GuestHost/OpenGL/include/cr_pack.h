/* Copyright (c) 2001, Stanford University
 * All rights reserved.
 *
 * See the file LICENSE.txt for information on redistributing this software.
 */

#ifndef CR_PACK_H
#define CR_PACK_H

#include "cr_compiler.h"
#include "cr_error.h"
#include "cr_protocol.h"
#include "cr_opcodes.h"
#include "cr_endian.h"
#include "state/cr_statetypes.h"
#include "state/cr_currentpointers.h"
#include "state/cr_client.h"
#ifdef CHROMIUM_THREADSAFE
#include "cr_threads.h"
#endif

#include <iprt/types.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct CRPackContext_t CRPackContext;

/**
 * Packer buffer
 */
typedef struct
{
    void          *pack;  /**< the actual storage space/buffer */
    unsigned int   size;  /**< size of pack[] buffer */
    unsigned int   mtu;
    unsigned char *data_start, *data_current, *data_end;
    unsigned char *opcode_start, *opcode_current, *opcode_end;
    GLboolean geometry_only;  /**< just used for debugging */
    GLboolean holds_BeginEnd;
    GLboolean in_BeginEnd;
    GLboolean canBarf;
    GLboolean holds_List;
    GLboolean in_List;
    CRPackContext *context;
} CRPackBuffer;

typedef void (*CRPackFlushFunc)(void *arg);
typedef void (*CRPackSendHugeFunc)(CROpcode, void *);
typedef void (*CRPackErrorHandlerFunc)(int line, const char *file, GLenum error, const char *info);

typedef enum
{
    CRPackBeginEndStateNone = 0, /* not in begin end */
    CRPackBeginEndStateStarted,       /* begin issued */
    CRPackBeginEndStateFlushDone /* begin issued & buffer flash is done thus part of commands is issued to host */
} CRPackBeginEndState;
/**
 * Packer context
 */
struct CRPackContext_t
{
    CRPackBuffer buffer;   /**< not a pointer, see comments in pack_buffer.c */
    CRPackFlushFunc Flush;
    void *flush_arg;
    CRPackSendHugeFunc SendHuge;
    CRPackErrorHandlerFunc Error;
    CRCurrentStatePointers current;
    CRPackBeginEndState enmBeginEndState;
    GLvectorf bounds_min, bounds_max;
    int updateBBOX;
    int swapping;
    CRPackBuffer *currentBuffer;
#ifdef CHROMIUM_THREADSAFE
    CRmutex mutex;
#endif
    char *file;  /**< for debugging only */
    int line;    /**< for debugging only */
};

#if !defined(IN_RING0)
# define CR_PACKER_CONTEXT_ARGSINGLEDECL
# define CR_PACKER_CONTEXT_ARGDECL
# define CR_PACKER_CONTEXT_ARG
# define CR_PACKER_CONTEXT_ARGCTX(C)
# ifdef CHROMIUM_THREADSAFE
extern CRtsd _PackerTSD;
#  define CR_GET_PACKER_CONTEXT(C) CRPackContext *C = (CRPackContext *) crGetTSD(&_PackerTSD)
#  define CR_LOCK_PACKER_CONTEXT(PC) crLockMutex(&((PC)->mutex))
#  define CR_UNLOCK_PACKER_CONTEXT(PC) crUnlockMutex(&((PC)->mutex))
# else
extern DLLDATA(CRPackContext) cr_packer_globals;
#  define CR_GET_PACKER_CONTEXT(C) CRPackContext *C = &cr_packer_globals
#  define CR_LOCK_PACKER_CONTEXT(PC)
#  define CR_UNLOCK_PACKER_CONTEXT(PC)
# endif
#else /* if defined IN_RING0 */
# define CR_PACKER_CONTEXT_ARGSINGLEDECL CRPackContext *_pCtx
# define CR_PACKER_CONTEXT_ARGDECL CR_PACKER_CONTEXT_ARGSINGLEDECL,
# define CR_PACKER_CONTEXT_ARG _pCtx,
# define CR_PACKER_CONTEXT_ARGCTX(C) C,
# define CR_GET_PACKER_CONTEXT(C) CRPackContext *C = _pCtx
# define CR_LOCK_PACKER_CONTEXT(PC)
# define CR_UNLOCK_PACKER_CONTEXT(PC)
#endif

extern DECLEXPORT(CRPackContext *) crPackNewContext(int swapping);
extern DECLEXPORT(void) crPackDeleteContext(CRPackContext *pc);
extern DECLEXPORT(void) crPackSetContext( CRPackContext *pc );
extern DECLEXPORT(CRPackContext *) crPackGetContext( void );

extern DECLEXPORT(void) crPackSetBuffer( CRPackContext *pc, CRPackBuffer *buffer );
extern DECLEXPORT(void) crPackSetBufferDEBUG( const char *file, int line, CRPackContext *pc, CRPackBuffer *buffer );
extern DECLEXPORT(void) crPackReleaseBuffer( CRPackContext *pc );
extern DECLEXPORT(void) crPackResetPointers( CRPackContext *pc );

extern DECLEXPORT(int) crPackMaxOpcodes( int buffer_size );
extern DECLEXPORT(int) crPackMaxData( int buffer_size );
extern DECLEXPORT(void) crPackInitBuffer( CRPackBuffer *buffer, void *buf, int size, int mtu
#ifdef IN_RING0
        , unsigned int num_opcodes
#endif
        );
extern DECLEXPORT(void) crPackFlushFunc( CRPackContext *pc, CRPackFlushFunc ff );
extern DECLEXPORT(void) crPackFlushArg( CRPackContext *pc, void *flush_arg );
extern DECLEXPORT(void) crPackSendHugeFunc( CRPackContext *pc, CRPackSendHugeFunc shf );
extern DECLEXPORT(void) crPackErrorFunction( CRPackContext *pc, CRPackErrorHandlerFunc errf );
extern DECLEXPORT(void) crPackOffsetCurrentPointers( int offset );
extern DECLEXPORT(void) crPackNullCurrentPointers( void );

extern DECLEXPORT(void) crPackResetBoundingBox( CRPackContext *pc );
extern DECLEXPORT(GLboolean) crPackGetBoundingBox( CRPackContext *pc,
                                GLfloat *xmin, GLfloat *ymin, GLfloat *zmin,
                                GLfloat *xmax, GLfloat *ymax, GLfloat *zmax);

extern DECLEXPORT(void) crPackAppendBuffer( CR_PACKER_CONTEXT_ARGDECL const CRPackBuffer *buffer );
extern DECLEXPORT(void) crPackAppendBoundedBuffer( CR_PACKER_CONTEXT_ARGDECL const CRPackBuffer *buffer, const CRrecti *bounds );
extern DECLEXPORT(int) crPackCanHoldBuffer( CR_PACKER_CONTEXT_ARGDECL const CRPackBuffer *buffer );
extern DECLEXPORT(int) crPackCanHoldBoundedBuffer( CR_PACKER_CONTEXT_ARGDECL const CRPackBuffer *buffer );

#if defined(LINUX) || defined(WINDOWS)
#define CR_UNALIGNED_ACCESS_OKAY
#else
#undef CR_UNALIGNED_ACCESS_OKAY
#endif
extern DECLEXPORT(void) crWriteUnalignedDouble( void *buffer, double d );
extern DECLEXPORT(void) crWriteSwappedDouble( void *buffer, double d );

extern DECLEXPORT(void) *crPackAlloc( CR_PACKER_CONTEXT_ARGDECL unsigned int len );
extern DECLEXPORT(void) crHugePacket( CR_PACKER_CONTEXT_ARGDECL CROpcode op, void *ptr );
extern DECLEXPORT(void) crPackFree( CR_PACKER_CONTEXT_ARGDECL void *ptr );
extern DECLEXPORT(void) crNetworkPointerWrite( CRNetworkPointer *, void * );

extern DECLEXPORT(void) crPackExpandDrawArrays(GLenum mode, GLint first, GLsizei count, CRClientState *c);
extern DECLEXPORT(void) crPackExpandDrawArraysSWAP(GLenum mode, GLint first, GLsizei count, CRClientState *c);

extern DECLEXPORT(void) crPackUnrollDrawElements(GLsizei count, GLenum type, const GLvoid *indices);
extern DECLEXPORT(void) crPackUnrollDrawElementsSWAP(GLsizei count, GLenum type, const GLvoid *indices);

extern DECLEXPORT(void) crPackExpandDrawElements(GLenum mode, GLsizei count, GLenum type, const GLvoid *indices, CRClientState *c);
extern DECLEXPORT(void) crPackExpandDrawElementsSWAP(GLenum mode, GLsizei count, GLenum type, const GLvoid *indices, CRClientState *c);

extern DECLEXPORT(void) crPackExpandDrawRangeElements(GLenum mode, GLuint start, GLuint end, GLsizei count, GLenum type, const GLvoid *indices, CRClientState *c);
extern DECLEXPORT(void) crPackExpandDrawRangeElementsSWAP(GLenum mode, GLuint start, GLuint end, GLsizei count, GLenum type, const GLvoid *indices, CRClientState *c);

extern DECLEXPORT(void) crPackExpandArrayElement(GLint index, CRClientState *c);
extern DECLEXPORT(void) crPackExpandArrayElementSWAP(GLint index, CRClientState *c);

extern DECLEXPORT(void) crPackExpandMultiDrawArraysEXT( GLenum mode, GLint *first, GLsizei *count, GLsizei primcount, CRClientState *c );
extern DECLEXPORT(void) crPackExpandMultiDrawArraysEXTSWAP( GLenum mode, GLint *first, GLsizei *count, GLsizei primcount, CRClientState *c );

extern DECLEXPORT(void) crPackExpandMultiDrawElementsEXT( GLenum mode, const GLsizei *count, GLenum type, const GLvoid **indices, GLsizei primcount, CRClientState *c );
extern DECLEXPORT(void) crPackExpandMultiDrawElementsEXTSWAP( GLenum mode, const GLsizei *count, GLenum type, const GLvoid **indices, GLsizei primcount, CRClientState *c );


/**
 * Return number of opcodes in given buffer.
 */
static INLINE int
crPackNumOpcodes(const CRPackBuffer *buffer)
{
    CRASSERT(buffer->opcode_start - buffer->opcode_current >= 0);
    return buffer->opcode_start - buffer->opcode_current;
}


/**
 * Return amount of data (in bytes) in buffer.
 */
static INLINE int
crPackNumData(const CRPackBuffer *buffer)
{
    CRASSERT(buffer->data_current - buffer->data_start >= 0);
    return buffer->data_current - buffer->data_start; /* in bytes */
}


static INLINE int
crPackCanHoldOpcode(const CRPackContext *pc, int num_opcode, int num_data)
{
  int fitsInMTU, opcodesFit, dataFits;

  CRASSERT(pc->currentBuffer);

  fitsInMTU = (((pc->buffer.data_current - pc->buffer.opcode_current - 1
                 + num_opcode + num_data
                 + 0x3 ) & ~0x3) + sizeof(CRMessageOpcodes)
               <= pc->buffer.mtu);
  opcodesFit = (pc->buffer.opcode_current - num_opcode >= pc->buffer.opcode_end);
  dataFits = (pc->buffer.data_current + num_data <= pc->buffer.data_end);

  return fitsInMTU && opcodesFit && dataFits;
}


/**
 * Alloc space for a message of 'len' bytes (plus 1 opcode).
 * Only flush if buffer is full.
 */
#define CR_GET_BUFFERED_POINTER_NO_BEGINEND_FLUSH(pc, len, lock)    \
  do {                                                              \
    THREADASSERT( pc );                                             \
    if (lock) CR_LOCK_PACKER_CONTEXT(pc);                           \
    CRASSERT( pc->currentBuffer );                                  \
    if ( !crPackCanHoldOpcode( pc, 1, (len) ) ) {                   \
      pc->Flush( pc->flush_arg );                                   \
      CRASSERT(crPackCanHoldOpcode( pc, 1, (len) ) );               \
      if (pc->enmBeginEndState == CRPackBeginEndStateStarted) {     \
        pc->enmBeginEndState = CRPackBeginEndStateFlushDone;        \
      }                                                             \
    }                                                               \
    data_ptr = pc->buffer.data_current;                             \
    pc->buffer.data_current += (len);                               \
  } while (0)


/**
 * As above, flush if the buffer contains vertex data and we're
 * no longer inside glBegin/glEnd.
 */
#define CR_GET_BUFFERED_POINTER( pc, len )                          \
  do {                                                              \
    CR_LOCK_PACKER_CONTEXT(pc);                                     \
    CRASSERT( pc->currentBuffer );                                  \
    if ( pc->buffer.holds_BeginEnd && !pc->buffer.in_BeginEnd ) {   \
      CRASSERT( 0 );  /* should never be here currently */          \
      pc->Flush( pc->flush_arg );                                   \
      pc->buffer.holds_BeginEnd = 0;                                \
    }                                                               \
    CR_GET_BUFFERED_POINTER_NO_BEGINEND_FLUSH( pc, len, GL_FALSE ); \
  } while (0)

/**
 * As above, but without lock.
 */
#define CR_GET_BUFFERED_POINTER_NOLOCK( pc, len )                   \
  do {                                                              \
    CRASSERT( pc->currentBuffer );                                  \
    if ( pc->buffer.holds_BeginEnd && !pc->buffer.in_BeginEnd ) {   \
      CRASSERT( 0 );  /* should never be here currently */          \
      pc->Flush( pc->flush_arg );                                   \
      pc->buffer.holds_BeginEnd = 0;                                \
    }                                                               \
    CR_GET_BUFFERED_POINTER_NO_BEGINEND_FLUSH( pc, len, GL_FALSE ); \
  } while (0)


/**
 * As above, but for vertex data between glBegin/End (counts vertices).
 */
#define CR_GET_BUFFERED_COUNT_POINTER( pc, len )        \
  do {                                                  \
    CR_LOCK_PACKER_CONTEXT(pc);                         \
    CRASSERT( pc->currentBuffer );                      \
    if ( !crPackCanHoldOpcode( pc, 1, (len) ) ) {       \
      pc->Flush( pc->flush_arg );                       \
      CRASSERT( crPackCanHoldOpcode( pc, 1, (len) ) );  \
      if (pc->enmBeginEndState == CRPackBeginEndStateStarted) {     \
        pc->enmBeginEndState = CRPackBeginEndStateFlushDone;        \
      }                                                             \
    }                                                   \
    data_ptr = pc->buffer.data_current;                 \
    pc->current.vtx_count++;                            \
    pc->buffer.data_current += (len);                   \
  } while (0)


/**
 * Allocate space for a msg/command that has no arguments, such
 * as glFinish().
 */
#define CR_GET_BUFFERED_POINTER_NO_ARGS( pc )  \
  CR_GET_BUFFERED_POINTER( pc, 4 );            \
  WRITE_DATA( 0, GLuint, 0xdeadbeef )

#define WRITE_DATA( offset, type, data ) \
  *( (type *) (data_ptr + (offset))) = (data)

/* Write data to current location and auto increment */
#define WRITE_DATA_AI(type, data)     \
  {                                   \
      *((type*) (data_ptr)) = (data); \
      data_ptr += sizeof(type);       \
  }

#ifdef CR_UNALIGNED_ACCESS_OKAY
#define WRITE_DOUBLE( offset, data ) \
  WRITE_DATA( offset, GLdouble, data )
#else
#define WRITE_DOUBLE( offset, data ) \
  crWriteUnalignedDouble( data_ptr + (offset), (data) )
#endif

#define WRITE_SWAPPED_DOUBLE( offset, data ) \
    crWriteSwappedDouble( data_ptr + (offset), (data) )

#define WRITE_OPCODE( pc, opcode )  \
  *(pc->buffer.opcode_current--) = (unsigned char) opcode

#define WRITE_NETWORK_POINTER( offset, data ) \
  crNetworkPointerWrite( (CRNetworkPointer *) ( data_ptr + (offset) ), (data) )

#ifdef __cplusplus
}
#endif

#endif /* CR_PACK_H */
