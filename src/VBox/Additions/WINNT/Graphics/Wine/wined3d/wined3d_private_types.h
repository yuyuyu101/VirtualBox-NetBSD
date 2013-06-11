/*
 * Direct3D wine internal header: D3D equivalent types
 *
 * Copyright 2002-2003 Jason Edmeades
 * Copyright 2002-2003 Raphael Junqueira
 * Copyright 2005 Oliver Stieber
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301, USA
 */

/*
 * Oracle LGPL Disclaimer: For the avoidance of doubt, except that if any license choice
 * other than GPL or LGPL is available it will apply instead, Oracle elects to use only
 * the Lesser General Public License version 2.1 (LGPLv2) at this time for any software where
 * a choice of LGPL license versions is made available with the language indicating
 * that LGPLv2 or any later version may be used, or where a choice of which version
 * of the LGPL is applied is otherwise unspecified.
 */

#ifndef __WINE_WINED3D_TYPES_INTERNAL_H
#define __WINE_WINED3D_TYPES_INTERNAL_H

/* WineD3D pixel format flags */
#define WINED3DFMT_FLAG_POSTPIXELSHADER_BLENDING 0x1
#define WINED3DFMT_FLAG_FILTERING                0x2
#define WINED3DFMT_FLAG_DEPTH                    0x4
#define WINED3DFMT_FLAG_STENCIL                  0x8
#define WINED3DFMT_FLAG_RENDERTARGET             0x10

/** DCL usage masks **/
#define WINED3DSP_DCL_USAGE_SHIFT 0
#define WINED3DSP_DCL_USAGE_MASK  0x0000000f
#define WINED3DSP_DCL_USAGEINDEX_SHIFT 16
#define WINED3DSP_DCL_USAGEINDEX_MASK  0x000f0000

/** DCL sampler texture type **/
#define WINED3DSP_TEXTURETYPE_SHIFT 27
#define WINED3DSP_TEXTURETYPE_MASK  0x78000000

typedef enum _WINED3DSAMPLER_TEXTURE_TYPE {
  WINED3DSTT_UNKNOWN      = 0 << WINED3DSP_TEXTURETYPE_SHIFT,
  WINED3DSTT_1D           = 1 << WINED3DSP_TEXTURETYPE_SHIFT,
  WINED3DSTT_2D           = 2 << WINED3DSP_TEXTURETYPE_SHIFT,
  WINED3DSTT_CUBE         = 3 << WINED3DSP_TEXTURETYPE_SHIFT,
  WINED3DSTT_VOLUME       = 4 << WINED3DSP_TEXTURETYPE_SHIFT,

  WINED3DSTT_FORCE_DWORD  = 0x7FFFFFFF
} WINED3DSAMPLER_TEXTURE_TYPE;

/** Register number mask **/
#define WINED3DSP_REGNUM_MASK        0x000007FF

/** Register type masks  **/
#define WINED3DSP_REGTYPE_SHIFT      28
#define WINED3DSP_REGTYPE_SHIFT2     8
#define WINED3DSP_REGTYPE_MASK       (0x7 << WINED3DSP_REGTYPE_SHIFT)
#define WINED3DSP_REGTYPE_MASK2      0x00001800

/** Register types **/
typedef enum _WINED3DSHADER_PARAM_REGISTER_TYPE {
  WINED3DSPR_TEMP         =  0, 
  WINED3DSPR_INPUT        =  1,
  WINED3DSPR_CONST        =  2,
  WINED3DSPR_ADDR         =  3,
  WINED3DSPR_TEXTURE      =  3,
  WINED3DSPR_RASTOUT      =  4,
  WINED3DSPR_ATTROUT      =  5,
  WINED3DSPR_TEXCRDOUT    =  6,
  WINED3DSPR_OUTPUT       =  6,
  WINED3DSPR_CONSTINT     =  7,
  WINED3DSPR_COLOROUT     =  8,
  WINED3DSPR_DEPTHOUT     =  9,
  WINED3DSPR_SAMPLER      = 10,
  WINED3DSPR_CONST2       = 11,
  WINED3DSPR_CONST3       = 12,
  WINED3DSPR_CONST4       = 13,
  WINED3DSPR_CONSTBOOL    = 14,
  WINED3DSPR_LOOP         = 15,
  WINED3DSPR_TEMPFLOAT16  = 16,
  WINED3DSPR_MISCTYPE     = 17,
  WINED3DSPR_LABEL        = 18,
  WINED3DSPR_PREDICATE    = 19,

  WINED3DSPR_FORCE_DWORD  = 0x7FFFFFFF
} WINED3DSHADER_PARAM_REGISTER_TYPE;

/* RASTOUT register offsets */
typedef enum _WINED3DVS_RASTOUT_OFFSETS {
  WINED3DSRO_POSITION     = 0,
  WINED3DSRO_FOG          = 1,
  WINED3DSRO_POINT_SIZE   = 2,

  WINED3DSRO_FORCE_DWORD  = 0x7FFFFFFF
} WINED3DVS_RASTOUT_OFFSETS;

/** Source register modifiers **/
#define WINED3DVS_SWIZZLE_SHIFT      16
#define WINED3DVS_SWIZZLE_MASK       (0xFF << WINED3DVS_SWIZZLE_SHIFT)
#define WINED3DSP_SWIZZLE_SHIFT      16
#define WINED3DSP_SWIZZLE_MASK       (0xFF << WINED3DSP_SWIZZLE_SHIFT)

#define WINED3DVS_X_X       (0 << WINED3DVS_SWIZZLE_SHIFT)
#define WINED3DVS_X_Y       (1 << WINED3DVS_SWIZZLE_SHIFT)
#define WINED3DVS_X_Z       (2 << WINED3DVS_SWIZZLE_SHIFT)
#define WINED3DVS_X_W       (3 << WINED3DVS_SWIZZLE_SHIFT)

#define WINED3DVS_Y_X       (0 << (WINED3DVS_SWIZZLE_SHIFT + 2))
#define WINED3DVS_Y_Y       (1 << (WINED3DVS_SWIZZLE_SHIFT + 2))
#define WINED3DVS_Y_Z       (2 << (WINED3DVS_SWIZZLE_SHIFT + 2))
#define WINED3DVS_Y_W       (3 << (WINED3DVS_SWIZZLE_SHIFT + 2))

#define WINED3DVS_Z_X       (0 << (WINED3DVS_SWIZZLE_SHIFT + 4))
#define WINED3DVS_Z_Y       (1 << (WINED3DVS_SWIZZLE_SHIFT + 4))
#define WINED3DVS_Z_Z       (2 << (WINED3DVS_SWIZZLE_SHIFT + 4))
#define WINED3DVS_Z_W       (3 << (WINED3DVS_SWIZZLE_SHIFT + 4))

#define WINED3DVS_W_X       (0 << (WINED3DVS_SWIZZLE_SHIFT + 6))
#define WINED3DVS_W_Y       (1 << (WINED3DVS_SWIZZLE_SHIFT + 6))
#define WINED3DVS_W_Z       (2 << (WINED3DVS_SWIZZLE_SHIFT + 6))
#define WINED3DVS_W_W       (3 << (WINED3DVS_SWIZZLE_SHIFT + 6))

#define WINED3DVS_NOSWIZZLE (WINED3DVS_X_X | WINED3DVS_Y_Y | WINED3DVS_Z_Z | WINED3DVS_W_W)

#define WINED3DSP_NOSWIZZLE \
    ((0 << (WINED3DSP_SWIZZLE_SHIFT + 0)) | (1 << (WINED3DSP_SWIZZLE_SHIFT + 2)) | \
     (2 << (WINED3DSP_SWIZZLE_SHIFT + 4)) | (3 << (WINED3DSP_SWIZZLE_SHIFT + 6)))

#define WINED3DSP_SRCMOD_SHIFT      24
#define WINED3DSP_SRCMOD_MASK       (0xF << WINED3DSP_SRCMOD_SHIFT)

typedef enum _WINED3DSHADER_PARAM_SRCMOD_TYPE {
  WINED3DSPSM_NONE         =  0 << WINED3DSP_SRCMOD_SHIFT,
  WINED3DSPSM_NEG          =  1 << WINED3DSP_SRCMOD_SHIFT,
  WINED3DSPSM_BIAS         =  2 << WINED3DSP_SRCMOD_SHIFT,
  WINED3DSPSM_BIASNEG      =  3 << WINED3DSP_SRCMOD_SHIFT,
  WINED3DSPSM_SIGN         =  4 << WINED3DSP_SRCMOD_SHIFT,
  WINED3DSPSM_SIGNNEG      =  5 << WINED3DSP_SRCMOD_SHIFT,
  WINED3DSPSM_COMP         =  6 << WINED3DSP_SRCMOD_SHIFT,
  WINED3DSPSM_X2           =  7 << WINED3DSP_SRCMOD_SHIFT,
  WINED3DSPSM_X2NEG        =  8 << WINED3DSP_SRCMOD_SHIFT,
  WINED3DSPSM_DZ           =  9 << WINED3DSP_SRCMOD_SHIFT,
  WINED3DSPSM_DW           = 10 << WINED3DSP_SRCMOD_SHIFT,
  WINED3DSPSM_ABS          = 11 << WINED3DSP_SRCMOD_SHIFT,
  WINED3DSPSM_ABSNEG       = 12 << WINED3DSP_SRCMOD_SHIFT,
  WINED3DSPSM_NOT          = 13 << WINED3DSP_SRCMOD_SHIFT,

  WINED3DSPSM_FORCE_DWORD  = 0x7FFFFFFF
} WINED3DSHADER_PARAM_SRCMOD_TYPE;

/** Destination register modifiers **/
#define WINED3DSP_WRITEMASK_0       0x00010000 /* .x r */
#define WINED3DSP_WRITEMASK_1       0x00020000 /* .y g */
#define WINED3DSP_WRITEMASK_2       0x00040000 /* .z b */
#define WINED3DSP_WRITEMASK_3       0x00080000 /* .w a */
#define WINED3DSP_WRITEMASK_ALL     0x000F0000 /* all */

#define WINED3DSP_DSTMOD_SHIFT      20
#define WINED3DSP_DSTMOD_MASK       (0xF << WINED3DSP_DSTMOD_SHIFT)

typedef enum _WINED3DSHADER_PARAM_DSTMOD_TYPE {
  WINED3DSPDM_NONE             = 0 << WINED3DSP_DSTMOD_SHIFT,
  WINED3DSPDM_SATURATE         = 1 << WINED3DSP_DSTMOD_SHIFT,
  WINED3DSPDM_PARTIALPRECISION = 2 << WINED3DSP_DSTMOD_SHIFT,
  WINED3DSPDM_MSAMPCENTROID    = 4 << WINED3DSP_DSTMOD_SHIFT,

  WINED3DSPDM_FORCE_DWORD  = 0x7FFFFFFF
} WINED3DSHADER_PARAM_DSTMOD_TYPE;

#define WINED3DSP_DSTSHIFT_SHIFT     24
#define WINED3DSP_DSTSHIFT_MASK      (0xF << WINED3DSP_DSTSHIFT_SHIFT)

/** Register addressing modes **/
#define WINED3DSHADER_ADDRESSMODE_SHIFT 13
#define WINED3DSHADER_ADDRESSMODE_MASK  (1 << WINED3DSHADER_ADDRESSMODE_SHIFT)

typedef enum _WINED3DSHADER_ADDRESSMODE_TYPE {
  WINED3DSHADER_ADDRMODE_ABSOLUTE    = 0 << WINED3DSHADER_ADDRESSMODE_SHIFT,
  WINED3DSHADER_ADDRMODE_RELATIVE    = 1 << WINED3DSHADER_ADDRESSMODE_SHIFT,

  WINED3DSHADER_ADDRMODE_FORCE_DWORD = 0x7FFFFFFF
} WINED3DSHADER_ADDRESSMODE_TYPE;

/** Opcode types */
typedef enum _WINED3DSHADER_INSTRUCTION_OPCODE_TYPE {
  WINED3DSIO_NOP          =  0,
  WINED3DSIO_MOV          =  1,
  WINED3DSIO_ADD          =  2,
  WINED3DSIO_SUB          =  3,
  WINED3DSIO_MAD          =  4,
  WINED3DSIO_MUL          =  5,
  WINED3DSIO_RCP          =  6,
  WINED3DSIO_RSQ          =  7,
  WINED3DSIO_DP3          =  8,
  WINED3DSIO_DP4          =  9,
  WINED3DSIO_MIN          = 10,
  WINED3DSIO_MAX          = 11,
  WINED3DSIO_SLT          = 12,
  WINED3DSIO_SGE          = 13,
  WINED3DSIO_EXP          = 14,
  WINED3DSIO_LOG          = 15,
  WINED3DSIO_LIT          = 16,
  WINED3DSIO_DST          = 17,
  WINED3DSIO_LRP          = 18,
  WINED3DSIO_FRC          = 19,
  WINED3DSIO_M4x4         = 20,
  WINED3DSIO_M4x3         = 21,
  WINED3DSIO_M3x4         = 22,
  WINED3DSIO_M3x3         = 23,
  WINED3DSIO_M3x2         = 24,
  WINED3DSIO_CALL         = 25,
  WINED3DSIO_CALLNZ       = 26,
  WINED3DSIO_LOOP         = 27,
  WINED3DSIO_RET          = 28,
  WINED3DSIO_ENDLOOP      = 29,
  WINED3DSIO_LABEL        = 30,
  WINED3DSIO_DCL          = 31,
  WINED3DSIO_POW          = 32,
  WINED3DSIO_CRS          = 33,
  WINED3DSIO_SGN          = 34,
  WINED3DSIO_ABS          = 35,
  WINED3DSIO_NRM          = 36,
  WINED3DSIO_SINCOS       = 37,
  WINED3DSIO_REP          = 38,
  WINED3DSIO_ENDREP       = 39,
  WINED3DSIO_IF           = 40,
  WINED3DSIO_IFC          = 41,
  WINED3DSIO_ELSE         = 42,
  WINED3DSIO_ENDIF        = 43,
  WINED3DSIO_BREAK        = 44,
  WINED3DSIO_BREAKC       = 45,
  WINED3DSIO_MOVA         = 46,
  WINED3DSIO_DEFB         = 47,
  WINED3DSIO_DEFI         = 48,

  WINED3DSIO_TEXCOORD     = 64,
  WINED3DSIO_TEXKILL      = 65,
  WINED3DSIO_TEX          = 66,
  WINED3DSIO_TEXBEM       = 67,
  WINED3DSIO_TEXBEML      = 68,
  WINED3DSIO_TEXREG2AR    = 69,
  WINED3DSIO_TEXREG2GB    = 70,
  WINED3DSIO_TEXM3x2PAD   = 71,
  WINED3DSIO_TEXM3x2TEX   = 72,
  WINED3DSIO_TEXM3x3PAD   = 73,
  WINED3DSIO_TEXM3x3TEX   = 74,
  WINED3DSIO_TEXM3x3DIFF  = 75,
  WINED3DSIO_TEXM3x3SPEC  = 76,
  WINED3DSIO_TEXM3x3VSPEC = 77,
  WINED3DSIO_EXPP         = 78,
  WINED3DSIO_LOGP         = 79,
  WINED3DSIO_CND          = 80,
  WINED3DSIO_DEF          = 81,
  WINED3DSIO_TEXREG2RGB   = 82,
  WINED3DSIO_TEXDP3TEX    = 83,
  WINED3DSIO_TEXM3x2DEPTH = 84,
  WINED3DSIO_TEXDP3       = 85,
  WINED3DSIO_TEXM3x3      = 86,
  WINED3DSIO_TEXDEPTH     = 87,
  WINED3DSIO_CMP          = 88,
  WINED3DSIO_BEM          = 89,
  WINED3DSIO_DP2ADD       = 90,
  WINED3DSIO_DSX          = 91,
  WINED3DSIO_DSY          = 92,
  WINED3DSIO_TEXLDD       = 93,
  WINED3DSIO_SETP         = 94,
  WINED3DSIO_TEXLDL       = 95,
  WINED3DSIO_BREAKP       = 96,

  WINED3DSIO_PHASE        = 0xFFFD,
  WINED3DSIO_COMMENT      = 0xFFFE,
  WINED3DSIO_END          = 0XFFFF,

  WINED3DSIO_FORCE_DWORD  = 0X7FFFFFFF /** for 32-bit alignment */
} WINED3DSHADER_INSTRUCTION_OPCODE_TYPE;

/** opcode-related masks **/

#define WINED3DSI_OPCODE_MASK       0x0000FFFF
#define WINED3DSI_INSTLENGTH_MASK   0x0F000000
#define WINED3DSI_INSTLENGTH_SHIFT  24

#define WINED3DSI_COISSUE 0x40000000

#define WINED3DSI_COMMENTSIZE_SHIFT 16
#define WINED3DSI_COMMENTSIZE_MASK (0x7FFF << WINED3DSI_COMMENTSIZE_SHIFT)
#define WINED3DSHADER_COMMENT(commentSize) \
  ((((commentSize) << WINED3DSI_COMMENTSIZE_SHIFT) & WINED3DSI_COMMENTSIZE_MASK) | WINED3DSIO_COMMENT)

#define WINED3DSHADER_INSTRUCTION_PREDICATED (1 << 28)

/* Undocumented opcode control to identify projective texture lookups in ps 2.0 and later */
#define WINED3DSI_TEXLD_PROJECT 0x00010000
#define WINED3DSI_TEXLD_BIAS    0x00020000

/** Shader version tokens, and shader end tokens **/

#define WINED3DPS_VERSION(major, minor) (0xFFFF0000 | ((major) << 8) | (minor))
#define WINED3DVS_VERSION(major, minor) (0xFFFE0000 | ((major) << 8) | (minor))
#define WINED3DSHADER_VERSION_MAJOR(version) (((version) >> 8) & 0xFF)
#define WINED3DSHADER_VERSION_MINOR(version) (((version) >> 0) & 0xFF)
#define WINED3DPS_END() 0x0000FFFF
#define WINED3DVS_END() 0x0000FFFF

/* Multithreaded flag. Removed from the public header to signal that IWineD3D::CreateDevice ignores it */
#define WINED3DCREATE_MULTITHREADED                 0x00000004

#endif
