/* Copyright (c) 2001, Stanford University
 * All rights reserved.
 *
 * See the file LICENSE.txt for information on redistributing this software.
 */

#ifndef CR_VERSION_H
#define CR_VERSION_H

#define SHCROGL_SSM_VERSION_BEFORE_CTXUSAGE_BITS       28
#define SHCROGL_SSM_VERSION_BEFORE_FRONT_DRAW_TRACKING 29
#define SHCROGL_SSM_VERSION                            30

/* These define the Chromium release number.
 * Alpha Release = 0.1.0, Beta Release = 0.2.0
 */
#define CR_MAJOR_VERSION 1
#define CR_MINOR_VERSION 9
#define CR_PATCH_VERSION 0

#define CR_VERSION_STRING "1.9"   /* Chromium version, not OpenGL version */


/* These define the OpenGL version that Chromium supports.
 * This lets users easily recompile Chromium with/without OpenGL 1.x support.
 * We use OpenGL's GL_VERSION_1_x convention.
 */
#define CR_OPENGL_VERSION_1_0 1
#define CR_OPENGL_VERSION_1_1 1
#define CR_OPENGL_VERSION_1_2 1
#define CR_OPENGL_VERSION_1_2_1 1
#define CR_OPENGL_VERSION_1_3 1
#define CR_OPENGL_VERSION_1_4 1
#define CR_OPENGL_VERSION_1_5 1
#define CR_OPENGL_VERSION_2_0 1
#define CR_OPENGL_VERSION_2_1 1

/* Version (string) of OpenGL functionality supported by Chromium */
#ifdef CR_OPENGL_VERSION_2_1
# define CR_OPENGL_VERSION_STRING "2.1"
#elif defined(CR_OPENGL_VERSION_2_0)
# define CR_OPENGL_VERSION_STRING "2.0"
#else
# define CR_OPENGL_VERSION_STRING "1.5"
#endif


/* These define the OpenGL extensions that Chromium supports.
 * Users can enable/disable support for particular OpenGL extensions here.
 * Again, use OpenGL's convention.
 * WARNING: if you add new extensions here, also update this file:
 * state_tracker/state_limits.c
 */

/*#define CR_ARB_imaging 1    not yet */
#define CR_ARB_depth_texture 1
#define CR_ARB_fragment_program 1
#define CR_ARB_multitexture 1
#define CR_ARB_multisample 1
#define CR_ARB_occlusion_query 1
#define CR_ARB_point_parameters 1
#define CR_ARB_point_sprite 1
#define CR_ARB_shadow 1
#define CR_ARB_shadow_ambient 1
#define CR_ARB_texture_border_clamp 1
#define CR_ARB_texture_compression 1
#define CR_ARB_texture_cube_map 1
#define CR_ARB_texture_env_add 1
#define CR_ARB_texture_env_combine 1
#define CR_ARB_texture_env_crossbar 1
#define CR_ARB_texture_env_dot3 1
#define CR_ARB_texture_mirrored_repeat 1
#define CR_ATI_texture_mirror_once 1
#define CR_ARB_texture_non_power_of_two 1
#define CR_ARB_transpose_matrix 1
#define CR_ARB_vertex_buffer_object 1
#define CR_ARB_vertex_program 1
#define CR_ARB_window_pos 1

#define CR_EXT_blend_color 1
#define CR_EXT_blend_equation 1
#define CR_EXT_blend_minmax 1
#define CR_EXT_blend_logic_op 1
#define CR_EXT_blend_subtract 1
#define CR_EXT_blend_func_separate 1
#define CR_EXT_clip_volume_hint 1
#define CR_EXT_fog_coord 1
#define CR_EXT_multi_draw_arrays 1
#define CR_EXT_shadow_funcs 1
#define CR_EXT_secondary_color 1
#ifndef CR_OPENGL_VERSION_1_2
#define CR_EXT_separate_specular_color 1
#endif
#define CR_EXT_stencil_wrap 1
#define CR_EXT_texture_cube_map 1
#define CR_EXT_texture_edge_clamp 1
#define CR_EXT_texture_env_add 1
#define CR_EXT_texture_filter_anisotropic 1
#define CR_EXT_texture_lod_bias 1
#define CR_EXT_texture_object 1
#define CR_EXT_texture3D 1

#define CR_IBM_rasterpos_clip 1

#define CR_NV_fog_distance 1
#define CR_NV_register_combiners 1
#define CR_NV_register_combiners2 1
#define CR_NV_texgen_reflection 1
#define CR_NV_texture_rectangle  1
#define CR_NV_vertex_program 1
#define CR_NV_vertex_program1_1 1
#define CR_NV_vertex_program2 1
#define CR_NV_vertex_program2_option 1
#define CR_NV_vertex_program3 1
#define CR_NV_fragment_program 1
#define CR_NV_fragment_program_option 1
#define CR_NV_fragment_program2 1

#define CR_SGIS_texture_border_clamp 1
#define CR_SGIS_texture_edge_clamp 1
#define CR_SGIS_generate_mipmap 1

#define CR_EXT_texture_from_pixmap 1
#define CR_EXT_draw_range_elements 1
#define CR_EXT_texture_compression_s3tc 1

#define CR_ARB_shader_objects 1
#define CR_ARB_vertex_shader 1
#define CR_ARB_fragment_shader 1
#define CR_ARB_shading_language_100 1

#define CR_EXT_framebuffer_object 1
#define CR_EXT_compiled_vertex_array 1

#define CR_ARB_pixel_buffer_object 1
#define CR_EXT_texture_sRGB 1

#define CR_EXT_framebuffer_blit 1
#define CR_EXT_blend_equation_separate 1
#define CR_EXT_stencil_two_side 1

#endif /* CR_VERSION_H */
