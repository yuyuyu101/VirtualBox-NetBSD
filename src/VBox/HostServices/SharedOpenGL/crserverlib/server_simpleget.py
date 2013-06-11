# Copyright (c) 2001, Stanford University
# All rights reserved.
#
# See the file LICENSE.txt for information on redistributing this software.

import sys

import apiutil


apiutil.CopyrightC()

print """#include "cr_spu.h"
#include "chromium.h"
#include "cr_error.h"
#include "cr_mem.h"
#include "cr_net.h"
#include "server_dispatch.h"
#include "server.h"
"""

from get_sizes import *;


funcs = [ 'GetIntegerv', 'GetFloatv', 'GetDoublev', 'GetBooleanv' ]
types = [ 'GLint', 'GLfloat', 'GLdouble', 'GLboolean' ]

for index in range(len(funcs)):
    func_name = funcs[index]
    params = apiutil.Parameters(func_name)
    print 'void SERVER_DISPATCH_APIENTRY crServerDispatch%s( %s )' % ( func_name, apiutil.MakeDeclarationString(params))
    print '{'
    print '\t%s *get_values;' % types[index]
    print '\tint tablesize;'
    print """
    #ifdef CR_ARB_texture_compression
    if (GL_COMPRESSED_TEXTURE_FORMATS_ARB == pname)
    {
        GLint numtexfmts = 0;
        cr_server.head_spu->dispatch_table.GetIntegerv(GL_NUM_COMPRESSED_TEXTURE_FORMATS_ARB, &numtexfmts);
        tablesize = numtexfmts * sizeof(%s);
    }
    else
    #endif
    {
        tablesize = __numValues( pname ) * sizeof(%s);
    }
    """ % (types[index], types[index])
    print '\t(void) params;'
    print '\tget_values = (%s *) crAlloc( tablesize );' % types[index]
    print '\tif (tablesize>0)'
    print '\tcr_server.head_spu->dispatch_table.%s( pname, get_values );' % func_name
    print """
    if (GL_TEXTURE_BINDING_1D==pname
        || GL_TEXTURE_BINDING_2D==pname
        || GL_TEXTURE_BINDING_3D==pname
        || GL_TEXTURE_BINDING_RECTANGLE_ARB==pname
        || GL_TEXTURE_BINDING_CUBE_MAP_ARB==pname)
    {
        GLuint texid;
        CRASSERT(tablesize/sizeof(%s)==1);
        texid = (GLuint) *get_values;
        *get_values = (%s) crStateTextureHWIDtoID(texid);
    }
    else if (GL_CURRENT_PROGRAM==pname)
    {
        GLuint programid;
        CRASSERT(tablesize/sizeof(%s)==1);
        programid = (GLuint) *get_values;
        *get_values = (%s) crStateGLSLProgramHWIDtoID(programid);
    }
    else if (GL_FRAMEBUFFER_BINDING_EXT==pname
             ||GL_READ_FRAMEBUFFER_BINDING==pname)
    {
        GLuint fboid;
        CRASSERT(tablesize/sizeof(%s)==1);
        fboid = crStateFBOHWIDtoID((GLuint) *get_values);
        if (cr_server.curClient->currentMural->bUseFBO
            && crServerIsRedirectedToFBO()
            && fboid==cr_server.curClient->currentMural->idFBO)
        {
            fboid = 0;
        }
        *get_values = (%s) fboid;
    }
    else if (GL_READ_BUFFER==pname)
    {
        if (cr_server.curClient->currentMural->bUseFBO && crServerIsRedirectedToFBO()
            && cr_server.curClient->currentMural->idFBO
            && !crStateGetCurrent()->framebufferobject.readFB)
        {
            *get_values = (%s) crStateGetCurrent()->buffer.readBuffer;
        }
    }
    else if (GL_DRAW_BUFFER==pname)
    {
        if (cr_server.curClient->currentMural->bUseFBO && crServerIsRedirectedToFBO()
            && cr_server.curClient->currentMural->idFBO
            && !crStateGetCurrent()->framebufferobject.drawFB)
        {
            *get_values = (%s) crStateGetCurrent()->buffer.drawBuffer;
        }
    }
    else if (GL_RENDERBUFFER_BINDING_EXT==pname)
    {
        GLuint rbid;
        CRASSERT(tablesize/sizeof(%s)==1);
        rbid = (GLuint) *get_values;
        *get_values = (%s) crStateRBOHWIDtoID(rbid);
    }
    else if (GL_ARRAY_BUFFER_BINDING_ARB==pname
             || GL_ELEMENT_ARRAY_BUFFER_BINDING_ARB==pname
             || GL_VERTEX_ARRAY_BUFFER_BINDING_ARB==pname
             || GL_NORMAL_ARRAY_BUFFER_BINDING_ARB==pname
             || GL_COLOR_ARRAY_BUFFER_BINDING_ARB==pname
             || GL_INDEX_ARRAY_BUFFER_BINDING_ARB==pname
             || GL_TEXTURE_COORD_ARRAY_BUFFER_BINDING_ARB==pname
             || GL_EDGE_FLAG_ARRAY_BUFFER_BINDING_ARB==pname
             || GL_SECONDARY_COLOR_ARRAY_BUFFER_BINDING_ARB==pname
             || GL_FOG_COORDINATE_ARRAY_BUFFER_BINDING_ARB==pname
             || GL_WEIGHT_ARRAY_BUFFER_BINDING_ARB==pname)
    {
        GLuint bufid;
        CRASSERT(tablesize/sizeof(%s)==1);
        bufid = (GLuint) *get_values;
        *get_values = (%s) crStateBufferHWIDtoID(bufid);
    }
    else if (GL_MAX_COMBINED_TEXTURE_IMAGE_UNITS==pname)
    {
    	if (CR_MAX_TEXTURE_UNITS < (GLuint)*get_values)
    	{
    		*get_values = (%s)CR_MAX_TEXTURE_UNITS;
    	} 
    }
    """ % (types[index], types[index], types[index], types[index], types[index], types[index], types[index], types[index], types[index], types[index], types[index], types[index], types[index])
    print '\tcrServerReturnValue( get_values, tablesize );'
    print '\tcrFree(get_values);'
    print '}\n'
