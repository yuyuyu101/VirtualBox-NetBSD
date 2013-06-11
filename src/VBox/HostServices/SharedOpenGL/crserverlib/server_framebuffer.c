/* $Id: server_framebuffer.c $ */

/** @file
 * VBox OpenGL: EXT_framebuffer_object
 */

/*
 * Copyright (C) 2009-2012 Oracle Corporation
 *
 * This file is part of VirtualBox Open Source Edition (OSE), as
 * available from http://www.virtualbox.org. This file is free software;
 * you can redistribute it and/or modify it under the terms of the GNU
 * General Public License (GPL) as published by the Free Software
 * Foundation, in version 2 as it comes in the "COPYING" file of the
 * VirtualBox OSE distribution. VirtualBox OSE is distributed in the
 * hope that it will be useful, but WITHOUT ANY WARRANTY of any kind.
 */

#include "cr_spu.h"
#include "chromium.h"
#include "cr_mem.h"
#include "cr_net.h"
#include "server_dispatch.h"
#include "server.h"

void SERVER_DISPATCH_APIENTRY
crServerDispatchGenFramebuffersEXT(GLsizei n, GLuint *framebuffers)
{
    GLuint *local_buffers = (GLuint *) crAlloc(n * sizeof(*local_buffers));
    (void) framebuffers;

    crStateGenFramebuffersEXT(n, local_buffers);

    crServerReturnValue(local_buffers, n * sizeof(*local_buffers));
    crFree(local_buffers);
}

void SERVER_DISPATCH_APIENTRY
crServerDispatchGenRenderbuffersEXT(GLsizei n, GLuint *renderbuffers)
{
    GLuint *local_buffers = (GLuint *) crAlloc(n * sizeof(*local_buffers));
    (void) renderbuffers;

    crStateGenRenderbuffersEXT(n, local_buffers);

    crServerReturnValue(local_buffers, n * sizeof(*local_buffers));
    crFree(local_buffers);
}

void SERVER_DISPATCH_APIENTRY crServerDispatchFramebufferTexture1DEXT(GLenum target, GLenum attachment, GLenum textarget, GLuint texture, GLint level)
{
    crStateFramebufferTexture1DEXT(target, attachment, textarget, texture, level);
    cr_server.head_spu->dispatch_table.FramebufferTexture1DEXT(target, attachment, textarget, crStateGetTextureHWID(texture), level);
}

void SERVER_DISPATCH_APIENTRY crServerDispatchFramebufferTexture2DEXT(GLenum target, GLenum attachment, GLenum textarget, GLuint texture, GLint level)
{
    crStateFramebufferTexture2DEXT(target, attachment, textarget, texture, level);
    cr_server.head_spu->dispatch_table.FramebufferTexture2DEXT(target, attachment, textarget, crStateGetTextureHWID(texture), level);
}

void SERVER_DISPATCH_APIENTRY crServerDispatchFramebufferTexture3DEXT(GLenum target, GLenum attachment, GLenum textarget, GLuint texture, GLint level, GLint zoffset)
{
    crStateFramebufferTexture3DEXT(target, attachment, textarget, texture, level, zoffset);
    cr_server.head_spu->dispatch_table.FramebufferTexture3DEXT(target, attachment, textarget, crStateGetTextureHWID(texture), level, zoffset);
}

void SERVER_DISPATCH_APIENTRY crServerDispatchBindFramebufferEXT(GLenum target, GLuint framebuffer)
{
#ifdef DEBUG_misha
    GLint rfb = 0, dfb = 0;
#endif
	crStateBindFramebufferEXT(target, framebuffer);

    if (0==framebuffer)
    {
        CRContext *ctx = crStateGetCurrent();
        if (ctx->buffer.drawBuffer == GL_FRONT || ctx->buffer.drawBuffer == GL_FRONT_LEFT)
            cr_server.curClient->currentMural->bFbDraw = GL_TRUE;
    }

    if (0==framebuffer && crServerIsRedirectedToFBO())
    {
        cr_server.head_spu->dispatch_table.BindFramebufferEXT(target, cr_server.curClient->currentMural->idFBO);
#ifdef DEBUG_misha
        Assert(0);
        cr_server.head_spu->dispatch_table.GetIntegerv(GL_READ_FRAMEBUFFER_BINDING_EXT, &rfb);
        cr_server.head_spu->dispatch_table.GetIntegerv(GL_DRAW_FRAMEBUFFER_BINDING_EXT, &dfb);
        if (GL_FRAMEBUFFER_EXT == target)
        {
            Assert(rfb == cr_server.curClient->currentMural->idFBO);
            Assert(dfb == cr_server.curClient->currentMural->idFBO);
        }
        else if (GL_READ_FRAMEBUFFER_EXT == target)
        {
            Assert(rfb == cr_server.curClient->currentMural->idFBO);
        }
        else if (GL_DRAW_FRAMEBUFFER_EXT == target)
        {
            Assert(dfb == cr_server.curClient->currentMural->idFBO);
        }
        else
        {
            Assert(0);
        }
#endif
    }
    else
    {
        cr_server.head_spu->dispatch_table.BindFramebufferEXT(target, crStateGetFramebufferHWID(framebuffer));
#ifdef DEBUG_misha
        cr_server.head_spu->dispatch_table.GetIntegerv(GL_READ_FRAMEBUFFER_BINDING_EXT, &rfb);
        cr_server.head_spu->dispatch_table.GetIntegerv(GL_DRAW_FRAMEBUFFER_BINDING_EXT, &dfb);
        if (GL_FRAMEBUFFER_EXT == target)
        {
            Assert(rfb == crStateGetFramebufferHWID(framebuffer));
            Assert(dfb == crStateGetFramebufferHWID(framebuffer));
        }
        else if (GL_READ_FRAMEBUFFER_EXT == target)
        {
            Assert(rfb == crStateGetFramebufferHWID(framebuffer));
        }
        else if (GL_DRAW_FRAMEBUFFER_EXT == target)
        {
            Assert(dfb == crStateGetFramebufferHWID(framebuffer));
        }
        else
        {
            Assert(0);
        }
#endif
    }
}

void SERVER_DISPATCH_APIENTRY crServerDispatchBindRenderbufferEXT(GLenum target, GLuint renderbuffer)
{
	crStateBindRenderbufferEXT(target, renderbuffer);
	cr_server.head_spu->dispatch_table.BindRenderbufferEXT(target, crStateGetRenderbufferHWID(renderbuffer));
}

void SERVER_DISPATCH_APIENTRY crServerDispatchDeleteFramebuffersEXT(GLsizei n, const GLuint * framebuffers)
{
	crStateDeleteFramebuffersEXT(n, framebuffers);
}

void SERVER_DISPATCH_APIENTRY crServerDispatchDeleteRenderbuffersEXT(GLsizei n, const GLuint * renderbuffers)
{
	crStateDeleteRenderbuffersEXT(n, renderbuffers);
}

void SERVER_DISPATCH_APIENTRY
crServerDispatchFramebufferRenderbufferEXT(GLenum target, GLenum attachment, GLenum renderbuffertarget, GLuint renderbuffer)
{
	crStateFramebufferRenderbufferEXT(target, attachment, renderbuffertarget, renderbuffer);
	cr_server.head_spu->dispatch_table.FramebufferRenderbufferEXT(target, attachment, renderbuffertarget, crStateGetRenderbufferHWID(renderbuffer));
}

void SERVER_DISPATCH_APIENTRY
crServerDispatchGetFramebufferAttachmentParameterivEXT(GLenum target, GLenum attachment, GLenum pname, GLint * params)
{
	GLint local_params[1];
	(void) params;
	crStateGetFramebufferAttachmentParameterivEXT(target, attachment, pname, local_params);

	crServerReturnValue(&(local_params[0]), 1*sizeof(GLint));
}

GLboolean SERVER_DISPATCH_APIENTRY crServerDispatchIsFramebufferEXT( GLuint framebuffer )
{
    /* since GenFramebuffers/Renderbuffers issued to host ogl only on bind + some other ops, the host drivers may not know about them
     * so use state data*/
    GLboolean retval = crStateIsFramebufferEXT(framebuffer);
    crServerReturnValue( &retval, sizeof(retval) );
    return retval; /* WILL PROBABLY BE IGNORED */
}

GLboolean SERVER_DISPATCH_APIENTRY crServerDispatchIsRenderbufferEXT( GLuint renderbuffer )
{
    /* since GenFramebuffers/Renderbuffers issued to host ogl only on bind + some other ops, the host drivers may not know about them
     * so use state data*/
    GLboolean retval = crStateIsRenderbufferEXT(renderbuffer);
    crServerReturnValue( &retval, sizeof(retval) );
    return retval; /* WILL PROBABLY BE IGNORED */
}
