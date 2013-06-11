/* $Id: sbuf.c $ */
/** @file
 * NAT - sbuf implemenation.
 */

/*
 * Copyright (C) 2006-2011 Oracle Corporation
 *
 * This file is part of VirtualBox Open Source Edition (OSE), as
 * available from http://www.virtualbox.org. This file is free software;
 * you can redistribute it and/or modify it under the terms of the GNU
 * General Public License (GPL) as published by the Free Software
 * Foundation, in version 2 as it comes in the "COPYING" file of the
 * VirtualBox OSE distribution. VirtualBox OSE is distributed in the
 * hope that it will be useful, but WITHOUT ANY WARRANTY of any kind.
 */

/*
 * This code is based on:
 *
 * Copyright (c) 1995 Danny Gasparovski.
 *
 * Please read the file COPYRIGHT for the
 * terms and conditions of the copyright.
 */

#include <slirp.h>
#ifndef VBOX_WITH_SLIRP_BSD_SBUF

/* Done as a macro in socket.h */
/* int
 * sbspace(struct sockbuff *sb)
 * {
 *      return SB_DATALEN - sb->sb_cc;
 * }
 */

void
sbfree(struct sbuf *sb)
{
    /*
     * Catch double frees. Actually tcp_close() already filters out listening sockets
     * passing NULL.
     */
    Assert((sb->sb_data));

    /*
     * Don't call RTMemFree() for an already freed buffer, the EFence could complain
     */
    if (sb->sb_data)
    {
        RTMemFree(sb->sb_data);
        sb->sb_data = NULL;
    }
}

void
sbdrop(struct sbuf *sb, int num)
{
    /*
     * We can only drop how much we have
     * This should never succeed
     */
    if (num > sb->sb_cc)
        num = sb->sb_cc;
    sb->sb_cc -= num;
    sb->sb_rptr += num;
    if (sb->sb_rptr >= sb->sb_data + sb->sb_datalen)
        sb->sb_rptr -= sb->sb_datalen;

}

void
sbreserve(PNATState pData, struct sbuf *sb, int size)
{
    NOREF(pData);
    if (sb->sb_data)
    {
        /* Already alloced, realloc if necessary */
        if (sb->sb_datalen != size)
        {
            sb->sb_wptr =
            sb->sb_rptr =
            sb->sb_data = (char *)RTMemRealloc(sb->sb_data, size);
            sb->sb_cc = 0;
            if (sb->sb_wptr)
                sb->sb_datalen = size;
            else
                sb->sb_datalen = 0;
        }
    }
    else
    {
        sb->sb_wptr = sb->sb_rptr = sb->sb_data = (char *)RTMemAlloc(size);
        sb->sb_cc = 0;
        if (sb->sb_wptr)
            sb->sb_datalen = size;
        else
            sb->sb_datalen = 0;
    }
}

/*
 * Try and write() to the socket, whatever doesn't get written
 * append to the buffer... for a host with a fast net connection,
 * this prevents an unnecessary copy of the data
 * (the socket is non-blocking, so we won't hang)
 */
void
sbappend(PNATState pData, struct socket *so, struct mbuf *m)
{
    int ret = 0;
    int mlen = 0;
    caddr_t buf = NULL;

    STAM_PROFILE_START(&pData->StatIOSBAppend_pf, a);
    LogFlow(("sbappend: so = %lx, m = %lx, m->m_len = %d\n", (long)so, (long)m, m ? m->m_len : 0));

    STAM_COUNTER_INC(&pData->StatIOSBAppend);
    /* Shouldn't happen, but...  e.g. foreign host closes connection */
    mlen = m_length(m, NULL);
    if (mlen <= 0)
    {
        STAM_COUNTER_INC(&pData->StatIOSBAppend_zm);
        goto done;
    }

    /*
     * If there is urgent data, call sosendoob
     * if not all was sent, sowrite will take care of the rest
     * (The rest of this function is just an optimisation)
     */
    if (so->so_urgc)
    {
        sbappendsb(pData, &so->so_rcv, m);
        m_freem(pData, m);
        sosendoob(so);
        return;
    }

    /*
     * We only write if there's nothing in the buffer,
     * ottherwise it'll arrive out of order, and hence corrupt
     */
    if (m->m_next)
    {
        buf = RTMemAlloc(mlen);
        if (buf == NULL)
        {
            ret = 0;
            goto no_sent;
        }
        m_copydata(m, 0, mlen, buf);
    }
    else
        buf = mtod(m, char *);

    if(!so->so_rcv.sb_cc)
        ret = send(so->s, buf, mlen, 0);

    if (m->m_next)
        RTMemFree(buf);
no_sent:

    if (ret <= 0)
    {
        STAM_COUNTER_INC(&pData->StatIOSBAppend_wf);
        /*
         * Nothing was written
         * It's possible that the socket has closed, but
         * we don't need to check because if it has closed,
         * it will be detected in the normal way by soread()
         */
        sbappendsb(pData, &so->so_rcv, m);
        STAM_PROFILE_STOP(&pData->StatIOSBAppend_pf_wf, a);
        goto done;
    }
    else if (ret != mlen)
    {
        STAM_COUNTER_INC(&pData->StatIOSBAppend_wp);
        /*
         * Something was written, but not everything..
         * sbappendsb the rest
         */
        m_adj(m, ret);
        sbappendsb(pData, &so->so_rcv, m);
        STAM_PROFILE_STOP(&pData->StatIOSBAppend_pf_wp, a);
        goto done;
    } /* else */
    /* Whatever happened, we free the mbuf */
    STAM_COUNTER_INC(&pData->StatIOSBAppend_wa);
    STAM_PROFILE_STOP(&pData->StatIOSBAppend_pf_wa, a);
done:
    m_freem(pData, m);
}

/*
 * Copy the data from m into sb
 * The caller is responsible to make sure there's enough room
 */
void
sbappendsb(PNATState pData, struct sbuf *sb, struct mbuf *m)
{
    int len, n,  nn;
#ifndef VBOX_WITH_STATISTICS
    NOREF(pData);
#endif

    len = m_length(m, NULL);

    STAM_COUNTER_INC(&pData->StatIOSBAppendSB);
    if (sb->sb_wptr < sb->sb_rptr)
    {
        STAM_COUNTER_INC(&pData->StatIOSBAppendSB_w_l_r);
        n = sb->sb_rptr - sb->sb_wptr;
        if (n > len)
            n = len;
        m_copydata(m, 0, n, sb->sb_wptr);
    }
    else
    {
        STAM_COUNTER_INC(&pData->StatIOSBAppendSB_w_ge_r);
        /* Do the right edge first */
        n = sb->sb_data + sb->sb_datalen - sb->sb_wptr;
        if (n > len)
            n = len;
        m_copydata(m, 0, n, sb->sb_wptr);
        len -= n;
        if (len)
        {
            /* Now the left edge */
            nn = sb->sb_rptr - sb->sb_data;
            if (nn > len)
                nn = len;
            m_copydata(m, n, nn, sb->sb_data);
            n += nn;
        }
    }

    sb->sb_cc += n;
    sb->sb_wptr += n;
    if (sb->sb_wptr >= sb->sb_data + sb->sb_datalen)
    {
        STAM_COUNTER_INC(&pData->StatIOSBAppendSB_w_alter);
        sb->sb_wptr -= sb->sb_datalen;
    }
}

/*
 * Copy data from sbuf to a normal, straight buffer
 * Don't update the sbuf rptr, this will be
 * done in sbdrop when the data is acked
 */
void
sbcopy(struct sbuf *sb, int off, int len, char *to)
{
    char *from;

    from = sb->sb_rptr + off;
    if (from >= sb->sb_data + sb->sb_datalen)
        from -= sb->sb_datalen;

    if (from < sb->sb_wptr)
    {
        if (len > sb->sb_cc)
            len = sb->sb_cc;
        memcpy(to, from, len);
    }
    else
    {
        /* re-use off */
        off = (sb->sb_data + sb->sb_datalen) - from;
        if (off > len)
            off = len;
        memcpy(to, from, off);
        len -= off;
        if (len)
            memcpy(to+off, sb->sb_data, len);
    }
}
#else /* VBOX_WITH_SLIRP_BSD_SBUF */
void
sbappend (PNATState pData, struct socket *so, struct mbuf *m)
{
    int ret = 0;
    int mlen = 0;
    caddr_t buf = NULL;

    STAM_PROFILE_START(&pData->StatIOSBAppend_pf, a);
    LogFlow(("sbappend: so = %R[natsock], m = %lx, m->m_len = %d\n", so, (long)m, m ? m->m_len : 0));

    STAM_COUNTER_INC(&pData->StatIOSBAppend);
    mlen = m_length(m, NULL);
    if (mlen <= 0)
    {
        STAM_COUNTER_INC(&pData->StatIOSBAppend_zm);
        goto done;
    }

    /*
     * We only write if there's nothing in the buffer,
     * ottherwise it'll arrive out of order, and hence corrupt
     */
    buf = RTMemAlloc(mlen);
    if (buf == NULL)
    {
        ret = 0;
        goto no_sent;
    }
    m_copydata(m, 0, mlen, buf);

    /*
     * If there is urgent data, call sosendoob
     * if not all was sent, sowrite will take care of the rest
     * (The rest of this function is just an optimisation)
     */
    if (so->so_urgc)
    {
        sbuf_bcpy(&so->so_rcv, buf, mlen);
        RTMemFree(buf);
        m_free(pData, m);
        sosendoob(so);
        return;
    }

    if(!sbuf_len(&so->so_rcv))
        ret = send(so->s, buf, mlen, 0);
no_sent:

    if (ret <= 0)
    {
        STAM_COUNTER_INC(&pData->StatIOSBAppend_wf);
        /*
         * Nothing was written
         * It's possible that the socket has closed, but
         * we don't need to check because if it has closed,
         * it will be detected in the normal way by soread()
         */
        sbuf_bcpy(&so->so_rcv, buf, mlen);
        STAM_PROFILE_STOP(&pData->StatIOSBAppend_pf_wf, a);
        goto done;
    }
    else if (ret != mlen)
    {
        STAM_COUNTER_INC(&pData->StatIOSBAppend_wp);
        /*
         * Something was written, but not everything..
         * sbappendsb the rest
         */
        sbuf_bcpy(&so->so_rcv, &buf[ret + 1], mlen - ret);
        STAM_PROFILE_STOP(&pData->StatIOSBAppend_pf_wp, a);
        goto done;
    } /* else */
    /* Whatever happened, we free the mbuf */
    STAM_COUNTER_INC(&pData->StatIOSBAppend_wa);
    STAM_PROFILE_STOP(&pData->StatIOSBAppend_pf_wa, a);
done:
    RTMemFree(buf);
    m_freem(pData, m);
}
#endif
