/*
    Copyright (C) 2014 Parrot SA

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in
      the documentation and/or other materials provided with the 
      distribution.
    * Neither the name of Parrot nor the names
      of its contributors may be used to endorse or promote products
      derived from this software without specific prior written
      permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
    OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED 
    AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
    OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
    OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
    SUCH DAMAGE.
*/
/**
 * @file libARSAL/ARSAL_Socket.c
 * @brief This file contains sources about socket abstraction layer
 * @date 06/06/2012
 * @author frederic.dhaeyer@parrot.com
 */
#include <config.h>
#include <stdlib.h>
#include <unistd.h>
#include <libARSAL/ARSAL_Socket.h>
#include <errno.h>

#ifdef HAVE_SYS_UIO_H
#include <sys/uio.h>
#else

#ifdef HAVE_LIMITS_H
#include <limits.h>
#endif

/* IOV_MAX should normally be defined in limits.h. in case it's not, just
 * take a standard value of 1024. It should not cause issue since we emulate
 * writev/readv */
#ifndef IOV_MAX
#define IOV_MAX 1024
#endif

/* SSIZE_MAX should normally be defined in limits.h. In case it's not, hope
 * compiler define __SIZE_MAX__ and derived SSIZE_MAX from it. */
#ifndef SSIZE_MAX
#define SSIZE_MAX (__SIZE_MAX__ >> 1)
#endif

#ifndef HAVE_STRUCT_IOVEC
struct iovec
{
	void *iov_base;
	size_t iov_len;
}
#endif

static ssize_t writev(int sockfd, const struct iovec *iov, int iovcnt)
{
    int i;
    ssize_t wbytes = 0;
    ssize_t ret = 0;
    ssize_t sum = 0;

    if (iovcnt <= 0 || iovcnt > IOV_MAX)
    {
        errno = EINVAL;
        return -1;
    }

    /* check we don't overflow SSIZE_MAX */
    for (i = 0; i < iovcnt; i++)
    {
        if (SSIZE_MAX - sum < iov[i].iov_len)
        {
            errno = EINVAL;
            return -1;
        }

        sum += iov[i].iov_len;
    }

    for (i = 0; i < iovcnt; i++)
    {
        ret = write(sockfd, iov[i].iov_base, iov[i].iov_len);
        if (ret < 0)
        {
            return -1;
        }

        wbytes += ret;

        /* short write means we can't proceed to the next area so break */
        if (ret < iov[i].iov_len)
        {
            break;
        }
    }

    return wbytes;
}

static ssize_t readv(int sockfd, const struct iovec *iov, int iovcnt)
{
    int i;
    ssize_t rbytes = 0;
    ssize_t ret = 0;
    ssize_t sum = 0;

    if (iovcnt <= 0 || iovcnt > IOV_MAX)
    {
        errno = EINVAL;
        return -1;
    }

    /* check we don't overflow SSIZE_MAX */
    for (i = 0; i < iovcnt; i++)
    {
        if (SSIZE_MAX - sum < iov[i].iov_len)
        {
            errno = EINVAL;
            return -1;
        }

        sum += iov[i].iov_len;
    }

    for (i = 0; i < iovcnt; i++)
    {
        ret = read(sockfd, iov[i].iov_base, iov[i].iov_len);
        if (ret < 0)
        {
            return -1;
        }

        rbytes += ret;

        /* short read means we can't proceed to the next area */
        if (ret < iov[i].iov_len)
        {
            break;
        }
    }

    return rbytes;
}
#endif

int ARSAL_Socket_Create(int domain, int type, int protocol)
{
    return socket(domain, type, protocol);
}

int ARSAL_Socket_Connect(int sockfd, const struct sockaddr *addr, socklen_t addrlen)
{
    int ret;

    while (((ret = connect(sockfd, addr, addrlen)) == -1) &&
           (errno == EINTR));

    return ret;
}

ssize_t ARSAL_Socket_Sendto(int sockfd, const void *buf, size_t buflen, int flags, const struct sockaddr *dest_addr, socklen_t addrlen)
{
    ssize_t ret;

    while (((ret = sendto(sockfd, buf, buflen, flags, dest_addr, addrlen)) == -1) &&
           (errno == EINTR));

    return ret;
}

ssize_t ARSAL_Socket_Send(int sockfd, const void *buf, size_t buflen, int flags)
{
    ssize_t res;
    int tries = 10;
    int i;
    for (i = 0; i < tries; i++)
    {
        while (((res = send(sockfd, buf, buflen, flags)) == -1) &&
               (errno == EINTR));

        if (res >= 0 || errno != ECONNREFUSED)
        {
            break;
        }
    }
    return res;
}

ssize_t ARSAL_Socket_Recvfrom(int sockfd, void *buf, size_t buflen, int flags, struct sockaddr *src_addr, socklen_t *addrlen)
{
    ssize_t ret;

    while (((ret = recvfrom(sockfd, buf, buflen, flags, src_addr, addrlen)) == -1) &&
           (errno == EINTR));

    return ret;
}

ssize_t ARSAL_Socket_Recv(int sockfd, void *buf, size_t buflen, int flags)
{
    ssize_t ret;

    while (((ret = recv(sockfd, buf, buflen, flags)) == -1) &&
           (errno == EINTR));

    return ret;
}

ssize_t ARSAL_Socket_Writev (int sockfd, const struct iovec *iov, int iovcnt)
{
    ssize_t ret;

    while (((ret = writev (sockfd, iov, iovcnt)) == -1) &&
           (errno == EINTR));

    return ret;
}

ssize_t ARSAL_Socket_Readv (int sockfd, const struct iovec *iov, int iovcnt)
{
    ssize_t ret;

    while (((ret = readv (sockfd, iov, iovcnt)) == -1) &&
           (errno == EINTR));

    return ret;
}

int ARSAL_Socket_Bind(int sockfd, const struct sockaddr *addr, socklen_t addrlen)
{
    return bind(sockfd, addr, addrlen);
}

int ARSAL_Socket_Listen(int sockfd, int backlog)
{
    return listen(sockfd, backlog);
}

int ARSAL_Socket_Accept(int sockfd, struct sockaddr *addr, socklen_t *addrlen)
{
    int ret;

    while (((ret = accept(sockfd, addr, addrlen)) == -1) &&
           (errno == EINTR));

    return ret;
}

int ARSAL_Socket_Close(int sockfd)
{
    int ret;

    while (((ret = close(sockfd)) == -1) &&
           (errno == EINTR));

    return ret;
}

int ARSAL_Socket_Setsockopt(int sockfd, int level, int optname, const void *optval, socklen_t optlen)
{
    return setsockopt(sockfd, level, optname, optval, optlen);
}

int ARSAL_Socket_Getsockopt(int sockfd, int level, int optname, void *optval, socklen_t *optlen)
{
    return getsockopt(sockfd, level, optname, optval, optlen);
}

int ARSAL_Socket_Getsockname(int sockfd, struct sockaddr *addr, socklen_t *addrlen)
{
    return getsockname(sockfd, addr, addrlen);
}
