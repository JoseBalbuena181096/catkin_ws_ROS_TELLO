/*
    Copyright (C) 2016 Parrot SA

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

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stddef.h>
#include <errno.h>
#include <sys/uio.h>

#ifdef BUILD_LIBMUX

#include <libARSAL/ARSAL.h>
#include <libpomp.h>

#include <libARDiscovery/ARDISCOVERY_MuxDiscovery.h>
#include "libmux.h"
#include "libmux-arsdk.h"

#define TAG "ARDISCOVERY_MuxDiscovery"

struct MuxDiscoveryCtx {
	struct mux_ctx        *muxctx;
	device_added_cb_t     device_added_cb;
	device_removed_cb_t   device_removed_cb;
	eof_cb_t              eof_cb;
	void                  *userdata;
};

struct MuxConnectionCtx {
	struct mux_ctx        *muxctx;
	device_conn_resp_cb_t device_conn_resp_cb;
	void                  *userdata;
};

/**
 */
static void rx_device_added(struct pomp_msg *msg, struct MuxDiscoveryCtx *ctx)
{
	int res = 0;
	char *name = NULL;
	uint32_t type = 0;
	char *id = NULL;

	res = pomp_msg_read(msg, MUX_ARSDK_MSG_FMT_DEC_DEVICE_ADDED,
			&name, &type, &id);
	if (res < 0) {
		ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "pomp_msg_read error %d", -res);
		return;
	}
	ctx->device_added_cb(name, type, id, ctx->userdata);
	free(name);
	free(id);
}

/**
 */
static void rx_device_removed(struct pomp_msg *msg, struct MuxDiscoveryCtx *ctx)
{
	int res = 0;
	char *name = NULL;
	uint32_t type = 0;
	char *id = NULL;

	res = pomp_msg_read(msg, MUX_ARSDK_MSG_FMT_DEC_DEVICE_ADDED,
			&name, &type, &id);
	if (res < 0) {
		ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "pomp_msg_read error %d", -res);
		return;
	}
	ctx->device_removed_cb(name, type, id, ctx->userdata);
	free(name);
	free(id);
}

static void mux_rx_conn_resp(struct pomp_msg *msg, struct MuxConnectionCtx *ctx)
{
	int res = 0;
	uint32_t status;
	char *json = NULL;

	res = pomp_msg_read(msg, MUX_ARSDK_MSG_FMT_DEC_CONN_RESP, &status, &json);
	if (res < 0) {
		ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "pomp_msg_read error %d", -res);
		return;
	}

	ctx->device_conn_resp_cb(status, json, ctx->userdata);

	free(json);
}


static void mux_backend_channel_cb(struct mux_ctx *mux_ctx, uint32_t chanid,
			enum mux_channel_event event,
			struct pomp_buffer *buf, void *userdata)
{
	struct MuxConnectionCtx *ctx = (struct MuxConnectionCtx*)userdata;
	struct pomp_msg *msg = NULL;

	switch (event) {
	case MUX_CHANNEL_RESET:
		ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "mux backend channel reset");
	break;
	case MUX_CHANNEL_DATA:
		/* Create pomp message from buffer */
		msg = pomp_msg_new_with_buffer(buf);
		if (msg) {
			/* Decode message */
			switch (pomp_msg_get_id(msg)) {
			case MUX_ARSDK_MSG_ID_CONN_RESP:
				mux_rx_conn_resp(msg, ctx);
			break;
			default:
				ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "unsupported backend channel msg %d", pomp_msg_get_id(msg));
			break;
			}
			pomp_msg_destroy(msg);
		}
	break;
	default:
		ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "unsupported backend channel event %d", event);
	break;
	}
}

static void mux_discovery_channel_cb(struct mux_ctx *mux_ctx, uint32_t chanid,
		enum mux_channel_event event,
		struct pomp_buffer *buf, void *userdata)
{
	struct MuxDiscoveryCtx *ctx = (struct MuxDiscoveryCtx*)userdata;
	struct pomp_msg *msg = NULL;

	switch (event) {
	case MUX_CHANNEL_RESET:
		ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "mux discovery channel reset");
		if (ctx->eof_cb)
			ctx->eof_cb(ctx->userdata);
	break;
	case MUX_CHANNEL_DATA:
		/* Create pomp message from buffer */
		msg = pomp_msg_new_with_buffer(buf);
		if (msg) {
			/* Decode message */
			switch (pomp_msg_get_id(msg)) {
			case MUX_ARSDK_MSG_ID_DEVICE_ADDED:
				rx_device_added(msg, ctx);
			break;
			case MUX_ARSDK_MSG_ID_DEVICE_REMOVED:
				rx_device_removed(msg, ctx);
			break;
			default:
				ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "unsupported discovery channel msg %d", pomp_msg_get_id(msg));
			break;
			}
			pomp_msg_destroy(msg);
		}
	break;
	default:
		ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "unsupported discovery channel event %d", event);
	break;
	}
}

static void discovery_cleanup(struct MuxDiscoveryCtx *ctx)
{
	if (ctx) {
		mux_channel_close(ctx->muxctx, MUX_ARSDK_CHANNEL_ID_DISCOVERY);
		mux_unref(ctx->muxctx);
		free(ctx);
	}
}

static void connection_cleanup(struct MuxConnectionCtx *ctx)
{
	if (ctx) {
		mux_channel_close(ctx->muxctx, MUX_ARSDK_CHANNEL_ID_BACKEND);
		mux_unref(ctx->muxctx);
		free(ctx);
	}
}

/**
 */
POMP_ATTRIBUTE_FORMAT_PRINTF(4, 5)
static int mux_write_msg(struct mux_ctx *mux, uint32_t chanid, uint32_t msgid,
		const char *fmt, ...)
{
	int res = 0;
	struct pomp_msg *msg = NULL;
	va_list args;

	msg = pomp_msg_new();
	if (msg == NULL)
		return -ENOMEM;

	va_start(args, fmt);
	res = pomp_msg_writev(msg, msgid, fmt, args);
	va_end(args);
	if (res < 0) {
		ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "error pomp_msg_write %d", -res);
		goto out;
	}

	res = mux_encode(mux, chanid, pomp_msg_get_buffer(msg));
	if (res < 0 && res != -EPIPE) {
		ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "error mux_encode %d", -res);
		goto out;
	}

out:
	pomp_msg_destroy(msg);
	return res;
}


struct MuxDiscoveryCtx* ARDiscovery_MuxDiscovery_new(struct mux_ctx *muxctx, device_added_cb_t device_added_cb,
						     device_removed_cb_t device_removed_cb, eof_cb_t eof_cb, void* userdata)
{
	int ret;
	struct MuxDiscoveryCtx *ctx = NULL;

	/* Allocate internal object */
	ctx = calloc(1, sizeof(*ctx));
	if (ctx == NULL)
		return NULL;

	ctx->muxctx = muxctx;
	mux_ref(ctx->muxctx);

	ctx->device_added_cb = device_added_cb;
	ctx->device_removed_cb = device_removed_cb;
	ctx->eof_cb = eof_cb;
	ctx->userdata = userdata;

	// open discovery channel
	ret = mux_channel_open(ctx->muxctx, MUX_ARSDK_CHANNEL_ID_DISCOVERY, &mux_discovery_channel_cb, ctx);
	if (ret < 0) {
		ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Error opening discovery channel %d", ret);
		goto fail;
	}

	// send discovery request
	ret = mux_write_msg(ctx->muxctx, MUX_ARSDK_CHANNEL_ID_DISCOVERY, MUX_ARSDK_MSG_ID_DISCOVER, NULL);
	if (ret < 0) {
		ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Error sending discovery request %d", ret);
		goto fail;
	}

	return ctx;

fail:
	discovery_cleanup(ctx);
	return NULL;
}

struct MuxConnectionCtx* ARDiscovery_MuxConnection_new(struct mux_ctx *muxctx,
						       device_conn_resp_cb_t device_conn_resp_cb,
						       void *userdata)
{
	int ret;
	struct MuxConnectionCtx *ctx = NULL;

	/* Allocate internal object */
	ctx = calloc(1, sizeof(*ctx));
	if (ctx == NULL)
		return NULL;

	ctx->muxctx = muxctx;
	mux_ref(ctx->muxctx);

	ctx->device_conn_resp_cb = device_conn_resp_cb;
	ctx->userdata = userdata;

	// open backend channel
	ret = mux_channel_open(ctx->muxctx, MUX_ARSDK_CHANNEL_ID_BACKEND, &mux_backend_channel_cb, ctx);
	if (ret < 0) {
		ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Error opening backend channel %d", ret);
		goto fail;
	}

	return ctx;

fail:
	connection_cleanup(ctx);
	return NULL;
}

int ARDiscovery_MuxConnection_sendConnReq(struct MuxConnectionCtx* ctx,
		const char* controllerName, const char* controllerType,
		const char* deviceId, const char* json)
{
	int ret;
	ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "mux_write_msg MUX_ARSDK_MSG_ID_CONN_REQ");
	ret = mux_write_msg(ctx->muxctx, MUX_ARSDK_CHANNEL_ID_BACKEND, MUX_ARSDK_MSG_ID_CONN_REQ, MUX_ARSDK_MSG_FMT_ENC_CONN_REQ,
			controllerName, controllerType, deviceId, json) ;
	if (ret < 0) {
		ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Error sending Connection request %d", ret);
	}
	ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "mux_write_msg MUX_ARSDK_MSG_ID_CONN_REQ done %d", ret);
	return ret;
}

void ARDiscovery_MuxDiscovery_dispose(struct MuxDiscoveryCtx *ctx)
{
	discovery_cleanup(ctx);
}

void ARDiscovery_MuxConnection_dispose(struct MuxConnectionCtx *ctx)
{
	connection_cleanup(ctx);
}

#endif // BUILD_LIBMUX
