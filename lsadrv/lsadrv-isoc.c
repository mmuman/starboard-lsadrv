/*==========================================================================
 * lsadrv-isoc.c : Linux driver for eIT-Xiroku optical touch sensor
 *
 * Copyright (C) 2009  eIT Co., Ltd. and Xiroku Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
============================================================================*/

#include "lsadrv.h"
#include "lsadrv-ioctl.h"

#define STREAM_TRANSFER_COUNT	2U

/* ring buffer for isochronous stream data */
struct lsadrv_ring_buffer
{
	unsigned char	*inPtr;
	unsigned char	*outPtr;
	unsigned int	 totalSize;
	unsigned int	 currentSize;
	unsigned char	*buffer;
	spinlock_t	*spinLock;	/* for manipulating the buffer pointers */
	wait_queue_head_t *waitq;	/* waken up when ring buffer have available data */
};

/* isochronous transfer object per urb */
struct lsadrv_iso_transfer_object
{
	unsigned int frame;
	struct lsadrv_iso_stream_object *stream;
	struct urb *urb;
	unsigned char *data;
#if LSADRV_DEBUG
/* for debug */
	unsigned int trans_count;
#endif //LSADRV_DEBUG
};

/* isochronous stream object */
struct lsadrv_iso_stream_object
{
	struct lsadrv_device *xdev;
	unsigned int PacketSize;
	unsigned int TransferBufferLength;
	unsigned int FramesPerBuffer;
	unsigned int BufferCount;
	unsigned int TransferCount;
	unsigned int PendingTransfers;
	struct lsadrv_ring_buffer *RingBuffer;
	struct lsadrv_iso_transfer_object *transferObjects;
	// data error count
	unsigned int TotalDataErrorCount;
};

/***************************************************************************/
/* Private functions */

static void
FreeRingBuffer(struct lsadrv_ring_buffer *ringBuffer)
{
	Trace(LSADRV_TRACE_MEMORY, "FreeRingBuffer:0x%p\n", ringBuffer);
	if (ringBuffer) {
		lsadrv_free_waitqueue_head(ringBuffer->waitq);
		lsadrv_spin_lock_term(ringBuffer->spinLock);
		lsadrv_free(ringBuffer->buffer);
		lsadrv_free(ringBuffer);
	}
}

static struct lsadrv_ring_buffer*
AllocRingBuffer(size_t size)
{
	struct lsadrv_ring_buffer *ringBuffer = NULL;

	ringBuffer = lsadrv_malloc(sizeof(struct lsadrv_ring_buffer));
	if (!ringBuffer) {
		return NULL;
	}

	ringBuffer->buffer = lsadrv_malloc(size);
	if (!ringBuffer->buffer) {
		lsadrv_free(ringBuffer);
		return NULL;
	}

	ringBuffer->inPtr = ringBuffer->buffer;
	ringBuffer->outPtr = ringBuffer->buffer;
	ringBuffer->totalSize = size;
	ringBuffer->currentSize = 0;

	lsadrv_spin_lock_init(&ringBuffer->spinLock);
	if (ringBuffer->spinLock == NULL) {
		lsadrv_free(ringBuffer->buffer);
		lsadrv_free(ringBuffer);
		return NULL;
	}
	lsadrv_init_waitqueue_head(&ringBuffer->waitq);	/* waken up when ring buffer have available data */
	if (ringBuffer->waitq == NULL) {
		lsadrv_spin_lock_term(ringBuffer->spinLock);
		lsadrv_free(ringBuffer->buffer);
		lsadrv_free(ringBuffer);
		return NULL;
	}
	return ringBuffer;
}

static unsigned int
ReadRingBuffer(
	struct lsadrv_ring_buffer *ringBuffer,
	unsigned char *readBuffer,
	unsigned int   numberOfBytesToRead)
{
	unsigned int	byteCount;
	unsigned long	flags;

	if (numberOfBytesToRead > ringBuffer->totalSize) {
		return 0;
	}

	//printk(">R ");
	lsadrv_spin_lock(ringBuffer->spinLock, &flags);
	byteCount = ringBuffer->currentSize;
	if (byteCount == 0) {
		lsadrv_spin_unlock(ringBuffer->spinLock, &flags);
		return 0;
	}

	if (numberOfBytesToRead < byteCount) {
		byteCount = numberOfBytesToRead;
	}

	/*
	 * two cases.  Read either wraps or it doesn't.
	 * Handle the non-wrapped case first
	 */
	if ((ringBuffer->outPtr + byteCount - 1) < (ringBuffer->buffer + ringBuffer->totalSize)) {
		if (readBuffer) {
			memcpy(readBuffer, ringBuffer->outPtr, byteCount);
		}
		ringBuffer->outPtr += byteCount;
      		if (ringBuffer->outPtr == ringBuffer->buffer + ringBuffer->totalSize) {
			ringBuffer->outPtr = ringBuffer->buffer;
		}
	}
	/* now handle the wrapped case */
	else {
		unsigned int fragSize;

		fragSize = ringBuffer->buffer + ringBuffer->totalSize - ringBuffer->outPtr;
		if (readBuffer) {
			// get the first half of the read
			memcpy(readBuffer, ringBuffer->outPtr, fragSize);
			// now get the rest
			memcpy(readBuffer + fragSize, ringBuffer->buffer, byteCount - fragSize);
		}
		ringBuffer->outPtr = ringBuffer->buffer + byteCount - fragSize;
	}
 
	/*
	 * update the current size of the ring buffer.  Use spinlock to insure
	 * atomic operation.
	 */
	ringBuffer->currentSize -= byteCount;
	lsadrv_spin_unlock(ringBuffer->spinLock, &flags);

	//printk("<R%d ", byteCount);
	Trace(LSADRV_TRACE_FLOW, "R(%d)", byteCount);
	return byteCount;
}

static unsigned int
WriteRingBuffer(
   	struct lsadrv_ring_buffer *ringBuffer,
   	unsigned char *	writeBuffer,
   	unsigned int 	numberOfBytesToWrite,
	int		overWriteFlg)
{
	unsigned int byteCount;
	unsigned int maxBytes;
	unsigned long flags;

	//printk(">W%d ", numberOfBytesToWrite);
	//Trace(LSADRV_TRACE_FLOW, "W(%u)", numberOfBytesToWrite);
	if (numberOfBytesToWrite > ringBuffer->totalSize) {
		return 0;
	}

	lsadrv_spin_lock(ringBuffer->spinLock, &flags);
	maxBytes = ringBuffer->totalSize - ringBuffer->currentSize;
	if (numberOfBytesToWrite > maxBytes) {
		if (!overWriteFlg) {
			lsadrv_spin_unlock(ringBuffer->spinLock, &flags);
			return 0;
		}
		else {
			/* waste oldest data */
			byteCount = numberOfBytesToWrite - maxBytes;
			ringBuffer->outPtr += byteCount;
			if (ringBuffer->outPtr >= ringBuffer->buffer + ringBuffer->totalSize) {
				ringBuffer->outPtr -= ringBuffer->totalSize;
			}
			ringBuffer->currentSize -= byteCount;
		}
	}
	
	if (numberOfBytesToWrite > 0) {
		/*
		 * two cases.  Write either wraps or it doesn't.
		 * Handle the non-wrapped case first
		 */
		if ((ringBuffer->inPtr + numberOfBytesToWrite - 1) < (ringBuffer->buffer + ringBuffer->totalSize))
		{
			if (writeBuffer) {
				memcpy(ringBuffer->inPtr, writeBuffer, numberOfBytesToWrite);
			}
			ringBuffer->inPtr += numberOfBytesToWrite;
			if (ringBuffer->inPtr == ringBuffer->buffer + ringBuffer->totalSize) {
				ringBuffer->inPtr = ringBuffer->buffer;
			}
		}
		/* now handle the wrapped case */
		else {
			unsigned int fragSize;

			fragSize = ringBuffer->buffer + ringBuffer->totalSize - ringBuffer->inPtr;
			if (writeBuffer) {
				/* write the first fragment */
				memcpy(ringBuffer->inPtr, writeBuffer, fragSize);
				/* now write the rest */
				memcpy(ringBuffer->buffer, writeBuffer + fragSize, numberOfBytesToWrite - fragSize);
			}
			ringBuffer->inPtr = ringBuffer->buffer + numberOfBytesToWrite - fragSize;
		}
	}

	/* 
	 * update the current size of the ring buffer.
	 */
	ringBuffer->currentSize += numberOfBytesToWrite;

	lsadrv_spin_unlock(ringBuffer->spinLock, &flags);

	/* wake up the waiting threads */
	lsadrv_wake_up_interruptible(ringBuffer->waitq);

	//printk("<W%d ", numberOfBytesToWrite);
	return numberOfBytesToWrite;
}

static unsigned int
GetRingBufferCurrentSize(struct lsadrv_ring_buffer *ringBuffer)
{
	unsigned int byteCount;
	unsigned long flags;

	lsadrv_spin_lock(ringBuffer->spinLock, &flags); /* not necessary ? */
	byteCount = ringBuffer->currentSize;
	lsadrv_spin_unlock(ringBuffer->spinLock, &flags);

	Trace(LSADRV_TRACE_FLOW, "G(%d)", byteCount);
	return byteCount;
}

#if LSADRV_DEBUG
static void dump(unsigned char *dat, unsigned int len)
{
	unsigned int i;
	char buf[60], *p = buf;
	for (i = 0; i < len; i++) {
		p += sprintf(p, "%02x ", dat[i]);
		if ((i & 0xf) == 0xf) {
			lsadrv_printk("%s\n", buf);
			p = buf;
		}
	}
	if (i & 0xf) {
		lsadrv_printk("%s\n", buf);
	}
}
#endif /*LSADRV_DEBUG*/

/*
 * Isochronous transfer urb completion routine
 */
void
lsadrv_isoc_handler(void *context, int status)
{
	struct lsadrv_iso_transfer_object *trans = (struct lsadrv_iso_transfer_object *) context;
	struct lsadrv_iso_stream_object *stream;
	struct lsadrv_device *xdev;
	int i;
	unsigned int num_packets;
	unsigned char *src;
	struct lsadrv_iso_packet_desc *mydesc;
	unsigned int recSize;
	unsigned long flags;

	//if (status == 0) {
	//	lsadrv_printk(">>hdr(%d)\n", trans->frame);
	//}
	//else {
	//	lsadrv_printk(">>hdr(%d):status=%d\n", trans->frame, status);
	//}
	Trace(LSADRV_TRACE_STREAM, ">>isoc_handler %d\n", trans->frame);

	stream = trans->stream;
	if (stream == NULL) {
		Err("isoc_handler: stream==NULL\n");
		Trace(LSADRV_TRACE_STREAM, "<<isoc_handler %d\n", trans->frame);
		return;
	}

	xdev = stream->xdev;
	if (xdev == NULL) {
		Err("isoc_handler() called with NULL device?!\n");
		Trace(LSADRV_TRACE_STREAM, "<<isoc_handler %d\n", trans->frame);
		return;
	}

	/* report error status */
	if (status != 0) {
		char *errmsg = NULL;
		/* error case */
		switch (status)
		{
		case -ENOENT:
			Trace(LSADRV_TRACE_STREAM, "%s: URB(%p) unlinked synchronuously.\n", __func__, trans->urb);
			break;
		case -ECONNRESET:
			Trace(LSADRV_TRACE_STREAM, "%s: URB(%p) unlinked asynchronuously.\n", __func__, trans->urb);
			break;
		case -EINPROGRESS:
			break;
		case -ECOMM:	//USB_ST_BUFFEROVERRUN
			errmsg = "Buffer overrun"; break;
		case -ENOSR:	//USB_ST_BUFFERUNDERRUN
			errmsg = "Buffer underrun"; break;
		case -EXDEV:	//USB_ST_PARTIAL_ERROR
			errmsg = "partial error"; break;
		case -ENODEV:
			errmsg = "device removed"; break;
		case -EPIPE:
			errmsg = "Stalled (device not responding)"; break;
		case -EOVERFLOW:	//USB_ST_DATAOVERRUN
			errmsg = "Data overrun"; break;
		case -EREMOTEIO:	//USB_ST_DATAUNDERRUN,USB_ST_SHORT_PACKET
			errmsg = "Data underrun"; break;
		case -EPROTO:	//USB_ST_BITSTUFF,USB_ST_INTERNALERROR
			errmsg = "Bit-stuff/internal error"; break;
		case -EILSEQ:	//USB_ST_CRC
			errmsg = "CRC error"; break;
		case -ETIMEDOUT:	//USB_ST_NORESPONSE
			errmsg = "timed out"; break;
		default:
			errmsg = "unexpected error"; break;
		}
		if (errmsg) {
			Info("isoc_handler: status %d [%s].\n", status, errmsg);
		}
	}

	num_packets = stream->FramesPerBuffer;
	recSize = stream->PacketSize + sizeof(struct lsadrv_iso_packet_desc); /* data + packet descriptor */
	for (i = 0; i < num_packets; i++) {
		src = trans->data + i * recSize;
		mydesc = (struct lsadrv_iso_packet_desc *)(src + stream->PacketSize);
		lsadrv_get_isoc_desc(trans->urb, i, &mydesc->Status, &mydesc->Length);
	}

	if (status == -ENOSR ||
	    status == -EXDEV ||
	    status == -EILSEQ)
	{
		Info("isoc_handler: status %d [Buffer underrun].\n", status);
		for (i = 0; i < num_packets; i++) {
			src = trans->data + i * recSize;
			mydesc = (struct lsadrv_iso_packet_desc *)(src + stream->PacketSize);
			if (mydesc->Length != 0) {
				Info("  %d: length=%d, status=%d\n", i, mydesc->Length, mydesc->Status);
				mydesc->Length = 0;
			}
		}
		status = 0;
	}
	/* add data to ring buffer */
	if (status == 0) {
#if LSADRV_DEBUG
int dump_flg = 0;
#endif /*LSADRV_DEBUG*/
		for (i = 0; i < num_packets; i++) {
			src = trans->data + i * recSize;
			mydesc = (struct lsadrv_iso_packet_desc *)(src + stream->PacketSize);

			//Info("%d:[%d]", i, mydesc->Length);
			//Trace(LSADRV_TRACE_FLOW, "[%d]", mydesc->Length);
			if (mydesc->Status == 0) {
				if (mydesc->Length > 0) {
#if LSADRV_DEBUG
					if (trans->trans_count <= 100) {
						lsadrv_printk("%d: len=%d\n", trans->trans_count, mydesc->Length);
				//		dump(src, mydesc->Length);
						trans->trans_count++;
				//		dump_flg = 1;
					}
#endif /*LSADRV_DEBUG*/
	      				WriteRingBuffer(stream->RingBuffer,
						src,
						recSize,
						1);	/* overwrite */
				}
			}
			/* This is normally not interesting to the user, unless you are really debugging something */
			else {
  				stream->TotalDataErrorCount++;
				Trace(LSADRV_TRACE_FLOW, "Iso frame %d of USB has error %d\n", i, mydesc->Status);
			}
		}
#if LSADRV_DEBUG
		if (dump_flg) {
			lsadrv_printk("%d: alldump: trans:data=0x%p,len=%u\n",
				trans->trans_count,
				trans->data, stream->TransferBufferLength);
			for (i = 0; i < num_packets; i++) {
				dump(trans->data + recSize * i, stream->TransferBufferLength);
			}
		}
#endif /*LSADRV_DEBUG*/
	}

	if (status == 0 && !xdev->StopIsoStream && !xdev->CancelIsoStream
		 && !xdev->unplugged
		 && xdev->statusStreamStopReason == 0		//one error will stop all transfers
	)
	{
		int ret;
		/* resubmit urb */
		Trace(LSADRV_TRACE_STREAM, "isoc_handler %d: submit urb\n", trans->frame);
		//printk("submit(%d)\n", trans->frame);
		ret = lsadrv_usb_resubmit_urb(trans->urb, xdev->udev);
		if (!ret) {
			//lsadrv_modunlock(xdev);
			//printk("<<hdr(%d)\n", trans->frame);
			Trace(LSADRV_TRACE_STREAM, "<<isoc_handler %d\n", trans->frame);
			return;
		}
		Err("submit_urb %d:0x%p failed with error %d\n", trans->frame, trans->urb, ret);
		status = ret;
	}

	/**** error or cancel case ****/

	//printk("h:locking\n");
	//lsadrv_modlock(xdev);
	lsadrv_spin_lock(xdev->streamLock, &flags);

	/* 
	 * Set error code 
	 */
	if (xdev->statusStreamStopReason == 0 ||
	    status == -ENOENT || status == -ECONNRESET) {
		if (status) {
			xdev->statusStreamStopReason = status;
		}
		else if (xdev->StopIsoStream || xdev->CancelIsoStream) {
			xdev->statusStreamStopReason = 1;	//STATUS_CANCELLED
		}
		else if (xdev->unplugged) {
			xdev->statusStreamStopReason = -ENODEV;
		}
		else {
			xdev->statusStreamStopReason = -EFAULT;
		}
  		if (status) {
			xdev->LastFailedStreamUrbStatus = status;
		}
	}

	Trace(LSADRV_TRACE_STREAM, "isoc_handler: stopping transfer %d\n", trans->frame);
	stream->PendingTransfers--;

	//printk("h:unlocking\n");
	//lsadrv_modunlock(xdev);
	lsadrv_spin_unlock(xdev->streamLock, &flags);

	/*
	 * stop stream
	 */
	//printk("h:waking-up\n");
     	lsadrv_wake_up_interruptible(stream->RingBuffer->waitq);
	Trace(LSADRV_TRACE_STREAM, "<<isoc_handler %d\n", trans->frame);
}


static void
WaitForIsoStreamDone(struct lsadrv_iso_stream_object *stream)
{
	struct lsadrv_ring_buffer *ringBuffer;
	//DECLARE_WAITQUEUE(wait, current);
	unsigned char waitbuf[64];	/* sufficient size */
	wait_queue_t *wait = (wait_queue_t *) waitbuf;
	struct lsadrv_device *xdev;
	unsigned int pendingTransfers;

	if (stream == NULL || (ringBuffer = stream->RingBuffer) == NULL) {
		return;
	}

	lsadrv_init_waitqueue_entry(waitbuf, sizeof(waitbuf));

	xdev = stream->xdev;

	//printk(">>Wait\n");
	//printk("add_wait_queue\n");
	lsadrv_add_wait_queue(ringBuffer->waitq, wait);
	lsadrv_set_current_state(TASK_INTERRUPTIBLE);
	while (1) {
		unsigned long flags;
		//lsadrv_modlock(xdev);
		lsadrv_spin_lock(xdev->streamLock, &flags);
		pendingTransfers = stream->PendingTransfers;
		lsadrv_spin_unlock(xdev->streamLock, &flags);
		//lsadrv_modunlock(xdev);
		if (pendingTransfers == 0) {
			break;
		}
		Trace(LSADRV_TRACE_STREAM, "waiting stream done: transfers=%d\n", pendingTransfers);
		//printk("sched\n");
		lsadrv_schedule();
	}
	lsadrv_set_current_state(TASK_RUNNING);
	//printk("remove_wait_queue\n");
	lsadrv_remove_wait_queue(ringBuffer->waitq, wait);
	//printk("<<Wait\n");
}

static void
FreeStreamObject(struct lsadrv_iso_stream_object *stream)
{
	unsigned int i;
	
	if (!stream) {
		return;
	}

	Trace(LSADRV_TRACE_MEMORY, "FreeStreamObject\n");
	/* free transfer objects */
	if (stream->transferObjects) {
		/* free transfer buffers and urbs */
		for (i = 0; i < stream->TransferCount; i++) {
			struct lsadrv_iso_transfer_object *trans = &stream->transferObjects[i];
			lsadrv_usb_unlink_urb(trans->urb);
			lsadrv_usb_free_urb(trans->urb);
			lsadrv_free(trans->data);
		}
		lsadrv_free(stream->transferObjects);
	}

	/* free ring buffer */
   	FreeRingBuffer(stream->RingBuffer);

	/* free stream object */
	lsadrv_free(stream);
}

/* start isochronous stream */
int
lsadrv_start_iso_stream(
	struct lsadrv_device *xdev,
	unsigned int ep,	/* endpoint address(1-15) + direction(0x80 for IN) */
	unsigned int PacketSize, /* ISO packet size. how much data is transferred each frame.
	 			  * Should be equal to the maxpacketsize for the endpoint.
	 			  */
	unsigned int PacketCount, /* Total number of ISO packets to transfer. */
	unsigned int FramesPerBuffer,
	unsigned int BufferCount)
{
	struct usb_device *udev = xdev->udev;
	unsigned int pipe;
	unsigned int max_packet_size;
	struct lsadrv_iso_stream_object *stream = NULL;
	unsigned int transferCount;
	unsigned int recSize;
	unsigned int i;
	
	Trace(LSADRV_TRACE_STREAM, ">> start_iso_stream\n");

	/* device is ready ? */
	if (xdev->unplugged) {
		Err("%s: device is absent\n", __func__);
		return -ENODEV;
	}

	/* Only one stream can be opened at a time */
	if (xdev->iso_init) {
		Err("%s: stream is already started\n", __func__);
		return -EFAULT;
	}

	/* pipe attribute check */
	if ((ep & ~(USB_DIR_IN|0xf))  ||
	    !(ep & USB_DIR_IN)) {
		Info("%s: Pipe number 0x%x is invalid\n", __func__, ep);
		return -EINVAL;
	}

	if (lsadrv_usb_check_epnum(udev, ep & 0x8f)) {
		Info("%s: endpoint 0x%x not found\n", __func__, ep);
		return -EINVAL;
	}

	pipe = lsadrv_usb_rcvisocpipe(udev, ep & 0xf);
	max_packet_size = lsadrv_usb_maxpacket(udev, pipe, 0);
	if (max_packet_size != PacketSize) {
		Info("%s: Packet size mismatch: actual=%d, specified=%d\n", __func__, max_packet_size, PacketSize);
		return -EINVAL;
	}

#ifdef STREAM_TRANSFER_COUNT
	transferCount = min(BufferCount, STREAM_TRANSFER_COUNT);
#else //STREAM_TRANSFER_COUNT
	transferCount = BufferCount;
#endif //STREAM_TRANSFER_COUNT

	/* buffer size per packet (including packet descriptor) */
	recSize = PacketSize + sizeof(struct lsadrv_iso_packet_desc); /* data + packet descriptor */

	/* allocate stream object */
	stream = lsadrv_malloc(sizeof(struct lsadrv_iso_stream_object));
	if (!stream) {
		return -ENOMEM;
	}
	memset(stream, 0, sizeof(*stream));

	stream->xdev = xdev;
	stream->PacketSize = PacketSize;
	stream->TransferBufferLength = recSize * FramesPerBuffer;
	stream->FramesPerBuffer = FramesPerBuffer;
	stream->BufferCount = BufferCount;
	stream->TransferCount = transferCount;
	stream->PendingTransfers = 0;
	stream->TotalDataErrorCount = 0;
	stream->RingBuffer = NULL;
	stream->transferObjects = NULL;


	/* allocate ring buffer */
   	stream->RingBuffer = AllocRingBuffer(PacketCount * recSize);
	if (!stream->RingBuffer) {
		lsadrv_free(stream);
		return -ENOMEM;
	}

	/* allocate transfer objects */
   	stream->transferObjects = lsadrv_malloc(sizeof(struct lsadrv_iso_transfer_object) * transferCount);
	if (!stream->transferObjects) {
		FreeRingBuffer(stream->RingBuffer);
		lsadrv_free(stream);
		return -ENOMEM;
	}
	memset(stream->transferObjects, 0, sizeof(struct lsadrv_iso_transfer_object) * transferCount);

	/* allocate transfer buffers and urbs */
	for (i = 0; i < transferCount; i++) {
		struct lsadrv_iso_transfer_object *trans = &stream->transferObjects[i];

		trans->frame = i;
		trans->stream = stream;

		/* allocate transfer buffers */
		trans->data = lsadrv_malloc(stream->TransferBufferLength);
		if (!trans->data) {
			FreeStreamObject(stream);
			return -ENOMEM;
		}

		/* allocate urb */
		trans->urb = lsadrv_usb_alloc_urb(stream->FramesPerBuffer);
		if (trans->urb == NULL) {
			Err("Failed to allocate urb %d\n", i);
			FreeStreamObject(stream);
			return -ENOMEM;
		}
	}


	/* init URB structure */
	for (i = 0; i < transferCount; i++) {
		struct lsadrv_iso_transfer_object *trans = &stream->transferObjects[i];
		lsadrv_fill_isoc_urb(trans->urb, udev, pipe, trans, 
			trans->data, stream->FramesPerBuffer, max_packet_size, recSize);
	}

	xdev->stream = stream;
	xdev->StopIsoStream = 0;
	xdev->CancelIsoStream = 0;
	xdev->statusStreamStopReason = 0;
	xdev->LastFailedStreamUrbStatus = 0;

	/* submit urbs */
	for (i = 0; i < transferCount; i++) {
		struct lsadrv_iso_transfer_object *trans = &stream->transferObjects[i];
		int ret;
		ret = lsadrv_usb_submit_urb(trans->urb);
		if (!ret) {
			unsigned long flags;
			lsadrv_spin_lock(xdev->streamLock, &flags);
			//lsadrv_modlock(xdev);
			stream->PendingTransfers++;
			lsadrv_spin_unlock(xdev->streamLock, &flags);
			//lsadrv_modunlock(xdev);
			Trace(LSADRV_TRACE_STREAM, "URB 0x%p submitted.\n", trans->urb);
		}
		else {
			Err("start_iso_stream: submit_urb %d failed with error %d\n", i, ret);
		}
	}

	/* All is done... */
	xdev->iso_init = 1;
	Trace(LSADRV_TRACE_STREAM, "<< start_iso_stream\n");
	return 0;
}

int lsadrv_stop_iso_stream(struct lsadrv_device *xdev)
{
	unsigned long flags;
	//printk(">>stop_iso_stream\n");
	Trace(LSADRV_TRACE_STREAM, ">> stop_iso_stream\n");
	//printk("locking ");
	lsadrv_spin_lock(xdev->streamLock, &flags);
	//lsadrv_modlock(xdev);
	xdev->StopIsoStream = 1;
	//printk("unlocking ");
	lsadrv_spin_unlock(xdev->streamLock, &flags);
	//lsadrv_modunlock(xdev);
	if (xdev->stream) {
		struct lsadrv_iso_stream_object *stream = xdev->stream;
		int transferCount = stream->TransferCount;
		int i;
		for (i = 0; i < transferCount; i++) {
			struct lsadrv_iso_transfer_object *trans = &stream->transferObjects[i];
//			printk("locking(%d)", trans->frame);
//			lsadrv_modlock(xdev);
			//printk("unlink(%d)", trans->frame);
			lsadrv_usb_unlink_urb(trans->urb);
//			printk("unlocking(%d)", trans->frame);
//			lsadrv_modunlock(xdev);
		}
		WaitForIsoStreamDone(xdev->stream);
		FreeStreamObject(xdev->stream);
		xdev->stream = NULL;
	}
	xdev->iso_init = 0;
	//printk("<<stop_iso_stream\n");
	Trace(LSADRV_TRACE_STREAM, "<< stop_iso_stream\n");
	return 0;
}

int lsadrv_read_iso_buffer(
	struct lsadrv_device *xdev,
	unsigned int   PacketCount,
	unsigned int   PacketSize,
	unsigned char* dataBuffer,
	unsigned int*  pBytesRead,
	signed long    timeout)		/* jiffies */
{
	struct lsadrv_iso_stream_object *stream = xdev->stream;
	struct lsadrv_ring_buffer *ringBuffer;
	//DECLARE_WAITQUEUE(wait, current);
	unsigned char waitbuf[64];	/* sufficient size */
	wait_queue_t *wait = (wait_queue_t *) waitbuf;
	unsigned int recSize = PacketSize + sizeof(struct lsadrv_iso_packet_desc);
	unsigned int bytesToRead = PacketCount * recSize;
	unsigned int bytesRead = 0;
	unsigned int ret = 0;
	unsigned int size = 0;

//	Trace(LSADRV_TRACE_READ, ">> read_iso_buffer\n");

	*pBytesRead = 0;

	if (stream == NULL || (ringBuffer = stream->RingBuffer) == NULL) {
		Err("read_iso_buffer: buffer is absent\n");
		return -EFAULT;
	}

	if (stream->PacketSize != PacketSize) {
		Err("read_iso_buffer: PacketSize mismatch\n");
		return -EINVAL;
	}

	// check error status
	if (xdev->statusStreamStopReason != 0 || xdev->StopIsoStream || xdev->CancelIsoStream) {
		if (xdev->statusStreamStopReason != 0) {
			Info("read_iso_buffer: stop reason=%d\n", xdev->statusStreamStopReason);
			return xdev->statusStreamStopReason;
		}
		else {
			Err("read_iso_buffer: stream is stopped\n");
			return -EFAULT;
		}
	}

	lsadrv_init_waitqueue_entry(waitbuf, sizeof(waitbuf));

	lsadrv_add_wait_queue(ringBuffer->waitq, wait);
	lsadrv_set_current_state(TASK_INTERRUPTIBLE);
	while (timeout) {
		if (xdev->statusStreamStopReason != 0) {
			ret = xdev->statusStreamStopReason;
			break;
		}
		else if (xdev->StopIsoStream || xdev->CancelIsoStream) {
			Info("read_iso_buffer: stream is stopped\n");
			//ret = -EFAULT;
			break;
		}
		else if ((size = GetRingBufferCurrentSize(ringBuffer))) {
			break;
		}
		timeout = lsadrv_schedule_timeout(timeout);
	}
	//Trace(LSADRV_TRACE_FLOW, "\n");
	lsadrv_set_current_state(TASK_RUNNING);
	lsadrv_remove_wait_queue(ringBuffer->waitq, wait);

	if (ret) {	/* error */
		Info("read_iso_buffer: stop reason=%d\n", ret);
	}
	else if (size) {
		// read data & descriptors from ring buffer
		bytesRead = ReadRingBuffer(ringBuffer, dataBuffer, bytesToRead);
		//Trace(LSADRV_TRACE_FLOW, "R[%d]\n", bytesRead);
		*pBytesRead = bytesRead;
		Trace(LSADRV_TRACE_FLOW, "read_iso_buffer: %d bytes, frame=%d\n", bytesRead, (dataBuffer[4] | (int) dataBuffer[5] << 8));
	}
	else {	/* timedout */
		Trace(LSADRV_TRACE_FLOW, "read_iso_buffer: timed out\n");
	}

//	Trace(LSADRV_TRACE_READ, "<< read_iso_buffer\n");

	return ret;
}
