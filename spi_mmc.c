#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/bio.h>
#include <linux/dma-mapping.h>
#include <linux/crc7.h>
#include <linux/crc-itu-t.h>
#include <linux/scatterlist.h>

/* mmc related header file */
#include <linux/mmc/host.h>
#include <linux/mmc/mmc.h>		/* for R1_SPI_* bit values */
#include <linux/mmc/slot-gpio.h>

#include <linux/spi/spi.h>
#include <linux/spi/mmc_spi.h>

#include <asm/unaligned.h>



/****************************************************************************/

/*
 * Local Data Structures
 */

/* "scratch" is per-{command,block} data exchanged with the card */
struct scratch {
	u8			status[29];
	u8			data_token;
	__be16			crc_val;
};

struct mmc_spi_host {
	struct mmc_host		*mmc;
	struct spi_device	*spi;

	unsigned char		power_mode;
	u16			powerup_msecs;

	struct mmc_spi_platform_data	*pdata;

	/* for bulk data transfers */
	struct spi_transfer	token, t, crc, early_status;
	struct spi_message	m;

	/* for status readback */
	struct spi_transfer	status;
	struct spi_message	readback;

	/* underlying DMA-aware controller, or null */
	struct device		*dma_dev;

	/* buffer used for commands and for message "overhead" */
	struct scratch		*data;
	dma_addr_t		data_dma;

	/* Specs say to write ones most of the time, even when the card
	 * has no need to read its input data; and many cards won't care.
	 * This is our source of those ones.
	 */
	void			*ones;
	dma_addr_t		ones_dma;
};

/**
 * crc_itu_t - Compute the CRC-ITU-T for the data buffer
 *
 * @crc:     previous CRC value
 * @buffer:  data pointer
 * @len:     number of bytes in the buffer
 *
 * Returns the updated CRC value
 */
/*
 * Write one block:
 *  - caller handled preceding N(WR) [1+] all-ones bytes
 *  - data block
 *	+ token
 *	+ data bytes
 *	+ crc16
 *  - an all-ones byte ... card writes a data-response byte
 *  - followed by N(EC) [0+] all-ones bytes, card writes zero/'busy'
 *
 * Return negative errno, else success.
 */

static int
mmc_spi_writeblock(struct mmc_spi_host *host, struct spi_transfer *t,
	unsigned long timeout)
{
	struct spi_device	*spi = host->spi;
	int			status, i;
	struct scratch		*scratch = host->data;
	u32			pattern;
	
	/*To convert from the processor's native format into little-endian*/
	if (host->mmc->use_spi_crc)
		scratch->crc_val = cpu_to_be16(
				crc_itu_t(0, t->tx_buf, t->len));
	if (host->dma_dev)
		dma_sync_single_for_device(host->dma_dev,
				host->data_dma, sizeof(*scratch),
				DMA_BIDIRECTIONAL);
	status = spi_sync_locked(spi, &host->m);
	if (status != 0) {
		dev_dbg(&spi->dev, "write error (%d)\n", status);
		return status;
	}

	if (host->dma_dev)
		dma_sync_single_for_cpu(host->dma_dev,
				host->data_dma, sizeof(*scratch),
				DMA_BIDIRECTIONAL);
	
	
	status = spi_sync_locked(spi, &host->m);

	if (status != 0) {
		dev_dbg(&spi->dev, "write error (%d)\n", status);
		return status;
	}

	if (host->dma_dev)
		dma_sync_single_for_cpu(host->dma_dev,
				host->data_dma, sizeof(*scratch),
				DMA_BIDIRECTIONAL);

	/*
	 * Get the transmission data-response reply.  It must follow
	 * immediately after the data block we transferred.  This reply
	 * doesn't necessarily tell whether the write operation succeeded;
	 * it just says if the transmission was ok and whether *earlier*
	 * writes succeeded; see the standard.
	 *
	 * In practice, there are (even modern SDHC-)cards which are late
	 * in sending the response, and miss the time frame by a few bits,
	 * so we have to cope with this situation and check the response
	 * bit-by-bit. Arggh!!!
	 */
	
}
/* Build data message with up to four separate transfers.  For TX, we
 * start by writing the data token.  And in most cases, we finish with
 * a status transfer.
 *
 * We always provide TX data for data and CRC.  The MMC/SD protocol
 * requires us to write ones; but Linux defaults to writing zeroes;
 * so we explicitly initialize it to all ones on RX paths.
 *
 * We also handle DMA mapping, so the underlying SPI controller does
 * not need to (re)do it for each message.
 */
static void
mmc_spi_setup_data_message(
	struct mmc_spi_host	*host,
	int			multiple,
	enum dma_data_direction	direction)
{
	struct spi_transfer	*t;
	struct scratch		*scratch = host->data;
	dma_addr_t		dma = host->data_dma;

	spi_message_init(&host->m);
	if (dma)
		host->m.is_dma_mapped = 1;
	if (direction == DMA_TO_DEVICE) {
		t = &host->token;
		memset(t, 0, sizeof(*t));
		t->len = 1;
		if (multiple)
			scratch->data_token = SPI_TOKEN_MULTI_WRITE;
		else
			scratch->data_token = SPI_TOKEN_SINGLE;
		t->tx_buf = &scratch->data_token;
		if (dma)
			t->tx_dma = dma + offsetof(struct scratch, data_token);
		spi_message_add_tail(t, &host->m);
	}
	/* Body of transfer is buffer, then CRC ...
	 * either TX-only, or RX with TX-ones.
	 */
	t = &host->t;
	memset(t, 0, sizeof(*t));
	t->tx_buf = host->ones;
	t->tx_dma = host->ones_dma;
	/* length and actual buffer info are written later */
	spi_message_add_tail(t, &host->m);

	t = &host->crc;
	memset(t, 0, sizeof(*t));
	t->len = 2;
	if (direction == DMA_TO_DEVICE) {
		/* the actual CRC may get written later */
		t->tx_buf = &scratch->crc_val;
		if (dma)
			t->tx_dma = dma + offsetof(struct scratch, crc_val);
	} else {
		t->tx_buf = host->ones;
		t->tx_dma = host->ones_dma;
		t->rx_buf = &scratch->crc_val;
		if (dma)
			t->rx_dma = dma + offsetof(struct scratch, crc_val);
	}
	spi_message_add_tail(t, &host->m);
	
	/*
	 * A single block read is followed by N(EC) [0+] all-ones bytes
	 * before deselect ... don't bother.
	 *
	 * Multiblock reads are followed by N(AC) [1+] all-ones bytes before
	 * the next block is read, or a STOP_TRANSMISSION is issued.  We'll
	 * collect that single byte, so readblock() doesn't need to.
	 *
	 * For a write, the one-byte data response follows immediately, then
	 * come zero or more busy bytes, then N(WR) [1+] all-ones bytes.
	 * Then single block reads may deselect, and multiblock ones issue
	 * the next token (next data block, or STOP_TRAN).  We can try to
	 * minimize I/O ops by using a single read to collect end-of-busy.
	 */
	if (multiple || direction == DMA_TO_DEVICE) {
		t = &host->early_status;
		memset(t, 0, sizeof(*t));
		t->len = (direction == DMA_TO_DEVICE)
				? sizeof(scratch->status)
				: 1;
		t->tx_buf = host->ones;
		t->tx_dma = host->ones_dma;
		t->rx_buf = scratch->status;
		if (dma)
			t->rx_dma = dma + offsetof(struct scratch, status);
		t->cs_change = 1;
		spi_message_add_tail(t, &host->m);
	}
}

//TODO :-->mmc_spi_readblock
/*
 * Read one block:
 *  - skip leading all-ones bytes ... either
 *      + N(AC) [1..f(clock,CSD)] usually, else
 *      + N(CX) [0..8] when reading CSD or CID
 *  - data block
 *	+ token ... if error token, no data or crc
 *	+ data bytes
 *	+ crc16
 *
 * After single block reads, we're done; N(EC) [0+] all-ones bytes follow
 * before dropping chipselect.
 *
 * For multiblock reads, caller either reads the next block or issues a
 * STOP_TRANSMISSION command.
 */
static int
mmc_spi_readblock(struct mmc_spi_host *host, struct spi_transfer *t,
	unsigned long timeout)
{
	struct spi_device	*spi = host->spi;
	int			status;
	struct scratch		*scratch = host->data;
	unsigned int 		bitshift;
	u8			leftover;
	/* At least one SD card sends an all-zeroes byte when N(CX)
	 * applies, before the all-ones bytes ... just cope with that.
	 */
	status = mmc_spi_readbytes(host, 1);  //TODO
	if (status < 0)
		return status;
	status = scratch->status[0];
	if (status == 0xff || status == 0)
		status = mmc_spi_readtoken(host, timeout);  //TODO

	if (status < 0) {
		dev_dbg(&spi->dev, "read error %02x (%d)\n", status, status);
		return status;
	}
	/* The token may be bit-shifted...
	 * the first 0-bit precedes the data stream.
	 */
	bitshift = 7;
	while (status & 0x80) {
		status <<= 1;
		bitshift--;
	}
	leftover = status << 1;
	if (host->dma_dev) {
		dma_sync_single_for_device(host->dma_dev,
				host->data_dma, sizeof(*scratch),
				DMA_BIDIRECTIONAL);
		dma_sync_single_for_device(host->dma_dev,
				t->rx_dma, t->len,
				DMA_FROM_DEVICE);
	}

	status = spi_sync_locked(spi, &host->m);

	if (host->dma_dev) {
		dma_sync_single_for_cpu(host->dma_dev,
				host->data_dma, sizeof(*scratch),
				DMA_BIDIRECTIONAL);
		dma_sync_single_for_cpu(host->dma_dev,
				t->rx_dma, t->len,
				DMA_FROM_DEVICE);
	}
	if (bitshift) {
		/* Walk through the data and the crc and do
		 * all the magic to get byte-aligned data.
		 */
		u8 *cp = t->rx_buf;
		unsigned int len;
		unsigned int bitright = 8 - bitshift;
		u8 temp;
		for (len = t->len; len; len--) {
			temp = *cp;
			*cp++ = leftover | (temp >> bitshift);
			leftover = temp << bitright;
		}
		cp = (u8 *) &scratch->crc_val;
		temp = *cp;
		*cp++ = leftover | (temp >> bitshift);
		leftover = temp << bitright;
		temp = *cp;
		*cp = leftover | (temp >> bitshift);
	}

	if (host->mmc->use_spi_crc) {
		/* crc_itu_t — Compute the CRC-ITU-T for the data buffer: Returns the updated CRC value */
		u16 crc = crc_itu_t(0, t->rx_buf, t->len); 

		be16_to_cpus(&scratch->crc_val);
		if (scratch->crc_val != crc) {
			dev_dbg(&spi->dev, "read - crc error: crc_val=0x%04x, "
					"computed=0x%04x len=%d\n",
					scratch->crc_val, crc, t->len);
			return -EILSEQ;
		}
	}

	t->rx_buf += t->len;
	if (host->dma_dev)
		t->rx_dma += t->len;

	return 0;	

}
/**
 * list_add - add a new entry
 * @new: new entry to be added
 * @head: list head to add it after
 *
 * Insert a new entry after the specified head.
 * This is good for implementing stacks.
 */
/**
 * spi_sync_locked - version of spi_sync with exclusive bus usage
 * @spi: device with which data will be exchanged
 * @message: describes the data transfers
 * Context: can sleep
 *
 * This call may only be used from a context that may sleep.  The sleep
 * is non-interruptible, and has no timeout.  Low-overhead controller
 * drivers may DMA directly into and out of the message buffers.
 *
 * This call should be used by drivers that require exclusive access to the
 * SPI bus. It has to be preceded by a spi_bus_lock call. The SPI bus must
 * be released by a spi_bus_unlock call when the exclusive access is over.
 *
 * It returns zero on success, else a negative error code.
 */
/*
 * An MMC/SD data stage includes one or more blocks, optional CRCs,
 * and inline handshaking.  That handhaking makes it unlike most
 * other SPI protocol stacks.
 */
static void
mmc_spi_data_do(struct mmc_spi_host *host, struct mmc_command *cmd,
		struct mmc_data *data, u32 blk_size)
{
	struct spi_device	*spi = host->spi;
	struct device		*dma_dev = host->dma_dev;
	struct spi_transfer	*t;
	enum dma_data_direction	direction;
	struct scatterlist	*sg;
	int			multiple = (data->blocks > 1);
	u32			clock_rate;
	unsigned long		timeout;

	if (data->flags & MMC_DATA_READ)
		direction = DMA_FROM_DEVICE;
	else
		direction = DMA_TO_DEVICE;

	mmc_spi_setup_data_message(host, multiple, direction);  //TODO
	
	t = &host->t;
		
	if (t->speed_hz)
		clock_rate = t->speed_hz;
	else
		clock_rate = spi->max_speed_hz;
	timeout = data->timeout_ns +
		  data->timeout_clks * 1000000 / clock_rate;
	timeout = usecs_to_jiffies((unsigned int)(timeout / 1000)) + 1;	

	/* Handle scatterlist segments one at a time, with synch for
	 * each 512-byte block
	 */
	for (sg = data->sg, n_sg = data->sg_len; n_sg; n_sg--, sg++) {
		int			status = 0;
		dma_addr_t		dma_addr = 0;
		void			*kmap_addr;
		unsigned		length = sg->length;
		enum dma_data_direction	dir = direction;
		
		/* set up dma mapping for controller drivers that might
		 * use DMA ... though they may fall back to PIO
		 */
		if (dma_dev) {
			/* never invalidate whole *shared* pages ... */
			if ((sg->offset != 0 || length != PAGE_SIZE)
					&& dir == DMA_FROM_DEVICE)
				dir = DMA_BIDIRECTIONAL;

			dma_addr = dma_map_page(dma_dev, sg_page(sg), 0,
						PAGE_SIZE, dir);
			if (direction == DMA_TO_DEVICE)
				t->tx_dma = dma_addr + sg->offset;
			else
				t->rx_dma = dma_addr + sg->offset;
		}
		/* allow pio too; we don't allow highmem */
		kmap_addr = kmap(sg_page(sg));
		if (direction == DMA_TO_DEVICE)
			t->tx_buf = kmap_addr + sg->offset;
		else
			t->rx_buf = kmap_addr + sg->offset;

		/* transfer each block, and update request status */
		/* transfer each block, and update request status */
		while (length) {
			t->len = min(length, blk_size);

			dev_dbg(&host->spi->dev,
				"    mmc_spi: %s block, %d bytes\n",
				(direction == DMA_TO_DEVICE)
				? "write"
				: "read",
				t->len);

			if (direction == DMA_TO_DEVICE)
				status = mmc_spi_writeblock(host, t, timeout);
			else
				status = mmc_spi_readblock(host, t, timeout);
			if (status < 0)
				break;

			data->bytes_xfered += t->len;
			length -= t->len;

			if (!multiple)
				break;
		}
		/* discard mappings */ //TODO
		if (direction == DMA_FROM_DEVICE)
			flush_kernel_dcache_page(sg_page(sg));
		kunmap(sg_page(sg));
		if (dma_dev)
			dma_unmap_page(dma_dev, dma_addr, PAGE_SIZE, dir);

		if (status < 0) {
			data->error = status;
			dev_dbg(&spi->dev, "%s status %d\n",
				(direction == DMA_TO_DEVICE)
					? "write" : "read",
				status);
			break;
		}
	}
	/* NOTE some docs describe an MMC-only SET_BLOCK_COUNT (CMD23) that
	 * can be issued before multiblock writes.  Unlike its more widely
	 * documented analogue for SD cards (SET_WR_BLK_ERASE_COUNT, ACMD23),
	 * that can affect the STOP_TRAN logic.   Complete (and current)
	 * MMC specs should sort that out before Linux starts using CMD23.
	 */
	 //TODO
	if (direction == DMA_TO_DEVICE && multiple) {
		
		struct scratch	*scratch = host->data;
		int		tmp;
		const unsigned	statlen = sizeof(scratch->status);

		dev_dbg(&spi->dev, "    mmc_spi: STOP_TRAN\n");

		/* Tweak the per-block message we set up earlier by morphing
		 * it to hold single buffer with the token followed by some
		 * all-ones bytes ... skip N(BR) (0..1), scan the rest for
		 * "not busy any longer" status, and leave chip selected.
		 */
		INIT_LIST_HEAD(&host->m.transfers);
		list_add(&host->early_status.transfer_list,
				&host->m.transfers);
		memset(scratch->status, 0xff, statlen);
		scratch->status[0] = SPI_TOKEN_STOP_TRAN;      /* terminate multiblock write */

		host->early_status.tx_buf = host->early_status.rx_buf;  
		host->early_status.tx_dma = host->early_status.rx_dma;
		host->early_status.len = statlen;
		
		if (host->dma_dev)
			dma_sync_single_for_device(host->dma_dev,
					host->data_dma, sizeof(*scratch),
					DMA_BIDIRECTIONAL);

		tmp = spi_sync_locked(spi, &host->m);
		//TODO	
		
		if (host->dma_dev)
			dma_sync_single_for_cpu(host->dma_dev,
					host->data_dma, sizeof(*scratch),
					DMA_BIDIRECTIONAL);

		if (tmp < 0) {
			if (!data->error)
				data->error = tmp;
			return;
		}

		/* Ideally we collected "not busy" status with one I/O,
		 * avoiding wasteful byte-at-a-time scanning... but more
		 * I/O is often needed.
		 */
		for (tmp = 2; tmp < statlen; tmp++) {
			if (scratch->status[tmp] != 0)
				return;
		}
		tmp = mmc_spi_wait_unbusy(host, timeout);
		if (tmp < 0 && !data->error)
			data->error = tmp;
	}
}

/****************************************************************************/
/**
 * spi_bus_lock - obtain a lock for exclusive SPI bus usage
 * @master: SPI bus master that should be locked for exclusive bus access
 * Context: can sleep
 *
 * This call may only be used from a context that may sleep.  The sleep
 * is non-interruptible, and has no timeout.
 *
 * This call should be used by drivers that require exclusive access to the
 * SPI bus. The SPI bus must be released by a spi_bus_unlock call when the
 * exclusive access is over. Data transfer must be done by spi_sync_locked
 * and spi_async_locked calls when the SPI bus lock is held.
 *
 * It returns zero on success, else a negative error code.
 */
/**
 *	mmc_request_done - finish processing an MMC request
 *	@host: MMC host which completed request
 *	@mrq: MMC request which request
 *
 *	MMC drivers should call this function when they have completed
 *	their processing of a request.
 */
/*
 *  * MMC driver implementation -- the interface to the MMC stack
 *   */

static void mmc_spi_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct mmc_spi_host	*host = mmc_priv(mmc);
	int			status = -EINVAL;
	int			crc_retry = 5;
	struct mmc_command	stop; 
#ifdef DEBUG
	/* MMC core and layered drivers *MUST* issue SPI-aware commands */
	{
		struct mmc_command	*cmd;
		int			invalid = 0;

		cmd = mrq->cmd;
		if (!mmc_spi_resp_type(cmd)) {
			dev_dbg(&host->spi->dev, "bogus command\n");
			cmd->error = -EINVAL;
			invalid = 1;
		}

		cmd = mrq->stop;
		if (cmd && !mmc_spi_resp_type(cmd)) {
			dev_dbg(&host->spi->dev, "bogus STOP command\n");
			cmd->error = -EINVAL;
			invalid = 1;
		}

		if (invalid) {
			dump_stack();
			mmc_request_done(host->mmc, mrq);
			return;
		}
	}
#endif
	/* request exclusive bus access */
	spi_bus_lock(host->spi->master);

crc_recover:
	/* issue command; then optionally data and stop */
	status = mmc_spi_command_send(host, mrq, mrq->cmd, mrq->data != NULL);
	if (status == 0 && mrq->data) {
		mmc_spi_data_do(host, mrq->cmd, mrq->data, mrq->data->blksz);

		/*
		 * The SPI bus is not always reliable for large data transfers.
		 * If an occasional crc error is reported by the SD device with
		 * data read/write over SPI, it may be recovered by repeating
		 * the last SD command again. The retry count is set to 5 to
		 * ensure the driver passes stress tests.
		 */
		if (mrq->data->error == -EILSEQ && crc_retry) {
			stop.opcode = MMC_STOP_TRANSMISSION;
			stop.arg = 0;
			stop.flags = MMC_RSP_SPI_R1B | MMC_RSP_R1B | MMC_CMD_AC;
			status = mmc_spi_command_send(host, mrq, &stop, 0);
			crc_retry--;
			mrq->data->error = 0;
			goto crc_recover;
		}

		if (mrq->stop)
			status = mmc_spi_command_send(host, mrq, mrq->stop, 0);
		else
			mmc_cs_off(host);
	}

	/* release the bus */
	spi_bus_unlock(host->spi->master);
	
	mmc_request_done(host->mmc, mrq);
}
/* See Section 6.4.1, in SD "Simplified Physical Layer Specification 2.0"
 *
 * NOTE that here we can't know that the card has just been powered up;
 * not all MMC/SD sockets support power switching.
 *
 * FIXME when the card is still in SPI mode, e.g. from a previous kernel,
 * this doesn't seem to do the right thing at all...
 */
static void mmc_spi_initsequence(struct mmc_spi_host *host)
{
	/* Try to be very sure any previous command has completed;
	 * wait till not-busy, skip debris from any old commands.
	 */
	mmc_spi_wait_unbusy(host, r1b_timeout);
	mmc_spi_readbytes(host, 10);
	/*
	 * Do a burst with chipselect active-high.  We need to do this to
	 * meet the requirement of 74 clock cycles with both chipselect
	 * and CMD (MOSI) high before CMD0 ... after the card has been
	 * powered up to Vdd(min), and so is ready to take commands.
	 *
	 * Some cards are particularly needy of this (e.g. Viking "SD256")
	 * while most others don't seem to care.
	 *
	 * Note that this is one of the places MMC/SD plays games with the
	 * SPI protocol.  Another is that when chipselect is released while
	 * the card returns BUSY status, the clock must issue several cycles
	 * with chipselect high before the card will stop driving its output.
	 */
	host->spi->mode |= SPI_CS_HIGH;
	if (spi_setup(host->spi) != 0) {
		/* Just warn; most cards work without it. */
		dev_warn(&host->spi->dev,
				"can't change chip-select polarity\n");
		host->spi->mode &= ~SPI_CS_HIGH;
	}else {
		mmc_spi_readbytes(host, 18); //TODO

		host->spi->mode &= ~SPI_CS_HIGH;
		if (spi_setup(host->spi) != 0) {
			/* Wot, we can't get the same setup we had before? */
			dev_err(&host->spi->dev,
					"can't restore chip-select polarity\n");
		}
	}
}

static  char *mmc_powerstring(u8 power_mode)
{
	switch (power_mode) {
	case MMC_POWER_OFF: return "off";
	case MMC_POWER_UP:  return "up";
	case MMC_POWER_ON:  return "on";
	}
	return "?";
}

static void mmc_spi_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct mmc_spi_host *host = mmc_priv(mmc);
	if (host->power_mode != ios->power_mode) {
		int		canpower;
		canpower = host->pdata && host->pdata->setpower;
		
		dev_dbg(&host->spi->dev, "mmc_spi: power %s (%d)%s\n",
				mmc_powerstring(ios->power_mode),
				ios->vdd,
				canpower ? ", can switch" : "");

		/* switch power on/off if possible, accounting for
		 * max 250msec powerup time if needed.
		 */
		if (canpower) {
			switch (ios->power_mode) {
			case MMC_POWER_OFF:
			case MMC_POWER_UP:
				host->pdata->setpower(&host->spi->dev,
						ios->vdd);
				if (ios->power_mode == MMC_POWER_UP)
					msleep(host->powerup_msecs);
			}
		}
		/* See 6.4.1 in the simplified SD card physical spec 2.0 */
		if (ios->power_mode == MMC_POWER_ON)
			mmc_spi_initsequence(host); //TODO:

		/* If powering down, ground all card inputs to avoid power
		 * delivery from data lines!  On a shared SPI bus, this
		 * will probably be temporary; 6.4.2 of the simplified SD
		 * spec says this must last at least 1msec.
		 *
		 *   - Clock low means CPOL 0, e.g. mode 0
		 *   - MOSI low comes from writing zero
		 *   - Chipselect is usually active low...
		 */
		if (canpower && ios->power_mode == MMC_POWER_OFF) {
			int mres;
			u8 nullbyte = 0;
			host->spi->mode &= ~(SPI_CPOL|SPI_CPHA);
			mres = spi_setup(host->spi);
			if (mres < 0)
				dev_dbg(&host->spi->dev,
					"switch to SPI mode 0 failed\n");
			if (spi_write(host->spi, &nullbyte, 1) < 0)
				dev_dbg(&host->spi->dev,
					"put spi signals to low failed\n");
			/*
			 * Now clock should be low due to spi mode 0;
			 * MOSI should be low because of written 0x00;
			 * chipselect should be low (it is active low)
			 * power supply is off, so now MMC is off too!
			 *
			 * FIXME no, chipselect can be high since the
			 * device is inactive and SPI_CS_HIGH is clear...
			 */
			msleep(10);
			if (mres == 0) {
				host->spi->mode |= (SPI_CPOL|SPI_CPHA);
				mres = spi_setup(host->spi);
				if (mres < 0)
					dev_dbg(&host->spi->dev,
						"switch back to SPI mode 3"
						" failed\n");
			}		
		}	
		host->power_mode = ios->power_mode; 	
	}

	if (host->spi->max_speed_hz != ios->clock && ios->clock != 0) {
		int		status;

		host->spi->max_speed_hz = ios->clock;
		status = spi_setup(host->spi);
		dev_dbg(&host->spi->dev,
			"mmc_spi:  clock to %d Hz, %d\n",
			host->spi->max_speed_hz, status);
	}				
}
/*  .request : The upper MMC block  driver will send a struct mmc_request *mrq 
 *  to do this. The mrq will include what want to do. When you finish the request 
 *  you must call mmc_request_done() to note upper MMC block device driver what it is 
 *  finished. There include command and data.
 *
 *  .set_ios : 
 *  		1.Set/disable Clock.
 *  		2.Power on/off to offer SD Card or not
 *  		3.Set SD card width 1 or 4.
 *  .get_ro :
 *  		1.To check the SD card is write protect or not.
 *
 *  .get_cd : Card detection 
 */
 
static const struct mmc_host_ops mmc_spi_ops = {
	.request	= mmc_spi_request,
	.set_ios	= mmc_spi_set_ios,
	.get_ro		= mmc_gpio_get_ro,
	.get_cd		= mmc_gpio_get_cd,
};


/****************************************************************************/

/*-------------------------------------------------------------------------*/

/* Core methods for SPI master protocol drivers.  Some of the
 * other core methods are currently defined as inline functions.
 */

/**
 * spi_setup - setup SPI mode and clock rate
 * @spi: the device whose settings are being modified
 * Context: can sleep, and no requests are queued to the device
 *
 * SPI protocol drivers may need to update the transfer mode if the
 * device doesn't work with its default.  They may likewise need
 * to update clock rates or word sizes from initial values.  This function
 * changes those settings, and must be called from a context that can sleep.
 * Except for SPI_CS_HIGH, which takes effect immediately, the changes take
 * effect the next time the device is selected and data is transferred to
 * or from it.  When this function returns, the spi device is deselected.
 *
 * Note that this call will fail if the protocol driver specifies an option
 * that the underlying controller or its driver does not support.  For
 * example, not all hardware supports wire transfers using nine bit words,
 * LSB-first wire encoding, or active-high chipselects.
 */


/**
 *	mmc_alloc_host - initialise the per-host structure.
 *	@extra: sizeof private data structure
 *	@dev: pointer to host device model structure
 *
 *	Initialise the per-host structure.
 */
/*It is sometimes necessary, however, to allow the processor to access a 
  mapped streaming DMA buffer. To that end, the DMA layer has long provided 
  a set of functions (like dma_sync_single() and pci_sync_single()) which 
  transfer ownership of the buffer to the CPU. What has always been lacking, 
  however, is a way to transfer ownership back to the device. To fill in that gap, 
  the various synchronization functions have been split in two; instead of 
   dma_sync_single() a driver must now call one or both of:

    dma_sync_single_for_cpu(struct device *dev, 
                            dma_addr_t dma_handle, 
			    size_t size,
			    enum dma_data_direction direction);

    dma_sync_single_for_device(struct device *dev, 
                               dma_addr_t dma_handle, 
			       size_t size,
			       enum dma_data_direction direction);

 dma_sync_single_for_cpu() gives ownership of the DMA buffer back to the processor. 
 After that call, driver code can read or modify the buffer, but the device should 
 not touch it. A call to dma_sync_single_for_device() is required to allow the device 
 to access the buffer again. The other synchronization functions 
  (for scatter/gather and DAC mappings) have been changed as well.

 As might be expected from a change like this, the result was a lot of broken drivers. 
 The patch fixes the in-tree users of the discontinued DMA functions. Out-of-tree and 
 binary-only drivers, however, will have to be fixed separately.
*/
/**
 *	mmc_add_host - initialise host hardware
 *	@host: mmc host
 *
 *	Register the host with the driver model. The host must be
 *	prepared to start servicing requests before this function
 *	completes.
 */
/**
 * mmc_gpio_request_cd - request a gpio for card-detection
 * @host: mmc host
 * @gpio: gpio number requested
 *
 * As devm_* managed functions are used in mmc_gpio_request_cd(), client
 * drivers do not need to explicitly call mmc_gpio_free_cd() for freeing up,
 * if the requesting and freeing are only needed at probing and unbinding time
 * for once.  However, if client drivers do something special like runtime
 * switching for card-detection, they are responsible for calling
 * mmc_gpio_request_cd() and mmc_gpio_free_cd() as a pair on their own.
 *
 * Returns zero on success, else an error.
 */
/**
 * mmc_gpio_request_ro - request a gpio for write-protection
 * @host: mmc host
 * @gpio: gpio number requested
 *
 * As devm_* managed functions are used in mmc_gpio_request_ro(), client
 * drivers do not need to explicitly call mmc_gpio_free_ro() for freeing up,
 * if the requesting and freeing are only needed at probing and unbinding time
 * for once.  However, if client drivers do something special like runtime
 * switching for write-protection, they are responsible for calling
 * mmc_gpio_request_ro() and mmc_gpio_free_ro() as a pair on their own.
 *
 * Returns zero on success, else an error.
 */
static int mmc_spi_probe(struct spi_device *spi)
{
	
	void			*ones;
	struct mmc_host		*mmc;
	struct mmc_spi_host	*host;
	int			status;
	bool			has_ro = false;
	
	/* We rely on full duplex transfers, mostly to reduce
	 * per-transfer overheads (by making fewer transfers).
	 *
	//	#define SPI_MASTER_HALF_DUPLEX	BIT(0)		can't do full duplex */
	if (spi->master->flags & SPI_MASTER_HALF_DUPLEX)
		return EINVAL;
	
	/* MMC and SD specs only seem to care that sampling is on the
	 * rising edge ... meaning SPI modes 0 or 3.  So either SPI mode
	 * should be legit.  We'll use mode 0 since the steady state is 0,
	 * which is appropriate for hotplugging, unless the platform data
	 * specify mode 3 (if hardware is not compatible to mode 0).
	 */
	
	if (spi->mode != SPI_MODE_3)
		spi->mode = SPI_MODE_0;
	spi->bits_per_word = 8;

	/*setup SPI mode and clock rate*/	
	status = spi_setup(spi);
	if (status < 0) {
		dev_dbg(&spi->dev, "needs SPI mode %02x, %d KHz; %d\n",
				spi->mode, spi->max_speed_hz / 1000,
				status);
		return status;
	}
	
	/* We need a supply of ones to transmit.  This is the only time
	 * the CPU touches these, so cache coherency isn't a concern.
	 *
	 * NOTE if many systems use more than one MMC-over-SPI connector
	 * it'd save some memory to share this.  That's evidently rare.
	 */
	status = -ENOMEM;
	ones = kmalloc(MMC_SPI_BLOCKSIZE, GFP_KERNEL);
	if (!ones)
		goto nomem;
	memset(ones, 0xff, MMC_SPI_BLOCKSIZE);

	mmc = mmc_alloc_host(sizeof(*host), &spi->dev);
	if (!mmc)
		goto nomem;
	
	mmc->ops = &mmc_spi_ops;
	mmc->max_blk_size = MMC_SPI_BLOCKSIZE;
	mmc->max_segs = MMC_SPI_BLOCKSATONCE;
	mmc->max_req_size = MMC_SPI_BLOCKSATONCE * MMC_SPI_BLOCKSIZE;
	mmc->max_blk_count = MMC_SPI_BLOCKSATONCE;

	mmc->caps = MMC_CAP_SPI;

	/* SPI doesn't need the lowspeed device identification thing for
	 * MMC or SD cards, since it never comes up in open drain mode.
	 * That's good; some SPI masters can't handle very low speeds!
	 *
	 * However, low speed SDIO cards need not handle over 400 KHz;
	 * that's the only reason not to use a few MHz for f_min (until
	 * the upper layer reads the target frequency from the CSD).
	 */
	mmc->f_min = 400000;
	mmc->f_max = spi->max_speed_hz;
	
	host = mmc_priv(mmc);
	host->mmc = mmc;
	host->spi = spi;

	host->ones = ones;

/*-------------------------------------------------------------------------*/
	/* Platform data is used to hook up things like card sensing
	 * and power switching gpios.
	 */
	host->pdata = mmc_spi_get_pdata(spi);
	if (host->pdata)
		mmc->ocr_avail = host->pdata->ocr_mask;

	if (!mmc->ocr_avail) {
		dev_warn(&spi->dev, "ASSUMING 3.2-3.4 V slot power\n");
		mmc->ocr_avail = MMC_VDD_32_33|MMC_VDD_33_34;
	}
	if (host->pdata && host->pdata->setpower) {
		host->powerup_msecs = host->pdata->powerup_msecs;
		if (!host->powerup_msecs || host->powerup_msecs > 250)
			host->powerup_msecs = 250;
	}
	dev_set_drvdata(&spi->dev, mmc);
	
/*-------------------------------------------------------------------------*/
	/* preallocate dma buffers */
	/*Streaming DMA mapping :https://lwn.net/Articles/75780/ */
	host->data = kmalloc(sizeof(*host->data), GFP_KERNEL);
	if (!host->data)
		goto fail_nobuf1;
	if (spi->master->dev.parent->dma_mask) {
		struct device	*dev = spi->master->dev.parent;
		host->dma_dev = dev;
		host->ones_dma = dma_map_single(dev, ones,
				MMC_SPI_BLOCKSIZE, DMA_TO_DEVICE);
		
		/* REVISIT in theory those map operations can fail... */

		dma_sync_single_for_cpu(host->dma_dev,
				host->data_dma, sizeof(*host->data),
				DMA_BIDIRECTIONAL);	
	}
			
	/* setup message for status/busy readback */
	spi_message_init(&host->readback);
	host->readback.is_dma_mapped = (host->dma_dev != NULL);

	spi_message_add_tail(&host->status, &host->readback);
	host->status.tx_buf = host->ones;
	host->status.tx_dma = host->ones_dma;
	host->status.rx_buf = &host->data->status;
	host->status.rx_dma = host->data_dma + offsetof(struct scratch, status);
	host->status.cs_change = 1;
	
	/* register card detect irq */
	if (host->pdata && host->pdata->init) {
		status = host->pdata->init(&spi->dev, mmc_spi_detect_irq, mmc);
		if (status != 0)
			goto fail_glue_init;
	}
	
	/* pass platform capabilities, if any */
	if (host->pdata) {
		mmc->caps |= host->pdata->caps;
		mmc->caps2 |= host->pdata->caps2;
	}
	status = mmc_add_host(mmc);
	if (status != 0)
		goto fail_add_host;

	if (host->pdata && host->pdata->flags & MMC_SPI_USE_CD_GPIO) {
		status = mmc_gpio_request_cd(mmc, host->pdata->cd_gpio,
					     host->pdata->cd_debounce);
		if (status != 0)
			goto fail_add_host;

		/* The platform has a CD GPIO signal that may support
		 * interrupts, so let mmc_gpiod_request_cd_irq() decide
		 * if polling is needed or not.
		 */
		mmc->caps &= ~MMC_CAP_NEEDS_POLL;
		mmc_gpiod_request_cd_irq(mmc);
	}
	if (host->pdata && host->pdata->flags & MMC_SPI_USE_RO_GPIO) {
		has_ro = true;
		status = mmc_gpio_request_ro(mmc, host->pdata->ro_gpio);
		if (status != 0)
			goto fail_add_host;
	}
	dev_info(&spi->dev, "SD/MMC host %s%s%s%s%s\n",
			dev_name(&mmc->class_dev),
			host->dma_dev ? "" : ", no DMA",
			has_ro ? "" : ", no WP",
			(host->pdata && host->pdata->setpower)
				? "" : ", no poweroff",
			(mmc->caps & MMC_CAP_NEEDS_POLL)
				? ", cd polling" : "");
	return 0;

fail_add_host:
	mmc_remove_host (mmc);
fail_glue_init:
	if (host->dma_dev)
		dma_unmap_single(host->dma_dev, host->data_dma,
				sizeof(*host->data), DMA_BIDIRECTIONAL);
	kfree(host->data);

fail_nobuf1:
	mmc_free_host(mmc);
	mmc_spi_put_pdata(spi);
	dev_set_drvdata(&spi->dev, NULL);

nomem:
	kfree(ones);
	return status

}
static int mmc_spi_remove(struct spi_device *spi)
{
	struct mmc_host		*mmc = dev_get_drvdata(&spi->dev);
	struct mmc_spi_host	*host;
	if (mmc) {
		host = mmc_priv(mmc);

		/* prevent new mmc_detect_change() calls */
		if (host->pdata && host->pdata->exit)
			host->pdata->exit(&spi->dev, mmc);

		mmc_remove_host(mmc);

		if (host->dma_dev) {
			dma_unmap_single(host->dma_dev, host->ones_dma,
				MMC_SPI_BLOCKSIZE, DMA_TO_DEVICE);
			dma_unmap_single(host->dma_dev, host->data_dma,
				sizeof(*host->data), DMA_BIDIRECTIONAL);
		}

		kfree(host->data);
		kfree(host->ones);

		spi->max_speed_hz = mmc->f_max;
		mmc_free_host(mmc);
		mmc_spi_put_pdata(spi);
		dev_set_drvdata(&spi->dev, NULL);
	}
	return 0;
}
static const struct of_device_id mmc_spi_of_match_table[] = {
	{ .compatible = "mmc-spi-slot", },
	{},
}; 

MODULE_DEVICE_TABLE(of, mmc_spi_of_match_table);

static struct spi_driver mmc_spi_driver = {
	.driver = {
		.name =		"mmc_spi",
		.of_match_table = mmc_spi_of_match_table,
	},
	.probe =	mmc_spi_probe,
	.remove =	mmc_spi_remove,
};

module_spi_driver(mmc_spi_driver);

MODULE_AUTHOR("Chandan Jha <beingchandanjha@gmail.com>");
MODULE_DESCRIPTION("SPI SD/MMC host driver");
MODULE_LICENSE("GPL");
