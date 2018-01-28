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


static int mmc_spi_probe(struct spi_device *spi)
{
	return 0; 
}

static int mmc_spi_remove(struct spi_device *spi)
{
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
