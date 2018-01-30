obj-m := spi_mmc.o

#DEBUG Enable  
#CFLAGS_spi_mmc.o := -DDEBUG

KDIR=/lib/modules/$(shell uname -r)/build

all:
	$(MAKE) -C $(KDIR) M=$(PWD) modules 
clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean 	 
