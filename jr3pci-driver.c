/***************************************************************************
 *   Copyright (C) 2014 by Wojciech Domski                                 *
 *   wojciech.domski@gmail.com                                             *
 *                                                                         *
 *   This is a driver for jr3 sensor for Xenomai. It was based on driver   *
 *   by Mario Prats                                                        *
 *   mprats@icc.uji.es                                                     *
 *                                                                         *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

#define __MODULE__

#include <linux/module.h>

#define JR3_DESC	"JR3 PCI force/torque sensor Xenomai kernel module"
#define JR3_AUTHOR	"Wojciech Domski (wojciech.domski@gmail.com)"
#define JR3_MAJOR	39

MODULE_DESCRIPTION(JR3_DESC);
MODULE_AUTHOR(JR3_AUTHOR);
MODULE_LICENSE("GPL");

#include <linux/version.h>

#include <linux/kernel.h>
#include <linux/pci.h>

#include <rtdm/rtdm_driver.h>


#include "jr3pci-driver.h"
#include "jr3pci-firmware.h"
#include "jr3pci-ioctl.h"
#include "jr3pci-devicename.h"

#define DATA_SIZE_MAX		16
#define SOME_SUB_CLASS		4711

typedef struct buffer_s {
	int size;
	char data[DATA_SIZE_MAX];
} buffer_t;

unsigned long memregion;
int size;

void __iomem *jr3_base_address;

static int jr3_open(struct rtdm_dev_context *context,
		rtdm_user_info_t * user_info, int oflags);

static int jr3_close(struct rtdm_dev_context *context,
		 rtdm_user_info_t * user_info);

int jr3_ioctl(struct rtdm_dev_context *context, rtdm_user_info_t *user_info,
		unsigned int request, void __user *arg);


/** Board address to PCI virtual address conversion */
void __iomem *board2virtual(int ba) {
	return (void*)jr3_base_address+4*ba;
}

/** Read data memory */
short read_data(int ba, int card) {
	return (short)readl(board2virtual(ba+card*CARD_OFFSET));
}

/** Write data memory */
void write_data(int ba, int data, int card) {
	writel(data,board2virtual(ba+card*CARD_OFFSET));
}

void write_program(int pa, short data0, short data1, int card) {
	writew(data0,board2virtual(pa+card*CARD_OFFSET));
	writew(data1,(board2virtual(pa+card*CARD_OFFSET)+JR3_P8BIT_OFFSET));
}

/** Read program memory */
int read_program(int pa, int card) {
	int r;
	r=readw(board2virtual(pa+card*CARD_OFFSET)) << 8;
	r=r|readb(board2virtual(pa+card*CARD_OFFSET)+JR3_P8BIT_OFFSET);
	return r;
}

int jr3_reset(int card) {
	write_data(JR3_RESET_ADDRESS,0,card);
	return 0;
}

int jr3_zero_offs(int card) {
	write_data(JR3_COMMAND0,JR3_CMD_RESETOFFSETS,card);
	return 0;
}

int jr3_filter(unsigned long arg, int num_filter, int card) {
	int i;
	int ret=0;
	int axload[6];
	int address;

	for (i = 0; i < 6; i++) {
		address = JR3_FILTER0+0x08*num_filter+i;
		axload[i]= (short) read_data(address, card);
	}

	ret = copy_to_user((void *) arg, (int *) axload, sizeof(jr3_six_axis_array));
	return ret;
}

/* Not tested */
int jr3_set_full_scales(unsigned long arg, int card) {
	int fs[8];
	int ret=copy_from_user((int*) fs, (void *) arg, sizeof(jr3_force_array));
	int i;
	int address;

	for (i=0; i< 8; i++) {
		address=JR3_FULLSCALE+i;
		write_data(address,fs[i],card);
	}
	write_data(JR3_COMMAND0,JR3_CMD_SETFULLSCALES,card);
	return ret;
}

static int jr3_open(struct rtdm_dev_context *context,
		rtdm_user_info_t * user_info, int oflags)
{
	return 0;
}

static int jr3_close(struct rtdm_dev_context *context,
		 rtdm_user_info_t * user_info)
{
	return 0;
}

int jr3_get_full_scales(unsigned long arg, int card) {
  int i;
  int ret=0;
  int fullscales[8];

  for (i = 0; i < 8; i++)
	    fullscales[i]= read_data(JR3_FULLSCALE+i, card);
  ret = copy_to_user((void *) arg, (int *) fullscales, sizeof(jr3_force_array));

  return ret;
}

int jr3_ioctl(struct rtdm_dev_context *context, rtdm_user_info_t *user_info,
		unsigned int request, void __user *arg)
//(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int err=0;
	int ret=0;
	int size = _IOC_SIZE(request); /* the size bitfield in cmd */
	int type = _IOC_TYPE(request);
	int nr = _IOC_NR(request);

	if ( type != JR3_IOC_MAGIC)
		return -ENOTTY;

	if ( nr > IOCTL_JR3_MAXNR)
		return -ENOTTY;

#ifndef LINUX_20
	  if (_IOC_DIR(request) & _IOC_READ)
		      err = !access_ok(VERIFY_WRITE, arg, size);

	  if (_IOC_DIR(request) & _IOC_WRITE)
	              err =  !access_ok(VERIFY_READ, arg, size);

	  if (err) return -EFAULT;
#else
	  if (_IOC_DIR(cmd) & _IOC_READ)
	              err = verify_area(VERIFY_WRITE, arg, size);

	  if (_IOC_DIR(cmd) & _IOC_WRITE)
	              err =  verify_area(VERIFY_READ, arg, size);

	  if (err) return err;
#endif
	switch(request) {
		case IOCTL0_JR3_RESET:
			ret = jr3_reset(0);
			break;
		case IOCTL0_JR3_ZEROOFFS:
			ret = jr3_zero_offs(0);
			break;
		case IOCTL0_JR3_FILTER0:
			ret = jr3_filter((unsigned long int)arg, 0,0);
			break;
		case IOCTL0_JR3_FILTER1:
			ret = jr3_filter((unsigned long int)arg, 1,0);
			break;
		case IOCTL0_JR3_FILTER2:
			ret = jr3_filter((unsigned long int)arg, 2,0);
			break;
		case IOCTL0_JR3_FILTER3:
			ret = jr3_filter((unsigned long int)arg, 3,0);
			break;
		case IOCTL0_JR3_FILTER4:
			ret = jr3_filter((unsigned long int)arg, 4,0);
			break;
		case IOCTL0_JR3_FILTER5:
			ret = jr3_filter((unsigned long int)arg, 5,0);
			break;
		case IOCTL0_JR3_FILTER6:
			ret = jr3_filter((unsigned long int)arg, 6,0);
			break;
		case IOCTL0_JR3_GET_FULL_SCALES:
			ret = jr3_get_full_scales((unsigned long int)arg,0);
			break;
		case IOCTL1_JR3_RESET:
			if (PCI_DEVICE_ID_JR3==0x3112)
				ret = jr3_reset(1);
			else ret=-1;
			break;
		case IOCTL1_JR3_ZEROOFFS:
			if (PCI_DEVICE_ID_JR3==0x3112)
				ret = jr3_zero_offs(1);
			else ret=-1;
			break;
		case IOCTL1_JR3_FILTER0:
			if (PCI_DEVICE_ID_JR3==0x3112)
				ret = jr3_filter((unsigned long int)arg, 0,1);
			else ret=-1;
			break;
		case IOCTL1_JR3_FILTER1:
			if (PCI_DEVICE_ID_JR3==0x3112)
				ret = jr3_filter((unsigned long int)arg, 1,1);
			else ret=-1;
			break;
		case IOCTL1_JR3_FILTER2:
			if (PCI_DEVICE_ID_JR3==0x3112)
				ret = jr3_filter((unsigned long int)arg, 2,1);
			else ret=-1;
			break;
		case IOCTL1_JR3_FILTER3:
			if (PCI_DEVICE_ID_JR3==0x3112)
				ret = jr3_filter((unsigned long int)arg, 3,1);
			else ret=-1;
			break;
		case IOCTL1_JR3_FILTER4:
			if (PCI_DEVICE_ID_JR3==0x3112)
				ret = jr3_filter((unsigned long int)arg, 4,1);
			else ret=-1;
			break;
		case IOCTL1_JR3_FILTER5:
			if (PCI_DEVICE_ID_JR3==0x3112)
				ret = jr3_filter((unsigned long int)arg, 5,1);
			else ret=-1;
			break;
		case IOCTL1_JR3_FILTER6:
			if (PCI_DEVICE_ID_JR3==0x3112)
				ret = jr3_filter((unsigned long int)arg, 6,1);
			else ret=-1;
			break;
		case IOCTL1_JR3_GET_FULL_SCALES:
			if (PCI_DEVICE_ID_JR3==0x3112)
				ret = jr3_get_full_scales((unsigned long int)arg,1);
			else ret=-1;
			break;
		default:
			return -ENOTTY;
	}
	return ret;
}


static int show_copyright(short card) {
	int i;
	char copyright[19];
	char units_str[16];
	int base=0x6040;
	short day, year, units;
	
	for (i=0;i<18;i++) {
		copyright[i]=(char)(read_data(base,card) >> 8);
		base++;
	}
	copyright[18]='\0';
	day=read_data(0x60f6,card);
	year=read_data(0x60f7,card);
	units=read_data(0x60fc,card);
	if (units==0) {
		strncpy(units_str,"lbs\0",4);
	} else if (units==1) {
		strncpy(units_str,"Newtons\0",8);
	} else if (units==2) {
		strncpy(units_str,"Kilograms-force\0",16);
	} else if (units==3) {
		strncpy(units_str,"1000 lbs\0",8);
	} else {
		sprintf(units_str,"Unknown (id %d)",units);
	}
	rtdm_printk("jr3pci(%d): %s\n",card,copyright);
	rtdm_printk("jr3pci(%d): DSP Software updated day %d, year %d. Units: %s\n",card,day,year,units_str);
	return 0;
}

/** Download code to DSP */
static int jr3pci_init_dsp(int card) {
	int i=0;
	int count,address,value, value2;
	
	count=jr3_firmware[i++];
	while (count != 0xffff) {
		address=jr3_firmware[i++];
		rtdm_printk("jr3pci(%d): DSP Code. File pos. %d, Address 0x%x, %d times\n", card, i, address,count);
		
		while (count>0) {
			if (address & JR3_PD_DEVIDER) {
				value=jr3_firmware[i++];
				write_data(address,value,card);
				if (value!=read_data(address,card)) {
					rtdm_printk("data write error at address 0x%x\n", address);
				}
				count--;	
			} else {
				value=jr3_firmware[i++];
				value2=jr3_firmware[i++];
				write_program(address,value,value2,card);
				if ( ((value << 8) | (value2 & 0x00ff)) != read_program(address, card) ) {
					rtdm_printk("program write error at address 0x%x\n", address);
				}
				count-=2;
			}
			address++;
		}
		count=jr3_firmware[i++];
	}
	return 0;
}

/** pci probe function */
int jr3pci_probe(void) {
	int err;
	struct pci_dev *pci=NULL;

	pci=pci_get_device(PCI_VENDOR_ID_JR3, PCI_DEVICE_ID_JR3,pci);

	if (pci)
	{
		err = pci_enable_device(pci);
		if (err == 0)
		{
			memregion = pci_resource_start(pci, 0);
			size = pci_resource_len(pci,0);
			if (check_mem_region(memregion,size))
			{
				rtdm_printk("jr3pci: memory already in use\n");
				return -EBUSY;
			}

			if (request_mem_region(memregion,size,"JR3pci"))
			{
				jr3_base_address=ioremap(memregion,size);
				rtdm_printk("jr3pci: memory mapped successfully\n");
				err=0;
			}
		}
		else
		{
			rtdm_printk("jr3pci: pci_enable_device failed %d\n", err);
			return err;
		}
	}
	else
	{
		return -ENODEV;
	}

	return err;
}

//pci remove function
void jr3pci_remove(void) {
	iounmap(jr3_base_address);
	release_mem_region(memregion,size);
	rtdm_printk( "jr3pci: Module unloaded\n" );
}

void jr3_init(void)
{
	//Reset DSP
	write_data(JR3_RESET_ADDRESS,0,0);
	if (PCI_DEVICE_ID_JR3==0x3112)
		write_data(JR3_RESET_ADDRESS,0,1);

	//Download DSP code
	jr3pci_init_dsp(0);
	if (PCI_DEVICE_ID_JR3==0x3112)
		jr3pci_init_dsp(1);

	show_copyright(0);
	if (PCI_DEVICE_ID_JR3==0x3112)
		show_copyright(1);
}

static struct rtdm_device device = {
	.struct_version = RTDM_DEVICE_STRUCT_VER,

	.device_flags = RTDM_NAMED_DEVICE,
	.context_size = sizeof(buffer_t),
	.device_name = JR3_DEVICE_NAME,

	.open_nrt = jr3_open,
	.open_rt = jr3_open,

	.ops = {
		.close_nrt = jr3_close,
		.close_rt = jr3_close,
		.ioctl_nrt = jr3_ioctl,
		.ioctl_rt = jr3_ioctl,
	},

	.device_class = RTDM_CLASS_EXPERIMENTAL,
	.device_sub_class = SOME_SUB_CLASS,
	.profile_version = 1,
	.driver_name = "jr3-rtdm",
	.driver_version = RTDM_DRIVER_VER(1, 0, 0),
	.peripheral_name = "jr3 sensor",
	.provider_name = "Wojciech Domski",
	.proc_name = device.device_name,
};

int jr3pci_init_module(void)
{
	int err;
	
	err = rtdm_dev_register(&device);

	if(err < 0)
		goto rtdm_dev_register_err;

	rtdm_printk( "jr3pci: %s by %s\n", JR3_DESC, JR3_AUTHOR);
	
	err = jr3pci_probe();

	if( err < 0)
		goto jr3pci_probe_err;

	rtdm_printk( "jr3pci: JR3 PCI card detected at 0x%lx\n", (long int)jr3_base_address);

	jr3_init();
	
	rtdm_printk( "jr3pci: DSP code downloaded!! You can start playing  :)\n");

	return err;

jr3pci_probe_err:
	rtdm_printk( "jr3pci: No devices found\n");
	rtdm_dev_unregister(&device, 100);

rtdm_dev_register_err:

	return err;
}

void jr3pci_exit_module(void)
{
	jr3pci_remove();

	rtdm_dev_unregister(&device, 1000);

	rtdm_printk( "jr3pci: driver removed\n");
}

module_init(jr3pci_init_module);
module_exit(jr3pci_exit_module);
