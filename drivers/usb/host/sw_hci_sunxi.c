/*
 * drivers/usb/host/sw_hci_sunxi.c
 *
 * (C) Copyright 2007-2012
 * Allwinner Technology Co., Ltd. <www.allwinnertech.com>
 *
 * Author: javen
 * History:
 *    <author>          <time>          <version>               <desc>
 *    yangnaitian      2011-5-24            1.0          create this file
 *    javen            2011-7-18            1.1          添加了时钟开关和供电开关
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/ioport.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/timer.h>
#include <linux/list.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/clk.h>

#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>

#include <asm/byteorder.h>
#include <asm/irq.h>
#include <asm/system.h>
#include <asm/unaligned.h>
#include <mach/irqs.h>

#include <mach/clock.h>
#include <mach/platform.h>
#include <plat/system.h>
#include <plat/sys_config.h>

#include "sw_hci_sunxi.h"

#define SUNXI_USB_DMA_ALIGN ARCH_DMA_MINALIGN

static char *usbc_name[3] = { "usbc0", "usbc1", "usbc2" };
static char *usbc_ahb_ehci_name[3] = { "", "ahb_ehci0", "ahb_ehci1" };
static char *usbc_ahb_ohci_name[3] = { "", "ahb_ohci0", "ahb_ohci1" };
static char *usbc_phy_gate_name[3] = { "usb_phy", "usb_phy", "usb_phy" };
static char *ohci_phy_gate_name[3] = { "", "usb_ohci0", "usb_ohci1" };
static char *usbc_phy_reset_name[3] = { "usb_phy0", "usb_phy1", "usb_phy2" };

static u32 usbc_base[3] = {
	SW_VA_USB0_IO_BASE, SW_VA_USB1_IO_BASE, SW_VA_USB2_IO_BASE
};
static u32 ehci_irq_no[3] = { 0, SW_INT_SRC_EHCI0, SW_INT_SRC_EHCI1 };
static u32 ohci_irq_no[3] = { 0, SW_INT_SRC_OHCI0, SW_INT_SRC_OHCI1 };

static u32 usb1_set_vbus_cnt;
static u32 usb2_set_vbus_cnt;

static s32 get_usb_cfg(struct sw_hci_hcd *sw_hci)
{
	__s32 ret = 0;

	/* usbc enable */
	ret = script_parser_fetch(usbc_name[sw_hci->usbc_no], "usb_used",
				  (int *)&sw_hci->used, 64);
	if (ret != 0) {
		DMSG_PANIC("ERR: get usbc2 enable failed\n");
		/*return -1;*/
	}

	/* request gpio */
	ret = script_parser_fetch(usbc_name[sw_hci->usbc_no],
				  "usb_drv_vbus_gpio",
				  (int *)&sw_hci->drv_vbus_gpio_set, 64);
	if (ret != 0) {
		DMSG_PANIC("ERR: get usbc%d(%s) id failed\n", sw_hci->usbc_no,
			   usbc_name[sw_hci->usbc_no]);
		return -1;
	}

	/* host_init_state */
	ret = script_parser_fetch(usbc_name[sw_hci->usbc_no],
				  "usb_host_init_state",
				  (int *)&(sw_hci->host_init_state), 64);
	if (ret != 0) {
		DMSG_PANIC("ERR: script_parser_fetch host_init_state failed\n");
		return -1;
	}

	return 0;
}

static __u32 USBC_Phy_GetCsr(__u32 usbc_no)
{
	__u32 val = 0x0;

	/*
	 * XXX: TODO: Check if this really is correct, function is returning
	 * same 'val' for usbc_no == 0,1,2 !??!?!?!
	 *
	 * Maybe this should use SW_VA_USB1_IO_BASE for usbc_no==1 and
	 * SW_VA_USB2_IO_BASE for usbc_no==2?
	 */

	switch (usbc_no) {
	case 0:
		val = SW_VA_USB0_IO_BASE + 0x404;
		break;

	case 1:
		val = SW_VA_USB0_IO_BASE + 0x404;
		break;

	case 2:
		val = SW_VA_USB0_IO_BASE + 0x404;
		break;

	default:
		break;
	}

	return val;
}

static __u32 USBC_Phy_Write(__u32 usbc_no, __u32 addr, __u32 data, __u32 len)
{
	__u32 temp = 0, dtmp = 0;
	__u32 j = 0;

	dtmp = data;
	for (j = 0; j < len; j++) {
		/* set  the bit address to be write */
		temp = USBC_Readl(USBC_Phy_GetCsr(usbc_no));
		temp &= ~(0xff << 8);
		temp |= ((addr + j) << 8);
		USBC_Writel(temp, USBC_Phy_GetCsr(usbc_no));

		temp = USBC_Readb(USBC_Phy_GetCsr(usbc_no));
		temp &= ~(0x1 << 7);
		temp |= (dtmp & 0x1) << 7;
		temp &= ~(0x1 << (usbc_no << 1));
		USBC_Writeb(temp, USBC_Phy_GetCsr(usbc_no));

		temp = USBC_Readb(USBC_Phy_GetCsr(usbc_no));
		temp |= (0x1 << (usbc_no << 1));
		USBC_Writeb(temp, USBC_Phy_GetCsr(usbc_no));

		temp = USBC_Readb(USBC_Phy_GetCsr(usbc_no));
		temp &= ~(0x1 << (usbc_no << 1));
		USBC_Writeb(temp, USBC_Phy_GetCsr(usbc_no));
		dtmp >>= 1;
	}

	return data;
}

static void UsbPhyInit(__u32 usbc_no)
{
/*	DMSG_INFO("csr1: usbc%d: 0x%x\n", usbc_no, (u32)USBC_Readl(USBC_Phy_GetCsr(usbc_no))); */

	/* 调节45欧阻抗 */
	if (usbc_no == 0)
		USBC_Phy_Write(usbc_no, 0x0c, 0x01, 1);

/*	DMSG_INFO("csr2-0: usbc%d: 0x%x\n", usbc_no, (u32)USBC_Phy_Read(usbc_no, 0x0c, 1)); */

	/* 调整 USB0 PHY 的幅度和速率 */
	USBC_Phy_Write(usbc_no, 0x20, 0x14, 5);

/*	DMSG_INFO("csr2-1: usbc%d: 0x%x\n", usbc_no, (u32)USBC_Phy_Read(usbc_no, 0x20, 5)); */

	/* 调节 disconnect 域值 */
	USBC_Phy_Write(usbc_no, 0x2a, 3, 2);

/*	DMSG_INFO("csr2: usbc%d: 0x%x\n", usbc_no, (u32)USBC_Phy_Read(usbc_no, 0x2a, 2));
	DMSG_INFO("csr3: usbc%d: 0x%x\n", usbc_no, (u32)USBC_Readl(USBC_Phy_GetCsr(usbc_no))); */

	return;
}

static s32 clock_init(struct sw_hci_hcd *sw_hci, u32 ohci)
{
	if (ohci) {		/* ohci */
		sw_hci->sie_clk =
		    clk_get(NULL, usbc_ahb_ohci_name[sw_hci->usbc_no]);
		if (IS_ERR(sw_hci->sie_clk)) {
			DMSG_PANIC("ERR: get ohci%d abh clk failed.\n",
				   (sw_hci->usbc_no));
			goto failed;
		}

		sw_hci->ohci_gate =
		    clk_get(NULL, ohci_phy_gate_name[sw_hci->usbc_no]);
		if (IS_ERR(sw_hci->ohci_gate)) {
			DMSG_PANIC("ERR: get ohci%d gate clk failed.\n",
				   (sw_hci->usbc_no));
			goto failed;
		}
	} else {		/* ehci */
		sw_hci->sie_clk =
		    clk_get(NULL, usbc_ahb_ehci_name[sw_hci->usbc_no]);
		if (IS_ERR(sw_hci->sie_clk)) {
			DMSG_PANIC("ERR: get ehci%d abh clk failed.\n",
				   (sw_hci->usbc_no));
			goto failed;
		}
	}

	sw_hci->phy_gate = clk_get(NULL, usbc_phy_gate_name[sw_hci->usbc_no]);
	if (IS_ERR(sw_hci->phy_gate)) {
		DMSG_PANIC("ERR: get usb%d phy_gate failed.\n",
			   sw_hci->usbc_no);
		goto failed;
	}

	sw_hci->phy_reset = clk_get(NULL, usbc_phy_reset_name[sw_hci->usbc_no]);
	if (IS_ERR(sw_hci->phy_reset)) {
		DMSG_PANIC("ERR: get usb%d phy_reset failed.\n",
			   sw_hci->usbc_no);
		goto failed;
	}

	return 0;

failed:
	if (sw_hci->sie_clk) {
		clk_put(sw_hci->sie_clk);
		sw_hci->sie_clk = NULL;
	}

	if (sw_hci->phy_gate) {
		clk_put(sw_hci->phy_gate);
		sw_hci->phy_gate = NULL;
	}

	if (sw_hci->phy_reset) {
		clk_put(sw_hci->phy_reset);
		sw_hci->phy_reset = NULL;
	}

	if (sw_hci->ohci_gate) {
		clk_put(sw_hci->ohci_gate);
		sw_hci->ohci_gate = NULL;
	}

	return -1;
}

static s32 clock_exit(struct sw_hci_hcd *sw_hci, u32 ohci)
{
	if (sw_hci->ohci_gate) {
		clk_put(sw_hci->ohci_gate);
		sw_hci->ohci_gate = NULL;
	}

	if (sw_hci->sie_clk) {
		clk_put(sw_hci->sie_clk);
		sw_hci->sie_clk = NULL;
	}

	if (sw_hci->phy_gate) {
		clk_put(sw_hci->phy_gate);
		sw_hci->phy_gate = NULL;
	}

	if (sw_hci->phy_reset) {
		clk_put(sw_hci->phy_reset);
		sw_hci->phy_reset = NULL;
	}

	return 0;
}

static int open_clock(struct sw_hci_hcd *sw_hci, u32 ohci)
{
	DMSG_INFO("[%s]: open clock\n", sw_hci->hci_name);

	if (sw_hci->sie_clk && sw_hci->phy_gate
	    && sw_hci->phy_reset && !sw_hci->clk_is_open) {
		sw_hci->clk_is_open = 1;

		clk_enable(sw_hci->phy_gate);
		clk_enable(sw_hci->phy_reset);
		clk_reset(sw_hci->phy_reset, 0);

		if (ohci && sw_hci->ohci_gate)
			clk_enable(sw_hci->ohci_gate);

		mdelay(10);

		clk_enable(sw_hci->sie_clk);

		mdelay(10);

		UsbPhyInit(sw_hci->usbc_no);
	} else {
		DMSG_PANIC
		    ("[%s]: wrn: open clock failed, (0x%p, 0x%p, 0x%p, %d, 0x%p)\n",
		     sw_hci->hci_name, sw_hci->sie_clk, sw_hci->phy_gate,
		     sw_hci->phy_reset, sw_hci->clk_is_open, sw_hci->ohci_gate);
	}

	DMSG_DEBUG("[%s]: open clock, 0x60(0x%x), 0xcc(0x%x)\n",
		   sw_hci->hci_name,
		   (u32) USBC_Readl(SW_VA_CCM_IO_BASE + 0x60),
		   (u32) USBC_Readl(SW_VA_CCM_IO_BASE + 0xcc));

	return 0;
}

static int close_clock(struct sw_hci_hcd *sw_hci, u32 ohci)
{
	DMSG_INFO("[%s]: close clock\n", sw_hci->hci_name);

	if (sw_hci->sie_clk && sw_hci->phy_gate
	    && sw_hci->phy_reset && sw_hci->clk_is_open) {

		sw_hci->clk_is_open = 0;

		if (ohci && sw_hci->ohci_gate)
			clk_disable(sw_hci->ohci_gate);

		clk_reset(sw_hci->phy_reset, 1);
		clk_disable(sw_hci->phy_reset);
		clk_disable(sw_hci->phy_gate);

		clk_disable(sw_hci->sie_clk);
	} else {
		DMSG_PANIC
		    ("[%s]: wrn: open clock failed, (0x%p, 0x%p, 0x%p, %d, 0x%p)\n",
		     sw_hci->hci_name, sw_hci->sie_clk, sw_hci->phy_gate,
		     sw_hci->phy_reset, sw_hci->clk_is_open, sw_hci->ohci_gate);
	}

	DMSG_DEBUG("[%s]: close clock, 0x60(0x%x), 0xcc(0x%x)\n",
		   sw_hci->hci_name,
		   (u32) USBC_Readl(SW_VA_CCM_IO_BASE + 0x60),
		   (u32) USBC_Readl(SW_VA_CCM_IO_BASE + 0xcc));

	return 0;
}

static void usb_passby(struct sw_hci_hcd *sw_hci, u32 enable)
{
	unsigned long reg_value = 0;
	static DEFINE_SPINLOCK(lock);
	unsigned long flags = 0;

	spin_lock_irqsave(&lock, flags);

	/*enable passby */
	if (enable) {
		reg_value =
		    USBC_Readl(sw_hci->usb_vbase + SW_USB_PMU_IRQ_ENABLE);
		reg_value |= (1 << 10);	/* AHB Master interface INCR8 enable */
		reg_value |= (1 << 9);	/* AHB Master interface burst type
					   INCR4 enable */
		reg_value |= (1 << 8);	/* AHB Master interface INCRX align
					   enable */
		reg_value |= (1 << 0);	/* ULPI bypass enable */
		USBC_Writel(reg_value,
			    (sw_hci->usb_vbase + SW_USB_PMU_IRQ_ENABLE));
	} else {
		reg_value =
		    USBC_Readl(sw_hci->usb_vbase + SW_USB_PMU_IRQ_ENABLE);
		reg_value &= ~(1 << 10);/* AHB Master interface INCR8 disable */
		reg_value &= ~(1 << 9);	/* AHB Master interface burst type
					   INCR4 disable */
		reg_value &= ~(1 << 8);	/* AHB Master interface INCRX align
					   disable */
		reg_value &= ~(1 << 0);	/* ULPI bypass disable */
		USBC_Writel(reg_value,
			    (sw_hci->usb_vbase + SW_USB_PMU_IRQ_ENABLE));
	}

	spin_unlock_irqrestore(&lock, flags);

	return;
}

static void hci_port_configure(struct sw_hci_hcd *sw_hci, u32 enable)
{
	unsigned long reg_value = 0;
	u32 usbc_sdram_hpcr = 0;

	if (sw_hci->usbc_no == 1) {
		usbc_sdram_hpcr = SW_SDRAM_REG_HPCR_USB1;
	} else if (sw_hci->usbc_no == 2) {
		usbc_sdram_hpcr = SW_SDRAM_REG_HPCR_USB2;
	} else {
		DMSG_PANIC("EER: unkown usbc_no(%d)\n", sw_hci->usbc_no);
		return;
	}

	reg_value = USBC_Readl(sw_hci->sdram_vbase + usbc_sdram_hpcr);
	if (enable)
		reg_value |= (1 << SW_SDRAM_BP_HPCR_ACCESS_EN);
	else
		reg_value &= ~(1 << SW_SDRAM_BP_HPCR_ACCESS_EN);

	USBC_Writel(reg_value, (sw_hci->sdram_vbase + usbc_sdram_hpcr));

	return;
}

static u32 alloc_pin(user_gpio_set_t *gpio_list)
{
	u32 pin_handle = 0;

	pin_handle = sunxi_gpio_request_array(gpio_list, 1);
	if (pin_handle == 0) {
		DMSG_PANIC("ERR: gpio_request failed\n");
		return 0;
	}

	/* set config, ouput */
	printk("hci io status start\n");
	gpio_set_one_pin_io_status(pin_handle, 1, NULL);
	printk("hci io status end\n");

	/* reserved is pull down */
	printk("hci pull start\n");
	gpio_set_one_pin_pull(pin_handle, 2, NULL);
	printk("hci pull end\n");

	return pin_handle;
}

static void free_pin(u32 pin_handle)
{
	if (pin_handle)
		gpio_release(pin_handle, 0);

	return;
}

static void __sw_set_vbus(struct sw_hci_hcd *sw_hci, int is_on)
{
	u32 on_off = 0;

	DMSG_INFO("[%s]: Set USB Power %s\n", sw_hci->hci_name,
		  (is_on ? "ON" : "OFF"));

	/* set power flag */
	sw_hci->power_flag = is_on;

	/* set power */
	if (sw_hci->drv_vbus_gpio_set.data == 0)
		on_off = is_on ? 1 : 0;
	else
		on_off = is_on ? 0 : 1;

	printk("hci gpio_write_one_pin_value start\n");
	gpio_write_one_pin_value(sw_hci->drv_vbus_Handle, on_off, NULL);
	printk("hci gpio_write_one_pin_value end\n");

	return;
}

static void sw_set_vbus(struct sw_hci_hcd *sw_hci, int is_on)
{
	DMSG_DEBUG("[%s]: sw_set_vbus cnt %d\n",
		   sw_hci->hci_name,
		   (sw_hci->usbc_no ==
		    1) ? usb1_set_vbus_cnt : usb2_set_vbus_cnt);

	if (sw_hci->usbc_no == 1) {
		if (is_on && usb1_set_vbus_cnt == 0)
			__sw_set_vbus(sw_hci, is_on);	/* power on */
		else if (!is_on && usb1_set_vbus_cnt == 1)
			__sw_set_vbus(sw_hci, is_on);	/* power off */

		if (is_on)
			usb1_set_vbus_cnt++;
		else
			usb1_set_vbus_cnt--;
	} else {
		if (is_on && usb2_set_vbus_cnt == 0)
			__sw_set_vbus(sw_hci, is_on);	/* power on */
		else if (!is_on && usb2_set_vbus_cnt == 1)
			__sw_set_vbus(sw_hci, is_on);	/* power off */

		if (is_on)
			usb2_set_vbus_cnt++;
		else
			usb2_set_vbus_cnt--;
	}

	return;
}

struct temp_buffer {
	void *kmalloc_ptr;
	void *old_buffer;
	u8 data[];
};

static void *alloc_temp_buffer(size_t size, gfp_t mem_flags)
{
	struct temp_buffer *temp, *kmalloc_ptr;
	size_t kmalloc_size;

	kmalloc_size = size + sizeof(struct temp_buffer) +
			SUNXI_USB_DMA_ALIGN - 1;

	kmalloc_ptr = kmalloc(kmalloc_size, mem_flags);
	if (!kmalloc_ptr)
		return NULL;

	/* Position our struct temp_buffer such that data is aligned.
	 *
	 * Note: kmalloc_ptr is type 'struct temp_buffer *' and PTR_ALIGN
	 * returns pointer with the same type 'struct temp_buffer *'.
	 */
	temp = PTR_ALIGN(kmalloc_ptr + 1, SUNXI_USB_DMA_ALIGN) - 1;

	temp->kmalloc_ptr = kmalloc_ptr;
	return temp;
}

static void sunxi_hcd_free_temp_buffer(struct urb *urb)
{
	enum dma_data_direction dir;
	struct temp_buffer *temp;

	if (!(urb->transfer_flags & URB_ALIGNED_TEMP_BUFFER))
		return;

	dir = usb_urb_dir_in(urb) ? DMA_FROM_DEVICE : DMA_TO_DEVICE;

	temp = container_of(urb->transfer_buffer, struct temp_buffer, data);

	if (dir == DMA_FROM_DEVICE)
		memcpy(temp->old_buffer, temp->data,
		       urb->transfer_buffer_length);

	urb->transfer_buffer = temp->old_buffer;
	kfree(temp->kmalloc_ptr);

	urb->transfer_flags &= ~URB_ALIGNED_TEMP_BUFFER;
}

static int sunxi_hcd_alloc_temp_buffer(struct urb *urb, gfp_t mem_flags)
{
	enum dma_data_direction dir;
	struct temp_buffer *temp;

	if (urb->num_sgs)
		return 0;
	if (urb->sg)
		return 0;
	if (urb->transfer_buffer_length == 0)
		return 0;
	if (urb->transfer_flags & URB_NO_TRANSFER_DMA_MAP)
		return 0;

	/* sunxi hardware requires transfer buffers to be DMA aligned */
	if (!((uintptr_t)urb->transfer_buffer & (SUNXI_USB_DMA_ALIGN - 1)))
		return 0;

	/* Allocate a buffer with enough padding for alignment */
	temp = alloc_temp_buffer(urb->transfer_buffer_length, mem_flags);
	if (!temp)
		return -ENOMEM;

	dir = usb_urb_dir_in(urb) ? DMA_FROM_DEVICE : DMA_TO_DEVICE;

	temp->old_buffer = urb->transfer_buffer;
	if (dir == DMA_TO_DEVICE)
		memcpy(temp->data, urb->transfer_buffer,
		       urb->transfer_buffer_length);
	urb->transfer_buffer = temp->data;

	urb->transfer_flags |= URB_ALIGNED_TEMP_BUFFER;

	return 0;
}

static void sunxi_hcd_free_temp_setup(struct urb *urb)
{
	struct temp_buffer *temp;

	if (!(urb->transfer_flags & URB_ALIGNED_TEMP_SETUP))
		return;

	temp = container_of((void *)urb->setup_packet, struct temp_buffer,
			    data);

	urb->setup_packet = temp->old_buffer;
	kfree(temp->kmalloc_ptr);

	urb->transfer_flags &= ~URB_ALIGNED_TEMP_SETUP;
}

static int sunxi_hcd_alloc_temp_setup(struct urb *urb, gfp_t mem_flags)
{
	struct temp_buffer *temp;

	if (!usb_endpoint_xfer_control(&urb->ep->desc))
		return 0;

	/* sunxi hardware requires setup packet to be DMA aligned */
	if (!((uintptr_t)urb->setup_packet & (SUNXI_USB_DMA_ALIGN - 1)))
		return 0;

	/* Allocate a buffer with enough padding for alignment */
	temp = alloc_temp_buffer(sizeof(struct usb_ctrlrequest), mem_flags);
	if (!temp)
		return -ENOMEM;

	temp->old_buffer = urb->setup_packet;
	memcpy(temp->data, urb->setup_packet, sizeof(struct usb_ctrlrequest));
	urb->setup_packet = temp->data;

	urb->transfer_flags |= URB_ALIGNED_TEMP_SETUP;

	return 0;
}

int sunxi_hcd_map_urb_for_dma(struct usb_hcd *hcd, struct urb *urb,
				     gfp_t mem_flags)
{
	int ret;

	ret = sunxi_hcd_alloc_temp_buffer(urb, mem_flags);
	if (ret)
		return ret;

	ret = sunxi_hcd_alloc_temp_setup(urb, mem_flags);
	if (ret) {
		sunxi_hcd_free_temp_buffer(urb);
		return ret;
	}

	ret = usb_hcd_map_urb_for_dma(hcd, urb, mem_flags);
	if (ret) {
		sunxi_hcd_free_temp_setup(urb);
		sunxi_hcd_free_temp_buffer(urb);
		return ret;
	}

	return ret;
}
EXPORT_SYMBOL_GPL(sunxi_hcd_map_urb_for_dma);

void sunxi_hcd_unmap_urb_for_dma(struct usb_hcd *hcd, struct urb *urb)
{
	usb_hcd_unmap_urb_for_dma(hcd, urb);
	sunxi_hcd_free_temp_setup(urb);
	sunxi_hcd_free_temp_buffer(urb);
}
EXPORT_SYMBOL_GPL(sunxi_hcd_unmap_urb_for_dma);

/*
 *---------------------------------------------------------------
 * EHCI
 *---------------------------------------------------------------
 */

#define  SW_EHCI_NAME		"sw-ehci"
static const char ehci_name[] = SW_EHCI_NAME;

static struct sw_hci_hcd sw_ehci1;
static struct sw_hci_hcd sw_ehci2;

static u64 sw_ehci_dmamask = DMA_BIT_MASK(32);

static struct resource sw_ehci1_resources[] = {
		DEFINE_RES_MEM(SW_PA_USB1_IO_BASE + SW_USB_EHCI_BASE_OFFSET, SW_USB_EHCI_LEN),
		DEFINE_RES_IRQ(SW_INT_SRC_EHCI0)
};

static struct resource sw_ehci2_resources[] = {
		DEFINE_RES_MEM(SW_PA_USB2_IO_BASE + SW_USB_EHCI_BASE_OFFSET, SW_USB_EHCI_LEN),
		DEFINE_RES_IRQ(SW_INT_SRC_EHCI1)
};

static struct platform_device sw_usb_ehci_device[] = {
	[0] = {
	       .name = ehci_name,
	       .id = 1,
	       .dev = {
		       .dma_mask = &sw_ehci_dmamask,
		       .coherent_dma_mask = DMA_BIT_MASK(32),
		       .platform_data = &sw_ehci1,
		       },
	       .resource = sw_ehci1_resources,
	       .num_resources = ARRAY_SIZE(sw_ehci1_resources),
	       },

	[1] = {
	       .name = ehci_name,
	       .id = 2,
	       .dev = {
		       .dma_mask = &sw_ehci_dmamask,
		       .coherent_dma_mask = DMA_BIT_MASK(32),
		       .platform_data = &sw_ehci2,
		       },
	       .resource = sw_ehci2_resources,
	       .num_resources = ARRAY_SIZE(sw_ehci2_resources),
	       },
};

/*
 *---------------------------------------------------------------
 * OHCI
 *---------------------------------------------------------------
 */

#define  SW_OHCI_NAME		"sw-ohci"
static const char ohci_name[] = SW_OHCI_NAME;

static struct sw_hci_hcd sw_ohci1;
static struct sw_hci_hcd sw_ohci2;

static u64 sw_ohci_dmamask = DMA_BIT_MASK(32);

static struct resource sw_ohci1_resources[] = {
		DEFINE_RES_MEM(SW_PA_USB1_IO_BASE + SW_USB_OHCI_BASE_OFFSET, SW_USB_OHCI_LEN),
		DEFINE_RES_IRQ(SW_INT_SRC_OHCI0)
};

static struct resource sw_ohci2_resources[] = {
		DEFINE_RES_MEM(SW_PA_USB2_IO_BASE + SW_USB_OHCI_BASE_OFFSET, SW_USB_OHCI_LEN),
		DEFINE_RES_IRQ(SW_INT_SRC_OHCI1)
};

static struct platform_device sw_usb_ohci_device[] = {
	[0] = {
	       .name = ohci_name,
	       .id = 1,
	       .dev = {
		       .dma_mask = &sw_ohci_dmamask,
		       .coherent_dma_mask = DMA_BIT_MASK(32),
		       .platform_data = &sw_ohci1,
		       },
	       .resource = sw_ohci1_resources,
	       .num_resources = ARRAY_SIZE(sw_ohci1_resources),
	       },
	[1] = {
	       .name = ohci_name,
	       .id = 2,
	       .dev = {
		       .dma_mask = &sw_ohci_dmamask,
		       .coherent_dma_mask = DMA_BIT_MASK(32),
		       .platform_data = &sw_ohci2,
		       },
	       .resource = sw_ohci2_resources,
	       .num_resources = ARRAY_SIZE(sw_ohci2_resources),
	       },
};

static void print_sw_hci(struct sw_hci_hcd *sw_hci)
{
	DMSG_DEBUG("\n------%s config------\n", sw_hci->hci_name);
	DMSG_DEBUG("hci_name             = %s\n", sw_hci->hci_name);
	DMSG_DEBUG("irq_no               = %d\n", sw_hci->irq_no);
	DMSG_DEBUG("usbc_no              = %d\n", sw_hci->usbc_no);

	DMSG_DEBUG("usb_vbase            = 0x%p\n", sw_hci->usb_vbase);
	DMSG_DEBUG("sram_vbase           = 0x%p\n", sw_hci->sram_vbase);
	DMSG_DEBUG("clock_vbase          = 0x%p\n", sw_hci->clock_vbase);
	DMSG_DEBUG("sdram_vbase          = 0x%p\n", sw_hci->sdram_vbase);

	DMSG_DEBUG("used                 = %d\n", sw_hci->used);
	DMSG_DEBUG("host_init_state      = %d\n", sw_hci->host_init_state);

	DMSG_DEBUG("gpio_name            = %s\n",
		   sw_hci->drv_vbus_gpio_set.gpio_name);
	DMSG_DEBUG("port                 = %d\n",
		   sw_hci->drv_vbus_gpio_set.port);
	DMSG_DEBUG("port_num             = %d\n",
		   sw_hci->drv_vbus_gpio_set.port_num);
	DMSG_DEBUG("mul_sel              = %d\n",
		   sw_hci->drv_vbus_gpio_set.mul_sel);
	DMSG_DEBUG("pull                 = %d\n",
		   sw_hci->drv_vbus_gpio_set.pull);
	DMSG_DEBUG("drv_level            = %d\n",
		   sw_hci->drv_vbus_gpio_set.drv_level);
	DMSG_DEBUG("data                 = %d\n",
		   sw_hci->drv_vbus_gpio_set.data);

	DMSG_DEBUG("\n--------------------------\n");

	return;
}

static int init_sw_hci(struct sw_hci_hcd *sw_hci, u32 usbc_no, u32 ohci,
		       const char *hci_name)
{
	s32 ret = 0;

	memset(sw_hci, 0, sizeof(struct sw_hci_hcd));

	sw_hci->usbc_no = usbc_no;

	if (ohci)
		sw_hci->irq_no = ohci_irq_no[sw_hci->usbc_no];
	else
		sw_hci->irq_no = ehci_irq_no[sw_hci->usbc_no];

	sprintf(sw_hci->hci_name, "%s%d", hci_name, sw_hci->usbc_no);

	sw_hci->usb_vbase = (void __iomem *)usbc_base[sw_hci->usbc_no];
	sw_hci->sram_vbase = (void __iomem *)SW_VA_SRAM_IO_BASE;
	sw_hci->clock_vbase = (void __iomem *)SW_VA_CCM_IO_BASE;
	sw_hci->gpio_vbase = (void __iomem *)SW_VA_PORTC_IO_BASE;
	sw_hci->sdram_vbase = (void __iomem *)SW_VA_DRAM_IO_BASE;

	get_usb_cfg(sw_hci);
	sw_hci->open_clock = open_clock;
	sw_hci->close_clock = close_clock;
	sw_hci->set_power = sw_set_vbus;
	sw_hci->usb_passby = usb_passby;
	sw_hci->port_configure = hci_port_configure;

	ret = clock_init(sw_hci, ohci);
	if (ret != 0) {
		DMSG_PANIC("ERR: clock_init failed\n");
		goto failed1;
	}

	print_sw_hci(sw_hci);

	return 0;

failed1:
	return -1;
}

static int exit_sw_hci(struct sw_hci_hcd *sw_hci, u32 ohci)
{
	clock_exit(sw_hci, ohci);

	return 0;
}

static int __init sw_hci_sunxi_init(void)
{
/* XXX Should be rewtitten with checks if CONFIG_USB_EHCI_HCD or CONFIG_USB_OHCI_HCD
       are actually defined. Original code assumes that EHCI is always on.
*/
	u32 usb1_drv_vbus_Handle = 0;
	u32 usb2_drv_vbus_Handle = 0;

	init_sw_hci(&sw_ehci1, 1, 0, ehci_name);
	init_sw_hci(&sw_ohci1, 1, 1, ohci_name);

	usb1_drv_vbus_Handle = alloc_pin(&sw_ehci1.drv_vbus_gpio_set);
	if (usb1_drv_vbus_Handle == 0) {
		DMSG_PANIC("ERR: usb1 alloc_pin failed\n");
		goto failed0;
	}

	sw_ehci1.drv_vbus_Handle = usb1_drv_vbus_Handle;
	sw_ohci1.drv_vbus_Handle = usb1_drv_vbus_Handle;

	if (sunxi_is_sun4i() || sunxi_is_sun7i()) {
		/* A13 has only one *HCI USB controller */
		init_sw_hci(&sw_ehci2, 2, 0, ehci_name);
		init_sw_hci(&sw_ohci2, 2, 1, ohci_name);

		usb2_drv_vbus_Handle = alloc_pin(&sw_ehci2.drv_vbus_gpio_set);
		if (usb2_drv_vbus_Handle == 0) {
			DMSG_PANIC("ERR: usb2 alloc_pin failed\n");
			goto failed0;
		}

		sw_ehci2.drv_vbus_Handle = usb2_drv_vbus_Handle;
		sw_ohci2.drv_vbus_Handle = usb2_drv_vbus_Handle;
	} else {
		sw_ehci2.used = 0;
	}

/* XXX '.used' flag is for USB port, not for EHCI or OHCI. So it can be checked this way */
	if (sw_ehci1.used) {
		platform_device_register(&sw_usb_ehci_device[0]);
		platform_device_register(&sw_usb_ohci_device[0]);
	} else {
/*      DMSG_PANIC("ERR: usb%d %s is disabled in script.bin\n", sw_ehci1.usbc_no, sw_ehci1.hci_name); */
	}

	if (sw_ehci2.used) {
		platform_device_register(&sw_usb_ehci_device[1]);
		platform_device_register(&sw_usb_ohci_device[1]);
	} else {
/*      DMSG_PANIC("ERR: usb%d %s is disabled in script.bin\n", sw_ehci2.usbc_no, sw_ehci2.hci_name); */
	}

	return 0;

failed0:
	return -1;
}

static void __exit sw_hci_sunxi_exit(void)
{
/* XXX '.used' flag is for USB port, not for EHCI or OHCI. So it can be checked this way */
	if (sw_ehci1.used) {
		platform_device_unregister(&sw_usb_ehci_device[0]);
		platform_device_unregister(&sw_usb_ohci_device[0]);

		exit_sw_hci(&sw_ehci1, 0);
		exit_sw_hci(&sw_ohci1, 1);

		free_pin(sw_ehci1.drv_vbus_Handle);
	}

	if (sw_ehci2.used) {
		platform_device_unregister(&sw_usb_ehci_device[1]);
		platform_device_unregister(&sw_usb_ohci_device[1]);

		exit_sw_hci(&sw_ehci2, 0);
		exit_sw_hci(&sw_ohci2, 1);

		free_pin(sw_ehci2.drv_vbus_Handle);
	}

	return;
}

module_init(sw_hci_sunxi_init);
module_exit(sw_hci_sunxi_exit);
