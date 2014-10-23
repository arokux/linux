#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/usb/of.h>
#include <linux/of_gpio.h>
#include <linux/usb/musb.h>
#include <linux/usb/usb_phy_generic.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>
#include <linux/dma-mapping.h>

#include "musb_core.h"

/* reg offsets */
#define  USBC_REG_o_ISCR	0x0400
#define  USBC_REG_o_PHYCTL	0x0404
#define  USBC_REG_o_PHYBIST	0x0408
#define  USBC_REG_o_PHYTUNE	0x040c

#define  USBC_REG_o_VEND0	0x0043

/* Interface Status and Control */
#define  USBC_BP_ISCR_VBUS_VALID_FROM_DATA	30
#define  USBC_BP_ISCR_VBUS_VALID_FROM_VBUS	29
#define  USBC_BP_ISCR_EXT_ID_STATUS		28
#define  USBC_BP_ISCR_EXT_DM_STATUS		27
#define  USBC_BP_ISCR_EXT_DP_STATUS		26
#define  USBC_BP_ISCR_MERGED_VBUS_STATUS	25
#define  USBC_BP_ISCR_MERGED_ID_STATUS		24

#define  USBC_BP_ISCR_ID_PULLUP_EN		17
#define  USBC_BP_ISCR_DPDM_PULLUP_EN		16
#define  USBC_BP_ISCR_FORCE_ID			14
#define  USBC_BP_ISCR_FORCE_VBUS_VALID		12
#define  USBC_BP_ISCR_VBUS_VALID_SRC		10

#define  USBC_BP_ISCR_HOSC_EN			7
#define  USBC_BP_ISCR_VBUS_CHANGE_DETECT	6
#define  USBC_BP_ISCR_ID_CHANGE_DETECT		5
#define  USBC_BP_ISCR_DPDM_CHANGE_DETECT	4
#define  USBC_BP_ISCR_IRQ_ENABLE		3
#define  USBC_BP_ISCR_VBUS_CHANGE_DETECT_EN	2
#define  USBC_BP_ISCR_ID_CHANGE_DETECT_EN	1
#define  USBC_BP_ISCR_DPDM_CHANGE_DETECT_EN	0

/* usb id type */
#define  USBC_ID_TYPE_DISABLE		0
#define  USBC_ID_TYPE_HOST		1
#define  USBC_ID_TYPE_DEVICE		2

/* usb vbus valid type */
#define  USBC_VBUS_TYPE_DISABLE		0
#define  USBC_VBUS_TYPE_LOW		1
#define  USBC_VBUS_TYPE_HIGH		2

/* usb io type */
#define  USBC_IO_TYPE_PIO		0
#define  USBC_IO_TYPE_DMA		1

/* usb ep type */
#define  USBC_EP_TYPE_IDLE		0
#define  USBC_EP_TYPE_EP0		1
#define  USBC_EP_TYPE_TX		2
#define  USBC_EP_TYPE_RX		3

/* vendor0 */
#define  USBC_BP_VEND0_DRQ_SEL		1
#define  USBC_BP_VEND0_BUS_SEL		0

struct sunxi_glue {
	struct device		*dev;
	struct platform_device	*musb;
	struct clk		*clk;
	struct regulator	*vbus;
	struct regmap		*sc;
	int			id_det_gpio;
	int			vbus_gpio;
	int			id_det_irq;
	int			vbus_irq;
};

static irqreturn_t sunxi_musb_interrupt(int irq, void *__hci)
{
	unsigned long   flags;
	irqreturn_t     retval = IRQ_NONE;
	struct musb     *musb = __hci;

	spin_lock_irqsave(&musb->lock, flags);

	/* read and clear interrupts 
	 * NOTE: clearing is necessary!
	 */
	musb->int_usb = musb_readb(musb->mregs, MUSB_INTRUSB);
	if (musb->int_usb)
		musb_writeb(musb->mregs, MUSB_INTRUSB, musb->int_usb);

	musb->int_tx = musb_readw(musb->mregs, MUSB_INTRTX);
	if (musb->int_tx)
		musb_writew(musb->mregs, MUSB_INTRTX, musb->int_tx);

	musb->int_rx = musb_readw(musb->mregs, MUSB_INTRRX);
	if (musb->int_rx)
		musb_writew(musb->mregs, MUSB_INTRRX, musb->int_rx);

	if (musb->int_usb || musb->int_tx || musb->int_rx)
		retval |= musb_interrupt(musb);

	spin_unlock_irqrestore(&musb->lock, flags);

	return retval;
}

static u32 __USBC_WakeUp_ClearChangeDetect(u32 reg_val)
{
	u32 temp = reg_val;

	temp &= ~(1 << USBC_BP_ISCR_VBUS_CHANGE_DETECT);
	temp &= ~(1 << USBC_BP_ISCR_ID_CHANGE_DETECT);
	temp &= ~(1 << USBC_BP_ISCR_DPDM_CHANGE_DETECT);

	return temp;
}

void USBC_EnableIdPullUp(__iomem void *base)
{
	u32 reg_val;

	reg_val = musb_readl(base, USBC_REG_o_ISCR);
	reg_val |= (1 << USBC_BP_ISCR_ID_PULLUP_EN);
	reg_val = __USBC_WakeUp_ClearChangeDetect(reg_val);
	musb_writel(base, USBC_REG_o_ISCR, reg_val);
}

void USBC_DisableIdPullUp(__iomem void *base)
{
	u32 reg_val;

	reg_val = musb_readl(base, USBC_REG_o_ISCR);
	reg_val &= ~(1 << USBC_BP_ISCR_ID_PULLUP_EN);
	reg_val = __USBC_WakeUp_ClearChangeDetect(reg_val);
	musb_writel(base, USBC_REG_o_ISCR, reg_val);
}

static void USBC_EnableDpDmPullUp(__iomem void *base)
{
	u32 reg_val;

	reg_val = musb_readl(base, USBC_REG_o_ISCR);
	reg_val |= (1 << USBC_BP_ISCR_DPDM_PULLUP_EN);
	reg_val = __USBC_WakeUp_ClearChangeDetect(reg_val);
	musb_writel(base, USBC_REG_o_ISCR, reg_val);
}

static void __USBC_ForceVbusValidDisable(__iomem void *base)
{
	u32 reg_val;

	reg_val = musb_readl(base, USBC_REG_o_ISCR);
	reg_val &= ~(0x03 << USBC_BP_ISCR_FORCE_VBUS_VALID);
	reg_val = __USBC_WakeUp_ClearChangeDetect(reg_val);
	musb_writel(base, USBC_REG_o_ISCR, reg_val);
}

static void __USBC_ForceVbusValidToLow(__iomem void *base)
{
	u32 reg_val;

	reg_val = musb_readl(base, USBC_REG_o_ISCR);
	reg_val &= ~(0x03 << USBC_BP_ISCR_FORCE_VBUS_VALID);
	reg_val |= (0x02 << USBC_BP_ISCR_FORCE_VBUS_VALID);
	reg_val = __USBC_WakeUp_ClearChangeDetect(reg_val);
	musb_writel(base, USBC_REG_o_ISCR, reg_val);
}

static void __USBC_ForceVbusValidToHigh(__iomem void *base)
{
	u32 reg_val;

	reg_val = musb_readl(base, USBC_REG_o_ISCR);
	reg_val &= ~(0x03 << USBC_BP_ISCR_FORCE_VBUS_VALID);
	reg_val |= (0x03 << USBC_BP_ISCR_FORCE_VBUS_VALID);
	reg_val = __USBC_WakeUp_ClearChangeDetect(reg_val);
	musb_writel(base, USBC_REG_o_ISCR, reg_val);
}

/* force vbus valid to (id_type) */
static void USBC_ForceVbusValid(__iomem void *base, u32 vbus_type)
{
	pr_debug("musb: %s(): vbus_type %s\n", __func__,
		vbus_type == USBC_VBUS_TYPE_LOW ? "low" :
		(vbus_type == USBC_VBUS_TYPE_HIGH ? "high" : "disable"));

	switch (vbus_type) {
	case USBC_VBUS_TYPE_LOW:
		__USBC_ForceVbusValidToLow(base);
		break;

	case USBC_VBUS_TYPE_HIGH:
		__USBC_ForceVbusValidToHigh(base);
		break;

	default:
		__USBC_ForceVbusValidDisable(base);
		break;
	}
}

static void __USBC_ForceIdDisable(__iomem void *base)
{
	u32 reg_val;

	reg_val = musb_readl(base, USBC_REG_o_ISCR);
	reg_val &= ~(0x03 << USBC_BP_ISCR_FORCE_ID);
	reg_val = __USBC_WakeUp_ClearChangeDetect(reg_val);
	musb_writel(base, USBC_REG_o_ISCR, reg_val);
}

static void __USBC_ForceIdToLow(__iomem void *base)
{
	u32 reg_val;

	reg_val = musb_readl(base, USBC_REG_o_ISCR);
	reg_val &= ~(0x03 << USBC_BP_ISCR_FORCE_ID);
	reg_val |= (0x02 << USBC_BP_ISCR_FORCE_ID);
	reg_val = __USBC_WakeUp_ClearChangeDetect(reg_val);
	musb_writel(base, USBC_REG_o_ISCR, reg_val);
}

static void __USBC_ForceIdToHigh(__iomem void *base)
{
	u32 reg_val;

	reg_val = musb_readl(base, USBC_REG_o_ISCR);
	reg_val &= ~(0x03 << USBC_BP_ISCR_FORCE_ID);
	reg_val |= (0x03 << USBC_BP_ISCR_FORCE_ID);
	reg_val = __USBC_WakeUp_ClearChangeDetect(reg_val);
	musb_writel(base, USBC_REG_o_ISCR, reg_val);
}

/* force id to (id_type) */
static void USBC_ForceId(__iomem void *base, u32 id_type)
{
	pr_debug("musb %s(): id_type %s\n", __func__,
		id_type == USBC_ID_TYPE_HOST ? "host" :
		(id_type == USBC_ID_TYPE_DEVICE ? "device" : "disable"));

	switch (id_type) {
	case USBC_ID_TYPE_HOST:
		__USBC_ForceIdToLow(base);
		break;

	case USBC_ID_TYPE_DEVICE:
		__USBC_ForceIdToHigh(base);
		break;

	default:
		__USBC_ForceIdDisable(base);
		break;
	}
}

#define SW_VA_SRAM_IO_BASE                0xf1c00000
static void USBC_ConfigFIFO_Base(void)
{
	static DEFINE_SPINLOCK(lock);
	unsigned long flags = 0;
	u32 reg_value;
	void __iomem *reg = ioremap(SW_VA_SRAM_IO_BASE, 0x8);

	/* config usb fifo, 8kb mode */
	spin_lock_irqsave(&lock, flags);

	reg_value = __raw_readl(reg);
	reg_value &= ~(0x03 << 0);
	reg_value |= (1 << 0);
	__raw_writel(reg_value, reg);

	spin_unlock_irqrestore(&lock, flags);
}

static int sunxi_musb_init(struct musb *musb)
{
	struct device *dev = musb->controller;
	int res;

	dev_dbg(musb->controller, "%s() mode = %d\n", __func__, musb->port_mode);

	musb->phy = devm_phy_get(dev->parent, "usb");
	if (IS_ERR(musb->phy)) {
		if (PTR_ERR(musb->phy) == -EPROBE_DEFER) {
			dev_info(dev, "phy probe deferred\n");
			return -EPROBE_DEFER;
		}
		dev_err(dev, "no phy configured\n");
		return PTR_ERR(musb->phy);
	}

	usb_phy_generic_register();
	musb->xceiv = usb_get_phy(USB_PHY_TYPE_USB2);
	if (IS_ERR(musb->xceiv)) {
		if (PTR_ERR(musb->xceiv) == -EPROBE_DEFER) {
			dev_info(dev, "usb phy probe deferred\n");
			return -EPROBE_DEFER;
		}
		dev_err(dev, "no usb phy configured\n");
		return PTR_ERR(musb->xceiv);
	}

	res = phy_init(musb->phy);
	if (res)
		return res;

	res = usb_phy_init(musb->xceiv);
	if (res)
		return res;

	musb->isr = sunxi_musb_interrupt;

	/* TODO: pin init -- DONE!*/

	USBC_EnableDpDmPullUp(musb->mregs);
	USBC_EnableIdPullUp(musb->mregs);

	return 0;
}

static int sunxi_musb_exit(struct musb *musb)
{
	dev_dbg(musb->controller, "%s()\n", __func__);

	return 0;
}

static void USBC_SelectBus(__iomem void *base, __u32 io_type, __u32 ep_type,
			   __u32 ep_index)
{
	u32 reg_val = 0;

	reg_val = musb_readb(base, USBC_REG_o_VEND0);

	if (io_type == USBC_IO_TYPE_DMA) {
		if (ep_type == USBC_EP_TYPE_TX) {
			/* drq_sel */
			reg_val |= ((ep_index - 1) << 1) <<
					USBC_BP_VEND0_DRQ_SEL;
			/* io_dma */
			reg_val |= 1 << USBC_BP_VEND0_BUS_SEL;
		} else {
			reg_val |= ((ep_index << 1) - 1) <<
					USBC_BP_VEND0_DRQ_SEL;
			reg_val |= 1 << USBC_BP_VEND0_BUS_SEL;
		}
	} else {
		/* Clear drq_sel, select pio */
		reg_val &= 0x00;
	}

	musb_writeb(base, USBC_REG_o_VEND0, reg_val);
}

static void sunxi_musb_set_vbus(struct musb *musb, int is_on)
{
	u8            devctl;
	unsigned long timeout = jiffies + msecs_to_jiffies(1000);
	/* HDRC controls CPEN, but beware current surges during device
	 * connect.  They can trigger transient overcurrent conditions
	 * that must be ignored.
	 */
	dev_dbg(musb->controller, "%s()\n", __func__);

	devctl = musb_readb(musb->mregs, MUSB_DEVCTL);

	if (is_on) {
		if (musb->xceiv->state == OTG_STATE_A_IDLE) {
			/* start the session */
			dev_dbg(musb->controller, "%s(): starting the session\n", __func__);
			devctl |= MUSB_DEVCTL_SESSION;
			musb_writeb(musb->mregs, MUSB_DEVCTL, devctl);
			/*
			 * Wait for the musb to set as A device to enable the
			 * VBUS
			 */
			while (musb_readb(musb->mregs, MUSB_DEVCTL) & 0x80) {

				if (time_after(jiffies, timeout)) {
					dev_err(musb->controller,
					"configured as A device timeout");
					break;
				}
			}

		} else {
			musb->is_active = 1;
			musb->xceiv->otg->default_a = 1;
			musb->xceiv->state = OTG_STATE_A_WAIT_VRISE;
			devctl |= MUSB_DEVCTL_SESSION;
			MUSB_HST_MODE(musb);
		}
	} else {
		musb->is_active = 0;

		/* NOTE: we're skipping A_WAIT_VFALL -> A_IDLE and jumping
		 * right to B_IDLE...
		 */
		musb->xceiv->otg->default_a = 0;
		devctl &= ~MUSB_DEVCTL_SESSION;
		MUSB_DEV_MODE(musb);
	}
	musb_writeb(musb->mregs, MUSB_DEVCTL, devctl);

	/*
	 * Devctl values will be updated after vbus goes below
	 * session_valid. The time taken depends on the capacitance
	 * on VBUS line. The max discharge time can be upto 1 sec
	 * as per the spec. Typically on our platform, it is 200ms
	 */
	if (!is_on)
		mdelay(200);

	dev_dbg(musb->controller, "VBUS %s, devctl %02x\n",
		usb_otg_state_string(musb->xceiv->state),
		musb_readb(musb->mregs, MUSB_DEVCTL));
}

static void sunxi_musb_enable(struct musb *musb)
{
	dev_dbg(musb->controller, "%s()\n", __func__);

	/* select PIO mode */
	USBC_SelectBus(musb->mregs, USBC_IO_TYPE_PIO, 0, 0);

	//sunxi_musb_set_vbus(musb, 1);
}

static void sunxi_musb_disable(struct musb *musb)
{
	dev_dbg(musb->controller, "%s()\n", __func__);
}


static int sunxi_musb_set_mode(struct musb *musb, u8 musb_mode)
{
	dev_dbg(musb->controller, "%s(): musb_mode %d\n", __func__, musb_mode);

	switch (musb_mode) {
	case MUSB_HOST:
		USBC_ForceId(musb->mregs, USBC_ID_TYPE_HOST);
		USBC_ForceVbusValid(musb->mregs, USBC_VBUS_TYPE_HIGH);
		dev_info(musb->controller, "Set USB VBUS power on\n");
		phy_power_on(musb->phy);
	//	sunxi_musb_set_vbus(musb, 1);
		break;

	case MUSB_PERIPHERAL:
		USBC_ForceId(musb->mregs, USBC_ID_TYPE_DEVICE);
		USBC_ForceVbusValid(musb->mregs, USBC_VBUS_TYPE_DISABLE);
		dev_info(musb->controller, "Set USB VBUS power off\n");
		phy_power_off(musb->phy);
		sunxi_musb_set_vbus(musb, 0);
		break;

	case MUSB_OTG:
	default:
		USBC_ForceId(musb->mregs, USBC_ID_TYPE_DISABLE);
		USBC_ForceVbusValid(musb->mregs, USBC_VBUS_TYPE_DISABLE);
		/* set vbus? */
		break;
	}

	return 0;
}

static const struct musb_platform_ops sunxi_ops = {
	.init		= sunxi_musb_init,
	.exit		= sunxi_musb_exit,

	.enable		= sunxi_musb_enable,
	.disable	= sunxi_musb_disable,

	.set_vbus	= sunxi_musb_set_vbus, 

	.set_mode	= sunxi_musb_set_mode,
};

/* Allwinner OTG supports up to 5 endpoints */
#define SUNXI_MUSB_MAX_EP_NUM	6
#define SUNXI_MUSB_RAM_BITS	11

static struct musb_fifo_cfg sunxi_musb_mode_cfg[] = {
	MUSB_EP_FIFO_SINGLE(1, FIFO_TX, 512),
	MUSB_EP_FIFO_SINGLE(1, FIFO_RX, 512),
	MUSB_EP_FIFO_SINGLE(2, FIFO_TX, 512),
	MUSB_EP_FIFO_SINGLE(2, FIFO_RX, 512),
	MUSB_EP_FIFO_SINGLE(3, FIFO_TX, 512),
	MUSB_EP_FIFO_SINGLE(3, FIFO_RX, 512),
	MUSB_EP_FIFO_SINGLE(4, FIFO_TX, 512),
	MUSB_EP_FIFO_SINGLE(4, FIFO_RX, 512),
	MUSB_EP_FIFO_SINGLE(5, FIFO_TX, 512),
	MUSB_EP_FIFO_SINGLE(5, FIFO_RX, 512),
};

static struct musb_hdrc_config sunxi_musb_hdrc_config = {
	.fifo_cfg       = sunxi_musb_mode_cfg,
	.fifo_cfg_size  = ARRAY_SIZE(sunxi_musb_mode_cfg),
	.multipoint	= true,
	.dyn_fifo	= true, /* deprecated, but was in Allwinner driver */
	.soft_con       = true, /* deprecated, but was in Allwinner driver */ 
	.num_eps	= SUNXI_MUSB_MAX_EP_NUM,
	.ram_bits	= SUNXI_MUSB_RAM_BITS,
	.dma		= 0,
};

static u64 sunxi_dmamask = DMA_BIT_MASK(32);

static int sunxi_probe(struct platform_device *pdev)
{
	struct musb_hdrc_platform_data	*pdata;
	struct platform_device		*musb;
	struct device_node		*np = pdev->dev.of_node;
	struct sunxi_glue		*glue;
	struct regmap			*sc;
	struct clk			*clk;
	int				gpio, ret = -ENOMEM;
	u32				flags;
	int mode;

	dev_dbg(&pdev->dev, "%s()\n", __func__);

	if (!np) {
		dev_err(&pdev->dev, "no device tree node found\n");
		return -EINVAL;
	}

	pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

	mode = of_usb_get_dr_mode(np);

	switch (mode) {
		case USB_DR_MODE_HOST:
			pdata->mode = MUSB_HOST;
			break;

		case USB_DR_MODE_PERIPHERAL:
			pdata->mode = MUSB_PERIPHERAL;
			break;

		case USB_DR_MODE_OTG:
			pdata->mode = MUSB_OTG;
			break;

		case USB_DR_MODE_UNKNOWN:
		default:
			dev_err(&pdev->dev, "No 'dr_mode' property found\n");
			goto err0;
	}

	glue = devm_kzalloc(&pdev->dev, sizeof(*glue), GFP_KERNEL);
	if (!glue) {
		dev_err(&pdev->dev, "failed to allocate glue context\n");
		return -ENOMEM;
	}

	gpio = of_get_named_gpio_flags(np, "id_det-gpios", 0, &flags);
	if (gpio < 0) {
		dev_err(&pdev->dev, "no id_det-gpios gpio found\n");
		return -EINVAL;
	}
	glue->id_det_gpio = gpio;

	gpio = of_get_named_gpio_flags(np, "vbus_det-gpios", 0, &flags);
	if (gpio < 0) {
		dev_err(&pdev->dev, "no vbus_det-gpios gpio found\n");
		return -EINVAL;
	}
	glue->vbus_gpio = gpio;

	clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(clk)) {
		dev_err(&pdev->dev, "failed to get clock\n");
		ret = PTR_ERR(clk);
		goto err2;
	}

	ret = clk_prepare_enable(clk);
	if (ret) {
		dev_err(&pdev->dev, "failed to enable clock\n");
		goto err3;
	}

	sc = syscon_regmap_lookup_by_phandle(np, "syscons");
	if (IS_ERR(sc)) {
		ret = PTR_ERR(sc);
		dev_err(&pdev->dev, "failed to get syscon\n");
		goto err0;
	}

	/* TODO: give nice name to bit #0 */
	ret = regmap_update_bits(sc, 0, BIT(0), BIT(0));
	if (ret) {
		dev_err(&pdev->dev, "failed to set SRAM mapping\n");
		goto err0;
	}

	musb = platform_device_alloc("musb-hdrc", PLATFORM_DEVID_AUTO);
	if (!musb) {
		dev_err(&pdev->dev, "failed to allocate musb device\n");
		goto err3;
	}

	glue->dev			= &pdev->dev;
	glue->musb			= musb;
	glue->clk			= clk;
	/* glue->sc			= sc; */

	pdata->platform_ops		= &sunxi_ops;
	pdata->config 			= &sunxi_musb_hdrc_config;

	musb->dev.parent		= &pdev->dev;
	musb->dev.dma_mask		= &sunxi_dmamask;//&pdev->dev.coherent_dma_mask;
	musb->dev.coherent_dma_mask	= sunxi_dmamask; //pdev->dev.coherent_dma_mask;

	platform_set_drvdata(pdev, glue);

	ret = platform_device_add_resources(musb, pdev->resource,
					    pdev->num_resources);
	if (ret) {
		dev_err(&pdev->dev, "failed to add resources\n");
		goto err5;
	}

	ret = platform_device_add_data(musb, pdata, sizeof(*pdata));
	if (ret) {
		dev_err(&pdev->dev, "failed to add platform_data\n");
		goto err5;
	}

	ret = platform_device_add(musb);
	if (ret) {
		dev_err(&pdev->dev, "failed to register musb device\n");
		goto err5;
	}

	return 0;

err5:
	platform_device_put(musb);

err3:
	clk_disable_unprepare(clk);

err2:
	/* regmap_update_bits(sc, 0, BIT(0), 0); */

err0:
	return ret;
}

static int sunxi_remove(struct platform_device *pdev)
{
	return 0;
}

#ifdef CONFIG_PM
static int sunxi_suspend(struct device *dev)
{
	return 0;
}

static int sunxi_resume(struct device *dev)
{
	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(sunxi_pm_ops, sunxi_suspend, sunxi_resume);

static const struct of_device_id sunxi_match[] = {
	{ .compatible = "allwinner,sun4i-a10-musb", },
	{}
};

static struct platform_driver sunxi_driver = {
	.probe = sunxi_probe,
	.remove = sunxi_remove,
	.driver = {
		.name= "musb-sunxi",
		.pm= &sunxi_pm_ops,
		.of_match_table = sunxi_match,
	},
};

MODULE_DESCRIPTION("Allwinner sunxi MUSB Glue Layer");
MODULE_AUTHOR("Roman Byshko <rbyshko@gmail.com>, Chen-Yu Tsai <wens@csie.org>");
MODULE_LICENSE("GPL v2");
module_platform_driver(sunxi_driver);
