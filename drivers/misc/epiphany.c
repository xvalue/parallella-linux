#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/clk.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/mm.h>
#include <linux/list.h>
#include <linux/uaccess.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/cdev.h>

#include "epiphany.h"

#define DRIVERNAME	"epiphany"

#define E_DEV_NUM_MINORS	MINORMASK	/* Total to reserve */

static dev_t epiphany_devt;

struct class *epiphany_class;

struct epiphany_device;
/* Singleton */
static struct epiphany_device *epiphany_device;

static DEFINE_IDR(epiphany_minor_idr);
/* Used by minor_get() / minor_put() */
static DEFINE_SPINLOCK(epiphany_minor_idr_lock);

/* One big lock for everything */
static DEFINE_MUTEX(driver_lock);

#define COREID_SHIFT 20

/* Be careful, no range check */
#define COORDS(row, col) ((row) * 64 | (col))
#define ROW(coreid) ((coreid) / 64)
#define COL(coreid) ((coreid) % 64)

enum elink_side {
	E_SIDE_N = 0,
	E_SIDE_E,
	E_SIDE_S,
	E_SIDE_W,
	E_SIDE_MAX
};

enum connection_type {
	E_CONN_DISCONNECTED = 0,
	E_CONN_ELINK,
	E_CONN_ARRAY,
	E_CONN_MAX
};

enum e_chip_type {
	E_CHIP_INVAL = 0,
	E_CHIP_E16G301,
	E_CHIP_E64G401,
	E_CHIP_MAX
};

static const u32 ctrlmode_hints[E_SIDE_MAX] = {
	[E_SIDE_N] = E_CTRLMODE_NORTH,
	[E_SIDE_E] = E_CTRLMODE_EAST,
	[E_SIDE_S] = E_CTRLMODE_SOUTH,
	[E_SIDE_W] = E_CTRLMODE_WEST
};

struct epiphany_chip_info {
	int rows;
	int cols;
	size_t core_mem;
	u16 elink_coreid[E_SIDE_MAX]; /* relative */

	/* In uVolts */
	int vdd_default;
	int vdd_min;
	int vdd_max;

	u32 linkcfg_tx_divider;
};
static const struct epiphany_chip_info epiphany_chip_info[E_CHIP_MAX] = {
	[E_CHIP_E16G301] = {
		.rows = 4,
		.cols = 4,
		.core_mem = 32768,

		.elink_coreid[E_SIDE_N] = COORDS(0, 2),
		.elink_coreid[E_SIDE_E] = COORDS(2, 3),
		.elink_coreid[E_SIDE_S] = COORDS(3, 2),
		.elink_coreid[E_SIDE_W] = COORDS(2, 0),

		/* Recommended operating conditions */
		.vdd_default	= 1000000,
		.vdd_min	=  900000,
		.vdd_max	= 1200000,

		.linkcfg_tx_divider = 1
	},
	[E_CHIP_E64G401] = {
		.rows = 8,
		.cols = 8,
		.core_mem = 32768,

		.elink_coreid[E_SIDE_N] = COORDS(0, 2),
		.elink_coreid[E_SIDE_E] = COORDS(2, 7),
		.elink_coreid[E_SIDE_S] = COORDS(7, 2),
		.elink_coreid[E_SIDE_W] = COORDS(2, 0),

		/* Recommended operating conditions */
		.vdd_default	= 1000000,
		.vdd_min	=  900000,
		.vdd_max	= 1100000,

		/* TODO: Verify */
		.linkcfg_tx_divider = 0
	}
};

enum elink_generation {
	E_GEN_INVAL = 0,
	E_GEN_PARALLELLA1,
	E_GEN_MAX
};

enum elink_platform {
	E_PLATF_INVAL = 0,
	E_PLATF_E16_7Z020_GPIO,
	E_PLATF_E16_7Z020_NO_GPIO,
	E_PLATF_E16_7Z010_GPIO,
	E_PLATF_E16_7Z010_NO_GPIO,
	E_PLATF_E64_7Z020_GPIO,
	E_PLATF_MAX
};

static const enum e_chip_type elink_platform_chip_match[E_PLATF_MAX] = {
	[E_PLATF_INVAL]			= E_CHIP_INVAL,
	[E_PLATF_E16_7Z020_GPIO]	= E_CHIP_E16G301,
	[E_PLATF_E16_7Z020_NO_GPIO]	= E_CHIP_E16G301,
	[E_PLATF_E16_7Z010_GPIO]	= E_CHIP_E16G301,
	[E_PLATF_E16_7Z010_NO_GPIO]	= E_CHIP_E16G301,
	[E_PLATF_E64_7Z020_GPIO]	= E_CHIP_E64G401
};

struct connection {
	enum connection_type type; /* remote type */
	enum elink_side side; /* remote side */
	union {
		struct elink_device *elink;
		struct array_device *array;
	};

	phandle phandle;
};

struct elink_device {
	struct list_head list;
	struct device *dev;

	void __iomem *regs;
	phys_addr_t regs_start;
	size_t regs_size;

	/* TODO: Rename */
	/* Host --> emesh bus address range */
	phys_addr_t emesh_start;
	size_t emesh_size;

	struct clk **clocks;

	s16 coreid_pinout; /* core id pinout */
	bool quirk_coreid_is_noop;

	union e_syscfg_version version;
	enum e_chip_type chip_type;

	struct connection connection;

	/* TODO: Have our own cdev */
	struct device *char_dev;
	struct cdev cdev;
	int minor;

	phandle phandle;
};

struct array_device {
	struct list_head list;

	struct device dev;

	u16 id; /* north-west-most core */
	unsigned int chip_rows;
	unsigned int chip_cols;
	enum e_chip_type chip_type;

	struct connection connections[E_SIDE_MAX];

	struct regulator *supply;
	int vdd_wanted;

	phandle phandle;
};

struct mem_region {
	struct list_head list;
	phys_addr_t start;
	size_t size;

	phandle phandle;
};

struct epiphany_device {
	bool reserved; /* Singleton */

	int u_count; /* User count */

	struct list_head elink_list;
	struct list_head mem_region_list;
	struct list_head chip_array_list;
};


/* Any point suggesting these three for drivers/of/base.c ? */
#define count_phandle_with_fixed_args(np, list_name, cell_count) \
	of_parse_phandle_with_fixed_args(np, list_name, cell_count, -1, NULL)

static inline struct device_node
*get_next_compatible_child(struct device_node *parent,
			   struct device_node *child,
			   char *compatible)
{
	for (child = of_get_next_child(parent, child); child != NULL;
	     child = of_get_next_child(parent, child)) {
		if (of_device_is_compatible(child, compatible))
			return child;
	}
	return NULL;
}

#define for_each_compatible_child(parent, dn, compat) \
	for (dn = get_next_compatible_child(parent, NULL, compat); \
	     dn != NULL; \
	     dn = get_next_compatible_child(parent, dn, compat))


/* TODO: Sledge-hammer approach. Needed on some Kickstarter boards. Ultimately
 * these long sleeps should only be needed when modifying clocks. */
static inline void epiphany_sleep(void)
{
	usleep_range(2000, 2100);
}

static inline void reg_write(u32 value, void __iomem *base, u32 offset)
{
	iowrite32(value, (u8 __iomem *)base + offset);
	/* Sledge hammer fix. See comment for epiphany_sleep() */
	epiphany_sleep();
}

static inline u32 reg_read(void __iomem *base, u32 offset)
{
	return ioread32((u8 __iomem *)base + offset);
}

static inline struct elink_device *file_to_elink(struct file *file)
{
	return container_of(file->private_data, struct elink_device, cdev);
}

static inline struct array_device *device_to_array(struct device *dev)
{
	return container_of(dev, struct array_device, dev);
}


static int coreid_to_phys(struct elink_device *elink, u16 coreid,
			  phys_addr_t *out)
{
	u32 rel_coreid, rel_row, rel_col;
	struct array_device *array = elink->connection.array;
	const struct epiphany_chip_info *cinfo;
	phys_addr_t offs;

	if (elink->connection.type != E_CONN_ARRAY)
		return -EINVAL;

	if (coreid < array->id)
		return -ERANGE;

	cinfo = &epiphany_chip_info[array->chip_type];
	rel_coreid = coreid - array->id;
	rel_row = ROW(rel_coreid);
	rel_col = COL(rel_coreid);

	if (rel_row >= array->chip_rows * cinfo->rows)
		return -ERANGE;

	if (rel_col >= array->chip_cols * cinfo->cols)
		return -ERANGE;

	/* Offset from array start */
	offs = ((phys_addr_t) rel_coreid) << ((phys_addr_t) COREID_SHIFT);

	/* Adjust for offset from elink mem region (align by row) */
	offs += ((phys_addr_t) COL(array->id)) << ((phys_addr_t) COREID_SHIFT);

	if (offs >= elink->emesh_size)
		return -ERANGE;

	*out = offs + elink->emesh_start;

	return 0;
}

/* Disable chip elink */
static void elink_disable_chip_elink(struct elink_device *elink,
				     struct array_device *array,
				     u16 chipid,
				     enum elink_side side)
{
	int err;
	const struct epiphany_chip_info *cinfo =
		&epiphany_chip_info[array->chip_type];
	phys_addr_t core_phys, regs_phys;
	u16 coreid;
	void __iomem *regs;
	union e_syscfg_tx txcfg;

	coreid = chipid + cinfo->elink_coreid[side];

	dev_dbg(elink->dev,
		"Disabling elink 0x%03x (%02u, %02u) in array 0x%03x.\n",
		coreid, ROW(coreid), COL(coreid), array->id);

	err = coreid_to_phys(elink, coreid, &core_phys);
	WARN_ON(err);
	if (err)
		return;

	regs_phys = (core_phys | E_REG_BASE) & PAGE_MASK;
	regs = ioremap_nocache(regs_phys, PAGE_SIZE);
	WARN_ON(!regs);
	if (!regs)
		return;

	txcfg.reg = reg_read(elink->regs, E_SYS_CFGTX);
	txcfg.ctrlmode = ctrlmode_hints[side];
	reg_write(txcfg.reg, elink->regs, E_SYS_CFGTX);

	reg_write(0xfff, regs, E_REG_LINKTXCFG & ~PAGE_MASK);
	reg_write(0xfff, regs, E_REG_LINKRXCFG & ~PAGE_MASK);

	txcfg.ctrlmode = 0;
	reg_write(txcfg.reg, elink->regs, E_SYS_CFGTX);

	iounmap(regs);
}

static void array_disable_disconnected_elinks(struct elink_device *elink,
					      struct array_device *array)
{
	int i;
	const struct epiphany_chip_info *cinfo =
		&epiphany_chip_info[array->chip_type];
	enum elink_side side;
	u32 mask = 0;
	u16 north_chip, south_chip, east_chip, west_chip;

	for (side = 0; side < ARRAY_SIZE(array->connections); side++) {
		if (array->connections[side].type == E_CONN_DISCONNECTED)
			mask |= 1 << side;

	}

	/* Walk north and south cols */
	if (mask & ((1 << E_SIDE_N) | (1 << E_SIDE_S))) {
		for (i = 0, north_chip = array->id;
		     i < array->chip_cols;
		     i++, north_chip += cinfo->cols) {
			south_chip = north_chip +
				COORDS((array->chip_rows - 1) * cinfo->rows, 0);

			if (mask & (1 << E_SIDE_N)) {
				elink_disable_chip_elink(elink, array,
							 north_chip, E_SIDE_N);
			}
			if (mask & (1 << E_SIDE_S)) {
				elink_disable_chip_elink(elink, array,
							 south_chip, E_SIDE_S);
			}
		}
	}

	/* Walk east and west rows */
	if (mask & ((1 << E_SIDE_E) | (1 << E_SIDE_W))) {
		for (i = 0, west_chip = array->id;
		     i < array->chip_rows;
		     i++, west_chip += COORDS(1, 0)) {
			east_chip = west_chip +
				COORDS(0, (array->chip_cols - 1) * cinfo->cols);

			if (mask & (1 << E_SIDE_W)) {
				elink_disable_chip_elink(elink, array,
							 west_chip, E_SIDE_W);
			}
			if (mask & (1 << E_SIDE_E)) {
				elink_disable_chip_elink(elink, array,
							 east_chip, E_SIDE_E);
			}
		}
	}
}

static void array_enable_clock_gating(struct elink_device *elink,
				      struct array_device *array)
{
	int err, i, j, row0, col0, last_row, last_col;
	const struct epiphany_chip_info *cinfo =
		&epiphany_chip_info[array->chip_type];
	phys_addr_t core, paddr;
	void __iomem *core_mem;
	u32 config, meshconfig;

	row0 = ROW(array->id);
	col0 = COL(array->id);
	last_row = row0 + array->chip_rows * cinfo->rows;
	last_col = col0 + array->chip_cols * cinfo->cols;

	for (i = row0; i < last_row; i++) {
		for (j = col0; j < last_col; j++) {
			err = coreid_to_phys(elink, COORDS(i, j), &core);
			WARN_ON(err);
			if (err)
				continue;

			paddr = (core | E_REG_BASE) & PAGE_MASK;
			core_mem = ioremap_nocache(paddr, PAGE_SIZE);
			WARN_ON(!core_mem);
			if (!core_mem)
				continue;


			config = E_REG_CONFIG & ~(PAGE_MASK);
			reg_write(0x00400000, core_mem, config);

			meshconfig = E_REG_MESHCONFIG & ~(PAGE_MASK);
			reg_write(0x00000002, core_mem, meshconfig);

			iounmap(core_mem);
		}
	}
}

static int configure_chip_tx_divider(struct elink_device *elink,
				     u16 chipid,
				     enum elink_side side)
{
	int err;
	struct array_device *array = elink->connection.array;
	const struct epiphany_chip_info *cinfo =
		&epiphany_chip_info[array->chip_type];
	phys_addr_t core_phys, regs_phys;
	u16 coreid;
	void __iomem *regs;
	union e_syscfg_tx txcfg;
	u32 offset;


	/* Figure out which divider we need for the TX reg */
	if (!cinfo->linkcfg_tx_divider)
		return 0;

	coreid = chipid + cinfo->elink_coreid[side];

	dev_dbg(elink->dev,
		"chip requires programming the link clock divider.\n");

	txcfg.reg = reg_read(elink->regs, E_SYS_CFGTX);
	txcfg.ctrlmode = ctrlmode_hints[side];

	err = coreid_to_phys(elink, coreid, &core_phys);
	WARN_ON(err);
	if (err)
		return err;

	regs_phys = (core_phys | E_REG_BASE) & PAGE_MASK;
	regs = ioremap_nocache(regs_phys, PAGE_SIZE);
	offset = E_REG_LINKCFG & ~(PAGE_MASK);
	WARN_ON(!regs);
	if (!regs)
		return -EIO;

	txcfg.reg = reg_read(elink->regs, E_SYS_CFGTX);
	txcfg.ctrlmode = ctrlmode_hints[side];
	reg_write(txcfg.reg, elink->regs, E_SYS_CFGTX);

	reg_write(cinfo->linkcfg_tx_divider, regs, offset);

	txcfg.ctrlmode = 0;
	reg_write(txcfg.reg, elink->regs, E_SYS_CFGTX);

	iounmap(regs);
	return 0;
}

static int configure_adjacent_links(struct elink_device *elink)
{
	int i;
	const struct epiphany_chip_info *cinfo;
	struct array_device *array;
	u16 north_chip, south_chip, east_chip, west_chip;

	if (elink->connection.type != E_CONN_ARRAY)
		return 0;

	array = elink->connection.array;
	cinfo = &epiphany_chip_info[array->chip_type];

	if (elink->connection.side == E_SIDE_N ||
	    elink->connection.side == E_SIDE_S) {
		for (i = 0, north_chip = array->id;
		     i < array->chip_cols;
		     i++, north_chip += cinfo->cols) {
			south_chip = north_chip +
				COORDS((array->chip_rows - 1) * cinfo->rows, 0);

			if (elink->connection.side == E_SIDE_N) {
				return configure_chip_tx_divider(elink,
								 north_chip,
								 E_SIDE_N);
			} else {
				return configure_chip_tx_divider(elink,
								 south_chip,
								 E_SIDE_S);
			}
		}
	} else {
		for (i = 0, west_chip = array->id;
		     i < array->chip_rows;
		     i++, west_chip += COORDS(1, 0)) {
			east_chip = west_chip +
				COORDS(0, (array->chip_cols - 1) * cinfo->cols);

			if (elink->connection.side == E_SIDE_W) {
				return configure_chip_tx_divider(elink,
								 west_chip,
								 E_SIDE_W);
			} else {
				return configure_chip_tx_divider(elink,
								 east_chip,
								 E_SIDE_E);
			}
		}
	}

	return 0;
}

/* Reset the Epiphany platform */
static int reset_elink(struct elink_device *elink)
{
	int retval = 0;
	union e_syscfg_tx txcfg = {0};
	union e_syscfg_rx rxcfg = {0};
	union e_syscfg_clk clkcfg = {0};

	epiphany_sleep();

	/* Assert reset */
	reg_write(1, elink->regs, E_SYS_RESET);

	/* Disable TX */
	txcfg.reg = 0;
	reg_write(txcfg.reg, elink->regs, E_SYS_CFGTX);

	/* Disable RX */
	rxcfg.reg = 0;
	reg_write(rxcfg.reg, elink->regs, E_SYS_CFGRX);

	/* Start C-clock */
	clkcfg.divider = 7; /* Full speed */
	reg_write(clkcfg.reg, elink->regs, E_SYS_CFGCLK);

	/* Stop C-clock for setup/hold time on reset */
	clkcfg.divider = 0;
	reg_write(clkcfg.reg, elink->regs, E_SYS_CFGCLK);

	/* Configure core id. Here should be the right place? clocks disabled
	 * and reset asserted. */
	reg_write(elink->coreid_pinout, elink->regs, E_SYS_COREID);

	/* Deassert reset */
	reg_write(0, elink->regs, E_SYS_RESET);

	/* Restart C-clock */
	clkcfg.divider = 7; /* Full speed */
	reg_write(clkcfg.reg, elink->regs, E_SYS_CFGCLK);

	/* Start TX L-clock */
	txcfg.clkmode = 0; /* Full speed */
	reg_write(txcfg.reg, elink->regs, E_SYS_CFGTX);

	/* enable eLink TX */
	txcfg.enable  = 1;
	reg_write(txcfg.reg, elink->regs, E_SYS_CFGTX);

	/* Enable eLink RX */
	rxcfg.enable = 1;
	reg_write(rxcfg.reg, elink->regs, E_SYS_CFGRX);

	retval = configure_adjacent_links(elink);

	epiphany_sleep();
	return retval;
}

static void disable_elink(struct elink_device *elink)
{
	union e_syscfg_clk clkcfg;
	union e_syscfg_tx txcfg;
	union e_syscfg_rx rxcfg;

	clkcfg.reg = reg_read(elink->regs, E_SYS_CFGCLK);
	txcfg.reg = reg_read(elink->regs, E_SYS_CFGTX);
	rxcfg.reg = reg_read(elink->regs, E_SYS_CFGRX);

	clkcfg.divider = 0;
	txcfg.enable = 0;
	rxcfg.enable = 0;

	reg_write(clkcfg.reg, elink->regs, E_SYS_CFGCLK);
	reg_write(txcfg.reg, elink->regs, E_SYS_CFGTX);
	reg_write(rxcfg.reg, elink->regs, E_SYS_CFGRX);

	epiphany_sleep();
}

static int epiphany_regulator_enable(struct array_device *array)
{
	int ret;
	const struct epiphany_chip_info *cinfo =
		&epiphany_chip_info[array->chip_type];

	if (!array->supply)
		return 0;

	ret = regulator_set_voltage(array->supply, array->vdd_wanted,
				    cinfo->vdd_max);
	if (ret)
		return ret;

	ret = regulator_enable(array->supply);
	if (ret)
		return ret;

	return 0;
}

static int epiphany_reset(void)
{
	struct array_device *array;
	struct elink_device *elink;
	int err, retval = 0;

	/* Unsafe to manipulate power if already in use. At any rate we should
	 * not call regulator_enable() again since that would screw up the
	 * regulator's refcount */
	if (!epiphany_device->u_count) {
		list_for_each_entry(array, &epiphany_device->chip_array_list,
				    list) {
			if (epiphany_regulator_enable(array)) {
				/* Not much else we can do? */
				retval = -EIO;
				goto out;
			}
		}
	}

	list_for_each_entry(elink, &epiphany_device->elink_list, list) {
		err = reset_elink(elink);
		if (err) {
			retval = -EIO;
			goto out;
		}
	}

	list_for_each_entry(elink, &epiphany_device->elink_list, list) {
		if (elink->connection.type != E_CONN_ARRAY)
			continue;
		array_enable_clock_gating(elink, elink->connection.array);
		array_disable_disconnected_elinks(elink,
						  elink->connection.array);
	}

out:
	return retval;
}

static int elink_char_open(struct inode *inode, struct file *file)
{
	int ret = 0;

	file->private_data = inode->i_cdev;

	if (mutex_lock_interruptible(&driver_lock))
		return -ERESTARTSYS;

	if (!epiphany_device->u_count) {
		/* if !epiphany_device->param_no_reset (or no power mgmt) */
		ret = epiphany_reset();
		if (ret)
			goto mtx_unlock;
	}

	epiphany_device->u_count++;

mtx_unlock:
	mutex_unlock(&driver_lock);

	return ret;
}

static void epiphany_disable(void)
{
	struct elink_device *elink;
	struct array_device *array;

	list_for_each_entry(elink, &epiphany_device->elink_list, list)
		disable_elink(elink);

	/* ??? TODO: Move arrays to elinks ? */
	list_for_each_entry(array, &epiphany_device->chip_array_list, list) {
		if (array->supply)
			regulator_disable(array->supply);
	}

}

static int elink_char_release(struct inode *inode, struct file *file)
{
	/* Not sure if interruptible is a good idea here ... */
	mutex_lock(&driver_lock);

	epiphany_device->u_count--;

	if (!epiphany_device->u_count) {
		/* if (!epiphany_device->param_no_powersave) */
		epiphany_disable();
		pr_debug("epiphany_device: no users\n");
	}

	mutex_unlock(&driver_lock);
	return 0;
}

static int epiphany_map_memory(struct vm_area_struct *vma, bool device_mem)
{
	int err;

	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

	if (device_mem) {
		err = io_remap_pfn_range(vma, vma->vm_start, vma->vm_pgoff,
					 vma->vm_end - vma->vm_start,
					 vma->vm_page_prot);
	} else {
		err = remap_pfn_range(vma, vma->vm_start, vma->vm_pgoff,
				      vma->vm_end - vma->vm_start,
				      vma->vm_page_prot);
	}

	if (err) {
		pr_err("Failed mapping memory to vma 0x%08lx, size 0x%08lx, page offset 0x%08lx\n",
		       vma->vm_start, vma->vm_end - vma->vm_start,
		       vma->vm_pgoff);
	}

	return err;
}

static const struct vm_operations_struct mmap_mem_ops = {
#ifdef CONFIG_HAVE_IOREMAP_PROT
	.access = generic_access_phys
#endif
};

static int elink_char_mmap(struct file *file, struct vm_area_struct *vma)
{
	unsigned long off = vma->vm_pgoff << PAGE_SHIFT;
	unsigned long size = vma->vm_end - vma->vm_start;
	struct elink_device *elink = file_to_elink(file);
	struct mem_region *region;

	vma->vm_ops = &mmap_mem_ops;

	/* TODO: adjust address */

	if (elink->emesh_start <= off &&
	    off + size <= elink->emesh_start + elink->emesh_size)
		return epiphany_map_memory(vma, true);

	/* TODO: Should only be allowed if param_unsafe is set */
	if (elink->regs_start <= off &&
	    off + size <= elink->regs_start + elink->regs_size)
		return epiphany_map_memory(vma, true);

	/* TODO: Access through separate device? /dev/emem */
	list_for_each_entry(region, &epiphany_device->mem_region_list, list) {
		if (region->start <= off &&
		    off + size <= region->start + region->size)
			return epiphany_map_memory(vma, false);
	}

	dev_dbg(elink->dev,
		"elink_char_mmap: invalid request to map 0x%08lx, length 0x%08lx bytes\n",
		off, size);

	return -EINVAL;
}

static long elink_char_ioctl(struct file *file, unsigned int cmd,
				unsigned long arg)
{
	int err = 0;
	/* ??? TODO: Reset elink only instead of entire system ? */
	/* struct elink_device *elink = file_to_elink(file)->epiphany; */

	if (_IOC_TYPE(cmd) != EPIPHANY_IOC_MAGIC)
		return -ENOTTY;

	if (_IOC_NR(cmd) > EPIPHANY_IOC_MAXNR)
		return -ENOTTY;

	/* Do we really need to do this check?
	 * Isn't copy_to_user() already doing that? */
	if (_IOC_DIR(cmd) & _IOC_READ) {
		err =
		    !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
	} else if (_IOC_DIR(cmd) & _IOC_WRITE) {
		err =
		    !access_ok(VERIFY_WRITE, (void __user *)arg,
			       _IOC_SIZE(cmd));
	}

	if (err)
		return -EFAULT;

	switch (cmd) {
	case EPIPHANY_IOC_RESET:
		/* This is unsafe since another thread might be accessing the
		 * emesh concurrently. Either we need to suspend all tasks that
		 * have the device open or perhaps we can do it with a fault
		 * handler ? */
		if (mutex_lock_interruptible(&driver_lock))
			return -ERESTARTSYS;
		/* Reset all devices, might be a better idea to register a
		 * "ectrl" control device for the class to make things more
		 * explicit
		 */
		err = epiphany_reset();
		mutex_unlock(&driver_lock);

		if (err)
			return err;

		break;
	default:
		return -ENOTTY;
	}

	return 0;
}

static int minor_get(void *ptr)
{
	int retval;

	idr_preload(GFP_KERNEL);
	spin_lock(&epiphany_minor_idr_lock);
	retval = idr_alloc(&epiphany_minor_idr, ptr, 0, E_DEV_NUM_MINORS,
			   GFP_NOWAIT);
	spin_unlock(&epiphany_minor_idr_lock);
	idr_preload_end();
	return retval;
}

static void minor_put(int minor)
{
	spin_lock(&epiphany_minor_idr_lock);
	idr_remove(&epiphany_minor_idr, minor);
	spin_unlock(&epiphany_minor_idr_lock);
}

static const struct file_operations elink_char_driver_ops = {
	.owner		= THIS_MODULE,
	.open		= elink_char_open,
	.release	= elink_char_release,
	.mmap		= elink_char_mmap,
	.unlocked_ioctl	= elink_char_ioctl
};

ssize_t array_attr_vdd_current_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	int ret = 0, vdd_curr;
	struct array_device *array = device_to_array(dev);

	if (mutex_lock_interruptible(&driver_lock))
		return -ERESTARTSYS;

	if (!array->supply) {
		ret = -ENODEV;
		goto out;
	}

	vdd_curr = regulator_get_voltage(array->supply);
	if (vdd_curr < 0) {
		ret = vdd_curr;
		goto out;
	}

	ret = sprintf(buf, "%d\n", vdd_curr);

out:
	mutex_unlock(&driver_lock);
	return ret;
}

ssize_t array_attr_vdd_wanted_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	int ret = 0;
	struct array_device *array = device_to_array(dev);

	if (mutex_lock_interruptible(&driver_lock))
		return -ERESTARTSYS;

	if (!array->supply) {
		ret = -ENODEV;
		goto out;
	}

	ret = sprintf(buf, "%d\n", array->vdd_wanted);

out:
	mutex_unlock(&driver_lock);
	return ret;
}

/* Must hold driver lock before calling this function */
static int array_set_vdd_wanted(struct array_device *array, int vdd)
{
	unsigned int step;
	const struct epiphany_chip_info *cinfo =
		&epiphany_chip_info[array->chip_type];

	if (!array->supply)
		return -ENODEV;

	/* Zero or below resets to default vdd */
	if (vdd <= 0) {
		array->vdd_wanted = cinfo->vdd_default;
		return 0;
	}

	/* Round vdd down to closest step */
	step = regulator_get_linear_step(array->supply);
	vdd = vdd - vdd % step;

	if (vdd < cinfo->vdd_min || cinfo->vdd_max < vdd)
		return -ERANGE;

	array->vdd_wanted = vdd;

	return 0;
}

static ssize_t array_attr_vdd_wanted_store(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf,
					   size_t len)
{
	struct array_device *array = device_to_array(dev);
	int ret, data;

	ret = kstrtoint(buf, 10, &data);
	if (ret < 0)
		return ret;

	if (mutex_lock_interruptible(&driver_lock))
		return -ERESTARTSYS;

	ret = array_set_vdd_wanted(array, data);
	if (!ret)
		ret = len;

	mutex_unlock(&driver_lock);
	return ret;
}

#define DEVICE_ATTR_PFX(_pfx, _name, _mode, _show, _store) \
	struct device_attribute dev_attr_##_pfx##_##_name = \
		__ATTR(_name, _mode, _show, _store)

static DEVICE_ATTR_PFX(array, vdd_current, 0444, array_attr_vdd_current_show,
		       NULL);
static DEVICE_ATTR_PFX(array, vdd_wanted, 0644, array_attr_vdd_wanted_show,
		       array_attr_vdd_wanted_store);

static struct attribute *dev_attrs_array[] = {
	&dev_attr_array_vdd_current.attr,
	&dev_attr_array_vdd_wanted.attr,
	NULL
};

static struct attribute_group dev_attr_group_array = {
	.attrs = dev_attrs_array
};

static const struct attribute_group *dev_attr_groups_array[] = {
	&dev_attr_group_array,
	NULL
};


static void array_device_release(struct device *dev)
{
	/* No-op since we use devm_* */
}

static int array_register(struct array_device *array, struct device *parent)
{
	int ret;

	array->dev.class = epiphany_class;
	array->dev.parent = parent;
	array->dev.groups = dev_attr_groups_array;
	array->dev.release = array_device_release;
	/* There can only be one array per elink, no name conflicts */
	dev_set_name(&array->dev, "array");
	dev_set_drvdata(&array->dev, array);

	ret = device_register(&array->dev);
	if (ret) {
		dev_err(parent, "unable to create device array device\n");
		goto err_dev_create;
	}

	mutex_lock(&driver_lock);
	list_add_tail(&array->list, &epiphany_device->chip_array_list);
	mutex_unlock(&driver_lock);

	dev_dbg(&array->dev, "array_register: registered device\n");
	return 0;

err_dev_create:
	return ret;
}

static void array_unregister(struct array_device *array)
{
	mutex_lock(&driver_lock);
	list_del(&array->list);
	mutex_unlock(&driver_lock);

	device_unregister(&array->dev);
}


static struct array_device
*elink_of_probe_chip_array(struct elink_device *elink,
			   struct device_node *np,
			   enum elink_side *out_side)
{
	struct device_node *supply_node;
	struct array_device *array;
	struct regulator *supply;
	const struct epiphany_chip_info *cinfo =
		&epiphany_chip_info[elink->chip_type];
	enum elink_side side;
	u32 reg[4];
	int err;
	const char *supply_name;

	array = devm_kzalloc(elink->dev, sizeof(*array), GFP_KERNEL);
	if (!array)
		return ERR_PTR(-ENOMEM);

	array->phandle = np->phandle;

	/* There is probably a better way for doing this */
	err = of_property_read_u32_array(np, "reg", reg, 4);
	if (err) {
		dev_err(elink->dev, "arrays: invalid reg property\n");
		return ERR_PTR(err);
	}

	array->id = (u16) reg[0];
	side = reg[1];
	array->chip_rows = reg[2];
	array->chip_cols = reg[3];

	if (elink->quirk_coreid_is_noop && array->id != 0x808) {
		dev_warn(elink->dev,
			 "arrays: non default id and elink coreid is no-op\n");
	}

	if (elink->coreid_pinout == -1) {
		dev_dbg(elink->dev,
			"arrays: setting elink coreid to array id 0x%03x\n",
			array->id);
		elink->coreid_pinout = array->id;
	}

	array->chip_type = elink->chip_type;

	/* TODO: Support more than one regulator per array */
	supply_node = of_parse_phandle(np, "vdd-supply", 0);
	if (!supply_node) {
		dev_warn(elink->dev,
			 "arrays: no supply node specified, no power management.\n");
		goto no_supply_node;
	}

	array->vdd_wanted = cinfo->vdd_default;

	err = of_property_read_string(supply_node, "regulator-name",
				      &supply_name);
	if (err) {
		dev_info(elink->dev, "arrays: no regulator name\n");
		of_node_put(supply_node);
		return ERR_PTR(err);
	}

	supply = devm_regulator_get(elink->dev, supply_name);
	if (IS_ERR(supply)) {
		if (PTR_ERR(supply) == -EPROBE_DEFER) {
			dev_info(elink->dev,
				 "arrays: %s regulator not ready, retry\n",
				 np->name);
		} else {
			dev_info(elink->dev, "arrays: no regulator %s: %ld\n",
				 np->name, PTR_ERR(supply));
		}
		of_node_put(supply_node);
		return ERR_PTR(PTR_ERR(supply));
	}

	of_node_put(supply_node);
	array->supply = supply;

no_supply_node:

	switch (side) {
	case E_SIDE_N ... E_SIDE_W:
		array->connections[side].type = E_CONN_ELINK;
		array->connections[side].elink = elink;
		dev_dbg(elink->dev, "arrays: added connection\n");
		break;

	default:
		dev_err(elink->dev, "Invalid side %u\n", (u32) side);
		return ERR_PTR(-EINVAL);
	}

	/* TODO: Parse other connections */

	*out_side = side;
	return array;
}

/* Place holder */
static const struct attribute_group *dev_attr_groups_elink[] = {
	NULL
};

static int elink_clks_get(struct elink_device *elink)
{
	int ret, i;

	/* We might not need clocks for e.g., PCI. Error should have been
	 * raised in probe */
	if (!elink->clocks)
		return 0;

	for (i = 0; elink->clocks[i]; i++) {
		ret = clk_prepare_enable(elink->clocks[i]);
		if (ret)
			goto err;
	}

	return 0;

err:
	dev_err(elink->dev,
		"elink_clks_get: failed clk=%d, err=%d\n", i, ret);

	for (i--; i >= 0; i--)
		clk_disable_unprepare(elink->clocks[i]);

	return ret;
}

static void elink_clks_put(struct elink_device *elink)
{
	int i;

	if (!elink->clocks)
		return;

	/* Release in opposite order (not that it really matters atm). */

	for (i = 0; elink->clocks[i]; i++)
		;

	for (i--; i >= 0; i--)
		clk_disable_unprepare(elink->clocks[i]);
}

/* ??? TODO: Rename to elink_register_chardev ? */
static int elink_register(struct elink_device *elink)
{
	int ret;
	dev_t devt;

	ret = elink_clks_get(elink);
	if (ret)
		goto err_clks;

	ret = minor_get(elink);
	if (ret < 0)
		goto err_minor;

	elink->minor = ret;
	devt = MKDEV(MAJOR(epiphany_devt), elink->minor);
	cdev_init(&elink->cdev, &elink_char_driver_ops);
	elink->cdev.owner = THIS_MODULE;

	ret = cdev_add(&elink->cdev, devt, 1);
	if (ret) {
		dev_err(elink->dev,
			"CHAR registration failed for elink driver\n");
		goto err_cdev_add;
	}

	elink->char_dev =
		device_create_with_groups(epiphany_class, NULL, devt,
					  elink, dev_attr_groups_elink,
					  "elink%d", elink->minor);

	if (IS_ERR(elink->char_dev)) {
		dev_err(elink->dev, "unable to create device %d:%d\n",
			MAJOR(devt), MINOR(devt));
		ret = PTR_ERR(elink->char_dev);
		goto err_dev_create;
	}

	dev_dbg(elink->char_dev, "elink_register: registered char device\n");
	return 0;

err_dev_create:
	cdev_del(&elink->cdev);
err_cdev_add:
	minor_put(elink->minor);
err_minor:
	elink_clks_put(elink);
err_clks:
	return ret;
}

void elink_unregister(struct elink_device *elink)
{
	dev_t devt;

	mutex_lock(&driver_lock);
	list_del(&elink->list);
	mutex_unlock(&driver_lock);

	devt = elink->cdev.dev;

	cdev_del(&elink->cdev);

	device_destroy(epiphany_class, devt);

	minor_put(MINOR(devt));

	elink_clks_put(elink);

	/* Everything else is allocated with devm_* */

}


static int probe_elink(struct elink_device *elink)
{
	union e_syscfg_version version;
	int ret = 0;

	/* Since we're still in the probing phase we need to get clocks
	 * temporarily. */
	ret = elink_clks_get(elink);
	if (ret) {
		dev_err(elink->dev, "elinks: Could not get clocks\n");
		return ret;
	}

	version.reg = reg_read(elink->regs, E_SYS_VERSION);

	if (!version.generation || version.generation >= E_GEN_MAX) {
		dev_err(elink->dev, "elink: unsupported generation: 0x%x.\n",
			version.generation);
		ret = -EINVAL;
		goto out_clks;
	}

	if (!version.platform || version.platform >= E_PLATF_MAX) {
		dev_err(elink->dev, "elink: unsupported platform: 0x%x.\n",
			version.platform);
		ret = -EINVAL;
		goto out_clks;
	}

	/* setting coreid in fpga elink regs is a no-op with current
	 * bitstreams. */
	if (true) {
		elink->quirk_coreid_is_noop = true;
		elink->coreid_pinout = reg_read(elink->regs, E_SYS_COREID);
		dev_dbg(elink->dev,
			"elinks: quirk: setting coreid reg is no-op\n");
	}

	elink->version = version;
	elink->chip_type = elink_platform_chip_match[version.platform];

	dev_info(elink->dev, "Epiphany FPGA elink at address %pa\n",
		 &elink->regs_start);
	dev_info(elink->dev,
		 "revision %02x type %02x platform %02x generation %02x\n",
		 version.revision,
		 version.type,
		 version.platform,
		 version.generation);

out_clks:
	elink_clks_put(elink);

	return ret;
}

static int elink_of_probe_clks(struct elink_device *elink)
{
	int ret = 0, i = 0;

	static const char const *names[] = {
		"fclk0", "fclk1", "fclk2", "fclk3"
	};

	elink->clocks = devm_kcalloc(elink->dev, ARRAY_SIZE(names) + 1,
				     sizeof(struct clock *), GFP_KERNEL);
	if (!elink->clocks)
		return -ENOMEM;

	for (i = 0; i < ARRAY_SIZE(names); i++) {
		elink->clocks[i] = devm_clk_get(elink->dev, names[i]);
		if (IS_ERR(elink->clocks[i])) {
			ret = PTR_ERR(elink->clocks[i]);
			goto err;
		}

		dev_dbg(elink->dev, "Added clock: %s\n", names[i]);
	}

	return 0;

err:
	for (i--; i >= 0; i--)
		devm_clk_put(elink->dev, elink->clocks[i]);

	devm_kfree(elink->dev, elink->clocks);
	elink->clocks = NULL;

	return ret;
}

static int elink_of_probe(struct elink_device *elink)
{
	struct platform_device *pdev = to_platform_device(elink->dev);
	struct resource res;
	struct device_node *array_np;
	enum elink_side side = -1;
	u16 coreid;
	int ret;

	elink->phandle = elink->dev->of_node->phandle;
	elink->coreid_pinout = -1;

	/* Control regs */
	ret = of_address_to_resource(elink->dev->of_node, 0, &res);
	if (ret) {
		dev_err(elink->dev, "no control reg resource\n");
		return ret;
	}

	if (!devm_request_mem_region(elink->dev, res.start,
				     resource_size(&res), pdev->name)) {
		dev_err(elink->dev,
			"failed requesting control reg mem region\n");
		return -ENOMEM;
	}

	elink->regs_start = res.start;
	elink->regs_size = resource_size(&res);
	elink->regs = devm_ioremap_nocache(elink->dev, elink->regs_start,
					   elink->regs_size);

	if (!elink->regs) {
		dev_err(elink->dev, "Mapping eLink registers failed.\n");
		return -ENOMEM;
	}

	/* Host bus slave address range for emesh */
	ret = of_address_to_resource(elink->dev->of_node, 1, &res);
	if (ret) {
		dev_err(elink->dev, "no bus resource\n");
		return ret;
	}

	if (!devm_request_mem_region(elink->dev, res.start,
				     resource_size(&res), pdev->name)) {
		dev_err(elink->dev, "failed requesting emesh mem region\n");
		return -ENOMEM;
	}

	elink->emesh_start = res.start;
	elink->emesh_size = resource_size(&res);

	/* Clocks */
	ret = elink_of_probe_clks(elink);
	if (ret) {
		dev_err(elink->dev, "Could not get clocks\n");
		return ret;
	}

	/* Manually override coreid pinout. Set by array probe otherwise */
	ret = of_property_read_u16(elink->dev->of_node, "adapteva,coreid",
				   &coreid);
	if (!ret) {
		elink->coreid_pinout = (s16) coreid;
	} else if (ret == -EINVAL) {
		elink->coreid_pinout = -1;
	} else {
		dev_err(elink->dev, "Malformed coreid pinout dt property\n");
		return ret;
	}

	/* Need to probe elink here since some array options might not be
	 * supported by firmware version. */
	ret = probe_elink(elink);
	if (ret)
		return ret;

	/* Only support array connections for now */
	/* (and we will always only support 1 connection/elink) */
	array_np = get_next_compatible_child(elink->dev->of_node, NULL,
					     "adapteva,chip-array");
	if (!array_np) {
		dev_info(elink->dev, "elink has no connections\n");
		return 0;
	}

	elink->connection.array = elink_of_probe_chip_array(elink, array_np,
							    &side);
	if (IS_ERR(elink->connection.array)) {
		ret = PTR_ERR(elink->connection.array);
		elink->connection.array = NULL;
		return ret;
	}
	elink->connection.type = E_CONN_ARRAY;
	elink->connection.side = side;

	ret = elink_register(elink);
	if (ret)
		return ret;

	ret = array_register(elink->connection.array, elink->char_dev);
	if (ret) {
		dev_err(elink->dev, "Failed registering array dev\n");
		goto err_array_register;
	}

	list_add_tail(&elink->list, &epiphany_device->elink_list);

	return 0;

err_array_register:
	elink_unregister(elink);
	return ret;
}

static int elink_platform_probe(struct platform_device *pdev)
{
	struct elink_device *elink;
	int ret;

	elink = devm_kzalloc(&pdev->dev, sizeof(*elink), GFP_KERNEL);
	if (!elink)
		return -ENOMEM;

	elink->dev = &pdev->dev;

	ret = elink_of_probe(elink);
	if (ret) {
		if (ret == -EPROBE_DEFER)
			dev_info(elink->dev, "Deferring probe.\n");
		else
			dev_warn(elink->dev, "Failed parsing device tree\n");

		return ret;
	}

	platform_set_drvdata(pdev, elink);

	return 0;
}

static int elink_platform_remove(struct platform_device *pdev)
{
	struct elink_device *elink = platform_get_drvdata(pdev);

	if (elink->connection.type == E_CONN_ARRAY)
		array_unregister(elink->connection.array);

	elink_unregister(elink);

	dev_dbg(elink->dev, "device removed\n");

	return 0;
}

static const struct of_device_id elink_of_match[] = {
	{ .compatible = "adapteva,elink" },
	{ }
};
MODULE_DEVICE_TABLE(of, elink_of_match);

static struct platform_driver elink_driver = {
	.probe	= elink_platform_probe,
	.remove	= elink_platform_remove,
	.driver	= {
		.name		= "elink",
		.of_match_table	= of_match_ptr(elink_of_match)
	}
};

static char *epiphany_devnode(struct device *dev, umode_t *mode)
{
	return kasprintf(GFP_KERNEL, "epiphany/%s", dev_name(dev));
}

static int epiphany_probe_reserved_mem(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *mem_node;
	struct mem_region *mem_region;
	struct resource res;
	int ret = 0;

	/* TODO: Only one memory region supported for now */

	mem_node = of_parse_phandle(dev->of_node, "memory-region", 0);
	if (!mem_node) {
		/* TODO: When elink firmware has mmu we should accept no
		 * memory region and downgrade to a warning. */
		dev_err(dev, "reserved-mem: no memory-region\n");
		return -ENODEV;
	}

	ret = of_address_to_resource(mem_node, 0, &res);
	if (ret) {
		dev_warn(dev, "reserved-mem: no resource\n");
		goto out;
	}

	if (!devm_request_mem_region(dev, res.start, resource_size(&res),
				     pdev->name)) {
		dev_warn(dev, "reserved-mem: failed requesting mem region\n");
		ret = -ENOMEM;
		goto out;
	}

	mem_region = devm_kzalloc(dev, sizeof(*mem_region), GFP_KERNEL);
	if (!mem_region) {
		ret = -ENOMEM;
		goto out;
	}

	mem_region->phandle = mem_node->phandle;
	mem_region->start = res.start;
	mem_region->size = resource_size(&res);

	list_add_tail(&mem_region->list, &epiphany_device->mem_region_list);
	dev_dbg(dev, "reserved-mem: added mem region at 0x%x\n", res.start);

out:
	of_node_put(mem_node);
	return ret;
}

static int epiphany_platform_probe(struct platform_device *pdev)
{
	int ret;

	mutex_lock(&driver_lock);

	if (epiphany_device->reserved) {
		dev_err(&pdev->dev,
			"error: epiphany device already reserved\n");
		ret = -EBUSY;
		goto err;
	}

	epiphany_device->reserved = true;

	ret = epiphany_probe_reserved_mem(pdev);
	if (ret) {
		dev_err(&pdev->dev, "platform probe failed\n");
		goto err;
	}

	dev_dbg(&pdev->dev, "device added\n");

	mutex_unlock(&driver_lock);
	return 0;

err:
	mutex_unlock(&driver_lock);
	return ret;
}

static int epiphany_platform_remove(struct platform_device *pdev)
{
	struct list_head *curr, *next;

	mutex_lock(&driver_lock);
	list_for_each_safe(curr, next, &epiphany_device->mem_region_list)
		list_del(curr);
	epiphany_device->reserved = false;
	mutex_unlock(&driver_lock);

	dev_dbg(&pdev->dev, "device removed\n");
	return 0;
}

static const struct of_device_id epiphany_of_match[] = {
	{ .compatible = "adapteva,epiphany" },
	{ }
};
MODULE_DEVICE_TABLE(of, epiphany_of_match);

static struct platform_driver epiphany_driver = {
	.probe	= epiphany_platform_probe,
	.remove	= epiphany_platform_remove,
	.driver	= {
		.name		= "epiphany",
		.of_match_table	= of_match_ptr(epiphany_of_match)
	}
};

static int __init epiphany_module_init(void)
{
	int ret;

	epiphany_device = kzalloc(sizeof(*epiphany_device), GFP_KERNEL);
	if (!epiphany_device) {
		ret = -ENOMEM;
		goto err_kzalloc;
	}

	INIT_LIST_HEAD(&epiphany_device->elink_list);
	INIT_LIST_HEAD(&epiphany_device->mem_region_list);
	INIT_LIST_HEAD(&epiphany_device->chip_array_list);

	epiphany_class = class_create(THIS_MODULE, "epiphany");
	if (IS_ERR(epiphany_class)) {
		pr_err("Unable to create epiphany class\n");
		ret = PTR_ERR(epiphany_class);
		goto err_class;
	}
	epiphany_class->devnode = epiphany_devnode;

	ret = alloc_chrdev_region(&epiphany_devt, 0, E_DEV_NUM_MINORS,
				  "epiphany");
	if (ret) {
		pr_err("Failed allocating epiphany major number: %i\n", ret);
		goto err_chrdev;
	}
	pr_devel("epiphany device allocated, major %i\n", MAJOR(epiphany_devt));

	ret = platform_driver_register(&epiphany_driver);
	if (ret) {
		pr_err("Failed to register epiphany platform driver.\n");
		goto err_register_epiphany;
	}

	ret = platform_driver_register(&elink_driver);
	if (ret) {
		pr_err("Failed to register elink platform driver\n");
		goto err_register_elink;
	}

	return 0;

err_register_elink:
err_register_epiphany:
	unregister_chrdev_region(epiphany_devt, E_DEV_NUM_MINORS);
err_chrdev:
	class_destroy(epiphany_class);
err_class:
	kfree(epiphany_device);
err_kzalloc:
	return ret;
}
module_init(epiphany_module_init);

static void __exit epiphany_module_exit(void)
{
	platform_driver_unregister(&elink_driver);
	platform_driver_unregister(&epiphany_driver);
	unregister_chrdev_region(epiphany_devt, E_DEV_NUM_MINORS);
	class_destroy(epiphany_class);
	kfree(epiphany_device);
}
module_exit(epiphany_module_exit);

MODULE_DESCRIPTION("Adapteva Epiphany driver");
MODULE_VERSION("0.1");
MODULE_LICENSE("GPL");
