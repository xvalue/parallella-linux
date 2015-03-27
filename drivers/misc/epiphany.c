#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/mm.h>
#include <linux/list.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>

#include "epiphany.h"

#define DRIVERNAME	"epiphany"

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

enum e_chip_type {
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

static const struct {
	int rows;
	int cols;
	size_t core_mem;
	u16 elink_coreid[E_SIDE_MAX]; /* relative */
} epiphany_chip_info[E_CHIP_MAX] = {
	[E_CHIP_E16G301] = {
		.rows = 4,
		.cols = 4,
		.core_mem = 32768,

		.elink_coreid[E_SIDE_N] = COORDS(0, 2),
		.elink_coreid[E_SIDE_E] = COORDS(2, 3),
		.elink_coreid[E_SIDE_S] = COORDS(3, 2),
		.elink_coreid[E_SIDE_W] = COORDS(2, 0)
	},
	[E_CHIP_E64G401] = {
		.rows = 8,
		.cols = 8,
		.core_mem = 32768,

		.elink_coreid[E_SIDE_N] = COORDS(0, 2),
		.elink_coreid[E_SIDE_E] = COORDS(2, 7),
		.elink_coreid[E_SIDE_S] = COORDS(7, 2),
		.elink_coreid[E_SIDE_W] = COORDS(2, 0)
	}
};

static const struct {
	char *name;
	enum e_chip_type type;
} epiphany_chip_of_match[] = {
	{ "epiphany3", E_CHIP_E16G301 },
	{ "epiphany4", E_CHIP_E64G401 }
};

struct elink {
	phandle phandle;
	struct list_head list;

	void __iomem *regs;
	phys_addr_t regs_start;
	size_t regs_size;

	u16 coreid_pinout; /* core id pinout */

	struct connection *connection;
};

struct connection {
	struct list_head list;

	void *u;
	bool u_is_host_bridge;
	u32 u_rel_coreid;

	void *v;
	bool v_is_host_bridge;
	u32 v_rel_coreid;

	/* Helpers for parsing device tree */
	phandle u_phandle;
	phandle v_phandle;
};

struct chip_array {
	phandle phandle;
	struct list_head list;

	u16 id; /* north-west-most core */
	unsigned int chip_rows;
	unsigned int chip_cols;
	enum e_chip_type chip_type;
	struct connection **connections;
};

struct mem_region {
	phandle phandle;
	struct list_head list;
	phys_addr_t start;
	size_t size;
};

struct epiphany_device {
	struct platform_device *pdev;

	struct list_head elink_list;
	struct list_head connection_list;
	struct list_head mem_region_list;
	struct list_head chip_array_list;

	struct miscdevice misc;
};

static inline void epiphany_sleep(void)
{
	usleep_range(1000, 10000);
}

static inline void reg_write(u32 value, void __iomem *base, u32 offset)
{
	iowrite32(value, (u8 __iomem *)base + offset);
}

static inline u32 reg_read(void __iomem *base, u32 offset)
{
	return ioread32((u8 __iomem *)base + offset);
}

static inline struct epiphany_device *to_epiphany_device(struct file *file)
{
	return container_of(file->private_data, struct epiphany_device, misc);
}

static int epiphany_char_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int epiphany_char_release(struct inode *inode, struct file *file)
{
	return 0;
}

static int epiphany_char_mmap(struct file *file, struct vm_area_struct *vma)
{
	return -EINVAL;
}

static long epiphany_char_ioctl(struct file *file, unsigned int cmd,
				unsigned long arg)
{
	int err = 0;

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
	default:
		return -ENOTTY;
	}

	return 0;
}

static const struct file_operations epiphany_char_driver_ops = {
	.owner		= THIS_MODULE,
	.open		= epiphany_char_open,
	.release	= epiphany_char_release,
	.mmap		= epiphany_char_mmap,
	.unlocked_ioctl	= epiphany_char_ioctl
};

static int epiphany_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct epiphany_device *epiphany;
	int retval;

	epiphany = devm_kzalloc(dev, sizeof(*epiphany), GFP_KERNEL);
	if (!epiphany)
		return -ENOMEM;

	epiphany->pdev = pdev;

	epiphany->misc.minor = MISC_DYNAMIC_MINOR;
	epiphany->misc.name = DRIVERNAME;
	epiphany->misc.fops = &epiphany_char_driver_ops;

	retval = misc_register(&epiphany->misc);
	if (retval) {
		dev_warn(dev, "CHAR registration failed for epiphany driver\n");
		return retval;
	}

	INIT_LIST_HEAD(&epiphany->elink_list);
	INIT_LIST_HEAD(&epiphany->connection_list);
	INIT_LIST_HEAD(&epiphany->mem_region_list);
	INIT_LIST_HEAD(&epiphany->chip_array_list);

	platform_set_drvdata(pdev, epiphany);

	return 0;
}

static int epiphany_remove(struct platform_device *pdev)
{
	struct epiphany_device *epiphany = platform_get_drvdata(pdev);

	misc_deregister(&epiphany->misc);

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id epiphany_of_match[] = {
	{ .compatible = "adapteva,epiphany" },
	{ }
};
MODULE_DEVICE_TABLE(of, epiphany_of_match);
#endif

static struct platform_driver epiphany_driver = {
	.probe	= epiphany_probe,
	.remove	= epiphany_remove,
	.driver	= {
		.name		= "epiphany",
		.of_match_table	= of_match_ptr(epiphany_of_match),
	}
};
module_platform_driver(epiphany_driver);

MODULE_DESCRIPTION("Adapteva Epiphany driver");
MODULE_VERSION("0.1");
MODULE_LICENSE("GPL");
