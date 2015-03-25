#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_address.h>
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

#define MAX_ELINK_BRIDGES	256
#define MAX_CONNECTIONS		1024

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

static int dt_probe_reserved_mem(struct epiphany_device *epiphany)
{
	struct platform_device *pdev = epiphany->pdev;
	struct device *dev = &pdev->dev;
	struct device_node *memory;
	struct mem_region *mem_region;
	struct resource res;
	int err, retval = 0;

	/* TODO: Only one memory region supported for now */

	memory = of_parse_phandle(dev->of_node, "memory-region", 0);
	if (!memory) {
		dev_warn(dev, "reserved-mem: no memory-region\n");
		return -ENODEV;
	}

	err = of_address_to_resource(memory, 0, &res);
	if (err) {
		dev_warn(dev, "reserved-mem: no resource\n");
		retval = err;
		goto out;
	}

	if (!devm_request_mem_region(dev, res.start, resource_size(&res),
				     pdev->name)) {
		dev_warn(dev, "reserved-mem: failed requesting mem region\n");
		retval = -ENOMEM;
		goto out;
	}

	mem_region = devm_kzalloc(dev, sizeof(*mem_region), GFP_KERNEL);
	if (!mem_region) {
		retval = -ENOMEM;
		goto out;
	}

	mem_region->phandle = memory->phandle;
	mem_region->start = res.start;
	mem_region->size = resource_size(&res);

	list_add_tail(&mem_region->list, &epiphany->mem_region_list);
	dev_dbg(dev, "reserved-mem: added mem region at 0x%x\n", res.start);

out:
	of_node_put(memory);
	return retval;
}

static int dt_probe_connections(struct epiphany_device *epiphany)
{
	struct platform_device *pdev = epiphany->pdev;
	struct device *dev = &pdev->dev;
	struct connection *connection;
	struct of_phandle_args u_args, v_args;
	int err;
	int retval = 0; /* Allow 0 connections - hot-plugging */
	int i;

	for (i = 0; i < MAX_CONNECTIONS; i++) {
		err = of_parse_phandle_with_fixed_args(
				dev->of_node, "adapteva,connections", 1,
				(2 * i), &u_args);
		if (err)
			break;

		err = of_parse_phandle_with_fixed_args(
				dev->of_node, "adapteva,connections", 1,
				(2 * i + 1), &v_args);
		if (err) {
			retval = -EINVAL;
			goto put_u;
		}

		connection = devm_kzalloc(dev, sizeof(*connection), GFP_KERNEL);
		if (!connection) {
			retval = -ENOMEM;
			goto put_v;
		}

		connection->u_phandle = u_args.np->phandle;
		connection->u_rel_coreid = u_args.args[0];
		connection->v_phandle = v_args.np->phandle;
		connection->v_rel_coreid = v_args.args[0];

		list_add_tail(&connection->list, &epiphany->connection_list);

		dev_dbg(dev, "connection: added connection %s  <-->  %s\n",
			u_args.np->name, v_args.np->name);

put_v:
		of_node_put(v_args.np);
put_u:
		of_node_put(u_args.np);

		if (retval)
			break;
	}

	return retval;
}

static int dt_probe_elinks(struct epiphany_device *epiphany)
{
	struct platform_device *pdev = epiphany->pdev;
	struct device *dev = &pdev->dev;
	struct device_node *np;
	struct elink *elink;
	struct connection *connection;
	struct resource res;
	int err;
	int retval = -ENOENT; /* Require at least one entry */
	int i;

	for (i = 0; i < MAX_ELINK_BRIDGES; i++) {
		np = of_parse_phandle(dev->of_node, "adapteva,elinks", i);
		if (!np)
			break;

		err = of_address_to_resource(np, 0, &res);
		if (err) {
			dev_warn(dev, "elinks: no resource\n");
			retval = err;
			break;
		}

		if (!devm_request_mem_region(dev, res.start,
					     resource_size(&res), pdev->name)) {
			dev_warn(dev, "elinks: failed requesting mem region\n");
			retval = -ENOMEM;
			break;
		}

		elink = devm_kzalloc(dev, sizeof(*elink), GFP_KERNEL);
		if (!elink) {
			retval = -ENOMEM;
			break;
		}

		elink->regs_start = res.start;
		elink->regs_size = resource_size(&res);
		elink->regs = devm_ioremap_nocache(dev, elink->regs_start,
						   elink->regs_size);

		if (!elink->regs) {
			dev_warn(dev, "elinks: Mapping eLink registers failed.\n");
			retval = -ENOMEM;
			break;
		}

		elink->phandle = np->phandle;
		err = of_property_read_u16(np, "adapteva,coreids",
					   &elink->coreid_pinout);
		if (err) {
			dev_warn(dev, "elinks: Could not read coreid pinout\n");
			retval = err;
			break;
		}

		/* elink->type = probe_elink(epiphany, elink) */

		list_for_each_entry(connection, &epiphany->connection_list,
				    list) {
			if (connection->u_phandle != elink->phandle &&
			    connection->v_phandle != elink->phandle)
				continue;

			if (connection->u_phandle == elink->phandle) {
				connection->u = elink;
				connection->u_is_host_bridge = true;
			} else {
				connection->v = elink;
				connection->v_is_host_bridge = true;
			}
			elink->connection = connection;
			dev_dbg(dev, "elinks: added connection\n");
		}

		list_add_tail(&elink->list, &epiphany->elink_list);
		dev_dbg(dev, "elinks: added elink\n");

		/* Found at least one elink */
		retval = 0;
	}

	return retval;
}

static int dt_probe_chip_arrays(struct epiphany_device *epiphany)
{
	struct platform_device *pdev = epiphany->pdev;
	struct device *dev = &pdev->dev;
	struct device_node *np, *emesh;
	struct chip_array *array;
	struct connection *connection;
	struct connection **conn;
	u32 reg[3];
	int nconns = 0;
	int err;
	int i;
	int retval = 0; /* Allow 0 arrays - hot-plugging */
	const char *model;

	emesh = of_get_child_by_name(dev->of_node, "epiphany-mesh");
	if (!emesh) {
		dev_warn(dev, "arrays: no epiphany-mesh node\n");
		return -ENOENT;
	}

	for_each_child_of_node(emesh, np) {

		array = devm_kzalloc(dev, sizeof(*array), GFP_KERNEL);
		if (!array) {
			retval = -ENOMEM;
			break;
		}

		/* Figure out how many connections array has */
		list_for_each_entry(connection, &epiphany->connection_list,
				    list) {
			if (connection->u_phandle == np->phandle ||
			    connection->v_phandle == np->phandle)
				nconns++;
			dev_dbg(dev, "arrays: found connection %d\n",
				np->phandle);
		}
		/* ... so we can allocate the right amount of memory */

		array->connections = devm_kcalloc(dev, nconns + 1,
						  sizeof(*conn), GFP_KERNEL);
		array->phandle = np->phandle;

		/* There is probably a better way for doing this */
		err = of_property_read_u32_array(np, "reg", reg, 3);
		if (err) {
			dev_warn(dev, "arrays: invalid reg property\n");
			retval = err;
			break;
		}

		array->id = (u16) reg[0];
		array->chip_rows = reg[1];
		array->chip_cols = reg[2];

		err = of_property_read_string(np, "adapteva,chip-model",
					      &model);
		if (err) {
			dev_warn(dev, "arrays: invalid chip model property\n");
			retval = err;
			break;
		}

		err = -ENOENT;
		for (i = 0; i < ARRAY_SIZE(epiphany_chip_of_match); i++) {
			if (!strcmp(model, epiphany_chip_of_match[i].name)) {
				array->chip_type =
					epiphany_chip_of_match[i].type;
				err = 0;
				break;
			}
		}
		if (err) {
			dev_warn(dev, "arrays: invalid chip model\n");
			retval = err;
			break;
		}

		conn = &array->connections[0];
		list_for_each_entry(connection, &epiphany->connection_list,
				    list) {
			if (connection->u_phandle != array->phandle &&
			    connection->v_phandle != array->phandle)
				continue;

			if (connection->u_phandle == array->phandle) {
				connection->u = array;
				connection->u_is_host_bridge = false;
			} else {
				connection->v = array;
				connection->v_is_host_bridge = false;
			}
			*conn = connection;
			conn++;
			dev_dbg(dev, "arrays: added connection\n");
		}

		list_add_tail(&array->list, &epiphany->chip_array_list);
	}

	return retval;
}

static int dt_probe(struct epiphany_device *epiphany)
{
	struct platform_device *pdev = epiphany->pdev;
	struct device *dev = &pdev->dev;
	int err;

	err = dt_probe_reserved_mem(epiphany);
	if (err) {
		dev_warn(dev, "dt: error parsing reserved memory.\n");
		return err;
	}
	err = dt_probe_connections(epiphany);
	if (err) {
		dev_warn(dev, "dt: error parsing connections.\n");
		return err;
	}
	err = dt_probe_elinks(epiphany);
	if (err) {
		dev_warn(dev, "dt: error parsing elinks.\n");
		return err;
	}
	err = dt_probe_chip_arrays(epiphany);
	if (err) {
		dev_warn(dev, "dt: error parsing arrays.\n");
		return err;
	}

	return 0;
}

static int epiphany_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct epiphany_device *epiphany;
	struct elink *elink;
	union e_syscfg_version version;
	int retval;

	epiphany = devm_kzalloc(dev, sizeof(*epiphany), GFP_KERNEL);
	if (!epiphany)
		return -ENOMEM;

	INIT_LIST_HEAD(&epiphany->elink_list);
	INIT_LIST_HEAD(&epiphany->connection_list);
	INIT_LIST_HEAD(&epiphany->mem_region_list);
	INIT_LIST_HEAD(&epiphany->chip_array_list);

	epiphany->pdev = pdev;

	retval = dt_probe(epiphany);
	if (retval) {
		if (retval == -EPROBE_DEFER)
			dev_info(dev, "Deferring probe.\n");
		else
			dev_warn(dev, "Failed parsing device tree\n");

		return retval;
	}

	epiphany->misc.minor = MISC_DYNAMIC_MINOR;
	epiphany->misc.name = DRIVERNAME;
	epiphany->misc.fops = &epiphany_char_driver_ops;

	retval = misc_register(&epiphany->misc);
	if (retval) {
		dev_warn(dev, "CHAR registration failed for epiphany driver\n");
		return retval;
	}

	platform_set_drvdata(pdev, epiphany);

	list_for_each_entry(elink, &epiphany->elink_list, list) {
		dev_info(dev, "Epiphany FPGA elink at address %pa\n",
			 &elink->regs_start);
		version.reg = reg_read(elink->regs, E_SYS_VERSION);
		dev_info(dev,
			 "revision %02x type %02x platform %02x generation %02x\n",
			 version.revision,
			 version.type,
			 version.platform,
			 version.generation);
	}

	return 0;
}

static int epiphany_remove(struct platform_device *pdev)
{
	struct epiphany_device *epiphany = platform_get_drvdata(pdev);

	misc_deregister(&epiphany->misc);

	/* Everything else is allocated with devm_* */

	dev_dbg(&epiphany->pdev->dev, "device removed\n");

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
