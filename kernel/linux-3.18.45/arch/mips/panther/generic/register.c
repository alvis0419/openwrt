/*=============================================================================+
|                                                                              |
| Copyright 2008                                                               |
| Montage Inc. All right reserved.                                             |
|                                                                              |
+=============================================================================*/
/*! 
*   \file register.c
*   \brief  R/W register module
*   \author Montage
*/

#include <linux/types.h>
#include <linux/module.h>
#include <linux/skbuff.h>
#include <linux/proc_fs.h>
#include <linux/version.h>
#include <asm/mach-cheetah/idb.h>
#include <linux/seq_file.h>

MODULE_AUTHOR("Montage");
MODULE_DESCRIPTION("Register module");
MODULE_LICENSE("GPL");

#define REG_NAME "reg"
#define DBG(a...) //printk

#define BUFSIZE 1024
#define MAX_ARGV 8

static char buf[300];

int get_args (const char *string, char *argvs[])
{
	char *p;
	int n;

	argvs[0]=0;
	n = 0;
//  memset ((void*)argvs, 0, MAX_ARGV * sizeof (char *));
  p = (char *) string;
  while (*p == ' ')
	p++;
  while (*p)
    {
      argvs[n] = p;
      while (*p != ' ' && *p)
		p++;
	if (0==*p)
		goto out;
      *p++ = '\0';
      while (*p == ' ' && *p)
		p++;
out:
      n++;
      if (n == MAX_ARGV)
		break;
    }
    return n;
}
EXPORT_SYMBOL(get_args);

static int step_sz (char c)
{
    int step;
    switch (c) {
    case 'b':
        step = 1;
        break;

    case 'h':
        step = 2;
        break;

    default:
        step = 4;
    }
    return step;
}

static int vaddr_check(unsigned int addr)
{
    if (addr < 0x8000000)
        return -1;
    return 0;
}

static int hextoul (char *str, void *v)
{
    return(sscanf(str,"%x", (unsigned *)v)==1);
}

static unsigned int buf_address = 0x80000000UL;
int mem_dump_cmd (struct seq_file *s, int argc, char *argv[])
{
    unsigned int addr, caddr;
    unsigned int size;
    int step;

    step = step_sz (argv[-1][1]);
    addr = buf_address;
    size = 128;
    if (argc > 0 && !hextoul (argv[0], &addr))
        goto help;
    if (argc > 1 && !hextoul (argv[1], &size))
        goto help;

    if (size > BUFSIZE)
        size = BUFSIZE;

    addr &= ~(step - 1);
    if (vaddr_check(addr))
        goto err2;
    caddr = addr;
    
	while (caddr < (addr + size)) {
        if (caddr % 16 == 0 || caddr == addr) {
			seq_printf(s,"\n%08x ",(u32)caddr);
        }
        switch (step) {
        case 1:
            seq_printf(s, "%02x ", *((volatile unsigned char *) caddr));
            break;
        case 2:
            seq_printf(s, "%04x ", *((volatile unsigned short *) caddr));
            break;
        case 4:
            seq_printf(s, "%08x ", *((volatile unsigned int *) caddr));
            break;
        }
        caddr += step;
    }
    buf_address = addr + size;
    seq_printf(s, "\n");
    return 0;

    err2:
    return -1;

    help:
    return -2;
}

int mem_enter_cmd (struct seq_file *s, int argc, char *argv[])
{
    unsigned int addr;
    unsigned int size, val, i;
	u32 caddr;
    int step;

    step = step_sz (argv[-1][1]);

    size = 1;
    if (argc < 2)
        goto help;

    if (!hextoul (argv[0], &addr))
        goto err1;
    if (!hextoul (argv[1], &val))
        goto err1;
    if (argc > 2 && !hextoul (argv[2], &size))
        goto err1;

    addr &= ~(step - 1);
    if (vaddr_check(addr))
        goto err2;
	for (i = 0; i < size; i += step) {
		caddr = addr + i;
        switch (step) {
        case 1:
            *(volatile unsigned char *) caddr = val;
			seq_printf(s, "ADDR %08x = %02x\n", caddr, *((volatile unsigned char *) caddr));
          break;
        case 2:
            *(volatile unsigned short *) caddr = val;
			seq_printf(s, "ADDR %08x = %04x\n", caddr, *((volatile unsigned short *) caddr));
            break;
        case 4:
            *(volatile unsigned int *) caddr = val;
			seq_printf(s, "ADDR %08x = %08x\n", caddr, *((volatile unsigned int *) caddr));
            break;
        }
    }
    return 0;

    err1:
    return -1;
    err2:
    return -2;
    help:
    return -3;

}

int reg_cmd(struct seq_file *s, int argc, char *argv[])
{
	if (argc < 1)
	{	
		goto err;
	}

	if ( !strcmp(argv[0], "ew")
		 || !strcmp(argv[0], "eh")
		 || !strcmp(argv[0], "eb") )
	{
		mem_enter_cmd(s, argc-1, &argv[1]);
		goto done;
	}

	if ( !strcmp(argv[0], "dw")
		 || !strcmp(argv[0], "dh")
		 || !strcmp(argv[0], "db") )
	{
		mem_dump_cmd(s, argc-1, &argv[1]);
		goto done;
	}
	
done:
	return 0;

err:
	return 0;
}

static int reg_write(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
{
	memset(buf, 0, 300);
	if (count > 0 && count < 299) {
		if (copy_from_user(buf, buffer, count))
			return -EFAULT;
		buf[count-1]='\0';
	}
	
	return count;
}

static int reg_show(struct seq_file *s, void *priv)
{
	int	argc ;
	char * argv[MAX_ARGV] ;
	
	argc = get_args( (const char *)buf , argv );
	reg_cmd(s, argc, argv);
    
    return 0;
}

static int reg_open(struct inode *inode, struct file *file)
{
    int ret;
	
    ret = single_open(file, reg_show, NULL);
	
	return ret;
}

static const struct file_operations reg_fops = {
    .open       = reg_open,
    .read       = seq_read,
	.write		= reg_write,
    .llseek     = seq_lseek,
    .release    = single_release,
};

static int __init init(void)
{
#ifdef	CONFIG_PROC_FS
	struct proc_dir_entry *res;

	res = create_proc_entry(REG_NAME, S_IWUSR | S_IRUGO, NULL);
	if (!res)
		return -ENOMEM;

	res->proc_fops = &reg_fops;
#endif

	return 0 ;
}

static void __exit fini(void)
{
    remove_proc_entry(REG_NAME, NULL);
}

module_init(init);
module_exit(fini);
