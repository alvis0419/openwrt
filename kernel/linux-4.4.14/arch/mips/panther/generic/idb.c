#if defined(CONFIG_MAGIC_SYSRQ)
    #define SUPPORT_SYSRQ
    #include <linux/sysrq.h>
#endif

#include <linux/types.h>
#include <linux/serial.h>
#include <linux/serialP.h>
#include <linux/serial_reg.h>

#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/timer.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/console.h>
#include <linux/major.h>
#include <linux/string.h>
#include <linux/fcntl.h>
#include <linux/ioport.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/bitops.h>
#include <linux/utsname.h>

#if defined(CONFIG_KGDB)
#include <linux/kgdb.h>
#endif

#include <asm/reboot.h>

#include <asm/mach-cheetah/cheetah.h>
#include <asm/mach-cheetah/idb.h>

#define IDB_BUFSIZE     256
#define CLI_MAX_ARGV	12

static char idb_buf[IDB_BUFSIZE];
static char idb_cmdline[IDB_BUFSIZE];

static struct idb_command *idb_commands_head = NULL;

int register_idb_command(struct idb_command *idb_command)
{
    struct idb_command *cmds;

    idb_command->next = NULL;

    if(idb_commands_head == NULL) 
    {
        idb_commands_head = idb_command;
    }
    else
    {
        cmds = idb_commands_head;
        while(cmds->next != NULL)
            cmds = cmds->next;

        cmds->next = idb_command;
    }

    return 0;
}

void unregister_idb_command(struct idb_command *idb_command)
{
    struct idb_command *prev_cmd, *next_cmd;

    prev_cmd = next_cmd = idb_commands_head;

    while(next_cmd) 
    {
        if(next_cmd == idb_command) 
        {
            if(next_cmd == prev_cmd)
            {
                /* we are the first element */
                idb_commands_head = next_cmd->next;
            }
            else
            {
                prev_cmd->next = next_cmd->next;
            }
            break;
        }

        prev_cmd = next_cmd;
        next_cmd = next_cmd->next;       
    }

    idb_command->next = NULL;
}

EXPORT_SYMBOL(register_idb_command);
EXPORT_SYMBOL(unregister_idb_command);

int cheetah_uart_poll_init(struct tty_driver *driver, int line, char *options);
int cheetah_uart_poll_get_char(struct tty_driver *driver, int line);
void cheetah_uart_poll_put_char(struct tty_driver *driver, int line, char ch);

static void idb_putchar(char ch)
{
    if (ch == '\n') {
        cheetah_uart_poll_put_char(NULL, 0, 0xa);
        cheetah_uart_poll_put_char(NULL, 0, 0xd);
    } else
        cheetah_uart_poll_put_char(NULL, 0, ch);
}

static int idb_getchar(void)
{
    return cheetah_uart_poll_get_char(NULL, 0);
}

int idb_print(const char *fmt, ...)
{
    va_list args;
    int r;
    int i;

    va_start(args, fmt);
    r = vsnprintf(idb_buf,IDB_BUFSIZE,fmt,args);
    va_end(args);

    for (i=0;i<r;i++)
        idb_putchar(idb_buf[i]);

    return r;
}

EXPORT_SYMBOL(idb_print);

static int idb_getline(void)
{
    int ch;
    int count = 0;

    idb_buf[0] = '\0';
    idb_cmdline[0] = '\0';

    while (1) {
        ch = idb_getchar();

        switch (ch) {
        case 0xd:         /* CR*/
            goto Out;
            break;

        case 0x3:         /* Ctrl-C */
            idb_putchar('^');
            idb_putchar('C');
            idb_buf[0] = '\0';
            idb_cmdline[0] = '\0';
            count = 0;
            goto Out;
            break;

        case 0x08:
        case 0x7f:        /* Backspace */
            if (count >= 1) {
                idb_putchar(0x08);
                idb_putchar(' ');
                idb_putchar(0x08);
                idb_buf[--count] = '\0';
            }
            break;

        default:
            if((ch >= 0x20) && (ch < 0x80)) 
            {
                idb_buf[count++] = ch;
                idb_putchar(ch);
            }
            break;
        }
    }

    Out:
    if (count) {
        idb_buf[count] = '\0';

        strncpy(idb_cmdline, idb_buf, IDB_BUFSIZE);
    }

    return count;
}

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
static int vaddr_check(unsigned long addr)
{
    if (addr < 0x8000000)
        return -1;
    return 0;
}

static int hextoul (char *str, void *v)
{
    return(sscanf(str,"%x", (unsigned *)v)==1);
}

static unsigned long buf_address = 0x80000000UL;
static int mem_dump_cmd (int argc, char *argv[])
{
    unsigned long addr, caddr;
    unsigned long size;
    int step;

    step = step_sz (argv[-1][1]);
    addr = buf_address;
    size = 128;
    if (argc > 0 && !hextoul (argv[0], &addr))
        goto help;
    if (argc > 1 && !hextoul (argv[1], &size))
        goto help;

    if (size > 0x20000)
        size = 0x20000;

    addr &= ~(step - 1);
    if (vaddr_check(addr))
        goto err2;
    caddr = addr;
    while (caddr < (addr + size)) {
        if (caddr % 16 == 0 || caddr == addr) {
            idb_print("\n%08x: ", caddr);
        }
        switch (step) {
        case 1:
            idb_print("%02x ", *((volatile unsigned char *) caddr));
            break;
        case 2:
            idb_print("%04x ", *((volatile unsigned short *) caddr));
            break;
        case 4:
            idb_print("%08x ", *((volatile unsigned long *) caddr));
            break;
        }
        caddr += step;
    }
    buf_address = addr + size;
    idb_print("\n");
    return 0;

    err2:
    return -1; //E_ADDR;

    help:
    return -2; //E_HELP;
}

static int mem_enter_cmd (int argc, char *argv[])
{
    unsigned long addr;
    unsigned long size, Value, i;
    int step;

    step = step_sz (argv[-1][1]);

    size = 1;
    if (argc < 2)
        goto help;

    if (!hextoul (argv[0], &addr))
        goto err1;
    if (!hextoul (argv[1], &Value))
        goto err1;
    if (argc > 2 && !hextoul (argv[2], &size))
        goto err1;

    addr &= ~(step - 1);
    if (vaddr_check(addr))
        goto err2;
    for (i = 0; i < size; i += step) {
        switch (step) {
        case 1:
            *(volatile unsigned char *) (addr + i) = Value;

            idb_print("ADDR %08x = %02x\n", (addr + i), *((volatile unsigned char *) (addr + i)));
            break;
        case 2:
            *(volatile unsigned short *) (addr + i) = Value;

            idb_print("ADDR %08x = %04x\n", (addr + i), *((volatile unsigned short *) (addr + i)));
            break;
        case 4:
            *(volatile unsigned long *) (addr + i) = Value;

            idb_print("ADDR %08x = %08x\n", (addr + i), *((volatile unsigned long *) (addr + i)));
            break;
        }
    }
    return 0;

    err1:
    return -1; //E_PARM;
    err2:
    return -2; //E_UNALIGN;
    help:
    return -3; //E_HELP;

}

static int mem_search_cmd (int argc, char *argv[])
{
    unsigned long start_addr, end_addr;
    unsigned long i;
    unsigned long value;
    int step;

    step = step_sz (argv[-1][1]);

    if (argc != 3)
        goto help;

    if (!hextoul (argv[0], &start_addr))
        goto err1;
    if (!hextoul (argv[1], &end_addr))
        goto err1;
    if (!hextoul (argv[2], &value))
        goto err1;

    if((start_addr >= end_addr) || vaddr_check(start_addr))
        goto err2;

    start_addr &= ~(step - 1);

    for (i = start_addr; i < end_addr; i += step)
    {
        switch (step)
        {
        case 1:
            if(value == *((volatile unsigned char *)i))
                idb_print("ADDR %08x\n", i);
            break;
        case 2:
            if(value == *((volatile unsigned short *)i))
                idb_print("ADDR %08x\n", i);
            break;
        case 4:
            if(value == *((volatile unsigned long *)i))
                idb_print("ADDR %08x\n", i);
            break;
        }
    }
    return 0;
    err1:
    return -1; //E_PARM;
    err2:
    return -2; //E_UNALIGN;
    help:
    return -3; //E_HELP;
}

#if defined(CONFIG_KGDB)
static struct timer_list kgdb_breakpoint_timer;
static void exec_kgdb_breakpoint(unsigned long arg)
{
    kgdb_breakpoint();
}
void trigger_kgdb_breakpoint(void)
{
    setup_timer(&kgdb_breakpoint_timer, exec_kgdb_breakpoint, 0);
    kgdb_breakpoint_timer.expires = jiffies + 1;
    add_timer(&kgdb_breakpoint_timer);
}
#endif

static int kgdb_cmd(int argc, char *argv[])
{
#if defined(CONFIG_KGDB)
    if((argc>=1) && !strcmp(argv[0], "now"))
    {
        kgdb_breakpoint();
    }
    else
    {
        trigger_kgdb_breakpoint();
        idb_print("KGDB breakpoint will be triggered after leaving IDB command prompt.\n");
    }
    return 0;
#else
    idb_print("KGDB option does not enabled in current kernel configuration.\n");
    return -EINVAL;
#endif
}

static int reset_cmd(int argc, char *argv[])
{
	if(_machine_restart)
        _machine_restart(NULL);

    return 0;
}

static int backtrace_cmd(int argc, char *argv[])
{
    dump_stack();
    return 0;
}

static int uname_cmd(int argc, char *argv[])
{
    idb_print("%s %s %s\n", utsname()->sysname, utsname()->release, utsname()->version);

    return 0;
}

static int ___get_args (const char *string, char *argv[])
{
    char *p;
    int n;

    n = 0;
    p = (char *) string;
    while (*p == ' ')
        p++;
    while (*p) {
        argv[n] = p;
        while (*p != ' ' && *p)
            p++;
        *p++ = '\0';
        while (*p == ' ' && *p)
            p++;
        n++;
        if (n == CLI_MAX_ARGV)
            break;
    }
    return n;
}

void camelot_kernel_debugger(void)
{
	int	argc;
	char *argv[CLI_MAX_ARGV];
    int matched;
    struct idb_command *cmd;

    idb_print("Entering internal debugger, type \'?\' for help.\n");

    while (1) {
        idb_print("DB>");
        idb_getline();
        idb_print("\n");

        argc = ___get_args((const char *)idb_cmdline, argv);
        //idb_print("\r\n(%s)\r\n", idb_cmdline);

        if(argc) 
        {
            if(!strcmp(argv[0], "c"))
            {
                idb_print("Continue.\n");
                break;
            }
            else if(!strcmp(argv[0], "?"))
            {
                idb_print("c                                Leave the debugger\n");

                cmd = idb_commands_head;
                while(cmd) 
                {
                    idb_print("%s\n", cmd->help_msg);
                    cmd = cmd->next;
                }
            }
            else
            {
                matched = 0;
                cmd = idb_commands_head;
                while(cmd) 
                {
                    if(!strncmp(argv[0], cmd->cmdline, strlen(cmd->cmdline)))
                    {
                        cmd->func(argc - 1, argv + 1);
                        matched = 1;
                        break;
                    }
                    cmd = cmd->next;
                }

                if(matched == 0)
                    idb_print("Unknown command, type \'?\' for help.\n");
            }
        }
    }
}

#ifdef CONFIG_MAGIC_SYSRQ
static void sysrq_handle_idb(int key, struct tty_struct *tty)
{
    camelot_kernel_debugger();
}

static struct sysrq_key_op sysrq_idb_op = {
    .handler    = sysrq_handle_idb,
    .help_msg   = "debugger(A)",
    .action_msg = "Trigger internal debugger",
};
#endif

struct idb_command idb_mem_enter_cmd = 
{
    .cmdline = "e",
    .help_msg = "e<b,h,w> <addr> <value> <size>   Write memory/register",
    .func = mem_enter_cmd,
};

struct idb_command idb_mem_dump_cmd = 
{
    .cmdline = "d",
    .help_msg = "d<b,h,w> <addr> <size>           Dump memory/register",
    .func = mem_dump_cmd,
};

struct idb_command idb_mem_search_cmd = 
{
    .cmdline = "s",
    .help_msg = "s<b,h,w> <start addr> <end addr> <value>     Search memory/register",
    .func = mem_search_cmd,
};

struct idb_command idb_kgdb_cmd = 
{
    .cmdline = "gdb",
    .help_msg = "gdb <now>                        Toggle KGDB <now>",
    .func = kgdb_cmd,
};

struct idb_command idb_reset_cmd = 
{
    .cmdline = "reset",
    .help_msg = "reset                            Reset the system",
    .func = reset_cmd,
};

struct idb_command idb_backtrace_cmd = 
{
    .cmdline = "bt",
    .help_msg = "bt                               Backtrace",
    .func = backtrace_cmd,
};

struct idb_command idb_uname_cmd = 
{
    .cmdline = "uname",
    .help_msg = "uname                            Display system name and version",
    .func = uname_cmd,
};

static int __init idb_init(void)
{
#ifdef CONFIG_MAGIC_SYSRQ
    register_sysrq_key('a', &sysrq_idb_op);
#endif
    register_idb_command(&idb_mem_enter_cmd);
    register_idb_command(&idb_mem_dump_cmd);
    register_idb_command(&idb_mem_search_cmd);
    register_idb_command(&idb_kgdb_cmd);
    register_idb_command(&idb_reset_cmd);
    register_idb_command(&idb_backtrace_cmd);
    register_idb_command(&idb_uname_cmd);

    return 0;
}

static __exit void idb_exit(void) 
{
#ifdef CONFIG_MAGIC_SYSRQ
    unregister_sysrq_key('a', &sysrq_idb_op);
#endif
    unregister_idb_command(&idb_mem_enter_cmd);
    unregister_idb_command(&idb_mem_dump_cmd);
    unregister_idb_command(&idb_mem_search_cmd);
    unregister_idb_command(&idb_kgdb_cmd);
    unregister_idb_command(&idb_reset_cmd);
    unregister_idb_command(&idb_backtrace_cmd);
    unregister_idb_command(&idb_uname_cmd);
}

module_init(idb_init)
module_exit(idb_exit)

