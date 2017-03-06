#ifndef __ASM_CHEETAH_IDB_H__
#define __ASM_CHEETAH_IDB_H__


struct idb_command 
{
    const char *cmdline;
    const char *help_msg;
    int (*func) (int argc, char *argv[]);

    struct idb_command* next;
};

int register_idb_command(struct idb_command *idb_command);
void unregister_idb_command(struct idb_command *idb_command);
int idb_print(const char *fmt, ...);

#endif /* __ASM_CHEETAH_IDB_H__ */

