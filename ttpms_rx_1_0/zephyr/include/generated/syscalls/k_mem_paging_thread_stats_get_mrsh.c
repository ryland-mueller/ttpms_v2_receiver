/* auto-generated by gen_syscalls.py, don't edit */

#include <syscalls/mem_manage.h>

extern void z_vrfy_k_mem_paging_thread_stats_get(struct k_thread * thread, struct k_mem_paging_stats_t * stats);
uintptr_t z_mrsh_k_mem_paging_thread_stats_get(uintptr_t arg0, uintptr_t arg1, uintptr_t arg2,
		uintptr_t arg3, uintptr_t arg4, uintptr_t arg5, void *ssf)
{
	_current->syscall_frame = ssf;
	(void) arg2;	/* unused */
	(void) arg3;	/* unused */
	(void) arg4;	/* unused */
	(void) arg5;	/* unused */
	union { uintptr_t x; struct k_thread * val; } parm0;
	parm0.x = arg0;
	union { uintptr_t x; struct k_mem_paging_stats_t * val; } parm1;
	parm1.x = arg1;
	z_vrfy_k_mem_paging_thread_stats_get(parm0.val, parm1.val);
	_current->syscall_frame = NULL;
	return 0;
}

