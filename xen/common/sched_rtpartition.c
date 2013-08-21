/******************************************************************************
 * Preemptive Partition EDF/RM scheduler for xen
 *
 * by Sisu Xi, 2013, Washington University in Saint Louis
 * based on code of credit Scheduler
 */


#include <xen/config.h>
#include <xen/init.h>
#include <xen/lib.h>
#include <xen/sched.h>
#include <xen/domain.h>
#include <xen/delay.h>
#include <xen/event.h>
#include <xen/time.h>
#include <xen/perfc.h>
#include <xen/sched-if.h>
#include <xen/softirq.h>
#include <asm/atomic.h>
#include <xen/errno.h>
#include <xen/trace.h>
#include <xen/cpu.h>
#include <xen/keyhandler.h>
#include <xen/trace.h>

/*
 * Systme-wide private data, include a global RunQueue
 * The global lock is referenced by schedule_data.schedule_lock from all physical cpus.
 * It can be grabbed via vcpu_schedule_lock_irq()
 */
struct rtpartition_private {
    struct list_head sdom;  /* list of availalbe domains, used for dump */
    cpumask_t cpus;         /* cpumask_t of available physical cpus */
    unsigned priority_scheme;
};

static struct rtpartition_private _rtpartition_priv;

const struct scheduler sched_rtpartition_def = {
    .name           = "SMP RTPARTITION Scheduler",
    .opt_name       = "rtpartition",
    .sched_id       = XEN_SCHEDULER_RTPARTITION,
    .sched_data     = &_rtpartition_priv,

    // .dump_cpu_state = rtpartition_dump_pcpu,
    // .dump_settings  = rtpartition_dump,
    // .init           = rtpartition_init,
    // .deinit         = rtpartition_deinit,
    // .alloc_pdata    = rtpartition_alloc_pdata,
    // .free_pdata     = rtpartition_free_pdata,
    // .alloc_domdata  = rtpartition_alloc_domdata,
    // .free_domdata   = rtpartition_free_domdata,
    // .init_domain    = rtpartition_dom_init,
    // .destroy_domain = rtpartition_dom_destroy,
    // .alloc_vdata    = rtpartition_alloc_vdata,
    // .free_vdata     = rtpartition_free_vdata,
    // .insert_vcpu    = rtpartition_vcpu_insert,
    // .remove_vcpu    = rtpartition_vcpu_remove,

    // .adjust         = rtpartition_dom_cntl,

    // .pick_cpu       = rtpartition_cpu_pick,
    // .do_schedule    = rtpartition_schedule,
    // .sleep          = rtpartition_vcpu_sleep,
    // .wake           = rtpartition_vcpu_wake,
    // .context_saved  = rtpartition_context_saved,

    .yield          = NULL,
    .migrate        = NULL,
};