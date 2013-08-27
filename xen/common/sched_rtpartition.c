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
 * TODO:
 *
 * How to show individual VCPU info?
 * More testing with xentrace and xenanalyze
 */

/*
 * Design:
 *
 * Follows the pre-emptive Partition EDF/RM theory.
 * Each VCPU can have a dedicated period/budget pair of parameter. When a VCPU is running, it burns its budget, and when there are no budget, the VCPU need to wait unitl next period to get replensihed. Any unused budget is discarded in the end of period.
 * Server mechanism: deferrable server is used here. Therefore, when a VCPU has no task but with budget left, the budget is preserved.
 * Priority scheme: a global variable called priority_scheme is used to switch between EDF and RM
 * Queue scheme: partitioned runqueue is used here. Each PCPU holds a dedicated run queue. VCPUs are divided into two parts: with and without remaining budget. Among each part, VCPUs are sorted by their current deadlines.
 * Scheduling quanta: 1 ms is picked as the scheduling quanta, but the accounting is done in microsecond level.
 * Other: VCPU migration is supported via command. No load balancing is provided by default.
 */

 /*
 * Locking:
 * This is a partitioned queue implementation
 */

/*
 * Default parameters
 */
#define RTPARTITION_DEFAULT_PERIOD     10
#define RTPARTITION_DEFAULT_BUDGET      4

#define EDF                             0
#define RM                              1

/*
 * Useful macros
 */
#define RTPARTITION_PRIV(_ops)      ((struct rtpartition_private *)((_ops)->sched_data))
#define RTPARTITION_PCPU(_cpu)      ((struct rtpartition_pcpu *)per_cpu(schedule_data, _cpu).sched_priv)
#define RTPARTITION_VCPU(_vcpu)     ((struct rtpartition_vcpu *)(_vcpu)->sched_priv)
#define RTPARTITION_DOM(_dom)       ((struct rtpartition_dom *)(_dom)->sched_priv)
#define RUNQ(_cpu)                  (&RTPARTITION_PCPU(_cpu)->runq)

/*
 * Used to printout debug information
 */
#define printtime()     ( printk("%d : %3ld.%3ld : %-19s ", smp_processor_id(), NOW()/MILLISECS(1), NOW()%MILLISECS(1)/1000, __func__) )

/*
 * Systme-wide private data, include a global RunQueue
 * The global lock is referenced by schedule_data.schedule_lock from all physical cpus.
 * It can be grabbed via vcpu_schedule_lock_irq()
 */
struct rtpartition_private {
    spinlock_t lock;        /* used for irqsave */
    struct list_head sdom;  /* list of availalbe domains, used for dump */
    cpumask_t cpus;         /* cpumask_t of available physical cpus */
    unsigned priority_scheme;       /* EDF or RM */
};

/*
 * Physical CPU
 */
struct rtpartition_pcpu {
    struct list_head runq;  /* per pcpu runq */
};

/*
 * Virtual CPU
 */
struct rtpartition_vcpu {
    struct list_head runq_elem; /* On the runqueue list */
    struct list_head sdom_elem; /* On the domain VCPU list */

    /* Up-pointers */
    struct rtpartition_dom *sdom;
    struct vcpu *vcpu;

    /* RTPARTITION parameters, in milliseconds */
    int period;
    int budget;

    /* VCPU current infomation */
    long cur_budget;             /* current budget in microseconds, if none, should not schedule unless extra */
    s_time_t last_start;        /* last start time, used to calculate budget */
    s_time_t cur_deadline;      /* current deadline, used to do EDF */
};

/*
 * Domain
 */
struct rtpartition_dom {
    struct list_head vcpu;      /* link its VCPUs */
    struct list_head sdom_elem; /* link list on rtpartition_priv */
    struct domain *dom;         /* pointer to upper domain */
    int    extra;               /* not evaluated */
};

/*
 * RunQueue helper functions
 */
static int
__vcpu_on_runq(struct rtpartition_vcpu *svc)
{
   return !list_empty(&svc->runq_elem);
}

static struct rtpartition_vcpu *
__runq_elem(struct list_head *elem)
{
    return list_entry(elem, struct rtpartition_vcpu, runq_elem);
}

/* lock is grabbed before calling this function */
static inline void
__runq_remove(struct rtpartition_vcpu *svc)
{
    if ( __vcpu_on_runq(svc) )
        list_del_init(&svc->runq_elem);
}

/* lock is grabbed before calling this function */
static void
__runq_insert(const struct scheduler *ops, unsigned int cpu, struct rtpartition_vcpu *svc)
{
    struct list_head *runq = RUNQ(cpu);
    struct list_head *iter;
    struct rtpartition_private *prv = RTPARTITION_PRIV(ops);
    ASSERT( spin_is_locked(per_cpu(schedule_data, svc->vcpu->processor).schedule_lock) );

    if ( __vcpu_on_runq(svc) )
        return;

    list_for_each(iter, runq) {
        struct rtpartition_vcpu * iter_svc = __runq_elem(iter);

        if ( svc->cur_budget > 0 ) { // svc still has budget
            if ( iter_svc->cur_budget == 0 ||
                 ( ( prv->priority_scheme == EDF && svc->cur_deadline < iter_svc->cur_deadline ) ||
                   ( prv->priority_scheme == RM && svc->period < iter_svc->period )) ) {
                    break;
            }
        } else { // svc has no budget
            if ( iter_svc->cur_budget == 0 &&
                 ( ( prv->priority_scheme == EDF && svc->cur_deadline < iter_svc->cur_deadline ) ||
                   ( prv->priority_scheme == RM && svc->period < iter_svc->period )) ) {
                    break;
            }
        }
    }

    list_add_tail(&svc->runq_elem, iter);
}

/*
 * Debug related code, dump vcpu/pcpu
 */
static void
rtpartition_dump_vcpu(struct rtpartition_vcpu *svc)
{
    if ( svc == NULL ) {
        printk("NULL!\n");
        return;
    }
// #define cpustr keyhandler_scratch
    // cpumask_scnprintf(cpustr, sizeof(cpustr), svc->vcpu->cpu_affinity);
    printk("[%5d.%-2d] cpu %d, (%-2d, %-2d), cur_b=%ld cur_d=%lu last_start=%lu onR=%d runnable=%d\n",
        // , affinity=%s\n",
            svc->vcpu->domain->domain_id,
            svc->vcpu->vcpu_id,
            svc->vcpu->processor,
            svc->period,
            svc->budget,
            svc->cur_budget,
            svc->cur_deadline,
            svc->last_start,
            __vcpu_on_runq(svc),
            vcpu_runnable(svc->vcpu));
            // cpustr);
// #undef cpustr
}

static void
rtpartition_dump_pcpu(const struct scheduler *ops, int cpu)
{
    struct rtpartition_vcpu *svc = RTPARTITION_VCPU(curr_on_cpu(cpu));
    struct list_head *runq, *iter;
    int loop = 0;

    printtime();
    printk("On cpu %d, running: ", cpu);
    rtpartition_dump_vcpu(svc);

    printk("RunQ info: \n");
    runq = RUNQ(cpu);
    list_for_each( iter, runq ) {
        svc = __runq_elem(iter);
        printk("\t%3d: ", ++loop);
        rtpartition_dump_vcpu(svc);
    }
}

/* should not need lock here. only showing stuff */
static void
rtpartition_dump(const struct scheduler *ops)
{
    struct list_head *iter_sdom, *iter_svc;
    struct rtpartition_private *prv = RTPARTITION_PRIV(ops);
    struct rtpartition_vcpu *svc;
    int cpu = 0;
    int loop = 0;

    printtime();
    if ( prv->priority_scheme == EDF ) printk("EDF\n");
    else printk ("RM\n");

    printk("PCPU info: \n");
    for_each_cpu(cpu, &prv->cpus) {
        rtpartition_dump_pcpu(ops, cpu);
    }

    printk("Domain info: \n");
    loop = 0;
    list_for_each( iter_sdom, &prv->sdom ) {
        struct rtpartition_dom *sdom;
        sdom = list_entry(iter_sdom, struct rtpartition_dom, sdom_elem);
        printk("\tdomain: %d\n", sdom->dom->domain_id);

        list_for_each( iter_svc, &sdom->vcpu ) {
            svc = list_entry(iter_svc, struct rtpartition_vcpu, sdom_elem);
            printk("\t\t%3d: ", ++loop);
            rtpartition_dump_vcpu(svc);
        }
    }

    printk("\n");
}

/*
 * Init/Free related code
 */
static int
rtpartition_init(struct scheduler *ops)
{
    struct rtpartition_private *prv;

    prv = xmalloc(struct rtpartition_private);
    if ( prv == NULL ) {
        printk("malloc failed at rtpartition_private\n");
        return -ENOMEM;
    }
    memset(prv, 0, sizeof(*prv));

    ops->sched_data = prv;
    spin_lock_init(&prv->lock);
    INIT_LIST_HEAD(&prv->sdom);
    cpumask_clear(&prv->cpus);
    prv->priority_scheme = EDF;     /* by default, use EDF scheduler */

    printk("This is the Deferrable Server version of the preemptive RTPARTITION scheduler\n");
    printk("If you want to use it as a periodic server, please run a background busy CPU task\n");

    printtime();
    printk("\n");

    return 0;
}

static void
rtpartition_deinit(const struct scheduler *ops)
{
    struct rtpartition_private *prv;

    printtime();
    printk("\n");

    prv = RTPARTITION_PRIV(ops);
    if ( prv )
        xfree(prv);
}

/* point per_cpu spinlock to the global system lock */
static void *
rtpartition_alloc_pdata(const struct scheduler *ops, int cpu)
{
    struct rtpartition_private *prv = RTPARTITION_PRIV(ops);
    struct rtpartition_pcpu *spc;
    unsigned long flags;

    spc = xzalloc(struct rtpartition_pcpu);
    if ( spc == NULL )
        return NULL;

    spin_lock_irqsave(&prv->lock, flags);

    cpumask_set_cpu(cpu, &prv->cpus);
    INIT_LIST_HEAD(&spc->runq);
    if ( per_cpu(schedule_data, cpu).sched_priv == NULL )
        per_cpu(schedule_data, cpu).sched_priv = spc;

    spin_unlock_irqrestore(&prv->lock, flags);

    printtime();
    printk("total cpus: %d", cpumask_weight(&prv->cpus));
    return spc;
}

static void
rtpartition_free_pdata(const struct scheduler *ops, void *pcpu, int cpu)
{
    struct rtpartition_private * prv = RTPARTITION_PRIV(ops);
    struct rtpartition_pcpu * spc = pcpu;

    cpumask_clear_cpu(cpu, &prv->cpus);
    printtime();
    printk("cpu=%d\n", cpu);
    xfree(spc);
}

static void *
rtpartition_alloc_domdata(const struct scheduler *ops, struct domain *dom)
{
    unsigned long flags;
    struct rtpartition_dom *sdom;
    struct rtpartition_private * prv = RTPARTITION_PRIV(ops);

    printtime();
    printk("dom=%d\n", dom->domain_id);

    sdom = xmalloc(struct rtpartition_dom);
    if ( sdom == NULL ) {
        printk("%s, xmalloc failed\n", __func__);
        return NULL;
    }
    memset(sdom, 0, sizeof(*sdom));

    INIT_LIST_HEAD(&sdom->vcpu);
    INIT_LIST_HEAD(&sdom->sdom_elem);
    sdom->dom = dom;
    sdom->extra = 0;         /* by default, should not allow extra time */

    /* spinlock here to insert the dom */
    spin_lock_irqsave(&prv->lock, flags);
    list_add_tail(&sdom->sdom_elem, &(prv->sdom));
    spin_unlock_irqrestore(&prv->lock, flags);

    return (void *)sdom;
}

static void
rtpartition_free_domdata(const struct scheduler *ops, void *data)
{
    unsigned long flags;
    struct rtpartition_dom *sdom = data;
    struct rtpartition_private * prv = RTPARTITION_PRIV(ops);

    printtime();
    printk("dom=%d\n", sdom->dom->domain_id);

    spin_lock_irqsave(&prv->lock, flags);
    list_del_init(&sdom->sdom_elem);
    spin_unlock_irqrestore(&prv->lock, flags);
    xfree(data);
}

static int
rtpartition_dom_init(const struct scheduler *ops, struct domain *dom)
{
    struct rtpartition_dom *sdom;

    printtime();
    printk("dom=%d\n", dom->domain_id);

    /* IDLE Domain does not link on rtpartition_private */
    if ( is_idle_domain(dom) ) { return 0; }

    sdom = rtpartition_alloc_domdata(ops, dom);
    if ( sdom == NULL ) {
        printk("%s, failed\n", __func__);
        return -ENOMEM;
    }
    dom->sched_priv = sdom;

    return 0;
}

static void
rtpartition_dom_destroy(const struct scheduler *ops, struct domain *dom)
{
    printtime();
    printk("dom=%d\n", dom->domain_id);

    rtpartition_free_domdata(ops, RTPARTITION_DOM(dom));
}

static void *
rtpartition_alloc_vdata(const struct scheduler *ops, struct vcpu *vc, void *dd)
{
    struct rtpartition_vcpu *svc;
    s_time_t now = NOW();
    long count;

    /* Allocate per-VCPU info */
    svc = xmalloc(struct rtpartition_vcpu);
    if ( svc == NULL ) {
        printk("%s, xmalloc failed\n", __func__);
        return NULL;
    }
    memset(svc, 0, sizeof(*svc));

    INIT_LIST_HEAD(&svc->runq_elem);
    INIT_LIST_HEAD(&svc->sdom_elem);
    svc->sdom = dd;
    svc->vcpu = vc;
    svc->last_start = 0;            /* init last_start is 0 */

    svc->period = RTPARTITION_DEFAULT_PERIOD;
    if ( !is_idle_vcpu(vc) && vc->domain->domain_id != 0 ) {
        svc->budget = RTPARTITION_DEFAULT_BUDGET;
    } else {
        svc->budget = RTPARTITION_DEFAULT_PERIOD;
    }

    count = (now/MILLISECS(svc->period)) + 1;
    svc->cur_deadline += count*MILLISECS(svc->period); /* sync all VCPU's start time to 0 */

    svc->cur_budget = svc->budget*1000; /* counting in microseconds level */
    printtime();
    rtpartition_dump_vcpu(svc);

    return svc;
}

static void
rtpartition_free_vdata(const struct scheduler *ops, void *priv)
{
    struct rtpartition_vcpu *svc = priv;
    printtime();
    rtpartition_dump_vcpu(svc);
    xfree(svc);
}

/* lock is grabbed before calling this function */
/* only link VCPU to dom, insert to runq is deferred */
static void
rtpartition_vcpu_insert(const struct scheduler *ops, struct vcpu *vc)
{
    struct rtpartition_vcpu *svc = RTPARTITION_VCPU(vc);

    printtime();
    rtpartition_dump_vcpu(svc);

    /* IDLE VCPU not allowed on RunQ */
    if ( is_idle_vcpu(vc) )
        return;

    list_add_tail(&svc->sdom_elem, &svc->sdom->vcpu);   /* add to dom vcpu list */
}

/* lock is grabbed before calling this function */
/* vcpu should already be off runqueue */
static void
rtpartition_vcpu_remove(const struct scheduler *ops, struct vcpu *vc)
{
    struct rtpartition_vcpu * const svc = RTPARTITION_VCPU(vc);
    struct rtpartition_dom * const sdom = svc->sdom;

    printtime();
    rtpartition_dump_vcpu(svc);

    BUG_ON( sdom == NULL );
    BUG_ON( __vcpu_on_runq(svc) );

    if ( !is_idle_vcpu(vc) ) {
        list_del_init(&svc->sdom_elem);
    }
}

/*
 * Other important functions
 */
/* do we need the lock here? */
/* TODO: How to return the per VCPU parameters? Right now return the sum of budgets */
static int
rtpartition_dom_cntl(const struct scheduler *ops, struct domain *d, struct xen_domctl_scheduler_op *op)
{
    struct rtpartition_dom * const sdom = RTPARTITION_DOM(d);
    struct list_head *iter;
    struct rtpartition_private * prv = RTPARTITION_PRIV(ops);

    if ( op->cmd == XEN_DOMCTL_SCHEDOP_getinfo ) {
        /* for debug use, whenever adjust Dom0 parameter, do global dump */
        if ( d->domain_id == 0 ) {
            rtpartition_dump(ops);
        }

        /* TODO: how to return individual VCPU parameters? */
        op->u.rtpartition.budget = 0;
        op->u.rtpartition.extra = sdom->extra;
        list_for_each( iter, &sdom->vcpu ) {
            struct rtpartition_vcpu * svc = list_entry(iter, struct rtpartition_vcpu, sdom_elem);
            op->u.rtpartition.budget += svc->budget;
            op->u.rtpartition.period = svc->period;
        }
    } else {
        ASSERT(op->cmd == XEN_DOMCTL_SCHEDOP_putinfo);

        if ( d->domain_id == 0 ) {
            if ( prv->priority_scheme == EDF ) {
                prv->priority_scheme = RM;
                printk("priority changed to RM\n");
            } else {
                prv->priority_scheme = EDF;
                printk("priority changed to EDF\n");
            }
        }

        sdom->extra = op->u.rtpartition.extra;
        list_for_each( iter, &sdom->vcpu ) {
            struct rtpartition_vcpu * svc = list_entry(iter, struct rtpartition_vcpu, sdom_elem);
            if ( op->u.rtpartition.vcpu == svc->vcpu->vcpu_id ) { /* adjust per VCPU parameter */
                svc->period = op->u.rtpartition.period;
                svc->budget = op->u.rtpartition.budget;
                break;
            }
        }
    }

    return 0;
}

/* return a VCPU considering affinity */
static int
rtpartition_cpu_pick(const struct scheduler *ops, struct vcpu *vc)
{
    cpumask_t cpus;
    int cpu;
    struct rtpartition_private * prv = RTPARTITION_PRIV(ops);

    cpumask_copy(&cpus, vc->cpu_affinity);
    cpumask_and(&cpus, &prv->cpus, &cpus);

    cpu = cpumask_test_cpu(vc->processor, &cpus)
            ? vc->processor 
            : cpumask_cycle(vc->processor, &cpus);
    ASSERT( !cpumask_empty(&cpus) && cpumask_test_cpu(cpu, &cpus) );

// #ifdef RTXEN_DEBUG
//     if ( vc->domain->domain_id != 0 && rtxen_counter[RTXEN_PICK] < RTXEN_MAX ) {
//         printtime();
//         rtpartition_dump_vcpu(RTPARTITION_VCPU(vc));
//         rtxen_counter[RTXEN_PICK]++;
//     }
// #endif

    return cpu;
}

/* Implemented as deferrable server. */
/* burn budget at microsecond level */
static void
burn_budgets(const struct scheduler *ops, struct rtpartition_vcpu *svc, s_time_t now) {
    s_time_t delta;
    unsigned int consume;
    long count = 0;

    /* first time called for this svc, update last_start */
    if ( svc->last_start == 0 ) {
        svc->last_start = now;
        return;
    }

    /* don't burn budget for idle VCPU */
    if ( is_idle_vcpu(svc->vcpu) ) {
        return;
    }

    /* update deadline info */
    delta = now - svc->cur_deadline;
    if ( delta >= 0 ) {
        count = ( delta/MILLISECS(svc->period) ) + 1;
        svc->cur_deadline += count * MILLISECS(svc->period);
        svc->cur_budget = svc->budget * 1000;
        return;
    }

    delta = now - svc->last_start;
    if ( delta < 0 ) {
        printk("%s, delta = %ld for ", __func__, delta);
        rtpartition_dump_vcpu(svc);
        svc->last_start = now;  /* update last_start */
        svc->cur_budget = 0;
        return;
    }

    if ( svc->cur_budget == 0 ) return;

    /* burn at microseconds level */
    consume = ( delta/MICROSECS(1) );
    if ( delta%MICROSECS(1) > MICROSECS(1)/2 ) consume++;

    svc->cur_budget -= consume;
    if ( svc->cur_budget < 0 ) svc->cur_budget = 0;
}

/* RunQ is sorted. Pick first one budget. If no one, return NULL */
/* lock is grabbed before calling this function */
static struct rtpartition_vcpu *
__runq_pick(unsigned int cpu)
{
    struct list_head *runq = RUNQ(cpu);
    struct list_head *iter;
    struct rtpartition_vcpu *iter_svc = NULL;

    list_for_each(iter, runq) {
        iter_svc = __runq_elem(iter);   

        if ( iter_svc->cur_budget <= 0 && iter_svc->sdom->extra == 0 )
            continue;

        return iter_svc;
    }

    return NULL;
}

/* lock is grabbed before calling this function */
static void
__repl_update(const struct scheduler *ops, unsigned int cpu, s_time_t now)
{
    struct list_head *runq = RUNQ(cpu);
    struct list_head *iter;
    struct list_head *tmp;
    struct rtpartition_vcpu *svc = NULL;

    s_time_t diff;
    long count;

    list_for_each_safe(iter, tmp, runq) {
        svc = __runq_elem(iter);

        if ( now > svc->cur_deadline ) {
            diff = now - svc->cur_deadline;    
            count = (diff/MILLISECS(svc->period)) + 1;
            svc->cur_deadline += count * MILLISECS(svc->period);
            svc->cur_budget = svc->budget * 1000;
            __runq_remove(svc);
            __runq_insert(ops, cpu, svc);   // need ops to get the priority_scheme
        }
    }
}

/* The lock is already grabbed in schedule.c, no need to lock here */
static struct task_slice
rtpartition_schedule(const struct scheduler *ops, s_time_t now, bool_t tasklet_work_scheduled)
{
    const int cpu = smp_processor_id();
    struct rtpartition_private * prv = RTPARTITION_PRIV(ops);
    struct rtpartition_vcpu * const scurr = RTPARTITION_VCPU(current);
    struct rtpartition_vcpu * snext = NULL;
    struct task_slice ret;

    /* burn_budget would return for IDLE VCPU */
    burn_budgets(ops, scurr, now);

#ifdef RTXEN_DEBUG
    if ( !is_idle_vcpu(scurr->vcpu) && scurr->vcpu->domain->domain_id != 0 && rtxen_counter[RTXEN_SCHED] < RTXEN_MAX ) {
    // if ( rtxen_counter[RTXEN_SCHED] < RTXEN_MAX ) {
        printtime();
        printk("from: ");
        rtpartition_dump_vcpu(scurr);
        rtxen_counter[RTXEN_SCHED]++;
    }
#endif

    __repl_update(ops, cpu, now);

    if ( tasklet_work_scheduled ) {
        snext = RTPARTITION_VCPU(idle_vcpu[cpu]);
    } else {
        snext = __runq_pick(cpu);
        if ( snext == NULL )
            snext = RTPARTITION_VCPU(idle_vcpu[cpu]);

        /* if scurr has higher priority and budget, still pick scurr */
        if ( !is_idle_vcpu(current) &&
             vcpu_runnable(current) &&
             scurr->cur_budget > 0 &&
             ( is_idle_vcpu(snext->vcpu) ||
               ( prv->priority_scheme == EDF && scurr->cur_deadline <= snext->cur_deadline ) ||
               ( prv->priority_scheme == RM && scurr->period <= snext->period)) ) {
               // scurr->vcpu->domain->domain_id == snext->vcpu->domain->domain_id ) ) {
            snext = scurr;
        }
    }

    /* Trace switch self problem */
    // if ( snext != scurr &&
    //      !is_idle_vcpu(snext->vcpu) &&
    //      !is_idle_vcpu(scurr->vcpu) &&
    //      snext->vcpu->domain->domain_id == scurr->vcpu->domain->domain_id &&
    //      scurr->cur_budget > 0 &&
    //      vcpu_runnable(current) &&
    //      snext->cur_deadline < scurr->cur_deadline ) {
    //     TRACE_3D(TRC_SCHED_RTPARTITION_SWITCHSELF, scurr->vcpu->domain->domain_id, scurr->vcpu->vcpu_id, snext->vcpu->vcpu_id);
    // }

    if ( snext != scurr &&
         !is_idle_vcpu(current) &&
         vcpu_runnable(current) ) {
        __runq_insert(ops, cpu, scurr); // insert scurr back to runq
    }

    snext->last_start = now;
    ret.migrated = 0;
    if ( !is_idle_vcpu(snext->vcpu) ) {
        if ( snext != scurr ) {
            __runq_remove(snext);
        }
        if ( snext->vcpu->processor != cpu ) {
            snext->vcpu->processor = cpu;
            ret.migrated = 1;
        }
    }

    if ( is_idle_vcpu(snext->vcpu) || snext->cur_budget > MILLISECS(1)) {
        ret.time = MILLISECS(1);
    } else if ( snext->cur_budget > 0 ) {
        ret.time = MICROSECS(snext->budget);
    } else {
        ret.time = MICROSECS(500);  /* only let extra run for 500 us */
    }

    ret.task = snext->vcpu;

#ifdef RTXEN_DEBUG
    if ( !is_idle_vcpu(snext->vcpu) && snext->vcpu->domain->domain_id != 0 && rtxen_counter[RTXEN_SCHED] < RTXEN_MAX ) {
    // if ( rtxen_counter[RTXEN_SCHED] < RTXEN_MAX ) {
        printtime();
        printk(" to : ");
        rtpartition_dump_vcpu(snext);
    }
#endif

    return ret;
}

/* Remove VCPU from RunQ */
/* The lock is already grabbed in schedule.c, no need to lock here */
static void
rtpartition_vcpu_sleep(const struct scheduler *ops, struct vcpu *vc)
{
    struct rtpartition_vcpu * const svc = RTPARTITION_VCPU(vc);

#ifdef RTXEN_DEBUG
    if ( vc->domain->domain_id != 0 && rtxen_counter[RTXEN_SLEEP] < RTXEN_MAX ) {
        printtime();
        rtpartition_dump_vcpu(svc);
        rtxen_counter[RTXEN_SLEEP]++;
    }
#endif

    BUG_ON( is_idle_vcpu(vc) );

    if ( curr_on_cpu(vc->processor) == vc ) {
        cpu_raise_softirq(vc->processor, SCHEDULE_SOFTIRQ);
    } else if ( __vcpu_on_runq(svc) ) {
        __runq_remove(svc);
    }
}

/* Should always wake up runnable, put it back to RunQ. Check priority to raise interrupt */
/* The lock is already grabbed in schedule.c, no need to lock here */
static void
rtpartition_vcpu_wake(const struct scheduler *ops, struct vcpu *vc)
{
    struct rtpartition_vcpu * const svc = RTPARTITION_VCPU(vc);
    const unsigned int cpu = vc->processor;
    struct rtpartition_private * prv = RTPARTITION_PRIV(ops);
    struct rtpartition_vcpu * snext = NULL;
    struct rtpartition_vcpu * scurr = RTPARTITION_VCPU(curr_on_cpu(cpu));
    s_time_t now = NOW();
    

#ifdef RTXEN_DEBUG
    if ( vc->domain->domain_id != 0 && rtxen_counter[RTXEN_WAKE] < RTXEN_MAX ) {
        printtime();
        rtpartition_dump_vcpu(svc);
        rtxen_counter[RTXEN_WAKE]++;
    }
#endif

    BUG_ON( is_idle_vcpu(vc) );

    if ( unlikely(curr_on_cpu(cpu) == vc) ||
         unlikely(__vcpu_on_runq(svc)) )
        return;

    __runq_insert(ops, cpu, svc);
    __repl_update(ops, cpu, now);
    snext = __runq_pick(cpu);   /* highest priority VCPU on RunQ */
    if ( (prv->priority_scheme == EDF && snext->cur_deadline < scurr->cur_deadline) ||
         (prv->priority_scheme == RM && snext->period < scurr->period) ) {
        cpu_raise_softirq(cpu, SCHEDULE_SOFTIRQ);
    }

    return;
}

static struct rtpartition_private _rtpartition_priv;

const struct scheduler sched_rtpartition_def = {
    .name           = "SMP RTPARTITION Scheduler",
    .opt_name       = "rtpartition",
    .sched_id       = XEN_SCHEDULER_RTPARTITION,
    .sched_data     = &_rtpartition_priv,

    .dump_cpu_state = rtpartition_dump_pcpu,
    .dump_settings  = rtpartition_dump,
    .init           = rtpartition_init,
    .deinit         = rtpartition_deinit,
    .alloc_pdata    = rtpartition_alloc_pdata,
    .free_pdata     = rtpartition_free_pdata,
    .alloc_domdata  = rtpartition_alloc_domdata,
    .free_domdata   = rtpartition_free_domdata,
    .init_domain    = rtpartition_dom_init,
    .destroy_domain = rtpartition_dom_destroy,
    .alloc_vdata    = rtpartition_alloc_vdata,
    .free_vdata     = rtpartition_free_vdata,
    .insert_vcpu    = rtpartition_vcpu_insert,
    .remove_vcpu    = rtpartition_vcpu_remove,

    .adjust         = rtpartition_dom_cntl,

    .pick_cpu       = rtpartition_cpu_pick,
    .do_schedule    = rtpartition_schedule,
    .sleep          = rtpartition_vcpu_sleep,
    .wake           = rtpartition_vcpu_wake,
    
    .context_saved  = NULL,
    .yield          = NULL,
    .migrate        = NULL,
};