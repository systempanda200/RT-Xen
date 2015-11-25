/****************************************************************************
 *
 *        File: xc_rt.c
 *      Author: Sisu Xi
 *              Meng Xu
 *
 * Description: XC Interface to the rtds scheduler
 * Note: VCPU's parameter (period, budget) is in microsecond (us).
 *       All VCPUs of the same domain have same period and budget.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation;
 * version 2.1 of the License.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; If not, see <http://www.gnu.org/licenses/>.
 */

#include "xc_private.h"

int xc_sched_rtds_domain_set(xc_interface *xch,
                           uint32_t domid,
                           xen_domctl_sched_rtds_t *sdom)
{
    int rc;
    DECLARE_DOMCTL;

    domctl.cmd = XEN_DOMCTL_scheduler_op;
    domctl.domain = (domid_t) domid;
    domctl.u.scheduler_op.sched_id = XEN_SCHEDULER_RTDS;
    domctl.u.scheduler_op.cmd = XEN_DOMCTL_SCHEDOP_putinfo;
    domctl.u.scheduler_op.u.rtds.period = sdom->period;
    domctl.u.scheduler_op.u.rtds.budget = sdom->budget;

    rc = do_domctl(xch, &domctl);

    return rc;
}

int xc_sched_rtds_domain_get(xc_interface *xch,
                           uint32_t domid,
                           xen_domctl_sched_rtds_t *sdom)
{
    int rc;
    DECLARE_DOMCTL;

    domctl.cmd = XEN_DOMCTL_scheduler_op;
    domctl.domain = (domid_t) domid;
    domctl.u.scheduler_op.sched_id = XEN_SCHEDULER_RTDS;
    domctl.u.scheduler_op.cmd = XEN_DOMCTL_SCHEDOP_getinfo;

    rc = do_domctl(xch, &domctl);

    if ( rc == 0 )
        *sdom = domctl.u.scheduler_op.u.rtds;

    return rc;
}
int
xc_sched_rtds_params_set(
    xc_interface *xch,
    uint32_t cpupool_id,
    struct xen_sysctl_rtds_schedule *schedule)
{
    int rc;
    DECLARE_SYSCTL;

    sysctl.cmd = XEN_SYSCTL_scheduler_op;
    sysctl.u.scheduler_op.cpupool_id = cpupool_id;
    sysctl.u.scheduler_op.sched_id = XEN_SCHEDULER_RTDS;
    sysctl.u.scheduler_op.cmd = XEN_SYSCTL_SCHEDOP_putinfo;

    sysctl.u.scheduler_op.u.sched_rtds = *schedule;

    rc = do_sysctl(xch, &sysctl);

    *schedule = sysctl.u.scheduler_op.u.sched_rtds;

    return rc;
}

int
xc_sched_rtds_params_get(
    xc_interface *xch,
    uint32_t cpupool_id,
    struct xen_sysctl_rtds_schedule *schedule)
{
    int rc;
    DECLARE_SYSCTL;

    sysctl.cmd = XEN_SYSCTL_scheduler_op;
    sysctl.u.scheduler_op.cpupool_id = cpupool_id;
    sysctl.u.scheduler_op.sched_id = XEN_SCHEDULER_RTDS;
    sysctl.u.scheduler_op.cmd = XEN_SYSCTL_SCHEDOP_getinfo;

    rc = do_sysctl(xch, &sysctl);

    *schedule = sysctl.u.scheduler_op.u.sched_rtds;

    return rc;
}

int xc_sched_rtds_vcpu_set(
    xc_interface *xch,
    uint32_t domid,
    xen_domctl_schedparam_vcpu_t *vcpus,
    uint16_t num_vcpus)
{
    int rc;
    DECLARE_DOMCTL;
    DECLARE_HYPERCALL_BOUNCE(vcpus, sizeof(*vcpus) * num_vcpus,
            XC_HYPERCALL_BUFFER_BOUNCE_IN);

    if ( xc_hypercall_bounce_pre(xch, vcpus) )
        return -1;

    domctl.cmd = XEN_DOMCTL_scheduler_op;
    domctl.domain = (domid_t) domid;
    domctl.u.scheduler_op.sched_id = XEN_SCHEDULER_RTDS;
    domctl.u.scheduler_op.cmd = XEN_DOMCTL_SCHEDOP_putvcpuinfo;
    domctl.u.scheduler_op.u.v.nr_vcpus = num_vcpus;
    set_xen_guest_handle(domctl.u.scheduler_op.u.v.vcpus, vcpus);

    rc = do_domctl(xch, &domctl);

    xc_hypercall_bounce_post(xch, vcpus);

    return rc;
}

int xc_sched_rtds_vcpu_get(
    xc_interface *xch,
    uint32_t domid,
    xen_domctl_schedparam_vcpu_t *vcpus,
    uint16_t num_vcpus)
{
    int rc;
    DECLARE_DOMCTL;
    DECLARE_HYPERCALL_BOUNCE(vcpus, sizeof(*vcpus) * num_vcpus,
            XC_HYPERCALL_BUFFER_BOUNCE_BOTH);

    if ( xc_hypercall_bounce_pre(xch, vcpus) )
        return -1;

    domctl.cmd = XEN_DOMCTL_scheduler_op;
    domctl.domain = (domid_t) domid;
    domctl.u.scheduler_op.sched_id = XEN_SCHEDULER_RTDS;
    domctl.u.scheduler_op.cmd = XEN_DOMCTL_SCHEDOP_getvcpuinfo;
    domctl.u.scheduler_op.u.v.nr_vcpus = num_vcpus;
    set_xen_guest_handle(domctl.u.scheduler_op.u.v.vcpus, vcpus);

    rc = do_domctl(xch, &domctl);

    xc_hypercall_bounce_post(xch, vcpus);

    return rc;
}
