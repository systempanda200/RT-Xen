/****************************************************************************
 *
 *        File: xc_rt.c
 *      Author: Sisu Xi 
 *              Meng Xu
 *
 * Description: XC Interface to the rt scheduler
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
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include "xc_private.h"

int xc_sched_rt_domain_set(xc_interface *xch,
                           uint32_t domid,
                           struct xen_domctl_sched_rt_params *sdom)
{
    int rc;
    DECLARE_DOMCTL;

    domctl.cmd = XEN_DOMCTL_scheduler_op;
    domctl.domain = (domid_t) domid;
    domctl.u.scheduler_op.sched_id = XEN_SCHEDULER_RT_DS;
    domctl.u.scheduler_op.cmd = XEN_DOMCTL_SCHEDOP_putinfo;
    domctl.u.scheduler_op.u.rt.vcpu_index = sdom->index;
    domctl.u.scheduler_op.u.rt.period = sdom->period;
    domctl.u.scheduler_op.u.rt.budget = sdom->budget;

    rc = do_domctl(xch, &domctl);

    return rc;
}

int xc_sched_rt_domain_get(xc_interface *xch,
                           uint32_t domid,
                           struct xen_domctl_sched_rt_params *sdom,
                           uint16_t num_vcpus)
{
    int rc;
    DECLARE_DOMCTL;
    DECLARE_HYPERCALL_BOUNCE(sdom, 
        sizeof(*sdom) * num_vcpus, 
        XC_HYPERCALL_BUFFER_BOUNCE_OUT);

    if ( xc_hypercall_bounce_pre(xch, sdom) )
        return -1;

    domctl.cmd = XEN_DOMCTL_scheduler_op;
    domctl.domain = (domid_t) domid;
    domctl.u.scheduler_op.sched_id = XEN_SCHEDULER_RT_DS;
    domctl.u.scheduler_op.cmd = XEN_DOMCTL_SCHEDOP_getinfo;
    set_xen_guest_handle(domctl.u.scheduler_op.u.rt.vcpu, sdom);

    rc = do_domctl(xch, &domctl);

    xc_hypercall_bounce_post(xch, sdom);

    return rc;
}

int xc_sched_rt_domain_get_num_vcpus(xc_interface *xch,
                                     uint32_t domid,
                                     uint16_t *num_vcpus)
{
    int rc;
    DECLARE_DOMCTL;

    domctl.cmd = XEN_DOMCTL_scheduler_op;
    domctl.domain = (domid_t) domid;
    domctl.u.scheduler_op.sched_id = XEN_SCHEDULER_RT_DS;
    domctl.u.scheduler_op.cmd = XEN_DOMCTL_SCHEDOP_getnumvcpus;

    rc = do_domctl(xch, &domctl);

    *num_vcpus = domctl.u.scheduler_op.u.rt.nr_vcpus;
    return rc;
}
