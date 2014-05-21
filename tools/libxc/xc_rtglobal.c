/****************************************************************************
 * (C) 2006 - Emmanuel Ackaouy - XenSource Inc.
 ****************************************************************************
 *
 *        File: xc_rtglobal.c
 *      Author: Sisu Xi
 *
 * Description: XC Interface to the rtglobal scheduler
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

int
xc_sched_rtglobal_domain_set(
    xc_interface *xch,
    uint32_t domid,
    struct xen_domctl_sched_rtglobal_params *sdom)
{
    int rc;
    DECLARE_DOMCTL;
    DECLARE_HYPERCALL_BOUNCE(sdom, 
        sizeof(*sdom), 
        XC_HYPERCALL_BUFFER_BOUNCE_IN);

    if ( xc_hypercall_bounce_pre(xch, sdom) )
        return -1;

    domctl.cmd = XEN_DOMCTL_scheduler_op;
    domctl.domain = (domid_t) domid;
    domctl.u.scheduler_op.sched_id = XEN_SCHEDULER_RTGLOBAL;
    domctl.u.scheduler_op.cmd = XEN_DOMCTL_SCHEDOP_putinfo;
    set_xen_guest_handle(domctl.u.scheduler_op.u.rtglobal.schedule, sdom);

    rc = do_domctl(xch, &domctl);

    xc_hypercall_bounce_post(xch, sdom);

    return rc;
}

int
xc_sched_rtglobal_domain_get(
    xc_interface *xch,
    uint32_t domid,
    struct xen_domctl_sched_rtglobal_params *sdom)
{
    int rc;
    DECLARE_DOMCTL;
    DECLARE_HYPERCALL_BOUNCE(sdom, 
        sizeof(*sdom), 
        XC_HYPERCALL_BUFFER_BOUNCE_OUT);

    if ( xc_hypercall_bounce_pre(xch, sdom) )
        return -1;

    domctl.cmd = XEN_DOMCTL_scheduler_op;
    domctl.domain = (domid_t) domid;
    domctl.u.scheduler_op.sched_id = XEN_SCHEDULER_RTGLOBAL;
    domctl.u.scheduler_op.cmd = XEN_DOMCTL_SCHEDOP_getinfo;
    set_xen_guest_handle(domctl.u.scheduler_op.u.rtglobal.schedule, sdom);

    rc = do_domctl(xch, &domctl);

    xc_hypercall_bounce_post(xch, sdom);

    return rc;
}

int
xc_sched_rtglobal_params_set(
    xc_interface *xch,
    struct xen_sysctl_rtglobal_schedule *schedule)
{
    int rc;
    DECLARE_SYSCTL; 
    
    sysctl.cmd = XEN_SYSCTL_scheduler_op;
    sysctl.u.scheduler_op.sched_id = XEN_SCHEDULER_RTGLOBAL;
    sysctl.u.scheduler_op.cmd = XEN_SYSCTL_SCHEDOP_putinfo;

    sysctl.u.scheduler_op.u.sched_rtglobal = *schedule;
    
    rc = do_sysctl(xch, &sysctl);

    *schedule = sysctl.u.scheduler_op.u.sched_rtglobal;
    
    return rc;
}

int
xc_sched_rtglobal_params_get(
    xc_interface *xch,
    struct xen_sysctl_rtglobal_schedule *schedule)
{
    int rc;
    DECLARE_SYSCTL;

    sysctl.cmd = XEN_SYSCTL_scheduler_op;
    sysctl.u.scheduler_op.sched_id = XEN_SCHEDULER_RTGLOBAL;
    sysctl.u.scheduler_op.cmd = XEN_SYSCTL_SCHEDOP_getinfo;

    rc = do_sysctl(xch, &sysctl);
    if ( rc == 0 )
        *schedule = sysctl.u.scheduler_op.u.sched_rtglobal;

    return rc;
}
