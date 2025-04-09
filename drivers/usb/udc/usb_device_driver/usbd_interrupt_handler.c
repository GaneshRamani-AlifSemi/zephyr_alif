/* Copyright (C) 2022 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

/******************************************************************************
 * @file     usbd_interrupt_handler.c
 * @author   Mahesh Avula
 * @email    mahesh.avula@alifsemi.com
 * @version  V1.0.0
 * @date     10-Feb-2022
 * @brief    This function is the interrupt handler for dwc3 controller.
 *           The controller will trigger an interrupt when something happens on
 *           an endpoint whose mask has been set in the interrupt enable
 *           register, or when a bus reset is detected.   .
 * @bug      None.
 * @Note     None.
 ******************************************************************************/

/* Include necessary system files.  */

#include "usbd_dwc3.h"
//#include "RTE_Device.h"
//#include "RTE_Components.h"
//#include CMSIS_device_header
//extern USB_DRIVER Drv;
uint32_t status_sent = 0;
uint32_t status_done = 0;
uint32_t status_done_two_stage = 0;
uint32_t setup_done = 0;
uint32_t data_done = 0;
uint8_t xfer_cmplt = 0;
uint8_t bulk_out_cb = 0;
uint8_t prepare_Setup_cnt = 0;
uint8_t reset_cnt = 0;
uint8_t connection_done_cnt = 0;


static void usbd_event_buffer_handler(USB_DRIVER *drv, USBD_EVENT_BUFFER *event_buffer)
{
    uint32_t reg;
    //RTSS_InvalidateDCache_by_Addr(event_buffer -> buf, USB_EVENT_BUFFER_SIZE);
    SCB_InvalidateDCache_by_Addr(event_buffer -> buf, USB_EVENT_BUFFER_SIZE);


    while(event_buffer->count > 0)
    {
        reg = ((uint32_t)(*((uint32_t *) (event_buffer -> buf + event_buffer -> lpos))));
        /* Check type of event */
        if((reg & 0x1) == 0x1)
        {
            /* process device specific events  */
            usbd_devt_handler(drv, reg);
        }
        else if((reg & 0x1) == 0)
        {
            /* process  endpoint specific events  */
            usbd_depevt_handler(drv, reg);
        }
        else
        {
#ifdef DEBUG
            printf("Unknown events\n");
#endif
        }
        event_buffer -> lpos = (event_buffer -> lpos + 4) % USB_EVENT_BUFFER_SIZE;
        event_buffer -> count -= 4;
        drv->regs->GEVNTCOUNT0 = 4;
    }
    /* Unmask interrupt */
    event_buffer -> count = 0;
}

void  usbd_interrupt_handler(USB_DRIVER  *drv)
{
    uint32_t pending_interrupt;
    uint32_t mask_interrupt;
    //USB_DRIVER  *drv;
    USBD_EVENT_BUFFER  *event_buf;
    //drv = &Drv;
    /* Get event pointer ...*/
    event_buf = drv -> event_buf;
    pending_interrupt = drv->regs->GEVNTCOUNT0;
    pending_interrupt &= USB_GEVNTCOUNT_MASK;
    if(!pending_interrupt)
    {
#ifdef DEBUG
        printf("no pending irq\n");
#endif
        return;
    }
    event_buf -> count = pending_interrupt;
    /* Set the Event Interrupt Mask */
    mask_interrupt = drv->regs->GEVNTSIZ0;
    mask_interrupt |= USB_GEVNTSIZ_INTMASK;
    drv->regs->GEVNTSIZ0 = mask_interrupt;

    /* Processes events in an Event Buffer */
    usbd_event_buffer_handler(drv, event_buf);
    /* Clear the Event Interrupt Mask */
    mask_interrupt = drv->regs->GEVNTSIZ0;
    mask_interrupt &= ~USB_GEVNTSIZ_INTMASK;
    drv->regs->GEVNTSIZ0 = mask_interrupt;
}

void usbd_prepare_Setup(USB_DRIVER *drv)
{
    USBD_EP_PARAMS Params;
    USBD_TRB      *TrbPtr;
    USBD_EP       *Ept;
    uint32_t Ret;
    Params.Param0 = 0;
    Params.Param1 = 0;
    Params.Param2 = 0;
    /* Setup packet always on EP0 */
    Ept = &drv->eps[0U];
    TrbPtr = &drv->endp0_trb;
    //memset(TrbPtr, 0x00, 16);
    //RTSS_CleanDCache_by_Addr(&drv->SetupData, 8);
    SCB_CleanDCache_by_Addr(&drv->SetupData, 8);
    TrbPtr->BufferPtrLow =  lower_32_bits(local_to_global(&drv->SetupData));
    TrbPtr->BufferPtrHigh = 0;
    TrbPtr->Size = 8;
    TrbPtr->Ctrl = USB_TRBCTL_CONTROL_SETUP;
    TrbPtr->Ctrl |= (USB_TRB_CTRL_HWO
            | USB_TRB_CTRL_LST
            | USB_TRB_CTRL_IOC
            | USB_TRB_CTRL_ISP_IMI);
   // RTSS_CleanDCache_by_Addr(TrbPtr, sizeof(*TrbPtr));
    SCB_CleanDCache_by_Addr(TrbPtr, sizeof(*TrbPtr));
    Params.Param1 = (uint32_t)TrbPtr;
    drv->ep0state = EP0_SETUP_PHASE;
    /*if(prepare_Setup_cnt == 2)
    {
    	printk("break\n");
    }*/
    prepare_Setup_cnt ++;
    Ret = usbd_SendEpCmd(drv, 0U, USB_DEPCMD_STARTTRANSFER, Params);
    if (Ret)
    {
#ifdef DEBUG
        printf("Failed in send the command for setup pkt and status: %d\n",Ret);
#endif
    }
    Ept->ep_status |= USB_EP_BUSY;
    Ept->ep_resource_index = usbd_EpGetTransferIndex(drv,
            Ept->ep_index,
            Ept->ep_dir);
}

uint32_t usbd_EpGetTransferIndex(USB_DRIVER *drv, uint8_t ep_num, uint8_t Dir)
{
    uint8_t PhyEpNum;
    uint32_t ResourceIndex;
    PhyEpNum = USB_PhysicalEp(ep_num, Dir);
    /* [22:16]: Transfer Resource Index (XferRscIdx). The hardware-assigned
     * transfer resource index for the transfer */
    ResourceIndex = drv->regs->USB_ENDPNT_CMD[PhyEpNum].DEPCMD;
    return USB_DEPCMD_GET_RSC_IDX(ResourceIndex);
}

int32_t usbd_transfer_wakeup(USB_DRIVER * drv)
{
    int32_t    retries = USB_TRANSFER_WAKEUP_RETRY;
    int32_t   ret;
    uint32_t   reg;
    uint32_t   link_state;
    /*
     * According to the Databook Remote wakeup request should
     * be issued only when the device is in early suspend state.
     *
     * We can check that via USB Link State bits in DSTS register.
     */
    reg = drv->regs->DSTS;
    link_state = USB_DSTS_DWC3_USBLNKST(reg);

    switch (link_state) {
        case USB_LINK_STATE_EARLY_SUS:    /* in HS, means Early Suspend */
        case USB_LINK_STATE_L2:        /* in HS, means SUSPEND */
        case USB_LINK_STATE_L1:        /* in HS, means SLEEP */
            break;
        default:
            return -1;
    }

    ret = usbd_set_link_state(drv, USB_LINK_STATE_RECOV);
    if (ret < 0)
    {
#ifdef DEBUG
        printf("failed to put link in Recovery\n");
#endif
        return -1;
    }

    /* poll until Link State changes to ON */
    while (retries--)
    {
        reg = drv->regs->DSTS;
        /* in HS, means ON */
        if (USB_DSTS_DWC3_USBLNKST(reg) == USB_LINK_STATE_ON)
            break;
    }
    if (USB_DSTS_DWC3_USBLNKST(reg) != USB_LINK_STATE_ON)
    {
#ifdef DEBUG
        printf("failed to send remote wakeup: Controller state not changed\n");
#endif
        return -1;
    }
    return 0;
}

int32_t usbd_set_link_state(USB_DRIVER *drv, USBD_LINK_STATE state)
{
    uint32_t reg;
    int32_t retries = USB_LINK_STATE_RETRY;
    while (--retries)
    {
        reg = drv->regs->DSTS;
        if (reg & USB_DSTS_DWC3_DCNRD)
        	k_busy_wait(5);
        else
            break;
    }
    if(retries <= 0)
    {
#ifdef DEBUG
        printf("expired retries\n");
#endif
        return -1;
    }
    reg = drv->regs->DCTL;
    reg &= ~USB_DCTL_ULSTCHNGREQ_MASK;

    /* set requested state */
    reg |= USB_DCTL_ULSTCHNGREQ(state);
    drv->regs->DCTL = reg;
    /* wait for a change in DSTS */
    retries = 10000;
    while (--retries)
    {
        reg = drv->regs->DSTS;
        if (USB_DSTS_DWC3_USBLNKST(reg) == state)
        {
            return 0;
        }
        k_busy_wait(5);
    }
    return -1;
}
void usbd_ClearStallAllEp(USB_DRIVER *drv)
{
    uint8_t ep_num;
    for (ep_num = 1U; ep_num < USB_ENDPOINTS_NUM; ep_num++)
    {
        USBD_EP *Ept;
        Ept = &drv->eps[ep_num];
        if ((Ept->ep_status & USB_EP_ENABLED) == 0U)
            continue;

        if ((Ept->ep_status & USB_EP_STALL) == 0U)
            continue;

        usbd_EpClearStall(drv, Ept->ep_index,
                Ept->ep_dir);
    }
}

int32_t usbd_StopTransfer(USB_DRIVER *drv, uint8_t ep_num,
        uint8_t Dir, uint32_t ForceRM)
{
    USBD_EP *Ept;
    USBD_EP_PARAMS Params;
    uint8_t PhyEpNum;
    uint32_t Cmd;
    uint32_t Ret;

    PhyEpNum = USB_PhysicalEp(ep_num, Dir);
    Params.Param2 = 0;
    Params.Param1 = 0;
    Params.Param0 = 0;
    Ept = &drv->eps[PhyEpNum];
    if (Ept->ep_resource_index == 0U)
        return 1;

    /* Data book says for end transfer  HW needs some
     * extra time to synchronize with the interconnect
     * - Issue EndTransfer WITH CMDIOC bit set
     * - Wait 100us
     */
    Cmd = USB_DEPCMD_ENDTRANSFER;
    Cmd |= (ForceRM == 1) ? USB_DEPCMD_HIPRI_FORCERM : 0U;
    Cmd |= USB_DEPCMD_CMDIOC;
    Cmd |= USB_DEPCMD_PARAM(Ept->ep_resource_index);

    Ret = usbd_SendEpCmd(drv, PhyEpNum, Cmd, Params);
    if(Ret < 0)
    {
#ifdef DEBUG
        printf("Failed to send command at END transfer\n");
#endif
        return 1;
    }
    if (ForceRM == 1)
    {
        Ept->ep_resource_index = 0U;
    }
    Ept->ep_status &= ~USB_EP_BUSY;
    k_busy_wait(100);
    return 0;
}

void usbd_EpClearStall(USB_DRIVER *drv, uint8_t ep_num, uint8_t Dir)
{
    uint8_t  PhyEpNum;
    USBD_EP *Ept;
    USBD_EP_PARAMS Params;
    uint32_t Ret;

    PhyEpNum = USB_PhysicalEp(ep_num, Dir);
    Ept = &drv->eps[PhyEpNum];
    Params.Param2 = 0;
    Params.Param1 = 0;
    Params.Param0 = 0;
    Ret = usbd_SendEpCmd(drv, PhyEpNum, USB_DEPCMD_CLEARSTALL, Params);
    if(Ret < 0)
    {
#ifdef DEBUG
        printf("Failed to send command at STALL Ep\n");
#endif
        return;
    }
    Ept->ep_status &= ~USB_EP_STALL;
}

void usbd_ep0_end_control_data(USB_DRIVER *drv, USBD_EP *Ept)
{
    USBD_EP_PARAMS Params;
    uint32_t     Cmd;
    uint32_t Ret;
    uint8_t PhyEpNum;
    if (Ept->ep_resource_index == 0U)
        return;
    PhyEpNum = USB_PhysicalEp(Ept->ep_index, Ept->ep_dir);
    Params.Param2 = 0;
    Params.Param1 = 0;
    Params.Param0 = 0;
    Cmd = USB_DEPCMD_ENDTRANSFER;
    Cmd |= USB_DEPCMD_CMDIOC;
    Cmd |= USB_DEPCMD_PARAM(Ept->ep_resource_index);
    Ret = usbd_SendEpCmd(drv, PhyEpNum, Cmd, Params);
    if(Ret < 0)
    {
#ifdef DEBUG
        printf("Failed to send command at Endcontrol data\n");
#endif
        return;
    }
    Ept->ep_resource_index = 0U;
}

void usbd_ep0_stall_restart(USB_DRIVER *drv)
{
    USBD_EP *Ept;
    /* reinitialize physical ep1 */
    Ept = &drv->eps[1U];
    Ept->ep_status = USB_EP_ENABLED;

    /* stall is always issued on EP0 */
    usbd_ep0_set_stall(drv, 0U, USB_DIR_OUT);
    Ept = &drv->eps[0U];
    Ept->ep_status = USB_EP_ENABLED;
    drv->ep0state = EP0_SETUP_PHASE;
    usbd_prepare_Setup(drv);
}

void  usbd_ep0_set_stall(USB_DRIVER *drv, uint8_t ep_num, uint8_t Dir)
{
    uint8_t  PhyEpNum;
    USBD_EP *Ept = NULL;
    USBD_EP_PARAMS Params;
    uint32_t Ret;

    PhyEpNum = USB_PhysicalEp(ep_num, Dir);
    Ept = &drv->eps[PhyEpNum];

    Params.Param2 = 0;
    Params.Param1 = 0;
    Params.Param0 = 0;
    Ret = usbd_SendEpCmd(drv, PhyEpNum, USB_DEPCMD_SETSTALL, Params);
    if(Ret < 0)
    {
#ifdef DEBUG
        printf("failed to send STALL command\n");
#endif
        return;
    }
    Ept->ep_status |= USB_EP_STALL;
}

void usbd_ep0_data_done(USB_DRIVER *drv, uint8_t endp_number)
{
    USBD_EP       *Ept;
    USBD_TRB      *TrbPtr;
    uint32_t     Status;
    uint8_t     Length;
    Ept = &drv->eps[endp_number];
    TrbPtr = &drv->endp0_trb;
    drv->actual_length = 0;
   // RTSS_InvalidateDCache_by_Addr(TrbPtr, sizeof(*TrbPtr));
    SCB_InvalidateDCache_by_Addr(TrbPtr, sizeof(*TrbPtr));

    Status = USB_TRB_SIZE_TRBSTS(TrbPtr->Size);

    if (Status == USB_TRBSTS_SETUP_PENDING)
    {
        drv->setup_packet_pending = 1;
#ifdef DEBUG
        printf("TRB transmission pending in control DATA_PHASE\n");
#endif
        return;
    }
    Length = TrbPtr->Size & USB_TRB_SIZE_MASK;
    Ept->BytesTxed = Ept->ep_requested_bytes - Length;
    drv->actual_length = Ept->BytesTxed;
    if(endp_number != 0)
    {
        if(drv->usbd_data_in_cb != NULL)
            drv->usbd_data_in_cb(drv, 0 | 0x80);
    }
    else
    {
    	 if(drv->usbd_data_out_cb != NULL)
    	            drv->usbd_data_out_cb(drv, 0);
    }
}

int32_t usbd_ep0_start_status(USB_DRIVER *drv, uint8_t endp_number)
{
    USBD_EP       *Ept;
    USBD_TRB      *TrbPtr;
    USBD_EP_PARAMS Params;
    uint32_t Type;
    uint32_t Ret;
    uint8_t Dir;
    Ept = &drv->eps[endp_number];
    Params.Param2 = 0;
    Params.Param1 = 0;
    Params.Param0 = 0;

    if ((Ept->ep_status & USB_EP_BUSY) != 0U)
    {
#ifdef DEBUG
        printf("Ep0StartStatus not done\n");
#endif
        return -1;
    }

    Type = (drv->three_stage_setup != 0U) ?
        USB_TRBCTL_CONTROL_STATUS3
        : USB_TRBCTL_CONTROL_STATUS2;
    TrbPtr = &drv->endp0_trb;
    //RTSS_CleanDCache_by_Addr(&drv->SetupData, 8);
    SCB_CleanDCache_by_Addr(&drv->SetupData, 8);
    /* we use same TrbPtr for setup packet */
    TrbPtr->BufferPtrLow = lower_32_bits(local_to_global(&drv->SetupData));
    TrbPtr->BufferPtrHigh = 0;
    TrbPtr->Size = 0U;
    TrbPtr->Ctrl = Type;
    TrbPtr->Ctrl |= (USB_TRB_CTRL_HWO
            | USB_TRB_CTRL_LST
            | USB_TRB_CTRL_IOC
            | USB_TRB_CTRL_ISP_IMI);


   // RTSS_CleanDCache_by_Addr(TrbPtr, sizeof(*TrbPtr));
    SCB_CleanDCache_by_Addr(TrbPtr, sizeof(*TrbPtr));
    Params.Param0 = 0U;
    Params.Param1 = (uint32_t)TrbPtr;
    drv->ep0state = EP0_STATUS_PHASE; //added
    /*
     * Control OUT transfer - Status stage happens on EP0 IN - EP1
     * Control IN transfer - Status stage happens on EP0 OUT - EP0
     */
    Dir = !drv->ep0_expect_in;
    Ret = usbd_SendEpCmd(drv, 0U|Dir, USB_DEPCMD_STARTTRANSFER, Params);

    if (Ret < 0)
    {
#ifdef DEBUG
        printf("failed to execute the command at control status\n");
#endif
        return Ret;
    }

    Ept->ep_status |= USB_EP_BUSY;
    Ept->ep_resource_index = usbd_EpGetTransferIndex(drv,
            Ept->ep_index,
            Ept->ep_dir);

    return Ret;
}

void usbd_ep0_status_done(USB_DRIVER *drv)
{
    USBD_TRB *TrbPtr;
    uint32_t Status;

    TrbPtr = &drv->endp0_trb;
    Status = USB_TRB_SIZE_TRBSTS(TrbPtr->Size);
    if (Status == USB_TRBSTS_SETUP_PENDING)
    {
        drv->setup_packet_pending = 1;
#ifdef DEBUG
        printf("status pending at status done\n");
#endif
    }
    drv->actual_length = 0;
    drv->ep0state = EP0_SETUP_PHASE;
    if(drv->endp_number != 0)
    {
        status_done_two_stage ++;
       // if(drv->cb_endpoint_event != NULL)
       //   drv->cb_endpoint_event(0 | ARM_USB_ENDPOINT_DIRECTION_MASK, ARM_USBD_EVENT_IN);
        //if(drv->usbd_status_cb != NULL)
        //             drv->usbd_status_cb(drv, 0);
    }
    else
    {

      // if(drv->usbd_status_cb != NULL)
        //     drv->usbd_status_cb(drv, 0 | 0x80);
    }
    usbd_prepare_Setup(drv);
}


void usbd_EpXferComplete(USB_DRIVER *drv, uint8_t endp_number)
{
    USBD_TRB      *TrbPtr;
    USBD_EP       *Ept;
    uint32_t     Length;
    uint8_t      Dir;
    uint32_t Status;
    Ept = &drv->eps[endp_number];
    Dir = Ept->ep_dir;
    TrbPtr = &Ept->EpTrb[Ept->trb_dequeue];
   // RTSS_InvalidateDCache_by_Addr(TrbPtr, sizeof(*TrbPtr));
    SCB_InvalidateDCache_by_Addr(TrbPtr, sizeof(*TrbPtr));
    Status = USB_TRB_SIZE_TRBSTS(TrbPtr->Size);
    if (Status == USB_TRBSTS_SETUP_PENDING)
    {
        drv->setup_packet_pending = 1;
#ifdef DEBUG
        printf("TRB transmission pending in BULK DATA\n");
#endif
        return;
    }
    Ept->trb_dequeue++;
    if (Ept->trb_dequeue == NO_OF_TRB_PER_EP)
    {
        Ept->trb_dequeue = 0U;
    }
    Length = TrbPtr->Size & USB_TRB_SIZE_MASK;
    if (Length == 0U)
    {
        Ept->BytesTxed = Ept->ep_requested_bytes;
    }
    else
    {
        if (Dir == USB_DIR_IN)
        {
            Ept->BytesTxed = Ept->ep_requested_bytes - Length;
        }
        else
        {
            if (Ept->UnalignedTx == 1U)
            {
                Ept->BytesTxed = roundup(Ept->ep_requested_bytes,
                        Ept->ep_maxpacket);
                Ept->BytesTxed -= Length;
                Ept->UnalignedTx = 0U;
            }
            else
            {
                /* Get the actual number of bytes transmitted by host */
                Ept->BytesTxed = Ept->ep_requested_bytes - Length;
            }
        }
    }
    drv -> NumBytes =  Ept->BytesTxed;
    if((drv->usbd_data_in_cb != NULL) && Dir)
    {
        drv->usbd_data_in_cb(drv, 2 | 0x80);// bulk in endpoint number is 2 need to fix
    }
    else
    {
    	bulk_out_cb ++;
        if(drv->usbd_data_out_cb != NULL)
           drv->usbd_data_out_cb(drv, 1); //bulk out endpoint number is 1, need to fix
    }
}

void usbd_clear_trb(USB_DRIVER *drv, uint8_t endp_number)
{
    USBD_TRB      *TrbPtr;
    USBD_EP       *Ept;
    Ept = &drv->eps[endp_number];
    TrbPtr = &Ept->EpTrb[Ept->trb_dequeue];
    if(TrbPtr->Ctrl & USB_TRB_CTRL_HWO)
    {
        TrbPtr->Ctrl &= ~(USB_TRB_CTRL_HWO);
    }
}
/* Device specific events   */
void usbd_devt_handler(USB_DRIVER *drv, uint32_t reg)
{
    uint32_t  event_info;
    uint32_t event_type;
    int32_t status;
    /* link state event info  */
    event_info = USB_DEVT_LINK_STATE_INFO(reg);

    /* Device specific events */
    event_type = USB_DEVT_TYPE(reg);
    switch(event_type)
    {
        case USB_EVENT_WAKEUP:
            break;
        case USB_EVENT_DISCONNECT:
        	if(drv->usbd_disconnect_cb != NULL)
        		drv->usbd_disconnect_cb(drv);
            break;
        case USB_EVENT_EOPF:
            break;
        case USB_EVENT_RESET:

            /* disabling test mode  */
            usbd_reset_event(drv);
            reset_cnt ++;
            if(drv->usbd_device_reset_cb != NULL)
            	drv->usbd_device_reset_cb(drv);
            break;
        case USB_EVENT_CONNECT_DONE:
            usbd_connectionDone_event(drv);
            connection_done_cnt ++;
            if( drv->usbd_connect_cb != NULL)
                 drv->usbd_connect_cb(drv);

            usbd_prepare_Setup(drv);
            drv->IsConfigDone = 0;
            break;
        case USB_EVENT_LINK_STATUS_CHANGE:
            drv -> link_state = event_info;
            usbd_linksts_change_event(drv, event_info);
            break;
        case USB_EVENT_HIBER_REQ:
            break;
        default:
            break;
    }
}

void usbd_linksts_change_event(USB_DRIVER *drv, uint32_t event_info)
{
    /* not yet implemented fully  */
    switch(event_info)
    {
        case USB_LINK_STATE_ON: /* ON state */
            break;
        case USB_LINK_STATE_L1: /* Sleep mode  */
            break;

        case USB_LINK_STATE_L2:/* Suspend  */
            break;
        case USB_LINK_STATE_DIS: /*Disconnected  */
            break;
        case USB_LINK_STATE_EARLY_SUS:/* early Suspend  */
            break;
        case USB_LINK_STATE_RESET: /* Disconnected  */
            break;
        case USB_LINK_STATE_RESUME: /* Reseume  */
            break;
        default:
            break;
    }
}


/* Reset the USB device */
void usbd_reset_event(USB_DRIVER *drv)
{
    uint32_t reg;
    uint32_t Index;
    reg = drv->regs->DCTL;
    reg &= ~USB_DCTL_TSTCTRL_MASK;
    drv->regs->DCTL = reg;
    /* Clear STALL on all endpoints */
    //usbd_ClearStallAllEp(drv); //mahesh commented
    for (Index = 0U; Index < 8;Index++)
    {
        drv->eps[Index].ep_status = 0U;
    }
    drv->IsConfigDone = 0U;

    /* Reset device address to zero */
    reg = drv->regs->DCFG;
    reg &= ~(USB_DCFG_DEVADDR_MASK);
    drv->regs->DCFG = reg;

}
void usbd_connectionDone_event(USB_DRIVER *drv)
{
    uint32_t reg;
    uint32_t speed;
    reg = drv->regs->DSTS;
    /* if speed value is 0 then it's HIGH SPEED */
    speed = reg & USB_DSTS_CONNECTSPD;
    if(speed == USB_DSTS_HIGHSPEED)
    {
        //drv->speed = UX_HIGH_SPEED_DEVICE;
        drv->speed = 2;
    }
    /* Enable USB2 LPM Capability */
    reg = drv->regs->DCFG;
    reg |= USB_DCFG_LPM_CAP;
    drv->regs->DCFG = reg;
    reg = drv->regs->DCTL;
    reg |= USB_DCTL_HIRD_THRES_MASK;
    drv->regs->DCTL = reg;

}

/* Endpoint specific events */
void usbd_depevt_handler(USB_DRIVER *drv, uint32_t reg)
{
    USBD_EP *Ept;
    uint8_t endp_number;

    USBD_EP      *ed;

    uint32_t event_status;
    uint32_t event_type;
    uint32_t ret;
    /* Event status from bit[15:12]  */
    event_status =  (reg >> USB_EVT_EPSTATUS_MASK) & 0xf;
    endp_number = USB_GET_DEPEVT_EP_NUM(reg);
    drv->endp_number = endp_number;
    Ept = &drv->eps[endp_number];
    if (!(Ept->ep_status & USB_EP_ENABLED))
    {
#ifdef DEBUG
        printf("endpoint has not enabled\n");
#endif
        return;
    }
    /*  Get the event type  */
    event_type = USB_GET_DEPEVT_TYPE(reg);
#ifdef DEBUG
    /* Event type can be used for Debugging purpose */
    drv->event_type = event_type;
#endif
    if (endp_number == 0 || endp_number == 1)
    {
        //RTSS_InvalidateDCache_by_Addr(&drv->SetupData, 8);
    	SCB_InvalidateDCache_by_Addr(&drv->SetupData, 8);

        /* Get the physical endpoint associated with this endpoint.  */
        ed =  &drv -> eps[endp_number];

        /* Reset the endpoint transfer status. */
        ed -> ep_transfer_status =  USB_ED_TRANSFER_STATUS_IDLE;

        /* Process the ep0 interrupts bases on event_type. */
        if(event_type == USB_DEPEVT_XFERNOTREADY)
        {
            if (event_status == USB_DEPEVT_STATUS_CONTROL_DATA)
            {
                /* We already have a DATA transfer in the controller's cache,
                 * if we receive a XferNotReady(DATA) we will ignore it, unless
                 * it's for the wrong direction.
                 *
                 * In that case, we must issue END_TRANSFER command to the Data
                 * Phase we already have started and issue SetStall on the
                 * control endpoint.
                 */
                if (endp_number != drv -> ep0_expect_in)
                {
#ifdef DEBUG
                    printf("unexpected direction for the data phase\n");
#endif
                    usbd_ep0_end_control_data(drv, Ept);
                    usbd_ep0_stall_restart(drv);
                }

            }
            else if (event_status == USB_DEPEVT_STATUS_CONTROL_STATUS)
            {
                //drv->ep0state = EP0_STATUS_PHASE;
                if( status_sent == 1)
                {
                    usbd_SetDeviceAddress(drv, drv->SetupData.wValue);
                }
                usbd_ep0_start_status(drv, endp_number);

                status_sent ++;
            }
            else
            {
#ifdef DEBUG
                /* Do nothing  */
#endif
            }
        }
        else if(event_type == USB_DEPEVT_XFERCOMPLETE)
        {
            USBD_CTRL_REQUEST *Ctrl;
            Ctrl = &drv->SetupData;
            Ept = &drv->eps[endp_number];
            Ept->ep_status &= ~(USB_EP_BUSY);
            Ept->ep_resource_index = 0U;
            drv->setup_packet_pending = 0;
            switch (drv->ep0state)
            {
                case EP0_SETUP_PHASE:
                    ed -> ep_transfer_status =  USB_ED_TRANSFER_STATUS_SETUP;
                    if (Ctrl->wLength == 0U)
                    {
                        drv -> three_stage_setup = 0;
                        drv -> ep0_expect_in = USB_DIR_OUT;
                    }
                    else
                    {
                        drv -> three_stage_setup = 1U;
                        drv -> ep0_expect_in = !!(Ctrl->bRequestType & UX_REQUEST_IN);
                    }
                    setup_done ++;
                    if(drv->usbd_setupstage_cb != NULL)
                        drv->usbd_setupstage_cb(drv);

                    break;

                case EP0_DATA_PHASE:
                    ed -> ep_transfer_status =  USB_ED_TRANSFER_STATUS_OUT_COMPLETION;
                    usbd_ep0_data_done(drv, endp_number);
                    data_done ++;
                    break;
                case EP0_STATUS_PHASE:
                    usbd_ep0_status_done(drv);
                    status_done ++;
                    break;
                default:
                    break;
            }
        }
        else
        {
#ifdef DEBUG
            printf("some other events\n");
#endif
        }
    }
    else
    {

        /* BULK IN and BULK OUT events */
        ed =  &drv -> eps[endp_number];
        switch (event_type)
        {
            case USB_DEPEVT_XFERINPROGRESS:
            	xfer_cmplt ++;
                usbd_EpXferComplete(drv, endp_number);
                break;
            case USB_DEPEVT_XFERCOMPLETE:
                break;
            case USB_DEPEVT_XFERNOTREADY:
                break;
            default:
                break;
        }
    }
}


/* save the current phy state and disable the lpm and suspend  */
void  usbd_usb2phy_config_check(USB_DRIVER *drv)
{
    uint32_t reg;
    drv -> endp_config = 0;
    reg = drv->regs->GUSB2PHYCFG0;

    if (reg & USB_GUSB2PHYCFG_SUSPHY)
    {
        drv -> endp_config |= USB_GUSB2PHYCFG_SUSPHY;
        reg &= ~USB_GUSB2PHYCFG_SUSPHY;
    }

    if (reg & USB_GUSB2PHYCFG_ENBLSLPM)
    {
        drv -> endp_config |= USB_GUSB2PHYCFG_ENBLSLPM;
        reg &= ~USB_GUSB2PHYCFG_ENBLSLPM;
    }

    if (drv -> endp_config)
        drv->regs->GUSB2PHYCFG0 = reg;
}
/* Restore the phy state */
void  usbd_usb2phy_config_reset(USB_DRIVER *drv)
{
    uint32_t reg;
    if (drv -> endp_config)
    {
        reg = drv->regs->GUSB2PHYCFG0;
        reg |= drv -> endp_config;
        drv->regs->GUSB2PHYCFG0 = reg;
    }
}

/**
  \fn          usbd_SetDeviceAddress
  \brief       This function is used to set device address
  \param[in]   pointer to controller context structure
  \param[in]   Address to be set
  \return      On success 0, failure cases error
 */
uint32_t usbd_SetDeviceAddress(USB_DRIVER *drv, uint8_t  Addr)
{
    uint32_t reg;
    if (Addr > 127)
    {
#ifdef DEBUG
        printf("Invalid address\n");
#endif
        return 1;
    }
    if (drv->config_state == USB_STATE_CONFIGURED)
    {
#ifdef DEBUG
        printf("address can't set from Configured State\n");
#endif
        return 1;
    }
    reg = drv->regs->DCFG;
    reg &= ~(USB_DCFG_DEVADDR_MASK);
    reg |= USB_DCFG_DEVADDR(Addr);
    drv->regs->DCFG = reg;

    if (Addr > 0U)
    {
        drv -> config_state = USB_STATE_ADDRESS;
    }
    else
    {
        drv -> config_state = USB_STATE_DEFAULT;
    }
    return 0;
}

void usbd_ep0_start_send_receive(USB_DRIVER *drv, uint8_t ep_num, uint8_t Dir)
{
    uint8_t PhyEpNum;
    PhyEpNum = USB_PhysicalEp(ep_num, Dir);
    usbd_ep0_start_status(drv, PhyEpNum);
}

