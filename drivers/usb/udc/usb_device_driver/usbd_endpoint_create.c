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
 * @file     usbd_endpoint_create.c
 * @author   Mahesh Avula
 * @email    mahesh.avula@alifsemi.com
 * @version  V1.0.0
 * @date     10-Feb-2022
 * @brief    This function will create and configure the all the endpoints.
 * @bug      None.
 * @Note     None.
 ******************************************************************************/



/* Include necessary system files.  */

#include "usbd_dwc3.h"
//#include "system_utils.h"

/**
  \fn           usbd_endpoint_create
  \brief        This function will creates the all types of endpoints.
  \param[in]    pointer to the controller context structure
  \param[in]    endpoint type
  \param[in]    endpoint num
  \param[in]    endpoint direction
  \param[in]    max endpoint pkt size
  \return       On success 0, failure case returns error.
 **/

int32_t  usbd_endpoint_create(USB_DRIVER *drv,uint8_t ep_type, uint8_t ep_num, uint8_t Dir,
		uint16_t ep_max_packet_size, uint8_t ep_interval)
{
    int32_t Status;
    switch (ep_type)
    {
        case USB_CONTROL_EP:

            /* Enable the control endpoint  */
            Status = usbd_dwc3_ep_enable(drv, ep_num, Dir, ep_type, ep_max_packet_size, ep_interval);
            if(Status)
            {
#ifdef DEBUG
                printf("Failed to enable control ep num : %d Dir: %d\n",ep_num, Dir);
#endif
                return Status;
            }
            break;
        case USB_BULK_EP:
            Status = usbd_dwc3_ep_enable(drv, ep_num, Dir, ep_type, ep_max_packet_size, ep_interval);
            if(Status)
            {
#ifdef DEBUG
                printf("Failed to enable bulk ep num %d: Dir: %d\n",ep_num, Dir);
#endif
                return Status;
            }
            break;
        case USB_ISOCRONOUS_EP:

            /* Not yet implemented   */
            break;
        case USB_INTERRUPT_EP:
            /* Enable the Interrupt endpoint  */
            Status = usbd_dwc3_ep_enable(drv, ep_num, Dir, ep_type, ep_max_packet_size, ep_interval);
            if(Status)
            {
#ifdef DEBUG
                printf("failed to enable interrupt ep num %d: Dir: %d\n",ep_num, Dir);
#endif
                return Status;
            }
            break;
        default:
            return -1;
    }
    /* Return successful completion.  */
    return Status;
}

uint32_t usbd_dwc3_ep_enable(USB_DRIVER *drv, uint8_t ep_num, uint8_t Dir, uint8_t ep_type,
     uint16_t ep_max_packet_size, uint8_t ep_interval)
{
    USBD_EP *Ept;
    //USBD_EP_PARAMS Params;
    uint32_t reg;
    uint8_t PhyEpNum;
    uint32_t ret;
    PhyEpNum = USB_PhysicalEp(ep_num, Dir);
    Ept = &drv -> eps[PhyEpNum];
    Ept->ep_index = ep_num;
    Ept->ep_dir = Dir;
    Ept->ep_maxpacket = ep_max_packet_size;
    Ept->phy_epnum   = PhyEpNum;
    if (!(Ept->ep_status & USB_EP_ENABLED))
    {
        ret =  usbd_start_ep_config(drv, ep_num, Dir);
        if(ret)
        {
#ifdef DEBUG
            printf("Start EPConfig failed\n");
#endif
            return ret;
        }
    }
    ret = usbd_set_ep_config(drv, ep_num, Dir, ep_type, ep_max_packet_size, ep_interval);
    if(ret)
    {
#ifdef DEBUG
        printf("SetEP failed\n");
#endif
        return ret;
    }

    if (!(Ept->ep_status & USB_EP_ENABLED))
    {
        ret = usbd_set_xferResource(drv, PhyEpNum);
        if(ret)
        {
#ifdef DEBUG
            printf("Set XferResource failed\n");
#endif
            return ret;
        }
    }
    if (!(Ept->ep_status & USB_EP_ENABLED))
    {
        Ept->ep_status |= USB_EP_ENABLED;

        reg = drv->regs->DALEPENA;
        reg |=  USB_DALEPENA_EP(Ept->phy_epnum);
        drv->regs->DALEPENA = reg;
        if(PhyEpNum > 1)
        {
            USBD_TRB *TrbStHw, *TrbLink;
            /* Initialize TRB ring   */
            Ept->trb_enqueue = 0;
            Ept->trb_dequeue = 0;
            TrbStHw = &Ept->EpTrb[0U];
            /* Link TRB. The HWO bit is never reset */
            TrbLink = &Ept->EpTrb[NO_OF_TRB_PER_EP];
            memset(TrbLink, 0x0, sizeof(USBD_TRB));

            TrbLink->BufferPtrLow = lower_32_bits(local_to_global(TrbStHw));
            TrbLink->BufferPtrHigh =  upper_32_bits(local_to_global(TrbStHw));
            TrbLink->Ctrl |= USB_TRBCTL_LINK_TRB;
            TrbLink->Ctrl |= USB_TRB_CTRL_HWO;
            return 0;
        }
        return 0;
    }
    return 1;

}
uint32_t usbd_set_ep_config(USB_DRIVER *drv, uint8_t ep_num, uint8_t Dir,uint8_t ep_type,
     uint16_t ep_max_packet_size, uint8_t ep_interval)
{
    USBD_EP *Ept;
    USBD_EP_PARAMS Params;
    uint8_t PhyEpNum;
    Params.Param0 = 0x00U;
    Params.Param1 = 0x00U;
    Params.Param2 = 0x00U;
    PhyEpNum = USB_PhysicalEp(ep_num, Dir);

    Params.Param0 = USB_DEPCFG_EP_TYPE(ep_type)
        | USB_DEPCFG_MAX_PACKET_SIZE(ep_max_packet_size);

    Params.Param0 |= USB_DEPCFG_ACTION_INIT;
    Params.Param1 = USB_DEPCFG_XFER_COMPLETE_EN | USB_DEPCFG_XFER_NOT_READY_EN;

    /*
     * We are doing 1:1 mapping for endpoints, meaning
     * Physical Endpoints 2 maps to Logical Endpoint 2 and
     * so on. We consider the direction bit as part of the physical
     * endpoint number. So USB endpoint 0x81 is 0x03.
     */
    Params.Param1 |= USB_DEPCFG_EP_NUMBER(PhyEpNum);

    if (Dir != USB_DIR_OUT)
    {
        Params.Param0 |= USB_DEPCFG_FIFO_NUMBER(PhyEpNum >> 1U);
    }
    if(ep_type != 0)
          Params.Param1 |= USB_DEPCFG_XFER_IN_PROGRESS_EN;

    if(ep_interval)
    {
        Params.Param1 |= USB_DEPCFG_BINTERVAL_M1(ep_interval - 1);
    }

   /* if(ep_type == 3)
     {
           Params.Param1 |= USB_DEPCFG_BINTERVAL_M1(9-1);//value hardcoded
     }*/
    return usbd_SendEpCmd(drv, PhyEpNum, USB_DEPCMD_SETEPCONFIG, Params);
}


uint32_t usbd_start_ep_config(USB_DRIVER *drv, uint8_t ep_num, uint8_t Dir)
{
    USBD_EP_PARAMS Params;
    uint32_t     Cmd;
    uint8_t PhyEpNum;
    uint32_t ret;
    PhyEpNum = USB_PhysicalEp(ep_num, Dir);
    Params.Param2 = 0;
    Params.Param1 = 0;
    Params.Param0 = 0;

    if (PhyEpNum != 1U)
    {
        Cmd = USB_DEPCMD_DEPSTARTCFG;
        /* XferRscIdx == 0 for EP0 and 2 for the remaining */
        if (PhyEpNum > 1U)
        {
            if (drv->IsConfigDone != 0U)
            {
                return 0;
            }
            drv->IsConfigDone = 1U;
            Cmd |= USB_DEPCMD_PARAM(2U);
        }
        ret  = usbd_SendEpCmd(drv, 0, Cmd, Params);
        if(ret)
        {
#ifdef DEBUG
            printf("Ep cmd failed\n");
#endif
            return 1;
        }
        else
        {
            return ret;
        }
    }
    return 0;
}
uint32_t usbd_set_xferResource(USB_DRIVER *drv, uint8_t PhyEpNum)
{
    USBD_EP_PARAMS Params;
    Params.Param0 = 0;
    Params.Param1 = 0;
    Params.Param2 = 0;
    Params.Param0 = USB_DEPXFERCFG_NUM_XFER_RES(1U);
    return usbd_SendEpCmd(drv, PhyEpNum, USB_DEPCMD_SETTRANSFRESOURCE, Params);
}

int32_t usbd_dwc3_ep_disable(USB_DRIVER *drv, uint8_t ep_num, uint8_t Dir)
{
    USBD_EP *Ept;
    uint32_t reg;
    uint8_t PhyEpNum;
    uint32_t ret;
    PhyEpNum = USB_PhysicalEp(ep_num, Dir);
    Ept = &drv -> eps[PhyEpNum];
    reg = drv->regs->DALEPENA;
    reg &= ~USB_DALEPENA_EP(PhyEpNum);
    drv->regs->DALEPENA = reg;
    Ept->ep_status &= ~USB_EP_ENABLED;
    return 0;
}


int32_t usbd_ep_stall(USB_DRIVER *drv, uint8_t ep_num, uint8_t Dir)
{
    uint8_t  PhyEpNum;
    USBD_EP *Ept = NULL;
    USBD_EP_PARAMS Params;
    int32_t ret;
    PhyEpNum = USB_PhysicalEp(ep_num, Dir);
    Params.Param2 = 0;
    Params.Param1 = 0;
    Params.Param0 = 0;
    Ept = &drv->eps[0];
    ret = usbd_SendEpCmd(drv, 0, USB_DEPCMD_SETSTALL, Params);
    if(ret < 0)
    {
#ifdef DEBUG
        printf("failed to send STALL command\n");
#endif
        return ret;
    }
    Ept->ep_status |= USB_EP_STALL;

    /*else
    {
        Ept = &drv->eps[PhyEpNum];
        ret = usbd_SendEpCmd(drv, PhyEpNum, USB_DEPCMD_CLEARSTALL, Params);
        if(ret < 0)
        {
#ifdef DEBUG
             printf("failed to send CLEARSTALL command\n");
#endif
             return ret;
        }
        Ept->ep_status  &= ~(USB_EP_STALL | USB_EP_WEDGE);
    }*/
    return ret;
}

int32_t usbd_ep_clearstall(USB_DRIVER *drv, uint8_t ep_num, uint8_t Dir)
{
	uint8_t  PhyEpNum;
	USBD_EP *Ept = NULL;
	USBD_EP_PARAMS Params;
	int32_t ret;
	PhyEpNum = USB_PhysicalEp(ep_num, Dir);
	Params.Param2 = 0;
	Params.Param1 = 0;
	Params.Param0 = 0;
	Ept = &drv->eps[PhyEpNum];
	ret = usbd_SendEpCmd(drv, PhyEpNum, USB_DEPCMD_CLEARSTALL, Params);
	if(ret < 0)
	{
	#ifdef DEBUG
		printf("failed to send CLEARSTALL command\n");
	#endif
		return ret;
	}
	Ept->ep_status  &= ~(USB_EP_STALL | USB_EP_WEDGE);
	return ret;
}

int32_t usbd_ep_transfer_abort(USB_DRIVER *drv, uint8_t ep_num, uint8_t Dir)
{
    uint8_t ForceRM = 1;
    int32_t ret;
    /* As per the data sheet ENDTRANSFER COMMAND,
     * Software issues this command requesting DMA to stop for the endpoint/stream specifying
     * the transfer resource index of the TRB and the ForceRM parameter to be set to 1 in
     *  the DEPCMD register. */

    /* When issuing an End Transfer command, software must set the CmdIOC bit (field 8)
     * so that an Endpoint Command Complete event is generated after the transfer ends.
     */
    ret = usbd_StopTransfer(drv, ep_num, Dir ,ForceRM);
    return ret;
}

