/* Copyright (C) 2024 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

/**************************************************************************//**
 * @file     usbd_control_transfer.c
 * @author   Mahesh Avula
 * @email    mahesh.avula@alifsemi.com
 * @version  V1.0.0
 * @date     12-Dec-2023
 * @brief    dwc3 controller driver.
 * @bug      None
 * @Note     None
 ******************************************************************************/

/* Include necessary system files.  */

#include "usbd_dwc3.h"
//#include "system_utils.h"


/**
  \fn           usbd_ep0_recv
  \brief        This function prepares the TRB for recv control data from host
  \param[in]    pointer to the controller context structure
  \param[in]    endpoint number
  \param[in]    endpoint direction
  \param[in]    data buffer pointer
  \param[in]    data buffer lenth
  \return       On success 0, failure case returns error.
 **/

int32_t usbd_ep0_recv(USB_DRIVER *drv, uint8_t ep_num, uint8_t Dir, uint8_t *bufferptr, uint32_t buf_len)
{
    uint8_t PhyEpNum;
    USBD_EP_PARAMS Params;
    USBD_EP       *Ept;
    USBD_TRB      *TrbPtr;
    int32_t ret;

    PhyEpNum = USB_PhysicalEp(ep_num, Dir);
    Ept = &drv->eps[PhyEpNum];

    Params.Param2 = 0;
    Params.Param1 = 0;
    Params.Param0 = 0;

    if ((Ept->ep_status & USB_EP_BUSY) != 0U)
    {
#ifdef DEBUG
        printf("Endpoint 0 already busy returning\n");
#endif
        return -1;
    }
    if(buf_len < 64)
        	buf_len = 64;
    Ept->ep_requested_bytes = buf_len;
    Ept->BytesTxed = 0U;
    TrbPtr = &drv->endp0_trb;
   // RTSS_CleanDCache_by_Addr(bufferptr, buf_len);
    SCB_CleanDCache_by_Addr(bufferptr, buf_len);
    TrbPtr->BufferPtrLow  =  lower_32_bits(local_to_global((uint32_t *)bufferptr));
    TrbPtr->BufferPtrHigh  = 0;

    TrbPtr->Size = buf_len;
    TrbPtr->Ctrl = USB_TRBCTL_CONTROL_DATA;

    TrbPtr->Ctrl |= (USB_TRB_CTRL_HWO
            | USB_TRB_CTRL_LST | USB_TRB_CTRL_ISP_IMI
            | USB_TRB_CTRL_IOC);
    //RTSS_CleanDCache_by_Addr(TrbPtr, sizeof(*TrbPtr));
    SCB_CleanDCache_by_Addr(TrbPtr, sizeof(*TrbPtr));
    Params.Param0 = 0U;
    Params.Param1 = (uint32_t)TrbPtr;
    drv->ep0state = EP0_DATA_PHASE;
    ret = usbd_SendEpCmd(drv, PhyEpNum, USB_DEPCMD_STARTTRANSFER, Params);
    if (ret < 0)
    {
#ifdef DEBUG
        printf("Failed to send command over EP0\n");
#endif
        return ret;
    }

    Ept->ep_status |= USB_EP_BUSY;
    /* In response to the Start Transfer command, the hardware assigns this transfer a resource index number
     * (XferRscIdx) and returns the index in the DEPCMDn register and in the Command Complete event.
     *  This index must be used in subsequent Update and End Transfer commands  */
    Ept->ep_resource_index = usbd_EpGetTransferIndex(drv,
            Ept->ep_index,
            Ept->ep_dir);
    return ret;
}

/**
  \fn           usbd_ep0_send
  \brief        This function prepares the TRB for control data send to host
  \param[in]    pointer to the controller context structure
  \param[in]    endpoint number
  \param[in]    endpoint direction
  \param[in]    data buffer pointer
  \param[in]    data buffer lenth
  \return       On success 0, failure case returns error.
 **/

int32_t usbd_ep0_send(USB_DRIVER *drv, uint8_t ep_num, uint8_t Dir, uint8_t *bufferptr, uint32_t buf_len)
{
    uint8_t PhyEpNum;
    USBD_EP_PARAMS Params;
    USBD_EP       *Ept;
    USBD_TRB      *TrbPtr;
    int32_t ret;
    if(buf_len ==0)
    {
        return -1;
    }
    /* Control IN - EP1 */
    PhyEpNum = USB_PhysicalEp(ep_num, Dir);
    Ept = &drv->eps[PhyEpNum];

    Params.Param2 = 0;
    Params.Param1 = 0;
    Params.Param0 = 0;

    if ((Ept->ep_status & USB_EP_BUSY) != 0U)
    {
#ifdef DEBUG
        printf("Endpoint 1 already busy returning\n");
#endif
        return -1;
    }

    Ept->ep_requested_bytes = buf_len;
    Ept->BytesTxed = 0U;
    TrbPtr = &drv->endp0_trb;
   // RTSS_CleanDCache_by_Addr(bufferptr, buf_len);
    SCB_CleanDCache_by_Addr(bufferptr, buf_len);
    TrbPtr->BufferPtrLow  =  lower_32_bits(local_to_global((uint32_t *)bufferptr));
    TrbPtr->BufferPtrHigh  = 0;
    TrbPtr->Size = buf_len;
    TrbPtr->Ctrl = USB_TRBCTL_CONTROL_DATA;
    TrbPtr->Ctrl |= (USB_TRB_CTRL_HWO
            | USB_TRB_CTRL_LST | USB_TRB_CTRL_ISP_IMI
            | USB_TRB_CTRL_IOC);

   // RTSS_CleanDCache_by_Addr(TrbPtr, sizeof(*TrbPtr));
    SCB_CleanDCache_by_Addr(TrbPtr, sizeof(*TrbPtr));
    Params.Param0 = 0U;
    Params.Param1 = (uint32_t)TrbPtr;
    drv->ep0state = EP0_DATA_PHASE;
    ret = usbd_SendEpCmd(drv, PhyEpNum, USB_DEPCMD_STARTTRANSFER, Params);
    if (ret < 0)
    {
#ifdef DEBUG
        printf("Failed in sending data over EP1\n");
#endif
        return ret;
    }

    Ept->ep_status |= USB_EP_BUSY;
    /* In response to the Start Transfer command, the hardware assigns this transfer a resource index number
     * (XferRscIdx) and returns the index in the DEPCMDn register and in the Command Complete event.
     *  This index must be used in subsequent Update and End Transfer commands  */
    Ept->ep_resource_index = usbd_EpGetTransferIndex(drv,
            Ept->ep_index,
            Ept->ep_dir);

    return ret;
}
/**
  \fn           usbd_SendEpCmd
  \brief        This function sends the controller endpoint commands.
  \param[in]    pointer to our controller context structure
  \param[in]    physical endpoint number
  \param[in]    command to be send
  \param[in]    endpoint params which contain trb address
  \return       On success 0, failure case returns error.
 **/
int32_t usbd_SendEpCmd(USB_DRIVER *drv, uint8_t PhyEpNum, uint32_t Cmd, USBD_EP_PARAMS Params)
{
    int32_t ret;
    int32_t      cmd_status = -1;
    int32_t      timeout = USB_DEPCMD_TIMEOUT;

    /* Check usb2phy config before issuing DEPCMD  */
    usbd_usb2phy_config_check(drv);
    if (USB_DEPCMD_CMD(Cmd) == USB_DEPCMD_UPDATETRANSFER )
    {
        Cmd &= ~(USB_DEPCMD_CMDIOC | USB_DEPCMD_CMDACT);
    }
    else
    {
        Cmd |= USB_DEPCMD_CMDACT;
    }

   // __asm ("dsb");
   // __asm ("isb");
    if(USB_DEPCMD_CMD(Cmd) == USB_DEPCMD_STARTTRANSFER)
    {
        drv->regs->USB_ENDPNT_CMD[PhyEpNum].DEPCMDPAR1 = (uint32_t)local_to_global((void *)(Params.Param1));
    }
    else
    {
        drv->regs->USB_ENDPNT_CMD[PhyEpNum].DEPCMDPAR1 = Params.Param1;
    }
    /* Issuing DEPCFG Command for appropriate endpoint */
    drv->regs->USB_ENDPNT_CMD[PhyEpNum].DEPCMDPAR0 = Params.Param0;
    drv->regs->USB_ENDPNT_CMD[PhyEpNum].DEPCMD = Cmd;
    do
    {
        /* Read the device endpoint Command register.  */
        ret = drv->regs->USB_ENDPNT_CMD[PhyEpNum].DEPCMD;
        if (!(ret & USB_DEPCMD_CMDACT))
        {
            cmd_status = USB_DEPCMD_STATUS(ret);
            switch(cmd_status)
            {
                case 0:
                    ret = cmd_status;
                    break;
                case USB_DEPEVT_TRANSFER_NO_RESOURCE:
                    ret = -1;
                    break;
                case USB_DEPEVT_TRANSFER_BUS_EXPIRY:
                    ret =  -1;
                    break;
                default:
#ifdef DEBUG
                    printf("Unknown cmd status\n");
#endif
                    break;
            }
            break;
        }
    }while(--timeout);

    if (timeout == 0)
    {
        ret = -1;
#ifdef DEBUG
        printf("timeout\n");
#endif
    }
    /* Restore the USB2 phy state  */
    usbd_usb2phy_config_reset(drv);
    return ret;
}
