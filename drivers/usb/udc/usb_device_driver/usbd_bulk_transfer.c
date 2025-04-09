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
 * @file     usbd_bulk_transfer.c
 * @author   Mahesh Avula
 * @email    mahesh.avula@alifsemi.com
 * @version  V1.0.0
 * @date     13-March-2024
 * @brief    This file will transfers the BULK IN and BULK OUT data
 * @bug      None
 * @Note     None
 ******************************************************************************/

/* Include necessary system files.  */

#include "usbd_dwc3.h"
//#include "system_utils.h"

/**
  \fn           usbd_bulk_send
  \brief        This function will prepare the TRB for sending bulk data to the host.
  \param[in]    pointer to the controller context structure
  \param[in]    endpoint number
  \param[in]    endpoint direction
  \param[in]    buffer pointer
  \param[in]    buffer length
  \return       On success 0, failure case returns error.
 **/
int32_t usbd_bulk_send(USB_DRIVER *drv, uint8_t ep_num, uint8_t Dir, uint8_t *bufferptr, uint32_t buf_len)
{
    USBD_EP *Ept;
    USBD_TRB      *TrbPtr;
    USBD_EP_PARAMS Params;
    uint8_t      PhyEpNum;
    int32_t     Ret;
    uint32_t      cmd;
    PhyEpNum = USB_PhysicalEp(ep_num, Dir);
    Ept = &drv->eps[PhyEpNum];
    if (Ept->ep_dir != USB_DIR_IN)
    {
#ifdef  DEBUG
        printf("Direction is wrong returning\n");
#endif
        return -1;
    }
    Ept->BytesTxed = 0U;
    Ept->ep_requested_bytes = buf_len;

    TrbPtr = &Ept->EpTrb[Ept->trb_enqueue];
    Ept->trb_enqueue++;
    if (Ept->trb_enqueue == NO_OF_TRB_PER_EP)
    {
        Ept->trb_enqueue = 0U;
    }
   // RTSS_CleanDCache_by_Addr(bufferptr, buf_len);
    SCB_CleanDCache_by_Addr(bufferptr, buf_len);
    TrbPtr->BufferPtrLow  = lower_32_bits(local_to_global((uint32_t *)bufferptr));
    TrbPtr->BufferPtrHigh  = 0;
    TrbPtr->Size = USB_TRB_SIZE_LENGTH(buf_len);
    if(buf_len == 0)
    {
        /* Normal ZLP(BULK IN) - set to 9 for BULK IN TRB for zero length packet termination */
        TrbPtr->Ctrl = USB_TRBCTL_NORMAL_ZLP;
    }
    else
    {
        /* For Bulk TRB control(TRBCTL) as Normal  */
        TrbPtr->Ctrl = USB_TRBCTL_NORMAL;
    }

    TrbPtr->Ctrl |= (USB_TRB_CTRL_HWO | USB_TRB_CTRL_IOC);
    //RTSS_CleanDCache_by_Addr(TrbPtr, sizeof(*TrbPtr));
    SCB_CleanDCache_by_Addr(TrbPtr, sizeof(*TrbPtr));
    Params.Param0 = 0U;
    Params.Param1 = (uint32_t)TrbPtr;
    if ((Ept->ep_status & USB_EP_BUSY) != 0U)
    {
        cmd = USB_DEPCMD_UPDATETRANSFER;
        cmd |= USB_DEPCMD_PARAM(Ept->ep_resource_index);
    }
    else
    {
        cmd = USB_DEPCMD_STARTTRANSFER;
    }
    //k_busy_wait(50);
    Ret = usbd_SendEpCmd(drv, PhyEpNum, cmd, Params);
    if (Ret < 0)
    {
#ifdef DEBUG
        printf("failed to send the command\n");
#endif
        return Ret;
    }
    //k_busy_wait(50);
    if ((Ept->ep_status & USB_EP_BUSY) == 0U)
    {
        Ept->ep_resource_index = usbd_EpGetTransferIndex(drv,
                Ept->ep_index, Ept->ep_dir);

        Ept->ep_status |= USB_EP_BUSY;
    }
    return Ret;
}
/**
  \fn           usbd_bulk_recv
  \brief        This function will prepare the TRB for receiving bulk data from host.
  \param[in]    pointer to the controller context structure
  \param[in]    endpoint number
  \param[in]    endpoint direction
  \param[in]    buffer pointer
  \param[in]    buffer length
  \return       On success 0, failure case returns error.
 **/

int32_t usbd_bulk_recv(USB_DRIVER *drv, uint8_t ep_num, uint8_t Dir, uint8_t *bufferptr, uint32_t buf_len)
{
    USBD_EP *Ept;
    USBD_TRB   *TrbPtr;
    USBD_EP_PARAMS Params;
    uint8_t      PhyEpNum;
    uint32_t size;
    uint32_t cmd;
    int32_t     Ret;
    PhyEpNum = USB_PhysicalEp(ep_num, Dir);
    Params.Param0 = 0U;
    Params.Param1 = 0U;
    Params.Param2 = 0U;

    Ept = &drv->eps[PhyEpNum];
    if (Ept->ep_dir != Dir)
    {
#ifdef DEBUG
        printf("Wrong BULK endpoint direction\n");
#endif
        return -1;
    }
    if(buf_len < Ept->ep_maxpacket)
    {
        size = Ept->ep_maxpacket;
        Ept->ep_requested_bytes = Ept->ep_maxpacket;
    }
    else
    {
        size = buf_len;
        Ept->ep_requested_bytes = buf_len;
    }
    Ept->BytesTxed = 0U;
    /*
     * An OUT transfer size (Total TRB buffer allocation)
     * must be a multiple of MaxPacketSize even if software is expecting a
     * fixed non-multiple of MaxPacketSize transfer from the Host.
     */
    if (IS_ALIGNED(buf_len, Ept->ep_maxpacket))
    {
        size = roundup(buf_len, Ept->ep_maxpacket);
        Ept->UnalignedTx = 1U;
    }

    TrbPtr = &Ept->EpTrb[Ept->trb_enqueue];

    //memset((uint8_t *)TrbPtr - 16, 0x00, 16);

    Ept->trb_enqueue ++;
    if (Ept->trb_enqueue == NO_OF_TRB_PER_EP)
    {
        Ept->trb_enqueue = 0U;
    }
    //RTSS_CleanDCache_by_Addr(bufferptr, buf_len);
    SCB_CleanDCache_by_Addr(bufferptr, buf_len);

    TrbPtr->BufferPtrLow  = lower_32_bits(local_to_global((uint32_t *)bufferptr));
    TrbPtr->BufferPtrHigh  = 0;

    TrbPtr->Size = USB_TRB_SIZE_LENGTH(size);
    TrbPtr->Ctrl = USB_TRBCTL_NORMAL;
    TrbPtr->Ctrl |= ( USB_TRB_CTRL_CSP | USB_TRB_CTRL_IOC
            | USB_TRB_CTRL_ISP_IMI
            | USB_TRB_CTRL_HWO);
    //RTSS_CleanDCache_by_Addr(TrbPtr, sizeof(*TrbPtr));
    SCB_CleanDCache_by_Addr(TrbPtr, sizeof(*TrbPtr));
   // printk("trb add %x trb low %x size %d trb ctrl %x\n",TrbPtr, TrbPtr->BufferPtrLow, TrbPtr->Size, TrbPtr->Ctrl);
    Params.Param0 = 0;
    Params.Param1 = (uint32_t)TrbPtr;
    if ((Ept->ep_status & USB_EP_BUSY) != 0U)
    {
        cmd = USB_DEPCMD_UPDATETRANSFER;
        cmd |= USB_DEPCMD_PARAM(Ept->ep_resource_index);
    }
    else
    {
        cmd = USB_DEPCMD_STARTTRANSFER;
    }
   // printk("buff legnth: %d\n",buf_len);
    Ret = usbd_SendEpCmd(drv, PhyEpNum, cmd, Params);
    if (Ret < 0)
    {
#ifdef DEBUG
        printf("SendEpCmd failed\n");
#endif
        return Ret;
    }

    if ((Ept->ep_status & USB_EP_BUSY) == 0U)
    {
        Ept->ep_resource_index = usbd_EpGetTransferIndex(drv,
                Ept->ep_index,
                Ept->ep_dir);

        Ept->ep_status |= USB_EP_BUSY;
    }
    return Ret;
}
