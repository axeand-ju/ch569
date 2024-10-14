#include "CH56xusb30h_LIB.h"


/*******************************************************************************
* @fn        USB30HOST_Init
*
* @brief     Initializes or de-initializes the USB 3.0 host controller.
*
* @param     sta        - Functional state: ENABLE or DISABLE.
*            endpTXbuff - Pointer to the host transmit buffer.
*            endpRXbuff - Pointer to the host receive buffer.
*
* @return    None
*/
void MY_USB30HOST_Init(FunctionalState sta, PUINT8 endpTXbuff, PUINT8 endpRXbuff)
{
	printf("USB30HOST_Init()\n");
   if (sta == ENABLE)
   {
       // Initialize USB 3.0 Host

       // Reset the receive control register
       USBSSH->UH_RX_CTRL = 0;

       // Wait until the power mode transition is complete (bit 2 of LINK_STATUS is cleared)
       while ((USBSSH->LINK_STATUS & LINK_BUSY) != 0);

       // Configure the USB_CONTROL register
       USBSSH->USB_CONTROL = HOST_MODE | ITP_EN | INT_BUSY_EN | DMA_EN; // 0xC9

       // Reset the transmit control register
       USBSSH->UH_TX_CTRL = 0;

       // Initialize USB_STATUS register
       USBSSH->USB_STATUS = USB_ACT_FLAG | USB_LMP_RX_FLAG | USB_LMP_TX_FLAG | USB_OV_FLAG; // 0x17

       // Set DMA addresses for transmit and receive buffers
       USBSSH->UH_TX_DMA = (UINT32)endpTXbuff;
       USBSSH->UH_RX_DMA = (UINT32)endpRXbuff;

       // Configure endpoint settings
       USBSSH->UEP_CFG = EP1_T_EN | EP1_R_EN; // 0x0202

       // Configure LINK_CFG register
       USBSSH->LINK_CFG = DOWN_FLAG | TERM_EN | CFG_EQ_EN | DEEMPH_CFG | TIE_BRK; // 0x00100143

       // Enable link interrupts
       USBSSH->LINK_INT_CTRL = LINK_RDY_IE | LINK_INACT_IE | LINK_DISABLE_IE | TERM_PRESENT_IE | LINK_TXEQ_IE | LINK_RX_DET_IE; // 0x00100C0D

       // Set LINK_CTRL register to start the host controller
       USBSSH->LINK_CTRL = TERM_EN; // 0x02
   }
   else
   {
       // De-initialize USB 3.0 Host

       // Disable link interrupts
       USBSSH->LINK_INT_CTRL = 0x00;

       // Set LINK_CTRL register to stop the host controller
       USBSSH->LINK_CTRL = DOWN_FLAG | TERM_EN | COMPLIANCE_EN; // 0x13

       // Configure LINK_CFG register (specific settings for de-initialization)
       USBSSH->LINK_CFG = PIPE_RESET | LFPS_RX_PD | DEEMPH_CFG | LPM_EN; // 0x00001128

       // Reset USB_CONTROL register
       USBSSH->USB_CONTROL = USB_FORCE_RST | USB_ALL_CLR; // 0x06

       // Reset transmit and receive control registers
       USBSSH->UH_TX_CTRL = 0x00;
       USBSSH->UH_RX_CTRL = 0x00;
   }

   return;
}


/*******************************************************************************
 * @fn     USB30H_Lmp_Init
 *
 * @brief  USB30 lmp initialization
 *
 * @return   None
 */
void MY_USB30H_Lmp_Init(void)
{
//	printf("USB30H_Lmp_Init()\n");

    // Initialize LMP Transmit Data Registers with first set of values
    USBSSH->LMP_TX_DATA0 = 0x280;
    USBSSH->LMP_TX_DATA1 = 0x10004;
    USBSSH->LMP_TX_DATA2 = 0x00000000;

	// Wait for LMP Ready flag to be set
	while ((USBSSH->USB_STATUS & USB_LMP_RX_FLAG) == 0)
		;

	// Clear status bits
	USBSSH->USB_STATUS = USB_LMP_RX_FLAG;

	// Initialize LMP Transmit Data Registers again
	USBSSH->LMP_TX_DATA0 = 0x2A0;
	USBSSH->LMP_TX_DATA1 = 0x00000000;
	USBSSH->LMP_TX_DATA2 = 0x00000000;

	// Wait for LMP Ready flag to be set
	while ((USBSSH->USB_STATUS & USB_LMP_RX_FLAG) == 0)
		;

	// Clear status bits
	USBSSH->USB_STATUS = USB_LMP_RX_FLAG;

	return;
}


/*******************************************************************************
* @fn      USB30H_Send_Setup
*
* @brief   Sends a SETUP packet over USB 3.0 host controller.
*
* @param   tx_len - Length of the SETUP packet.
*
* @return  Status code:
*          - 0: Success
*          - 1: Timeout occurred
*/
UINT8 MY_USB30H_Send_Setup(UINT32 tx_len)
{
	printf("USB30H_Send_Setup(%d)\n", tx_len);
   UINT32 timeout = 100000;

   // Start the SETUP transfer by writing to UH_TX_CTRL
   // The control word includes tx_len and specific control bits
   USBSSH->UH_TX_CTRL = (tx_len & 0xFFFF) | UH_TX_SETUP | UH_RTX_VALID | UH_RX_EOB; // 0x44010000

   // Clear HOST_STATUS
   USBSSH->HOST_STATUS = 0;

   while (1)
   {
       // Check if the transfer is complete and no errors
       if ((USBSSH->USB_STATUS & USB_ACT_FLAG) != 0)
       {
           if ((USBSSH->USB_STATUS & USB_INT_RES_MASK) == 0)
           {
               // Transfer successful
               USBSSH->UH_TX_CTRL = 0;
               USBSSH->USB_STATUS = USB_ACT_FLAG; // Clear completion status bit
               return 0; // Success
           }
           else
           {
               // Error occurred (NRDY or STALL)
               // You can handle errors here if needed
               // For this function, errors are not distinguished
           }
       }

       // Decrement the timeout counter
       if (--timeout == 0)
       {
           // Timeout occurred
           USBSSH->UH_TX_CTRL = 0; // Reset the TX control register
           return 1; // Timeout
       }
   }
}


/*******************************************************************************
 * @fn     USB30H_Send_Status
 *
 * @brief  Control transmission status phase status package
 *
 * @param  None
 *
 * @return  Status code:
 *          - 0: Success
 *          - 1: Timeout occurred
 */
UINT8 MY_USB30H_Send_Status(void)
{
	printf("USB30H_Send_Status()\n");
    UINT32 timeout = 2000000;

    // Initialize HOST_STATUS and initiate the STATUS stage transfer
    USBSSH->HOST_STATUS = 0;

    // Construct the control word to initiate the STATUS stage transfer
    UINT32 control_word = UH_TX_STATUS | UH_RTX_VALID | USB_NUMP_1; // 0x24010000
    USBSSH->UH_TX_CTRL = control_word;

    while (1)
    {
        // Check if the transfer is not complete
        if ((USBSSH->USB_STATUS & USB_ACT_FLAG) == 0)
        {
            // If an ERDY is received, re-initiate the transfer
            if ((USBSSH->USB_STATUS & USB_ERDY_FLAG) != 0)
            {
                // Re-issue the transfer
                USBSSH->UH_TX_CTRL = control_word;

                // Clear the ERDY status bit
                USBSSH->USB_STATUS = USB_ERDY_FLAG;
            }
        }
        else
        {
            // Transfer is complete; check for errors
            UINT32 error_status = USBSSH->USB_STATUS & USB_INT_RES_MASK;

            if (error_status == USB_RES_ACK || error_status == USB_RES_STALL)
            {
                // Transfer successful or STALL received
                USBSSH->UH_TX_CTRL = 0;
                USBSSH->USB_STATUS = USB_ACT_FLAG; // Clear the completion status bit
                return 0; // Success
            }
            else if (error_status == USB_RES_NRDY)
            {
                // NRDY (Not Ready) received
                USBSSH->UH_TX_CTRL = 0;
                USBSSH->USB_STATUS = USB_ACT_FLAG; // Clear the completion status bit
                // Continue looping to retry
            }
            else
            {
                // Other errors
                USBSSH->UH_TX_CTRL = 0;
                USBSSH->USB_STATUS = USB_ACT_FLAG; // Clear the completion status bit
                // Continue looping
            }
        }

        // Decrement the timeout counter
        if (--timeout == 0)
        {
            // Timeout occurred
        	USBSSH->UH_TX_CTRL = 0;
            return 1;
        }
    }
}



/*******************************************************************************
 * @fn     USB30H_Erdy_Status
 *
 * @brief  Checks for ERDY status and retrieves sequence number and endpoint number.
 *
 * @param  nump: Pointer to store the sequence number.
 *         endp: Pointer to store the endpoint number.
 *
 * @return Status code:
 *         - 1: ERDY status is set, data retrieved.
 *         - 0: ERDY status is not set.
 */
UINT8 MY_USB30H_Erdy_Status( PUINT8 nump, PUINT8 endp  )
{
    // Check if USB_ERDY_FLAG in USB_STATUS is set
    if ((USBSSH->USB_STATUS & USB_ERDY_FLAG) != 0)
    {
        // Extract sequence number (bits [12:8]) from HOST_STATUS
        *nump = (UINT8)((USBSSH->HOST_STATUS >> 8) & 0x1F);

        // Extract endpoint number (bits [7:4]) from HOST_STATUS
        *endp = (UINT8)((USBSSH->HOST_STATUS >> 4) & 0x0F);

        // Clear the ERDY status bit by writing 0x20 to USB_STATUS
        USBSSH->USB_STATUS = USB_ERDY_FLAG;

        return 1;
    }
    return 0;
}


/*******************************************************************************
* @fn     USB30H_IN_Data
*
* @brief  Get data from device endpoint
*
* @param  seq_num    - Packet sequence number
* @param  packet_num - Pointer to store the packet number
* @param  endp_num   - Endpoint number
*
* @return  UINT16:
*            Bits 0~11: Packet length
*            Bits 12~13:
*              - 1: ACK
*              - 2: NRDY
*              - 3: STALL
*              - 4: Error
*            Bit 15: EOB/LPF flag
*/
UINT16 MY_USB30H_IN_Data(UINT8 seq_num, PUINT8 packet_num, UINT8 endp_num)
{
   UINT32 rx_length;
   UINT32 timeout = 2000000; // Arbitrary large timeout value
   UINT16 status;

   // Set endpoint to host status
   USBSSH->HOST_STATUS = endp_num;

   // Start the IN transfer by writing to UH_RX_CTRL
   USBSSH->UH_RX_CTRL = UH_RTX_VALID | (seq_num << 21) | (*packet_num << 16);

   // Wait for transfer to complete or timeout
   while ((USBSSH->USB_STATUS & USB_ACT_FLAG) == 0)
   {
       if (--timeout == 0)
       {
           // Timeout occurred
           USBSSH->UH_RX_CTRL = 0; // Reset the RX control register
           return 0; // Indicate failure due to timeout
       }
   }

   // Check for errors in the USB_STATUS register
   UINT32 error_status = USBSSH->USB_STATUS & USB_INT_RES_MASK;

   if (error_status == USB_RES_ACK)
   {
       // Transfer successful (ACK received)

	   // Save current USB_RX_LEN (received packet length)
	   rx_length = USBSSH->USB_RX_LEN;

       // Retrieve packet number from UH_RX_CTRL
       *packet_num = (UINT8)((USBSSH->UH_RX_CTRL >> 16) & 0x1F);

       // Build the return value
       status = (UINT16)(rx_length & 0x0FFF);

       // Set bits 12~13 to 1 (ACK)
       status |= 0x1000;

       // Check for EOB/LPF flag in HOST_STATUS (assuming bit 16 indicates this)
       if ((USBSSH->HOST_STATUS & 0x00010000) != 0)
       {
           status |= 0x8000; // Set bit 15 for EOB/LPF
       }

       // Clear the RX control register and status bits
       USBSSH->UH_RX_CTRL = 0;
       USBSSH->USB_STATUS = USB_ACT_FLAG; // Clear completion status bit

       return status;
   }
   else if (error_status == USB_RES_NRDY)
   {
       // NRDY received
       USBSSH->UH_RX_CTRL = 0;
       USBSSH->USB_STATUS = USB_ACT_FLAG; // Clear status bits
       return 0x2000; // Bits 12~13 set to 2 (NRDY)
   }
   else if (error_status == USB_RES_STALL)
   {
       // STALL received
       USBSSH->UH_RX_CTRL = 0;
       USBSSH->USB_STATUS = USB_ACT_FLAG; // Clear status bits
       return 0x3000; // Bits 12~13 set to 3 (STALL)
   }
   else
   {
       // Other errors
       USBSSH->UH_RX_CTRL = 0;
       USBSSH->USB_STATUS = USB_ACT_FLAG; // Clear status bits
       return 0x4000; // Bits 12~13 set to 4 (Error)
   }
}


/*******************************************************************************
* @fn     USB30H_OUT_Data
*
* @brief  Sends data to a device endpoint over USB 3.0.
*
* @param  seq_num    - Packet sequence number.
*         packet_num - Number of packets.
*         endp_num   - Endpoint number.
*         tx_len     - Packet length.
*
* @return Status code:
*         - 1: ACK (Transfer successful).
*         - 2: NRDY (Not Ready).
*         - 3: STALL (Endpoint stalled).
*         - 4: Error occurred.
*/
inline UINT8 MY_USB30H_OUT_Data(UINT8 seq_num, UINT8 packet_num, UINT8 endp_num, UINT32 tx_len)
{
   UINT32 control_word;
   UINT32 timeout = 20000; // Timeout counter
   UINT32 status;

   R32_PB_OUT = (R32_PB_OUT & 0xffff0000) | (USBSSH->USB_STATUS & 0x0000ffff) | 0x02;

   // Set endpoint to host status
   USBSSH->HOST_STATUS = endp_num;

   R32_PB_OUT = (R32_PB_OUT & 0xffff0000) | (USBSSH->USB_STATUS & 0x0000ffff);

   // Construct the control word for UH_TX_CTRL
   control_word = (seq_num << 21) | (packet_num << 16) | UH_RTX_VALID | tx_len;

   // Initiate the OUT transfer by writing to UH_TX_CTRL
   USBSSH->UH_TX_CTRL = control_word;

   R32_PB_OUT = (R32_PB_OUT & 0xffff0000) | (USBSSH->USB_STATUS & 0x0000ffff);

   // Wait for the transfer to complete or timeout
   while ((USBSSH->USB_STATUS & USB_ACT_FLAG) == 0)
   {
	   R32_PB_OUT = (R32_PB_OUT & 0xffff0000) | (USBSSH->USB_STATUS & 0x0000ffff);
       if (--timeout == 0)
       {
           // Timeout occurred
           USBSSH->UH_TX_CTRL = 0; // Reset TX control register
           return 0; // Indicate timeout
       }
   }

   R32_PB_OUT = (R32_PB_OUT & 0xffff0000) | (USBSSH->USB_STATUS & 0x0000ffff);

   // Check for errors in USB_STATUS register
   UINT32 error_status = USBSSH->USB_STATUS & USB_INT_RES_MASK;

   if (error_status == USB_RES_ACK)
   {
       // Transfer successful (ACK received)
       status = USBSSH->USB_STATUS >> 5;
       USBSSH->UH_TX_CTRL = 0;
       USBSSH->USB_STATUS = USB_ACT_FLAG | USB_ERDY_FLAG; // Clear status bits
       R32_PB_OUT = (R32_PB_OUT & 0xffff0000) | (USBSSH->USB_STATUS & 0x0000ffff);
       return (status & 0xF8) | 1; // Return ACK status
   }
   else if (error_status == USB_RES_NRDY)
   {
       // NRDY received
       USBSSH->UH_TX_CTRL = 0;
       USBSSH->USB_STATUS = USB_ACT_FLAG; // Clear status bits
       R32_PB_OUT = (R32_PB_OUT & 0xffff0000) | (USBSSH->USB_STATUS & 0x0000ffff);
       return 2; // Return NRDY status
   }
   else if (error_status == USB_RES_STALL)
   {
       // STALL received
       USBSSH->USB_STATUS = USB_ACT_FLAG; // Clear status bits
       USBSSH->UH_TX_CTRL = 0;
       R32_PB_OUT = (R32_PB_OUT & 0xffff0000) | (USBSSH->USB_STATUS & 0x0000ffff);
       return 3; // Return STALL status
   }
   else
   {
       // Other errors
       USBSSH->USB_STATUS = USB_ACT_FLAG; // Clear status bits
       USBSSH->UH_TX_CTRL = 0;
       R32_PB_OUT = (R32_PB_OUT & 0xffff0000) | (USBSSH->USB_STATUS & 0x0000ffff);
       return 4; // Return error status
   }
}


/*******************************************************************************
 * @fn      USB30H_Set_Address
 *
 * @brief   Sets the destination address for the USB host controller.
 *
 * @param   address - The address to set (typically 7 bits for USB addresses).
 *
 * @return  None
 */
void MY_USB30H_Set_Address(UINT32 address)
{
	printf("USB30H_Set_Address(%d)\n", address);

    // Ensure the address is within valid range (0 to 127 for USB device addresses)
    address &= 0x7F;

    // Set the address in the upper 8 bits of USB_CONTROL register
    USBSSH->USB_CONTROL = (USBSSH->USB_CONTROL & 0x00FFFFFF) | (address << 24);

    return;
}


/*******************************************************************************
 * @fn      USB30H_Switch_Powermode
 *
 * @brief   Switches the USBSS (USB SuperSpeed) power mode.
 *
 * @param   pwr_mode - The desired power mode (POWER_MODE_0 to POWER_MODE_3).
 *
 * @return  None
 */
void MY_USB30H_Switch_Powermode(UINT32 pwr_mode)
{
	printf("USB30H_Switch_Powermode(%d)\n", pwr_mode);

    // Ensure pwr_mode is within valid range (0 to 3)
    pwr_mode &= 0x3;

    // Update the power mode bits in the LINK_CTRL register
    USBSSH->LINK_CTRL = (USBSSH->LINK_CTRL & ~0x3) | pwr_mode;

    // Wait until the power mode change is complete
    while ((USBSSH->LINK_STATUS & LINK_BUSY) != 0)
    {
        // Optional: Add a small delay or timeout mechanism if necessary
    }

    return;
}


//// DEVICE CODE just for reference
/*******************************************************************************
 * @fn     USB30_OUT_Set
 *
 * @brief  Endpoint receive settings
 *
 * @param  endp   - Set endpoint number
 *         status - Endpoint Status :0-NRDY,1-ACK,2-STALL
 *         nump   - The remaining number of packets that can be received by the endpoint
 *
 * @return   None
 *******************************************************************************/
void USB30_OUT_Set(UINT8 endp, UINT8 status, UINT8 nump)
{
    volatile uint32_t *uep_rx_ctrl = (volatile uint32_t *)((uint8_t *)&USBSS->UEP0_RX_CTRL + (endp * 0x10));
    *uep_rx_ctrl |= ((uint32_t)nump << 16) | ((uint32_t)status << 26);
}


/*******************************************************************************
 * @fn     USB30_IN_Set
 *
 * @brief  Endpoint transmit settings
 *
 * @param  endp   - Set endpoint number
 *         status - Endpoint Status :0-NRDY,1-ACK,2-STALL
 *         cmd    - Command for the endpoint
 *         flags  - Additional flags
 *         data   - Data field (11 bits)
 *
 * @return   None
 *******************************************************************************/
void USB30_IN_Set(UINT8 endp, UINT8 status, UINT8 cmd, UINT8 flags, uint data)
{
    volatile uint32_t *uep_tx_ctrl = (volatile uint32_t *)((uint8_t *)&USBSS->UEP0_TX_CTRL + (endp * 0x10));
    *uep_tx_ctrl |= ((uint32_t)cmd << 26) | ((uint32_t)status << 28) | ((uint32_t)flags << 16) | (data & 0x7FF);
}


