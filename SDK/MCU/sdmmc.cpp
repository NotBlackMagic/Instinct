/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    SDK/MCU/sdmmc.cpp
 */

#include "sdmmc.hpp"

SDMMC::SDMMC(SDMMC_TypeDef *instance) {
	this->instance = instance;
	this->irqPriority = 0x0E; 	//Lowest priority
}

Status SDMMC::Init(const Config &config) {
	if(config.sourceClockHz == 0) {
		return Status::Error;
	}

	if(this->isInitialized == true) {
		return Status::Ok;
	}

	// Create RTOS objects
	if(tx_mutex_create(&this->mutex, const_cast<char*>("sdmmc mutex"), TX_INHERIT) != TX_SUCCESS) {
		return Status::Error;
	}
	if(tx_event_flags_create(&this->event, const_cast<char*>("sdmmc event")) != TX_SUCCESS) {
		tx_mutex_delete(&this->mutex);
		return Status::Error;
	}

	// Enable bus clocks
	if(this->instance == SDMMC1) {
		LL_AHB5_GRP1_EnableClock(LL_AHB5_GRP1_PERIPH_SDMMC1);
		this->irqCall = SDMMC1_IRQn;
	}
	else if(this->instance == SDMMC2) {
		LL_AHB5_GRP1_EnableClock(LL_AHB5_GRP1_PERIPH_SDMMC2);
		this->irqCall = SDMMC2_IRQn;
	}
	else {
		return Status::Error;
	}
	this->sourceClockHz = config.sourceClockHz;

	// Configure RIF (enable IDMA secure region access, etc...)
	this->ConfigureRIF();

	// Configure SDMMC Interface
	uint32_t hwFlow = 0x00;
	if(config.hwFlowControl == true) {
		hwFlow = SDMMC_CLKCR_HWFC_EN;
		
	}

	// This register can only be written when the CPSM and DPSM are not active (CPSMACT = 0 and DPSMACT = 0).
	// At least seven sdmmc_hclk clock periods are needed between two write accesses to this register.
	MODIFY_REG(this->instance->CLKCR, CLKCR_CLEAR_MASK, ((0) << SDMMC_CLKCR_SELCLKRX_Pos) |		// Set receiver clock selection: internal
														0x00 |									// Set bus speed: 0 DS, HS, SDR12, SDR25
														0x00 |									// Set data rate signalling: 0 SDR mode
														hwFlow |								// Set hardware flow control
														0x00 |									// Set clock edge
														((0) << SDMMC_CLKCR_WIDBUS_Pos) |		// Set data bus width: 1-bit wide
														0x00 |									// Set power saving: 0 clock is always running
														125);									// Set clock divide factor (Fclk = Fkernel/[2*value]): Fkernel = 100 MHz, Fclk = 100/(2*125) = 400 kHz
														
	// Configure SDMMC Interrupts
	NVIC_SetPriority(this->irqCall, this->irqPriority);
	NVIC_EnableIRQ(this->irqCall);

	// Power down SDMMC (wait for card detect??)
	// MODIFY_REG(this->instance->POWER, SDMMC_POWER_PWRCTRL, 0x00);	// Power off
	MODIFY_REG(this->instance->POWER, SDMMC_POWER_PWRCTRL, SDMMC_POWER_PWRCTRL);		// Power up
	// MODIFY_REG(this->instance->POWER, SDMMC_POWER_PWRCTRL, SDMMC_POWER_PWRCTRL_1);	// Power cycle
	
	this->isInitialized = true;
	return Status::Ok;
}

Status SDMMC::DeInit(void) {
	if(this->isInitialized == false) {
		return Status::Ok;
	}

	// Disable IRQ first
	NVIC_DisableIRQ(this->irqCall);

	// Power down SDMMC
	MODIFY_REG(this->instance->POWER, SDMMC_POWER_PWRCTRL, 0x00);	// Power off

	// Force hardware reset through peripheral clock
	if(this->instance == SDMMC1) {
		LL_AHB5_GRP1_ForceReset(LL_AHB5_GRP1_PERIPH_SDMMC1);
		Time::Delay(2);
		LL_AHB5_GRP1_ReleaseReset(LL_AHB5_GRP1_PERIPH_SDMMC1);
		
		// Disable clocks
		LL_AHB5_GRP1_DisableClock(LL_AHB5_GRP1_PERIPH_SDMMC1);
	}
	else if(this->instance == SDMMC2) {
		LL_AHB5_GRP1_ForceReset(LL_AHB5_GRP1_PERIPH_SDMMC2);
		Time::Delay(2);
		LL_AHB5_GRP1_ReleaseReset(LL_AHB5_GRP1_PERIPH_SDMMC2);
		
		// Disable clocks
		LL_AHB5_GRP1_DisableClock(LL_AHB5_GRP1_PERIPH_SDMMC2);
	}
	else {
		return Status::Error;
	}

	// Abort RTOS waits/blocks and clean up RTOS Resources
	if (tx_event_flags_delete(&this->event) != TX_SUCCESS) {
		// Something went wrong...
	}

	if (tx_mutex_delete(&this->mutex) != TX_SUCCESS) {
		// Something went wrong...
	}

	this->isInitialized = false;
	return Status::Ok;
}

Status SDMMC::SetClock(uint32_t freqHz) {
	// This bit can only be written when the CPSM and DPSM are not active (CPSMACT = 0 and DPSMACT = 0).
	
	uint16_t clockDiv = 1;
	if(freqHz < this->sourceClockHz) {
		clockDiv = (this->sourceClockHz + (2 * freqHz - 1)) / (2 * freqHz);		
		// Limits check
		if(clockDiv > 0x3FFUL) {
			clockDiv = 0x3FFUL;
		}
		if(clockDiv == 0) {
			// Clock divider of 0 is allowed but DDR is not suported in that case!
			clockDiv = 1;
		}
	}

	MODIFY_REG(this->instance->CLKCR, SDMMC_CLKCR_CLKDIV, clockDiv);
	return Status::Ok;
}

Status SDMMC::SetBusWidth(BusWidth width) {
	// This bit can only be written when the CPSM and DPSM are not active (CPSMACT = 0 and DPSMACT = 0).
	MODIFY_REG(this->instance->CLKCR, SDMMC_CLKCR_WIDBUS, static_cast<uint32_t>(width));
	return Status::Ok;
}

Status SDMMC::SetSpeedMode(BusSpeed mode) {
	// This bit can only be written when the CPSM and DPSM are not active (CPSMACT = 0 and DPSMACT = 0).
	uint32_t busSpeed = 0x00;
	uint32_t busDDR = 0x00;
	switch (mode) {
		case SDMMC::BusSpeed::Default:
		case SDMMC::BusSpeed::HighSpeed:
		case SDMMC::BusSpeed::UHS_SDR12:
		case SDMMC::BusSpeed::UHS_SDR25:
			busSpeed = 0x00;	// DS, HS, SDR12, SDR25
			busDDR = 0x00;		// SDR Single data rate signaling
			break;
		case SDMMC::BusSpeed::UHS_SDR50:
			busSpeed = SDMMC_CLKCR_BUSSPEED;	// SDR50, DDR50, SDR104, HS200 
			busDDR = 0x00;
			break;
		case SDMMC::BusSpeed::UHS_DDR50:
			busSpeed = SDMMC_CLKCR_BUSSPEED;
			busDDR = SDMMC_CLKCR_DDR;	// DDR double data rate signaling
			break;
		case SDMMC::BusSpeed::UHS_SDR104:
			busSpeed = SDMMC_CLKCR_BUSSPEED;
			busDDR = 0x00;
			break;
		case SDMMC::BusSpeed::eMMC_DDR52:
			busSpeed = SDMMC_CLKCR_BUSSPEED;
			busDDR = SDMMC_CLKCR_DDR;
			break;
		case SDMMC::BusSpeed::eMMC_HS200:
			busSpeed = SDMMC_CLKCR_BUSSPEED;
			busDDR = 0x00;
			break;
		default:
			return Status::Error;
	}
	MODIFY_REG(this->instance->CLKCR, SDMMC_CLKCR_BUSSPEED | SDMMC_CLKCR_DDR, busSpeed | busDDR);

	return Status::Ok;
}

SDMMC::CommandResponse SDMMC::Command(uint8_t cmd, uint32_t args, ResponseType respType, TransferMode mode) {
	SDMMC::CommandResponse resp;
	resp.error = Error::None;

	// Clear static flags
	WRITE_REG(this->instance->ICR, SDMMC_STATIC_CMD_FLAGS);

	// Wait for CPSM  clear (command and data state machine idle)
	while(((this->instance->STA & (SDMMC_STA_CPSMACT)) == SDMMC_STA_CPSMACT));

	// Prepare data trigger logic (DPSM_ENABLE vs CMDTRANS)
	uint32_t cmdTrans = 0;
	if(mode == SDMMC::TransferMode::Manual) {
		// Manual Trigger: Enable DPSM immediately in DCTRL *before* sending command.
		// Used for SCR (ACMD51), SwitchFunc (CMD6), SDStatus (ACMD13)
		MODIFY_REG(this->instance->DCTRL, SDMMC_DPSM_ENABLE, SDMMC_DPSM_ENABLE);
	}
	else if (mode == TransferMode::Auto) {
		// Automatic/synchronized Trigger: Set CMDTRANS bit. hardware enables DPSM automatically after response.
		// Used for ReadBlocks/WriteBlocks
		cmdTrans = SDMMC_CMD_CMDTRANS;
    }
	
	// Set command arguments
	WRITE_REG(this->instance->ARG, args);

	// Set command parameters
	uint32_t cmdIndex = cmd & SDMMC_CMD_CMDINDEX;			// 0 to 64
	uint32_t responseType = static_cast<uint32_t>(respType);//SDMMC_RESPONSE_NO, SDMMC_RESPONSE_SHORT or SDMMC_RESPONSE_LONG
	uint32_t waitForInterrupt = SDMMC_WAIT_NO;				//SDMMC_WAIT_NO, SDMMC_WAIT_IT or SDMMC_WAIT_PEND
	uint32_t cpsm = SDMMC_CPSM_ENABLE;						//SDMMC_CPSM_DISABLE or SDMMC_CPSM_ENABLE

	MODIFY_REG(this->instance->CMD,	CMD_CLEAR_MASK | SDMMC_CMD_CMDTRANS, cmdIndex |
																		responseType |
																		waitForInterrupt |
																		cpsm |
																		cmdTrans);

	// Wait for command response received
	uint32_t timeoutMs = 1000;
	uint32_t timestamp = Time::GetMs();
	uint32_t status = 0;
	while(true) {
		status = this->instance->STA;

		// Check errors
		if((status & SDMMC_FLAG_CTIMEOUT) != 0x00) {
			// Clear flag
			WRITE_REG(this->instance->ICR, SDMMC_FLAG_CTIMEOUT);
			resp.error = Error::CmdTimeout;
			this->lastError = resp.error;
			return resp;
		}

		if((status & SDMMC_FLAG_CCRCFAIL) != 0x00) {
			// Clear flag
            WRITE_REG(this->instance->ICR, SDMMC_FLAG_CCRCFAIL);

			// Check if we expect CRC or not (ACMD41/CMD1 don't expect CRC so ignore)
			if(respType == SDMMC::ResponseType::Short_NoCRC) {
				// Not an error, expected
				break;
			}
			else {
				// Is an error
				resp.error = Error::CmdCrcFail;
				this->lastError = resp.error;
				return resp;
			}
		}

		// Check for timeout
		if((Time::GetMs() - timestamp) > timeoutMs) {
			resp.error = Error::CmdTimeout;
			this->lastError = resp.error;
			return resp;
		}

		// Check completion
		if(((status & (SDMMC_FLAG_CMDREND | SDMMC_FLAG_BUSYD0END)) != 0x00) || ((status & (SDMMC_FLAG_CMDACT)) == 0x00)) {
			break;
		}

		// Yield to RTOS (this can be limiting with throughput due to added latency of min. 1ms between checks)
		// tx_thread_sleep(1);
	}

	// Check command response
	if(respType == SDMMC::ResponseType::Short_CRC) {
		// Skip CMD2 and CMD3 because Response R2 (Long) does not contain the command index.
		if(cmdIndex != 2 && cmdIndex != 3) {
			if((uint8_t)(this->instance->RESPCMD) != cmdIndex) {
				// Command response not as expected.
				resp.error = Error::CmdIndexMismatch;
				this->lastError = resp.error;
				return resp;
			}
		}
	}

	// Get response
	resp.error = Error::None;
	if(respType == SDMMC::ResponseType::Short_CRC || respType == SDMMC::ResponseType::Short_NoCRC) {
		resp.resp[0] = this->instance->RESP1;
	}
	else if(respType == SDMMC::ResponseType::Long) {
		resp.resp[0] = this->instance->RESP1;
		resp.resp[1] = this->instance->RESP2;
		resp.resp[2] = this->instance->RESP3;
		resp.resp[3] = this->instance->RESP4;
	}

	// Clear static flags
	WRITE_REG(this->instance->ICR, SDMMC_STATIC_CMD_FLAGS);

	return resp;
}

Status SDMMC::TransferAsync(uint8_t *buf, uint32_t len, uint32_t blkSize, bool isRead) {
	// Wait for SDMMC mutex
	if(tx_mutex_get(&this->mutex, TIMEOUT_MUTEX) != TX_SUCCESS) {
		return Status::Timeout;
	}

	// Clear event flags
	tx_event_flags_set(&this->event, 0, TX_AND);

	// Calculate block size exponent (for SDMMC_DCTRL_DBLOCKSIZE)
	uint32_t blkSizeLog2 = 0;
	if(blkSize > 0) {
		blkSizeLog2 = 31UL - __CLZ(blkSize);
	}
	else {
		// Release SPI device
		tx_mutex_put(&this->mutex);
		return Status::Error;
	}

	// Limit to maximum allowed block size
	if(blkSizeLog2 > 14) {
		blkSizeLog2 = 14;
	}

	// 
	this->buffer = buf;
	this->length = len;
	this->dirRead = isRead;
	this->blkSize = (1UL << blkSizeLog2);

	// Handle cache coherence
	if(isRead == false) {
		System::CleanCache((uint32_t*)buf, len);
	}

	// Reset DCTRL
	WRITE_REG(this->instance->DCTRL, 0);

	// Set data timeout
	WRITE_REG(this->instance->DTIMER, 0xFFFFFFFF);	//Default to max timeout

	// Set data length
	WRITE_REG(this->instance->DLEN, len);

	// Set data configuration parameters
	// Note: If DPSM is enabled here and bytes are written only in the WaitForTransfer function then Hardware Flow Control must be enabled! Else write transfers will fail with FIFO underflow error!
	uint32_t dataBlockSize = (blkSizeLog2) << SDMMC_DCTRL_DBLOCKSIZE_Pos;		//SDMMC_DATABLOCK_SIZE_1B, SDMMC_DATABLOCK_SIZE_2B, ..., SDMMC_DATABLOCK_SIZE_16384B
	uint32_t transferDir = SDMMC_TRANSFER_DIR_TO_CARD;		//SDMMC_TRANSFER_DIR_TO_CARD or SDMMC_TRANSFER_DIR_TO_SDMMC
	uint32_t transferMode = SDMMC_TRANSFER_MODE_BLOCK;		//SDMMC_TRANSFER_MODE_BLOCK, SDMMC_TRANSFER_MODE_SDIO or SDMMC_TRANSFER_MODE_STREAM
	uint32_t dpsm = SDMMC_DPSM_DISABLE;						//Data path is enabled by the subsequent Command call

	if(isRead == true) {
		transferDir = SDMMC_TRANSFER_DIR_TO_SDMMC;	//From Card to MCU
	}

	// Clear all the static flags
	MODIFY_REG(this->instance->ICR, SDMMC_STATIC_DATA_FLAGS, SDMMC_STATIC_DATA_FLAGS);

	MODIFY_REG(this->instance->DCTRL, DCTRL_CLEAR_MASK, dataBlockSize |
														transferDir |
														transferMode |
														dpsm);

	// Prepare IDMA (DMA proprietary of the SDMMC peripheral)
	WRITE_REG(this->instance->IDMABASER, (uint32_t)buf);
	MODIFY_REG(this->instance->IDMACTRL, SDMMC_IDMA_IDMAEN, SDMMC_IDMA_IDMAEN);	

	// Enable IRQs (required for efficient IDMA use)
	SET_BIT(this->instance->MASK, SDMMC_MASK_DCRCFAILIE);
	SET_BIT(this->instance->MASK, SDMMC_MASK_DTIMEOUTIE);
	if(isRead == true) {
		SET_BIT(this->instance->MASK, SDMMC_MASK_RXOVERRIE);
	}
	else {
		SET_BIT(this->instance->MASK, SDMMC_MASK_TXUNDERRIE);
	}
	SET_BIT(this->instance->MASK, SDMMC_MASK_DATAENDIE);

	return Status::Ok;
}

Status SDMMC::TransferWait(uint32_t timeoutTicks) {
	// Wait for event
	ULONG events;
	UINT status = tx_event_flags_get(&this->event, EVT_DATA_CPLT | EVT_DATA_ERR, TX_OR_CLEAR, &events, timeoutTicks);

	bool result = true;
	this->lastError = Error::None;

	if(status == TX_DELETED || status == TX_WAIT_ABORTED) {
		// The driver was killed/DeInit while waiting on flags, do not access registers
		this->lastError = Error::Aborted;
		return Status::Error;
	}

	if(status == TX_SUCCESS) {
		if((events & 0x02) != 0) {
			// Error occurred, check error reason
			if((this->irqStatus & SDMMC_STA_DCRCFAIL) == SDMMC_STA_DCRCFAIL) {
				this->lastError = Error::DataCrcFail;
			}
			else if((this->irqStatus & SDMMC_STA_DTIMEOUT) == SDMMC_STA_DTIMEOUT) {
				this->lastError = Error::DataTimeout;
			}
			else if((this->irqStatus & SDMMC_STA_TXUNDERR) == SDMMC_STA_TXUNDERR) {
				this->lastError = Error::TxUnderrun;
			}
			else if((this->irqStatus & SDMMC_STA_RXOVERR) == SDMMC_STA_RXOVERR) {
				this->lastError = Error::RxOverrun;
			}
			else {
				this->lastError = Error::Hardware;
			}
			result = false;
		}
	}
	else {
		// OS Timeout
		this->lastError = Error::DataTimeout;
		result = false;
	}

	if(result == false) {
		// Error condition, cleanup
		MODIFY_REG(this->instance->IDMACTRL, SDMMC_IDMA_IDMAEN, 0);
		MODIFY_REG(this->instance->DCTRL, SDMMC_DCTRL_DTEN, 0);
		MODIFY_REG(this->instance->DCTRL, SDMMC_DCTRL_FIFORST, SDMMC_DCTRL_FIFORST);
		MODIFY_REG(this->instance->ICR, SDMMC_STATIC_DATA_FLAGS, SDMMC_STATIC_DATA_FLAGS);
	}
	else {
		// Handle cache coherency
		if(this->dirRead == true) {
			System::InvalidateCache((uint32_t*)this->buffer, this->length);
		}
	}

	// Release SPI device
	tx_mutex_put(&this->mutex);

	return Status::Ok;
}

Status SDMMC::SwitchTo1V8(void (*callback)(void*), void* context) {
	if(callback == nullptr) {
		return Status::Error;
	}
	
	// Enable voltage switching capability
	MODIFY_REG(this->instance->POWER, SDMMC_POWER_VSWITCHEN, SDMMC_POWER_VSWITCHEN);

	// Send voltage switch command
	CommandResponse resp;
	resp = this->Command(11, 0, ResponseType::Short_CRC);
	if(resp.error != Error::None) {
		// Disable VSWITCHEN on error
        MODIFY_REG(this->instance->POWER, SDMMC_POWER_VSWITCHEN, 0);
        return Status::Error;
	}

	// Wait for CKSTOP
	uint32_t timeoutMs = 100;
	uint32_t timestamp = Time::GetMs();
	while(true) {
		if((this->instance->STA & SDMMC_FLAG_CKSTOP) == SDMMC_FLAG_CKSTOP) {
			break;
		}

		if((Time::GetMs() - timestamp) > timeoutMs) {
			// Clock stop timeout
			return Status::Error;
		}
	}

	// Clear CKSTOP Flag
	MODIFY_REG(this->instance->ICR, SDMMC_FLAG_CKSTOP, SDMMC_FLAG_CKSTOP);

	// Verify card busy (D0 line low)
	if((this->instance->STA & SDMMC_FLAG_BUSYD0) != SDMMC_FLAG_BUSYD0) {
		return Status::Error;
	}

	// Perform external switch (change IO voltage level)
	callback(context);

	// Required switch of VDDIO setting for this SDMMC peripheral
	if(this->instance == SDMMC1) {
		// LL_PWR_EnableVddIO4();		//Enable VDD IO4 (PC[1], PC[12:6], and PH[9:2]) voltage supply
		// LL_PWR_SetVddIO4VoltageRange(LL_PWR_VDDIO_VOLTAGE_RANGE_3V3);		//Set the VDD IO4 voltage range: 3.3V
		// // while (LL_PWR_IsActiveFlag_VDDIO4RDY() == 0U);
	}
	else if(this->instance == SDMMC2) {
		// LL_PWR_EnableVddIO5();		//Enable VDD IO5 (PC[0], PC[5:2], and PE[4]) voltage supply
		// LL_PWR_SetVddIO5VoltageRange(LL_PWR_VDDIO_VOLTAGE_RANGE_3V3);		//Set the VDD IO5 voltage range: 3.3V
		// // while (LL_PWR_IsActiveFlag_VDDIO5RDY() == 0U);
	}
	else {
		return Status::Error;
	}

	// Switch is ready
	MODIFY_REG(this->instance->POWER, SDMMC_POWER_VSWITCH, SDMMC_POWER_VSWITCH);

	// Wait for VSWEND (voltage switch end flag)
	timestamp = Time::GetMs();
	while(true) {
		if((this->instance->STA & SDMMC_FLAG_VSWEND) == SDMMC_FLAG_VSWEND) {
			break;
		}

		if((Time::GetMs() - timestamp) > timeoutMs) {
			// Clock stop timeout
			return Status::Error;
		}
	}

	// Clear VSWEND flag
	MODIFY_REG(this->instance->ICR, SDMMC_FLAG_VSWEND, SDMMC_FLAG_VSWEND);

	// Verify card ready (D0 line high)
	if((this->instance->STA & SDMMC_FLAG_BUSYD0) == SDMMC_FLAG_BUSYD0) {
		return Status::Error;
	}

	// Clear voltage switch flags ()
	MODIFY_REG(this->instance->POWER, SDMMC_POWER_VSWITCHEN | SDMMC_POWER_VSWITCH, 0x00);

	// Clear all the static flags
	MODIFY_REG(this->instance->ICR, SDMMC_STATIC_DATA_FLAGS, SDMMC_STATIC_DATA_FLAGS);

	return Status::Ok;
}

Status SDMMC::WaitBusyD0(uint32_t timeoutMs) {
	uint32_t timestamp = Time::GetMs();
	while((this->instance->STA & SDMMC_FLAG_BUSYD0) == SDMMC_FLAG_BUSYD0) {
		if((Time::GetMs() - timestamp) > timeoutMs) {
			return Status::Timeout;
		}
	}
	return Status::Ok;
}

void SDMMC::ConfigureRIF(void) {
	// Essential stuff so that IDMA has access to SRAM regions!!
	// Without this the IDMA can't access the SRAM, the transfer would not fail but would write 0 and read to nullptr
	// https://github.com/STMicroelectronics/STM32CubeN6/blob/main/Projects/STM32N6570-DK/Examples/SD/SD_ReadWrite_DMA/FSBL/Src/main.c

	LL_AHB3_GRP1_EnableClock(LL_AHB3_GRP1_PERIPH_RIFSC);

	// Some RIF constants (from HAL)
	const uint32_t RIF_PERIPH_REG1 = 0x10000000U;
	const uint32_t RIF_CID_1 = 0x00000002U;
	const uint32_t RIF_PERIPH_REG_SHIFT = 28U;
	const uint32_t RIF_PERIPH_BIT_POSITION = 0x0000001FU;

	uint32_t masterID = 0;
	uint32_t periphID = 0;
	if(this->instance == SDMMC1) {
		masterID = 2U;		// RIMU (Refernce Manual 6.3.4 Table 22 pg. 248)
		periphID = (RIF_PERIPH_REG1 | RIFSC_RISC_SECCFGRx_SEC21_Pos);
	}
	else if(this->instance == SDMMC2) {
		masterID = 3U;		// RIMU (Refernce Manual 6.3.4 Table 22 pg. 248)
		periphID = (RIF_PERIPH_REG1 | RIFSC_RISC_SECCFGRx_SEC22_Pos);
	}
	else {
		return;
	}

	// RIMC_ATTRx: Controls if IDMA can read/write Secure/Privileged memory: Set to this master is secure and privilaged
	uint32_t masterCID = POSITION_VAL(RIF_CID_1);
	uint32_t wMask = (RIFSC_RIMC_ATTRx_MCID | RIFSC_RIMC_ATTRx_MPRIV | RIFSC_RIMC_ATTRx_MSEC);
	uint32_t wValue = ((masterCID << RIFSC_RIMC_ATTRx_MCID_Pos) | (0x03 << RIFSC_RIMC_ATTRx_MSEC_Pos));		//Bit 0: Master Secure; Bit 1: Master priviliged
	MODIFY_REG(RIFSC->RIMC_ATTRx[masterID], wMask, wValue);

	// Allows CPU to access SDMMC registers in Secure/Privileged mode.
	// Slave security configuration register: 0: Secure and nonsecure data access are granted to the peripheral; 1: Secure data access only are granted to the peripheral
	wMask = (1UL << (periphID & RIF_PERIPH_BIT_POSITION));
	wValue = ((0x01) << (periphID & RIF_PERIPH_BIT_POSITION));		//0: Secure and nonsecure data access are granted; 1: Only secure access granted
	MODIFY_REG(RIFSC->RISC_SECCFGRx[periphID >> RIF_PERIPH_REG_SHIFT], wMask, wValue);

	// Slave privileged configuration register: 0: Privileged and unprivileged data access are granted to the peripheral; 1: Privileged data access only are granted to the peripheral
	wMask = (1UL << (periphID & RIF_PERIPH_BIT_POSITION));
	wValue = (0x01) << (periphID & RIF_PERIPH_BIT_POSITION);		//0: Secure and nonsecure data access are granted; 1: Only secure access granted
	MODIFY_REG(RIFSC->RISC_PRIVCFGRx[periphID >> RIF_PERIPH_REG_SHIFT], wMask, wValue);
}

// ---------------------------------------------------------
// IRQ Handler
// ---------------------------------------------------------

void SDMMC::InterruptHandler(void) {
	this->irqStatus = this->instance->STA;
	uint32_t mask = this->instance->MASK;

	// Handle data end (DATAEND)
	if(((this->irqStatus & SDMMC_STA_DATAEND) == SDMMC_STA_DATAEND) && ((mask & SDMMC_MASK_DATAENDIE) == SDMMC_MASK_DATAENDIE)) {
		// Write or read complete
		// Disable DMA and data path
		WRITE_REG(this->instance->DLEN, 0);
		WRITE_REG(this->instance->DCTRL, 0);
		MODIFY_REG(this->instance->IDMACTRL, SDMMC_IDMA_IDMAEN, 0);
		
		// Clear flag
		MODIFY_REG(this->instance->ICR, SDMMC_STA_DATAEND, SDMMC_STA_DATAEND);

		// Disable interrupts
		MODIFY_REG(this->instance->MASK, SDMMC_MASK_DATAENDIE | SDMMC_MASK_DCRCFAILIE | SDMMC_MASK_DTIMEOUTIE | SDMMC_MASK_TXUNDERRIE | SDMMC_MASK_RXOVERRIE, 0);

		tx_event_flags_set(&this->event, EVT_DATA_CPLT, TX_OR);
	}

	// Handle data CRC failed, data timeout, rx overrun, tx underrun (DCRCFAIL, DTIMEOUT, RXOVERR, TXUNDERR)
	if(	(((this->irqStatus & SDMMC_STA_DCRCFAIL) == SDMMC_STA_DCRCFAIL) && ((mask & SDMMC_MASK_DCRCFAILIE) == SDMMC_MASK_DCRCFAILIE)) ||
		(((this->irqStatus & SDMMC_STA_DTIMEOUT) == SDMMC_STA_DTIMEOUT) && ((mask & SDMMC_MASK_DTIMEOUTIE) == SDMMC_MASK_DTIMEOUTIE)) ||
		(((this->irqStatus & SDMMC_STA_RXOVERR) == SDMMC_STA_RXOVERR) && ((mask & SDMMC_MASK_RXOVERRIE) == SDMMC_MASK_RXOVERRIE)) ||
		(((this->irqStatus & SDMMC_STA_TXUNDERR) == SDMMC_STA_TXUNDERR) && ((mask & SDMMC_MASK_TXUNDERRIE) == SDMMC_MASK_TXUNDERRIE))) {
		// Disable DMA and data path
		MODIFY_REG(this->instance->DCTRL, SDMMC_DCTRL_FIFORST, 0);
		MODIFY_REG(this->instance->IDMACTRL, SDMMC_IDMA_IDMAEN, 0);

		// Clear all the static flags
		MODIFY_REG(this->instance->ICR, SDMMC_STATIC_DATA_FLAGS, SDMMC_STATIC_DATA_FLAGS);

		// Disable interrupts
		MODIFY_REG(this->instance->MASK, SDMMC_MASK_DATAENDIE | SDMMC_MASK_DCRCFAILIE | SDMMC_MASK_DTIMEOUTIE | SDMMC_MASK_TXUNDERRIE | SDMMC_MASK_RXOVERRIE | SDMMC_STA_IDMATE, 0);

		tx_event_flags_set(&this->event, EVT_DATA_ERR, TX_OR);
	}
}