/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (PlumaLabs)
 *
 * File:    SDK/MCU/dcmipp.cpp
 */

#include "dcmipp.hpp"

Dcmipp::Dcmipp(DCMIPP_TypeDef *instance) : instance(instance) {
	this->isInitialized = false;
	this->irqPriority = 0x0E; // Lowest priority (safe default)
}

Status Dcmipp::Init() {
	if(this->isInitialized == true) {
		return Status::Ok;
	}

	// Create RTOS objects
	if(tx_event_flags_create(&this->event, const_cast<char*>("dcmipp event")) != TX_SUCCESS) {
		return Status::Error;
	}

	// Enable bus clocks
	if(this->instance == DCMIPP) {
		LL_APB5_GRP1_EnableClock(LL_APB5_GRP1_PERIPH_DCMIPP);
		this->irqCall = DCMIPP_IRQn;
	}

	// Configure RIF (enable IDMA secure region access, etc...)
	this->ConfigureRIF();

	// Configure the AXI IP-Plug to allow the DCMIPP to write to memory
	// Set IP-Plug to IDLE, to allow reconfiguration
	SET_BIT(this->instance->IPGR2, DCMIPP_IPGR2_PSTART);
	uint32_t timeout = 10000;
	while ((this->instance->IPGR3 & DCMIPP_IPGR3_IDLE) != DCMIPP_IPGR3_IDLE) {
		timeout = timeout - 1;
		if(timeout == 0) {
			return Status::Timeout;
		}
	}

	// Set Memory Page Size (Default 2: 256 bytes page boundary): 2^VAL * 64-bytes
	WRITE_REG(this->instance->IPGR1, 0x02);

	// Bandwidth allocation is set by:
	// Max outstanding as shared of combined 16 (IPCxR1.OTR)
	// Bandwith ratio as shared of combined SUM of all IPCxR2.WLRU values (IPCxR2.WLRU)

	// Available total FIFO: 640x64 bit (5120 bytes), i.e. 640 lines of 64-bit words
	uint32_t fifoSize = 128;	// Each client gets a unform FIFO slice of 1024 bytes
	uint32_t fifoAddStart = 0;
	uint32_t fifoAddrEnd = (fifoSize - 1);

	// IP-PLUG Client 1 (Pipe 0) Configuration

	// IP-PLUG Client 2 (Pipe 1, Y/RGB component) Configuration
	// Max Outstanding Transactions (e.g., 4) and Burst size (default: 4, 128 bytes/burst)
	WRITE_REG(this->instance->IPC1R1, (0x04 << DCMIPP_IPC1R1_OTR_Pos) | (0x04 << DCMIPP_IPC1R1_TRAFFIC_Pos));
	// Set ratio of total bandwith, arbitration between clients
	WRITE_REG(this->instance->IPC1R2, (0U << DCMIPP_IPC1R2_WLRU_Pos));
	// Set FIFO End/Start address (64-bit word address)
	WRITE_REG(this->instance->IPC1R3, (fifoAddStart << DCMIPP_IPC1R3_DPREGSTART_Pos) | (fifoAddrEnd << DCMIPP_IPC1R3_DPREGEND_Pos));
	fifoAddStart = fifoAddrEnd + 1;
	fifoAddrEnd += fifoSize;

	// IP-PLUG Client 3 (Pipe 1, U component) Configuration

	// IP-PLUG Client 4 (Pipe 1, V component) Configuration

	// IP-PLUG Client 5 (Pipe 2) Configuration

	// Release lock, IP-Plug runs on demand by background HW
	CLEAR_BIT(this->instance->IPGR2, DCMIPP_IPGR2_PSTART);

	// Configure Interrupts
	NVIC_SetPriority(this->irqCall, this->irqPriority);
	NVIC_EnableIRQ(this->irqCall);
	MODIFY_REG(this->instance->CMIER, 0x00, DCMIPP_CMIER_ATXERRIE);		// Enable IPPLUG AXI Transfer error interrupt

	isInitialized = true;
	return Status::Ok;
}

Status Dcmipp::LinkCSIVirtualChannel(PipeID pipe, Csi::VirtualChannel vc, Csi::MIPIDataType dataType) {
	uint32_t dtVal = static_cast<uint32_t>(dataType);
	uint32_t vcVal = static_cast<uint32_t>(vc);

	if(pipe == PipeID::Dump) {
		// Set Virtual Channel
		MODIFY_REG(this->instance->P0FSCR, DCMIPP_P0FSCR_VC_Msk, (vcVal << DCMIPP_P0FSCR_VC_Pos));

		// Filter specific Data Type Mode: Only data matching DTIDA
		// 0x0: Only flow DTIDA from the selected virtual channel is forwarded in the pipe
		// 0x1: Flows DTIDA and/or DTIDB from the selected virtual channel are forwarded in the pipe
		// 0x2: All data types from the selected virtual channel (except the DTIDA or DTIDB) are forwarded in the pipe
		// 0x3: All data types of the selected virtual channel VC are forwarded in the pipe
		MODIFY_REG(this->instance->P0FSCR, DCMIPP_P0FSCR_DTMODE_Msk, (0x00 << DCMIPP_P0FSCR_DTMODE_Pos));

		// Data Type Selection IDA
		MODIFY_REG(this->instance->P0FSCR, DCMIPP_P0FSCR_DTIDA_Msk, (dtVal << DCMIPP_P0FSCR_DTIDA_Pos));
	}
	else if(pipe == PipeID::Main) {
		// Set Virtual Channel
		MODIFY_REG(this->instance->P1FSCR, DCMIPP_P1FSCR_VC_Msk, (vcVal << DCMIPP_P1FSCR_VC_Pos));

		// Filter specific Data Type Mode: Only data matching DTIDA
		// 0x0: Only flow DTIDA from the selected virtual channel is forwarded in the pipe
		// 0x1: Flows DTIDA and/or DTIDB from the selected virtual channel are forwarded in the pipe
		MODIFY_REG(this->instance->P1FSCR, DCMIPP_P1FSCR_DTMODE_Msk, (0x00 << DCMIPP_P1FSCR_DTMODE_Pos));

		// Pipe 2 filtering: Set independent of pipe 1
		MODIFY_REG(this->instance->P1FSCR, DCMIPP_P1FSCR_PIPEDIFF, DCMIPP_P1FSCR_PIPEDIFF);
		
		// Data Type Selection IDA
		MODIFY_REG(this->instance->P1FSCR, DCMIPP_P1FSCR_DTIDA_Msk, (dtVal << DCMIPP_P1FSCR_DTIDA_Pos));		
	} 
	else if(pipe == PipeID::Auxiliary) {
		// Those bit fields are meaningful when PIPEDIFF = 1: Pipe1, Pipe2 is fully independent
		if((this->instance->P1FSCR & DCMIPP_P1FSCR_PIPEDIFF) == DCMIPP_P1FSCR_PIPEDIFF) {
			// Set Virtual Channel
			MODIFY_REG(this->instance->P2FSCR, DCMIPP_P2FSCR_VC, (vcVal << DCMIPP_P2FSCR_VC_Pos));

			// Data Type Selection IDA
			MODIFY_REG(this->instance->P2FSCR, DCMIPP_P2FSCR_DTIDA_Msk, (dtVal << DCMIPP_P2FSCR_DTIDA_Pos));
		}
	}
	else {
		return Status::Error;
	}

	// Disable Parallel interface
	CLEAR_BIT(this->instance->PRCR, DCMIPP_PRCR_ENABLE);

	// Set DCMIPP Input Selection: Input from CSI
	SET_BIT(this->instance->CMCR, DCMIPP_CMCR_INSEL);

	// Set DCMIPP Frame Counter: Pipe 1 (main)
	MODIFY_REG(this->instance->CMCR, DCMIPP_CMCR_PSFC_Msk, (0x01 << DCMIPP_CMCR_PSFC_Pos));
	
	// Clear frame counter
	SET_BIT(this->instance->CMCR, DCMIPP_CMCR_CFC);

	return Status::Ok;
}

Status Dcmipp::ConfigurePipe(PipeID pipe, const PipeConfig &config) {
	if(pipe == PipeID::Dump) {
		// Configure Frame Rate Decimation (no decimation)
		MODIFY_REG(this->instance->P0FCTCR, DCMIPP_P0FCTCR_FRATE_Msk, 0x00);

		// Configure Pixel Packer Format. Pipe 0 only supports swapping
		if(config.swapRBUV == true) {
			// Swaps the bytes from provided words, byte 0-vs.-1 and 2-vs.-3: This is, swap R-vs-B or U-vs-V
			MODIFY_REG(this->instance->P0PPCR, DCMIPP_P0PPCR_SWAPYUV_Msk, DCMIPP_P0PPCR_SWAPYUV);
		}
		else {
			MODIFY_REG(this->instance->P0PPCR, DCMIPP_P0PPCR_SWAPYUV_Msk, 0x00);
		}
	}
	else if(pipe == PipeID::Main) {
		// Configure Frame Rate Decimation (no decimation)
		MODIFY_REG(this->instance->P1FCTCR, DCMIPP_P1FCTCR_FRATE_Msk, 0x00);

		// Configure Pixel Packer Format
		MODIFY_REG(this->instance->P1PPCR, DCMIPP_P1PPCR_FORMAT_Msk, (static_cast<uint32_t>(config.format) << DCMIPP_P1PPCR_FORMAT_Pos));
		if(config.swapRBUV == true) {
			// Swap R-vs-B or U-vs-V
			MODIFY_REG(this->instance->P1PPCR, DCMIPP_P1PPCR_SWAPRB_Msk, DCMIPP_P1PPCR_SWAPRB);
		}
		else {
			MODIFY_REG(this->instance->P1PPCR, DCMIPP_P1PPCR_SWAPRB_Msk, 0x00);
		}

		// Configure Pixel Pipe Pitch (Memory Stride)
		MODIFY_REG(this->instance->P1PPM0PR, DCMIPP_P1PPM0PR_PITCH_Msk, (config.pixelPitch << DCMIPP_P1PPM0PR_PITCH_Pos));

		// If using YUV modes that require UV pitch configuration
		if(config.format == OutputFormat::YUV422 || config.format == OutputFormat::YUV420_NV21) {
			MODIFY_REG(this->instance->P1PPM1PR, DCMIPP_P1PPM1PR_PITCH_Msk, (config.pixelPitch << DCMIPP_P1PPM1PR_PITCH_Pos));
		}
		else if(config.format == OutputFormat::YUV420_YV12) {
			MODIFY_REG(this->instance->P1PPM1PR, DCMIPP_P1PPM1PR_PITCH_Msk, ((config.pixelPitch / 2) << DCMIPP_P1PPM1PR_PITCH_Pos));
		}
	} 
	else if(pipe == PipeID::Auxiliary) {
		// Configure Frame Rate Decimation (no decimation)
		MODIFY_REG(this->instance->P2FCTCR, DCMIPP_P2FCTCR_FRATE_Msk, 0x00);

		// Configure Pixel Packer Format
		MODIFY_REG(this->instance->P2PPCR, DCMIPP_P2PPCR_FORMAT_Msk, (static_cast<uint32_t>(config.format) << DCMIPP_P2PPCR_FORMAT_Pos));
		if(config.swapRBUV == true) {
			// Swap R-vs-B or U-vs-V
			MODIFY_REG(this->instance->P2PPCR, DCMIPP_P2PPCR_SWAPRB_Msk, DCMIPP_P2PPCR_SWAPRB);
		}
		else {
			MODIFY_REG(this->instance->P2PPCR, DCMIPP_P2PPCR_SWAPRB_Msk, 0x00);
		}

		// Configure Pixel Pipe Pitch (Memory Stride)
		MODIFY_REG(this->instance->P2PPM0PR, DCMIPP_P2PPM0PR_PITCH_Msk, (config.pixelPitch << DCMIPP_P2PPM0PR_PITCH_Pos));
	}
	else {
		return Status::Error;
	}

	return Status::Ok;
}

Status Dcmipp::CaptureAsync(PipeID pipe, const MemoryDestination &dest, CaptureMode mode) {
	if(pipe == PipeID::Dump) {
		// Ensure pipe is not active
		if(this->instance->P0FSCR & DCMIPP_P0FSCR_PIPEN) {
			return Status::Error;
		}

		// Primary Address (Y Plane or Interleaved)
		WRITE_REG(this->instance->P0PPM0AR1, dest.primaryAddress);

		// Handle Double Buffering
		if(dest.isDoubleBuffered) {
			WRITE_REG(this->instance->P0PPM0AR2, dest.secondaryAddress);
			SET_BIT(this->instance->P0PPCR, DCMIPP_P0PPCR_DBM);
		}
		else {
			CLEAR_BIT(this->instance->P0PPCR, DCMIPP_P0PPCR_DBM);
		}

		// Set Capture Mode (Snapshot / Continuous)
		if(mode == Dcmipp::CaptureMode::Continuous) {
			MODIFY_REG(this->instance->P0FCTCR, DCMIPP_P0FCTCR_CPTMODE_Msk, 0x00);
		}
		else {
			MODIFY_REG(this->instance->P0FCTCR, DCMIPP_P0FCTCR_CPTMODE_Msk, DCMIPP_P0FCTCR_CPTMODE);
		}

		// Enable Interrupts
		MODIFY_REG(this->instance->CMIER, 0x00, DCMIPP_CMIER_P0FRAMEIE | DCMIPP_CMIER_P0VSYNCIE | DCMIPP_CMIER_P0OVRIE);

		// Enable the Pipe
		SET_BIT(this->instance->P0FSCR, DCMIPP_P0FSCR_PIPEN);

		// Request Capture Start
		SET_BIT(this->instance->P0FCTCR, DCMIPP_P0FCTCR_CPTREQ);
	}
	else if(pipe == PipeID::Main) {
		// Ensure pipe is not active
		if((this->instance->P1FSCR & DCMIPP_P1FSCR_PIPEN) == DCMIPP_P1FSCR_PIPEN) {
			return Status::Error;
		}

		// Primary Address (Y Plane or Interleaved)
		WRITE_REG(this->instance->P1PPM0AR1, dest.primaryAddress);

		// Handle Double Buffering
		if(dest.isDoubleBuffered) {
			WRITE_REG(this->instance->P1PPM0AR2, dest.secondaryAddress);
			SET_BIT(this->instance->P1PPCR, DCMIPP_P1PPCR_DBM);
		}
		else {
			CLEAR_BIT(this->instance->P1PPCR, DCMIPP_P1PPCR_DBM);
		}

		// Handle Semi/Full Planar UV Addresses
		if(dest.isSemiPlanar || dest.isFullPlanar) {
			WRITE_REG(this->instance->P1PPM1AR1, dest.uAddress);
			// Note: If Double Buffered + SemiPlanar, P1PPM1AR2 needs configuration here too
		}
		
		if(dest.isFullPlanar) {
			WRITE_REG(this->instance->P1PPM2AR1, dest.vAddress);
		}

		// Set Capture Mode (Snapshot / Continuous)
		if(mode == Dcmipp::CaptureMode::Continuous) {
			MODIFY_REG(this->instance->P1FCTCR, DCMIPP_P1FCTCR_CPTMODE_Msk, 0x00);
		}
		else {
			MODIFY_REG(this->instance->P1FCTCR, DCMIPP_P1FCTCR_CPTMODE_Msk, DCMIPP_P1FCTCR_CPTMODE);
		}

		// Enable Interrupts
		MODIFY_REG(this->instance->CMIER, 0x00, DCMIPP_CMIER_P1FRAMEIE | DCMIPP_CMIER_P1VSYNCIE | DCMIPP_CMIER_P1OVRIE);

		// Enable the Pipe
		SET_BIT(this->instance->P1FSCR, DCMIPP_P1FSCR_PIPEN);

		// Request Capture Start
		SET_BIT(this->instance->P1FCTCR, DCMIPP_P1FCTCR_CPTREQ);
	} 
	else if(pipe == PipeID::Auxiliary) {
		// Ensure pipe is not active
		if(this->instance->P2FSCR & DCMIPP_P2FSCR_PIPEN) {
			return Status::Error;
		}

		// Primary Address (Y Plane or Interleaved)
		WRITE_REG(this->instance->P2PPM0AR1, dest.primaryAddress);

		// Handle Double Buffering
		if(dest.isDoubleBuffered) {
			WRITE_REG(this->instance->P2PPM0AR2, dest.secondaryAddress);
			SET_BIT(this->instance->P2PPCR, DCMIPP_P2PPCR_DBM);
		}
		else {
			CLEAR_BIT(this->instance->P2PPCR, DCMIPP_P2PPCR_DBM);
		}

		// Set Capture Mode (Snapshot / Continuous)
		if(mode == Dcmipp::CaptureMode::Continuous) {
			MODIFY_REG(this->instance->P2FCTCR, DCMIPP_P2FCTCR_CPTMODE_Msk, 0x00);
		}
		else {
			MODIFY_REG(this->instance->P2FCTCR, DCMIPP_P2FCTCR_CPTMODE_Msk, DCMIPP_P2FCTCR_CPTMODE);
		}

		// Enable Interrupts
		MODIFY_REG(this->instance->CMIER, 0x00, DCMIPP_CMIER_P2FRAMEIE | DCMIPP_CMIER_P2VSYNCIE | DCMIPP_CMIER_P2OVRIE);

		// Enable the Pipe
		SET_BIT(this->instance->P2FSCR, DCMIPP_P2FSCR_PIPEN);

		// Request Capture Start
		SET_BIT(this->instance->P2FCTCR, DCMIPP_P2FCTCR_CPTREQ);
	}
	else {
		return Status::Error;
	}

	return Status::Ok;
}

Status Dcmipp::CaptureAbort(PipeID pipe) {
	uint32_t timeout = 10000;

	if(pipe == PipeID::Dump) {
		// Stop the capture
		CLEAR_BIT(this->instance->P0FCTCR, DCMIPP_P0FCTCR_CPTREQ);

		// Wait until capture is no longer active
		while((this->instance->CMSR1 & DCMIPP_CMSR1_P0CPTACT) != 0U) {
			timeout = timeout - 1;
			if(timeout == 0) {
				return Status::Timeout;
			}
		}

		// Disable Double Buffering to reset state
		CLEAR_BIT(this->instance->P0PPCR, DCMIPP_P0PPCR_DBM);

		// Disable Pipe Processing
		CLEAR_BIT(this->instance->P0FSCR, DCMIPP_P0FSCR_PIPEN);

		// Disable Interrupts
		MODIFY_REG(this->instance->CMIER, DCMIPP_CMIER_P0LINEIE | DCMIPP_CMIER_P0FRAMEIE | DCMIPP_CMIER_P0VSYNCIE | DCMIPP_CMIER_P0LIMITIE | DCMIPP_CMIER_P0OVRIE, 0x00);
	}
	else if(pipe == PipeID::Main) {
		// Stop the capture
		CLEAR_BIT(this->instance->P1FCTCR, DCMIPP_P1FCTCR_CPTREQ);

		// Wait until capture is no longer active
		while((this->instance->CMSR1 & DCMIPP_CMSR1_P1CPTACT) != 0U) {
			timeout = timeout - 1;
			if(timeout == 0) {
				return Status::Timeout;
			}
		}

		// Disable Double Buffering to reset state
		CLEAR_BIT(this->instance->P1PPCR, DCMIPP_P1PPCR_DBM);

		// Disable Pipe Processing
		CLEAR_BIT(this->instance->P1FSCR, DCMIPP_P1FSCR_PIPEN);

		// Disable Interrupts
		MODIFY_REG(this->instance->CMIER, DCMIPP_CMIER_P1LINEIE | DCMIPP_CMIER_P1FRAMEIE | DCMIPP_CMIER_P1VSYNCIE | DCMIPP_CMIER_P1OVRIE, 0x00);
	}
	else if(pipe == PipeID::Auxiliary) {
		// Stop the capture
		CLEAR_BIT(this->instance->P2FCTCR, DCMIPP_P2FCTCR_CPTREQ);

		// Wait until capture is no longer active
		while((this->instance->CMSR1 & DCMIPP_CMSR1_P2CPTACT) != 0U) {
			timeout = timeout - 1;
			if(timeout == 0) {
				return Status::Timeout;
			}
		}

		// Disable Double Buffering to reset state
		CLEAR_BIT(this->instance->P2PPCR, DCMIPP_P2PPCR_DBM);

		// Disable Pipe Processing
		CLEAR_BIT(this->instance->P2FSCR, DCMIPP_P2FSCR_PIPEN);

		// Disable Interrupts
		MODIFY_REG(this->instance->CMIER, DCMIPP_CMIER_P2LINEIE | DCMIPP_CMIER_P2FRAMEIE | DCMIPP_CMIER_P2VSYNCIE | DCMIPP_CMIER_P2OVRIE, 0x00);
	}
	else {
		return Status::Error;
	}

	return Status::Ok;
}

Status Dcmipp::CaptureWait(PipeID pipe, uint32_t timeoutTicks) {
	// Wait for event
	ULONG events;
	UINT status = tx_event_flags_get(&this->event, EVT_TRANS_CPLT | EVT_ERR, TX_OR_CLEAR, &events, timeoutTicks);

	if(status != TX_SUCCESS) {
		this->CaptureAbort(pipe);
		return Status::Timeout;
	}

	if((events & EVT_ERR) == EVT_ERR) {
		// Transfer error occurred
		this->CaptureAbort(pipe);
		return Status::Error;
	}

	return Status::Ok;
}

void Dcmipp::ConfigureRIF(void) {
	// Essential stuff so that IDMA has access to SRAM regions!!
	// Without this the IDMA can't access the SRAM, the transfer would not fail but would write 0 and read to nullptr
	// https://github.com/STMicroelectronics/STM32CubeN6/blob/main/Projects/STM32N6570-DK/Examples/SD/SD_ReadWrite_DMA/FSBL/Src/main.c

	LL_AHB3_GRP1_EnableClock(LL_AHB3_GRP1_PERIPH_RIFSC);

	// Some RIF constants (from HAL)
	const uint32_t RIF_PERIPH_REG2 = 0x20000000U;
	const uint32_t RIF_CID_1 = 0x00000002U;
	const uint32_t RIF_PERIPH_REG_SHIFT = 28U;
	const uint32_t RIF_PERIPH_BIT_POSITION = 0x0000001FU;

	uint32_t masterID = 0;
	uint32_t periphID = 0;
	if(this->instance == DCMIPP) {
		masterID = 9U;		// RIMU (Refernce Manual 6.3.4 Table 22 pg. 248)
		periphID = (RIF_PERIPH_REG2 | RIFSC_RISC_SECCFGRx_SEC29_Pos);
	}
	else {
		return;
	}

	// RIMC_ATTRx: Controls if IDMA can read/write Secure/Privileged memory: Set to this master is secure and privilaged
	uint32_t masterCID = POSITION_VAL(RIF_CID_1);
	uint32_t wMask = (RIFSC_RIMC_ATTRx_MCID | RIFSC_RIMC_ATTRx_MPRIV | RIFSC_RIMC_ATTRx_MSEC);
	uint32_t wValue = ((masterCID << RIFSC_RIMC_ATTRx_MCID_Pos) | (0x03 << RIFSC_RIMC_ATTRx_MSEC_Pos));		//Bit 0: Master Secure; Bit 1: Master priviliged
	MODIFY_REG(RIFSC->RIMC_ATTRx[masterID], wMask, wValue);

	// Allows CPU to access DCMIPP registers in Secure/Privileged mode.
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

void Dcmipp::InterruptHandler() {
	uint32_t cmsr2 = this->instance->CMSR2;
	uint32_t cmier = this->instance->CMIER;

	// Pipe 0 Interrupts
	// Handle Pipe 0 LIMIT Error Event
	if(((cmsr2 & DCMIPP_CMSR2_P0LIMITF) == DCMIPP_CMSR2_P0LIMITF) && ((cmier & DCMIPP_CMIER_P0LIMITIE) == DCMIPP_CMIER_P0LIMITIE)) {
		// Clear Interrupt
		WRITE_REG(this->instance->CMFCR, DCMIPP_CMFCR_CP0LIMITF);
	}

	// Handle Pipe 0 VSYNC Event
	if(((cmsr2 & DCMIPP_CMSR2_P0VSYNCF) == DCMIPP_CMSR2_P0VSYNCF) && ((cmier & DCMIPP_CMIER_P0VSYNCIE) == DCMIPP_CMIER_P0VSYNCIE)) {
		// Clear Interrupt
		WRITE_REG(this->instance->CMFCR, DCMIPP_CMFCR_CP0VSYNCF);
	}

	// Handle Pipe 0 FRAME Event
	if(((cmsr2 & DCMIPP_CMSR2_P0FRAMEF) == DCMIPP_CMSR2_P0FRAMEF) && ((cmier & DCMIPP_CMIER_P0FRAMEIE) == DCMIPP_CMIER_P0FRAMEIE)) {
		// If snapshot mode, automatically disable interrupts and reset states
		if((this->instance->P1FCTCR & DCMIPP_P0FCTCR_CPTMODE) == DCMIPP_P0FCTCR_CPTMODE) {
			// Disable Interrupts
			MODIFY_REG(this->instance->CMIER, DCMIPP_CMIER_P0FRAMEIE | DCMIPP_CMIER_P0VSYNCIE | DCMIPP_CMIER_P0OVRIE, 0x00);
		}

		// Clear Interrupt
		WRITE_REG(this->instance->CMFCR, DCMIPP_CMFCR_CP0FRAMEF);
	}

	// Handle Pipe 0 LINE Event
	if(((cmsr2 & DCMIPP_CMSR2_P0LINEF) == DCMIPP_CMSR2_P0LINEF) && ((cmier & DCMIPP_CMIER_P0LINEIE) == DCMIPP_CMIER_P0LINEIE)) {
		// Clear Interrupt
		WRITE_REG(this->instance->CMFCR, DCMIPP_CMFCR_CP0LINEF);
	}

	// Handle Pipe 0 Overrun error Event
	if(((cmsr2 & DCMIPP_CMSR2_P0OVRF) == DCMIPP_CMSR2_P0OVRF) && ((cmier & DCMIPP_CMIER_P0OVRIE) == DCMIPP_CMIER_P0OVRIE)) {
		// Clear Interrupt
		WRITE_REG(this->instance->CMFCR, DCMIPP_CMFCR_CP0OVRF);
	}

	// Pipe 1 Interrupts
	// Handle Pipe 1 LINE Event
	if(((cmsr2 & DCMIPP_CMSR2_P1LINEF) == DCMIPP_CMSR2_P1LINEF) && ((cmier & DCMIPP_CMIER_P1LINEIE) == DCMIPP_CMIER_P1LINEIE)) {
		// Clear Interrupt
		WRITE_REG(this->instance->CMFCR, DCMIPP_CMFCR_CP1LINEF);
	}

	// Handle Pipe 1 VSYNC Event
	if(((cmsr2 & DCMIPP_CMSR2_P1VSYNCF) == DCMIPP_CMSR2_P1VSYNCF) && ((cmier & DCMIPP_CMIER_P1VSYNCIE) == DCMIPP_CMIER_P1VSYNCIE)) {
		// Clear Interrupt
		WRITE_REG(this->instance->CMFCR, DCMIPP_CMFCR_CP1VSYNCF);
	}

	// Handle Pipe 1 Frame End Event
	if(((cmsr2 & DCMIPP_CMSR2_P1FRAMEF) == DCMIPP_CMSR2_P1FRAMEF) && ((cmier & DCMIPP_CMIER_P1FRAMEIE) == DCMIPP_CMIER_P1FRAMEIE)) {
		// If snapshot mode, automatically disable interrupts and reset states
		if((this->instance->P1FCTCR & DCMIPP_P1FCTCR_CPTMODE) == DCMIPP_P1FCTCR_CPTMODE) {
			// Disable Interrupts
			MODIFY_REG(this->instance->CMIER, DCMIPP_CMIER_P1FRAMEIE | DCMIPP_CMIER_P1VSYNCIE | DCMIPP_CMIER_P1OVRIE, 0x00);

			// Disable Pipe Processing
			CLEAR_BIT(this->instance->P1FSCR, DCMIPP_P1FSCR_PIPEN);
		}

		// Clear Interrupt
		WRITE_REG(this->instance->CMFCR, DCMIPP_CMFCR_CP1FRAMEF);

		tx_event_flags_set(&this->event, EVT_TRANS_CPLT, TX_OR);
	}

	// Handle Overrun and Synchronization Errors as needed
	if(((cmsr2 & DCMIPP_CMSR2_P1OVRF) == DCMIPP_CMSR2_P1OVRF) && ((cmier & DCMIPP_CMIER_P1OVRIE) == DCMIPP_CMIER_P1OVRIE)) {
		// Clear Interrupt
		WRITE_REG(this->instance->CMFCR, DCMIPP_CMFCR_CP1OVRF);

		tx_event_flags_set(&this->event, EVT_ERR, TX_OR);
	}
	
	// Pipe 2 Interrupts
	// Handle Pipe 2 LINE Event
	if(((cmsr2 & DCMIPP_CMSR2_P2LINEF) == DCMIPP_CMSR2_P2LINEF) && ((cmier & DCMIPP_CMIER_P2LINEIE) == DCMIPP_CMIER_P2LINEIE)) {
		// Clear Interrupt
		WRITE_REG(this->instance->CMFCR, DCMIPP_CMFCR_CP2LINEF);
	}

	// Handle Pipe 2 VSYNC Event
	if(((cmsr2 & DCMIPP_CMSR2_P2VSYNCF) == DCMIPP_CMSR2_P2VSYNCF) && ((cmier & DCMIPP_CMIER_P2VSYNCIE) == DCMIPP_CMIER_P2VSYNCIE)) {
		// Clear Interrupt
		WRITE_REG(this->instance->CMFCR, DCMIPP_CMFCR_CP2VSYNCF);
	}

	// Handle Pipe 2 Frame End Event
	if(((cmsr2 & DCMIPP_CMSR2_P2FRAMEF) == DCMIPP_CMSR2_P2FRAMEF) && ((cmier & DCMIPP_CMIER_P2FRAMEIE) == DCMIPP_CMIER_P2FRAMEIE)) {
		// If snapshot mode, automatically disable interrupts and reset states
		if((this->instance->P2FCTCR & DCMIPP_P2FCTCR_CPTMODE) == DCMIPP_P2FCTCR_CPTMODE) {
			// Disable Interrupts
			MODIFY_REG(this->instance->CMIER, DCMIPP_CMIER_P2FRAMEIE | DCMIPP_CMIER_P2VSYNCIE | DCMIPP_CMIER_P2OVRIE, 0x00);
		}

		// Clear Interrupt
		WRITE_REG(this->instance->CMFCR, DCMIPP_CMFCR_CP2FRAMEF);
	}

	// Handle Overrun and Synchronization Errors as needed
	if(((cmsr2 & DCMIPP_CMSR2_P2OVRF) == DCMIPP_CMSR2_P2OVRF) && ((cmier & DCMIPP_CMIER_P2OVRIE) == DCMIPP_CMIER_P2OVRIE)) {
		// Clear Interrupt
		WRITE_REG(this->instance->CMFCR, DCMIPP_CMFCR_CP2OVRF);
	}

	// Synchronization Error Interrupt on the parallel interface
	if(((cmsr2 & DCMIPP_CMSR2_PRERRF) == DCMIPP_CMSR2_PRERRF) && ((cmier & DCMIPP_CMIER_PRERRIE) == DCMIPP_CMIER_PRERRIE)) {
		// Clear Interrupt
		WRITE_REG(this->instance->CMFCR, DCMIPP_CMFCR_CPRERRF);
	}

	// IPPLUG AXI transfer Error Interrupt
	if(((cmsr2 & DCMIPP_CMSR2_ATXERRF) == DCMIPP_CMSR2_ATXERRF) && ((cmier & DCMIPP_CMIER_ATXERRIE) == DCMIPP_CMIER_ATXERRIE)) {
		// Clear Interrupt
		WRITE_REG(this->instance->CMFCR, DCMIPP_CMFCR_CATXERRF);
	}
}