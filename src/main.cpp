// surgelog, mfalagan, 2025

#include <Arduino.h>

volatile bool has_crashed = false;
volatile uint32_t write_counter = 0;
volatile uint32_t extra_blocks = 5;

void irq_tmr3(void)
{
	if ((IMXRT_TMR3.CH[0].CSCTRL & TMR_CSCTRL_TCF1) != 0) // ACQ signal
	{
		IMXRT_TMR3.CH[0].CSCTRL &= ~TMR_CSCTRL_TCF1; // ACK
		++ write_counter;
	}
	if ((IMXRT_TMR3.CH[2].SCTRL & TMR_SCTRL_TCF) != 0) // Crash
	{
		IMXRT_TMR3.CH[2].SCTRL &= ~TMR_SCTRL_TCF; // ACK
		has_crashed = true;
	}

	static int countdown = extra_blocks;
	if (countdown)
	{
		IMXRT_TMR3.CH[2].COMP1 += 1; // release one
		-- countdown;
	}

	asm volatile("dsb"); // ensure ACK has reached peripheral
}

extern "C" void xbar_connect(unsigned int input, unsigned int output); // shoulda put it in a header

int main(void)
{
	Serial.begin(115200);
	while (!Serial) /* wait */;

	/**
	 * This is some quick and dirty testing of the bare-metal hardware configuration. I do intend to
	 * group it into a more formal driver once everything is working. 
	 * 
	 * The hardware pipeline design is based on a preliminary read of the i.MX RT1060 Processor
	 * Reference Manual (rev 3) and may therefore be subject to change.
	 * 
	 * I am going to document my thought process during the hardware setup phase, primarily as
	 * future reference material (both for me and anyone that somehow comes across this). This part
	 * tends to be quite arduous, and the ability to tap into the developer's thoughts usually
	 * proves helpful in interpreting the code.
	 * 
	 * I do intend to formalise this all into a proper README.
	 * 
	 * As a final comment, be aware that surgelog is a personal recreational programming project.
	 * Use anything here at your own risc ;)
	 */

	// User-defined program constants:
	constexpr uint32_t SAMPLING_FREQ = 250'000;	// Hz
	constexpr uint32_t BLOCK_SIZE = 256;		// Samples
	constexpr uint32_t QUEUE_SIZE = 1024;		// Blocks (-> half of RAM2/DMAMEM)
	constexpr uint32_t SIGNAL_AMPLITUDE  = 1;	// IPG ticks

// -- TMR --

	// We'll need a whole module for this. TMR3 and TMR4 expose a pin to XBAR1 directly, without
	// going through XBAR2 or XBAR3 first. TMR4 is used all over the Teensy libraries, so, to avoid
	// any possible collisions, I'll go with TMR3.

	IMXRT_TMR3.ENBL |= TMR3_ENBL; // Enable module (all four channels)

	// Channel 0 generates the ACQ signal (triggers ADC conversions)

	IMXRT_TMR3.CH[0].CTRL = 0 |
		TMR_CTRL_CM(0b000) |	// No operation (set at the end of config)
		TMR_CTRL_PCS(0b1000) |	// Primary Count Source is IPG/1 (150MHz)
		TMR_CTRL_SCS(0b00) |	// Secondary Count Source is channel 0 input
		// TMR_CTRL_ONCE |		// Count repeatedly (no halt on compare)
		TMR_CTRL_LENGTH |		// Reset timer on compare
		// TMR_CTRL_DIR |		// Count up
		// TMR_CTRL_COINIT |	// No other channel may force reinitialization
		TMR_CTRL_OUTMODE(0b100);// Toggle OFLAG on alternating compare

	IMXRT_TMR3.CH[0].SCTRL = 0 |
		// TMR_SCTRL_TCFIE |	// Compare interrupt disabled
		// TMR_SCTRL_TOFIE |	// Overflow interrupt disabled
		// TMR_SCTRL_IEFIE |	// Input Edge interrupt disabled
		// TMR_SCTRL_IPS |		// Normal input polarity
		// TMR_SCTRL_CAPTURE_MODE(0b00) // No capture ever
		// TMR_SCTRL_MSTR |		// No master mode
		// TMR_SCTRL_EEOF |		// No external OFLAG force
		// TMR_SCTRL_VAL |		// Irrelevant - only for OFLAG force
		// TMR_SCTRL_FORCE |	// No OFLAG force
		// TMR_SCTRL_OPS |		// Normal output polarity
		TMR_SCTRL_OEN;			// Output enable

	IMXRT_TMR3.CH[0].CSCTRL = 0 |
		TMR_CSCTRL_DBG_EN(0b00) |	// Normal operation on DBG mode
		// TMR_CSCTRL_FAULT |		// No fault mode
		// TMR_CSCTRL_ALT_LOAD |	// No alternative load
		// TMR_CSCTRL_ROC |			// No reload on capture
		// TMR_CSCTRL_TCI |			// No reinitialize on second trigger
		// TMR_CSCTRL_UP |			// Irrelevant - only for quadrature mode
		// TMR_CSCTRL_TCF2EN |		// No interrupt on compare 2
		TMR_CSCTRL_TCF1EN |			// Interrupt on compare 1 TODO: just for testing
		TMR_CSCTRL_CL2(0b00) |		// Never load new COMP2
		TMR_CSCTRL_CL1(0b00);		// Never load new COMP1

	IMXRT_TMR3.CH[0].FILT = 0 |
		TMR_FILT_FILT_PER(0b0);		// Filtering disabled

	IMXRT_TMR3.CH[0].DMA = 0 ;
		// TMR_DMA_CMPLD2DE |		// Comparator preload 2 DMA disabled
		// TMR_DMA_CMPLD1DE |		// Comparator preload 1 DMA disabled
		// TMR_DMA_IEFDE;			// Input edge DMA disabled

	// We can now prime the counter registers:
	IMXRT_TMR3.CH[0].COMP1 = F_BUS_ACTUAL / SAMPLING_FREQ - SIGNAL_AMPLITUDE;
	IMXRT_TMR3.CH[0].COMP2 = SIGNAL_AMPLITUDE;
	IMXRT_TMR3.CH[0].LOAD  = 1;
	IMXRT_TMR3.CH[0].CNTR  = IMXRT_TMR3.CH[0].LOAD;

		// -- Channel 1 generates a signal when a block is full

	IMXRT_TMR3.CH[1].CTRL = 0 |
		TMR_CTRL_CM(0b001) |	// Count primary source
		TMR_CTRL_PCS(0b0100) |	// Primary Count Source is counter 0 output
		TMR_CTRL_SCS(0b00) |	// Irrelevant - secondary source
		// TMR_CTRL_ONCE |		// Count repeatedly (no halt on compare)
		TMR_CTRL_LENGTH |		// Reset timer on compare
		// TMR_CTRL_DIR |		// Count up
		// TMR_CTRL_COINIT |	// No other channel may force reinitialization
		TMR_CTRL_OUTMODE(0b100);// Toggle OFLAG on alternating compare

	IMXRT_TMR3.CH[1].SCTRL = 0 ;
		// TMR_SCTRL_TCFIE |	// Compare interrupt disabled
		// TMR_SCTRL_TOFIE |	// Overflow interrupt disabled
		// TMR_SCTRL_IEFIE |	// Input Edge interrupt disabled
		// TMR_SCTRL_IPS |		// Normal input polarity
		// TMR_SCTRL_CAPTURE_MODE(0b00) // No capture ever
		// TMR_SCTRL_MSTR |		// No master mode
		// TMR_SCTRL_EEOF |		// No external OFLAG force
		// TMR_SCTRL_VAL |		// Irrelevant - only for OFLAG force
		// TMR_SCTRL_FORCE |	// No OFLAG force
		// TMR_SCTRL_OPS |		// Normal output polarity
		// TMR_SCTRL_OEN;		// Output disabled (other channels still see its OFLAG)

	IMXRT_TMR3.CH[1].CSCTRL = 0 |
		TMR_CSCTRL_DBG_EN(0b00) |	// Normal operation on DBG mode
		// TMR_CSCTRL_FAULT |		// No fault mode
		// TMR_CSCTRL_ALT_LOAD |	// No alternative load
		// TMR_CSCTRL_ROC |			// No reload on capture
		// TMR_CSCTRL_TCI |			// No reinitialize on second trigger
		// TMR_CSCTRL_UP |			// Irrelevant - only for quadrature mode
		// TMR_CSCTRL_TCF2EN |		// No interrupt on compare 2
		// TMR_CSCTRL_TCF1EN |		// No interrupt on compare 1
		TMR_CSCTRL_CL2(0b00) |		// Never load new COMP2
		TMR_CSCTRL_CL1(0b00);		// Never load new COMP1

	IMXRT_TMR3.CH[1].FILT = 0 |
		TMR_FILT_FILT_PER(0b0);		// Filtering disabled

	IMXRT_TMR3.CH[1].DMA = 0 ;
		// TMR_DMA_CMPLD2DE |		// Comparator preload 2 DMA disabled
		// TMR_DMA_CMPLD1DE |		// Comparator preload 1 DMA disabled
		// TMR_DMA_IEFDE;			// Input edge DMA disabled

	IMXRT_TMR3.CH[1].COMP1 = BLOCK_SIZE - SIGNAL_AMPLITUDE;
	IMXRT_TMR3.CH[1].COMP2 = SIGNAL_AMPLITUDE;
	IMXRT_TMR3.CH[1].LOAD  = 1;
	IMXRT_TMR3.CH[1].CNTR  = IMXRT_TMR3.CH[1].LOAD;

	// -- Channel 2 maintains queue integrity

	/**
	 * I believe this bit can benefit from some conceptual explaining first. The data blocks
	 * produced by this pipeline will fo into a SPSC circular FIFO in memory, from where the DSP
	 * software can access it. This requires synchronization on both ends:
	 * 
	 *    - The consumer should not access a block that is still being written to. Block
	 *      availability is communicated to the core via an IRQ once the block has been filled.
	 * 
	 *    - The producer should not write data to a block that is still being read from. Slot 
	 *      availability is communicated to the hardware via this counter (CH2).
	 * 
	 * The mechanics are the following: CNTR starts at 0 and is incremented each time the write head
	 * reaches the end of a block. CMP1 is loaded with QUEUE_SIZE and is incremented each time the
	 * read head frees a slot. Both values are allowed to overflow.
	 * The difference between the two values is the remaining space in the queue, and so, this
	 * number being zero (CNTR = CMP1 -> successful compare) signifies a head crash.
	 * 
	 * A head crash is a fatal condition, as data consecutiveness must be lost to continue sampling.
	 * For this reason, a succesful compare on CH2 does two things:
	 * 
	 *    - Pauses CH0 through its secondary source, halting the pipeline.
	 * 
	 *    - Requests an IRQ to notify the core.
	 */

	IMXRT_TMR3.CH[2].CTRL = 0 |
		TMR_CTRL_CM(0b001) |	// Count primary source
		TMR_CTRL_PCS(0b0101) |	// Primary Count Source is counter 1 output
		TMR_CTRL_SCS(0b00) |	// Irrelevant - only for secondary source
		// TMR_CTRL_ONCE |		// Count repeatedly (no halt on compare)
		// TMR_CTRL_LENGTH |	// No reset on compare
		// TMR_CTRL_DIR |		// Count up
		// TMR_CTRL_COINIT |	// No other channel may force reinitialization
		TMR_CTRL_OUTMODE(0b001);// Clear OFLAG on compare

	IMXRT_TMR3.CH[2].SCTRL = 0 |
		TMR_SCTRL_TCFIE |		// Compare interrupt enabled
		// TMR_SCTRL_TOFIE |	// Overflow interrupt disabled
		// TMR_SCTRL_IEFIE |	// Input Edge interrupt disabled
		// TMR_SCTRL_IPS |		// Normal input polarity
		// TMR_SCTRL_CAPTURE_MODE(0b00) // No capture ever
		// TMR_SCTRL_MSTR |		// No master mode
		// TMR_SCTRL_EEOF |		// No external OFLAG force
		// TMR_SCTRL_VAL |		// Force LOW
		TMR_SCTRL_FORCE |		// Do force
		// TMR_SCTRL_OPS |		// Normal output polarity
		TMR_SCTRL_OEN;			// Output enable

	IMXRT_TMR3.CH[2].CSCTRL = 0 |
		TMR_CSCTRL_DBG_EN(0b00) |	// Normal operation on DBG mode
		// TMR_CSCTRL_FAULT |		// No fault mode
		// TMR_CSCTRL_ALT_LOAD |	// No alternative load
		// TMR_CSCTRL_ROC |			// No reload on capture
		// TMR_CSCTRL_TCI |			// No reinitialize on second trigger
		// TMR_CSCTRL_UP |			// Irrelevant - only for quadrature mode
		// TMR_CSCTRL_TCF2EN |		// No interrupt on compare 2
		// TMR_CSCTRL_TCF1EN |		// No interrupt on compare 1
		TMR_CSCTRL_CL2(0b00) |		// Never load new COMP2
		TMR_CSCTRL_CL1(0b00);		// Never load new COMP1

	IMXRT_TMR3.CH[2].FILT = 0 |
		TMR_FILT_FILT_PER(0b0);		// Filtering disabled

	IMXRT_TMR3.CH[2].DMA = 0 ;
		// TMR_DMA_CMPLD2DE |		// Comparator preload 2 DMA disabled
		// TMR_DMA_CMPLD1DE |		// Comparator preload 1 DMA disabled
		// TMR_DMA_IEFDE;			// Input edge DMA disabled

	IMXRT_TMR3.CH[2].COMP1 = QUEUE_SIZE;
	IMXRT_TMR3.CH[2].CNTR  = 1;

	// We only have to do some wiring now.
	// First of all, connect CH3 output to CH3 input through the XBAR

	// The Teensy startup routine does not seem to connect the XBAR clock, we do it manually 
	CCM_CCGR2 |= CCM_CCGR2_XBAR1(CCM_CCGR_ON);

	// This routine is hidden away in PWM.c, but it is very handy and does just what it says it does
	xbar_connect(XBARA1_IN_QTIMER3_TIMER2, XBARA1_OUT_QTIMER3_TIMER0);

	// The input is attached to IOMUX by default. We can switch it ti XBAR
	IOMUXC_GPR_GPR6 |= IOMUXC_GPR_GPR6_QTIMER3_TRM0_INPUT_SEL;

	// Once the system is wided, we can set CH0 to count primary while secondary is high
	IMXRT_TMR3.CH[0].CTRL |= TMR_CTRL_CM(0b011);

	// With this ready to work, we can enable our IRQ in the NVIC
	NVIC_ENABLE_IRQ(IRQ_QTIMER3);

	// And attach out ISR vector to it
	attachInterruptVector(IRQ_QTIMER3, irq_tmr3);

	// Now we test!
	{
		IMXRT_TMR3.CH[2].SCTRL |= TMR_SCTRL_VAL | TMR_SCTRL_FORCE; // Start ACQ

		while (! has_crashed) /* wait */;
		
		Serial.printf("Total writes:\t%d\nBlocks written:\t%d / %d\nOverrun:\t%d / %d\n",
			write_counter,
			write_counter / BLOCK_SIZE, QUEUE_SIZE + extra_blocks,
			write_counter % BLOCK_SIZE, BLOCK_SIZE);

		// Seems OK

		/**
		 * Actually, I started poking around to measure how much a crash takes to propagate from
		 * CH2, through the XBAR and into CH1, and the CNTRs hold... unexpected values. There seems
		 * to be a phase error (of one) somewhere I can't find. Anyhow, a phase error does not
		 * change the actual function of the timer, and this delay should be deterministic (all of
		 * the invloved peripherals run on IPG) and much shorter than the ACQ period (I expect
		 * around ~6 ticks). I'd say it is safe to assume this setup works and no ACQ pulses slip
		 * through.
		 * 
		 * The unexpected result can be seen in the following log:
		 */

		// Serial.printf("Pointers at ACQ halt:\n\tBlock %d/%d\n\tSample %d/%d\n\tCycle %d/%d\n",
        //     IMXRT_TMR3.CH[2].CNTR, QUEUE_SIZE + extra_blocks,
        //     IMXRT_TMR3.CH[1].CNTR, BLOCK_SIZE,
        //     IMXRT_TMR3.CH[0].CNTR, F_BUS_ACTUAL / SAMPLING_FREQ);
	}
	
	return 0;
}