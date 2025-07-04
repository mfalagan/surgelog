// surgelog, mfalagan, 2025

#include <Arduino.h>

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

	constexpr uint32_t SAMPLING_FREQ = 250'000;	// Hz
	constexpr uint32_t BLOCK_SIZE = 256;		// Samples
	constexpr uint32_t QUEUE_SIZE = 1024;		// Blocks (-> half of RAM2/DMAMEM)
	constexpr uint32_t SIGNAL_AMPLITUDE  = 1;	// IPG ticks

// -- TMR --

	// We'll need a whole module for this. TMR3 and TMR4 expose a pin to XBAR1 directly, without
	// going through XBAR2 or XBAR3 first. TMR4 is used all over the Teensy libraries, so, to avoid
	// any possible collisions, I'll go with TMR3.

	IMXRT_TMR3.ENBL |= TMR3_ENBL; // Enable module (all four channels)

	// Channel 0 generates the ACQ signal (used ADC conversions)

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
		// TMR_CSCTRL_TCF1EN |		// No interrupt on compare 1
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

	// CH1 can temporarily be set up to count CH0 pulses

	IMXRT_TMR3.CH[1].CTRL = 0 |
		TMR_CTRL_CM(0b001) |	// Count primary source
		TMR_CTRL_PCS(0b0100) |	// Primary Count Source is counter 0 output
		TMR_CTRL_SCS(0b00) |	// Secondary Count Source is channel 0 input
		// TMR_CTRL_ONCE |		// Count repeatedly (no halt on compare)
		// TMR_CTRL_LENGTH |	//  No reset on compare
		// TMR_CTRL_DIR |		// Count up
		// TMR_CTRL_COINIT |	// No other channel may force reinitialization
		TMR_CTRL_OUTMODE(0b000);// Output asserted while counter active

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
		// TMR_SCTRL_OEN;		// Output disabled

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

	IMXRT_TMR3.CH[1].CNTR  = 0; // Reset counter

	// Now we test!
	{
		constexpr uint32_t DURATION_MS = (float) (1 << 16) * 900 / SAMPLING_FREQ;

		IMXRT_TMR3.CH[0].CTRL |= TMR_CTRL_CM(0b001); // Activate CH0
		delay(DURATION_MS);
		IMXRT_TMR3.CH[0].CTRL &= ~ TMR_CTRL_CM(0b001); // De-activate CH0

		Serial.printf("Counted ticks:\t%d\nBus frequency:\t%0.03f MHz\nTick frequency:\t%0.03f KHz\n",
			IMXRT_TMR3.CH[1].CNTR,
			(float) F_BUS_ACTUAL / 1'000'000,
			(float) IMXRT_TMR3.CH[1].CNTR / DURATION_MS);
		
		// Seems OK
	}
	
	return 0;
}