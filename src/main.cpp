// surgelog, mfalagan, 2025

#include <Arduino.h>
#include <atomic>

volatile bool has_crashed = false;
volatile uint16_t last_val = 0xFFFF;
std::atomic<size_t> sample_count = 0;

void irq_tmr3(void)
{
	IMXRT_TMR3.CH[2].SCTRL &= ~TMR_SCTRL_TCF; // ACK
	has_crashed = true;

	asm volatile("dsb"); // ensure ACK has reached peripheral
}

void irq_etc(void)
{
	ADC_ETC_DONE0_1_IRQ |= 0b1; // ACK - why the inconsistent IRQ clearing? Hardware magic I guess...

	sample_count.fetch_add(1);
	last_val = ADC_ETC_TRIG0_RESULT_1_0 & ((1<<16) - 1);

	asm volatile("dsb");
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

	/**
	 * We'll need a whole module for this. TMR3 and TMR4 expose a pin to XBAR1 directly, without
	 * going through XBAR2 or XBAR3 first. TMR4 is used all over the Teensy libraries, so, to avoid
	 * any possible collisions, I'll go with TMR3.
	 */

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

// -- ADC --

	/**
	 * With this all done, we can route the ACQ signal through the XBAR and into the ETC, to have it
	 * generate ADC triggers for us. For this purpose, we must first configure the ADC.
	 * 
	 * The configuration I am going to apply is the result of some quick testing and benchmarking, 
	 * they may vary later on, when the pipeline is fully working, but are good as a placeholder.
	 * The intent is to have both ADCs working together through the ETC's SYNC mode, but, for
	 * testing purposes, I am going to set up only one of them. I'll address this later on.
	 */

	/**
	 * The first step is to wire the IOMUX to connect the ADC to an external pin. Through the magic
	 * of having already wired a test setup and not being bothered to change it, I know ADC1 must be
	 * connected to pin A10/24, and ADC2 to pin A14/38. 
	 */

	// Pin 24 corresponds to pad AD_B0_12. We can set it to Input Mode
	IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B0_12 |= ((uint32_t)(1<<4)); // SION 1 (why is it not in imxrt.h?)

	// One other important thing, as per specified in the manual, the keeper needs to be disabled.
	IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B0_12 &= ~ IOMUXC_PAD_PKE;

	// We can now configure the ADC itself
	
	IMXRT_ADC1.CFG = 0 |
		ADC_CFG_OVWREN |		// Output register overwrite enabled
		ADC_CFG_AVGS(0b00) |	// 4 sample averaging
		// ADC_CFG_ADTRG |		// Software trigger selected (to enable calibration, this will be changed)
		ADC_CFG_REFSEL(0b00) |	// VREFH/VREFL voltage reference. It is the only one allowed lol
		// ADC_CFG_ADHSC |		// High speed mode disabled. I believe it only applies to ADACK
		ADC_CFG_ADSTS(0b11) |	// Sample period = 9 / 25 ticks (9 with the current ADLSMP)
		// ADC_CFG_ADLPC |		// Low power configuration disabled
		ADC_CFG_ADIV(0b00) |	// No clock divider
		ADC_CFG_ADLSMP |		// Long sample mode (25 ticks per sample, with the current ADSTS)
		ADC_CFG_MODE(0b10) |	// 12-bit conversion mode
		ADC_CFG_ADICLK(0b00);	// IPG clock as source

	IMXRT_ADC1.GC = 0 |
		// ADC_GC_CAL |		// this bit begins the calibration process, we don't want that yet
		// ADC_GC_ADCO |	// Continuous conversion disabled
		ADC_GC_AVGE ;		// Hardware averaging enabled
		// ADC_GC_ACFE |	// Compare function disabled
		// ADC_GC_ACFGT |	// Compare function greater than mode, does not apply
		// ADC_GC_ACREN |	// Compare function range mode, does not apply
		// ADC_GC_DMAEN |	// DMA requests disabled.
		// ADC_GC_ADACKEN;	// Asynchronous clock (ADACK) disabled

	// Finally we can calibrate
	IMXRT_ADC1.GC |= ADC_GC_CAL; // send calibration signal
	while ((IMXRT_ADC1.HS & ADC_HS_COCO0) == 0) /* wait */; // wait for calibration

	// With calibration done, we can set the ADC to Hardware Trigger mode
	IMXRT_ADC1.CFG |= ADC_CFG_ADTRG;

	// In testing I am only going to use trigger 0
	IMXRT_ADC1.HC0 = 0 |
		// ADC_HC_AIEN |		// Converison interrupt disabled
		ADC_HC_ADCH(0b10000);	// External channel selection from ETC
	
// -- ETC --

	// We may as well start by resetting the registers
	IMXRT_ADC_ETC.CTRL |= ADC_ETC_CTRL_SOFTRST;
	IMXRT_ADC_ETC.CTRL &= ~ ADC_ETC_CTRL_SOFTRST;

	IMXRT_ADC_ETC.CTRL = 0 |
		// ADC_ETC_CTRL_SOFTRST |			// Reset bit
		// ADC_ETC_CTRL_TSC_BYPASS |		// Irrelevant - I won't use TSC
		// ADC_ETC_CTRL_DMA_MODE_SEL|		// No DMA for the moment
		ADC_ETC_CTRL_PRE_DIVIDER(0b0) |		// No pre-divider
		ADC_ETC_CTRL_EXT1_TRIG_PRIORITY(0b0) |// Irrelevant - I won't use TSC
		// ADC_ETC_CTRL_EXT1_TRIG_ENABLE |	// TSC1 trigger disabled
		ADC_ETC_CTRL_EXT0_TRIG_PRIORITY(0b0) |// Irrelevant - I won't use TSC
		// ADC_ETC_CTRL_EXT0_TRIG_ENABLE |	// TSC2 trigger disabled
		ADC_ETC_CTRL_TRIG_ENABLE(0b1);		// Enable XBAR trigger 1
	
	IMXRT_ADC_ETC.TRIG[0].CTRL = 0 |
		// ADC_ETC_TRIG_CTRL_SYNC_MODE |		// No sync, ADCs are controlled separately
		ADC_ETC_TRIG_CTRL_TRIG_PRIORITY(0b111) |// Highest priority to trigger 0
		ADC_ETC_TRIG_CTRL_TRIG_CHAIN(0b0) ;		// Trigger chan length 1
		// ADC_ETC_TRIG_CTRL_TRIG_MODE |		// Hardware trigger

	IMXRT_ADC_ETC.TRIG[0].CHAIN_1_0 = 0 |
		ADC_ETC_TRIG_CHAIN_IE0(0b1) |	// Interrupt on Done0 (for testing)
		ADC_ETC_TRIG_CHAIN_B2B0 |		// Back-to-back enabled. Maybe later I'll set up chains with delays.
		ADC_ETC_TRIG_CHAIN_HWTS0(0b1) |	// ADC TRIG00 selected
		ADC_ETC_TRIG_CHAIN_CSEL0(0b1);	// ADC channel 2 selected
	
	IMXRT_ADC_ETC.DMA_CTRL = ADC_ETC_DMA_CTRL_TRIQ_ENABLE(0b0); // DMA disabled for now

	// Once again, we connect QTIMER3_TIMER0 (ACQ signal) to ADC_ETC_TRIG00 (ETC trigger)
	xbar_connect(XBARA1_IN_QTIMER3_TIMER0, XBARA1_OUT_ADC_ETC_TRIG00);

	// We enable the IRQ
	NVIC_ENABLE_IRQ(IRQ_ADC_ETC0);

	// And attach its vector
	attachInterruptVector(IRQ_ADC_ETC0, irq_etc);

	// Now we test!
	{
		IMXRT_TMR3.CH[2].SCTRL |= TMR_SCTRL_VAL | TMR_SCTRL_FORCE; // Start ACQ

		size_t block = 0;;
		while (! has_crashed)
		{
			size_t temp = sample_count.load();
			while (temp >= BLOCK_SIZE)
			{
				if (sample_count.compare_exchange_weak(temp, temp % BLOCK_SIZE))
				{
					uint16_t increment = temp / BLOCK_SIZE;
					IMXRT_TMR3.CH[2].COMP1 += increment;
					block += increment;
				}
			}

			if (last_val == 0xFFFF) Serial.printf("No data (block #%d, sample #%d)\n", block, sample_count.load());
			else Serial.printf("Last sample: %d (block #%d, sample #%d)\n", last_val, block, sample_count.load());

			delay(100);
		}

		Serial.printf("Crash -- should not happen!\n");

		// Seems OK
	}
	
	return 0;
}