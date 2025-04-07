#include <msp430.h>
#include <stdint.h>
#include <stdio.h>
volatile unsigned int intervalCount = 0;

// Light Sensor Reg commands
#define LTR303_ADDR     0x29      // 7-bit I2C address for the sensor
#define ALS_CONTR       0x80      // Control register for ALS operation
#define ALS_MEAS_RATE   0x85      // Measurement rate register
const uint8_t als_channel_regs[4] = { 0x88, 0x89, 0x8A, 0x8B };

// Global variables for sensor data
volatile int voltage_value = 0;
volatile int temp_value = 0;
volatile int water_level_value = 0;
volatile int light_value = 0;
volatile int zigbee_sleep_state = 1; // 1 = asleep, 0 = awake

//-------------------------------------------LIGHTSENSOR-------------------------------------------
void I2C_init(void) {
    UCB0CTL1 |= UCSWRST;                      // Enable software reset
    UCB0CTL0 = UCMST + UCMODE_3 + UCSYNC;       // I2C Master, synchronous mode
    UCB0CTL1 = UCSSEL_2 + UCSWRST;              // Use SMCLK; keep in reset
    UCB0BR0 = 12;                             // Set clock divider (adjust for desired SCL frequency)
    UCB0BR1 = 0;
    UCB0I2CSA = LTR303_ADDR;                   // Set the slave address
    UCB0CTL1 &= ~UCSWRST;                      // Clear software reset to start I2C
}

void I2C_WriteByte(unsigned char reg, unsigned char value) {
    while (UCB0CTL1 & UCTXSTP);                // Ensure stop condition is finished
    UCB0I2CSA = LTR303_ADDR;                   // Set slave address
    UCB0CTL1 |= UCTR + UCTXSTT;                // I2C TX, start condition

    while (!(IFG2 & UCB0TXIFG));               // Wait for start condition to be sent
    UCB0TXBUF = reg;                         // Send register address

    while (!(IFG2 & UCB0TXIFG));               // Wait until register address is transmitted
    UCB0TXBUF = value;                        // Send data byte

    while (!(IFG2 & UCB0TXIFG));               // Wait until data byte is transmitted
    UCB0CTL1 |= UCTXSTP;                       // Generate stop condition

    __delay_cycles(1000);                      // Short delay to ensure stop condition is sent
}

uint8_t I2C_readByte(uint8_t reg) {
  uint8_t data;
  UCB0I2CSA = LTR303_ADDR; // Set slave address

  while (UCB0CTL1 & UCTXSTP)
    ; // Wait if a previous stop is in progress

  // Transmit the register address in TX mode.
  UCB0CTL1 |= UCTR + UCTXSTT; // TX mode; start condition
  while (!(IFG2 & UCB0TXIFG))
    ;              // Wait for TX buffer ready
  UCB0TXBUF = reg; // Send register address

  while (!(IFG2 & UCB0TXIFG))
    ;                  // Wait for transmission
  UCB0CTL1 |= UCTXSTP; // Generate stop condition
  while (UCB0CTL1 & UCTXSTP)
    ; // Wait for stop condition to finish

  // Switch to receiver mode to read the data.
  UCB0CTL1 &= ~UCTR;   // Clear TX flag: receiver mode
  UCB0CTL1 |= UCTXSTT; // Start condition for read
  while (UCB0CTL1 & UCTXSTT)
    ; // Wait for start to complete

  data = UCB0RXBUF;    // Read received data
  UCB0CTL1 |= UCTXSTP; // Generate stop condition
  while (UCB0CTL1 & UCTXSTP)
    ; // Wait for stop condition to complete

  return data;
}

void calculate_lux(uint16_t ch0, uint16_t ch1) {
  int als_gain = 1;
  int als_int = 2;
  // Calibration for the covered top
  double pf_factor = 1.087;
  double ratio = ch1/(ch0 + ch1);

  if (ratio < 0.45) {
    light_value = (1.7743 * ch0 + 1.1059 * ch1)/ als_gain / als_int / pf_factor;
  } else if (ratio < 0.64 && ratio >= 0.45) {
    light_value = (4.2785 * ch0 - 1.9548 * ch1)/ als_gain / als_int / pf_factor;
  } else if (ratio < 0.85 && ratio >= 0.65) {
    light_value = (0.5926 * ch0 + 0.1185 * ch1)/ als_gain / als_int / pf_factor;
  } else {
    light_value = 0;
  }

}

void read_light_sensor(void) {
  unsigned char sensorData[4];
  uint16_t channel1, channel0, i;
  I2C_WriteByte(ALS_CONTR, 0x01); // Active mode, gain 1X

  __delay_cycles(500000); // Wait 500ms
  for (i = 0; i < 4; i++) {
    sensorData[i] = I2C_readByte(als_channel_regs[i]);
  }

  // Combine the bytes into 16-bit values
  channel1 = (sensorData[1] << 8) | sensorData[0];
  channel0 = (sensorData[3] << 8) | sensorData[2];
  calculate_lux(channel0, channel1);
  send_xbee_transmit_request(channel0, channel1, 0, light_value);

  I2C_WriteByte(ALS_CONTR, 0x00); // Standby
}
//--------------------------------------------------------------------------------------------------

//----------------------------------------------ZIGBEE----------------------------------------------
void uart_init(void) {
  WDTCTL = WDTPW | WDTHOLD; // Stop watchdog timer

  P1SEL |= BIT2;
  P1SEL2 |= BIT2;

  // Configure USCI_A0 for UART mode
  UCA0CTL1 |= UCSSEL_2; // Use SMCLK (1MHz default)
  UCA0BR0 = 8;          // 115200 baud: 1MHz / 115200 ≈ 8
  UCA0BR1 = 0;
  UCA0MCTL = UCBRS2 | UCBRS0; // UCBRSx = 5
  UCA0CTL1 &= ~UCSWRST;       // Initialize USCI state machine
}

// UART Send Function (No CTS Flow Control)
void uart_send(uint8_t *data, uint16_t length) {
  uint16_t i; // Declare loop variable outside for loop
  for (i = 0; i < length; i++) {
    while (!(IFG2 & UCA0TXIFG))
      ;                  // Wait for TX buffer to be ready
    UCA0TXBUF = data[i]; // Send byte
  }
}

// Send XBee API Frame (Transmit Request)
void send_xbee_transmit_request(int var1, int var2, int var3, int var4) {
  char payload[30];
  memset(payload, 0, sizeof(payload));
  snprintf(payload, sizeof(payload), "%d %d %d %d", var1, var2, var3, var4);
  uint8_t payload_length = strlen(payload);

  uint16_t frame_length = 14 + payload_length;
  uint8_t frame[100] = {0}; // Ensure buffer is large enough

  // Build XBee API Frame
  frame[0] = 0x7E;
  frame[1] = (frame_length >> 8) & 0xFF;
  frame[2] = frame_length & 0xFF;
  frame[3] = 0x10; // Transmit Request
  frame[4] = 0x01; // Frame ID
  // 64-bit Dest Address (Broadcast)
  frame[5] = 0x00;
  frame[6] = 0x00;
  frame[7] = 0x00;
  frame[8] = 0x00;
  frame[9] = 0x00;
  frame[10] = 0x00;
  frame[11] = 0x00;
  frame[12] = 0x00;
  // 16-bit Dest Address
  frame[13] = 0xFF;
  frame[14] = 0xFE;
  frame[15] = 0x00; // Broadcast radius
  frame[16] = 0x00; // Options

  // Copy Payload
  memcpy(&frame[17], payload, payload_length);

  // Compute Checksum
  uint8_t checksum = 0;
  uint8_t i;
  for (i = 3; i < (17 + payload_length); i++) {
    checksum += frame[i];
  }
  checksum = 0xFF - checksum; // XBee checksum calculation
  frame[17 + payload_length] = checksum;

  // Send Frame
  uart_send(frame, 18 + payload_length);
}
//--------------------------------------------------------------------------------------------------

// Function to read voltage sensor
void read_voltage_sensor() {
  // Configure ADC10CTL1:
  // - INCH_4: Select channel AA (P1.4).
  ADC10CTL0 = ADC10SHT_2 + ADC10ON;
  __delay_cycles(1000); // Increase delay if necessary
  ADC10CTL1 = INCH_4;

  P2OUT |= BIT0;            // Turn on voltage sensor
  __delay_cycles(1000);  // Small delay for stabilization (0.1s)
  voltage_value = adc_readout(); // Read ADC value from P1.4
  P2OUT &= ~BIT0;           // Turn off voltage sensor
}

// Function to read temperature sensor
void read_temp_sensor() {
  // Configure ADC10CTL1:
  // - INCH_3: Select channel A3 (P1.3).
  ADC10CTL0 = ADC10SHT_2 + ADC10ON;
  __delay_cycles(1000); // Increase delay if necessary
  ADC10CTL1 = INCH_3;

  // Convert voltage to temperature in °C
  // MCP9700 outputs ~500 mV at 0°C, 10 mV/°C slope.
  temp_value = (((adc_readout() * 3.3) / 1023.0) - 0.5) * 100;
}

// Function to read water level sensor
void read_water_level_sensor() {
  // Configure ADC10CTL1:
  // - INCH_3: Select channel A3 (P1.3).
  ADC10CTL1 = INCH_5;

  __delay_cycles(1000);  // Small delay for stabilization (0.1s)
  water_level_value = adc_readout(); // Read ADC value from P1.5
}

// Function to wake Zigbee
// Xbee initially is awake when device boots, stay awake for a minute, and then sleep for a while
// Once the device is in its 30 min cycles, this function will wake and wait for Xbee, then give 
// it 30 sec to connect to HA, it will send its data after being awake for a min, go back to sleep.
void toggle_zigbee_wake() {
  P2OUT ^= BIT5;        // Toggle P2.5
  __delay_cycles(5000); // Small delay
  P2OUT ^= BIT5;        // Toggle again

  while ((zigbee_sleep_state = (P2IN & BIT4) ? 1 : 0) == 1) {
    // Loop until P2.4 is low meaning the xbee is awake
    __delay_cycles(1000); // Small delay
  }
  __delay_cycles(500000); // Wait 30 sec for Xbee to connect to HA
}

// Function to control pump
void control_pump() {
  P2OUT |= BIT2;            // Turn pump on
  __delay_cycles(1000000); // Run for 1 sec
  P2OUT &= ~BIT2;           // Turn pump off
}

// Centralized reading of adc converions
int adc_readout() {
  // Start ADC conversion:
  ADC10CTL0 |= ENC | ADC10SC; // Enable conversion and start conversion

  __delay_cycles(1000); // Increase delay if necessary
  // Wait for conversion to complete:
  // (ADC10SC automatically clears when conversion is done)
  while (ADC10CTL1 & ADC10BUSY)
    ;

  int adcValue = ADC10MEM;
  ADC10CTL0 &= ~ADC10ON;
  return adcValue;
}

// Main function
int main(void) {
  int pump_counter = 0;

  WDTCTL = WDTPW | WDTHOLD; // Stop watchdog timer

  P2DIR |= BIT0 | BIT2 | BIT5;    // Set P2.0, P2.2, and P2.5 as outputs
  P2DIR &= ~BIT4;                 // Set P2.4 as input

  ADC10AE0 |= BIT3| BIT4 | BIT5;  // Enable analog input on P1.3
  P1DIR &= ~(BIT3 | BIT4 | BIT5); // Set P1.3, P1.4, P1.5 as inputs

  P1SEL |= BIT6 | BIT7;           // I2C: SCL on P1.6, SDA on P1.7
  P1SEL2 |= BIT6 + BIT7;

  // Comms Initialization
  uart_init(); // Initialize UART
  I2C_init(); // Initialize I2C
  I2C_WriteByte(ALS_CONTR, 0x01); // Active mode, gain 1X
  I2C_WriteByte(ALS_MEAS_RATE, 0x12); // 200ms integration & measurement rate

/*
  // Configure the watchdog timer in interval mode.
  // For many MSP430s, WDTIS_3 sets the interval to approximately 8 seconds when using VLO.
  WDTCTL = WDTPW | WDTTMSEL | WDTCNTCL | WDTIS_3;  
  IE1 |= WDTIE;  // Enable WDT interrupt
  __bis_SR_register(GIE); // Enable global interrupts
*/

  __delay_cycles(1000000); // Small delay (1s)
  while (1) {
    // Enter low-power mode (LPM3). The device will wake on the WDT interrupt.
    //__bis_SR_register(LPM3_bits + GIE);

    // Check if 30 minutes have elapsed.
    //if (intervalCount >= 225) {
    //  intervalCount = 0; // Reset the counter
      pump_counter++;
      read_voltage_sensor();
      read_water_level_sensor();
      read_temp_sensor();
      read_light_sensor();

      toggle_zigbee_wake();

      send_xbee_transmit_request(voltage_value, water_level_value, temp_value, light_value);

      // Water every 12 hrs
      if (pump_counter == 24) {
        pump_counter = 0;
        control_pump();
      }
    //}
    __delay_cycles(1000000); // Small delay (1s)
  }
}

/*
// Watchdog Timer ISR - fires approximately every 8 seconds.
#pragma vector=WDT_VECTOR
__interrupt void watchdog_timer(void)
{
    intervalCount++;  // Count the 8-second interval

    // Exit LPM3 so that main() can check the counter.
    __bic_SR_register_on_exit(LPM3_bits);
}
*/