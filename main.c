#include <msp430.h>
#include <stdint.h>
#include <stdio.h>
#define TX_BUFFER_SIZE 32 // Adjust based on data size

// Light Sensor Reg commands
#define SENSOR_ADDR 0x29
#define ALS_CONTR_REG 0x80
#define ALS_MEAS_RATE_REG 0x85
#define ALS_DATA_CH1_LOW 0x88
#define ALS_DATA_CH1_HIGH 0x89
#define ALS_DATA_CH0_LOW 0x8A
#define ALS_DATA_CH0_HIGH 0x8B

// Global variables for sensor data
volatile int voltage_value = 0;
volatile int temp_value = 0;
volatile int water_level_value = 0;
volatile int light_value = 0;
volatile int zigbee_sleep_state = 1; // 1 = asleep, 0 = awake

//-------------------------------------------LIGHTSENSOR-------------------------------------------
void i2c_init(void) {
  UCB0CTL1 |= UCSWRST;
  UCB0CTL0 = UCMST + UCMODE_3 + UCSYNC;
  UCB0CTL1 = UCSSEL_2 + UCSWRST;
  UCB0BR0 = 12;
  UCB0BR1 = 0;
  UCB0I2CSA = SENSOR_ADDR;
  UCB0CTL1 &= ~UCSWRST;
}

void i2c_write_register(unsigned char reg, unsigned char value) {
  while (UCB0CTL1 & UCTXSTP)
    ;
  UCB0CTL1 |= UCTR + UCTXSTT;
  while (!(IFG2 & UCB0TXIFG))
    ;
  UCB0TXBUF = reg;
  while (!(IFG2 & UCB0TXIFG))
    ;
  UCB0TXBUF = value;
  while (!(IFG2 & UCB0TXIFG))
    ;
  UCB0CTL1 |= UCTXSTP;
  while (UCB0CTL1 & UCTXSTP)
    ;
}

unsigned char i2c_read_register(unsigned char reg) {
  unsigned char data;
  while (UCB0CTL1 & UCTXSTP)
    ;
  UCB0CTL1 |= UCTR + UCTXSTT;
  while (!(IFG2 & UCB0TXIFG))
    ;
  UCB0TXBUF = reg;
  while (!(IFG2 & UCB0TXIFG))
    ;
  UCB0CTL1 &= ~UCTR;
  UCB0CTL1 |= UCTXSTT;
  while (UCB0CTL1 & UCTXSTT)
    ;
  UCB0CTL1 |= UCTXSTP;
  while (!(IFG2 & UCB0RXIFG))
    ;
  data = UCB0RXBUF;
  while (UCB0CTL1 & UCTXSTP)
    ;
  return data;
}

void calculate_lux(unsigned int ch0, unsigned int ch1) {
  double factor = 0.034; // Conversion factor
  light_value = (ch0 > ch1) ? (ch0 - ch1) * factor : 0;
}

void read_light_sensor(void) {
  unsigned int ch1_data, ch0_data;
  unsigned char low, high;

  low = i2c_read_register(ALS_DATA_CH1_LOW);
  high = i2c_read_register(ALS_DATA_CH1_HIGH);
  ch1_data = ((unsigned int)high << 8) | low;
  low = i2c_read_register(ALS_DATA_CH0_LOW);
  high = i2c_read_register(ALS_DATA_CH0_HIGH);
  ch0_data = ((unsigned int)high << 8) | low;

  calculate_lux(ch0_data, ch1_data);
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
  char payload[30]; // Enough space for "90 90 90 90"
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
  ADC10CTL1 = INCH_3;

  __delay_cycles(1000);  // Small delay for stabilization (0.1s)
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
void toggle_zigbee_wake() {
  P2OUT ^= BIT5;                              // Toggle P2.5
  __delay_cycles(5000);                       // Small delay
  P2OUT ^= BIT5;                              // Toggle again
  zigbee_sleep_state = (P2IN & BIT4) ? 1 : 0; // Read sleep indicator (P2.4)
}

// Function to control pump
void control_pump() {
  P2OUT |= BIT2;            // Turn pump on
  __delay_cycles(10000000); // Run for 10 sec
  P2OUT &= ~BIT2;           // Turn pump off
  __delay_cycles(10000000); // Stop for 10 sec
}

// Centralized reading of adc converions
int adc_readout() {
  // Start ADC conversion:
  ADC10CTL0 |= ENC | ADC10SC; // Enable conversion and start conversion

  // Wait for conversion to complete:
  // (ADC10SC automatically clears when conversion is done)
  while (ADC10CTL1 & ADC10BUSY)
    ;

  int adcValue = ADC10MEM;
  return adcValue;
}

// Main function
int main(void) {
  WDTCTL = WDTPW | WDTHOLD; // Stop watchdog timer

  P2DIR |= BIT0 | BIT2 | BIT5;    // Set P2.0, P2.2, and P2.5 as outputs
  P2DIR &= ~BIT4;                 // Set P2.4 as input
  ADC10AE0 |= BIT3| BIT4 | BIT5;  // Enable analog input on P1.3
  P1DIR &= ~(BIT3 | BIT4 | BIT5); // Set P1.3, P1.4, P1.5 as inputs
  P1SEL |= BIT6 + BIT7;           // I2C: SCL on P1.6, SDA on P1.7

  // Configure ADC10:
  // - ADC10SHT_2: 16 x ADC10CLK cycles sample and hold time.
  // - ADC10ON: Turn on ADC10 module.
  ADC10CTL0 = ADC10SHT_2 + ADC10ON;

  uart_init(); // Initialize UART
  // i2c_init(); // Initialize I2C
  // i2c_write_register(ALS_CONTR_REG, 0x01); // Active mode, gain 1X
  // i2c_write_register(ALS_MEAS_RATE_REG, 0x12); // 200ms integration &
  // measurement rate

  __delay_cycles(1000000); // Small delay (1s)

  while (1) {
    read_voltage_sensor();
    read_temp_sensor();
    /*
    read_water_level_sensor();
    read_light_sensor();
    toggle_zigbee_wake();
    */
    send_xbee_transmit_request(voltage_value, water_level_value, temp_value,
                               light_value);
    __delay_cycles(1000000); // Small delay before sending (1s)
    // control_pump();
  }
}
