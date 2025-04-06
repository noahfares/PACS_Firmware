#include <msp430.h>
#include <stdint.h>
#include <stdio.h>
#define TX_BUFFER_SIZE 32 // Adjust based on data size

// Global variables for sensor data
volatile uint16_t voltage_value = 0;
volatile uint16_t temp_value = 0;
volatile uint16_t water_level_value = 0;
volatile uint16_t light_value = 0;
volatile uint8_t zigbee_sleep_state = 1; // 1 = asleep, 0 = awake

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
    while (!(IFG2 & UCA0TXIFG));                  // Wait for TX buffer to be ready
    UCA0TXBUF = data[i]; // Send byte
  }
}

// Send XBee API Frame (Transmit Request)
void send_xbee_transmit_request(int var1, int var2, int var3,
                                int var4) {
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

// Function to read voltage sensor
void read_voltage_sensor() {
  P2OUT |= BIT0;            // Turn on voltage sensor
  __delay_cycles(1000000);     // Small delay for stabilization
  voltage_value = ADC10MEM; // Read ADC value from P1.4
  P2OUT &= ~BIT0;           // Turn off voltage sensor
}

// Function to read temperature sensor
void read_temp_sensor() {
  temp_value = ADC10MEM; // Read ADC value from P1.3
}

// Function to read water level sensor
void read_water_level_sensor() {
  water_level_value = ADC10MEM; // Read ADC value from P1.5
}

// Function to wake Zigbee
void toggle_zigbee_wake() {
  P2OUT ^= BIT5;                              // Toggle P2.5
  __delay_cycles(5000);                       // Small delay
  P2OUT ^= BIT5;                              // Toggle again
  zigbee_sleep_state = (P2IN & BIT4) ? 1 : 0; // Read sleep indicator (P2.4)
}

// Function to read light sensor via I2C
void read_light_sensor() {
  UCB0I2CSA = 0x29;           // Slave address
  UCB0CTL1 |= UCTR + UCTXSTT; // Transmit mode and start condition
  while (!(IFG2 & UCB0TXIFG))
    ;               // Wait for TX buffer ready
  UCB0TXBUF = 0x53; // Send read address
  while (UCB0CTL1 & UCTXSTT)
    ;                  // Wait for transmission to complete
  UCB0CTL1 &= ~UCTR;   // Set to receive mode
  UCB0CTL1 |= UCTXSTT; // Repeated start condition
  while (!(IFG2 & UCB0RXIFG))
    ;                      // Wait for RX buffer ready
  light_value = UCB0RXBUF; // Read received data
  UCB0CTL1 |= UCTXSTP;     // Send stop condition
}

// Function to control pump
void control_pump() {
  P2OUT |= BIT2;            // Turn pump on
  __delay_cycles(10000000); // Run for 10 sec
  P2OUT &= ~BIT2;           // Turn pump off
  __delay_cycles(10000000); // Stop for 10 sec
}

// Main function
int main(void) {
  WDTCTL = WDTPW | WDTHOLD; // Stop watchdog timer

  P2DIR |= BIT0 | BIT2 | BIT5;    // Set P2.0, P2.2, and P2.5 as outputs
  P2DIR &= ~BIT4;                 // Set P2.4 as input
  P1DIR &= ~(BIT3 | BIT4 | BIT5); // Set P1.3, P1.4, P1.5 as inputs

  uart_init(); // Initialize UART

  __delay_cycles(1000000); // Small delay before sending (1s)

  while (1) {
    read_voltage_sensor();
    read_water_level_sensor();
    read_temp_sensor();
    read_light_sensor();
    /*
    toggle_zigbee_wake();
    */
    send_xbee_transmit_request(voltage_value, water_level_value, temp_value, light_value);
    __delay_cycles(1000000); // Small delay before sending (1s)
    //control_pump();
  }
}
