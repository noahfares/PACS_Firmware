#include <msp430.h>
#include <stdint.h>
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
    while (!(IFG2 & UCA0TXIFG))
      ;                  // Wait for TX buffer to be ready
    UCA0TXBUF = data[i]; // Send byte
  }
}

// Calculate XBee Checksum
uint8_t calculate_checksum(uint8_t *frame, uint16_t length) {
  uint8_t sum = 0;
  uint16_t i;                    // Declare loop variable outside for loop
  for (i = 3; i < length; i++) { // Start after length bytes
    sum += frame[i];
  }
  return 0xFF - sum; // Checksum = 0xFF - (Sum of bytes)
}

// Send XBee API Frame (Transmit Request)
void send_xbee_transmit_request(uint16_t var1, uint16_t var2, uint16_t var3,
                                uint16_t var4) {
  char payload[20]; // Enough space for four 5-digit numbers and spaces
  snprintf(payload, sizeof(payload), "%u %u %u %u", var1, var2, var3, var4);
  uint8_t payload_length = strlen(payload);

  uint16_t frame_length = 14 + payload_length;
  uint8_t frame[TX_BUFFER_SIZE] = {0};

  // Build XBee API Frame
  frame[0] = 0x7E;
  frame[1] = (frame_length >> 8) & 0xFF;
  frame[2] = frame_length & 0xFF;
  frame[3] = 0x10;
  frame[4] = 0x01;
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
  frame[15] = 0x00;
  frame[16] = 0x00;

  // Copy Payload
  uint8_t i;
  for (i = 0; i < payload_length; i++) {
    frame[17 + i] = payload[i];
  }

  // Compute Checksum
  uint8_t checksum = calculate_checksum(frame, 17 + payload_length);
  frame[17 + payload_length] = checksum;

  // Send Frame
  uart_send(frame, 18 + payload_length);
}

// Function to read voltage sensor
void read_voltage_sensor() {
  P2OUT |= BIT0;            // Turn on voltage sensor
  __delay_cycles(1000);     // Small delay for stabilization
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
  P2OUT |= BIT2;               // Turn pump on
  __delay_cycles(60000000);    // Run for 1 minute (assuming 1MHz clock)
  P2OUT &= ~BIT2;              // Turn pump off
  __delay_cycles(28800000000); // Off for 8 hours
}

// Main function
int main(void) {
  WDTCTL = WDTPW | WDTHOLD; // Stop watchdog timer

  P2DIR |= BIT0 | BIT2 | BIT5;    // Set P2.0, P2.2, and P2.5 as outputs
  P2DIR &= ~BIT4;                 // Set P2.4 as input
  P1DIR &= ~(BIT3 | BIT4 | BIT5); // Set P1.3, P1.4, P1.5 as inputs

  uart_init(); // Initialize UART

  __delay_cycles(1000000); // Small delay before sending

  send_xbee_transmit_request(10, 10, 10, 10);

      while (1) {
    /*
    read_voltage_sensor();
    read_temp_sensor();
    read_water_level_sensor();
    read_light_sensor();
    toggle_zigbee_wake();
    control_pump();
    */
  }
}
