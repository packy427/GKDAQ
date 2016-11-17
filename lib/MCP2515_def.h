//----------------------------------------------------------
//  MCP2515_def.h
//  Patrick Kennedy
//
//  Macros defined as per MCP2515 Datasheet
//----------------------------------------------------------

#ifndef MCP2515_DEF
// SPI Instructions
#define MCP_RESET   0xc0    // Reset and set to config mode
#define MCP_READ    0x03    // Read from register, make next byte addr
#define MCP_READRXB0ID    0x90  // Read starting from RXB0SIDH
#define MCP_READRXB0DATA  0x92  // Read starting from RXB0D0
#define MCP_READRXB1ID    0x94  // Read starting from RXB1SIDH
#define MCP_READRXB1DATA  0x96  // Read starting from RXB1D0
#define MCP_WRITE   0x02    // Write to register, next bytes addr + data
#define MCP_LOADTXB0ID    0x40  // Write starting from TXB0SIDH
#define MCP_LOADTXB0DATA  0x41  // Write starting from TXB0D0
#define MCP_LOADTXB1ID    0x42  // Write starting from TXB1SIDH
#define MCP_LOADTXB1DATA  0x43  // Write starting from TXB1D0
#define MCP_LOADTXB2ID    0x44  // Write starting from TXB2SIDH
#define MCP_LOADTXB2DATA  0x45  // Write starting from TXB2D0
#define MCP_RTSTXB0   0x81  // Request to send TXB0
#define MCP_RTSTXB1   0x82  // Request to send TXB1
#define MCP_RTSTXB2   0x84  // Request to send TXB2
#define MCP_READSTATUS    0xA0  // Read Tx/Rx status bits of MCP2515
#define MCP_RXSTATUS      0xB0  // Read filter match and message type
#define MCP_BITMODIFY     0x05  // Modify specific bits of certain registers

// Control Registers
#define BFPCTRL   0x0C    // Buffer full interrupt pin control
#define TXRTSCNTRL  0x0D  // Tx request to send control
#define CANSTAT   0x0E    // CAN status
#define CANCTRL   0x0F    // CAN control
#define TEC   0x1C    // Transmit error counter
#define REC   0x1D    // Receive error counter
#define CNF3  0x28    // CAN timing configuration 3
#define CNF2  0x29    // CAN timing configuration 2
#define CNF1  0x2A    // CAN timing configuration 1
#define CANINTE   0x2B    // Control for interrupts on CAN errors
#define CANINTF   0x2C    // Control for interrupts on CAN events
#define EFLG  0x2D    // Flags for CAN errors
#define TXB0CTRL  0x30    // Control for TXB0
#define TXB1CTRL  0x40    // Control for TXB1
#define TXB2CTRL  0x50    // Control for TXB2
#define RXB0CTRL  0x60    // Control for RXB0
#define RXB1CTRL  0x70    // Control for RXB1

// Control bits
#define RTR 6

// CAN Interrupt Control
#define MERRE   7
#define WAKIE   6
#define ERRIE   5
#define TX2IE   4
#define TX1IE   3
#define TX0IE   2
#define RX1IE   1
#define RX0IE   0

// CAN Interrupt Flags
#define MERRF   7
#define WAKIF   6
#define ERRIF   5
#define TX2IF   4
#define TX1IF   3
#define TX0IF   2
#define RX1IF   1
#define RX0IF   0

// Transmit Control
#define ABTF    6
#define MLOA    5
#define TXERR   4
#define TXREQ   3
#define TXP1    1
#define TXP0    0



#endif // MCP2515_DEF

