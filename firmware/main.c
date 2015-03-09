/*
 * File:   main.c
 * Author: ufanders
 *
 * Created on February 10, 2015, 4:15 PM
 */

#include "xc.h"
#include <plib.h>

// PIC32MX120F032B Configuration Bit Settings

// DEVCFG3
// USERID = No Setting
#pragma config PMDL1WAY = OFF           // Peripheral Module Disable Configuration (Allow multiple reconfigurations)
#pragma config IOL1WAY = OFF            // Peripheral Pin Select Configuration (Allow multiple reconfigurations)
#pragma config FUSBIDIO = ON            // USB USID Selection (Controlled by the USB Module)
#pragma config FVBUSONIO = ON           // USB VBUS ON Selection (Controlled by USB Module)

// DEVCFG2
#pragma config FPLLIDIV = DIV_2         // PLL Input Divider (2x Divider)
#pragma config FPLLMUL = MUL_24         // PLL Multiplier (24x Multiplier)
#pragma config FPLLODIV = DIV_1         // System PLL Output Clock Divider (PLL Divide by 1)

// DEVCFG1
#pragma config FNOSC = FRCPLL           // Oscillator Selection Bits (Fast RC Osc with PLL)
#pragma config FSOSCEN = ON             // Secondary Oscillator Enable (Enabled)
#pragma config IESO = ON                // Internal/External Switch Over (Enabled)
#pragma config POSCMOD = OFF            // Primary Oscillator Configuration (Primary osc disabled)
#pragma config OSCIOFNC = OFF           // CLKO Output Signal Active on the OSCO Pin (Disabled)
#pragma config FPBDIV = DIV_1           // Peripheral Clock Divisor (Pb_Clk is Sys_Clk/1)
#pragma config FCKSM = CSDCMD           // Clock Switching and Monitor Selection (Clock Switch Disable, FSCM Disabled)
#pragma config WDTPS = PS1048576        // Watchdog Timer Postscaler (1:1048576)
#pragma config WINDIS = OFF             // Watchdog Timer Window Enable (Watchdog Timer is in Non-Window Mode)
#pragma config FWDTEN = ON              // Watchdog Timer Enable (WDT Enabled)
#pragma config FWDTWINSZ = WISZ_25      // Watchdog Timer Window Size (Window Size is 25%)

// DEVCFG0
#pragma config JTAGEN = OFF             // JTAG Enable (JTAG Disabled)
#pragma config ICESEL = ICS_PGx1        // ICE/ICD Comm Channel Select (Communicate on PGEC1/PGED1)
#pragma config PWP = OFF                // Program Flash Write Protect (Disable)
#pragma config BWP = OFF                // Boot Flash Write Protect bit (Protection Disabled)
#pragma config CP = OFF                 // Code Protect (Protection Disabled)

#define SYS_FREQ (48000000L)
#define	GetPeripheralClock()		(SYS_FREQ/(1 << OSCCONbits.PBDIV))
#define	GetInstructionClock()		(SYS_FREQ)

typedef enum {
    IDLE,
    RECEIVE_WAITING, RECEIVE_INCOMING, RECEIVE_FINISHED,
    VERIFY_FINISHED,
    APPLY_FINISHED
} state_t;

state_t packetState;

typedef struct
{
    char sync[2]; //0xAA, 0x55
    char address;
    char msgType;
    //char payload[25*7]; //one byte per dot
    char payload[22]; //one bit per dot
    char checksum; //all bytes inclusive of and after msgType XORed with each other?

} protestSignMsg_t;

protestSignMsg_t incoming;

char packetServiceNeededFlag, packetStaleFlag;
char display[22];

int main(void)
{
    SYSTEMConfig(SYS_FREQ, SYS_CFG_WAIT_STATES | SYS_CFG_PCACHE);
    // Explorer-16 LEDs are on lower 8-bits of PORTA and to use all LEDs, JTAG port must be disabled.
    mJTAGPortEnable(DEBUG_JTAGPORT_OFF);
    
    //init display refresh timer
    OpenTimer1(T1_ON | T1_SOURCE_INT | T1_PS_1_256, 223); //~120Hz for 7 rows

    // Set up the timer interrupt with a priority of 2
    INTEnable(INT_T1, INT_ENABLED);
    INTSetVectorPriority(INT_TIMER_1_VECTOR, INT_PRIORITY_LEVEL_2);
    INTSetVectorSubPriority(INT_TIMER_1_VECTOR, INT_SUB_PRIORITY_LEVEL_0);

    //init stale packet timer
    // Set up the timer interrupt with a priority of 2
    INTEnable(INT_T2, INT_ENABLED);
    INTSetVectorPriority(INT_TIMER_2_VECTOR, INT_PRIORITY_LEVEL_2);
    INTSetVectorSubPriority(INT_TIMER_2_VECTOR, INT_SUB_PRIORITY_LEVEL_0);
    
    //init uart
    // Configure UART module, set buad rate, turn on UART, etc.
    UARTConfigure(UART1, UART_ENABLE_PINS_TX_RX_ONLY);
    UARTSetFifoMode(UART1, /*UART_INTERRUPT_ON_TX_NOT_FULL |*/ UART_INTERRUPT_ON_RX_NOT_EMPTY);
    UARTSetLineControl(UART1, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
    UARTSetDataRate(UART1, GetPeripheralClock(), 250000);
    UARTEnable(UART1, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));

    // Configure UART RX Interrupt, used for first byte during reception
    INTSetVectorPriority(INT_VECTOR_UART(UART1), INT_PRIORITY_LEVEL_2);
    INTSetVectorSubPriority(INT_VECTOR_UART(UART1), INT_SUB_PRIORITY_LEVEL_0);

    //init DMA
    //================DMA CHANNEL 1 START=======================================
    // configure the channel
    DmaChnOpen(DMA_CHANNEL1, DMA_CHN_PRI2, DMA_OPEN_ENABLE);

    //DMA 1 cell transfer starts upon UART1 RX interrupt
    DmaChnSetEventControl(DMA_CHANNEL1, DMA_EV_START_IRQ_EN | DMA_EV_START_IRQ(_UART1_RX_IRQ));

    //From UART1 RX to incoming[], 1 byte per transfer trigger until whole packet is received
    DmaChnSetTxfer(DMA_CHANNEL1, (void*)&U1RXREG, &incoming, \
        1, sizeof(incoming), 1);

    DmaChnSetEvEnableFlags(DMA_CHANNEL1, DMA_EV_BLOCK_DONE); // enable the transfer done interrupt

    INTSetVectorPriority(INT_VECTOR_DMA(DMA_CHANNEL1), INT_PRIORITY_LEVEL_5);
    INTSetVectorSubPriority(INT_VECTOR_DMA(DMA_CHANNEL1), INT_SUB_PRIORITY_LEVEL_3);

    INTEnable(INT_SOURCE_DMA(DMA_CHANNEL1), INT_ENABLED);
    //================DMA CHANNEL 1 END=========================================

    //================DMA CHANNEL 2 START=======================================
    DmaChnOpen(DMA_CHANNEL2, DMA_CHN_PRI2, DMA_OPEN_ENABLE);

    //DMA 2 cell transfer starts upon DMA 1 DMA_EV_BLOCK_DONE interrupt
    DmaChnSetEventControl(DMA_CHANNEL2, DMA_EV_START_IRQ_EN | DMA_EV_START_IRQ(_DMA0_IRQ));

    //DMA 2 cell transfer starts upon manual start
    //DmaChnSetEventControl(DMA_CHANNEL2, DMA_EV_START_IRQ_EN);

    //From incoming.payload[] to display[], whole payload is transferred at once
    DmaChnSetTxfer(DMA_CHANNEL2, incoming.payload, display, \
        sizeof(incoming.payload), sizeof(display), sizeof(incoming.payload));

    DmaChnSetEvEnableFlags(DMA_CHANNEL2, DMA_EV_BLOCK_DONE);

    //enable checksum machine
    DmaSfmConfigure(DMA_CHKSUM_IP, DMA_BITO_MSb, DMA_REORDER_NOT);
    DmaSfmAttachChannel(DMA_CHANNEL2, TRUE);
    //TODO: how do we clear out the checksum value before the next transfer?

    INTSetVectorPriority(INT_VECTOR_DMA(DMA_CHANNEL2), INT_PRIORITY_LEVEL_5);
    INTSetVectorSubPriority(INT_VECTOR_DMA(DMA_CHANNEL2), INT_SUB_PRIORITY_LEVEL_3);

    INTEnable(INT_SOURCE_DMA(DMA_CHANNEL2), INT_ENABLED);
    //================DMA CHANNEL 2 END=========================================

    packetState = RECEIVE_WAITING;
    packetServiceNeededFlag = 0;
    packetStaleFlag = 0;
    
    DmaChnEnable(DMA_CHANNEL1);
    DmaChnEnable(DMA_CHANNEL2);
    INTEnable(INT_SOURCE_UART_RX(UART1), INT_ENABLED);
    
    // Enable multi-vector interrupts
    INTConfigureSystem(INT_SYSTEM_CONFIG_MULT_VECTOR);
    INTEnableInterrupts();

    while(1)
    {
        if(packetServiceNeededFlag)
        {
            switch(packetState)
            {
                case IDLE:
                    INTEnable(INT_SOURCE_UART_RX(UART1), INT_ENABLED);
                    packetState = RECEIVE_WAITING;
                    break;
                    
                case RECEIVE_WAITING:
                    //we have started receiving a packet, start timer.
                    OpenTimer2(T2_ON | T2_SOURCE_INT | T2_PS_1_256, 46875); // 0.250 sec
                    break;
                    
                case RECEIVE_INCOMING:
                    //TODO: if timer has expired (due to error or whatever),
                    //reset receiver.
                    if(packetStaleFlag)
                    {
                        CloseTimer1();

                        //TODO: reset receiver

                        packetStaleFlag = 0;
                    }

                    break;

                case RECEIVE_FINISHED:
                    //TODO: run all checksummed data through DMA CRC machine using second
                    //DMA channel. NOTE: this should be automatic.
                    
                    /*
                    //begin transfer and thus the checksum calculation
                    char res = DmaChnStartTxfer(DMA_CHANNEL2, DMA_WAIT_NOT, 0);
                    if(res != DMA_TXFER_OK)
                    {
                        return 0;
                    }
                    */

                    break;

                case VERIFY_FINISHED:
                    //TODO: If checksum is ok, copy payload to frame buffer via DMA
                    if((int)(incoming.checksum) == DmaSfmChecksum())
                    {
                        //checksum is ok, apply data to display
                        //TODO: set pointer from ping buffer to pong buffer
                        
                        
                    }

                    packetState = RECEIVE_FINISHED;

                    break;

                default:
                    break;
            }
            
            packetServiceNeededFlag = 0; //re-arm reception handler
        }
    }

    return 0;
}

//TODO: RS-485 interrupt
// handler for the DMA channel 1 interrupt
void __ISR(_DMA1_VECTOR, ipl5) DmaHandler1(void)
{
    int evFlags; // event flags when getting the interrupt

    INTClearFlag(INT_SOURCE_DMA(DMA_CHANNEL1));

    evFlags = DmaChnGetEvFlags(DMA_CHANNEL1); // get the DMA event flags

    if (evFlags & DMA_EV_BLOCK_DONE)
    {
        DmaChnClrEvFlags(DMA_CHANNEL1, DMA_EV_BLOCK_DONE);

        packetState = RECEIVE_FINISHED;
        packetServiceNeededFlag = 1;      
    }
}

// handler for the DMA channel 2 interrupt
void __ISR(_DMA2_VECTOR, ipl5) DmaHandler2(void)
{
    int evFlags; // event flags when getting the interrupt

    INTClearFlag(INT_SOURCE_DMA(DMA_CHANNEL2));

    evFlags = DmaChnGetEvFlags(DMA_CHANNEL2); // get the DMA event flags

    if (evFlags & DMA_EV_BLOCK_DONE)
    {
        DmaChnClrEvFlags(DMA_CHANNEL2, DMA_EV_BLOCK_DONE);

        packetState = VERIFY_FINISHED;
        packetServiceNeededFlag = 1;
    }
}

//TODO: no-data reception reset timer interrupt
void __ISR(_TIMER_2_VECTOR, ipl2) Timer2Handler(void)
{
    // Clear the interrupt flag
    INTClearFlag(INT_T2);
    
    packetStaleFlag = 1;

    //TODO: set application flag
    packetServiceNeededFlag = 1;
}

// UART 1 interrupt handler, set at priority level 2
void __ISR(_UART_1_VECTOR, ipl2) IntUart1Handler(void)
{
    // Is this an RX interrupt?
    if(INTGetFlag(INT_SOURCE_UART_RX(UART1)))
    {
        // Clear the RX interrupt Flag
        INTClearFlag(INT_SOURCE_UART_RX(UART1));

        //kill interrupt until we are ready to start a new packet
        INTEnable(INT_SOURCE_UART_RX(UART1), INT_DISABLED);

        packetState = RECEIVE_INCOMING;
    }
}

//TODO: refresh timer interrupt
// Configure the Timer 1 interrupt handler
void __ISR(_TIMER_1_VECTOR, ipl2) Timer1Handler(void)
{
    // Clear the interrupt flag
    INTClearFlag(INT_T1);

    //TODO: set application flag to perform next refresh
}
