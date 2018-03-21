/*
  test 001
  _____                              _
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2017 Semtech

Description: Main program

Maintainer: Gregory Cristian & Gilbert Menth
*/

#include "mbed.h"
#include "radio.h"
#include "sx1280-hal.h"


#define dbgBLE	1
void dbgPrintfHex(uint8_t dbgType, uint8_t *format, uint8_t length);

/*!
 * \brief Used to display firmware version on RS232
 */
#define FIRMWARE_VERSION ( ( char* )"Firmware Version: 170530" )

/*!
 * Use the Ping Ping in uncommented Mode
 */
//#define MODE_BLE
#define MODE_LORA
//#define MODE_GENERIC
//#define MODE_FLRC


/*!
 * \brief Defines the nominal frequency
 */
#define RF_FREQUENCY                                2425000000UL // Hz

/*!
 * \brief Defines the output power in dBm
 *
 * \remark The range of the output power is [-18..+13] dBm
 */
#define TX_OUTPUT_POWER                          13

/*!
 * \brief Defines the states of the application
 */
typedef enum
{
    APP_LOWPOWER,
    APP_RX,
    APP_RX_TIMEOUT,
    APP_RX_ERROR,
    APP_TX,
    APP_TX_TIMEOUT,
}AppStates_t;

/*!
 * \brief Defines the buffer size, i.e. the payload size
 */
#ifdef MODE_LORA
#define BUFFER_SIZE                                 30//FLRC MAX=127, LORA MAX=255
#else 
#define BUFFER_SIZE                                 65//FLRC MAX=127, LORA MAX=255
#endif
#define MAX_BUFFER_SIZE                             BUFFER_SIZE


/*!
 * \brief Define the possible message type for this application
 */
const uint8_t PingMsg[] = "PING";
const uint8_t PongMsg[] = "PONG";

/*!
 * \brief Defines the size of the token defining message type in the payload
 */
#define PINGPONGSIZE                    4

/*!
 * \brief The size of the buffer
 */
uint8_t BufferSize = BUFFER_SIZE;

/*!
 * \brief The buffer
 */
uint8_t Buffer[MAX_BUFFER_SIZE];

/*!
 * \brief The State of the application
 */
AppStates_t AppState = APP_LOWPOWER;

int8_t RssiValue = 0;
int8_t SnrValue = 0;

/*!
 * \brief Function to be executed on Radio Tx Done event
 */
void OnTxDone( void );

/*!
 * \brief Function to be executed on Radio Rx Done event
 */
void OnRxDone( void );

/*!
 * \brief Function executed on Radio Tx Timeout event
 */
void OnTxTimeout( void );

/*!
 * \brief Function executed on Radio Rx Timeout event
 */
void OnRxTimeout( void );

/*!
 * \brief Function executed on Radio Rx Error event
 */
void OnRxError( IrqErrorCode_t );


extern unsigned int myCRC16_Check (unsigned char* pchMsg, unsigned int wDataLen);
/*!
 * \brief All the callbacks are stored in a structure
 */
RadioCallbacks_t callbacks =
{
    &OnTxDone,        // txDone
    &OnRxDone,        // rxDone
    NULL,             // syncWordDone
    NULL,             // headerDone
    &OnTxTimeout,     // txTimeout
    &OnRxTimeout,     // rxTimeout
    &OnRxError,       // rxError
    NULL,             // rangingDone
    NULL,             // cadDone
};

// mosi, miso, sclk, nss, busy, dio1, dio2, dio3, rst, callbacks...
SX1280Hal Radio( D11, D12, D13, D7, D3, D5, NC, NC, A0, &callbacks );

DigitalOut ANT_SW( A3 );
DigitalOut TxLed( A4 );
DigitalOut RxLed( A5 );

/*!
 * \brief Define IO for Unused Pin
 */
DigitalOut F_CS( D6 );      // MBED description of pin
DigitalOut SD_CS( D8 );     // MBED description of pin

/*!
 * \brief Number of tick size steps for tx timeout
 */
#define TX_TIMEOUT_VALUE                            100 // ms

/*!
 * \brief Number of tick size steps for rx timeout
 */
#define RX_TIMEOUT_VALUE                            0xffff// 3000 // ms

/*!
 * \brief Size of ticks (used for Tx and Rx timeout)
 */
#define RX_TIMEOUT_TICK_SIZE                        RADIO_TICK_SIZE_1000_US

/*!
 * \brief Mask of IRQs to listen to in rx mode
 */
uint16_t RxIrqMask = IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT;

/*!
 * \brief Mask of IRQs to listen to in tx mode
 */
uint16_t TxIrqMask = IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT;

/*!
 * \brief Locals parameters and status for radio API
 * NEED TO BE OPTIMIZED, COPY OF STUCTURE ALREADY EXISTING
 */
PacketParams_t PacketParams;
PacketStatus_t PacketStatus;

//===============================UART START=====================================
typedef void (*pfunc)(void);
#define UART_RX_BUFFER_SIZE     256
uint8_t uartRxBuf[UART_RX_BUFFER_SIZE];
uint16_t uartRxLen=0,uartRxTOCnt=0;

Serial pc( USBTX, USBRX );

void Rx_interrupt() 
{
    //pc.putc(pc.getc());  
    if(uartRxLen < UART_RX_BUFFER_SIZE){
        uartRxBuf[uartRxLen] = pc.getc();
        uartRxLen ++;
        uartRxTOCnt = 0;
    }
    return;
}

void baud( int baudrate )
{
    pc.baud( baudrate );
    pc.attach(&Rx_interrupt, Serial::RxIrq);
    uartRxLen=0;
    uartRxTOCnt=0;
}

void UartRxProcess(pfunc func)
{
    if(uartRxLen == 0) return;
    if(uartRxTOCnt++ > 2000) {
        func();
        uartRxTOCnt = 0;
        uartRxLen = 0;
    }
}

///
void UartRxDoneTest(void)
{
    dbgPrintfHex(1, uartRxBuf, uartRxLen);
}



//===============================UART END=====================================

Timer DelayTimer;
int begin, end;

/*	  timer.start();
*	  begin = DelayTimer.read_us();
*	  led = !led;
*	  end = DelayTimer.read_us();
*	  printf("Toggle the led takes %d us", end - begin);
*/
RadioPacketTypes_t ptype;
void RadioInit(void)
{
	ModulationParams_t modulationParams;

    Radio.Init( );
	
	////test hoperf
	ptype = Radio.GetPacketType(0);
	Radio.SetPacketType( PACKET_TYPE_BLE );
	ptype = Radio.GetPacketType(0);
	
	
    Radio.SetRegulatorMode( USE_DCDC ); // Can also be set in LDO mode but consume more power

    //memset( &Buffer, 0x00, BufferSize );
    //printf( "\n\n\r     SX1280 Ping Pong Demo Application (%s)\n\n\r", FIRMWARE_VERSION );

#if defined( MODE_BLE )
    printf( "\nPing Pong running in BLE mode\n\r");
    modulationParams.PacketType                   = PACKET_TYPE_BLE;
    modulationParams.Params.Ble.BitrateBandwidth  = GFSK_BLE_BR_1_000_BW_2_4;//GFSK_BLE_BR_0_125_BW_0_3;//GFSK_BLE_BR_1_000_BW_2_4;
    modulationParams.Params.Ble.ModulationIndex   = GFSK_BLE_MOD_IND_1_00;
    modulationParams.Params.Ble.ModulationShaping = RADIO_MOD_SHAPING_BT_1_0;

    PacketParams.PacketType                 = PACKET_TYPE_BLE;
    PacketParams.Params.Ble.BleTestPayload  = BLE_EYELONG_1_0;
    PacketParams.Params.Ble.ConnectionState = BLE_ADVERTISER;///BLE_ADVERTISER;//BLE_MASTER_SLAVE;
    PacketParams.Params.Ble.CrcLength       = BLE_CRC_3B;
    PacketParams.Params.Ble.Whitening       = RADIO_WHITENING_OFF;

#elif defined( MODE_GENERIC )

    printf( "\nPing Pong running in GENERIC mode\n\r");
    modulationParams.PacketType                    = PACKET_TYPE_GFSK;
    modulationParams.Params.Gfsk.BitrateBandwidth  = GFSK_BLE_BR_1_000_BW_1_2;//GFSK_BLE_BR_0_125_BW_0_3;//GFSK_BLE_BR_1_000_BW_2_4
    modulationParams.Params.Gfsk.ModulationIndex   = GFSK_BLE_MOD_IND_0_50;
    modulationParams.Params.Gfsk.ModulationShaping = RADIO_MOD_SHAPING_BT_0_5;

    PacketParams.PacketType                     = PACKET_TYPE_GFSK;
    PacketParams.Params.Gfsk.PreambleLength     = PREAMBLE_LENGTH_32_BITS;
    PacketParams.Params.Gfsk.SyncWordLength     = GFSK_SYNCWORD_LENGTH_5_BYTE;//GFSK_SYNCWORD_LENGTH_5_BYTE
    PacketParams.Params.Gfsk.SyncWordMatch      = RADIO_RX_MATCH_SYNCWORD_1;//RADIO_RX_MATCH_SYNCWORD_1;
    PacketParams.Params.Gfsk.HeaderType         = RADIO_PACKET_VARIABLE_LENGTH;//RADIO_PACKET_VARIABLE_LENGTH;
    PacketParams.Params.Gfsk.PayloadLength      = BUFFER_SIZE; ///15
    PacketParams.Params.Gfsk.CrcLength          = RADIO_CRC_3_BYTES;
    PacketParams.Params.Gfsk.Whitening          = RADIO_WHITENING_OFF;


#elif defined( MODE_LORA )

    printf( "\nPing Pong running in LORA mode\n\r");
    modulationParams.PacketType                  = PACKET_TYPE_LORA;
    modulationParams.Params.LoRa.SpreadingFactor = LORA_SF5;
    modulationParams.Params.LoRa.Bandwidth       = LORA_BW_1600;
    modulationParams.Params.LoRa.CodingRate      = LORA_CR_4_5;

    PacketParams.PacketType                 = PACKET_TYPE_LORA;
    PacketParams.Params.LoRa.PreambleLength = 0x0C;
    PacketParams.Params.LoRa.HeaderType     = LORA_PACKET_IMPLICIT;
    PacketParams.Params.LoRa.PayloadLength  = BUFFER_SIZE;
    PacketParams.Params.LoRa.Crc            = LORA_CRC_ON;
    PacketParams.Params.LoRa.InvertIQ       = LORA_IQ_NORMAL;

#elif defined( MODE_FLRC )

    printf( "\nPing Pong running in FLRC mode\n\r");
    modulationParams.PacketType                    = PACKET_TYPE_FLRC;
    modulationParams.Params.Flrc.BitrateBandwidth  = FLRC_BR_0_650_BW_0_6;
    modulationParams.Params.Flrc.CodingRate        = FLRC_CR_1_0;
    modulationParams.Params.Flrc.ModulationShaping = RADIO_MOD_SHAPING_BT_OFF;

    PacketParams.PacketType                 = PACKET_TYPE_FLRC;
    PacketParams.Params.Flrc.PreambleLength = PREAMBLE_LENGTH_32_BITS;
    PacketParams.Params.Flrc.SyncWordLength = FLRC_SYNCWORD_LENGTH_4_BYTE;
    PacketParams.Params.Flrc.SyncWordMatch  = RADIO_RX_MATCH_SYNCWORD_1;
    PacketParams.Params.Flrc.HeaderType     = RADIO_PACKET_VARIABLE_LENGTH;
    PacketParams.Params.Flrc.PayloadLength  = BUFFER_SIZE;//15;
    PacketParams.Params.Flrc.CrcLength      = RADIO_CRC_3_BYTES;//RADIO_CRC_3_BYTES;
    PacketParams.Params.Flrc.Whitening      = RADIO_WHITENING_OFF;//In FLRC packet type, it is not possible to enable whitening. You must always set the value of packetParam7 to disabled

#else
#error "Please select the mode of operation for the Ping Ping demo"
#endif

    Radio.SetStandby( STDBY_RC );
    Radio.SetPacketType( modulationParams.PacketType );

	
    //if( GetPacketType( true ) == PACKET_TYPE_RANGING )
	ptype = Radio.GetPacketType(0);//test hoperf
	

	
    Radio.SetModulationParams( &modulationParams );
    Radio.SetPacketParams( &PacketParams );

    Radio.SetRfFrequency( RF_FREQUENCY );
    Radio.SetBufferBaseAddresses( 0x00, 0x00 );
    Radio.SetTxParams( TX_OUTPUT_POWER, RADIO_RAMP_20_US );

    // only used in GENERIC and BLE mode
    Radio.SetSyncWord( 1, ( uint8_t[] ){ 0xDD, 0xA0, 0x96, 0x69, 0xDD } );
	//Radio.SetSyncWord( 1, ( uint8_t[] ){ 0xD6, 0xBE, 0x89, 0x8E, 0xDD } );
}

uint32_t txPktCnt=0;
int main( )
{
    bool isMaster = 0;
	int	 TxIntervalMs = 3000;//only for master
	DigitalIn mybutton(USER_BUTTON);

    uint8_t txLen = BUFFER_SIZE; ///tx data length for test
    uint16_t tmpCrc,RxCrc;
    uint32_t curCnt = 0, preCnt = 0, lostCnt = 0, rxCnt = 0, PER = 0;
    uint32_t CrcErrCnt = 0;

	
	uint32_t tmpCnt=0;
	RadioStatus_t radiostatus;

	
    ModulationParams_t modulationParams;

    baud( 115200 );

    F_CS   = 1;
    SD_CS  = 1;
    RxLed  = 1;
    TxLed  = 1;
    ANT_SW = 1;

    wait_ms( 500 ); // wait for on board DC/DC start-up time
	DelayTimer.start();
    memset( &Buffer, 0x00, BufferSize );

    printf( "\n\n\r     SX1280 Ping Pong Demo Application (%s)\n\n\r", FIRMWARE_VERSION );
	
	RadioInit();

		
    RxLed = 0;
    TxLed = 0;

	if( isMaster == true ){
		begin = DelayTimer.read_ms();
	}
	else{
    	Radio.SetDioIrqParams( RxIrqMask, RxIrqMask, IRQ_RADIO_NONE, IRQ_RADIO_NONE );
    	Radio.SetRx( ( TickTime_t ) { RX_TIMEOUT_TICK_SIZE, RX_TIMEOUT_VALUE } );
	}
    AppState = APP_LOWPOWER;

    while( 1 )
    {
        UartRxProcess(UartRxDoneTest);
        switch( AppState )
        {
            case APP_RX:
                AppState = APP_LOWPOWER;
                RxLed = !RxLed;
                Radio.GetPayload( Buffer, &BufferSize, MAX_BUFFER_SIZE );
                
                tmpCrc = myCRC16_Check(Buffer, (uint16_t)(BufferSize-2));
                RxCrc = ((uint16_t)Buffer[BufferSize-2]&0x00FF) + (((uint16_t)Buffer[BufferSize-1]<<8)&0xFF00);
                if(tmpCrc != RxCrc){
                    CrcErrCnt += 1;
                    //printf("software CRC error\r\n");
                    //dbgPrintfHex(dbgBLE, Buffer, BufferSize);
                    break;
                }
				//printf( "RxData: " );
				//dbgPrintfHex(dbgBLE, Buffer, BufferSize);
				//printf( "Rx %d\r\n", ((uint32_t)Buffer[0]<<24)|((uint32_t)Buffer[1]<<16)|((uint32_t)Buffer[2]<<8)|((uint32_t)Buffer[3]));
                if( isMaster == true )
                {
					//begin = DelayTimer.read_ms();
                }
                else
                {
                    curCnt = ((uint32_t)Buffer[0]<<24)|((uint32_t)Buffer[1]<<16)|((uint32_t)Buffer[2]<<8)|((uint32_t)Buffer[3]);
                    if((rxCnt == 0)||(curCnt <= preCnt)){
                        preCnt = curCnt; lostCnt = 0; rxCnt = 1; PER = 0; CrcErrCnt = 0;
						dbgPrintfHex(dbgBLE, Buffer, BufferSize);
                        printf("rxCnt lostCnt CrcErrCnt PER‰ Rssi Snr\r\n");     
                        printf("%d %d %d %d‰ %d %d\r\n",rxCnt,lostCnt,CrcErrCnt,PER,RssiValue,SnrValue);                        
                    }
                    else{
                        lostCnt += curCnt - preCnt - 1;
                        preCnt = curCnt;
                        rxCnt += 1;
                        if(rxCnt%200==0){
                            PER = lostCnt * 1000 / (rxCnt+lostCnt);
                            printf("%d %d %d %d‰ %d %d\r\n",rxCnt,lostCnt,CrcErrCnt,PER,RssiValue,SnrValue);
                        }
                    }
                    
                	//Radio.SetBufferBaseAddresses( 0x00, 0x00 );///
					//Radio.SetDioIrqParams( RxIrqMask, RxIrqMask, IRQ_RADIO_NONE, IRQ_RADIO_NONE );
					//Radio.SetRx( ( TickTime_t ) { RX_TIMEOUT_TICK_SIZE, RX_TIMEOUT_VALUE } );
                }
                break;

            case APP_TX:
                AppState = APP_LOWPOWER;
                TxLed = !TxLed;
                if( isMaster == true )
                {
                    //printf( "Ping...\r\n" );
                }
                else
                {
                    //printf( "Pong...\r\n" );
                }
                //Radio.SetDioIrqParams( RxIrqMask, RxIrqMask, IRQ_RADIO_NONE, IRQ_RADIO_NONE );
                //Radio.SetRx( ( TickTime_t ) { RX_TIMEOUT_TICK_SIZE, RX_TIMEOUT_VALUE } );
                break;

            case APP_RX_TIMEOUT:
                AppState = APP_LOWPOWER;
                if( isMaster == true )
                {
                }
                else
                {
                	//RadioInit();
                	Radio.SetBufferBaseAddresses( 0x00, 0x00 );///
                    Radio.SetDioIrqParams( RxIrqMask, RxIrqMask, IRQ_RADIO_NONE, IRQ_RADIO_NONE );
                    Radio.SetRx( ( TickTime_t ) { RX_TIMEOUT_TICK_SIZE, RX_TIMEOUT_VALUE } );
                }
                break;

            case APP_RX_ERROR:
				AppState = APP_LOWPOWER;
				Radio.SetBufferBaseAddresses( 0x00, 0x00 );///
				Radio.SetDioIrqParams( RxIrqMask, RxIrqMask, IRQ_RADIO_NONE, IRQ_RADIO_NONE );
				Radio.SetRx( ( TickTime_t ) { RX_TIMEOUT_TICK_SIZE, RX_TIMEOUT_VALUE } );
				break;
				
            case APP_TX_TIMEOUT:
                AppState = APP_LOWPOWER;
				Radio.SetBufferBaseAddresses( 0x00, 0x00 );///
                Radio.SetDioIrqParams( RxIrqMask, RxIrqMask, IRQ_RADIO_NONE, IRQ_RADIO_NONE );
                Radio.SetRx( ( TickTime_t ) { RX_TIMEOUT_TICK_SIZE, RX_TIMEOUT_VALUE } );
                break;

            case APP_LOWPOWER:
                #if 0
				if( isMaster == true ){
					end = DelayTimer.read_ms();
					if(end - begin >= TxIntervalMs){
						begin = DelayTimer.read_ms();
						for(uint8_t i=1; i<40; i++) Buffer[i]=i;
						Buffer[0] = 0x83;
						Buffer[1] = 20;//0x24;  //83 0C SCAN_REQ
	                    //memcpy( Buffer, PingMsg, PINGPONGSIZE );
	                    Radio.SetDioIrqParams( TxIrqMask, TxIrqMask, IRQ_RADIO_NONE, IRQ_RADIO_NONE );
	                    Radio.SendPayload( Buffer, Buffer[1], ( TickTime_t ){ RX_TIMEOUT_TICK_SIZE, TX_TIMEOUT_VALUE } );
					}
				}
				#endif

                #if 1
				if (mybutton == 0) { // Button is pressed
					while(mybutton==0);
			      	isMaster = 1;
			      	printf("isMaster=1\r\n");
			      	
                    txPktCnt=1;
                    for(uint8_t i=0; i<txLen; i++) Buffer[i]=i;
                    Buffer[0] = (uint8_t)(txPktCnt>>24);
                    Buffer[1] = (uint8_t)(txPktCnt>>16);
                    Buffer[2] = (uint8_t)(txPktCnt>>8);
                    Buffer[3] = (uint8_t)(txPktCnt);
                    
                    //add sw crc
                    tmpCrc = myCRC16_Check(Buffer, (uint16_t)(txLen-2));
                    Buffer[txLen-2] = (uint8_t)tmpCrc;
                    Buffer[txLen-1] = (uint8_t)(tmpCrc>>8);
                    
                    begin = DelayTimer.read_ms();
                    while(DelayTimer.read_ms() - begin <= 3);
                        
                    Radio.SetDioIrqParams( TxIrqMask, TxIrqMask, IRQ_RADIO_NONE, IRQ_RADIO_NONE );
                    Radio.SendPayload( Buffer, txLen, ( TickTime_t ){ RX_TIMEOUT_TICK_SIZE, TX_TIMEOUT_VALUE } );   
                    printf("%d\n\r",txPktCnt);
			    }
			    #endif

                #if 0
				if( isMaster == 0 ){
					tmpCnt++;
					if(tmpCnt % 50000 == 0){
						radiostatus = Radio.GetStatus();
						//if((radiostatus.Fields.CmdStatus == 0) || (radiostatus.Fields.ChipMode == 2)){
						if(radiostatus.Fields.ChipMode != 5){
							printf("*********************SetRx******************\r\n");
							printf("A: radiostatus=%d, %d, %d, %d \r\n",radiostatus.Fields.CpuBusy,\
										radiostatus.Fields.DmaBusy,radiostatus.Fields.CmdStatus,radiostatus.Fields.ChipMode );
							Radio.SetDioIrqParams( RxIrqMask, RxIrqMask, IRQ_RADIO_NONE, IRQ_RADIO_NONE );
							Radio.SetRx( ( TickTime_t ) { RX_TIMEOUT_TICK_SIZE, RX_TIMEOUT_VALUE } );

							while(1);///for test
						}
					}
						
					if(tmpCnt % 500000 == 0){
						//radiostatus = Radio.GetStatus();
						printf("B: radiostatus=%d, %d, %d, %d \r\n",radiostatus.Fields.CpuBusy,\
									radiostatus.Fields.DmaBusy,radiostatus.Fields.CmdStatus,radiostatus.Fields.ChipMode );
						
						//}
					}
				}
				#endif
				
				
                break;

            default:
                // Set low power
                break;
        }
    }
}

void OnTxDone( void )
{
    AppState = APP_TX;
    
    begin = DelayTimer.read_ms();
	if(txPktCnt == 1){
        while(DelayTimer.read_ms() - begin <= 500);
	}
    else{
        while(DelayTimer.read_ms() - begin <= 3);
    }

    uint8_t txLen = BUFFER_SIZE;
    txPktCnt += 1;
    for(uint8_t i=0; i<txLen; i++) Buffer[i]=i;
    Buffer[0] = (uint8_t)(txPktCnt>>24);
    Buffer[1] = (uint8_t)(txPktCnt>>16);
    Buffer[2] = (uint8_t)(txPktCnt>>8);
    Buffer[3] = (uint8_t)(txPktCnt);

    //add sw crc
    uint16_t tmpCrc;
    tmpCrc = myCRC16_Check(Buffer, (uint16_t)(txLen-2));
    Buffer[txLen-2] = (uint8_t)tmpCrc;
    Buffer[txLen-1] = (uint8_t)(tmpCrc>>8);
			
    Radio.SetDioIrqParams( TxIrqMask, TxIrqMask, IRQ_RADIO_NONE, IRQ_RADIO_NONE );
    Radio.SendPayload( Buffer, txLen, ( TickTime_t ){ RX_TIMEOUT_TICK_SIZE, TX_TIMEOUT_VALUE } );	
    //printf("%d\n\r",txPktCnt);
}

void OnRxDone( void )
{
    PacketStatus_t packetStatus;
    AppState = APP_RX;
	//printf("RxDone\n\r");
	///*
    Radio.GetPacketStatus(&packetStatus);
#if ( defined( MODE_BLE ) || defined( MODE_GENERIC ) )
    RssiValue = packetStatus.Ble.RssiSync;
    SnrValue = 0;
    //printf("rssi: %d\n\r", RssiValue);
#elif defined( MODE_LORA )
    RssiValue = packetStatus.LoRa.RssiPkt;
    SnrValue = packetStatus.LoRa.SnrPkt;
    //printf("rssi: %d; snr: %d\n\r", RssiValue, SnrValue );
#elif defined( MODE_FLRC )
    RssiValue = packetStatus.Flrc.RssiSync;
    SnrValue = 0;
#endif
	//*/
}

void OnTxTimeout( void )
{
    AppState = APP_TX_TIMEOUT;
    printf( "<>>>>>>>>TXTO\r\n" );
}

void OnRxTimeout( void )
{
    AppState = APP_RX_TIMEOUT;
	printf("RxTimeout\n\r");
}

void OnRxError( IrqErrorCode_t errorCode )
{
    AppState = APP_RX_ERROR;
    printf( "RxError %d\r\n",(uint8_t)errorCode);
}

void OnRangingDone( IrqRangingCode_t val )
{
}

void OnCadDone( bool channelActivityDetected )
{
}


void dbgPrintfHex(uint8_t dbgType, uint8_t *format, uint8_t length)
{
	if(dbgType == 0) {
		return;
	}

	uint8_t i;
	printf(" >> ");
	
	printf("Len=%03d: ", length);
	for(i=0; i<length; i++){
		printf(" %02X", format[i]);
	}
	printf("\r\n");
}

