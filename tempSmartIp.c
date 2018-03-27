/*
author:vikram sanjay shelke
wipro technologies
vikram.shelke@wipro.com
*/

// C includes
#include "string.h"
#include "stdio.h"
#include "math.h"
// SDK includes
#include "dn_common.h"
#include "dnm_local.h"
#include "cli_task.h"
#include "loc_task.h"
#include "dn_system.h"
#include "dn_fs.h"
#include "well_known_ports.h"
#include "dn_exe_hdr.h"
#include "Ver.h"
#include "dn_gpio.h"


#include "dn_api_param.h"
#include "dn_time.h"

//------------------------added from spi.h------------
#include "dn_spi.h"
#include "dn_gpio.h"

// project includes
#include "app_task_cfg.h"

//=========================== defines =========================================


#define SYNCTEMP_UDP_PORT    WKP_USER_3
#define APP_CONFIG_FILENAME  "2syncBlk.cfg"
#define DEFAULT_RPT_PERIOD   30             // seconds
#define APP_MAGIC_NUMBER     0x73797470       // 'sytp'
#define MAX_RX_PAYLOAD       32

#define SPI_BUFFER_LENGTH         4
#define APP_DATA_BUF_SIZE         SPI_BUFFER_LENGTH
#define PERIOD_DEFAULT            10000

#define SPI_TX_BUFFER_LENGTH 2
#define SPI_RX_BUFFER_LENGTH 4

// command identifiers

#define CMDID_GET            0x01
#define CMDID_SET            0x02
#define CMDID_RESPONSE       0x03
#define CMDID_TEMP_DATA      0x04

// response codes

#define APP_RC_OK            0x00
#define APP_RC_ERR           0x01

//=== packet formats

PACKED_START

typedef struct{
   INT32U magic_number;      // APP_MAGIC_NUMBER, big endian
   INT8U  cmdId;             // command identifier
   INT8U  payload[];         // payload
} app_payload_ht;

PACKED_STOP

//=== contents of configuration file

typedef struct{
   INT32U reportPeriod;      // report period, in seconds
} app_cfg_t;

//=========================== variables =======================================

typedef struct {
   // tasks
   OS_STK               sampleTaskStack[TASK_APP_SAMPLE_STK_SIZE];  
   OS_EVENT*            sampleTaskStackTimerSem;      ///< posted when sampleTaskStackTimer expires
   OS_STK               processRxTaskStack[TASK_APP_PROCESSRX_STK_SIZE];
   // network
   OS_EVENT*            joinedSem;                    ///< posted when stack has joined
   INT8U                isJoined;                     ///< 0x01 if the mote has joined the network, 0x00 otherwise
   // configuration
   OS_EVENT*            dataLock;                     ///< locks shared resources
   app_cfg_t            app_cfg;                      ///< structure containing the application's configuration
   INT64S               nextReportUTCSec;             ///< time next report is scheduled
   // rx packet
   OS_EVENT*            rxPkLock;                     ///< locks received packet information
   dn_ipv6_addr_t       rxPkSrc;                      ///< IPv6 address of the sender
   INT8U                rxPkPayload[MAX_RX_PAYLOAD];  ///< received payload
   INT8U                rxPkPayloadLen;               ///< number of bytes in the received payload
   OS_EVENT*            rxPkReady;                    ///< posted when an rx packet is ready for consumption
   OS_TMR*              sampleTaskStackTimer;   
   INT8U                spiTxBuffer[SPI_TX_BUFFER_LENGTH];
   //INT8U                spiRxBuffer[SPI_BUFFER_LENGTH];
   INT16U               period;        ///< period (in ms) between transmissions
   INT8U                     *spiRxBuffer;
   char                      ledToggleFlag;
   
   
} synctemp_app_vars_t;

synctemp_app_vars_t synctemp_v;
// Each SPI transaction is required to write into RX buffer on a word boundary,
// so force word alignment is needed for the start address of the rx buffer
#pragma data_alignment = 4
INT8U                     spiRxBuffer[SPI_RX_BUFFER_LENGTH];

//=========================== prototypes ======================================

//=== CLI handlers
dn_error_t cli_getPeriod(const char* arg, INT32U len);
dn_error_t cli_setPeriod(const char* arg, INT32U len);

//=== tasks
static void sampleTask(void* unused);
//static void spiTask(void* unused);                                 --------added--------

void   sampleTaskStackTimer_cb(void* pTimer, void *pArgs);
static void processRxTask(void* unused);

//=== helpers

// network
dn_error_t rxNotif(dn_api_loc_notif_received_t* rxFrame, INT8U length);

// formatting
void printf_buffer(INT8U* buf, INT8U len);

// configuration
void setPeriod(INT32U reportPeriod);

// configuration file
void loadConfigFile();
void syncToConfigFile();

// lock
void lockData();
void unlockData();
void lockRxPk();
void unlockRxPk();

//=========================== const ===========================================

const dnm_ucli_cmdDef_t cliCmdDefs[] = {
   {&cli_getPeriod,      "getperiod",    "getperiod",              DN_CLI_ACCESS_LOGIN},
   {&cli_setPeriod,      "setperiod",    "setperiod <period>",     DN_CLI_ACCESS_LOGIN},
   {NULL,                 NULL,          NULL,                     DN_CLI_ACCESS_NONE},
};

//=========================== initialization ==================================

/**
\brief This is the entry point for the application code.
*/
int p2_init(void) {
   INT8U           osErr;
   
   // clear module variables
   memset(&synctemp_v,0,sizeof(synctemp_app_vars_t));
   synctemp_v.period = PERIOD_DEFAULT;
  
  /* memset(&spiNetApp_vars,0,sizeof(spiNetApp_vars_t));
   spiNetApp_vars.period     = PERIOD_DEFAULT;*/
   
   
   
   // the mote is not joined
   synctemp_v.isJoined = 0x00;
   
   // semaphores
   
   synctemp_v.sampleTaskStackTimerSem = OSSemCreate(0);    // NOT expired by default
   ASSERT(synctemp_v.sampleTaskStackTimerSem!=NULL);
   
   synctemp_v.joinedSem = OSSemCreate(0);                  // NOT joined by default
   ASSERT(synctemp_v.joinedSem!=NULL);
   
   /*spiNetApp_vars.joinedSem = OSSemCreate(0);
   ASSERT(spiNetApp_vars.joinedSem!=NULL);
   */
   
   synctemp_v.dataLock = OSSemCreate(1);                   // data unlocked by default
   ASSERT(synctemp_v.dataLock!=NULL);
   
   synctemp_v.rxPkLock = OSSemCreate(1);                    // rx packet unlocked by default
   ASSERT(synctemp_v.rxPkLock!=NULL);
   
   synctemp_v.rxPkReady = OSSemCreate(0);                   // rx packet NOT ready by default
   ASSERT(synctemp_v.rxPkReady!=NULL);
   
   //===== register a callback to receive packets
   
   dnm_loc_registerRxNotifCallback(rxNotif);
   
   //===== initialize helper tasks
   
   cli_task_init(
      "synctemp",                    // appName
      cliCmdDefs                     // cliCmds
   );
   
   loc_task_init(
      JOIN_YES,                      // fJoin
      NETID_NONE,                    // netId
      SYNCTEMP_UDP_PORT,             // udpPort
      synctemp_v.joinedSem,          // joinedSem
      BANDWIDTH_NONE,                // bandwidth  BANDWIDTH_NONE = use default BW (set at manager)
      NULL                           // serviceSem
   );
   
   
   //===== initialize tasks (and timer)
   
   // sampleTask
   osErr = OSTaskCreateExt(
      sampleTask,
      (void *) 0,
      (OS_STK*) (&synctemp_v.sampleTaskStack[TASK_APP_SAMPLE_STK_SIZE - 1]),
      TASK_APP_SAMPLE_PRIORITY,
      TASK_APP_SAMPLE_PRIORITY,
      (OS_STK*) synctemp_v.sampleTaskStack,
      TASK_APP_SAMPLE_STK_SIZE,
      (void *) 0,
      OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR
   );
   ASSERT(osErr==OS_ERR_NONE);
   OSTaskNameSet(TASK_APP_SAMPLE_PRIORITY, (INT8U*)TASK_APP_SAMPLE_NAME, &osErr);
   ASSERT(osErr==OS_ERR_NONE);
   


   // sampleTask Timer
   synctemp_v.sampleTaskStackTimer = OSTmrCreate(
      DEFAULT_RPT_PERIOD,                        // dly
      DEFAULT_RPT_PERIOD,                        // period
      OS_TMR_OPT_ONE_SHOT,                       // opt
      (OS_TMR_CALLBACK)&sampleTaskStackTimer_cb, // callback
      NULL,                                      // callback_arg
      NULL,                                      // pname
      &osErr                                     // perr
   );
   ASSERT(osErr==OS_ERR_NONE);
   
   // processRxTask
   osErr = OSTaskCreateExt(
      processRxTask,
      (void *) 0,
      (OS_STK*) (&synctemp_v.processRxTaskStack[TASK_APP_PROCESSRX_STK_SIZE - 1]),
      TASK_APP_PROCESSRX_PRIORITY,
      TASK_APP_PROCESSRX_PRIORITY,
      (OS_STK*) synctemp_v.processRxTaskStack,
      TASK_APP_PROCESSRX_STK_SIZE,
      (void *) 0,
      OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR
   );
   ASSERT(osErr==OS_ERR_NONE);
   OSTaskNameSet(TASK_APP_PROCESSRX_PRIORITY, (INT8U*)TASK_APP_PROCESSRX_NAME, &osErr);
   ASSERT(osErr==OS_ERR_NONE);
   
   return 0;
}

//=========================== CLI handlers ====================================

dn_error_t cli_getPeriod(const char* arg, INT32U len) {
   INT8U           isJoined;
   INT64S          nextReportUTCSec;
   dn_time_asn_t   currentASN;
   dn_time_utc_t   currentUTC;
   INT32U          timeToNext;
   
   // print current configuration
   dnm_ucli_printf(
      "Current configuration: reportPeriod = %u seconds\r\n",
      synctemp_v.app_cfg.reportPeriod
   );
   
   // retrieve what I need
   lockData();
   isJoined             = synctemp_v.isJoined;
   nextReportUTCSec     = synctemp_v.nextReportUTCSec;
   unlockData();
   
   // print next transmission time
   if (isJoined==0x01) {
      
      // get current time
      dn_getNetworkTime(
         &currentASN,
         &currentUTC
      );
      
      // calculate timeToNext
      timeToNext = (INT32U)nextReportUTCSec - (INT32U)currentUTC.sec;
      
      // print timeToNext
      dnm_ucli_printf(
         "Next trigger in %d seconds\r\n",
         timeToNext
      );
   }
   
   return DN_ERR_NONE;
}

dn_error_t cli_setPeriod(const char* arg, INT32U len) {
   INT32U     newReportPeriod;
   int        l;
   
   //--- param 0: a
   l = sscanf(arg, "%u", &newReportPeriod);
   if (l < 1) {
      return DN_ERR_INVALID;
   }
   
   // set the period
   setPeriod(newReportPeriod);
  
   return DN_ERR_NONE;
}

//=========================== tasks ===========================================

static void sampleTask(void* unused) {
   INT8U                osErr;
   INT8U                dnErr;
   dn_time_asn_t        currentASN;
   dn_time_utc_t        currentUTC;
  // INT8U                numBytesRead;
   INT16S               temperature;
   INT32U               reportRateMs;
   INT64U               timeToWaitMs;
   INT8U                pkBuf[sizeof(loc_sendtoNW_t) + sizeof(app_payload_ht) + sizeof(INT16U)];
   loc_sendtoNW_t*      pkToSend;
   app_payload_ht*      payloadToSend;
   INT8U                rc;
   dn_ioctl_spi_transfer_t   spiTransfer;
   dn_spi_open_args_t        spiOpenArgs;
   dn_gpio_ioctl_cfg_out_t    gpioOutCfg;
   INT8U                       transmit_address;
   INT8U                       received_byte;
   INT8U                       LSB_register;
   INT8U                       MSB_register;
   INT16U                      rtdValue;
   INT16U                      ADCcode;
   INT8U                       j;
   INT8U                      receivedSomething;
   INT8U                        i;
   float                      R_REF ;
   float                      Res0;
   float                       a;
   float                      b;
  // FP32                       c;
   float                      Res_RTD;
   float                      temp_C;
   char                       temp_buffer[24];   
   INT32S                    Send_temp;
   char                      finalbuff[10];
    
  
     //===== initialize LEDs
   // configure LEDs as output, with initial level low (LEDs off)
   gpioOutCfg.initialLevel = 0x00;
   // GPIO22 corresponds to the (blue) LED labeled D5 on the Huron board
   dn_open(DN_GPIO_PIN_22_DEV_ID,
           NULL,
           0);
   dn_ioctl(DN_GPIO_PIN_22_DEV_ID,
            DN_IOCTL_GPIO_CFG_OUTPUT,
            &gpioOutCfg,
            sizeof(gpioOutCfg));
   // GPIO23 corresponds to the (green) LED labeled D7 on the Huron board
   dn_open(DN_GPIO_PIN_23_DEV_ID,
           NULL,
           0);
   dn_ioctl(DN_GPIO_PIN_23_DEV_ID,
            DN_IOCTL_GPIO_CFG_OUTPUT,
            &gpioOutCfg,
            sizeof(gpioOutCfg));
   // GPIO20 corresponds to the (green) LED labeled D6 on the Huron board
   dn_open(DN_GPIO_PIN_20_DEV_ID,
           NULL,
           0);
   dn_ioctl(DN_GPIO_PIN_20_DEV_ID,
            DN_IOCTL_GPIO_CFG_OUTPUT,
            &gpioOutCfg,
            sizeof(gpioOutCfg));
   
   
  
   
   // wait a bit before sampling the first time
   OSTimeDly(1000);
   
   // configure reporting period
   loadConfigFile();
   dnm_ucli_printf("configured Reporting period \r\n"); 

  //pkToSend = (loc_sendtoNW_t*)pkBuf;
   
   //===== initialize SPI
   // open the SPI device
   // see doxygen documentation on maxTransactionLenForCPHA_1 when setting
   // spiTransfer.clockPhase = DN_SPI_CPHA_1;
   spiOpenArgs.maxTransactionLenForCPHA_1 = 4;
   dnErr = dn_open(
      DN_SPI_DEV_ID,
      &spiOpenArgs,
      sizeof(spiOpenArgs)
   );
   ASSERT((dnErr == DN_ERR_NONE) || (dnErr == DN_ERR_STATE));
   synctemp_v.spiRxBuffer = spiRxBuffer;
   // initialize spi communication parameters
   spiTransfer.txData             = synctemp_v.spiTxBuffer;
   spiTransfer.rxData             = synctemp_v.spiRxBuffer;
   spiTransfer.transactionLen     = sizeof(synctemp_v.spiTxBuffer);
   spiTransfer.numSamples         = 1;
   spiTransfer.startDelay         = 0;
   spiTransfer.clockPolarity      = DN_SPI_CPOL_1;
   spiTransfer.clockPhase         = DN_SPI_CPHA_1;
   spiTransfer.bitOrder           = DN_SPI_MSB_FIRST;
   spiTransfer.slaveSelect        = DN_SPIM_SS_0n;
   spiTransfer.clockDivider       = DN_SPI_CLKDIV_16;
   spiTransfer.rxBufferLen        = sizeof(synctemp_v.spiRxBuffer);
   
  
   // wait for mote to join
   dnm_ucli_printf("Waiting to mote join\r\n"); 
   OSSemPend(synctemp_v.joinedSem, 0, &osErr);
   ASSERT(osErr==OS_ERR_NONE);
   dnm_ucli_printf(" mote has joined now \r\n"); 
   // the mote is joined
   lockData();
   synctemp_v.isJoined = 0x01;
   unlockData();
  
    // toggle green LED
      synctemp_v.ledToggleFlag = ~synctemp_v.ledToggleFlag;
      dn_write(
         DN_GPIO_PIN_23_DEV_ID,
         &synctemp_v.ledToggleFlag,
         sizeof(synctemp_v.ledToggleFlag)
      );
   
   
  j=0;
      synctemp_v.spiTxBuffer[j] = 0x80; //send address of configure register
      j++;
      synctemp_v.spiTxBuffer[j] = 0xC2; //write config register with 0b 0b11010010  VBIAS=1,AUTO CONVERSION MODE =1, 3 WIRE RTD,FAULT STATUS CLEAR,60 HZ FILTER
      
        dnErr = dn_ioctl(
         DN_SPI_DEV_ID,
         DN_IOCTL_SPI_TRANSFER,
         &spiTransfer,
         sizeof(spiTransfer)
      );
      OSTimeDly(100);
     // dnm_ucli_printf("ioctl performed with code : %d\r\n ",err);
      ASSERT(dnErr >= DN_ERR_NONE);
     /* dnm_ucli_printf("SPI sent:    ",dnErr);
      for ( j=0;j<sizeof(synctemp_v.spiTxBuffer);j++) {
         dnm_ucli_printf(" %02x",synctemp_v.spiTxBuffer[j]);
      }
      
      dnm_ucli_printf("\r\n");
      dnm_ucli_printf("SPI received:",dnErr);  
      for (j=0;j<sizeof(synctemp_v.spiRxBuffer);j++) {
         dnm_ucli_printf(" %02x",synctemp_v.spiRxBuffer[j]);
      }
      dnm_ucli_printf("\r\n"); 
   */        
   
   while (1) { // this is a task, it executes forever
      
      //=== wait
      
      // get the current time
      dn_getNetworkTime(
         &currentASN,
         &currentUTC
      );
      
      // convert reporting period into ms
      lockData();
      reportRateMs = 1000*synctemp_v.app_cfg.reportPeriod;
      unlockData();
      
      // figure out how long to wait
      timeToWaitMs = (INT64U)(reportRateMs - (((currentUTC.sec * 1000) + (currentUTC.usec / 1000)) % reportRateMs));
      
      // update when the next report will be
      lockData();
      synctemp_v.nextReportUTCSec = currentUTC.sec + timeToWaitMs/1000;
      unlockData();
      
      // start the timer
      synctemp_v.sampleTaskStackTimer->OSTmrDly = timeToWaitMs;
      OSTmrStart(synctemp_v.sampleTaskStackTimer, &osErr);
      ASSERT (osErr == OS_ERR_NONE);
      
      // wait for timer to expire
      OSSemPend(
         synctemp_v.sampleTaskStackTimerSem,     // pevent
         0,                                      // timeout
         &osErr                                  // perr
      );
      ASSERT (osErr == OS_ERR_NONE);
      
      transmit_address=0;
      synctemp_v.spiTxBuffer[transmit_address] = 0x01;
      
      //write to spi data //
      dnErr = dn_ioctl(
         DN_SPI_DEV_ID,
         DN_IOCTL_SPI_TRANSFER,
         &spiTransfer,
         sizeof(spiTransfer)
      );
      ASSERT(dnErr >= DN_ERR_NONE);
      
      //===== step 2. verify we received something over SPI
      OSTimeDly(100);
      receivedSomething = 0;
      received_byte=1;
      //dnm_ucli_printf("SPI received data with error code %d :",err); 
         //dnm_ucli_printf("\t %02x ",spi_app_v.spiRxBuffer[received_byte]);
         if (synctemp_v.spiRxBuffer[received_byte]!=0xff) {
            receivedSomething = 1;   
         }
      
      if (receivedSomething==1) {
               MSB_register=synctemp_v.spiRxBuffer[received_byte];  
         }
      
       //---------------------------transmit second address---------------------------------
      OSTimeDly(100);
      transmit_address=0;
      synctemp_v.spiTxBuffer[transmit_address] = 0x02;
      // send bytes
      dnErr = dn_ioctl(
         DN_SPI_DEV_ID,
         DN_IOCTL_SPI_TRANSFER,
         &spiTransfer,
         sizeof(spiTransfer)
      );
      ASSERT(dnErr >= DN_ERR_NONE);
      
      // print on CLI
      //dnm_ucli_printf("SPI sent address: %02x with error code : %d ",spi_app_v.spiTxBuffer[transmit_address],err);
      //dnm_ucli_printf("\r\n");
      //
      //===== step 2. verify we received something over SPI
      OSTimeDly(100);
    receivedSomething = 0;
    received_byte=1;
      //dnm_ucli_printf("SPI received data with error code %d :",err);         
      //       dnm_ucli_printf("\t %02x ",spi_app_v.spiRxBuffer[received_byte]);
            if (synctemp_v.spiRxBuffer[received_byte]!=0xff) {
            receivedSomething = 1;   
          }
        
        
     if(receivedSomething==1) {
         LSB_register=synctemp_v.spiRxBuffer[received_byte]; 
     
      }
      rtdValue= ( MSB_register <<8 | LSB_register);
      ADCcode=rtdValue>>1;
      dnm_ucli_printf("ADCcode:%d  RTD(data):%x",ADCcode,rtdValue); 
      //dnm_ucli_printf("RTD value:%02x%02x",MSB_register,LSB_register);
      dnm_ucli_printf("\r\n");
    
       R_REF = 430.0 ;// Reference Resistor
		Res0 = 100.0; // Resistance at 0 degC for 400ohm R_Ref
		a = .00390830;
		b = -.000000577500;
		// c = -4.18301e-12 # for -200 <= T <= 0 (degC)
		//c = -0.00000000000418301
		//c = 0  for 0 <= T <= 850 (degC)
                //c=0.0;
		Res_RTD = (ADCcode*R_REF)/32768 ;// PT100 Resistance
                 dnm_ucli_printf("resistance:%f",Res_RTD); 
		/* Callendar-Van Dusen equation
		// Res_RTD = Res0 * (1 + a*T + b*T**2 + c*(T-100)*T**3)
                 Res_RTD = Res0 + a*Res0*T + b*Res0*T**2 # c = 0
		# (c*Res0)T**4 - (c*Res0)*100*T**3  
		# + (b*Res0)*T**2 + (a*Res0)*T + (Res0 - Res_RTD) = 0
		#*/
                //quadratic formula:
		// for 0 <= T <= 850 (degC)
		temp_C = -(a*Res0) + sqrt(a*a*Res0*Res0 - 4*(b*Res0)*(Res0 - Res_RTD));
		temp_C = temp_C / (2*(b*Res0));
                 //dnm_ucli_printf("temprature:%f",temp_C); 
                 sprintf(temp_buffer,"%.2f",temp_C);
                  dnm_ucli_printf("temprature in buffer :%s",temp_buffer); 
                  memset(temp_buffer,0,24);
                 // memeset(finalbuff,0,10)
                  for(j=0;j<7;j++)
                  {
                      if(temp_buffer[j]=='.')
                      {
                       // dnm_ucli_printf("fraction at %d ",j);                       
                        break;
                      }
                      finalbuff[j]=temp_buffer[j];
                     
                  }
                  
                   i=j;   
                  for (j=i+1;j<=i+2;j++)
                  {
                   finalbuff[j-1]=temp_buffer[j]; 

                  }
                 //dnm_ucli_printf("Temprature:%s\r\n",finalbuff);
                  Send_temp=atoi(finalbuff);
                  dnm_ucli_printf("Temprature to send :%d\r\n",Send_temp);
         
      //=== send packet
       temperature =Send_temp;
      // fill in packet metadata
      pkToSend = (loc_sendtoNW_t*)pkBuf;
      pkToSend->locSendTo.socketId          = loc_getSocketId();
      pkToSend->locSendTo.destAddr          = DN_MGR_IPV6_MULTICAST_ADDR;
      pkToSend->locSendTo.destPort          = SYNCTEMP_UDP_PORT;
      pkToSend->locSendTo.serviceType       = DN_API_SERVICE_TYPE_BW;   
      pkToSend->locSendTo.priority          = DN_API_PRIORITY_MED;   
      pkToSend->locSendTo.packetId          = 0xFFFF;
      
      // fill in packet payload
      payloadToSend = (app_payload_ht*)pkToSend->locSendTo.payload;
      payloadToSend->magic_number           = htonl(APP_MAGIC_NUMBER);
      payloadToSend->cmdId                  = CMDID_TEMP_DATA;
      temperature                           = htons(temperature);
      memcpy(payloadToSend->payload,&temperature,sizeof(INT16S));
      
      synctemp_v.ledToggleFlag = ~synctemp_v.ledToggleFlag;
      // toggle blue LED
         dn_write(
         DN_GPIO_PIN_22_DEV_ID,
         &synctemp_v.ledToggleFlag,
         sizeof(synctemp_v.ledToggleFlag)
      );
      OSTimeDly(500);
     synctemp_v.ledToggleFlag = ~synctemp_v.ledToggleFlag;
       toggle blue LED
         dn_write(
         DN_GPIO_PIN_22_DEV_ID,
         &synctemp_v.ledToggleFlag,
         sizeof(synctemp_v.ledToggleFlag)
     );
       
      // send packet
      dnErr = dnm_loc_sendtoCmd(
         pkToSend,
         sizeof(app_payload_ht)+sizeof(INT16S),
         &rc
      );
      ASSERT (dnErr == DN_ERR_NONE);
      if (rc!=DN_ERR_NONE){
         dnm_ucli_printf("ERROR sending data (RC=%d)\r\n", rc);
      }
      
      //=== extra delay
      // we add this delay to avoid for the node to send two packets in a row
      // because of a rounding error
      
      OSTimeDly(1000);
   }
}




void sampleTaskStackTimer_cb(void* pTimer, void *pArgs) {
   INT8U  osErr;
   
   // post the semaphore
   osErr = OSSemPost(synctemp_v.sampleTaskStackTimerSem);
   ASSERT(osErr == OS_ERR_NONE);
}

static void processRxTask(void* unused) {
   INT8U           osErr;
   app_payload_ht* appHdr;
   
   while (1) { // this is a task, it executes forever
      
      // wait for the rx packet to be ready
      OSSemPend(synctemp_v.rxPkReady, 0, &osErr);
      ASSERT(osErr==OS_ERR_NONE);
      
      // print received packet
      /*
      dnm_ucli_printf("rx packet\r\n");
      dnm_ucli_printf("- from:     ");
      printf_buffer(synctemp_v.rxPkSrc.byte,sizeof(dn_ipv6_addr_t));
      dnm_ucli_printf("\r\n");
      dnm_ucli_printf("- payload:  ");
      printf_buffer(synctemp_v.rxPkPayload,synctemp_v.rxPkPayloadLen);
      dnm_ucli_printf(" (%d bytes)\r\n", synctemp_v.rxPkPayloadLen);
      */
      
      // trick to be able to use "break"
      do {
         
         // parse header
         appHdr = (app_payload_ht*)synctemp_v.rxPkPayload;
         
         // filter magic_number
         if (appHdr->magic_number!=htonl(APP_MAGIC_NUMBER)) {
            // wrong magic number
            break;
         }
         
         switch(appHdr->cmdId) {
            case CMDID_GET:
               // filter length
               if (synctemp_v.rxPkPayloadLen!=sizeof(app_payload_ht)) {
                  break;
               }
               
               dnm_ucli_printf("TODO: GET\r\n");
               
               break;
            case CMDID_SET:
               // filter length
               if (synctemp_v.rxPkPayloadLen!=sizeof(app_payload_ht)+sizeof(INT32U)) {
                  break;
               }
               
               // set the period
               setPeriod(htonl(*((INT32U*)appHdr->payload)));
              
               break;
            default:
               dnm_ucli_printf("WARNING: unexpected cmdId %d\r\n", appHdr->cmdId);
               break;
         }
         
      } while(0);
      
      // unlock the rx packet
      unlockRxPk();
   }
}

//=========================== helpers =========================================

//===== network

/**
\brief Callback function when receiving a packet OTA.

\param[in] rxFrame The received packet.
\param[in] length  The length of the notification, including the metadata
   (#dn_api_loc_notif_received_t).

\return DN_ERR_NONE always
*/
dn_error_t rxNotif(dn_api_loc_notif_received_t* rxFrame, INT8U length) {
   INT8U                          payloadLen;

   // calc data size
   payloadLen = length-sizeof(dn_api_loc_notif_received_t);
   
   // filter packet
   if (rxFrame->socketId!=loc_getSocketId()) {
      // wrong destination UDP port
      return DN_ERR_NONE;
   }
   if (htons(rxFrame->sourcePort)!=SYNCTEMP_UDP_PORT) {
      // wrong source UDP port
      return DN_ERR_NONE;
   }
   if (payloadLen>MAX_RX_PAYLOAD) {
      // payload too long
      return DN_ERR_NONE;
   }
   
   // if you get here, the packet will be passed to sampleTask
   
   // lock the rx packet information
   // NOTE: will be unlocked by sampleTask when ready
   lockRxPk();
   
   // copy packet information to module variables
   memcpy(synctemp_v.rxPkSrc.byte,rxFrame->sourceAddr.byte,DN_IPV6_ADDR_SIZE);
   memcpy(synctemp_v.rxPkPayload,rxFrame->data,payloadLen);
   synctemp_v.rxPkPayloadLen = payloadLen;
   
   // tell sampleTask the rx packet is ready
   OSSemPost(synctemp_v.rxPkReady);
   
   // NOTE: sampleTask will call unlockRxPk()
   
   return DN_ERR_NONE;
}

//===== formatting

void printf_buffer(INT8U* buf, INT8U len) {
   INT8U i;
   
   for (i=0;i<len;i++) {
      dnm_ucli_printf("%02x",buf[i]);
   }
}

// configuration
void setPeriod(INT32U newPeriod) {
   INT8U      osErr;
   BOOLEAN    rc;
   INT32U     currentPeriod;
   
   // get current period
   lockData();
   currentPeriod = synctemp_v.app_cfg.reportPeriod;
   unlockData();
   
   // abort if nothing to change
   if (currentPeriod==newPeriod) {
      return;
   }
   
   // update period
   lockData();
   synctemp_v.app_cfg.reportPeriod = newPeriod;
   unlockData();
   
   // write to file
   syncToConfigFile();
   
   // print period
   dnm_ucli_printf(
      "Configuration updated: reportPeriod = %d seconds\r\n",
      newPeriod
   );
   
   // rearm timer
   rc = OSTmrStop(
      synctemp_v.sampleTaskStackTimer, // ptmr
      OS_TMR_OPT_NONE,                 // opt
      NULL,                            // callback_arg
      &osErr                           // perr
   );
   ASSERT(rc==OS_TRUE);
   ASSERT(osErr == OS_ERR_NONE);
   
   // call the timer callback
   sampleTaskStackTimer_cb(NULL,NULL);
   
}

//===== configuration file

void loadConfigFile(){
   dn_error_t        dnErr;
   dn_fs_handle_t    configFileHandle;
   
   configFileHandle = dn_fs_find(APP_CONFIG_FILENAME);
   
   if (configFileHandle>=0) {
      // file found: read it
      
      // open file
      configFileHandle = dn_fs_open(
         APP_CONFIG_FILENAME,
         DN_FS_OPT_CREATE,
         sizeof(app_cfg_t),
         DN_FS_MODE_OTH_RW
      );
      ASSERT(configFileHandle >= 0);
      
      // read file
      lockData();
      dnErr = dn_fs_read(
         configFileHandle,
         0, // offset
         (INT8U*)&(synctemp_v.app_cfg),
         sizeof(app_cfg_t)
      );
      unlockData();
      ASSERT(dnErr>=0);
      
      // close file
      dn_fs_close(configFileHandle);
      
      dnm_ucli_printf(
         "Current configuration: reportPeriod = %u seconds\r\n",
         synctemp_v.app_cfg.reportPeriod
      );
      
   } else {
      // file not found: create it
      
      // prepare file content
      lockData();
      synctemp_v.app_cfg.reportPeriod = DEFAULT_RPT_PERIOD;
      unlockData();
      
      // create file
      dnm_ucli_printf("---Create config file\r\n");
      configFileHandle = dn_fs_open(
         APP_CONFIG_FILENAME,
         DN_FS_OPT_CREATE,
         sizeof(app_cfg_t),
         DN_FS_MODE_SHADOW
      );
      ASSERT(configFileHandle>=0);
      
      // write file
      lockData();
      dnErr = dn_fs_write(
         configFileHandle,
         0, // offset
         (INT8U*)&(synctemp_v.app_cfg),
         sizeof(app_cfg_t)
      );
      unlockData();
      ASSERT(dnErr >= 0);
      
      // close file
      dn_fs_close(configFileHandle);
      
      dnm_ucli_printf(
         "Default Config created:  reportPeriod = %u seconds\r\n",
         synctemp_v.app_cfg.reportPeriod
      );
   }
}

void syncToConfigFile() {
   dn_error_t          dnErr;
   dn_fs_handle_t      configFileHandle;
   
   // open file
   configFileHandle = dn_fs_open(
      APP_CONFIG_FILENAME,
      DN_FS_OPT_CREATE,
      sizeof(app_cfg_t),
      DN_FS_MODE_OTH_RW
   );
   ASSERT(configFileHandle >= 0);
   
   // write file
   lockData();
   dnErr = dn_fs_write(
      configFileHandle,
      0, // offset
      (INT8U*)&(synctemp_v.app_cfg),
      sizeof(app_cfg_t)
   );
   unlockData();
   ASSERT(dnErr >= 0);
   
   // close file
   dn_fs_close(configFileHandle);
}

//===== lock

// data

void lockData() {
   INT8U      osErr;
   
   OSSemPend(synctemp_v.dataLock, 0, &osErr);
   ASSERT(osErr == OS_ERR_NONE);
}

void unlockData() {
   OSSemPost(synctemp_v.dataLock);
}

// rxPacket

void lockRxPk() {
   INT8U      osErr;
   
   OSSemPend(synctemp_v.rxPkLock, 0, &osErr);
   ASSERT(osErr == OS_ERR_NONE);
}

void unlockRxPk() {
   OSSemPost(synctemp_v.rxPkLock);
}




//=============================================================================
//=========================== install a kernel header =========================
//=============================================================================

/**
A kernel header is a set of bytes prepended to the actual binary image of this
application. This header is needed for your application to start running.
*/

DN_CREATE_EXE_HDR(DN_VENDOR_ID_NOT_SET,
                  DN_APP_ID_NOT_SET,
                  VER_MAJOR,
                  VER_MINOR,
                  VER_PATCH,
                  VER_BUILD);
