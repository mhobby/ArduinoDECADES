#include <avr/pgmspace.h>
#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <SdFat.h>
    // requires the AdaFruit SdFat library available from https://code.google.com/p/sdfatlib/

byte mac[] = { 
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };

EthernetClient dataClientA;                      //handle for TCP client transactions
EthernetClient dataClientB;                      //handle for TCP client transactions
EthernetUDP ntpClient;                           //handle for NTP client
EthernetUDP dataClientC;                         //handle for NTP client

IPAddress localIp(192, 168,   1,  79);           //Local IP(this machine) - default IP, overidden by arddConf.cfg on SD
IPAddress gateway(192, 168,   1, 254);

IPAddress dataServerA(192, 168, 1, 73);          //Data Server A (TCP) - default IP, overidden by arddConf.cfg on SD
unsigned int dataServerA_port=3502;
IPAddress dataServerB(192, 168, 1, 73);          //Data Server B (TCP) - default IP, overidden by arddConf.cfg on SD
unsigned int dataServerB_port=3502;
IPAddress dataServerC(239, 1, 4, 6);             //Data Server C (UDP)
unsigned int dataServerC_port=50001;

bool A =false;                                   //flag for whether Server A is connected
bool B =false;                                   //flag for whether Server B is connected
bool card =false;                                //flag for whether SD card is present
IPAddress ntpServer(192, 168, 1, 73);            //NTP Server - default IP, overidden by arddConf.cfg on SD

char fname[12];                                  //sd card data storage filename

#define NTP_PACKET_SIZE 48                       //NTP Packet size
char ntpBuf[NTP_PACKET_SIZE];                    //buffer for NTP data
#define POSIX_0 2208988800;                      // 1st Jan 1970 00:00Z

unsigned long syncTimeout = 10;                  //no. of ms to wait following an NTP request
byte osf = 1;                                    //oversample factor
byte osf_2 = 0;                                  //oversample factor in base-2; useful fir division/multiplies
 
unsigned int T=1000;                             //sample period
unsigned int syncT=5;                            //period between NTP resyncs in s
  
int sensorPin = A0;
char sensId[10]="$ARDD0001";                     //name of this sensor- overidden by arddConf.cfg on SD
char flight[5]="XXXX";                           //flight number
signed int sensVal=0;                            //sensor value 
unsigned int ardTemp=0;                          //arduino temperature (not currently sensed, but for future...)
unsigned long lNTP;                              //local NTP time
unsigned long lNTPfrac;                          //local NTP time (fractional part)
byte sampleCounter=0;                            //counter for counting 'over' samples
unsigned long tNow, tPrev;                       //local ms count since startup
unsigned long lastSync;                          //last local ms count when NTP was obtained
bool clockResyncFlag=0;                          //flag to show when local NTP has been synced to remote NTP
uint32_t size;                                   //SD card size
unsigned int sdCap;                              //free capacity on SD card
byte bkupFile=1;                                 //backup to SD flag - hi=backup to SD, low=no backup

Sd2Card sd;
SdVolume vol;
SdFat file;

uint16_t freeMem() {
  char top;
  extern char *__brkval;
  extern char __bss_end;
  Serial.print( __brkval ? &top - __brkval : &top - &__bss_end);
}

void showString (PGM_P s) {
  char c;
  while ((c = pgm_read_byte(s++)) != 0)
      Serial.print(c);
}
 
byte readConfig(char *ID, byte *IPL, byte *IPG, byte *IPA, byte* IPB, byte* IPC, byte* IPntp, char* MODE)
{
  // NOTE no error checking is carried on the SD card file. If errors occur
  //     in the file, then arduino performance is undefined. Will 
  //     probably hang or crash - BEWARE!!!
  char a[30];
  char b[10];
  byte i, j, k;
  byte offset;
  char* syncTimeout_;
  char osf_[5];

  ifstream sdin("arddConf.cfg"); 
  if (sdin.is_open()) {    
     Serial.println("Config file exists"); 

    //read in SENSOR ID and store to argin ID
    i=0; memset(a, 0, 30);
    do {
      a[i++]=sdin.get();
    }while(a[i-1]!='\n');
    for(j=0; j<(i-3<=9?i-3:9); j++) {
      ID[j]=a[j+2];  
    }
  
    //read in ADDRESS and store to argin IPL for local IP
    i=0; memset(a, 0, 30);
    do {
      a[i++]=sdin.get();
    }while(a[i-1]!='\n');
    j=0; offset=2;
    for(k=0; k<4; k++)          //extract individual octets from address
    {
      while((a[offset+j]!='.')&&(a[offset+j]!='\n')) b[j]=a[(j++)+offset];          
      b[j]='\n';
      offset+=j+1; j=0;  
      IPL[k]=atoi(b);  
    }
    //read in ADDRESS and store to argin IPG for gateway
    i=0; memset(a, 0, 30);
    do {
      a[i++]=sdin.get();
    }while(a[i-1]!='\n');
    j=0; offset=2;
    for(k=0; k<4; k++)          //extract individual octets from address
    {
      while((a[offset+j]!='.')&&(a[offset+j]!='\n')) b[j]=a[(j++)+offset];          
      b[j]='\n';
      offset+=j+1; j=0;  
      IPG[k]=atoi(b);  
    }
  
    //read in ADDRESS A and store to argin IPA
    i=0; memset(a, 0, 30);
    do {
      a[i++]=sdin.get();
    }while(a[i-1]!='\n');
    j=0; offset=2;
    for(k=0; k<4; k++)          //extract individual octets from address
    {
      while((a[offset+j]!='.')&&(a[offset+j]!=':')) b[j]=a[(j++)+offset];          
      b[j]='\n';
      offset+=j+1; j=0;  
      IPA[k]=atoi(b);  
    }
    while(a[offset+j]!='\n') b[j]=a[(j++)+offset];
    b[j]='\n';
    dataServerA_port=atoi(b);
  
    //read in ADDRESS B and store to argin IPB
    i=0; memset(a, 0, 30);
    do {
      a[i++]=sdin.get();
    }while(a[i-1]!='\n');
    j=0; offset=2;
    for(k=0; k<4; k++)          //extract individual octets from address
    {
      while((a[offset+j]!='.')&&(a[offset+j]!=':')) b[j]=a[(j++)+offset];          
      b[j]='\n';
      offset+=j+1; j=0;  
      IPB[k]=atoi(b); 
    }
    while(a[offset+j]!='\n') b[j]=a[(j++)+offset];
    b[j]='\n';
    dataServerB_port=atoi(b);
    
    //read in UDP multicast address
    i=0; memset(a, 0, 30);
    do {
      a[i++]=sdin.get();
    }while(a[i-1]!='\n');
    j=0; offset=2;
    for(k=0; k<4; k++)          //extract individual octets from address
    {
      while((a[offset+j]!='.')&&(a[offset+j]!=':')) b[j]=a[(j++)+offset];          
      b[j]='\n';
      offset+=j+1; j=0;  
      IPC[k]=atoi(b);         
    } 
    while(a[offset+j]!='\n') b[j]=a[(j++)+offset];
    b[j]='\n';
    dataServerC_port=atoi(b);
    
    //read in ADDRESS NTP and store to argin IPntp
    i=0; memset(a, 0, 30);
    do {
      a[i++]=sdin.get();
    }while(a[i-1]!='\n');
    j=0; offset=2;
    for(k=0; k<4; k++)          //extract individual octets from address
    {
      while((a[offset+j]!='.')&&(a[offset+j]!='\n')) b[j]=a[(j++)+offset];          
      b[j]='\n';
      offset+=j+1; j=0;  
      IPntp[k]=atoi(b);         
    } 
    
    //read in MODE
    i=0; memset(a, 0, 30);
    do {
      a[i++]=sdin.get();
    }while(a[i-1]!='\n');
    for(j=0; j<3; j++) {
      MODE[j]=a[j+2];  
    }

    //read in SYNC TIMEOUT
    i=0; memset(a, 0, 30);
    do {
      a[i++]=sdin.get();
    }while(a[i-1]!='\n');
    syncTimeout_=(char*)malloc(sizeof(byte)*i-1);
    for(j=0; j<i-2; j++) {
      syncTimeout_[j]=a[j+2];  
    }
    syncTimeout_[i-1]=0;
    syncTimeout = atoi(syncTimeout_);
    
    //read in OSF
    i=0; memset(a, 0, 30);
    do {
      a[i++]=sdin.get();
    }while(a[i-1]!='\n');
    for(j=0; j<i-2; j++) {
      osf_[j]=a[j+2];  
    }
    osf_[i-1]=0;
    osf = atoi(osf_);
    if( (osf!=1) && (osf!=2) && (osf!=4) && (osf!=8) ) {
      osf=1;
      Serial.println("Invalid Oversample Factor requested in arddCon.cfg (must be 1,2,4 or 8).");
      Serial.println("Setting Oversample Factor to 1...");
    }    
    
    //read in SD BACKUP FLAG   
    i=0; memset(a, 0, 30);
    do {
      a[i++]=sdin.get();
    }while(a[i-1]!='\n');
    bkupFile=a[2]-0x30;
    
    sdin.close();
    return 1;
  }
  else {
   Serial.println("config file does not exist"); 
  } return 0;
}

bool getNtpTime(unsigned long *epoch, unsigned long *epochFrac)
{
  unsigned long xmt, xmtFrac;                   // NTP times used to calculate remote NTP times
  sendNTPpacket(ntpServer);                     //request NTP timestamp
  unsigned long syncStart=millis();             //start timer for timeout
  unsigned long elapsed=millis()-syncStart;  
  while ( (ntpClient.parsePacket() < NTP_PACKET_SIZE) && (elapsed<syncTimeout) ) {
    elapsed = millis()-syncStart;               // wait for NTP response or timeout
  }
  if ( elapsed<syncTimeout ) {                  //if NTP response obtained before timeout...
    ntpClient.read(ntpBuf,NTP_PACKET_SIZE);     // read UDP/NTP packet into buffer
    xmt = ((unsigned long)word(ntpBuf[40], ntpBuf[41]) << 16 | word(ntpBuf[42], ntpBuf[43]))-POSIX_0;    
    xmtFrac = ((unsigned long)word(ntpBuf[44], ntpBuf[45]) << 16 | word(ntpBuf[46], ntpBuf[47]));      
    *epoch=xmt;
    *epochFrac=xmtFrac;
    return true;                                // return triumphant!
  }
  else {
    showString(PSTR("Out of sync\n"));
    return false;                               //return failure!
  }
} 

unsigned long sendNTPpacket(IPAddress& address)
{
  //sendNTPPacket - sends request to NTP server
  //   argin:  IPAddress address - address to which to send NTP request
  //           EthernetUDP client - handle for the NTP client
  byte packetBuffer[ NTP_PACKET_SIZE];       //buffer to hold incoming and outgoing packets     
  memset(packetBuffer, 0, NTP_PACKET_SIZE);  // set all bytes in the buffer to 0
                                             // Initialize values needed to form NTP request
  packetBuffer[0] = 0b00010011;              // LI, Version, Mode
  packetBuffer[1] = 0;                       // Stratum, or type of clock
  packetBuffer[2] = 6;                       // Polling Interval
  packetBuffer[3] = 0xEC;                    // Peer Clock Precision
                                             // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;                    // 1N14
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;

  ntpClient.beginPacket(address, 123);           //NTP requests are to port 123
  ntpClient.write(packetBuffer,NTP_PACKET_SIZE); //request a timestamp
  ntpClient.endPacket();                         //
}

void formDataPackets()
{
  #define UDP_PKT_SIZE 59                     //UDP packet size
  char UDPBuf[UDP_PKT_SIZE];                  //buffer for UDP data to send
  byte UDPBufptr=0;                           //pointer within the UDP buffer  
  
  #define TCP_PKT_SIZE 33                     //TCP packet size
  byte TCPBuf[TCP_PKT_SIZE];                  //buffer for TCP data to send
  byte TCPBufptr=0;                           //pointer within the TCP buffer  
                                              // data is sent big endian
  char* tempStr;
  static bool running=true;
  static unsigned int pktCount=0;

  //ID  
  for(byte i=0;i<9;i++) TCPBuf[TCPBufptr++]=sensId[i];
  for(byte i=0;i<9;i++) UDPBuf[UDPBufptr++]=sensId[i];    
  UDPBuf[UDPBufptr++]=','; //udp ptr 9

  //pkt size    
  TCPBuf[TCPBufptr++]=0;  
  TCPBuf[TCPBufptr++]=0;
  TCPBuf[TCPBufptr++]=0;  
  TCPBuf[TCPBufptr++]=20; 
  UDPBuf[UDPBufptr++]='0'; UDPBuf[UDPBufptr++]='5'; UDPBuf[UDPBufptr++]='6';
  UDPBuf[UDPBufptr++]=',';  //udp ptr 13

  //utc time
  TCPBuf[TCPBufptr++]=(0xFF000000&lNTP)>>24;
  TCPBuf[TCPBufptr++]=(0x00FF0000&lNTP)>>16;
  TCPBuf[TCPBufptr++]=(0x0000FF00&lNTP)>>8;
  TCPBuf[TCPBufptr++]=(0x000000FF&lNTP);
  tempStr = (char*)malloc(sizeof(char)*11);
  sprintf(tempStr, "%lud", lNTP);
  for(byte j=0; j<10; j++) UDPBuf[UDPBufptr++]=tempStr[j];
  free(tempStr);
  UDPBuf[UDPBufptr++]=',';  //udp ptr 24
  
  //ntp sync
  TCPBuf[TCPBufptr++]=clockResyncFlag; 
  if(clockResyncFlag) UDPBuf[UDPBufptr++]='1';
  else UDPBuf[UDPBufptr++]='0';
  UDPBuf[UDPBufptr++]=','; //udp ptr 26
  
  //ard temp
  TCPBuf[TCPBufptr++]=(0xFF00&ardTemp)>>8;  
  TCPBuf[TCPBufptr++]=(0x00FF&ardTemp);
  tempStr = (char*)malloc(sizeof(char)*7);
  sprintf(tempStr, "%05d", ardTemp);
  for(byte j=0; j<6; j++) UDPBuf[UDPBufptr++]=tempStr[j];
  free(tempStr);    
  UDPBuf[UDPBufptr++]=','; //udp ptr 33
  
  //flight number
  for(byte i=0;i<4;i++) {
    TCPBuf[TCPBufptr++]=flight[i];
    UDPBuf[UDPBufptr++]=flight[i];    
  } 
  UDPBuf[UDPBufptr++]=','; //udp ptr 38
  
  //rtc
  TCPBuf[TCPBufptr++]=0;
  TCPBuf[TCPBufptr++]=running;
  UDPBuf[UDPBufptr++]='0';
  if(running) UDPBuf[UDPBufptr++]='1';
  else UDPBuf[UDPBufptr++]='0';
  running = !running;
  UDPBuf[UDPBufptr++]=','; //udp ptr 40

  //udp number
  TCPBuf[TCPBufptr++]=0;
  TCPBuf[TCPBufptr++]=0;  
  TCPBuf[TCPBufptr++]=(0xFF00&pktCount)>>8;  
  TCPBuf[TCPBufptr++]=0x00FF&pktCount; 
  tempStr = (char*)malloc(sizeof(char)*7);
  sprintf(tempStr, "%05d", pktCount);
  for(byte j=0; j<6; j++) UDPBuf[UDPBufptr++]=tempStr[j];
  free(tempStr);    
  UDPBuf[UDPBufptr++]=','; //udp ptr 47
   
  //sd used
  TCPBuf[TCPBufptr++]=sdCap;
  tempStr = (char*)malloc(sizeof(char)*4);
  sprintf(tempStr, "%03d", sdCap);
  for(byte j=0; j<3; j++) UDPBuf[UDPBufptr++]=tempStr[j];
  free(tempStr);    
  UDPBuf[UDPBufptr++]=','; //udp ptr 51
  
  //E field
  TCPBuf[TCPBufptr++]=(0xFF00&sensVal)>>8;
  TCPBuf[TCPBufptr++]=(0x00FF&sensVal); 
  tempStr = (char*)malloc(sizeof(char)*7);
  sprintf(tempStr, "%05d", sensVal);
  for(byte j=0; j<6; j++) UDPBuf[UDPBufptr++]=tempStr[j];
  free(tempStr);
  
  dataClientA.write(TCPBuf, TCP_PKT_SIZE);  // ... send data to data server A
  dataClientB.write(TCPBuf, TCP_PKT_SIZE);  // ... send data to data server B 
  dataClientC.beginPacket(dataServerC, 50001);
  dataClientC.write((byte*)UDPBuf, UDP_PKT_SIZE);  // ... send data to data server C  
  dataClientC.endPacket();
  pktCount++;  
}

void setup() 
{   
  //setup - ARDUINO setup method. Run at startup by Boot loader  
  char ID[9]="";
  byte IPL[4];
  byte IPG[4];
  byte IPA[4];
  byte IPB[4];
  byte IPC[4];  
  byte IPntp[4];
  char MODE[4]="RLS";
  Serial.begin(9600);                     //start serial driver at baud of 9600
     while (!Serial);                     //wait for driver to initialise...
  delay(1000);   

  showString(PSTR("*** ARDUINO/DECADES interface v4.1 (2014 MHOBBY) ***\r\n"));
  
  if (!sd.init(SPI_FULL_SPEED, 4)) {
    showString(PSTR("SD Card failed, or not present\r\n"));
  }
  else {
    showString(PSTR("SD Card initialized.\r\n"));
    card=true; 
    vol.init(&sd);
    uint32_t volFree = vol.freeClusterCount();
    size = 0.000512 * sd.cardSize()+0.5;
    float fs = 0.000512*volFree*vol.blocksPerCluster();
    sdCap=100*(fs/size);
    Serial.print("Card size: "); Serial.print(size); Serial.println(" MB");
    Serial.print("Free space: "); Serial.print(fs); Serial.println(" MB");
    file.begin(4, SPI_FULL_SPEED);   // start FAT fs    
    ifstream sdin("arddConf.cfg"); 
    if (readConfig(ID, IPL, IPG, IPA, IPB, IPC, IPntp, MODE))
    {
       for(byte i=0; i<9; i++) {
         sensId[i]=ID[i];       
       }       
       localIp=IPAddress(IPL[0], IPL[1], IPL[2], IPL[3]);
       gateway=IPAddress(IPG[0], IPG[1], IPG[2], IPG[3]);
       dataServerA=IPAddress(IPA[0], IPA[1], IPA[2], IPA[3]);
       dataServerB=IPAddress(IPB[0], IPB[1], IPB[2], IPB[3]);  
       dataServerC=IPAddress(IPC[0], IPC[1], IPC[2], IPC[3]);  
       ntpServer=IPAddress(IPntp[0], IPntp[1], IPntp[2], IPntp[3]);              
    }  
  }
  Ethernet.begin(mac, localIp, IPAddress(0,0,0,0), gateway);      //start ethernet driver     
  showString(PSTR("IP address: "));       //print to term the local IP address from DHCP
     Serial.println(Ethernet.localIP());
       
  ntpClient.begin(8888);               //start UDP/NTP driver over port 8888
  showString(PSTR("NTP Server: ")); Serial.println(ntpServer);
  showString(PSTR("NTP Sync Timeout: ")); Serial.println(syncTimeout);
  
  sendNTPpacket(ntpServer);

  dataClientC.beginMC(dataServerC, dataServerC_port);               //start UDP/DATA driver over port 50001
  showString(PSTR("UDP Multicast on ")); Serial.print(dataServerC); Serial.print(":"); Serial.println(dataServerC_port); 

  if(bkupFile && osf>1) {
    osf=1;
    Serial.println("SD card backup must be disabled in arddCon.cfg to enable oversampling");
    Serial.println("  Oversampling Factor forced to 1.");
  }
  showString(PSTR("Oversample Factor: ")); Serial.println(osf); 
  switch(osf) {
    case 1: osf_2=0; break;
    case 2: osf_2=1; break;
    case 4: osf_2=2; break;
    case 8: osf_2=3; break;
  } 
  showString(PSTR("Raw Sampling Period: ")); Serial.print(T>>osf_2); Serial.println("ms [averaging & reporting period 1s]");

  if( (MODE[0]=='R') && (MODE[1]=='L') && (MODE[2]=='S') ) {
    Serial.println("MODE: Release [RLS]");
    #define WOW_BUF_SIZE 119
    unsigned char wowBuffer[WOW_BUF_SIZE];
    int wowPktSize;
    char pktLen_[4];
    int pktLen;
    char flightNo[3][6];
    bool validFlightNo;  
    unsigned char wowFlag;
    byte i,j,k,commaCount;
    
    validFlightNo=false;
    wowFlag=0;
    k=0;
    
    Serial.println("Waiting for FlightNo and 'Wait on Wheels' flag in PRTAFT UDP Multicast (5 consistent) packets ... ");
  
    while(wowFlag<5 || !validFlightNo) {                         //collect 5 packets from PRTAFT UDP Multicast packet where the WOW indicator is high and a valid flight number has been obtained
       wowPktSize=dataClientC.rawRead(wowBuffer, WOW_BUF_SIZE);  //collect raw data from UDP multicast
       if(wowPktSize>0) {        
          wowBuffer[wowPktSize]=0;                             //force last byte to be 'newline' for nice formatting to serial terminal                  
          if((wowBuffer[0]==IPL[0])&&(wowBuffer[1]==IPL[1])&&(wowBuffer[2]==IPL[2])) { //check that multicast packet came from the same subnet as the Arduino unit       
             if ( (wowBuffer[8]=='$') && (wowBuffer[9]=='P') && (wowBuffer[10]=='R') && (wowBuffer[11]=='T') &&
                  (wowBuffer[12]=='A') && (wowBuffer[13]=='F') && (wowBuffer[14]=='T') && (wowBuffer[15]=='0') && (wowBuffer[16]=='1') ) { 
                Serial.print(wowPktSize); Serial.print(" bytes from ");
                Serial.print(wowBuffer[0]); Serial.print(".");
                Serial.print(wowBuffer[1]); Serial.print(".");
                Serial.print(wowBuffer[2]); Serial.print(".");
                Serial.print(wowBuffer[3]); Serial.print(":\t");
                Serial.println((char*)&wowBuffer[8]);
                
                i=8;                                                                   //check the multicast packet came from PRTAFT data source
                commaCount=0;
                while( (i<wowPktSize) && (commaCount<1) ) {                            //count up to 1st comma
                   if(wowBuffer[i++]==0x2c) commaCount++;
                }                   
                j=0;
                while( (i<wowPktSize) && (commaCount<2) ) {                            //count up to 2nd comma
                   pktLen_[j++]=wowBuffer[i++];
                   if(wowBuffer[i]==0x2c) commaCount++;
                }
                pktLen_[j]=0;
                pktLen=atoi(pktLen_);

                if(wowPktSize-pktLen==21) {                
                  while( (i<wowPktSize) && (commaCount<6) ) {                            //count up to 5th comma
                     if(wowBuffer[i++]==0x2c) commaCount++;
                  }                                   
                  j=0;
                  while( (j<6) && (commaCount<7) ) {                                     //read data up to the next comma
                     flightNo[k][j++]=wowBuffer[i++];                                    //   and store as a flight number instance
                     if(wowBuffer[i]==0x2c) commaCount++;
                  }
                  for(byte l=0; l<4; l++) flight[l]=flightNo[k][l];
  
                  k++;
                  if(k>3) k=0;
    
                  while( (i<wowPktSize) && (commaCount<21) ) {                            //count up to 19th comma
                     if(wowBuffer[i++]==0x2c) commaCount++;
                  }
                  if(i<wowPktSize) {
                    if ( wowBuffer[i]=='1' ) {                                            // if wowFlag in UDP packet is high, increment local wow flag count
                      wowFlag++;              
                    }             
                    else wowFlag=0;
                  }
                  else wowFlag=0;
                  
                  if( (flightNo[0][0])==(flightNo[1][0]) && (flightNo[1][0])==(flightNo[2][0]) &&
                      (flightNo[0][1])==(flightNo[1][1]) && (flightNo[1][1])==(flightNo[2][1]) &&
                      (flightNo[0][2])==(flightNo[1][2]) && (flightNo[1][2])==(flightNo[2][2]) &&
                      (flightNo[0][3])==(flightNo[1][3]) && (flightNo[1][3])==(flightNo[2][3]) ) {  
                        validFlightNo = true;                     
                       flight[0]=flightNo[0][0];     
                       flight[1]=flightNo[0][1];
                       flight[2]=flightNo[0][2];
                       flight[3]=flightNo[0][3];                     
                  }
                  else validFlightNo = false;              
                }
                else {
                   wowFlag=0;
                   validFlightNo = false;
                }               
             }                
          }             
       }       
       for(byte i=0; i<WOW_BUF_SIZE; i++) wowBuffer[i]=0x0;       //clear packetBuffer
    }
    Serial.print("Flight No.: "); Serial.println(flight); 
  }
  else if( (MODE[0]=='D') && (MODE[1]=='B') && (MODE[2]=='G') ) Serial.println("MODE: Debug [DBG]");
  else {
    Serial.println("MODE: Unrecognised. Have you set M=? (line 8) in config file? Should be RLS (release) or DBG (debug)");
    Serial.println("  - please check and reboot");
    while(1);
  }
  
  if(dataClientA.connect(dataServerA, dataServerA_port)) { //attempt to connect to TCP data serverA
    A=true;
    showString(PSTR("TCP A connected to "));          
    Serial.print(dataServerA); Serial.print(":"); Serial.println(dataServerA_port);    
  }
  else {
    showString(PSTR("Could not connect to data server A: ")); 
    Serial.println(dataServerA);
  }
  
  if(dataClientB.connect(dataServerB, dataServerB_port)) { //attempt to connect to TCP data serverB
    B=true;
    showString(PSTR("TCP B connected to "));          
    Serial.print(dataServerB); Serial.print(":"); Serial.println(dataServerB_port); 
  }
  else {
    showString(PSTR("Could not connect to data server B: "));
    Serial.println(dataServerB); 
  }
      
  while(!getNtpTime(&lNTP, &lNTPfrac));   //set local NTP clock to xmt to name file
  tPrev = millis();                       //LAST TIMESTAMP (ms), when sample was obtained
  lastSync = tPrev;                       //LAST SYNCPOINT (ms), when tNTP was obtained
  
  if(card && bkupFile) {
    sprintf(fname, "%lda.CSV", lNTP/3600);
    while(file.exists(fname)) fname[6]++;     // if file exists, increment alphabetic extension (a....z)
    Serial.print("Logging to "); Serial.println(fname);
  }
  else Serial.println("Not creating backup log to SD card");
  showString(PSTR("RAM: ")); freeMem(); showString(PSTR("\r\n"));
}

void loop()
{ 
  float fs;
  uint32_t volFree;
  tNow=millis();                          // local ms clock time
  int tmp1, tmp2;

  if (tNow>=(tPrev+(T>>osf_2))) {                  //is now after LAST TIMESTAMP + sample period?
    tPrev=tNow;  
    tmp1=analogRead(sensorPin+1)*10;
    tmp2=analogRead(sensorPin)*10;
    sensVal += tmp1-tmp2;
    Serial.print(lNTP); // output time + sample to term
    Serial.print(","); Serial.print(tNow);
    Serial.print(","); Serial.print(sampleCounter);
    Serial.print(","); Serial.print(clockResyncFlag); 
    Serial.print(","); Serial.print(sdCap);
    Serial.print(","); Serial.print(tmp1);
    Serial.print(","); Serial.print(tmp2); 
    Serial.print(","); Serial.print(sensVal);
    Serial.print(","); Serial.println(sensVal>>osf_2);
    
    if(clockResyncFlag) clockResyncFlag=false;
       
    sampleCounter++;
    if(sampleCounter>=osf) {     
      sensVal=sensVal>>osf_2;
      lNTP++;
      
      if (tNow>(lastSync+(1000*(unsigned long)syncT))) {  //is now after LAST SYNCPOINT + sync period?
         if(getNtpTime(&lNTP, &lNTPfrac))                  // obtain NTP time  
         {                                      
           lastSync=tNow;                    //set LAST SYNCPOINT to now
           clockResyncFlag=true;
         }
       }      
       formDataPackets();                     // form UDP/TCP Packets and if buffer full forward to servers      

       if(card && bkupFile) {
          ofstream sdout(fname, ios::out | ios::app);
          if(sdout.is_open()){    // open file    
             sdout << lNTP; // output time + sample to term
             sdout << ","; sdout << lNTPfrac;
             sdout << ","; sdout << tNow;
             sdout << ","; sdout << clockResyncFlag; 
             sdout << ","; sdout << tmp1; 
             sdout << ","; sdout << tmp2;              
             sdout << ","; sdout << sensVal;            
             sdout << ","; sdout << endl;        
             sdout.close();
             volFree = vol.freeClusterCount();
             fs= 0.000512*volFree*vol.blocksPerCluster();
             sdCap=100*(fs/size);
          }
          else {
             Serial.print(fname); Serial.println(": failed to open file");    
          }
       }
       sensVal=0;
       sampleCounter=0;       
    }
  }      
}
