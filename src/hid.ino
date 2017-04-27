#include <SoftwareSerial.h>
#include "CH375.h"
SoftwareSerial sw(5,4);

bool isUSBReady = true;
bool isConnected = false;
bool isDataReady = false;
int r=0;
long time = 0;
void onReceived(){
    isUSBReady = true;
}
void setup(){
    Serial.begin(115200);
    sw.begin(9600);
    attachInterrupt(digitalPinToInterrupt(2), onReceived, LOW);
    setupUSB();
}
void loop(){
    int len = host_recv();
    if(len>0){
      for(int i=0;i<len;i++){
        Serial.print(recv_buffer[i],HEX);
        Serial.print(",");
      }
        Serial.println();
    }else{
    }
    toggle_recv();   
    while(issue_token( ( endp_in_addr << 4 ) | DEF_USB_PID_IN )!=USB_INT_SUCCESS);
}
void setupUSB(){
    set_usb_mode( 6 );
    while(getIrq()!=USB_INT_CONNECT);
    set_usb_mode( 7 );
    delay(10);
    set_usb_mode( 6 );
    delay(10);
    while(getIrq()!=USB_INT_CONNECT);
    int irq = get_descr(1);
    int len = 0;
    if(irq==USB_INT_SUCCESS){
       len = rd_usb_data( recv_buffer );
    }
    irq = set_addr(2);  
    if(irq==USB_INT_SUCCESS){
      irq = get_descr(2); 
       if(irq==USB_INT_SUCCESS){
           len = rd_usb_data( recv_buffer );
           if(p_cfg_descr->endp_descr[0].bDescriptorType==0x21){ // skip hid des
            tmpEp = (PUSB_ENDP_DESCR)((int8_t*)(&(p_cfg_descr->endp_descr[0]))+p_cfg_descr->endp_descr[0].bLength); // get the real ep position
          }
           endp_out_addr=endp_in_addr=0;
          int address =tmpEp->bEndpointAddress;
        if( address&0x80 ){
          endp_in_addr = address&0x0f;
        }else{
          endp_out_addr = address&0x0f;
          endp_out_size = p_cfg_descr->endp_descr[0].wMaxPacketSize;
          if( endp_out_size == 0 || endp_out_size > 64 )
            endp_out_size = 64;
        }
          irq = set_config(p_cfg_descr->cfg_descr.bConfigurationvalue);
        if(irq==USB_INT_SUCCESS){
            CH375_WR( CMD_SET_RETRY );
            CH375_WR( 0x25 );
            CH375_WR( 0x85 );
            isConnected = true; 
            toggle_recv();
            while(issue_token( ( endp_in_addr << 4 ) | DEF_USB_PID_IN )!=USB_INT_SUCCESS);
          }
       }
    }
}
uint8_t issue_token( uint8_t endp_and_pid )
{
  CH375_WR( CMD_ISSUE_TOKEN );
  CH375_WR( endp_and_pid );
  return  getIrq();
}
void toggle_recv()
{
  CH375_WR( CMD_SET_ENDP6 );
  CH375_WR( endp6_mode );
  endp6_mode^=0x40;
}
uint8_t rd_usb_data( uint8_t *buf )
{
  uint8_t i, len;
  CH375_WR( CMD_RD_USB_DATA );
  len=CH375_RD();
  for ( i=0; i!=len; i++ ) *buf++=CH375_RD();
  return( len );
}
void wr_usb_data( uint8_t len)
{
  CH375_WR( CMD_WR_USB_DATA7 );
  CH375_WR( len );
}
uint8_t clr_stall6(void)
{
  CH375_WR( CMD_CLR_STALL );
  CH375_WR( endp_out_addr | 0x80 );
  endp6_mode=0x80;
  return getIrq();
}
uint8_t host_recv()
{
    int16_t len = rd_usb_data(recv_buffer);
    return len;
}
void resetBus()
{
  int16_t c;
  c = set_usb_mode(7);
  delay(10);
  c = set_usb_mode(6);
  delay(10);
}
uint8_t set_config(uint8_t cfg){
  endp6_mode=endp7_mode=0x80; // reset the sync flags
  CH375_WR(CMD_SET_CONFIG);
  CH375_WR(cfg);
  return getIrq();
}
uint8_t set_addr( uint8_t addr )
{
  uint8_t irq;
  CH375_WR(CMD_SET_ADDRESS);
  CH375_WR(addr);
  irq = getIrq();
  CH375_WR(CMD_SET_USB_ADDR);
  CH375_WR(addr);
  return irq;
}
unsigned char get_descr( unsigned char type ) {
  CH375_WR( CMD_GET_DESCR );
  CH375_WR( type );
  return getIrq();
}
uint8_t getIrq()
{
  int t = 0;
  while(!isUSBReady){
    t++;
    delayMicroseconds(800);
    if(t>1000){
      Serial.println("timeout!");
      break;
    }
  }
  isUSBReady = false;
  CH375_WR(CMD_GET_STATUS);
  return CH375_RD();
}
int16_t set_usb_mode(int16_t mode)
{
  CH375_WR(CMD_SET_USB_MODE);
  CH375_WR(mode);
  endp6_mode=endp7_mode=0x80;
  return CH375_RD();
}
void CH375_WR(uint8_t c)
{
  sw.write(c);
  delay(2);
}
uint8_t CH375_RD()
{
  uint8_t c=0;
  delay(2);
  if(sw.available()){
    c = sw.read();
  }
  return c;
}
void set_freq(void)
{
  CH375_WR(0x0b);
  CH375_WR(0x17);
  CH375_WR(0xd8);
}