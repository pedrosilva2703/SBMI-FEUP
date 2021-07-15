#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include "serial_printf.h"

#include "i2c.h"
#include "lcd1602.h"  //Utiliza comunicacao i2c
#include "rtc3231.h"  //Utiliza comunicacao i2c

#define T1BOTTOM 65536-6250 // CNT = 2000, TP = 8
#define CP PB4
#define JK PB5

volatile uint16_t t=0;
volatile uint8_t min=0;

uint8_t ALARME_ST=0, SIRENE_ST=0, VALIDA_CARTAO_ST=0;

uint8_t sr_data=0, sr_old_data=0;
uint8_t RE_key=0, FE_key=0;
uint8_t A1_Protegida=0, A2_Protegida=0, A3_Protegida=0, A4_Protegida=0, A5_Protegida=0, A6_Protegida=0;
char key='O', new_key;
char string[16];

uint8_t protect=0;

uint8_t pos=0;
uint8_t valido=0;

char codigo[4]="1231";  //código numérico
char leitura[4];
uint8_t digito=0;


uint8_t mastertag[14]={2, 54, 50, 48, 48, 69, 50, 50, 51, 67, 51, 54, 48, 3};     //código do cartão
volatile uint8_t actualtag[14];
volatile uint8_t i=0;
volatile uint8_t serial_data;

ISR(USART_RX_vect){
  serial_data=UDR0;
  if(i<14){
    actualtag[i]=serial_data;
    i++;
  }
}



 
void init_timer(void) {
  // Parar o timer
  TCCR1B = 0;
  TIMSK1 = 0;

  // Definir valores e modo de funcionamento
  TCCR1A = 0; // Modo NORMAL
  TCNT1 = T1BOTTOM;           //T1BOTTOM 65536-6250  CNT = 2000, TP = 8


  // Iniciar o timer
  TIMSK1 |= (1 << TOIE1); //Ativar interrupt para overflow
  TCCR1B |= (1 << CS12); // Prescaler = 256
}

// Inicialize the pins of the arduino
void io_init(void)
{
  DDRB &= ~(1 << PB0 | 1 << PB1 | 1 << PB2 | 1 << PB3); // Matrix keypad input
  PORTB |= (1 << PB0 | 1 << PB1 | 1 << PB2 | 1 << PB3); // Pullup of the inputs


  DDRB |= (1 << PB4 | 1 << PB5);  // Shift register pins
  PORTB &= ~(1 << PB4 | 1 << PB5); // Inicialize at 0V


  DDRC |= (1 << PC0 | 1 << PC1 | 1 << PC2 | 1 << PC6); // Buzzer+Leds+Reset Outputs
  PORTC &= ~(1 << PC0 | 1 << PC1 | 1 << PC2);  // Inicialize at 0V
  PORTC |= (1 << PC6); // Pullup of Reset pin


  DDRD &= ~(1 << PD2 | 1 << PD3 | 1 << PD4 | 1 << PD5 | 1 << PD6 | 1 << PD7); // Sensor/Button inputs
  

}

//Read sensors/buttons
uint8_t read_S1(){
  return (PIND & (1<<PD7) );    // O S1 é o unico que esta ligado ao sensor PIR, os outros sao botoes para simular o funcionamento
}
uint8_t read_S2(){
  return !(PIND & (1<<PD6) );
}
uint8_t read_S3(){
  return !(PIND & (1<<PD5) );
}
uint8_t read_S4(){
  return !(PIND & (1<<PD4) );
}
uint8_t read_S5(){
  return !(PIND & (1<<PD3) );
}
uint8_t read_S6(){
  return !(PIND & (1<<PD2) );
}

//Set LEDS or BUZZER
void set_REDLED(uint8_t on){
  if (on) {
    PORTC |= (1 << PC0);
  } else {
    PORTC &= ~(1 << PC0);
  }
}
void set_GREENLED(uint8_t on){
  if (on) {
    PORTC |= (1 << PC1);
  } else {
    PORTC &= ~(1 << PC1);
  }
}
void set_BUZZER(uint8_t on){
  if (on) {
    PORTC |= (1 << PC2);
  } else {
    PORTC &= ~(1 << PC2);
  }
}

// Occurs every 100ms
ISR(TIMER1_OVF_vect) {
  
  if(t) t--; // Decrement timer t
  if(!t && min){
    t=600;
    min--;
  }
  TCNT1 = T1BOTTOM; // Restart countdown
}

// Turn CP of the shift register to LOW
void CP_Low(){
  PORTB &= ~(1 << CP);
  return;
}

// Turn CP of the shift register to HIGH
void CP_High(){
  PORTB |= (1 << CP);
  return;
}

// Turn J and NOT K of the shift register to LOW
void JK_Low(){
  PORTB &= ~(1 << JK);
  return;
}

// Turn J and NOT K of the shift register to HIGH
void JK_High(){
  PORTB |= (1 << JK);
  return;
}

// Read the current value of the (decrementing) t timer
uint16_t get_t(void) {
  uint16_t temp;
  cli();
  temp = t;
  sei();
  return temp;
}

// Define the outputs of the shift register to read the keypad
// The argument is a binary number, where its 4 LSBs define the corresponding outputs
// (ex: 0b00001110 defines 1st output as 0V and allows the reading of the 1st row)
void sr_out(uint8_t in) {
  
  if(in == sr_old_data) {
    return;
  }
  else {
    CP_Low();
  for (int i = 0; i < 4; i++) {
    if (in & (256>>i+5)) {
      JK_High();
      
    }
    CP_High();
    CP_Low();
    JK_Low();
  }
  CP_Low();

  sr_old_data=in;
  return;
  }
  
}

// Reset the outputs of the shift register
void sr_reset() {
  CP_Low();
  JK_Low();

  for (int i = 0; i <4; i++) {
    CP_High();
    CP_Low();
  }

  return;
}

// Detect which column is being pressed, and return the number of the column
uint8_t read_column() {
  
  if (!(PINB & (1<<PB0))) return 1;
  else if (!(PINB & (1<<PB1))) return 2;
  else if (!(PINB & (1<<PB2))) return 3;
  else if (!(PINB & (1<<PB3))) return 4;

  else return 0;

}

// Read any key from the matrix keypad and return the corresponding character
char read_key() {
  // Ler primeira linha
  sr_data=0b00001110;
  sr_out(sr_data);
  if (read_column()==1) return '1';
  else if (read_column()==2) return '2';
  else if (read_column()==3) return '3';
  else if (read_column()==4) return 'A';

  // Ler segunda linha
  sr_data=0b00001101;
  sr_out(sr_data);
  if (read_column()==1) return '4';
  else if (read_column()==2) return '5';
  else if (read_column()==3) return '6';
  else if (read_column()==4) return 'B';

  // Ler terceira linha
  sr_data=0b00001011;
  sr_out(sr_data);
  if (read_column()==1) return '7';
  else if (read_column()==2) return '8';
  else if (read_column()==3) return '9';
  else if (read_column()==4) return 'C';

  // Ler quarta linha
  sr_data=0b00000111;
  sr_out(sr_data);
  if (read_column()==1) return '*';
  else if (read_column()==2) return '0';
  else if (read_column()==3) return '#';
  else if (read_column()==4) return 'D';

  // Se não ler nenhuma tecla, retorna O (Óh)
  return 'O';
}


void MenuPrincipal(void){
  lcd1602_goto_xy(0,0);
  lcd1602_send_string("Protected Areas:");
  if( A1_Protegida==1 ){
    lcd1602_goto_xy(0,1);
    lcd1602_send_char('1');
  }
  if( A2_Protegida==1 ){
    lcd1602_goto_xy(1,1);
    lcd1602_send_char('2');
  }
  if( A3_Protegida==1 ){
    lcd1602_goto_xy(2,1);
    lcd1602_send_char('3');
  }
  if( A4_Protegida==1 ){
    lcd1602_goto_xy(3,1);
    lcd1602_send_char('4');
  }
  if( A5_Protegida==1 ){
    lcd1602_goto_xy(4,1);
    lcd1602_send_char('5');
  }
  if( A6_Protegida==1 ){
    lcd1602_goto_xy(5,1);
    lcd1602_send_char('6');
  }
}

void MenuRelogio(void){
  struct rtc_time time;
  struct rtc_date date;
  rtc3231_read_time(&time);
  rtc3231_init();
  rtc3231_read_date(&date);
  rtc3231_init();
  lcd1602_goto_xy(0,0);
  sprintf(string,"Time: %u:%u:%u  ", time.hour, time.min, time.sec);
  lcd1602_send_string(string);
  lcd1602_goto_xy(0,1);
  sprintf(string,"Date: %u-%u-%u  ", date.day, date.month, date.year);
  lcd1602_send_string(string);
  
}

void MenuPermissoes(void){
  lcd1602_goto_xy(0,0);
  lcd1602_send_string("A-Read Code");
  lcd1602_goto_xy(0,1);
  lcd1602_send_string("B-Read Card");
}
void MenuCode(void){
  lcd1602_goto_xy(0,0);
  lcd1602_send_string("A-Confirm");
  lcd1602_goto_xy(0,1);
  lcd1602_send_string("B-Cancel");
  
  if(RE_key && digito<4 && (key=='0'||key=='1'||key=='2'||key=='3'||key=='4'||key=='5'||key=='6'||key=='7'||key=='8'||key=='9')){
    leitura[digito]=key;
    lcd1602_goto_xy(11+digito, 0);
    lcd1602_send_char(key);
    digito++;

  }
  
  if(digito==4){
    if( leitura[0]==codigo[0] && leitura[1]==codigo[1] && leitura[2]==codigo[2] && leitura[3]==codigo[3] ) valido=1;
    else valido=0;
  }

}

void MenuCard(void){

  lcd1602_goto_xy(0,0);
  lcd1602_send_string("A-Confirm");
  lcd1602_goto_xy(0,1);
  lcd1602_send_string("B-Cancel");

  if(i==14){
    cli();
    if( mastertag[1]==actualtag[1] && mastertag[2]==actualtag[2] && mastertag[3]==actualtag[3] 
        && mastertag[4]==actualtag[4] && mastertag[5]==actualtag[5] && mastertag[6]==actualtag[6]
        && mastertag[7]==actualtag[7] && mastertag[8]==actualtag[8] && mastertag[9]==actualtag[9]
        && mastertag[10]==actualtag[10] && mastertag[11]==actualtag[11] && mastertag[12]==actualtag[12]){
      valido=1;    
    }
    else valido=0;
    sei();
  }


}
void MenuEdit(void){
  lcd1602_goto_xy(0,0);
  lcd1602_send_string("A-Protect Area");
  lcd1602_goto_xy(0,1);
  lcd1602_send_string("B-Unprotect Area");
}

void MenuSelect(void){
  lcd1602_goto_xy(0,0);
  lcd1602_send_string("Select Areas");
  lcd1602_goto_xy(0,1);
  lcd1602_send_string("A-Confirm");

  //Atualiza protecao das areas  
  if(RE_key && key=='1'){
    A1_Protegida=protect;
    eeprom_write_byte(22, protect);
  }
  else if(RE_key && key=='2'){
    A2_Protegida=protect;
    eeprom_write_byte(23, protect);
  }
  else if(RE_key && key=='3'){
    A3_Protegida=protect;
    eeprom_write_byte(24, protect);
  }
  else if(RE_key && key=='4'){
    A4_Protegida=protect;
    eeprom_write_byte(25, protect);
  }
  else if(RE_key && key=='5'){
    A5_Protegida=protect;
    eeprom_write_byte(26, protect);
  }
  else if(RE_key && key=='6'){
    A6_Protegida=protect;
    eeprom_write_byte(27, protect);
  }

  //Imprime
  if( A1_Protegida==protect ){
    lcd1602_goto_xy(10,1);
    lcd1602_send_char('1');
  }
  if( A2_Protegida==protect ){
    lcd1602_goto_xy(11,1);
    lcd1602_send_char('2');
  }
  if( A3_Protegida==protect ){
    lcd1602_goto_xy(12,1);
    lcd1602_send_char('3');
  }
  if( A4_Protegida==protect ){
    lcd1602_goto_xy(13,1);
    lcd1602_send_char('4');
  }
  if( A5_Protegida==protect ){
    lcd1602_goto_xy(14,1);
    lcd1602_send_char('5');
  }
  if( A6_Protegida==protect ){
    lcd1602_goto_xy(15,1);
    lcd1602_send_char('6');
  }

}

void MenuAlert(void){
  lcd1602_goto_xy(0,0);
  lcd1602_send_string("ALERT! AREA ");

  sprintf(string, "%u", eeprom_read_byte(pos) ); 
  lcd1602_goto_xy(12,0);
  lcd1602_send_string(string);

  lcd1602_goto_xy(0,1);
  sprintf(string, "%u:%u:%u", eeprom_read_byte(pos+1),eeprom_read_byte(pos+2),eeprom_read_byte(pos+3) );
  lcd1602_send_string(string);

  lcd1602_goto_xy(11,1);
  sprintf(string, "%u:%u ", min, t/10 );
  lcd1602_send_string(string);
}

void MenuLog(void){
  lcd1602_goto_xy(0,0);
  lcd1602_send_string("Alarm LOG");
  lcd1602_goto_xy(0,1);
  lcd1602_send_string("A-Check  B-Back");
}

void MenuShowL1(void){
  lcd1602_goto_xy(0,0);
  lcd1602_send_string("LOG1");
  lcd1602_goto_xy(0,1);
  lcd1602_send_string("Area");

  sprintf(string, "%u", eeprom_read_byte(1) ); 
  lcd1602_goto_xy(5,1);
  lcd1602_send_string(string);

  lcd1602_goto_xy(7,0);
  sprintf(string, "%u:%u:%u", eeprom_read_byte(2),eeprom_read_byte(3),eeprom_read_byte(4) );
  lcd1602_send_string(string);

  lcd1602_goto_xy(7,1);
  sprintf(string, "%u-%u-%u", eeprom_read_byte(5),eeprom_read_byte(6),eeprom_read_byte(7) );
  lcd1602_send_string(string);
}

void MenuShowL2(void){
  lcd1602_goto_xy(0,0);
  lcd1602_send_string("LOG2");
  lcd1602_goto_xy(0,1);
  lcd1602_send_string("Area");

  sprintf(string, "%u", eeprom_read_byte(8) ); 
  lcd1602_goto_xy(5,1);
  lcd1602_send_string(string);

  lcd1602_goto_xy(7,0);
  sprintf(string, "%u:%u:%u", eeprom_read_byte(9),eeprom_read_byte(10),eeprom_read_byte(11) );
  lcd1602_send_string(string);

  lcd1602_goto_xy(7,1);
  sprintf(string, "%u-%u-%u", eeprom_read_byte(12),eeprom_read_byte(13),eeprom_read_byte(14) );
  lcd1602_send_string(string);
}

void MenuShowL3(void){
  lcd1602_goto_xy(0,0);
  lcd1602_send_string("LOG3");
  lcd1602_goto_xy(0,1);
  lcd1602_send_string("Area");

  sprintf(string, "%u", eeprom_read_byte(15) ); 
  lcd1602_goto_xy(5,1);
  lcd1602_send_string(string);

  lcd1602_goto_xy(7,0);
  sprintf(string, "%u:%u:%u", eeprom_read_byte(16),eeprom_read_byte(17),eeprom_read_byte(18) );
  lcd1602_send_string(string);

  lcd1602_goto_xy(7,1);
  sprintf(string, "%u-%u-%u", eeprom_read_byte(19),eeprom_read_byte(20),eeprom_read_byte(21) );
  lcd1602_send_string(string);
}

uint8_t alarmeDISPARA(void){
  if( A1_Protegida==1 && read_S1() ) return 1;
  if( A2_Protegida==1 && read_S2() ) return 2;
  if( A3_Protegida==1 && read_S3() ) return 3;
  if( A4_Protegida==1 && read_S4() ) return 4;
  if( A5_Protegida==1 && read_S5() ) return 5;
  if( A6_Protegida==1 && read_S6() ) return 6;
  return 0;
}

int main(void)
{
  uint8_t x, y;


  uint8_t disparou=0;

  char tempo[16];
  struct rtc_time time;
  struct rtc_date date;
  time.hour = 15;
  time.min=04;
  time.sec=0;
  date.day=13;
  date.month=12;
  date.year=20;
  
  uint8_t serial_data=0;
  
  i2c_init();

  rtc3231_init();
  //rtc3231_write_time(&time);    //Use this function to set up RTC time
  //rtc3231_write_date(&date);    //Use this function to set up RTC date
  
  lcd1602_init();
  lcd1602_clear();

  init_timer();
  io_init();                           
  sei();                              // Enable interrupts after every initialization
 
  printf_init();                      // Init the serial port to have the ability to printf
  printf("Serial I/O activated\n");   // into a terminal  

  sr_reset();                         // Reset the outputs of the shift register


  pos = eeprom_read_byte(0);
  if(pos!=1 && pos!=8 && pos!=15){
    pos=1;
    eeprom_write_byte(0,1);
  }
  A1_Protegida = eeprom_read_byte(22);
  A2_Protegida = eeprom_read_byte(23);
  A3_Protegida = eeprom_read_byte(24);
  A4_Protegida = eeprom_read_byte(25);
  A5_Protegida = eeprom_read_byte(26);
  A6_Protegida = eeprom_read_byte(27);

  while (1) {                         // Hot loop
    
    
    // Verificar se ocorreu uma leitura do teclado
    cli();
    new_key = read_key();
    sei();

    if (new_key != key) {
      RE_key = (key=='O' && new_key!='O');
      FE_key = (key!='O' && new_key=='O');
      key=new_key;
      
    } else {
      RE_key = 0;
      FE_key = 0;
    }

    // Verificar se existem alarmes a serem disparados
    disparou=alarmeDISPARA();
    


    // -------------------------------------- //
    // ------------- Do stuff --------------- //

    //---------------------------------------CALCULATE NEXT STATE OF ALARME---------------------------------------//
    

    //*******************************************// ALARME_ST=0 //************************************************//
    if(ALARME_ST==0 && (RE_key && key=='D' ) ){
      lcd1602_clear();
      ALARME_ST=1;
    }
    else if(ALARME_ST==0 && (RE_key && key=='C' ) ){
      lcd1602_clear();
      ALARME_ST=2;
    }
    else if(ALARME_ST==0 && disparou ){
      lcd1602_clear();
      ALARME_ST=7;

      rtc3231_read_time(&time);
      rtc3231_init();
      rtc3231_read_date(&date);
      rtc3231_init();

      pos+=7;
      if(pos>21){
        pos=1;
      }
      eeprom_write_byte(0,pos);
      
      eeprom_write_byte(pos, disparou);
      eeprom_write_byte(pos+1, time.hour);
      eeprom_write_byte(pos+2, time.min);
      eeprom_write_byte(pos+3, time.sec);
      eeprom_write_byte(pos+4, date.day);
      eeprom_write_byte(pos+5, date.month);
      eeprom_write_byte(pos+6, date.year);


    }
    else if(ALARME_ST==0 && (RE_key && key=='B' ) ){
      lcd1602_clear();
      ALARME_ST=10;

    }
    
    //*******************************************// ALARME_ST=1 //************************************************//
    else if(ALARME_ST==1 && (RE_key && key=='D' ) ){
      lcd1602_clear();
      ALARME_ST=0;
    }

    //*******************************************// ALARME_ST=2 //************************************************//
    else if(ALARME_ST==2 && (RE_key && key=='A' ) ){
      lcd1602_clear();
      ALARME_ST=3;
      
      valido=0;
      digito=0;
      leitura[0]='0';
      leitura[1]='0';
      leitura[2]='0';
      leitura[3]='0';
      lcd1602_goto_xy(11,0);
      lcd1602_send_string("****");
    }
    else if(ALARME_ST==2 && (RE_key && key=='B' ) ){
      lcd1602_clear();
      ALARME_ST=4;
      
      valido=0;
      i=0;
      for(x=0; x<14; x++){
        actualtag[x]=0;
      } 
    }
    else if(ALARME_ST==2 && (RE_key && key=='C' ) ){
      lcd1602_clear();
      ALARME_ST=0;
    }

    //*******************************************// ALARME_ST=3 //************************************************//
    else if(ALARME_ST==3 && (RE_key && key=='A' ) && valido==1 ){
      lcd1602_clear();
      ALARME_ST=5;
    }
    else if(ALARME_ST==3 && (RE_key && key=='A' ) && valido==0 ){
      lcd1602_clear();
      ALARME_ST=3;
      
      digito=0;
      leitura[0]='0';
      leitura[1]='0';
      leitura[2]='0';
      leitura[3]='0';
      lcd1602_goto_xy(11,1);
      lcd1602_send_string("WRONG");

      lcd1602_goto_xy(11,0);
      lcd1602_send_string("****");
    }
    else if(ALARME_ST==3 && (RE_key && key=='B' ) ){
      lcd1602_clear();
      ALARME_ST=2;
    }
    
    //*******************************************// ALARME_ST=4 //************************************************//
    else if(ALARME_ST==4 && (RE_key && key=='A' ) && valido==1 ){
      lcd1602_clear();
      ALARME_ST=5;
    }
    else if(ALARME_ST==4 && (RE_key && key=='A' ) && valido==0 ){
      lcd1602_clear();
      ALARME_ST=4;
      
    
      lcd1602_goto_xy(11,1);
      lcd1602_send_string("WRONG");
      valido=0;
      i=0;
      for(x=0; x<14; x++){
        actualtag[x]=0;
      }

    }
    else if(ALARME_ST==4 && (RE_key && key=='B' ) ){
      lcd1602_clear();
      ALARME_ST=2;
    }

    //*******************************************// ALARME_ST=5 //************************************************//
    else if(ALARME_ST==5 && (RE_key && key=='A' ) ){
      lcd1602_clear();
      ALARME_ST=6;
      protect=1;
    }
    else if(ALARME_ST==5 && (RE_key && key=='B' ) ){
      lcd1602_clear();
      ALARME_ST=6;  
      protect=0;
    }

    //*******************************************// ALARME_ST=6 //************************************************//
    else if(ALARME_ST==6 && (RE_key && key=='A' ) ){
      lcd1602_clear();
      ALARME_ST=0;
    }

    //*******************************************// ALARME_ST=7 //************************************************//
    else if(ALARME_ST==7 && (RE_key && key=='A' ) ){
      lcd1602_clear();
      ALARME_ST=8;
      
      valido=0;
      digito=0;
      leitura[0]='0';
      leitura[1]='0';
      leitura[2]='0';
      leitura[3]='0';
      lcd1602_goto_xy(11,0);
      lcd1602_send_string("****");
    }
    else if(ALARME_ST==7 && (RE_key && key=='B' ) ){
      lcd1602_clear();
      ALARME_ST=9;
      
      valido=0;
      i=0;
      for(x=0; x<14; x++){
        actualtag[x]=0;
      } 
    }
    else if(ALARME_ST==7 && SIRENE_ST==1 ){
      lcd1602_clear();
      ALARME_ST=0;
      
    }

    //*******************************************// ALARME_ST=8 //************************************************//
    else if(ALARME_ST==8 &&  ( ( (RE_key && key=='A' ) && valido==1) || SIRENE_ST==1 ) ){
      lcd1602_clear();
      ALARME_ST=0;
    }
    else if(ALARME_ST==8 && (RE_key && key=='A' ) && valido==0 ){
      lcd1602_clear();
      ALARME_ST=8;
      
      digito=0;
      leitura[0]='0';
      leitura[1]='0';
      leitura[2]='0';
      leitura[3]='0';
      lcd1602_goto_xy(11,1);
      lcd1602_send_string("WRONG");
      lcd1602_goto_xy(11,0);
      lcd1602_send_string("****");
    }
    else if(ALARME_ST==8 && (RE_key && key=='B' ) ){
      lcd1602_clear();
      ALARME_ST=7;
    }

    //*******************************************// ALARME_ST=9 //************************************************//
    else if(ALARME_ST==9 && ( ( (RE_key && key=='A' ) && valido==1) || SIRENE_ST==1 ) ){
      lcd1602_clear();
      ALARME_ST=0;
    }
    else if(ALARME_ST==9 && (RE_key && key=='A' ) && valido==0 ){
      lcd1602_clear();
      ALARME_ST=9;
      
    
      lcd1602_goto_xy(11,1);
      lcd1602_send_string("WRONG");
      valido=0;
      i=0;
      for(x=0; x<14; x++){
        actualtag[x]=0;
      }
    }
    else if(ALARME_ST==9 && (RE_key && key=='B' ) ){
      lcd1602_clear();
      ALARME_ST=7;
    }


    //*******************************************// ALARME_ST=10 //************************************************//
    else if(ALARME_ST==10 && (RE_key && key=='A' ) ){
      lcd1602_clear();
      ALARME_ST=11;
    }
    else if(ALARME_ST==10 && (RE_key && key=='B' ) ){
      lcd1602_clear();
      ALARME_ST=0;
    }
    
    //*******************************************// ALARME_ST=11 //************************************************//
    else if(ALARME_ST==11 && (RE_key && key=='A' ) ){
      lcd1602_clear();
      ALARME_ST=12;
    }
    
    //*******************************************// ALARME_ST=12 //************************************************//
    else if(ALARME_ST==12 && (RE_key && key=='A' ) ){
      lcd1602_clear();
      ALARME_ST=13;
    }
    //*******************************************// ALARME_ST=13 //************************************************//
    else if(ALARME_ST==13 && (RE_key && key=='A' ) ){
      lcd1602_clear();
      ALARME_ST=10;
    }


    //---------------------------------------CALCULATE NEXT STATE OF SIRENE---------------------------------------//
    
    //SIRENE_ST=0
    if(SIRENE_ST==0 && ALARME_ST==0){
      SIRENE_ST=1;
    }

    //SIRENE_ST=1
    else if(SIRENE_ST==1 && ALARME_ST==7){
      SIRENE_ST=2;
      min=5;
      t=0;
    }

    //SIRENE_ST=2
    else if(SIRENE_ST==2 && (ALARME_ST==0 || (min==0 && t==0) ) ){
      SIRENE_ST=1;
    }



    //---------------------------------------CALCULATE OUTPUT OF ALARME---------------------------------------//
    if(ALARME_ST==0) MenuPrincipal();
    if(ALARME_ST==1) MenuRelogio();
    if(ALARME_ST==2) MenuPermissoes();
    if(ALARME_ST==3) MenuCode();
    if(ALARME_ST==4) MenuCard();
    if(ALARME_ST==5) MenuEdit();
    if(ALARME_ST==6) MenuSelect();
    if(ALARME_ST==7) MenuAlert();
    if(ALARME_ST==8) MenuCode();
    if(ALARME_ST==9) MenuCard();
    if(ALARME_ST==10) MenuLog();
    if(ALARME_ST==11) MenuShowL1();
    if(ALARME_ST==12) MenuShowL2();
    if(ALARME_ST==13) MenuShowL3();

    //---------------------------------------CALCULATE OUTPUT OF SIRENE---------------------------------------//
    set_GREENLED( SIRENE_ST==1 );
    set_REDLED( SIRENE_ST==2 );
    set_BUZZER( SIRENE_ST==2 );


    // ------------------------------------------ //


    //printf("%u %u %u %u %u %u\n", read_S1(), read_S2(), read_S3(), read_S4(), read_S5(), read_S6());
    //printf("%u %u %u %u \n", t/10, min, ALARME_ST, SIRENE_ST);
    //printf("%u %u %u %u %u %u %u\n", serial_data, actualtag[0], actualtag[1], actualtag[2], actualtag[3], actualtag[4], actualtag[5] );
    //printf("%u %u %u\n", ALARME_ST, VALIDA_CARTAO_ST, serial_data);
  }
}

