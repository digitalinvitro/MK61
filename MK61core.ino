#include <LiquidCrystal.h>

#define NOP 0

void Displayed(void);
void RefreshXBuff(void);
void PressKey(uint16_t kbd_code);
 
LiquidCrystal lcd(7, 6, 5, 4, 3, 2);   

const char MNEMO[48][4] PROGMEM = {
/* 0x40 */  
  {'-','>','0',' '}, {'-','>','1',' '}, {'-','>','2',' '}, {'-','>','3',' '}, {'-','>','4',' '}, {'-','>','5',' '}, {'-','>','6',' '}, {'-','>','7',' '},
  {'-','>','8',' '}, {'-','>','9',' '}, {'-','>','A',' '}, {'-','>','B',' '}, {'-','>','C',' '}, {'-','>','D',' '}, {'-','>','E',' '}, {'-','>','F',' '},
/* 0x50 */  
  {'H','L','T',' '}, {'B','R',' ',' '}, {'B','R','0',' '}, {'C','A','L','L'}, {'K','N','O','P'}, {' ',' ',' ',' '}, {' ',' ',' ',' '}, {'!','=','0',' '},
  {'F','L','2',' '}, {'>','=','0',' '}, {'F','L','3',' '}, {'F','L','1',' '}, {'<','0',' ',' '}, {'F','L','0',' '}, {'=','0',' ',' '}, {' ',' ',' ',' '},
/* 0x60 */  
  {'0','-','>',' '}, {'1','-','>',' '}, {'2','-','>',' '}, {'3','-','>',' '}, {'4','-','>',' '}, {'5','-','>',' '}, {'6','-','>',' '}, {'7','-','>',' '},
  {'8','-','>',' '}, {'9','-','>',' '}, {'A','-','>',' '}, {'B','-','>',' '}, {'C','-','>',' '}, {'D','-','>',' '}, {'E','-','>',' '}, {'F','-','>',' '},
};

float    data[16];
byte     code[256];
byte     input[40];
byte     *ihead;
byte     IP;
byte     TIP;

#define  KBD_DELAY  10000

/* ===== Коды системы комманд МК61-52 === */
#define       MK61_CX       0x0D
#define       MK61_BI       0x0E
#define       MK61_ADD      0x10
#define       MK61_SUB      0x11
#define       MK61_MUL      0x12
#define       MK61_DIV      0x13
#define       MK61_XY       0x14
#define       MK61_10POWX   0x15
#define       MK61_EXPX     0x16
#define       MK61_LNX      0x17
#define       MK61_LGX      0x18
#define       MK61_SQRTX    0x21
#define       MK61_SQRX     0x22
#define       MK61_REVX     0x23

#define  AUTO  0
#define  PRG   1
byte     MODE;

bool     RequestReady;
bool     Play;

byte     SignalsRefresh;
#define  SIG_LCD_LINE0_REFRESH 1
#define  SIG_LCD_LINE1_REFRESH 2

byte     *head_buff;
byte     xbuff[12];

float    X, Y, Z, T;
char     ibuff[12];
byte     SIG_PUSH;

uint8_t  scan;
//uint8_t  preKbCode, kbcode;
uint16_t wait;

#define  SHIFT_F    1
#define  SHIFT_K    8
#define  SHIFT_WR   2
#define  SHIFT_RD   4
char     shift;

byte customCharX[8] = {
  B10100,
  B10100,
  B01000,
  B01000,
  B10100,
  B10100,
  B00000,
  B00000
};

byte customCharY[8] = {
  B10100,
  B10100,
  B10100,
  B01000,
  B01000,
  B01000,
  B00000,
  B00000
};

void SerialPrintHEX(byte data){
  if(data < 0x10) Serial.write('0');
  Serial.print(data, HEX);   
}
 
void setup()
{
/* Keyboard setup */
// Scan matrix line
 pinMode(13, OUTPUT); 
 pinMode(17, OUTPUT); 
 pinMode(19, OUTPUT); 
 pinMode(21, OUTPUT); 
 pinMode(23, OUTPUT); 
 digitalWrite(13, LOW);
 digitalWrite(17, HIGH);
 digitalWrite(19, HIGH);
 digitalWrite(21, HIGH);
 digitalWrite(23, HIGH);
 scan = 0;
// Scan matrix column
 pinMode(14, INPUT); 
 pinMode(16, INPUT); 
 pinMode(18, INPUT); 
 pinMode(20, INPUT); 
 pinMode(22, INPUT); 
 pinMode(24, INPUT); 
 digitalWrite(14, HIGH);
 digitalWrite(16, HIGH);
 digitalWrite(18, HIGH);
 digitalWrite(20, HIGH);
 digitalWrite(22, HIGH);
 digitalWrite(24, HIGH);
// kbcode = 0xFF;
// preKbCode = 0xFF;
 shift = 0;
 wait = 0;
 
 Serial.begin(9600);
 Serial.println("MK-61/52...\n>>");
 ihead = &input[0];
 RequestReady = false;
 SignalsRefresh = SIG_LCD_LINE0_REFRESH | SIG_LCD_LINE1_REFRESH;
 Play = false;
 IP = 0;
 head_buff = &xbuff[0];
 SIG_PUSH = 0;
 
 /*for(i=0;i<200;i++) code[i] = NOP;
 code[0] = 0x68;
 code[1] = 0x66;
 code[2] = 0x12;
 code[3] = 0x65;
 code[4] = 0x69;
 code[5] = 0x12;
 code[6] = 0x11;
 code[7] = 0x50;
 code[8] = 0x50;*/
  
 lcd.begin(16, 2);  // Инициализирует LCD 16x2
 lcd.createChar(0, customCharX);
 lcd.createChar(1, customCharY);
 MODE = AUTO;
 Displayed();
}

void pop(){
   X = Y;
   Y = Z;
   Z = T;
}

void push(){
   T = Z;
   Z = Y;
   Y = X;
}

byte convert_xbuff(){
/*  
 *   Буфер ввода содержаший число не закрыт - необходимо конвертировать и внести в X число
 *     1. Если сигнал SIG_PUSH установлен то продвинем дополнительно стек вниз X->Y->Z->T
 *     2. в X записывается число из буфера ввода xbuff
 */
 if( head_buff != &xbuff[0] ){
/* Закрываем буфер X */
   if(SIG_PUSH){
     push();
     Serial.print("push ");
   } else {
     SIG_PUSH = 0xFF;
   }
 
   *head_buff++ = 0;
   head_buff = &xbuff[0];
   Serial.print((const char*) &xbuff[0]);
   Serial.print('#');
   X = atof((const char*) &xbuff[0]);
   Serial.println(X);
  
   return 0;
 } 
 return 0xFF; 
}

void execute(byte OPCODE){
float temp;
register byte address;

/* Отсечем коды заполняющие буфер ввода: 0,1,2,3,4,5,6,7,8,9 */
  if( OPCODE < 10 ){
       *head_buff++ = ('0' + OPCODE);
       *head_buff = 0;
       RefreshXBuff();
    return;
  }

  convert_xbuff();

  switch( OPCODE ){
    case MK61_BI:
            if( head_buff != &xbuff[0] ){
               /* Закрываем буфер X */
               *head_buff++ = 0;
               head_buff = &xbuff[0];
               X = atof((const char*) &xbuff[0]);
            }   
            SIG_PUSH = 0;
            push();
          break;
    case MK61_CX:
            head_buff = &xbuff[0];
            X = 0;
            SIG_PUSH = 0;
          break;
    case MK61_ADD: 
            Y = Y + X;
            pop();
          break;
    case MK61_SUB:  
            Y = Y - X;
            pop();
          break;
    case MK61_MUL:
            Y = Y * X;
            pop();
           break;
    case MK61_DIV:
            Y = Y / X;
            pop();
           break;
    case MK61_XY:
/* Размен содержимым между Y и  X   [ <-> ] */
            temp = X;
            X = Y;
            Y = temp;
           break;
    case MK61_LNX:
/* Натуральный логарифм X [ Ln(X) ] */    
            X = log(X);
           break;
    case MK61_LGX:
/* Десятичный логарифм X [ Lg(X) ] */    
            X = log10(X);
           break;

    case 0x20:
/* Вввод константы Пи в X   [ Pi ] */
            if( convert_xbuff() ) push();
            X = 3.1415926;
           break;
    case MK61_10POWX:
/* Возведение 10 в X   [ F10^X ] */
            X = pow(10,X);
           break;
    case MK61_EXPX:
/* Возведение E в X   [ Fe^X ] */
            X = exp(X);
           break;
    case MK61_SQRTX:
/* Корень из X   [ sqrt(X) ] */
            X = sqrt(X);
           break;
    case MK61_SQRX:
/* Возведение в квадрат   [ X^2 ] */
            X = X * X;
           break;
    case MK61_REVX:
/* деление 1 на X   [ sqrt(X) ] */
            X = 1/X;
           break;
    case 0x40:
    case 0x41:
    case 0x42:
    case 0x43:
    case 0x44:
    case 0x45:
    case 0x46:
    case 0x47:
    case 0x48:
    case 0x49:
    case 0x4A:
    case 0x4B:
    case 0x4C:
    case 0x4D:
    case 0x4E:
    case 0x4F:
/* Сохранение содержимого X в ячейку память 0...F   [ ППn ] */
            address = OPCODE - 0x40;
            data[address] = X;
           break;
    case 0x50:
/* остановка исполнения программы   [ С/П ] */
            Displayed();
            Play = false;
           break;
    case 0x51:
/* Безусловный переход. [БП ADDR]    */
            IP++;
            IP = (code[IP] - 1);
           break;        
    case 0x52:
/* Безусловный переход в начало сегмента. [В/O]    */
            IP = 0xFF;
           break;        
    case 0x57:
/* Условный переход. Если x != 0 продолжить [ Fx != 0 ]*/
            IP++;
            if( X == 0 ) IP = (code[IP] - 1);
           break;
    case 0x58:
/* Цикл по регистру 2. Если R2 != 0 то перейти на NN [ FL2 NN ]*/
            IP++;
            data[2] -= 1;
            if( data[2] != 0 ) IP = (code[IP] - 1);
           break;
    case 0x59:
/* Условный переход. Если x >= 0 продолжить [ Fx >= 0 ]*/
            IP++;
            if( X < 0 ) IP = (code[IP] - 1);
           break;
    case 0x5A:
/* Цикл по регистру 3. Если R3 != 0 то перейти на NN [ FL3 NN ]*/
            IP++;
            data[3] -= 1;
            if( data[3] != 0 ) IP = (code[IP] - 1);
           break;
    case 0x5B:
/* Цикл по регистру 1. Если R1 != 0 то перейти на NN [ FL1 NN ]*/
            IP++;
            data[1] -= 1;
            if( data[1] != 0 ) IP = (code[IP] - 1);
           break;
    case 0x5C:
/* Условный переход. Если x < 0 продолжить [ Fx < 0 ]*/
            IP++;
            if( X >= 0 ) IP = (code[IP] - 1);
           break;
    case 0x5D:
/* Цикл по регистру 0. Если R0 != 0 то перейти на NN [ FL0 NN ]*/
            IP++;
            data[0] -= 1;
            if( data[0] != 0 ) IP = (code[IP] - 1);
           break;
    case 0x5E:
/* Условный переход. Если x = 0 продолжить [ Fx = 0 ]*/
            IP++;
            if( X != 0 ) IP = (code[IP] - 1);
           break;
/* Чтение содержимого ячейки памяти 0...F в X  [ ИПn ] */
    case 0x60:
    case 0x61:
    case 0x62:
    case 0x63:
    case 0x64:
    case 0x65:
    case 0x66:
    case 0x67:
    case 0x68:
    case 0x69:
    case 0x6A:
    case 0x6B:
    case 0x6C:
    case 0x6D:
    case 0x6E:
    case 0x6F:
            push();
            address = OPCODE - 0x60;
            X = data[address];
           break;
/* Косвеная запись по содержимому ячейки памяти 0...F в X  [ КПn ] */
    case 0xB0:
    case 0xB1:
    case 0xB2:
    case 0xB3:
    case 0xB4:
    case 0xB5:
    case 0xB6:
    case 0xB7:
    case 0xB8:
    case 0xB9:
    case 0xBA:
    case 0xBB:
    case 0xBC:
    case 0xBD:
    case 0xBE:
    case 0xBF:
/* 
 *  Ксовенная запись по содержимому ячейке памяти 0...F в X  [ КПn ] 
 *    считанное содержимое из ячеки nn явдляется адресом и приводится к целому числу 
 *      при этом содержимое ячейки модифицируется приведенным значением
*/
            if( convert_xbuff() ) push();
            address = OPCODE - 0xB0;
            data[address] = (int32_t) data[address];  // модификация содержимого как целочисленного адреса
            address = (byte) data[address];           // адресуемся в страничке от 0..255
            if(address < 0x10){
               data[address] = X;                     // пишем X по адресу регистров 0..F
            } else {
               address -= 0x10;
               if(address < 0x10){
                    lcd.setCursor(address, 0);         // адресуем знакоместо в строке 
                    lcd.write((byte) X);                      
                    SignalsRefresh &= (~SIG_LCD_LINE0_REFRESH);
               }
            }
               
           break;
/* Чтение содержимого ячейки памяти 0...F в X  [ ИПn ] */
    case 0xD0:
    case 0xD1:
    case 0xD2:
    case 0xD3:
    case 0xD4:
    case 0xD5:
    case 0xD6:
    case 0xD7:
    case 0xD8:
    case 0xD9:
    case 0xDA:
    case 0xDB:
    case 0xDC:
    case 0xDD:
    case 0xDE:
    case 0xDF:
/* 
 *  Ксовенное чтение содержимого ячейки памяти 9...F в X  [ КИПn ] 
 *    считанное содержимое из ячеки nn явдляется адресом и приводится к целому числу 
 *      при этом содержимое ячейки модифицируется приведенным значением
*/
            if( convert_xbuff() ) push();
            address = OPCODE - 0xD0;
            data[address] = (int32_t) data[address];  // модификация содержимого как целочисленного адреса
            address = ((byte) data[address]) & 0x0F;           // запрет выхода адреса за значение 0..15
            X = data[address];                        // читаем в X косвенное значение 
           break;
  }

 Displayed(); 
}

uint8_t   prekbvector[5], kbvector[5];

void KbdScan(){
 register uint8_t i, mask = 1;

 if(scan == 5){
   if(wait++ < KBD_DELAY) return;
   wait = 0;
   scan = 0;

 
   for(i=0; i<5; i++){
     //Serial.print(prekbvector[i]);Serial.print('-');Serial.print(kbvector[i]);Serial.print(' ');
     if(prekbvector[i] != kbvector[i]){
       if(kbvector[i] == 0xFF){
         Serial.print(i);Serial.print('^');Serial.println(kbvector[i]);
       } else {
         Serial.print(i);Serial.print('v');Serial.println(kbvector[i]);
         PressKey((i<<8)|kbvector[i]);
       }
     }
   }
   //Serial.println();
   return;
 }

 switch(scan){
   case 0: 
      digitalWrite(23, HIGH);
      digitalWrite(13, LOW);
      break;
   case 1:
      digitalWrite(13, HIGH);
      digitalWrite(17, LOW);
      break;
   case 2: 
      digitalWrite(17, HIGH);
      digitalWrite(19, LOW);
      break;
   case 3: 
      digitalWrite(19, HIGH);
      digitalWrite(21, LOW);
      break;
   case 4: 
      digitalWrite(21, HIGH);
      digitalWrite(23, LOW);
 }

 i = 0xFF;
/* Читаем колонки клавиатурной матрицы */
 if(digitalRead(14) == 0) i &= ~mask;
 mask <<= 1;
 if(digitalRead(16) == 0) i &= ~mask;
 mask <<= 1;
 if(digitalRead(18) == 0) i &= ~mask;
 mask <<= 1;
 if(digitalRead(20) == 0) i &= ~mask;
 mask <<= 1;
 if(digitalRead(22) == 0) i &= ~mask;
 mask <<= 1;
 if(digitalRead(24) == 0) i &= ~mask;

 prekbvector[scan] = kbvector[scan];
 kbvector[scan] = i;
 scan++;
}
/*
void ExecuteStoreReg(uint8_t OPCODE){
  execute(OPCODE);
  shift &= ~SHIFT_WR;
  Refresh_SHIFT();
}
void ExecuteIndirectStoreReg(uint8_t OPCODE){
  execute(OPCODE);
  shift &= ~(SHIFT_WR|SHIFT_K);
  Refresh_SHIFT();
}
void ExecuteLoadReg(uint8_t OPCODE){
  execute(OPCODE);
  shift &= ~SHIFT_RD;
  Refresh_SHIFT();
}
void ExecuteIndirectLoadReg(uint8_t OPCODE){
  execute(OPCODE);
  shift &= ~(SHIFT_RD|SHIFT_K);
  Refresh_SHIFT();
}
void ExecuteAndClearF(uint8_t OPCODE){
  execute(OPCODE);
  shift &= ~SHIFT_F;
  Refresh_SHIFT();
}
*/
void PressKey(uint16_t kbd_code){
  SignalsRefresh |= SIG_LCD_LINE0_REFRESH;
  switch( ((uint16_t (shift)) << 11) | kbd_code){

      case (SHIFT_K << 11) | (SHIFT_WR << 11) | 0x1000|223:  /* KX->П0 */
        execute(0xB0);
        break;
      case (SHIFT_K << 11) | (SHIFT_WR << 11) | 0x1000|239:  /* KX->П1 */
        execute(0xB1);
        break;
      case (SHIFT_K << 11) | (SHIFT_WR << 11) | 0x1100|239:  /* KX->П2 */
        execute(0xB2);
        break;
      case (SHIFT_K << 11) | (SHIFT_WR << 11) | 0x1200|239:  /* KX->П3 */
        execute(0xB3);
        break;
      case (SHIFT_K << 11) | (SHIFT_WR << 11) | 0x1000|251:  /* KX->П4 */
        execute(0xB4);
        break;
      case (SHIFT_K << 11) | (SHIFT_WR << 11) | 0x1100|251:  /* KX->П5 */
        execute(0xB5);
        break;
      case (SHIFT_K << 11) | (SHIFT_WR << 11) | 0x1200|251:  /* KX->П6 */
        execute(0xB6);
        break;
      case (SHIFT_K << 11) | (SHIFT_WR << 11) | 0x1000|247:  /* KX->П7 */
        execute(0xB7);
        break;
      case (SHIFT_K << 11) | (SHIFT_WR << 11) | 0x1100|247:  /* KX->П8 */
        execute(0xB8);
        break;
      case (SHIFT_K << 11) | (SHIFT_WR << 11) | 0x1200|247:  /* KX->П9 */
        execute(0xB9);
        break;
      case (SHIFT_K << 11) | (SHIFT_WR << 11) | 0x1100|223:  /* KX->Пa */
        execute(0xBA);
        break;
      case (SHIFT_K << 11) | (SHIFT_WR << 11) | 0x1200|223:  /* KX->Пb */
        execute(0xBB);
        break;
      case (SHIFT_K << 11) | (SHIFT_WR << 11) | 0x1300|223:  /* KX->Пc */
        execute(0xBC);
        break;
      case (SHIFT_K << 11) | (SHIFT_WR << 11) | 0x1400|223:  /* KX->Пd */
        execute(0xBD);
        break;
      case (SHIFT_K << 11) | (SHIFT_WR << 11) | 0x1400|239:  /* KX->Пe */
        execute(0xBE);
        break;
      case (SHIFT_K << 11) | (SHIFT_WR << 11) | 0x1400|251:  /* KX->Пf */
        execute(0xBF);
        break;

      case (SHIFT_K << 11) | (SHIFT_RD << 11) | 223:  /* KП->X0 */
        execute(0xD0);
        break;
      case (SHIFT_K << 11) | (SHIFT_RD << 11) | 239:  /* KП->X1 */
        execute(0xD1);
        break;
      case (SHIFT_K << 11) | (SHIFT_RD << 11) | 0x0100|239:  /* KП->X2 */
        execute(0xD2);
        break;
      case (SHIFT_K << 11) | (SHIFT_RD << 11) | 0x0200|239:  /* KП->X3 */
        execute(0xD3);
        break;
      case (SHIFT_K << 11) | (SHIFT_RD << 11) | 251:  /* KП->X4 */
        execute(0xD4);
        break;
      case (SHIFT_K << 11) | (SHIFT_RD << 11) | 0x0100|251:  /* KП->X5 */
        execute(0xD5);
        break;
      case (SHIFT_K << 11) | (SHIFT_RD << 11) | 0x0200|251:  /* KП->X6 */
        execute(0xD6);
        break;
      case (SHIFT_K << 11) | (SHIFT_RD << 11) | 247:  /* KП->X7 */
        execute(0xD7);
        break;
      case (SHIFT_K << 11) | (SHIFT_RD << 11) | 0x0100|247:  /* KП->X8 */
        execute(0xD8);
        break;
      case (SHIFT_K << 11) | (SHIFT_RD << 11) | 0x0200|247:  /* KП->X9 */
        execute(0xD9);
        break;
      case (SHIFT_K << 11) | (SHIFT_RD << 11) | 0x0100|223:  /* KП->Xa */
        execute(0xDA);
        break;
      case (SHIFT_K << 11) | (SHIFT_RD << 11) | 0x0200|223:  /* KП->Xb */
        execute(0xDB);
        break;
      case (SHIFT_K << 11) | (SHIFT_RD << 11) | 0x0300|223:  /* KП->Xc */
        execute(0xDC);
        break;
      case (SHIFT_K << 11) | (SHIFT_RD << 11) | 0x0400|223:  /* KП->Xd */
        execute(0xDD);
        break;
      case (SHIFT_K << 11) | (SHIFT_RD << 11) | 0x0400|239:  /* KП->Xe */
        execute(0xDE);
        break;
      case (SHIFT_K << 11) | (SHIFT_RD << 11) | 0x0400|251:  /* KП->Xf */
        execute(0xDF);
        break;
        
      case 0x1000|223:  /* X->П0 */
        execute(0x40);
        break;
      case 0x1100|223:  /* X->Пa */
        execute(0x4A);
        break;
      case 0x1200|223:  /* X->Пb */
        execute(0x4B);
        break;
      case 0x1300|223:  /* X->Пc */
        execute(0x4C);
        break;
      case 0x1400|223:  /* X->Пd */
        execute(0x4d);
        break;
      case 0x1400|239:  /* X->Пe */
        execute(0x4e);
        break;
      case 0x1400|251:  /* X->Пf */
        execute(0x4f);
        break;
      case 0x1000|239:  /* X->П1 */
        execute(0x41);
        break;
      case 0x1000|247:  /* X->П7 */
        execute(0x47);
        break;
      case 0x1000|251:  /* X->П4 */
        execute(0x44);
        break;
      case 0x1100|239:  /* X->П2 */
        execute(0x42);
        break;
      case 0x1100|251:  /* X->П5 */
        execute(0x45);
        break;
      case 0x1100|247:  /* X->П8 */
        execute(0x48);
        break;
      case 0x1200|239:  /* X->П3 */
        execute(0x43);
        break;
      case 0x1200|247:  /* X->П9 */
        execute(0x49);
        break;
      case 0x1200|251:  /* X->П5 */
        execute(0x46);
        break;

      case 0x2000|223:  /* П->X0 */
        execute(0x60);
        break;
      case 0x2100|223:  /* П->Xa */
        execute(0x6A);
        break;
      case 0x2200|223:  /* П->Xb */
        execute(0x6B);
        break;
      case 0x2300|223:  /* П->Xc */
        execute(0x6C);
        break;
      case 0x2400|223:  /* П->Xd */
        execute(0x6d);
        break;
      case 0x2400|239:  /* П->Xe */
        execute(0x6e);
        break;
      case 0x2400|251:  /* П->Xf */
        execute(0x6f);
        break;
      case 0x2000|239:  /* П->X1 */
        execute(0x61);
        break;
      case 0x2000|247:  /* П->X7 */
        execute(0x67);
        break;
      case 0x2000|251:  /* П->X4 */
        execute(0x64);
        break;
      case 0x2100|239:  /* П->X2 */
        execute(0x62);
        break;
      case 0x2100|251:  /* П->X5 */
        execute(0x65);
        break;
      case 0x2100|247:  /* П->X8 */
        execute(0x68);
        break;
      case 0x2200|239:  /* П->X3 */
        execute(0x63);
        break;
      case 0x2200|247:  /* П->X9 */
        execute(0x69);
        break;
      case 0x2200|251:  /* П->X5 */
        execute(0x66);
        break;
       
      case 223:
        execute(0);
        break;
      case 239:
        execute(1);
        break;
      case 247:
        execute(7);
        break;
      case 251:
        execute(4);
        break;
      case 0x0100|239:
        execute(2);
        break;
      case 0x0100|251:
        execute(5);
        break;
      case 0x0100|247:
        execute(8);
        break;
      case 0x0200|239:
        execute(3);
        break;
      case 0x0200|247:
        execute(9);
        break;
      case 0x0200|251:
        execute(6);
        break;
      case (SHIFT_F << 11)  | 223: /* F10^X */
        execute(MK61_10POWX);
        break;
      case (SHIFT_F << 11)  | 0x0100 | 239: /* FlgX */
        execute(MK61_LGX);
        break;
      case (SHIFT_F << 11)  | 0x0200 | 239: /* FlnX */
        execute(MK61_LNX);
        break;
      case (SHIFT_F << 11)  | 239: /* Fe^X */
        execute(MK61_EXPX);
        break;
      case (SHIFT_F << 11)  | 0x0400 | 251: /* FX^2 */
        execute(MK61_SQRX);
        break;
      case (SHIFT_F << 11)  | 0x0400 | 247: /* Fsqrt(X) */
        execute(MK61_REVX);
        break;
      case (SHIFT_F << 11)  | 0x0300 | 247: /* Fsqrt(X) */
        execute(MK61_SQRTX);
        break;
        
/* ==== Shift pressed and released ===  */
      case (SHIFT_F << 11)  | 253:
      case (SHIFT_K << 11)  | 253:
      case (SHIFT_WR << 11) | 253:
      case (SHIFT_RD << 11) | 253:
      case (SHIFT_WR << 11) | (SHIFT_K << 11) | 253:
      case (SHIFT_RD << 11) | (SHIFT_K << 11) | 253:
      case 253:
        shift ^= SHIFT_K;
        shift &= ~SHIFT_F;
        Refresh_SHIFT();
        return;
//        break;
        
      case (SHIFT_K << 11) | 254:
      case (SHIFT_F << 11) | 254:
      case (SHIFT_WR << 11) | 254:
      case (SHIFT_RD << 11) | 254:
      case (SHIFT_WR << 11) | (SHIFT_K << 11) | 254:
      case (SHIFT_RD << 11) | (SHIFT_K << 11) | 254:
      case 254:
        shift ^= SHIFT_F;
        shift &= ~SHIFT_K;
        Refresh_SHIFT();
        return;
//        break;
        
      case (SHIFT_K << 11) | (SHIFT_RD<<11) | 0x0200 | 253:  /* prefix X->П */
      case (SHIFT_K << 11) | 0x0200 | 253:  /* prefix X->П */
      case (SHIFT_RD<<11) | 0x0200 | 253:  /* prefix X->П */
      case (SHIFT_WR<<11) | 0x0200 | 253:  /* prefix X->П */
      case 0x0200|253:  /* prefix X->П */
        shift ^= SHIFT_WR;
        shift &= ~SHIFT_RD;
        Refresh_SHIFT();
        return;
//        break;
        
      case (SHIFT_K << 11) | (SHIFT_RD<<11) | 0x0100 | 253:  /* prefix П->X */
      case (SHIFT_K << 11) | (SHIFT_WR<<11) | 0x0100 | 253:  /* prefix П->X */
      case (SHIFT_K << 11) | 0x0100 | 253:  /* prefix П->X */
      case (SHIFT_WR<<11) | 0x0100 | 253:  /* prefix П->X */
      case (SHIFT_RD<<11) | 0x0100 | 253:  /* prefix П->X */
      case 0x0100|253:  /* prefix П->X */
        shift ^= SHIFT_RD;
        shift &= ~SHIFT_WR;
        Refresh_SHIFT();
        return;
//        break;
        
      case 0x0300|239:  /* X <-> Y */
        execute(MK61_XY);
        break;
      case 0x0300|251:  /* + */
        execute(MK61_ADD);
        break;
      case 0x0300|247:  /* - */
        execute(MK61_SUB);
        break;
      case 0x0400|247:  /* : */
        execute(MK61_DIV);
        break;
      case 0x0400|251:  /* * */
        execute(MK61_MUL);
        break;
      case 0x0400|223:  /* CX */
        execute(MK61_CX);
        break;
      case 0x0400|239:  /* B| */
        execute(MK61_BI);
  }

  shift = 0;
  Refresh_SHIFT();
  Serial.print("kbd("); Serial.println(kbd_code,HEX);
}
 
void loop()
{
  register byte OPCODE, i, j;
  KbdScan();

  if(Play){
    OPCODE = code[IP];
    execute(OPCODE);
    if( ++IP == 200 ) IP = 0;
  }
  if(RequestReady){
    RequestReady = false;
    
    if( (MODE == AUTO) && (input[0] >= '0') && (input[0] <= '9') ){
        push();
        X = atof((const char*) &input[0]);
    } else if ( (MODE == PRG) && (input[0] != ':') ) {
        input[2] = 0;
/*        
 *          0        9
 *         0x30,...,0x39 - 00110000,...,00111001 => -0x30 => 0x00,...,0x09 - 00000000,...,00001001
 *          A        F 
 *         0x41,...,0x46 - 01000001,...,01000110 => -0x30 => 0x11,...,0x16 - 00010001,...,00010110
 *          a        f
 *         0x61,...,0x66 - 01100001,...,01100110 => -0x30 => 0x31,...,0x36 - 00110001,...,00110110
 */
        i = ((byte) input[0]) - 0x30;
        if(i > 9) i = (i & (~0x20)) - 7;
        i <<= 4;
        j = ((byte) input[1]) - 0x30;
        if(j > 9) j = (j & (~0x20)) - 7;;
        i += j;
        
        code[TIP++] = i;
        SerialPrintHEX(i); Serial.println();
        SerialPrintHEX(TIP); Serial.print("> ");
    } else {
      switch( input[0] ){
          case '+':
            execute(0x10);
            break;
          case '-':  
            execute(0x11);
            break;
          case '*':
            execute(0x12);
            break;
          case '/':
            execute(0x13);
            break;
          case '!':
            execute(0x10 + input[1]);
            break;
          case '>':  // Шаг по программе ПП
            OPCODE = code[IP];
            execute(OPCODE);
            if( ++IP == 200 ) IP = 0;
            break;
          case 'g':
            if( input[1] == '0' )
              IP = 0;
            else
              Play = true;
            return;
          case ':':
            if(input[1] != 0){
              TIP = (byte) atoi((const char*) &input[1]);
              SerialPrintHEX(TIP); Serial.print("> ");
              MODE = PRG;
            }
            else{
              Serial.println(">>");
              MODE = AUTO;
            }
            break;
          case 'd':
            for(i=0; i<8; i++){
              SerialPrintHEX(i<<3);
              for(j=0; j<8; j++){
                Serial.print(' '); SerialPrintHEX(code[j+(i<<3)]); 
              }
              Serial.println();
            }
            break;
          case 'l':
            for(i=0; i<8; i++){
              Serial.print('M'); Serial.print(i); Serial.print(" = "); dtostrf(data[i], 11, 2, &ibuff[0]); Serial.print(ibuff);
              Serial.print("  M"); Serial.print(i+8); Serial.print(" = "); dtostrf(data[i+8], 11, 2, &ibuff[0]); Serial.println(ibuff);
            }
            Serial.print("X  = "); dtostrf(X, 11, 2, &ibuff[0]); Serial.println(ibuff);
            Serial.print("Y  = "); dtostrf(Y, 11, 2, &ibuff[0]); Serial.println(ibuff);
            Serial.print("Z  = "); dtostrf(Z, 11, 2, &ibuff[0]); Serial.println(ibuff);
            Serial.print("T  = "); dtostrf(T, 11, 2, &ibuff[0]); Serial.println(ibuff);
            Serial.print("IP = "); Serial.println(IP);
      }
    }

    Displayed();
  }
}


void Refresh_SHIFT(void){
 lcd.setCursor(14,1);          // Установить курсор на вторую строку
 if((shift & SHIFT_F))
   lcd.write('F');
 else if((shift & SHIFT_K))
   lcd.write('K');
 else
   lcd.write(' ');
   
 lcd.setCursor(15,1);          // Установить курсор на вторую строку
 if((shift & SHIFT_WR))
   lcd.write('W');
 else if((shift & SHIFT_RD))
   lcd.write('R');
 else
   lcd.write(' ');
}

void Displayed(void){
 lcd.setCursor(0,1);           // Установить курсор на вторую строку
 lcd.write((uint8_t)0);
 dtostrf(X, 11, 2, &ibuff[0]);
 lcd.print(ibuff);             // Вывести X

 Refresh_SHIFT();

 if((SignalsRefresh & SIG_LCD_LINE0_REFRESH)){
    lcd.setCursor(0,0);           // Установить курсор на первую строку
    lcd.write((uint8_t)1);
    dtostrf(Y, 11, 2, &ibuff[0]);
    lcd.print(ibuff);             // Вывести Y
    if(MODE) 
      lcd.print(" PRG");          // Вывести режим
    else
      lcd.print(" ABT");          // Вывести режим
   }
 }

void Refresh_Y(void){
 lcd.setCursor(0,0);           // Установить курсор на вторую строку
 lcd.write((uint8_t)1);
 dtostrf(Y, 11, 2, &ibuff[0]);
 lcd.print(ibuff);             // Вывести Y
}

void RefreshXBuff(void){
 lcd.setCursor(1,1);                             // Установить курсор на вторую строку
 lcd.print("           ");                       // очистка содержимого строки
 lcd.setCursor(1,1);                             // Установить курсор на вторую строку
 lcd.print((const char*) &xbuff[0]);             // Вывести X из буфера ввода
}

/* ===================================================================================
 Читаем нулевой порт, при поступлении данных с общей шины с идентификатора BLV до \n
===================================================================================  */
void serialEvent() {
  register char inChar;
  if(Serial.available()) { // В UART что то свалилось - обработаем один символ
    inChar = Serial.read();
    if(inChar == '\r') { // достигнут конец строки
       *ihead++ = 0;
       ihead = &input[0];
       RequestReady = true;
       return;
    }
    *ihead++ = inChar;
  }
}
