/*
 * proj2_space_invaders.c
 *
 * Created: 2025-01-12 오전 1:35:32
 * Author : kmj
 */ 


#define F_CPU	16000000
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/pgmspace.h>

#define BAUD	38400
#define TIMER_DIVISOR 8
#define TIMER_TICKS		((F_CPU/TIMER_DIVISOR)/BAUD)

//encoder
#define INT_A_DDR	DDRA
#define	INT_A_PORT	PORTA
#define INT_A_BIT	1		//A: PCINT9
#define INT_B_DDR	DDRD
#define	INT_B_PORT	PORTD
#define INT_B_BIT	4		//B
static uint8_t a0;

//PS/2 Keyboard
static volatile int16_t	scanCode=0;
#define KBRD_CLK_DDR	DDRD
#define KBRD_CLK_PORT	PORTD
#define KBRD_CLK_BIT	2		//INT0
#define KBRD_DATA_DDR	DDRD
#define KBRD_DATA_PORT	PORTD
#define KBRD_DATA_BIT	3

//tx
static volatile uint16_t txBits=0;

#define TXD_DDR		DDRD
#define TXD_PORT	PORTD
#define TXD_BIT		1

//space invader
static uint8_t life=3;
static uint8_t enemy[8]={0,};//page0~7까지 하강
static int16_t shipLoc=60; //우주선 위치 - 엔코더에서 바꿈.


//7-segment
#define MAX_CS_DDR	DDRB
#define MAX_CS_PORT	PORTB
#define MAX_CS_BIT	5

//monocolor display
#define DC_DDR		DDRB
#define DC_PORT		PORTB
#define DC_BIT		0
#define OLD_CS_DDR	DDRD
#define OLD_CS_PORT	PORTD
#define OLD_CS_BIT	6


//tx
ISR(TIMER0_COMPA_vect){
	TXD_PORT&=~(1<<TXD_BIT);
	TXD_PORT|=((txBits&0x01)<<TXD_BIT);
	txBits=txBits>>1;
	if(txBits==0){
		TCCR0B&=~(0x07);//------000
	}
}

void uart_tx(uint8_t c){
	////
	while(1){
		if(txBits==0){
			break;
		}
	}
	txBits = (1 << 10) | (c << 1) | (0 << 0); // Stop Bit, Data Bits, Start Bit
	TCNT0 = 0; // 타이머 초기화
	TCCR0B = (1 << CS01); //-----010  --divisor 는 8.
}

void uart_tx_str(char * str){
	while(*str){
		uart_tx(*str++);
	}
}


//encoder_function
ISR(PCINT1_vect){
	uint8_t a=0x01&(PINA>>INT_A_BIT);
	uint8_t b=0x01&(PIND>>INT_B_BIT);
	if(1){
		if(a!=b){
			shipLoc+=1;
		}
		else{
			shipLoc-=1;
		}
		
		if(shipLoc>128){
			shipLoc=-10;
		}
		else if(shipLoc<-10){
			shipLoc=128;
		}
		a0=a;
	}
}

//PS/2 Keyboard
ISR(INT0_vect){
	static uint16_t receivedBits=0, bitCount=0;
	receivedBits|=(1&(PIND>>KBRD_DATA_BIT))<<bitCount;
	bitCount+=1;
	if(bitCount==11){
		scanCode=receivedBits;
		receivedBits=0;
		bitCount=0;
	}
}
static uint8_t ps2_scan_to_ascii(uint16_t code) {
	uint8_t s, c;
	static uint8_t sBreak=0, sModifier=0, sShift=0;
	static const char keymap_unshifted[] PROGMEM =
	"             \011`      q1   zsaw2  cxde43   vftr5  nbhgy6   mju78  ,kio09"
	"  ./l;p-   \' [=    \015] \\        \010  1 47   0.2568\033  +3-*9      ";
	static const char keymap_shifted[] PROGMEM =
	"             \011~      Q!   ZSAW@  CXDE$#   VFTR%  NBHGY^   MJU&*  <KIO)("
	"  >?L:P_   \" {+    \015} |        \010  1 47   0.2568\033  +3-*9       ";

	s = (code>>1) & 0xff;		// Remove start, parity, stop bits

	if (s==0xaa)
	return 0;				// Ignore BAT completion code
	if (s==0xf0) {
		sBreak=1;
		return 0;
	}
	if (s==0xe0) {
		sModifier=1;
		return 0;
	}
	if (sBreak) {						// if key released
		if ((s==0x12) || (s==0x59)) {	// Left or Right Shift Key
			sShift=0;
		}
		sBreak=0; sModifier=0;
		return 0;
	}
	if ((s==0x12) || (s==0x59))			// Left or Right Shift Key
	sShift=1;
	if (sModifier)						// If modifier ON, return 0
	return 0;
	if (sShift==0)
	c=pgm_read_byte(keymap_unshifted+s);
	else
	c=pgm_read_byte(keymap_shifted+s);
	if ((c==32) && (s!=0x29))			// Ignore unless real space key
	return 0;
	return c;
}



//7-segment function
static void SpiUSITx(uint8_t data){
	USIDR=data;
	uint8_t i;
	for(i=0;i<8;i++){
		USICR|=(1<<USIWM0);
		USICR|=(1<<USITC);
		
		USICR|=(1<<USIWM0);
		USICR|=(1<<USITC);
		USICR|=(1<<USICLK);
	}
}

static void max7219_init(void){
	const uint8_t initData[]={
		0x09, 0xff, 0x0a, 0x01,
	0x0b, 0x01, 0x0c, 0x01, 0x0f, 0x00};
	SCK_DDR |= (1<<SCK_BIT);
	DO_DDR |= (1<<DO_BIT);
	MAX_CS_DDR |= (1<<MAX_CS_BIT);
	MAX_CS_PORT |= (1<<MAX_CS_BIT);
	
	for(unsigned i=0; i<sizeof(initData);i+=2){
		MAX_CS_PORT &= ~(1<<MAX_CS_BIT);
		SpiUSITx(initData[i]);
		SpiUSITx(initData[i+1]);
		MAX_CS_PORT |= (1<<MAX_CS_BIT);
	}
}

//monocolor display function
void sh1106_init(void){
	const uint8_t init_commands[]={
		0xae, 0x00, 0x10, 0x40, 0x81, 0x80, 0xc0, 0xa8,
		0x3f, 0xd3, 0x00, 0xd5, 0x50, 0xd9, 0x22, 0xda,
	0x12, 0xdb, 0x35, 0xa4, 0xa6, 0xaf};
	DC_DDR		|= (1<<DC_BIT);
	OLD_CS_DDR	|= (1<<OLD_CS_BIT);
	
	OLD_CS_PORT |= (1<<OLD_CS_BIT);
	
	OLD_CS_PORT &= ~(1<<OLD_CS_BIT);
	DC_PORT |= (1<<DC_BIT);
	DC_PORT &= ~(1<<DC_BIT);
	
	for(uint8_t i=0; i<sizeof(init_commands);i++){
		SpiUSITx(init_commands[i]);
	}
	OLD_CS_PORT |= (1<<OLD_CS_BIT);
}

void sh1106_set_location(uint8_t page, uint8_t column){
	DC_PORT |= (1<<DC_BIT);
	DC_PORT &= ~(1<<DC_BIT);
	
	SpiUSITx(0xB0|page);
	SpiUSITx(0x00|(column&0x0F));
	SpiUSITx(0x10|((column>>4)&0x0F));
}

void sh1106_clear(void){
	OLD_CS_PORT |= (1<<OLD_CS_BIT);
	OLD_CS_PORT &= ~(1<<OLD_CS_BIT);
	
	for(uint8_t page=0;page<8;page++){
		sh1106_set_location(page, 0);
		DC_PORT |= (1<<DC_BIT);
		
		for(uint8_t count=0; count<132;count++){
			SpiUSITx(0);
		}
	}
	OLD_CS_PORT |= (1<<OLD_CS_BIT);
}


//space invader----------------------------------------
void SI_init(void){
	life=3;
	shipLoc=60; //우주선 위치 - 엔코더에서 바꿈. 
}

uint8_t enemyShape[16]={
	0b11111100, 
	0b00100000, 
	0b00110000, 
	0b00011000,
	0b01111101, 
	0b10110110, 
	0b10111100, 
	0b00111100,
	0b00111100, 
	0b10111100, 
	0b10110110, 
	0b01111101,
	0b00011000, 
	0b0011000, 
	0b00100000, 
	0b11111100};
	
void SI_drEnemy(void){//시작위치 입력받아서 적 그림 (이때 적 없는 곳엔 00000000보내지 않고 그냥 거넌뛰기로...... 총알때매
	OLD_CS_PORT |= (1<<OLD_CS_BIT);
	OLD_CS_PORT &= ~(1<<OLD_CS_BIT);
	uint8_t i,j;
	for(i=0;i<8;i++){
		sh1106_set_location(enemy[i]-1,i*16);
		if(enemy[i]!=0){
			for(j=0;j<16;j++){
				DC_PORT |= (1<<DC_BIT);
				SpiUSITx(enemyShape[j]);
			}
			if(enemy[i]==7){
				if(i*16-10<shipLoc && shipLoc<i*16+16){
					life-=1;
				}
				enemy[i]=0;
			}
			else{
				enemy[i]+=1;
			}
		}
		else{
			for(j=0;j<16;j++){
				DC_PORT |= (1<<DC_BIT);
				SpiUSITx(0x00);
			}
		}
	}
	OLD_CS_PORT |= (1<<OLD_CS_BIT);
}



void SI_drBorderNship(void){
	OLD_CS_PORT |= (1<<OLD_CS_BIT);
	OLD_CS_PORT &= ~(1<<OLD_CS_BIT);
	
	sh1106_set_location(7, 0);
	DC_PORT |= (1<<DC_BIT);
	for(uint8_t column =0; column<132; column++){
		SpiUSITx(0x80);
	}
	
	sh1106_set_location(7, shipLoc);
	DC_PORT |= (1<<DC_BIT);
	for(uint8_t column =shipLoc; column<shipLoc+10; column++){
		SpiUSITx(0x8C);
	}
	sh1106_set_location(7, shipLoc+3);
	DC_PORT |= (1<<DC_BIT);
	for(uint8_t column =shipLoc+3; column<shipLoc+7; column++){
		SpiUSITx(0x8F);
	}
	
	OLD_CS_PORT |= (1<<OLD_CS_BIT);
}



int main(void)
{
	////
	TXD_DDR|=(1<<TXD_BIT);
	TXD_PORT|=(1<<TXD_BIT);
	
	//Timer0
	TCCR0A = (1 << WGM01); // CTC.. ------10
	TCCR0B &= ~(0x07);//counter stopped
	OCR0A = TIMER_TICKS - 1; //어느 값까지 셀지...
	TIMSK |= (1 << OCIE0A); // Timer0 Compare Match 인터럽트 활성화
	
	//encoder
	INT_A_DDR &= ~(1<<INT_A_BIT);//input--0
	INT_A_PORT |= (1<<INT_A_BIT);//pullup
	INT_B_DDR &= ~(1<<INT_B_BIT);//input--0
	INT_B_PORT |= (1<<INT_B_BIT);//pullup
	PCMSK1 |= (1<<PCINT9);
	GIMSK |= (1<<PCIE1);
	
	//keyboard
	KBRD_CLK_DDR &= ~(1<<KBRD_CLK_BIT);//input--0
	KBRD_CLK_PORT&= ~(1<<KBRD_CLK_BIT);
	KBRD_CLK_PORT |= (1<<KBRD_CLK_BIT);//pullup
	KBRD_DATA_DDR &= ~(1<<KBRD_DATA_BIT);//input--0
	KBRD_DATA_PORT&= ~(1<<KBRD_DATA_BIT);
	KBRD_DATA_PORT |= (1<<KBRD_DATA_BIT);//pullup
	MCUCR &= ~(3<<0);
	MCUCR |= (2<<0);
	GIMSK &= ~(1<<INT0);
	GIMSK |= (1<<INT0);
	
	sei();
	////
	
	//7-segment
	max7219_init();
	_delay_ms(10);
	//monocolor display
	SCK_DDR |= (1<<SCK_BIT);
	DO_DDR |= (1<<DO_BIT);
	_delay_ms(10);
	sh1106_init();
	sh1106_clear();
	_delay_ms(2000);
	
	
	uart_tx_str("\n\n\n\nSpace Invaders\n");
	while(1){
			
		//space invader		
		uint8_t c=ps2_scan_to_ascii(scanCode);
		uart_tx(c);
		c=c-'0'-1;
		if(0<=c && c<=7){
			enemy[c]=1; //키보드 1~8번 키에서 입력 받으면 해당 레인에서 외계인 생성.
		}
		scanCode=0;
		
		
		
		//displaying========================
		//monocolor display
		OLD_CS_PORT |= (1<<OLD_CS_BIT);
		OLD_CS_PORT &= ~(1<<OLD_CS_BIT);
		DC_PORT |= (1<<DC_BIT);
		DC_PORT &= ~(1<<DC_BIT);
		SI_drEnemy();
		SI_drBorderNship();
		OLD_CS_PORT |= (1<<OLD_CS_BIT);
		_delay_ms(10);

		//7-segment    : score
		MAX_CS_PORT |= (1<<MAX_CS_BIT);
		MAX_CS_PORT &= ~(1<<MAX_CS_BIT);
		SpiUSITx(1);
		SpiUSITx(life);
		MAX_CS_PORT |= (1<<MAX_CS_BIT);
		
		_delay_ms(300);
		//clear====
		//monocolor display
		sh1106_clear();
		
		_delay_ms(2);
		
		
		
		//space invader
		if(life==0){
			uart_tx_str("END\n");
			return 0;
		}
	}
}


