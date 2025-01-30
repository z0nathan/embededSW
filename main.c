/*
 * proj1_gameOFlife.c
 *
 * Created: 2025-01-03 오전 11:48:15
 * Author : kmj
 */ 

#define F_CPU 8000000

#include <avr/io.h>
#include <util/delay.h>
#include <string.h> // memcpy 사용


//SWITCH (pause switch)
#define SWITCH_DDR	DDRA
#define SWITCH_PORT	PORTA
#define SWITCH_BIT	0

//LCD
#define	LCD_EN_DDR	DDRD
#define LCD_EN_PORT	PORTD
#define LCD_EN_BIT	4

#define LCD_RS_DDR	DDRD
#define	LCD_RS_PORT	PORTD
#define LCD_RS_BIT	6

#define LCD_DB_DDR	DDRB
#define LCD_DB_PORT	PORTB
#define LCD_DB_BIT	0

#define LED_DDR		DDRD
#define LED_PORT	PORTD
#define LED_PIN		PIND
#define LED_BIT		5

const uint8_t custom5x8[] = {				//LCD_CGRAM
	0b00001,0b00001,0b00001,0b00001,0b00001,0b00001,0b00001,0b00001, // Font for CGRAM Address 0x00 (ASCII Code 0)
	0b00000,0b00000,0b00000,0b00100,0b00000,0b00000,0b00000,0b00000, // Font for CGRAM Address 0x08 (ASCII Code 1)
	0b00000,0b11111,0b11111,0b11111,0b11111,0b11111,0b11111,0b00000, // Font for CGRAM Address 0x10 (ASCII Code 2)
	0b00111,0b00111,0b00111,0b00111,0b00111,0b00111,0b00111,0b00111, // Font for CGRAM Address 0x18 (ASCII Code 3)
	0b01110,0b01110,0b01110,0b01110,0b01110,0b01110,0b01110,0b01110, // Font for CGRAM Address 0x20 (ASCII Code 4)
	0b11100,0b11100,0b11100,0b11100,0b11100,0b11100,0b11100,0b11100, // Font for CGRAM Address 0x28 (ASCII Code 5)
	0b11000,0b11000,0b11000,0b11000,0b11000,0b11000,0b11000,0b11000, // Font for CGRAM Address 0x30 (ASCII Code 6)
	0b10000,0b10000,0b10000,0b10000,0b10000,0b10000,0b10000,0b10000, // Font for CGRAM Address 0x38 (ASCII Code 7)
};


//7-segment
#define MAX_CS_DDR	DDRB
#define MAX_CS_PORT	PORTB
#define MAX_CS_BIT	5

//monocolor display
#define DC_DDR		DDRD
#define DC_PORT		PORTD
#define DC_BIT		2

#define OLD_CS_DDR	DDRD
#define OLD_CS_PORT	PORTD
#define OLD_CS_BIT	3


//game of life
#define WIDTH 16
#define HEIGHT 8
#define ROW_BYTES (WIDTH / 8) // 한 행의 바이트 수
#define GRID_SIZE (HEIGHT * ROW_BYTES) // 전체 바이트 수

uint8_t grid[GRID_SIZE];      // 현재 세대
uint8_t next_grid[GRID_SIZE]; // 다음 세대
uint16_t total_cell=0;




//LCD function 
static void lcd_write_nibble(uint8_t rs, uint8_t data){
	LCD_RS_PORT &= ~(1<<LCD_RS_BIT);
	LCD_RS_PORT |= (rs<<LCD_RS_BIT);
	LCD_EN_PORT |= (1<<LCD_EN_BIT);
	LCD_DB_PORT = (data<<LCD_DB_BIT);
	LCD_EN_PORT &= ~(1<<LCD_EN_BIT);
}

static inline void lcd_wait(){
	_delay_us(60);
}

static void lcd_write_byte(uint8_t rs, uint8_t data){
	lcd_write_nibble(rs, (data>>4)&0x0F);
	lcd_write_nibble(rs, (data&0x0F));
}

static void lcd_init(){
	LCD_RS_DDR |= (1<<LCD_EN_BIT);
	LCD_EN_DDR |= (1<<LCD_RS_BIT);
	LCD_DB_DDR |= (0x0F<<LCD_DB_BIT);
	_delay_ms(20);
	
	lcd_write_nibble(0, 3);//0011	8비트모드
	lcd_wait();
	lcd_write_nibble(0, 2);//0010	4비트모드
	lcd_wait();	
	lcd_write_byte(0, 0x28);//001 010 00 //DL=0 N=1 F=0
	lcd_wait();
	lcd_write_byte(0, 0x0C);//0000 1 100 //D=1 C=0 B=0
	lcd_wait();
}

static void lcd_puts(char * str){
	uint8_t i;

	lcd_write_byte(0, 0x80);
	lcd_wait();
	for(i=0; i<16; i++){
		if(str[i]=='\0'){
			return;
		}
		lcd_write_byte(1, str[i]);
		lcd_wait();
	}

	lcd_write_byte(0, 0x80+0x40);
	lcd_wait();
	for(i=16; i<32; i++){
		if(str[i]=='\0'){
			return;
		}
		lcd_write_byte(1, str[i]);
		lcd_wait();
	}	
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
	////
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
		////
		sh1106_set_location(page, 0);
		DC_PORT |= (1<<DC_BIT);
		
		for(uint8_t count=0; count<132;count++){
			SpiUSITx(0);
		}
	}
	OLD_CS_PORT |= (1<<OLD_CS_BIT);
}


//game of life function
uint8_t getCell(uint8_t x, uint8_t y) {		// 특정 셀의 상태 가져오기
	if (x >= WIDTH || y >= HEIGHT) return 0;
	uint16_t index = (y * ROW_BYTES) + (x / 8);
	uint8_t bit = 1 << (7 - (x % 8));
	return (grid[index] & bit) != 0;
}

void setCell(uint8_t x, uint8_t y, uint8_t state) {		// 특정 셀의 상태 설정
	if (x >= WIDTH || y >= HEIGHT) return;
	uint16_t index = (y * ROW_BYTES) + (x / 8);
	uint8_t bit = 1 << (7 - (x % 8));
	if (state) {
		grid[index] |= bit;  // 셀 활성화
		} else {
		grid[index] &= ~bit; // 셀 비활성화
	}
}

void setCell_(uint8_t x, uint8_t y, uint8_t state) {		// 특정 셀의 상태 설정
	if (x >= WIDTH || y >= HEIGHT) return;
	uint16_t index = (y * ROW_BYTES) + (x / 8);
	uint8_t bit = 1 << (7 - (x % 8));
	if (state) {
		next_grid[index] |= bit;  // 셀 활성화
		} else {
		next_grid[index] &= ~bit; // 셀 비활성화
	}
}

uint8_t countAliveNeighbors(uint8_t x, uint8_t y) {		// 이웃한 살아있는 셀 수 계산
	uint8_t count = 0;
	for (int8_t dy = -1; dy <= 1; dy++) {
		for (int8_t dx = -1; dx <= 1; dx++) {
			if (dx == 0 && dy == 0) continue; // 자기 자신 제외
			
			count += getCell((x + dx+16)%16, (y + dy+8)%8);//화면 연속 처리
		}
	}
	return count;
}

void nextGeneration() {				// 다음 세대 계산
	memset(next_grid, 0, GRID_SIZE); // 다음 세대를 초기화
	total_cell=0;
	for (uint8_t y = 0; y < HEIGHT; y++) {
		for (uint8_t x = 0; x < WIDTH; x++) {
			uint8_t aliveNeighbors = countAliveNeighbors(x, y);
			uint8_t isAlive = getCell(x, y);
			if (isAlive && (aliveNeighbors == 2 || aliveNeighbors == 3)) {
				setCell_(x, y, 1); // 살아있는 상태 유지
				total_cell+=1;
			} 
			else if (!isAlive && aliveNeighbors == 3) {
				setCell_(x, y, 1); // 죽은 셀 부활
				total_cell+=1;
			}
		}
	}
	memcpy(grid, next_grid, GRID_SIZE); // 다음 세대를 현재 세대로 복사
}

// 초기 패턴 설정
void initializeGrid() {
	memset(grid, 0, GRID_SIZE); // 모든 셀을 죽은 상태로 초기화
	//패턴: 글라이더
	setCell(1, 0, 1);
	setCell(2, 1, 1);
	setCell(0, 2, 1);
	setCell(1, 2, 1);
	setCell(2, 2, 1);
	
	
	setCell(5, 5, 1);
	setCell(6, 5, 1);
	setCell(7, 5, 1);
	setCell(6, 4, 1);
	setCell(6, 6, 1);
}


int main(void)
{	
	uint8_t i=0;
	
	//SWITCH=============================s
	SWITCH_DDR &= ~(1<<SWITCH_BIT);//input--0으로 레지스터 설정..
	SWITCH_PORT&= ~(1<<SWITCH_BIT);
	SWITCH_PORT |= (1<<SWITCH_BIT);//pullup.
	
	//LCD=================================
	LED_DDR |= 1<<LED_BIT;
	lcd_init();
	//CGRAM reset
	lcd_wait();
	lcd_write_byte(0, 0x40);//01 000000
	lcd_wait();
	for(i=0;i<sizeof(custom5x8);i++){
		lcd_write_byte(1, custom5x8[i]);
		lcd_wait();
	}
	lcd_puts("Hello!");
	//7-segment====================================
	max7219_init();
	_delay_ms(10);
	//monocolor display=============================
	SCK_DDR |= (1<<SCK_BIT);
	DO_DDR |= (1<<DO_BIT);
	_delay_ms(10);
	sh1106_init();
	sh1106_clear();
	
	
	//game of life
	initializeGrid();
	uint8_t cellSCR=0;
	
	//----------------------------------------------------
	_delay_ms(1000);
	uint8_t cycle=0;
	uint8_t alive=0xFF;
	uint8_t s_state=0;
	uint8_t s_state_prev=0;
	while(1){
		//check SWITCH=======================================
		s_state=PINA;
		if((!s_state)&&s_state!=s_state_prev){ //정지 스위치 토글.
			alive=~(alive);
		}
		s_state_prev=s_state;
		//game of life================================
		if (alive){
			nextGeneration();
			cycle++;
		}
		
		//displaying========================
		//LCD
		char message[]="Game of life";
		char lcd[32]={1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1};
		for(i=0;i<sizeof(message)-1;i++){
			lcd[(i+cycle)%32]=message[i];
		}
		if (cycle>=32)	cycle=0;
		lcd_puts(lcd);
		//7-segment    : total_cell
		for(unsigned digit=1;digit<4;digit++){
			MAX_CS_PORT |= (1<<MAX_CS_BIT);
			MAX_CS_PORT &= ~(1<<MAX_CS_BIT);
			SpiUSITx(digit);
			SpiUSITx(total_cell%10);
			MAX_CS_PORT |= (1<<MAX_CS_BIT);
			total_cell=total_cell/10;
		}
		_delay_ms(10);
		
		//monocolor display
		OLD_CS_PORT |= (1<<OLD_CS_BIT);
		OLD_CS_PORT &= ~(1<<OLD_CS_BIT);
		DC_PORT |= (1<<DC_BIT);
		DC_PORT &= ~(1<<DC_BIT);
		////
		for(uint8_t page=0;page<8;page++){
			for(uint8_t x=0;x<16;x++){
				sh1106_set_location(page, x*8+2);
				DC_PORT |= (1<<DC_BIT);
				cellSCR=0xFF*getCell(x,page);

				for(uint8_t count =0; count<8; count++){
					SpiUSITx(cellSCR);
				}
			}
		}
		
		////
		OLD_CS_PORT |= (1<<OLD_CS_BIT);
		
		
		_delay_ms(300);
		//clear==============================
		//LCD
		lcd_write_byte(0, 0x01);//display clear
		//monocolor display
		sh1106_clear();
		
		_delay_ms(2);	
	}
}
