/*
 * MPU_6050_OLED_Integrated.c
 *
 * - MPU6050 (I2C): INT0 인터럽트로 데이터 수확
 * - SSD1306 (SPI): PROGMEM 폰트 기반 문자열 출력 (Polling)
 * - Power Control: D5 버튼으로 OLED ON/OFF 토글 및 D4 LED 상태 표시
 */ 

#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <avr/pgmspace.h> // 플래시 메모리 사용 헤더

// ==========================================
// 1. 하드웨어 핀 및 레지스터 매핑
// ==========================================
#define MPU6050_ADDR_W 0xD0 
#define MPU6050_ADDR_R 0xD1  
#define MPU6050_SMPRT_DIV 0x19
#define MPU6050_CONFIG 0x1A
#define MPU6050_INT_ENABLE 0x38
#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_ACCEL_XH 0x3B

#define OLED_PORT PORTB
#define OLED_DDR  DDRB
#define OLED_CS   PORTB2  // SS
#define OLED_DC   PORTB1  // D/C#
#define OLED_RES  PORTB0  // RES#

// 전역 변수
volatile uint8_t data_ready_flag = 0;
uint8_t power_state = 1; // 1: ON, 0: OFF
int16_t ax, ay, az, gx, gy, gz;

// ==========================================
// 2. 폰트 배열 (Flash Memory 저장)
// ==========================================
// 필요한 문자(' ', '-', '0'~'9', ':', 'A', 'X', 'Y', 'Z')만 포함
const uint8_t Font5x7[] PROGMEM = {
    0x00, 0x00, 0x00, 0x00, 0x00, // Space (0)
    0x08, 0x08, 0x08, 0x08, 0x08, // - (1)
    0x3E, 0x51, 0x49, 0x45, 0x3E, // 0 (2)
    0x00, 0x42, 0x7F, 0x40, 0x00, // 1
    0x42, 0x61, 0x51, 0x49, 0x46, // 2
    0x21, 0x41, 0x45, 0x4B, 0x31, // 3
    0x18, 0x14, 0x12, 0x7F, 0x10, // 4
    0x27, 0x45, 0x45, 0x45, 0x39, // 5
    0x3C, 0x4A, 0x49, 0x49, 0x30, // 6
    0x01, 0x71, 0x09, 0x05, 0x03, // 7
    0x36, 0x49, 0x49, 0x49, 0x36, // 8
    0x06, 0x49, 0x49, 0x29, 0x1E, // 9 (11)
    0x00, 0x36, 0x36, 0x00, 0x00, // : (12)
    0x7E, 0x11, 0x11, 0x11, 0x7E, // A (13)
    0x63, 0x14, 0x08, 0x14, 0x63, // X (14)
    0x03, 0x04, 0x78, 0x04, 0x03, // Y (15)
    0x61, 0x51, 0x49, 0x45, 0x43  // Z (16)
};

uint8_t Get_Font_Index(char c) {
    if (c == ' ') return 0;
    if (c == '-') return 1;
    if (c >= '0' && c <= '9') return (c - '0' + 2);
    if (c == ':') return 12;
    if (c == 'A') return 13;
    if (c == 'X') return 14;
    if (c == 'Y') return 15;
    if (c == 'Z') return 16;
    return 0;
}

// ==========================================
// 3. I2C (MPU6050) 함수
// ==========================================
void TWI_Init(void){ TWSR = 0x00; TWBR = 72; TWCR = (1 << TWEN); }
void TWI_Start(void){ TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN); while(!((TWCR>>TWINT)&0x01)); }
void TWI_Stop(void){ TWCR = (1<<TWINT)|(1<<TWSTO)|(1<<TWEN); }
uint8_t TWI_Write(uint8_t data){ TWDR = data; TWCR = (1<<TWINT)|(1<<TWEN); while(!((TWCR>>TWINT)&0x01)); return (TWSR & 0xF8); }
uint8_t TWI_ReadACK(void){ TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWEA); while(!((TWCR>>TWINT)&0x01)); return TWDR; }
uint8_t TWI_ReadNACK(void){ TWCR = (1<<TWINT)|(1<<TWEN); while(!((TWCR>>TWINT)&0x01)); return TWDR; }

void MPU6050_Write(uint8_t reg, uint8_t data){
    TWI_Start(); TWI_Write(MPU6050_ADDR_W); TWI_Write(reg); TWI_Write(data); TWI_Stop();
}

void MPU6050_Init(void){
    MPU6050_Write(MPU6050_PWR_MGMT_1, 0x01);
    MPU6050_Write(MPU6050_CONFIG, 0x03);
    MPU6050_Write(MPU6050_SMPRT_DIV, 0x04);
    MPU6050_Write(MPU6050_INT_ENABLE, 0x01);
}

void MPU6050_ReadBurst(int16_t* ax, int16_t* ay, int16_t* az){
    uint8_t buffer[6];
    TWI_Start(); TWI_Write(MPU6050_ADDR_W); TWI_Write(MPU6050_ACCEL_XH); 
    TWI_Start(); TWI_Write(MPU6050_ADDR_R); 
    for (uint8_t i = 0; i < 5; i++){ buffer[i] = TWI_ReadACK(); }
    buffer[5] = TWI_ReadNACK();
    TWI_Stop(); 
    *ax = (buffer[0] << 8) | buffer[1];
    *ay = (buffer[2] << 8) | buffer[3];
    *az = (buffer[4] << 8) | buffer[5];
}

// ==========================================
// 4. SPI 및 OLED 제어 함수
// ==========================================
void SPI_Init(void){
    OLED_DDR |= (1<<OLED_CS) | (1<<PORTB3) | (1<<PORTB5) | (1<<OLED_DC) | (1<<OLED_RES);
    OLED_DDR &= ~(1<<PORTB4); 
    OLED_PORT |= (1<<OLED_CS) | (1<<OLED_RES);
    // 폴링 모드이므로 SPIE(인터럽트) 비활성화
    SPCR = (1<<SPE) | (1<<MSTR);
}

void SPI_Transmit_Polling(uint8_t data) {
    SPDR = data;
    while(!(SPSR & (1<<SPIF))); 
}

void OLED_Write_Command(uint8_t cmd) {
    OLED_PORT &= ~(1<<OLED_DC); // D/C# Low
    OLED_PORT &= ~(1<<OLED_CS); // CS Low
    SPI_Transmit_Polling(cmd);
    OLED_PORT |= (1<<OLED_CS);  // CS High
}

// 화면 전체를 0x00으로 지우는 함수
void OLED_Clear(void) {
    for (uint8_t page = 0; page < 8; page++) {
        OLED_Write_Command(0xB0 + page);
        OLED_Write_Command(0x00);
        OLED_Write_Command(0x10);
        for (uint8_t col = 0; col < 128; col++) {
            OLED_PORT |= (1<<OLED_DC);  // Data 모드
            OLED_PORT &= ~(1<<OLED_CS);
            SPI_Transmit_Polling(0x00); 
            OLED_PORT |= (1<<OLED_CS);
        }
    }
}

void OLED_Init(void) {
    OLED_PORT &= ~(1<<OLED_RES); _delay_ms(10); OLED_PORT |= (1<<OLED_RES); _delay_ms(10);
    OLED_Write_Command(0x20); OLED_Write_Command(0x00); // 수평 어드레싱
    OLED_Write_Command(0x8D); OLED_Write_Command(0x14); // 차지 펌프 ON
    _delay_ms(100); // 전압 안정화 대기
    OLED_Write_Command(0xAF); // 디스플레이 ON
    _delay_ms(50);
    OLED_Clear(); // 쓰레기값 청소
}

void OLED_PowerOff(void) {
    OLED_Write_Command(0xAE); // Display OFF
    OLED_Write_Command(0x8D); OLED_Write_Command(0x10); // 차지 펌프 OFF
    _delay_ms(100); // 방전 대기
}

// ==========================================
// 5. 폰트 출력 함수
// ==========================================
void OLED_PrintChar(char c) {
    uint8_t idx = Get_Font_Index(c);
    OLED_PORT |= (1 << OLED_DC); // Data 모드
    for (uint8_t i = 0; i < 5; i++) {
        uint8_t pixels = pgm_read_byte(&Font5x7[(idx * 5) + i]);
        OLED_PORT &= ~(1 << OLED_CS);
        SPI_Transmit_Polling(pixels);
        OLED_PORT |= (1 << OLED_CS);
    }
    // 자간 비우기 (1픽셀)
    OLED_PORT &= ~(1 << OLED_CS);
    SPI_Transmit_Polling(0x00);
    OLED_PORT |= (1 << OLED_CS);
}

void OLED_PrintString(char* str) {
    while (*str) {
        OLED_PrintChar(*str++);
    }
}

// ==========================================
// 6. 인터럽트 및 메인 함수
// ==========================================
ISR(INT0_vect){
    data_ready_flag = 1;	
}

int main(void)
{
    // [하드웨어 핀 설정]
    PORTD &= ~(1 << PORTD4); DDRD |= (1 << PORTD4); // LED (D4) 출력 모드 (초기 LOW)
    PORTD |= (1 << PORTD5);  DDRD &= ~(1 << PORTD5); // 버튼 (D5) 입력 모드 (내부 풀업)
    
    // [통신 모듈 초기화]
    TWI_Init();
    MPU6050_Init();
    SPI_Init();
    
    // [디스플레이 초기화]
    OLED_Init();
    power_state = 1; // 켜진 상태로 시작
    PORTD |= (1 << PORTD4); // LED ON
    
    // [외부 인터럽트 설정 (MPU6050 INT)]
    PORTD &= ~(1 << PORTD2); DDRD &= ~(1 << PORTD2); 
    EICRA |= (1 << ISC01) | (1 << ISC00); // Rising Edge
    EIMSK |= (1 << INT0); 
    SREG |= (1 << 7); // 전역 인터럽트 활성화
    
    char buffer[32]; // 문자열 버퍼

    while (1) {	
        // --------------------------------------
        // [1] 전원 버튼 토글 로직 (Debounce 적용)
        // --------------------------------------
        if(!(PIND & (1 << PIND5))){
            _delay_ms(50); // 채터링 방지
            if(!(PIND & (1 << PIND5))){
                power_state = !power_state; // 상태 반전
                
                if (power_state) {
                    OLED_Init(); // 칩 깨우기 및 초기화
                    PORTD |= (1 << PORTD4); // LED ON
                } else {
                    OLED_PowerOff(); // 안전 종료
                    PORTD &= ~(1 << PORTD4); // LED OFF
                }
                
                while(!(PIND & (1 << PIND5))); // 버튼 뗄 때까지 대기
            }
        }
        
        // --------------------------------------
        // [2] 센서 데이터 렌더링 로직
        // --------------------------------------
        if (data_ready_flag && power_state) {
            data_ready_flag = 0;

            // 센서 값 읽어오기
            MPU6050_ReadBurst(&ax, &ay, &az);

            // X축 가속도 출력 (PAGE 0)
            OLED_Write_Command(0xB0); OLED_Write_Command(0x00); OLED_Write_Command(0x10);
            sprintf(buffer, "AX:%6d", ax); // 고정 길이 6자리 맞춤
            OLED_PrintString(buffer);

            // Y축 가속도 출력 (PAGE 2 - 두 줄 아래)
            OLED_Write_Command(0xB2); OLED_Write_Command(0x00); OLED_Write_Command(0x10);
            sprintf(buffer, "AY:%6d", ay);
            OLED_PrintString(buffer);

            // Z축 가속도 출력 (PAGE 4)
            OLED_Write_Command(0xB4); OLED_Write_Command(0x00); OLED_Write_Command(0x10);
            sprintf(buffer, "AZ:%6d", az);
            OLED_PrintString(buffer);
            
            _delay_ms(50); // 화면 갱신 주기 조절
        }
    }
}