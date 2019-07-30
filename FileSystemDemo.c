#include "cfs-fat.h"
#include "fs_test.h"

#include "string.h"
#include <inttypes.h>
#include <math.h>

/* config.h is the required application configuration; RAM layout, stack, chip type etc. */
#include "mxc_config.h" 
#include "board.h"

#include "rtc.h"
#include "spi.h"
#include "tmr.h"
#include "tmon.h"
#include "dac.h"
#include "adc.h"
#include "afe.h"

#include "icc.h"
#include "ioman.h"
#include "clkman.h"
#include "gpio.h"
#include "power.h"
#include "systick.h"
#include "lcd.h"
#include "trim_regs.h"
#include "mxc_config.h"
#include "uart.h"

static const char message[] = "         Hello World! MAX32600 Ev Kit ...       ";
static uint8_t led_mode = 0;

/* This array is an ASCII to LCD char mapping, starting at ascii 0x20, such that
 * offset  0 => ascii 0x20 or ' ' (space),·
 * offset 48 => ascii 0x30 or '1' ...·
 * values of 0xffff are invalid or undisplayable on this LCD.
 */
static const uint16_t lcd_value[] =
{
    0x0000, 0xFFFF, 0x00C0, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0x0008, 0x0050, 0xA000,
    0xF0F0, 0x50A0, 0x2000, 0x4020, 0x0100, 0x2040, // *+,-./
    0x2E47, 0x0006, 0x4C23, 0x4E21, 0x50A4, 0x8A21, 0x4A27, 0x0E00, 0x4E27, 0x4e25, // 0-9
    0xFFFF, 0xFFFF, 0x0050, 0xFFFF, 0xA000, 0xFFFF, 0xFFFF,
    0x4E26, 0x4857, 0x0807, 0x1E81, 0x4827, 0x4826, 0x0A37, 0x4626, 0x1881, 0x0603, // A-J
    0x10D0, 0x0007, 0x8646, 0x8616, 0x0E07, 0x4C26, 0x0E17, 0x4C36, 0x4A25, 0x1880, // K-T
    0x0607, 0x8610, 0x2616, 0xA050, 0x9040, 0x2841,           // U-Z
    0x0C01, 0x8010, 0x0807, 0xFFFF, 0x8080, 0x0100,
    0x4E26, 0x4857, 0x0807, 0x1E81, 0x4827, 0x4826, 0x0A37, 0x4626, 0x1881, 0x0603, // a-j
    0x10D0, 0x0007, 0x8646, 0x8616, 0x0E07, 0x4C26, 0x0E17, 0x4C36, 0x4A25, 0x1880, // k-t
    0x0607, 0x8610, 0x2616, 0xA050, 0x9040, 0x2841,           // u-z
    0xFFFF, 0x0090, 0xFFFF, 0xFFFF, 0xFFFF
};  


#define UART_PORT       0
#define BAUD_RATE       115200

/* buffer space used by libc stdout */
static char uart_stdout_buf0[96];

/* The following libc stub functions are required for a proper link with printf().
 * These can be tailored for a complete stdio implimentation
 */
int _open(const char *name, int flags, int mode)
{
    return -1;
}
int _close(int file)
{
    return -1;
}
int _isatty(int file)
{
    return -1;
}
int _lseek(int file, off_t offset, int whence)
{
    return -1;
}
int _fstat(int file, struct stat *st)
{
    return -1;
}
int _read(int file, uint8_t *ptr, size_t len)
{
    return -1;
}

/* newlib/libc printf() will eventually call _write() to get the data to the stdout */
int _write(int file, char *ptr, int len)
{
    int ret_val = UART_Write(UART_PORT, (uint8_t*)ptr, len);

    /* auto insert a carrage return to be nice to terminals
     * we enabled "buffered IO", therefore this will be called for
     * every '\n' in printf()
     */
    if(ptr[len-1] == '\n')
        UART_Write(UART_PORT, (uint8_t*)"\r", 1);

    return ret_val;
}

/* buffer space for UART input */
static uint8_t input_char;
static void uart_rx_handler(int32_t bytes)
{
    /* echo input char(s) back to UART */
    UART_Write(UART_PORT, &input_char, bytes);

    return;
}

/**
 * This function displays a single character on the LCD at the position
 * given.
 */
static int32_t lcd_display_char(uint8_t position, uint8_t ch)
{
    uint8_t index = ch - 0x20;
    uint8_t high_byte, low_byte;

    if (((ch >= 0x20) && (ch <= 0x7F)) && (position < 8 ))
    {
        if (index > (sizeof(lcd_value) >> 1)) /* Out of range */
        {
            return 1;
        }
        else if (lcd_value[index] == 0xFFFF ) /* Unsuported character */
        {
            return 1;
        }

        high_byte = (lcd_value[index] >> 8) & 0xFF;
        low_byte = lcd_value[index] & 0xFF;

        LCD_Write(position, low_byte);
        LCD_Write((15-position), high_byte);

        return 0;
    }
    else
    {
        return 1;
    }
}

/**
 * Initializes the three LCD LED's on the MAX32600 EV Kit
 */
void blinky_init(void)
{
    /* setup GPIO for the LED */
    GPIO_SetOutMode(GREEN_LED_PORT, GREEN_LED_PIN, MXC_E_GPIO_OUT_MODE_OPEN_DRAIN_W_PULLUP);
    GPIO_SetOutMode(YELLOW_LED_PORT, YELLOW_LED_PIN, MXC_E_GPIO_OUT_MODE_OPEN_DRAIN_W_PULLUP);
    GPIO_SetOutMode(RED_LED_PORT, RED_LED_PIN, MXC_E_GPIO_OUT_MODE_OPEN_DRAIN_W_PULLUP);
    GPIO_SetOutVal(GREEN_LED_PORT, GREEN_LED_PIN, LED_ON);
    GPIO_SetOutVal(YELLOW_LED_PORT, YELLOW_LED_PIN, LED_OFF);
    GPIO_SetOutVal(RED_LED_PORT, RED_LED_PIN, LED_OFF);
}

/**
 * Setups the LCD Drivers to work with the PD878 LCD Screen
 */
void LCD_InitPd878(void)
{
    IOMAN_LCD(1,0xffffffff, 0);
    LCD_Init(PD878_SEGMENTS, PD878_GND_ENABLE, PD878_FRM_VALUE, PD878_DUTY_CYCLE, 
            lcd_display_char, LCD_MAX_LENGTH);
    LCD_Enable();
    LCD_Clear();

}

/* Interrupt handler which changes the state based upon Test push button */
static void change_led_mode(void)
{
    if(GPIO_GetInVal(SW1_PORT, SW1_PIN))
    {
        led_mode = ~led_mode;
    }
}

/*
 * Main function of occurring things, modifies state of LCD and LED's
 * Called by the systick interrupt.
 */
void change_state(void)
{
    static uint8_t led_state = 0;
    static char* position = (char*)message;

    /* Scroll through the message and restart once we hit the end */
    ++position;
    if(*position == '\0')
    {
        position = (char*)message;
    }

    LCD_Display((uint8_t *) position);
    LCD_Update();

    /* If in first mode alternate lights up and down */
    if(!led_mode){
        GPIO_SetOutVal(GREEN_LED_PORT, GREEN_LED_PIN, LED_OFF);
        GPIO_SetOutVal(YELLOW_LED_PORT, YELLOW_LED_PIN, LED_OFF);
        GPIO_SetOutVal(RED_LED_PORT, RED_LED_PIN, LED_OFF);

        switch(led_state)
        {
            case 0:
                GPIO_SetOutVal(GREEN_LED_PORT, GREEN_LED_PIN, LED_ON);
                led_state = 1;
                break;
            case 1:
                GPIO_SetOutVal(RED_LED_PORT, RED_LED_PIN, LED_ON);
                led_state = 2;
                break;
            case 2:
                GPIO_SetOutVal(YELLOW_LED_PORT, YELLOW_LED_PIN, LED_ON);
                led_state = 3;
                break;
            case 3:
                GPIO_SetOutVal(RED_LED_PORT, RED_LED_PIN, LED_ON);
                led_state = 0;
                break;
        }
    }
    else
    {
        /* If in second mode toggle two lights together */
        if(led_state)
        {
            GPIO_SetOutVal(GREEN_LED_PORT, GREEN_LED_PIN, LED_OFF);
            GPIO_SetOutVal(YELLOW_LED_PORT, YELLOW_LED_PIN, LED_OFF);
            GPIO_SetOutVal(RED_LED_PORT, RED_LED_PIN, LED_ON);
            led_state = 0;
        }
        else
        {
            GPIO_SetOutVal(GREEN_LED_PORT, GREEN_LED_PIN, LED_ON);
            GPIO_SetOutVal(YELLOW_LED_PORT, YELLOW_LED_PIN, LED_ON);
            GPIO_SetOutVal(RED_LED_PORT, RED_LED_PIN, LED_OFF);
            led_state = 1;
        }
    }
}

/* called in interrupt context */
static void systick_handler(uint32_t ticks)
{

	uint8_t temp[16] = {10, 11, 12, 13, 10, 11, 12, 13, 10, 11, 12, 13, 10, 11, 12, 13};
	static int flag = 1;
	/*RUN_TEST("test_sdinit", test_sdinit);
	RUN_TEST("test_mkfs", test_mkfs);
	RUN_TEST("test_fwrite", test_fwrite);
	RUN_TEST("test_fseek", test_fseek);
	RUN_TEST("test_fread", test_fread);
	RUN_TEST("test_fremove", test_fremove);
	TESTS_DONE();*/

    /* toggle state every 256 ticks (1024Hz/256 => 0.25 sec) for a 4Hz blink */
    if((ticks & 255) == 0)
    {
        change_state();
        //FS_TEST_Run();
        if(flag == 1)
        {
        	flag = 0;
        }
        else
        {
        	flag = 1;
        }
        GPIO_SetOutVal(SPI0_PORT, SPI0_CS, flag);
    	//GPIO_SetOutVal(SPI0_PORT, SPI0_CS, 0);
    	//SPI_Transmit(&_g_sdcard_ss, temp, 16, NULL, 0, MXC_E_SPI_UNIT_BYTES, MXC_E_SPI_MODE_4_WIRE, 0, 1);
    	//GPIO_SetOutVal(SPI0_PORT, SPI0_CS, 1);
    }
}

#ifndef TOP_MAIN
int main(void)
{
    /* enable instruction cache */
    ICC_Enable();

    /* use the internal Ring Osc in 24MHz mode */
    CLKMAN_SetSystemClock(MXC_E_CLKMAN_SYSTEM_SOURCE_SELECT_24MHZ_RO);

    /* Setup the charge pump so the display acts normally */
    CLKMAN_SetClkScale(MXC_E_CLKMAN_CLK_LCD_CHPUMP, MXC_E_CLKMAN_CLK_SCALE_ENABLED);

    /* Setup the GPIO clock so the push button interrupt works */
    CLKMAN_SetClkScale(MXC_E_CLKMAN_CLK_GPIO, MXC_E_CLKMAN_CLK_SCALE_ENABLED);

    /* set systick to the RTC input 32.768KHz clock, not system clock; this is needed to keep JTAG alive */
    CLKMAN_SetRTOSMode(TRUE);

    /* enable real-time clock during run mode, this is needed to drive the systick with the RTC crystal */
    PWR_EnableDevRun(MXC_E_PWR_DEVICE_RTC);

    /* setup LED */
    blinky_init();

    /* Change LED state based upon button press through interrupt */
    GPIO_SetInMode(SW1_PORT, SW1_PIN, MXC_E_GPIO_IN_MODE_INVERTED);
    GPIO_SetIntMode(SW1_PORT, SW1_PIN, MXC_E_GPIO_INT_MODE_ANY_EDGE, change_led_mode);

    /* Setup the PD878 LCD Display */
    LCD_InitPd878();

    /*
     *  SPIO setup
     */

    GPIO_SetOutMode(SPI0_PORT, SPI0_CS, MXC_E_GPIO_OUT_MODE_NORMAL);
    GPIO_SetOutVal(SPI0_PORT, SPI0_CS, SDCARD_SLEEP);

    // setup SPIO for SD CARD module
    /*IOMAN_SPI0(MXC_E_IOMAN_MAPPING_A, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0);

    SPI_Config(&_g_sdcard_ss, SPI0_PORT);
    SPI_ConfigClock(&_g_sdcard_ss, 3, 3, 0, 0, 0, 0);
    SPI_ConfigSlaveSelect(&_g_sdcard_ss, SDCARD_SS, SDCARD_POLARITY, SDCARD_ACT_DELAY_TICKS, SDCARD_INACT_DELAY_TICKS);*/

    /* tie interrupt handler to systick */
    SysTick_Config(SYSTICK_PERIOD, systick_handler);

    /* setup UART0 pins, mapping as EvKit to onboard FTDI UART to USB */
    IOMAN_UART0(MXC_E_IOMAN_MAPPING_A, TRUE, FALSE, FALSE);

    /* setup UART */
    UART_Config(UART_PORT, BAUD_RATE, FALSE, FALSE, FALSE);

    /* give libc a buffer, this way libc will not malloc() a stdio buffer */
    setvbuf(stdout, uart_stdout_buf0, _IOLBF, (size_t)sizeof(uart_stdout_buf0));
    setvbuf(stdin, NULL, _IONBF, 0);

    /* set input handler */
    UART_ReadAsync(UART_PORT, &input_char, sizeof(input_char), uart_rx_handler);

    NVIC_SetPriority(SPI0_IRQn, 2);
    NVIC_SetPriority(GPIO_P0_IRQn, 6);
    NVIC_SetPriority(TMR0_IRQn, 6);
    NVIC_SetPriority(UART0_IRQn, 6);

    /* print out welcome message */
    DEBUG_PRINT("\n\nMaxim Integrated MAX32600\n");
    DEBUG_PRINT("\tFileSystemDemo\n\n");

    for(;;) {
        /* default sleep mode is "LP2"; core powered up, ARM in "Wait For Interrupt" mode */
        PWR_Sleep();
    }
    return 0;
}
#endif /* TOP_MAIN */
