#include <ctype.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "debug.h"
#include "flash.h"
#include "gpio.h"
#include "hw_i2c.h"
#include "hw_memmap.h"
#include "hw_types.h"
#include "i2c.h"
#include "interrupt.h"
#include "pin_map.h"
#include "pwm.h"
#include "sysctl.h"
#include "systick.h"
#include "tm4c1294ncpdt.h"
#include "uart.h"

#define SYSTICK_FREQUENCY 10000  // 10000hz

#define I2C_FLASHTIME 500   // 500mS
#define GPIO_FLASHTIME 300  // 300mS
//*****************************************************************************
//
// I2C GPIO chip address and resigster define
//
//*****************************************************************************
#define TCA6424_I2CADDR 0x22
#define PCA9557_I2CADDR 0x18

#define PCA9557_INPUT 0x00
#define PCA9557_OUTPUT 0x01
#define PCA9557_POLINVERT 0x02
#define PCA9557_CONFIG 0x03

#define TCA6424_CONFIG_PORT0 0x0c
#define TCA6424_CONFIG_PORT1 0x0d
#define TCA6424_CONFIG_PORT2 0x0e

#define TCA6424_INPUT_PORT0 0x00
#define TCA6424_INPUT_PORT1 0x01
#define TCA6424_INPUT_PORT2 0x02

#define TCA6424_OUTPUT_PORT0 0x04
#define TCA6424_OUTPUT_PORT1 0x05
#define TCA6424_OUTPUT_PORT2 0x06

#define MAX_COMMAND_ARGS 3         // 指令最大参数数量
#define MAX_COMMAND_ARG_LENGTH 10  // 每个参数最大长度
#define COMMAND_TYPES 7            // 指令类型数量

#define FLASH_USER_DATA_ADDR 0x20000  // flash user data address

void Delay(uint32_t value);
void S800_GPIO_Init(void);
uint8_t I2C0_WriteByte(uint8_t DevAddr, uint8_t RegAddr, uint8_t WriteData);
uint8_t I2C0_ReadByte(uint8_t DevAddr, uint8_t RegAddr);
void S800_I2C0_Init(void);
void S800_UART_Init(void);
void PWM_Init(void);
void MY_Init(void);
void FLASH_Init(void);
void UARTStringPut(const char* cMessage);

void process_SW(void);

// void UARTStringPutNonBlocking(const char* cMessage);
// void UARTStringGetNonBlocking(char* msg);
char ASCII2Disp(char* buff);
char ASCII2PointDisp(char* buff);
char ASCII2Disp_R(char* buff);
char ASCII2PointDisp_R(char* buff);

void displayTime(void);
void displayDate(void);
void displayAlarm(void);
void displayStopwatch(void);
void displayRuntime(void);

void updateTime(void);
void update_date_disp(void);
void update_alarm_disp(void);
void updateStopwatch(void);
void updateRuntime(void);

bool checkDate(uint32_t year, uint8_t month, uint8_t day);
void addOneDay(uint32_t* year, uint8_t* month, uint8_t* day);
void addOneMonth(uint32_t* year, uint8_t* month, uint8_t* day);

bool is_command_arg_empty(int arg_index);
bool is_time_arg_valid(int arg_index);
bool is_date_arg_valid(int arg_index);
bool helpEnable = false;
bool uartActivate = false;

void WriteToFlash(uint32_t year, uint8_t month, uint8_t day, uint32_t ui32Time);
bool ReadFromFlash(uint32_t* year,
                   uint8_t* month,
                   uint8_t* day,
                   uint32_t* ui32Time);

// systick software counter define
volatile uint16_t systick_10ms_couter, systick_100ms_couter;
volatile uint8_t systick_10ms_status, systick_100ms_status;
volatile uint16_t systick_1ms_couter, systick_2ms_couter, systick_500ms_couter;
volatile uint8_t systick_1ms_status, systick_2ms_status, systick_500ms_status;

volatile uint8_t result, cnt, key_value, gpio_status;
uint32_t ui32SysClock;

volatile uint8_t SW_n = 0xFF;
volatile uint8_t prev_SW_n = 0xFF;
volatile uint8_t rightshift = 0x01;

uint8_t const SWs[] = {0xff, 0xfe, 0xfd, 0xfb, 0xf7, 0xef, 0xdf, 0xbf, 0x7f};
uint8_t bits_selected = 0;
uint8_t const bits_select[] = {0xFF, 0x3F, 0xCF, 0xF3, 0xFC};
uint8_t const bits_select_R[] = {0xFF, 0xFC, 0xF3, 0xCF, 0x3F};
volatile uint8_t tmp_bits_select = 0xFF;

char disp_buff_time[8];
char disp_buff_date[8];
char disp_buff_alarm[8];
char disp_buff_stopwatch[8];
char disp_buff_runtime[8];
char disp_buff_static[8];

uint8_t read_buff_time[8];
uint8_t read_buff_date[8];

uint32_t year;
uint8_t month, day;

volatile bool reverse = 0;

// the disp_tab and disp_tab_7seg must be the Corresponding relation
// the last character should be the space(not display) character
char const disp_tab[] = {'0', '1', '2', '3', '4',
                         '5',  // the could display char and its segment
                               // code
                         '6', '7', '8', '9', 'A', 'b', 'C', 'd', 'E', 'F', 'H',
                         'L', 'P', 'o', '.', '-', '_', ' '};
char const disp_tab_7seg[] = {0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07,
                              0x7F, 0x6F, 0x77, 0x7C, 0x39, 0x5E, 0x79, 0x71,
                              0x76, 0x38, 0x73, 0x5c, 0x80, 0x40, 0x08, 0x00};
char const disp_tab_point_7seg[] = {  // with point
    0xBF, 0x86, 0xDB, 0xCF, 0xE6, 0xED, 0xFD, 0x87, 0xFF, 0xEF, 0xF7, 0xFC,
    0xB9, 0xDE, 0xF9, 0xF1, 0xF6, 0xB8, 0xF3, 0xDc, 0x80, 0xC0, 0x88, 0x00};
char const disp_tab_7seg_R[] = {0x3F, 0x30, 0x5B, 0x79, 0x74,
                                0x6D, 0x6F, 0x38, 0x7F, 0x7D};
char const disp_tab_point_7seg_R[] = {0xBF, 0xB0, 0xDB, 0xF9, 0xF4,
                                      0xED, 0xEF, 0xB8, 0xFF, 0xFD};

char const pattern[16][64] = {
    "       #               #               #                        ",
    "        #              #               #          #    #######  ",
    "        #              #              # #          #   #  #  #  ",
    " ##############        #             #   #         #   #######  ",
    "     #     #     ##############     #     #            #  #  #  ",
    "     #      #          #           #       #     ###   #  #  #  ",
    "    #        ##       # #         # ####### ###    #   #######  ",
    "   # #     #          # #        #     #     #     #      #     ",
    "  #   #   #           #  #             #           #  ######### ",
    "       # #           #   #         #########       #     ##     ",
    "        #            #    #            #           # #  # ##    ",
    "       # #          #     #        #   #   #       ##  #  # #   ",
    "     ##   ##       #       #        #  #  #        #  #   #  ## ",
    "   ##       ###   #         ###      # # #           #    #  #  ",
    " ##          #   #           #   #############            #     ",
    "                                                                ",
};

char const help_msg[COMMAND_TYPES][200] = {
    "You can use this as a terminal. Try command like \"INIT\", \"SET\", "
    "\"GET\", \"RUN\", \"REVERSE\", \"SAVE\".",
    "Initialize all timers, use \"INIT CLOCK\".",
    "Set time, date, alarm or stopwatch, use \"SET TIME HH:MM:SS\" or \"SET "
    "DATE "
    "YYYY.MM.DD\" or \"SET ALARM "
    "HH:MM:SS\" or \"SET STWATCH HH:MM:SS\".",
    "Get runtime, time, date, alarmtime or stopwatch time, use \"GET RUNTIME\" "
    "or \"GET TIME\" or \"GET "
    "DATE\" or \"GET "
    "ALARM\" or \"GET STWATCH\".",
    "Display runtime, time, date, alarmtime or stopwatch, use \"RUN RUNTIME\" "
    "or \"RUN TIME\" or \"RUN DATE\" or \"RUN ALARM\" or \"RUN STWATCH\".",
    "Reverse the display, use \"REVERSE\".",
    "Save the current time and date to flash, use \"SAVE\"."};

int arg_index = 0;
int arg_length = 0;
int needed_arg_count = 0;
bool is_command_prefix_valid = false;
bool is_command_arg_valids[MAX_COMMAND_ARGS] = {false};
int help_index = 0;

unsigned char RxBuf[256];
char buffer[256];  // 显示的信息
// char buffer[128];
bool RxEndFlag = 0;
uint8_t command_cnt = 0;
bool stopwatchEnable = false;

bool freeze = false;    // 显示冻结标志
bool half_sec = false;  // 0.5s标志

bool USR_SW1_n = 1, USR_SW2_n = 1;  // 红板按键状态
bool prev_USR_SW1_n = 1, prev_USR_SW2_n = 1;

uint32_t ui32Time, ui32Stopwatch;  // time in 0.01s
uint32_t ui32Alarm, ui32Stopwatch_static;
uint32_t ui32RunTime;  // time in 0.001s
uint32_t USR_SW1_start_time, USR_SW2_start_time, USR_SW1_stop_time,
    USR_SW2_stop_time;

uint32_t ui32PWMClock;  // PWM模块的时钟频率
// 定义数组，表示音乐<小星星>的频率，音符，节拍
uint16_t const music_freq[] = {0, 262, 294, 330, 349, 392, 440, 494, 523};
uint16_t music_note[] = {1, 1, 5, 5, 6, 6, 5, 4, 4, 3, 3, 2, 2, 1};
uint16_t music_time[] = {500, 500, 500, 500, 500, 500, 1000,
                         500, 500, 500, 500, 500, 500, 1000};

int note_index = sizeof(music_note) / sizeof(uint16_t);
int note_delay = 0;

uint8_t command_mode = 0;  // 当前模式，默认显示运行时间
uint8_t disp_mode =
    0;  // 当前显示模式，0为显示运行时间，1为显示时间，2为显示日期，3为显示闹钟，4为显示秒表
char command[MAX_COMMAND_ARGS]
            [MAX_COMMAND_ARG_LENGTH +
             1];  // [0]为指令关键字，[1]为参数1，[2]为参数2，[3]为参数3，...
char command_upper[MAX_COMMAND_ARGS][MAX_COMMAND_ARG_LENGTH +
                                     1];  // 存储转换为大写后的指令关键字和参数
char set_arg_2[MAX_COMMAND_ARG_LENGTH +
               1];  // 用于存储set指令的第二个参数，设置的时间或日期

int main(void) {
    volatile uint16_t i2c_flash_cnt, gpio_flash_cnt;
    ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_16MHZ | SYSCTL_OSC_INT |
                                       SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480),
                                      20000000);

    SysTickPeriodSet(ui32SysClock / SYSTICK_FREQUENCY);
    SysTickEnable();
    SysTickIntEnable();
    IntMasterEnable();

    S800_GPIO_Init();
    S800_I2C0_Init();
    S800_UART_Init();
    PWM_Init();
    MY_Init();

    while (1) {
        uint32_t ui32Hour;
        uint32_t ui32Minute;
        uint32_t ui32Second;
        uint32_t ui32Centisecond;
        uint32_t ui32Millisecond;
        uint32_t ui32PressTime;

        if (systick_2ms_status) {  // 逐位显示数码管, 2ms切换一位
            systick_2ms_status = 0;
            cnt++;
            rightshift = rightshift << 1;
            if (cnt >= 0x8) {
                rightshift = 0x01;
                cnt = 0;
            }
        }

        // Execute command
        switch (command_mode) {
            case 0:
                break;
            case 1:
                // INIT CLOCK
                ui32Time = 2885900;  // 08:00:59:00
                year = 2023;
                month = 6;
                day = 11;  // 2023.06.11
                update_date_disp();
                ui32Alarm = 9 * 60 * 60 * 100;  // 闹钟默认 09:00:00:00
                update_alarm_disp();
                ui32Stopwatch = 3000;  // 倒计时默认30s
                ui32Stopwatch_static = ui32Stopwatch;
                stopwatchEnable = false;
                ui32RunTime = 0;  // 运行时间从0开始
                reverse = 0;      // 取消翻转显示
                UARTStringPut((uint8_t*)"Initialize clock!\r\n");
                command_mode = 0;
                break;
            case 2:
                // SET TIME
                ui32Time = 0;
                ui32Time += ((set_arg_2[0] - '0') * 10 + (set_arg_2[1] - '0')) *
                            3600 * 100;
                ui32Time += ((set_arg_2[3] - '0') * 10 + (set_arg_2[4] - '0')) *
                            60 * 100;
                ui32Time +=
                    ((set_arg_2[6] - '0') * 10 + (set_arg_2[7] - '0')) * 100;
                snprintf(buffer, 128, "Set time to %s !\r\n", set_arg_2);
                UARTStringPut((uint8_t*)buffer);
                command_mode = 0;
                break;
            case 3:
                // SET DATE
                sscanf(set_arg_2, "%hhu.%hhu.%hhu", &year, &month, &day);
                update_date_disp();
                snprintf(buffer, 128, "Set date to %s !\r\n", set_arg_2);
                UARTStringPut((uint8_t*)buffer);
                command_mode = 0;
                break;
            case 4:
                // SET ALARM
                ui32Alarm = 0;
                ui32Alarm +=
                    ((set_arg_2[0] - '0') * 10 + (set_arg_2[1] - '0')) * 3600 *
                    100;
                ui32Alarm +=
                    ((set_arg_2[3] - '0') * 10 + (set_arg_2[4] - '0')) * 60 *
                    100;
                ui32Alarm +=
                    ((set_arg_2[6] - '0') * 10 + (set_arg_2[7] - '0')) * 100;
                update_alarm_disp();
                snprintf(buffer, 128, "Set alarm time to %s !\r\n", set_arg_2);
                UARTStringPut((uint8_t*)buffer);
                command_mode = 0;
                break;
            case 5:
                // SET STWATCH
                ui32Stopwatch = 0;
                ui32Stopwatch +=
                    ((set_arg_2[0] - '0') * 10 + (set_arg_2[1] - '0')) * 3600 *
                    100;
                ui32Stopwatch +=
                    ((set_arg_2[3] - '0') * 10 + (set_arg_2[4] - '0')) * 60 *
                    100;
                ui32Stopwatch +=
                    ((set_arg_2[6] - '0') * 10 + (set_arg_2[7] - '0')) * 100;
                snprintf(buffer, 128, "Set stopwatch time to %s !\r\n",
                         set_arg_2);
                ui32Stopwatch_static = ui32Stopwatch;
                UARTStringPut((uint8_t*)buffer);
                command_mode = 0;
                break;
            case 6:
                // GET RUNTIME
                ui32Hour = ui32RunTime / (100 * 60 * 60);
                ui32Minute = (ui32RunTime / (100 * 60)) % 60;
                ui32Second = (ui32RunTime / 100) % 60;
                ui32Centisecond = ui32RunTime % 100;
                snprintf(buffer, 128,
                         "Current runtime is %02d:%02d:%02d:%02d.\r\n",
                         ui32Hour, ui32Minute, ui32Second, ui32Centisecond);
                UARTStringPut((uint8_t*)buffer);
                command_mode = 0;
                break;
            case 7:
                // GET TIME
                ui32Hour = ui32Time / (100 * 60 * 60);
                ui32Minute = (ui32Time / (100 * 60)) % 60;
                ui32Second = (ui32Time / 100) % 60;
                ui32Centisecond = ui32Time % 100;
                snprintf(buffer, 128,
                         "Current time is %02d:%02d:%02d:%02d.\r\n", ui32Hour,
                         ui32Minute, ui32Second, ui32Centisecond);
                UARTStringPut((uint8_t*)buffer);
                command_mode = 0;
                break;
            case 8:
                // GET DATE
                snprintf(buffer, 128, "Current date is %.4s-%.2s-%.2s.\r\n",
                         disp_buff_date, disp_buff_date + 4,
                         disp_buff_date + 6);
                UARTStringPut((uint8_t*)buffer);
                command_mode = 0;
                break;
            case 9:
                // GET ALARM
                ui32Hour = ui32Alarm / (100 * 60 * 60);
                ui32Minute = (ui32Alarm / (100 * 60)) % 60;
                ui32Second = (ui32Alarm / 100) % 60;
                ui32Centisecond = ui32Alarm % 100;
                snprintf(buffer, 128,
                         "Current alarm time is %02d:%02d:%02d:%02d.\r\n",
                         ui32Hour, ui32Minute, ui32Second, ui32Centisecond);
                UARTStringPut((uint8_t*)buffer);
                command_mode = 0;
                break;
            case 10:
                // GET STWATCH
                ui32Hour = ui32Stopwatch / (100 * 60 * 60);
                ui32Minute = (ui32Stopwatch / (100 * 60)) % 60;
                ui32Second = (ui32Stopwatch / 100) % 60;
                ui32Centisecond = ui32Stopwatch % 100;
                snprintf(buffer, 128,
                         "Current stopwatch time is %02d:%02d:%02d:%02d.\r\n",
                         ui32Hour, ui32Minute, ui32Second, ui32Centisecond);
                UARTStringPut((uint8_t*)buffer);
                command_mode = 0;
                break;
            case 11:
                // RUN RUNTIME
                freeze = false;
                bits_selected = 0;
                stopwatchEnable = false;
                disp_mode = 0;
                UARTStringPut((uint8_t*)"Display runtime!\r\n");
                command_mode = 0;
                break;
            case 12:
                // RUN TIME
                freeze = false;
                bits_selected = 0;
                stopwatchEnable = false;
                disp_mode = 1;
                UARTStringPut((uint8_t*)"Display time!\r\n");
                command_mode = 0;
                break;
            case 13:
                // RUN DATE
                freeze = false;
                bits_selected = 0;
                stopwatchEnable = false;
                disp_mode = 2;
                UARTStringPut((uint8_t*)"Display date!\r\n");
                command_mode = 0;
                break;
            case 14:
                // RUN ALARM
                freeze = false;
                bits_selected = 0;
                stopwatchEnable = false;
                disp_mode = 3;
                UARTStringPut((uint8_t*)"Running alarm!\r\n");
                note_index = 0;
                note_delay = music_time[note_index] / 100;
                command_mode = 0;
                break;
            case 15:
                // RUN STWATCH
                freeze = false;
                bits_selected = 0;
                stopwatchEnable = true;
                if (ui32Stopwatch == 0) {
                    ui32Stopwatch = ui32Stopwatch_static;  // 重置秒表
                }
                disp_mode = 4;
                UARTStringPut((uint8_t*)"Run stopwatch!\r\n");
                command_mode = 0;
                break;
            case 16:
                // REVERSE
                reverse = !reverse;
                UARTStringPut((uint8_t*)"Reverse the display!\r\n");
                command_mode = 0;
                break;
            case 17:
                // SAVE
                WriteToFlash(year, month, day, ui32Time);
                ui32Hour = ui32Time / (100 * 60 * 60);
                ui32Minute = (ui32Time / (100 * 60)) % 60;
                ui32Second = (ui32Time / 100) % 60;
                ui32Centisecond = ui32Time % 100;
                snprintf(
                    buffer, 128,
                    "Save time %02d:%02d:%02d:%02d and date %.4s-%.2s-%.2s to "
                    "flash! Will be loaded after a reboot.\r\n",
                    ui32Hour, ui32Minute, ui32Second, ui32Centisecond,
                    disp_buff_date, disp_buff_date + 4, disp_buff_date + 6);
                UARTStringPut((uint8_t*)buffer);
                command_mode = 0;
                break;
            default:
                disp_mode = 0;
                command_mode = 0;
                break;
        }

        if (helpEnable) {  // 对 ? 类指令打印帮助信息
            UARTStringPut((uint8_t*)help_msg[help_index]);
            helpEnable = false;
        }
        while (uartActivate) {  // 对错误的指令给出帮助信息
            uartActivate = false;
            bool break_flag = false;
            int i;
            // Print help message if command is invalid
            if (!is_command_prefix_valid && !is_command_arg_empty(0)) {
                snprintf(buffer, 128, "Invalid command: %s\r\n", command[0]);
                UARTStringPut((uint8_t*)buffer);
                UARTStringPut((uint8_t*)help_msg[help_index]);
                break;
            }
            // Print help message if command argument is more than needed
            if (arg_index > needed_arg_count) {
                snprintf(buffer, 128, "Too many arguments for command %s\r\n",
                         command_upper[0]);
                UARTStringPut((uint8_t*)buffer);
                UARTStringPut((uint8_t*)help_msg[help_index]);
                break;
            }
            // Print help message if command argument is invalid
            for (i = 1; i < arg_index + 1; i++) {
                if (!is_command_arg_valids[i]) {  // 只输出第1个无效参数
                    snprintf(buffer, 128,
                             "Invalid argument-%d %s for command %s\r\n", i,
                             command[i], command_upper[0]);
                    UARTStringPut((uint8_t*)buffer);
                    UARTStringPut((uint8_t*)help_msg[help_index]);
                    break_flag = true;
                    break;
                }
            }
            if (break_flag) {
                break;
            }
            // Print help message if command argument is empty
            for (i = 1; i < needed_arg_count + 1; i++) {
                if (arg_index < i) {
                    snprintf(buffer, 128,
                             "Empty argument-%d for command %s\r\n", i,
                             command_upper[0]);
                    UARTStringPut((uint8_t*)buffer);
                    UARTStringPut((uint8_t*)help_msg[help_index]);
                    break;
                }
            }
        }

        // 显示模式
        switch (disp_mode) {
            case 0:
                // 显示运行时间
                displayRuntime();
                break;
            case 1:
                // 显示时间
                displayTime();
                break;
            case 2:
                // 显示日期
                displayDate();
                break;
            case 3:
                // 显示闹钟
                displayAlarm();
                break;
            case 4:
                // 显示秒表
                displayStopwatch();
                break;
            default:
                disp_mode = 0;
                displayRuntime();
                break;
        }

        // 输出红版按键触发时间
        if (systick_1ms_status) {
            systick_1ms_status = 0;
            USR_SW1_n = GPIOPinRead(GPIO_PORTJ_BASE, GPIO_PIN_0);
            USR_SW2_n = GPIOPinRead(GPIO_PORTJ_BASE, GPIO_PIN_1);
            if (USR_SW1_n == 0 && prev_USR_SW1_n == 1) {  // USR_SW1 is pressed
                USR_SW1_start_time = ui32RunTime;
                ui32Hour = USR_SW1_start_time / (1000 * 60 * 60);
                ui32Minute = (USR_SW1_start_time / (1000 * 60)) % 60;
                ui32Second = (USR_SW1_start_time / 1000) % 60;
                ui32Millisecond = USR_SW1_start_time % 1000;
                snprintf(buffer, 50,
                         "At %02d:%02d:%02d:%03d, USR_SW1 is pressed.\r\n",
                         ui32Hour, ui32Minute, ui32Second, ui32Millisecond);
                UARTStringPut((uint8_t*)buffer);

            } else if (USR_SW1_n == 1 &&
                       prev_USR_SW1_n == 0) {  // USR_SW1 is released
                USR_SW1_stop_time = ui32RunTime;
                ui32Hour = USR_SW1_stop_time / (1000 * 60 * 60);
                ui32Minute = (USR_SW1_stop_time / (1000 * 60)) % 60;
                ui32Second = (USR_SW1_stop_time / 1000) % 60;
                ui32Millisecond = USR_SW1_stop_time % 1000;
                snprintf(buffer, 50,
                         "At %02d:%02d:%02d:%03d, USR_SW1 is released.\r\n",
                         ui32Hour, ui32Minute, ui32Second, ui32Millisecond);
                UARTStringPut((uint8_t*)buffer);
                ui32PressTime = USR_SW1_stop_time - USR_SW1_start_time;
                ui32Hour = ui32PressTime / (1000 * 60 * 60);
                ui32Minute = (ui32PressTime / (1000 * 60)) % 60;
                ui32Second = (ui32PressTime / 1000) % 60;
                ui32Millisecond = ui32PressTime % 1000;
                snprintf(buffer, 50,
                         "USR_SW1 is pressed for %02d:%02d:%02d:%03d.\r\n\n",
                         ui32Hour, ui32Minute, ui32Second, ui32Millisecond);
                UARTStringPut((uint8_t*)buffer);
            }
            if (USR_SW2_n == 0 && prev_USR_SW2_n == 1) {  // USR_SW2 is pressed
                USR_SW2_start_time = ui32RunTime;
                ui32Hour = USR_SW2_start_time / (1000 * 60 * 60);
                ui32Minute = (USR_SW2_start_time / (1000 * 60)) % 60;
                ui32Second = (USR_SW2_start_time / 1000) % 60;
                ui32Millisecond = USR_SW2_start_time % 1000;
                snprintf(buffer, 50,
                         "At %02d:%02d:%02d:%03d, USR_SW2 is pressed.\r\n",
                         ui32Hour, ui32Minute, ui32Second, ui32Millisecond);
                UARTStringPut((uint8_t*)buffer);
                // Write data to flash
                WriteToFlash(year, month, day, ui32Time);

            } else if (USR_SW2_n == 1 &&
                       prev_USR_SW2_n == 0) {  // USR_SW2 is released
                USR_SW2_stop_time = ui32RunTime;
                ui32Hour = USR_SW2_stop_time / (1000 * 60 * 60);
                ui32Minute = (USR_SW2_stop_time / (1000 * 60)) % 60;
                ui32Second = (USR_SW2_stop_time / 1000) % 60;
                ui32Millisecond = USR_SW2_stop_time % 1000;
                snprintf(buffer, 50,
                         "At %02d:%02d:%02d:%03d, USR_SW2 is released.\r\n",
                         ui32Hour, ui32Minute, ui32Second, ui32Millisecond);
                UARTStringPut((uint8_t*)buffer);
                ui32PressTime = USR_SW2_stop_time - USR_SW2_start_time;
                ui32Hour = ui32PressTime / (1000 * 60 * 60);
                ui32Minute = (ui32PressTime / (1000 * 60)) % 60;
                ui32Second = (ui32PressTime / 1000) % 60;
                ui32Millisecond = ui32PressTime % 1000;
                snprintf(buffer, 50,
                         "USR_SW2 is pressed for %02d:%02d:%02d:%03d.\r\n\n",
                         ui32Hour, ui32Minute, ui32Second, ui32Millisecond);
                UARTStringPut((uint8_t*)buffer);
            }
            prev_USR_SW1_n = USR_SW1_n;
            prev_USR_SW2_n = USR_SW2_n;
        }

        if (systick_100ms_status) {
            systick_100ms_status = 0;
            // 播放音乐
            if (note_index < sizeof(music_note) / sizeof(uint16_t)) {
                if (note_delay == 0) {
                    PWMOutputState(PWM0_BASE, PWM_OUT_7_BIT, false);
                    note_index++;
                    note_delay = music_time[note_index] / 100;
                } else {
                    note_delay--;
                    PWMOutputState(PWM0_BASE, PWM_OUT_7_BIT, true);
                    PWMGenPeriodSet(
                        PWM0_BASE, PWM_GEN_3,
                        (ui32PWMClock / music_freq[music_note[note_index]]));
                    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7,
                                     PWMGenPeriodGet(PWM0_BASE, PWM_GEN_3) / 2);
                }
            }

            // 读取SW1-8状态
            SW_n = I2C0_ReadByte(TCA6424_I2CADDR, TCA6424_INPUT_PORT0);
            if (SW_n)
                process_SW();  // 处理SW1-8状态, 若SW_n ==
                               // 0可能是读取失败，不处理
            prev_SW_n = SW_n;
        }
    }
}

char ASCII2Disp(char* buff) {  // 显示一位ASCII字符
    char* pcDisp;
    pcDisp = (char*)strchr(disp_tab, *buff);
    if (pcDisp == NULL)
        return 0x0;
    else
        return (disp_tab_7seg[pcDisp - disp_tab]);
}

char ASCII2PointDisp(char* buff) {  // 显示一位带点的ASCII字符
    char* pcDisp;
    pcDisp = (char*)strchr(disp_tab, *buff);
    if (pcDisp == NULL)
        return 0x0;
    else
        return (disp_tab_point_7seg[pcDisp - disp_tab]);
}

char ASCII2Disp_R(char* buff) {
    char* pcDisp;
    pcDisp = (char*)strchr(disp_tab, *buff);
    if (pcDisp == NULL)
        return 0x0;
    else
        return (disp_tab_7seg_R[pcDisp - disp_tab]);
}

char ASCII2PointDisp_R(char* buff) {
    char* pcDisp;
    pcDisp = (char*)strchr(disp_tab, *buff);
    if (pcDisp == NULL)
        return 0x0;
    else
        return (disp_tab_point_7seg_R[pcDisp - disp_tab]);
}

void Delay(uint32_t value) {
    uint32_t ui32Loop;
    for (ui32Loop = 0; ui32Loop < value; ui32Loop++) {
    };
}

void UARTStringPut(const char* cMessage) {
    while (*cMessage != '\0')
        UARTCharPut(UART0_BASE, *(cMessage++));
    // Delay(500);
}

void S800_UART_Init(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);  // Enable PortA
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA))
        ;  // Wait for the GPIO moduleA ready

    GPIOPinConfigure(GPIO_PA0_U0RX);  // Set GPIO A0 and A1 as UART pins.
    GPIOPinConfigure(GPIO_PA1_U0TX);

    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    // Configure the UART for 115,200, 8-N-1 operation.
    UARTConfigSetExpClk(UART0_BASE, ui32SysClock, 115200,
                        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                         UART_CONFIG_PAR_NONE));  // 8数据位，1停止位，无校验位

    // Enable FIFO and set FIFO level to 30 bytes, 7/8 full
    UARTFIFOLevelSet(UART0_BASE, UART_FIFO_TX1_8, UART_FIFO_RX7_8);
    // 启用UART中断
    UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);
    IntEnable(INT_UART0);
    // 启用UART
    UARTEnable(UART0_BASE);

    UARTStringPut((uint8_t*)"\r\nWelcome to ddd's course project!\r\n");
}
void S800_GPIO_Init(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);  // Enable PortF
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF))
        ;  // Wait for the GPIO moduleF ready
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);  // Enable PortJ
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOJ))
        ;  // Wait for the GPIO moduleJ ready
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);  // Enable PortN
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPION))
        ;  // Wait for the GPIO moduleN ready

    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_0);  // Set PF0 as Output
                                                         // pin
    GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE,
                          GPIO_PIN_0 | GPIO_PIN_1);  // Set PN0 and PN1 as
                                                     // Output pin

    GPIOPinTypeGPIOInput(
        GPIO_PORTJ_BASE,
        GPIO_PIN_0 | GPIO_PIN_1);  // Set the PJ0,PJ1 as input pin
    GPIOPadConfigSet(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1,
                     GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
}

void S800_I2C0_Init(void) {
    uint8_t result;
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    GPIOPinConfigure(GPIO_PB2_I2C0SCL);
    GPIOPinConfigure(GPIO_PB3_I2C0SDA);
    GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
    GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);

    I2CMasterInitExpClk(I2C0_BASE, ui32SysClock, true);  // config I2C0 400k
    I2CMasterEnable(I2C0_BASE);

    result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_CONFIG_PORT0,
                            0x0ff);  // config port 0 as input
    result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_CONFIG_PORT1,
                            0x0);  // config port 1 as output
    result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_CONFIG_PORT2,
                            0x0);  // config port 2 as output

    result = I2C0_WriteByte(PCA9557_I2CADDR, PCA9557_CONFIG,
                            0x00);  // config port as output
    result = I2C0_WriteByte(PCA9557_I2CADDR, PCA9557_OUTPUT,
                            0x0ff);  // turn off the LED1-8
}

void PWM_Init(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);  // Enable PortK
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOK))
        ;
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);  // Enable PWM0
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_PWM0))
        ;

    // Set PK5 as PWM0 output
    GPIOPinTypePWM(GPIO_PORTK_BASE, GPIO_PIN_5);
    GPIOPinConfigure(GPIO_PK5_M0PWM7);

    // 配置PWM模块的时钟源和时钟分频
    ui32PWMClock = ui32SysClock / 64;
    PWMClockSet(PWM0_BASE, PWM_SYSCLK_DIV_64);

    // 配置PWM模块的输出信号
    PWMGenConfigure(PWM0_BASE, PWM_GEN_3,
                    PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenEnable(PWM0_BASE, PWM_GEN_3);
    // PWMOutputState(PWM0_BASE, PWM_OUT_7_BIT, true);  // 开启蜂鸣器
}

void FLASH_Init(void) {
    // Enable flash module
}

void MY_Init(void) {
    sprintf(disp_buff_static, "%08s", "11451419");  // 显示的初始内容, 学号后8位

    int count = 16;
    cnt = 0;
    while (count--) {  // 流水线显示2轮
        if (cnt >= 0x8) {
            rightshift = 0x01;
            cnt = 0;
        }
        result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1,
                                ASCII2Disp(disp_buff_static + cnt));
        result =
            I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, rightshift);
        result = I2C0_WriteByte(PCA9557_I2CADDR, PCA9557_OUTPUT, ~rightshift);
        Delay(1200000);
        cnt++;
        rightshift = rightshift << 1;
    }
    result =
        I2C0_WriteByte(PCA9557_I2CADDR, PCA9557_OUTPUT, 0xFF);  // 关闭所有LED

    // 从 Flash 中读取时间
    if (ReadFromFlash(&year, &month, &day, &ui32Time)) {
        // 从 Flash 中读取成功
        update_date_disp();
    } else {
        ui32Time = 2885900;  // 08:00:59:00
        year = 2023;
        month = 6;
        day = 11;  // 2023.06.11
        update_date_disp();
    }
    /*
    ui32Time = 2885900;  // 08:00:59:00
    year = 2023;
    month = 6;
    day = 11;  // 2023.06.11
    update_date_disp();
    */
    ui32Alarm = 9 * 60 * 60 * 100;  // 闹钟默认 09:00:00:00
    update_alarm_disp();
    ui32Stopwatch = 3000;  // 倒计时默认30s
    ui32Stopwatch_static = ui32Stopwatch;
    stopwatchEnable = false;
    ui32RunTime = 0;  // 运行时间从0开始
    reverse = 0;      // 取消翻转显示

    disp_mode = 0;
    SW_n = 0xFF;
    prev_SW_n = 0xFF;

    // 显示初始图案，“交大金课”字样
    char buffer[128];
    int index = 0;

    int row, col;
    // UARTStringPut("\n");
    for (row = 0; row < 16; row++) {
        index = 0;
        for (col = 0; col < 64; col++) {
            // 将当前像素添加到缓冲区中
            buffer[index++] = pattern[row][col];
        }
        // 添加换行符到缓冲区中
        buffer[index++] = '\r';
        buffer[index++] = '\n';
        buffer[++index] = '\0';
        // 将缓冲区中的内容发送到UART中
        UARTStringPut((uint8_t*)buffer);
    }
    buffer[index++] = '\0';
    // UARTStringPut((uint8_t*)buffer);
}

uint8_t I2C0_WriteByte(uint8_t DevAddr, uint8_t RegAddr, uint8_t WriteData) {
    uint8_t rop;
    while (I2CMasterBusy(I2C0_BASE)) {
    };
    I2CMasterSlaveAddrSet(I2C0_BASE, DevAddr, false);
    I2CMasterDataPut(I2C0_BASE, RegAddr);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    while (I2CMasterBusy(I2C0_BASE)) {
    };
    rop = (uint8_t)I2CMasterErr(I2C0_BASE);

    I2CMasterDataPut(I2C0_BASE, WriteData);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
    while (I2CMasterBusy(I2C0_BASE)) {
    };

    rop = (uint8_t)I2CMasterErr(I2C0_BASE);
    return rop;
}

uint8_t I2C0_ReadByte(uint8_t DevAddr, uint8_t RegAddr) {
    uint8_t value, rop;
    while (I2CMasterBusy(I2C0_BASE)) {
    };
    I2CMasterSlaveAddrSet(I2C0_BASE, DevAddr, false);
    I2CMasterDataPut(I2C0_BASE, RegAddr);
    //	I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_SEND);
    while (I2CMasterBusBusy(I2C0_BASE))
        ;
    rop = (uint8_t)I2CMasterErr(I2C0_BASE);
    Delay(500);
    // receive data
    I2CMasterSlaveAddrSet(I2C0_BASE, DevAddr, true);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);
    while (I2CMasterBusBusy(I2C0_BASE))
        ;
    value = I2CMasterDataGet(I2C0_BASE);
    Delay(500);
    return value;
}

// systick 中断，用于计时
void SysTick_Handler(void) {
    if (systick_1ms_couter != 0)
        systick_1ms_couter--;
    else {
        systick_1ms_couter = SYSTICK_FREQUENCY / 1000;
        systick_1ms_status = 1;
        updateRuntime();
    }

    if (systick_2ms_couter != 0)
        systick_2ms_couter--;
    else {
        systick_2ms_couter = SYSTICK_FREQUENCY / 500;
        systick_2ms_status = 1;
    }

    if (systick_100ms_couter != 0)
        systick_100ms_couter--;
    else {
        systick_100ms_couter = SYSTICK_FREQUENCY / 10;
        systick_100ms_status = 1;
    }

    if (systick_10ms_couter != 0)
        systick_10ms_couter--;
    else {
        systick_10ms_couter = SYSTICK_FREQUENCY / 100;
        systick_10ms_status = 1;
        updateTime();
        updateStopwatch();
    }
    if (systick_500ms_couter != 0)
        systick_500ms_couter--;
    else {
        systick_500ms_couter = SYSTICK_FREQUENCY / 2;
        systick_500ms_status = 1;
        half_sec = !half_sec;
    }
}

bool is_command_arg_empty(int arg_index) {
    return command_upper[arg_index][0] == '\0';
}

// 检查时间参数格式是否合法
bool is_time_arg_valid(int arg_index) {
    if (is_command_arg_empty(arg_index)) {
        return false;
    }
    if (!isdigit(command_upper[arg_index][0]) ||
        command_upper[arg_index][0] > '2') {
        return false;
    } else if (!isdigit(command_upper[arg_index][1]) ||
               command_upper[arg_index][0] == '2' &&
                   command_upper[arg_index][1] > '3') {
        return false;
    }
    if (command_upper[arg_index][2] != ':') {
        return false;
    }
    if (!isdigit(command_upper[arg_index][3]) ||
        command_upper[arg_index][3] > '5') {
        return false;
    }
    if (!isdigit(command_upper[arg_index][4])) {
        return false;
    }
    if (command_upper[arg_index][5] != ':') {
        return false;
    }
    if (!isdigit(command_upper[arg_index][6]) ||
        command_upper[arg_index][6] > '5') {
        return false;
    }
    if (!isdigit(command_upper[arg_index][7])) {
        return false;
    }
    if (command_upper[arg_index][8] != '\0') {
        return false;
    }
    return true;
}

// 检查日期是否合法
bool checkDate(uint32_t year, uint8_t month, uint8_t day) {
    if (year < 1 || year > 9999) {
        return false;
    }
    if (month < 1 || month > 12) {
        return false;
    }
    if (day < 1 || day > 31) {
        return false;
    }
    if ((month == 4 || month == 6 || month == 9 || month == 11) && day > 30) {
        return false;
    }
    if (month == 2) {
        if (day > 29) {
            return false;
        }
        if (day == 29 &&
            (year % 4 != 0 || (year % 100 == 0 && year % 400 != 0))) {
            return false;
        }
    }
    return true;
}

// 将日期加一天
void addOneDay(uint32_t* year, uint8_t* month, uint8_t* day) {
    if (*day < 28) {
        (*day)++;
    } else if (*day == 28) {
        if (*month == 2) {
            if ((*year % 4 == 0 && *year % 100 != 0) || *year % 400 == 0) {
                (*day)++;
            } else {
                *day = 1;
                (*month)++;
            }
        } else {
            (*day)++;
        }
    } else if (*day == 29) {
        if (*month == 2) {
            *day = 1;
            (*month)++;
        } else {
            (*day)++;
        }
    } else if (*day == 30) {
        if (month == 4 || month == 6 || month == 9 || month == 11) {
            *day = 1;
            (*month)++;
        } else {
            (*day)++;
        }
    } else if (*day == 31) {
        if (*month == 12) {
            *day = 1;
            *month = 1;
            (*year)++;
            if (*year == 10000) {
                *year = 1;
            }
        } else {
            *day = 1;
            (*month)++;
        }
    }
}

// 将日期加一月
void addOneMonth(uint32_t* year, uint8_t* month, uint8_t* day) {
    if (*month == 12) {
        *month = 1;
        (*year)++;
        if (*year == 10000) {
            *year = 1;
        }
    } else {
        (*month)++;
    }
    if (*day > 28) {
        if (*month == 2) {
            if ((*year % 4 == 0 && *year % 100 != 0) || *year % 400 == 0) {
                if (*day > 29) {
                    *day = 29;
                }
            } else {
                if (*day > 28) {
                    *day = 28;
                }
            }
        } else if ((*month == 4 || *month == 6 || *month == 9 ||
                    *month == 11) &&
                   *day > 30) {
            *day = 30;
        }
    }
}

// 检查日期参数格式是否合法
bool is_date_arg_valid(int arg_index) {
    if (is_command_arg_empty(arg_index)) {
        return false;
    }
    if (!isdigit(command_upper[arg_index][0]))
        return false;
    if (!isdigit(command_upper[arg_index][1]))
        return false;
    if (!isdigit(command_upper[arg_index][2]))
        return false;
    if (!isdigit(command_upper[arg_index][3]))
        return false;
    if (command_upper[arg_index][4] != '.')
        return false;
    if (!isdigit(command_upper[arg_index][5]) ||
        command_upper[arg_index][5] > '1')
        return false;
    if (!isdigit(command_upper[arg_index][6]) ||
        command_upper[arg_index][5] == '1' && command_upper[arg_index][6] > '2')
        return false;
    if (command_upper[arg_index][7] != '.')
        return false;
    if (!isdigit(command_upper[arg_index][8]) ||
        command_upper[arg_index][8] > '3')
        return false;
    if (!isdigit(command_upper[arg_index][9]) ||
        command_upper[arg_index][8] == '3' && command_upper[arg_index][9] > '1')
        return false;
    if (command_upper[arg_index][10] != '\0')
        return false;

    int check_year, check_month, check_day;
    check_year = (command_upper[arg_index][0] - '0') * 1000 +
                 (command_upper[arg_index][1] - '0') * 100 +
                 (command_upper[arg_index][2] - '0') * 10 +
                 command_upper[arg_index][3] - '0';
    check_month = (command_upper[arg_index][5] - '0') * 10 +
                  command_upper[arg_index][6] - '0';
    check_day = (command_upper[arg_index][8] - '0') * 10 +
                command_upper[arg_index][9] - '0';
    // return true;
    return checkDate(check_year, check_month, check_day);
}

// 指令处理
void UART0_Handler(void) {
    if (uartActivate) {  // 帮助信息未处理完毕
        return;
    }
    arg_index = 0;
    arg_length = 0;
    needed_arg_count = 0;
    is_command_prefix_valid = false;
    help_index = 0;
    int i, j;
    for (i = 0; i < MAX_COMMAND_ARGS; i++) {
        is_command_arg_valids[i] = false;
    }
    bool is_space = false;

    // Clear command
    memset(command, 0, sizeof(command));
    memset(command_upper, 0, sizeof(command_upper));
    // memset(RxBuf, '\0', sizeof(RxBuf));

    // Read command from UART
    i = 0;
    char* p = RxBuf;
    while (UARTCharsAvail(UART0_BASE)) {
        *p = UARTCharGetNonBlocking(UART0_BASE);
        char c = *p;
        if (c == '\0') {
            break;
        } else if (c == ' ') {  // 可处理连续空格
            if (!is_space) {
                arg_index++;
                arg_length = 0;
                is_space = true;
            }
        } else {
            is_space = false;
            if (arg_index < MAX_COMMAND_ARGS &&
                arg_length < MAX_COMMAND_ARG_LENGTH) {
                command[arg_index][arg_length++] = c;
            }
        }
        p++;
        Delay(120);
    }

    // Convert all alpha to uppercase
    for (i = 0; i < arg_index + 1; i++) {
        for (j = 0; j < MAX_COMMAND_ARG_LENGTH; j++) {
            command_upper[i][j] = toupper(command[i][j]);
        }
    }

    // Check command
    if (strcmp(command_upper[0], "?") == 0) {
        is_command_prefix_valid = true;
        needed_arg_count = 0;
        help_index = 0;
        helpEnable = true;
        return;
    } else if (strcmp(command_upper[0], "INIT") == 0) {
        is_command_prefix_valid = true;
        needed_arg_count = 1;
        help_index = 1;
        if (arg_index >= 1) {
            if (strcmp(command_upper[1], "CLOCK") == 0) {
                command_mode = 1;
                is_command_arg_valids[1] = true;
            } else if (strcmp(command_upper[1], "?") == 0) {
                helpEnable = true;
                return;
            }
        }
    } else if (strcmp(command_upper[0], "SET") == 0) {
        is_command_prefix_valid = true;
        needed_arg_count = 2;
        help_index = 2;
        if (strcmp(command_upper[1], "TIME") == 0) {
            is_command_arg_valids[1] = true;
            if (is_time_arg_valid(2)) {
                command_mode = 2;
                is_command_arg_valids[2] = true;
                strcpy((char*)set_arg_2, command[2]);
            }
        } else if (strcmp(command_upper[1], "DATE") == 0) {
            is_command_arg_valids[1] = true;
            if (is_date_arg_valid(2)) {
                command_mode = 3;
                is_command_arg_valids[2] = true;
                strcpy((char*)set_arg_2, command[2]);
            }
        } else if (strcmp(command_upper[1], "ALARM") == 0) {
            is_command_arg_valids[1] = true;
            if (is_time_arg_valid(2)) {
                command_mode = 4;
                is_command_arg_valids[2] = true;
                strcpy((char*)set_arg_2, command[2]);
            }
        } else if (strcmp(command_upper[1], "STWATCH") == 0) {
            is_command_arg_valids[1] = true;
            if (is_time_arg_valid(2)) {
                command_mode = 5;
                is_command_arg_valids[2] = true;
                strcpy((char*)set_arg_2, command[2]);
            }
        } else if (strcmp(command_upper[1], "?") == 0) {
            helpEnable = true;
            return;
        }
    } else if (strcmp(command_upper[0], "GET") == 0) {
        is_command_prefix_valid = true;
        needed_arg_count = 1;
        help_index = 3;
        if (arg_index == 1) {
            if (strcmp(command_upper[1], "RUNTIME") == 0) {
                command_mode = 6;
                is_command_arg_valids[1] = true;
            } else if (strcmp(command_upper[1], "TIME") == 0) {
                command_mode = 7;
                is_command_arg_valids[1] = true;
            } else if (strcmp(command_upper[1], "DATE") == 0) {
                command_mode = 8;
                is_command_arg_valids[1] = true;
            } else if (strcmp(command_upper[1], "ALARM") == 0) {
                command_mode = 9;
                is_command_arg_valids[1] = true;
            } else if (strcmp(command_upper[1], "STWATCH") == 0) {
                command_mode = 10;
                is_command_arg_valids[1] = true;
            } else if (strcmp(command_upper[1], "?") == 0) {
                helpEnable = true;
                return;
            }
        }
    } else if (strcmp(command_upper[0], "RUN") == 0) {
        is_command_prefix_valid = true;
        needed_arg_count = 1;
        help_index = 4;
        if (arg_index == 1) {
            if (strcmp(command_upper[1], "RUNTIME") == 0) {
                command_mode = 11;
                is_command_arg_valids[1] = true;
            } else if (strcmp(command_upper[1], "TIME") == 0) {
                command_mode = 12;
                is_command_arg_valids[1] = true;
            } else if (strcmp(command_upper[1], "DATE") == 0) {
                command_mode = 13;
                is_command_arg_valids[1] = true;
            } else if (strcmp(command_upper[1], "ALARM") == 0) {
                command_mode = 14;
                is_command_arg_valids[1] = true;
            } else if (strcmp(command_upper[1], "STWATCH") == 0) {
                command_mode = 15;
                is_command_arg_valids[1] = true;
            } else if (strcmp(command_upper[1], "?") == 0) {
                helpEnable = true;
                return;
            }
        }
    } else if (strcmp(command_upper[0], "REVERSE") == 0) {
        is_command_prefix_valid = true;
        needed_arg_count = 0;
        help_index = 5;
        if (arg_index >= 1) {
            if (strcmp(command_upper[1], "?") == 0) {
                helpEnable = true;
                return;
            }
        } else {
            command_mode = 16;
        }
    } else if (strcmp(command_upper[0], "SAVE") == 0) {
        is_command_prefix_valid = true;
        needed_arg_count = 0;
        help_index = 6;
        if (arg_index >= 1) {
            if (strcmp(command_upper[1], "?") == 0) {
                helpEnable = true;
                return;
            }
        } else {
            command_mode = 17;
        }
    }
    is_command_arg_valids[0] = is_command_prefix_valid;
    uartActivate = true;
}

void displayTime(void) {
    if (!reverse) {
        if (half_sec) {
            tmp_bits_select = bits_select[0];
        } else {
            tmp_bits_select = bits_select[bits_selected];  // for flash 2 bits
        }
        if (systick_2ms_couter > 2 &&
            systick_2ms_couter < 19) {  // adjust duty cycle, max=20
            if (cnt == 1 || cnt == 3 || cnt == 5) {  // write port 1
                result = I2C0_WriteByte(
                    TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1,
                    ASCII2PointDisp(disp_buff_time + cnt));  // with point
            } else {
                result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1,
                                        ASCII2Disp(disp_buff_time + cnt));
            }
            result =
                I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2,
                               rightshift & tmp_bits_select);  // write port 2
        } else {
            result = I2C0_WriteByte(
                TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1,
                ASCII2Disp(disp_buff_time + cnt));  // write port 1
            result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2,
                                    0x00);  // display nothing
        }
    } else {  // reverse
        if (half_sec) {
            tmp_bits_select = bits_select_R[0];
        } else {
            tmp_bits_select = bits_select_R[bits_selected];
        }
        if (systick_2ms_couter > 2 &&
            systick_2ms_couter < 19) {  // adjust duty cycle, max=20
            if (cnt == 1 || cnt == 3 || cnt == 5) {  // write port 1
                result = I2C0_WriteByte(
                    TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1,
                    ASCII2PointDisp_R(disp_buff_time + 7 - cnt));  // with point
            } else {
                result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1,
                                        ASCII2Disp_R(disp_buff_time + 7 - cnt));
            }
            result =
                I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2,
                               rightshift & tmp_bits_select);  // write port 2
        } else {
            result = I2C0_WriteByte(
                TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1,
                ASCII2Disp_R(disp_buff_time + 7 - cnt));  // write port 1
            result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2,
                                    0x00);  // display nothing
        }
    }
}

void displayDate(void) {
    if (!reverse) {
        if (half_sec) {
            tmp_bits_select = bits_select[0];
        } else {
            tmp_bits_select = bits_select[bits_selected];
        }
        if (systick_2ms_couter > 2 &&
            systick_2ms_couter < 19) {   // adjust duty cycle, max=20
            if (cnt == 3 || cnt == 5) {  // write port 1
                result = I2C0_WriteByte(
                    TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1,
                    ASCII2PointDisp(disp_buff_date + cnt));  // with point
            } else {
                result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1,
                                        ASCII2Disp(disp_buff_date + cnt));
            }
            result =
                I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2,
                               rightshift & tmp_bits_select);  // write port 2
        } else {
            result = I2C0_WriteByte(
                TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1,
                ASCII2Disp(disp_buff_date + cnt));  // write port 1
            result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2,
                                    0x00);  // display nothing
        }
    } else {  // reverse
        if (half_sec) {
            tmp_bits_select = bits_select_R[0];
        } else {
            tmp_bits_select = bits_select_R[bits_selected];
        }
        if (systick_2ms_couter > 2 &&
            systick_2ms_couter < 19) {   // adjust duty cycle, max=20
            if (cnt == 1 || cnt == 3) {  // write port 1
                result = I2C0_WriteByte(
                    TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1,
                    ASCII2PointDisp_R(disp_buff_date + 7 - cnt));  // with point
            } else {
                result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1,
                                        ASCII2Disp_R(disp_buff_date + 7 - cnt));
            }
            result =
                I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2,
                               rightshift & tmp_bits_select);  // write port 2
        } else {
            result = I2C0_WriteByte(
                TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1,
                ASCII2Disp_R(disp_buff_date + 7 - cnt));  // write port 1
            result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2,
                                    0x00);  // display nothing
        }
    }
}

void displayAlarm(void) {
    if (!reverse) {
        if (half_sec) {
            tmp_bits_select = bits_select[0];
        } else {
            tmp_bits_select = bits_select[bits_selected];
        }
        if (systick_2ms_couter > 2 &&
            systick_2ms_couter < 19) {  // adjust duty cycle, max=20
            if (cnt == 1 || cnt == 3 || cnt == 5) {  // write port 1
                result = I2C0_WriteByte(
                    TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1,
                    ASCII2PointDisp(disp_buff_alarm + cnt));  // with point
            } else {
                result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1,
                                        ASCII2Disp(disp_buff_alarm + cnt));
            }
            result =
                I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2,
                               rightshift & tmp_bits_select);  // write port 2
        } else {
            result = I2C0_WriteByte(
                TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1,
                ASCII2Disp(disp_buff_alarm + cnt));  // write port 1
            result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2,
                                    0x00);  // display nothing
        }
    } else {  // reverse
        if (half_sec) {
            tmp_bits_select = bits_select_R[0];
        } else {
            tmp_bits_select = bits_select_R[bits_selected];
        }
        if (systick_2ms_couter > 2 &&
            systick_2ms_couter < 19) {  // adjust duty cycle, max=20
            if (cnt == 1 || cnt == 3 || cnt == 5) {  // write port 1
                result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1,
                                        ASCII2PointDisp_R(disp_buff_alarm + 7 -
                                                          cnt));  // with point
            } else {
                result =
                    I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1,
                                   ASCII2Disp_R(disp_buff_alarm + 7 - cnt));
            }
            result =
                I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2,
                               rightshift & tmp_bits_select);  // write port 2
        } else {
            result = I2C0_WriteByte(
                TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1,
                ASCII2Disp_R(disp_buff_alarm + 7 - cnt));  // write port 1
            result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2,
                                    0x00);  // display nothing
        }
    }
}

void displayStopwatch(void) {
    if (!reverse) {
        if (half_sec) {
            tmp_bits_select = bits_select[0];
        } else {
            tmp_bits_select = bits_select[bits_selected];
        }
        if (systick_2ms_couter > 2 &&
            systick_2ms_couter < 19) {  // adjust duty cycle, max=20
            if (cnt == 1 || cnt == 3 || cnt == 5) {  // write port 1
                result = I2C0_WriteByte(
                    TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1,
                    ASCII2PointDisp(disp_buff_stopwatch + cnt));  // with point
            } else {
                result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1,
                                        ASCII2Disp(disp_buff_stopwatch + cnt));
            }
            result =
                I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2,
                               rightshift & tmp_bits_select);  // write port 2
        } else {
            result = I2C0_WriteByte(
                TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1,
                ASCII2Disp(disp_buff_stopwatch + cnt));  // write port 1
            result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2,
                                    0x00);  // display nothing
        }
    } else {  // reverse
        if (half_sec) {
            tmp_bits_select = bits_select_R[0];
        } else {
            tmp_bits_select = bits_select_R[bits_selected];
        }
        if (systick_2ms_couter > 2 &&
            systick_2ms_couter < 19) {  // adjust duty cycle, max=20
            if (cnt == 1 || cnt == 3 || cnt == 5) {  // write port 1
                result =
                    I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1,
                                   ASCII2PointDisp_R(disp_buff_stopwatch + 7 -
                                                     cnt));  // with point
            } else {
                result =
                    I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1,
                                   ASCII2Disp_R(disp_buff_stopwatch + 7 - cnt));
            }
            result =
                I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2,
                               rightshift & tmp_bits_select);  // write port 2
        } else {
            result = I2C0_WriteByte(
                TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1,
                ASCII2Disp_R(disp_buff_stopwatch + 7 - cnt));  // write port 1
            result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2,
                                    0x00);  // display nothing
        }
    }
}

void displayRuntime(void) {
    if (!reverse) {
        if (half_sec) {
            tmp_bits_select = bits_select[0];
        } else {
            tmp_bits_select = bits_select[bits_selected];
        }
        if (systick_2ms_couter > 2 &&
            systick_2ms_couter < 19) {  // adjust duty cycle, max=20
            if (cnt == 1 || cnt == 3 || cnt == 5) {  // write port 1
                result = I2C0_WriteByte(
                    TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1,
                    ASCII2PointDisp(disp_buff_runtime + cnt));  // with point
            } else {
                result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1,
                                        ASCII2Disp(disp_buff_runtime + cnt));
            }

            result =
                I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2,
                               rightshift & tmp_bits_select);  // write port 2
        } else {
            result = I2C0_WriteByte(
                TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1,
                ASCII2Disp(disp_buff_runtime + cnt));  // write port 1
            result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2,
                                    0x00);  // display nothing
        }
    } else {  // reverse
        if (half_sec) {
            tmp_bits_select = bits_select_R[0];
        } else {
            tmp_bits_select = bits_select_R[bits_selected];
        }
        if (systick_2ms_couter > 2 &&
            systick_2ms_couter < 19) {  // adjust duty cycle, max=20
            if (cnt == 1 || cnt == 3 || cnt == 5) {  // write port 1
                result =
                    I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1,
                                   ASCII2PointDisp_R(disp_buff_runtime + 7 -
                                                     cnt));  // with point
            } else {
                result =
                    I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1,
                                   ASCII2Disp_R(disp_buff_runtime + 7 - cnt));
            }
            result =
                I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2,
                               rightshift & tmp_bits_select);  // write port 2
        } else {
            result = I2C0_WriteByte(
                TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1,
                ASCII2Disp_R(disp_buff_runtime + 7 - cnt));  // write port 1
            result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2,
                                    0x00);  // display nothing
        }
    }
}

void updateTime(void) {
    // 更新时间，设置时间时，可乘出该数值
    if (!freeze || disp_mode != 1) {  // freeze时，不更新时间
        ui32Time += 1;
        if (ui32Time == ui32Alarm) {  // 到达闹钟时间, 播放音乐
            note_index = 0;
            note_delay = music_time[note_index] / 100;
        }
    }

    if (ui32Time >= 24 * 60 * 60 * 100) {  // 24小时后清零
        ui32Time -= 24 * 60 * 60 * 100;
        addOneDay(&year, &month, &day);  // 日期加一天
        update_date_disp();
    }

    // 更新数码管的显示
    uint32_t ui32Hour = ui32Time / (100 * 60 * 60);
    uint32_t ui32Minute = (ui32Time / (100 * 60)) % 60;
    uint32_t ui32Second = (ui32Time / 100) % 60;
    uint32_t ui32Centisecond = ui32Time % 100;

    uint32_t ui32Hour10 = ui32Hour / 10;
    uint32_t ui32Hour1 = ui32Hour % 10;
    char cHour10 = ui32Hour10 + '0';
    char cHour1 = ui32Hour1 + '0';

    uint32_t ui32Minute10 = ui32Minute / 10;
    uint32_t ui32Minute1 = ui32Minute % 10;
    char cMinute10 = ui32Minute10 + '0';
    char cMinute1 = ui32Minute1 + '0';

    uint32_t ui32Second10 = ui32Second / 10;
    uint32_t ui32Second1 = ui32Second % 10;
    char cSecond10 = ui32Second10 + '0';
    char cSecond1 = ui32Second1 + '0';

    uint32_t ui32Centisecond10 = ui32Centisecond / 10;
    uint32_t ui32Centisecond1 = ui32Centisecond % 10;
    char cCentisecond10 = ui32Centisecond10 + '0';
    char cCentisecond1 = ui32Centisecond1 + '0';

    disp_buff_time[0] = cHour10;
    disp_buff_time[1] = cHour1;
    disp_buff_time[2] = cMinute10;
    disp_buff_time[3] = cMinute1;
    disp_buff_time[4] = cSecond10;
    disp_buff_time[5] = cSecond1;
    disp_buff_time[6] = cCentisecond10;
    disp_buff_time[7] = cCentisecond1;
}

// 更新日期显示
void update_date_disp(void) {
    disp_buff_date[0] = year / 1000 + '0';
    disp_buff_date[1] = year / 100 % 10 + '0';
    disp_buff_date[2] = year / 10 % 10 + '0';
    disp_buff_date[3] = year % 10 + '0';
    disp_buff_date[4] = month / 10 + '0';
    disp_buff_date[5] = month % 10 + '0';
    disp_buff_date[6] = day / 10 + '0';
    disp_buff_date[7] = day % 10 + '0';
}

// 更新闹钟显示
void update_alarm_disp(void) {
    uint32_t ui32Hour = ui32Alarm / (100 * 60 * 60);
    uint32_t ui32Minute = (ui32Alarm / (100 * 60)) % 60;
    uint32_t ui32Second = (ui32Alarm / 100) % 60;
    uint32_t ui32Centisecond = ui32Alarm % 100;

    uint32_t ui32Hour10 = ui32Hour / 10;
    uint32_t ui32Hour1 = ui32Hour % 10;
    char cHour10 = ui32Hour10 + '0';
    char cHour1 = ui32Hour1 + '0';

    uint32_t ui32Minute10 = ui32Minute / 10;
    uint32_t ui32Minute1 = ui32Minute % 10;
    char cMinute10 = ui32Minute10 + '0';
    char cMinute1 = ui32Minute1 + '0';

    uint32_t ui32Second10 = ui32Second / 10;
    uint32_t ui32Second1 = ui32Second % 10;
    char cSecond10 = ui32Second10 + '0';
    char cSecond1 = ui32Second1 + '0';

    uint32_t ui32Centisecond10 = ui32Centisecond / 10;
    uint32_t ui32Centisecond1 = ui32Centisecond % 10;
    char cCentisecond10 = ui32Centisecond10 + '0';
    char cCentisecond1 = ui32Centisecond1 + '0';

    disp_buff_alarm[0] = cHour10;
    disp_buff_alarm[1] = cHour1;
    disp_buff_alarm[2] = cMinute10;
    disp_buff_alarm[3] = cMinute1;
    disp_buff_alarm[4] = cSecond10;
    disp_buff_alarm[5] = cSecond1;
    disp_buff_alarm[6] = cCentisecond10;
    disp_buff_alarm[7] = cCentisecond1;
}

void updateStopwatch(void) {
    if (!freeze || disp_mode != 4) {  // freeze时，不更新时间
        if (stopwatchEnable && ui32Stopwatch > 0) {
            ui32Stopwatch -= 1;
        } else {  // ui32Stopwatch == 0
            stopwatchEnable = false;
        }
    }

    if (ui32Stopwatch == 1) {  // 触发音乐播放，确保只执行一次
        note_index = 0;
        note_delay = music_time[note_index] / 100;
    }
    uint32_t ui32Hour = ui32Stopwatch / (100 * 60 * 60);
    uint32_t ui32Minute = (ui32Stopwatch / (100 * 60)) % 60;
    uint32_t ui32Second = (ui32Stopwatch / 100) % 60;
    uint32_t ui32Centisecond = ui32Stopwatch % 100;

    uint32_t ui32Hour10 = ui32Hour / 10;
    uint32_t ui32Hour1 = ui32Hour % 10;
    char cHour10 = ui32Hour10 + '0';
    char cHour1 = ui32Hour1 + '0';

    uint32_t ui32Minute10 = ui32Minute / 10;
    uint32_t ui32Minute1 = ui32Minute % 10;
    char cMinute10 = ui32Minute10 + '0';
    char cMinute1 = ui32Minute1 + '0';

    uint32_t ui32Second10 = ui32Second / 10;
    uint32_t ui32Second1 = ui32Second % 10;
    char cSecond10 = ui32Second10 + '0';
    char cSecond1 = ui32Second1 + '0';

    uint32_t ui32Centisecond10 = ui32Centisecond / 10;
    uint32_t ui32Centisecond1 = ui32Centisecond % 10;
    char cCentisecond10 = ui32Centisecond10 + '0';
    char cCentisecond1 = ui32Centisecond1 + '0';

    disp_buff_stopwatch[0] = cHour10;
    disp_buff_stopwatch[1] = cHour1;
    disp_buff_stopwatch[2] = cMinute10;
    disp_buff_stopwatch[3] = cMinute1;
    disp_buff_stopwatch[4] = cSecond10;
    disp_buff_stopwatch[5] = cSecond1;
    disp_buff_stopwatch[6] = cCentisecond10;
    disp_buff_stopwatch[7] = cCentisecond1;
}

void updateRuntime(void) {
    if (!freeze || disp_mode != 0) {  // freeze时，不更新时间
        ui32RunTime += 1;             // update every 1ms
    }

    uint32_t ui32Hour = ui32RunTime / (1000 * 60 * 60);
    uint32_t ui32Minute = (ui32RunTime / (1000 * 60)) % 60;
    uint32_t ui32Second = (ui32RunTime / 1000) % 60;
    uint32_t ui32Millisecond = ui32RunTime % 1000;

    uint32_t ui32Hour10 = ui32Hour / 10;
    uint32_t ui32Hour1 = ui32Hour % 10;
    char cHour10 = ui32Hour10 + '0';
    char cHour1 = ui32Hour1 + '0';

    uint32_t ui32Minute10 = ui32Minute / 10;
    uint32_t ui32Minute1 = ui32Minute % 10;
    char cMinute10 = ui32Minute10 + '0';
    char cMinute1 = ui32Minute1 + '0';

    uint32_t ui32Second10 = ui32Second / 10;
    uint32_t ui32Second1 = ui32Second % 10;
    char cSecond10 = ui32Second10 + '0';
    char cSecond1 = ui32Second1 + '0';

    uint32_t ui32Millisecond100 = ui32Millisecond / 100;
    uint32_t ui32Millisecond10 = (ui32Millisecond / 10) % 10;
    uint32_t ui32Millisecond1 = ui32Millisecond % 10;
    char cMillisecond100 = ui32Millisecond100 + '0';
    char cMillisecond10 = ui32Millisecond10 + '0';
    char cMillisecond1 = ui32Millisecond1 + '0';

    disp_buff_runtime[0] = cHour10;
    disp_buff_runtime[1] = cHour1;
    disp_buff_runtime[2] = cMinute10;
    disp_buff_runtime[3] = cMinute1;
    disp_buff_runtime[4] = cSecond10;
    disp_buff_runtime[5] = cSecond1;
    disp_buff_runtime[6] = cMillisecond100;
    disp_buff_runtime[7] = cMillisecond10;
}

// 蓝板按键处理
void process_SW(void) {
    if (SW_n == 0xFF) {
        return;
    }
    int i;
    for (i = 1; i < 9; i++) {
        if ((SW_n | SWs[i]) == SWs[i] &&
            (prev_SW_n | SWs[i]) == 0xFF) {  // SW[i] is pressed
            switch (i) {
                case 0:
                    disp_mode = 0;
                    break;
                case 1:
                    // display time
                    if (!freeze) {
                        disp_mode = 1;
                        stopwatchEnable = false;
                    }
                    break;
                case 2:
                    // display date
                    if (!freeze) {
                        disp_mode = 2;
                        stopwatchEnable = false;
                    }
                    break;
                case 3:
                    // display alarm
                    if (!freeze) {
                        disp_mode = 3;
                        stopwatchEnable = false;
                    }
                    break;
                case 4:
                    // run stopwatch
                    freeze = false;
                    bits_selected = 0;
                    stopwatchEnable = true;
                    ui32Stopwatch = ui32Stopwatch_static;  // reset and run
                    disp_mode = 4;
                    break;
                case 5:
                    // freeze to adjust displayment
                    freeze = !freeze;
                    bits_selected = 0;
                    break;
                case 6:
                    // 选取调整的2位数字，从右向左，0为全部显示
                    if (freeze) {
                        if (bits_selected == 4) {
                            bits_selected = 0;
                        } else {
                            bits_selected++;
                        }
                    }
                    break;
                case 7:
                    // 选取的2位数字加1
                    if (freeze) {
                        switch (disp_mode) {
                            case 0:
                                // adjust runtime
                                switch (bits_selected) {
                                    case 0:
                                        break;
                                    case 1:
                                        ui32RunTime += 10;
                                        break;
                                    case 2:
                                        ui32RunTime += 1000;
                                        break;
                                    case 3:
                                        ui32RunTime += 1000 * 60;
                                        break;
                                    case 4:
                                        ui32RunTime += 1000 * 60 * 60;
                                        if (ui32RunTime >= 1000 * 60 * 60 * 24)
                                            ui32RunTime -= 1000 * 60 * 60 * 24;
                                        break;
                                }
                                break;
                            case 1:
                                // adjust time
                                switch (bits_selected) {
                                    case 0:
                                        break;
                                    case 1:
                                        ui32Time += 1;
                                        break;
                                    case 2:
                                        ui32Time += 100;
                                        break;
                                    case 3:
                                        ui32Time += 100 * 60;
                                        break;
                                    case 4:
                                        ui32Time += 100 * 60 * 60;
                                        if (ui32Time >= 1000 * 60 * 60 * 24) {
                                            ui32Time -= 1000 * 60 * 60 * 24;
                                            addOneDay(&year, &month, &day);
                                            update_date_disp();
                                        }
                                        break;
                                }
                                break;
                            case 2:
                                // adjust date
                                switch (bits_selected) {
                                    case 0:
                                        break;
                                    case 1:
                                        addOneDay(&year, &month, &day);
                                        update_date_disp();
                                        break;
                                    case 2:
                                        addOneMonth(&year, &month, &day);
                                        update_date_disp();
                                        break;
                                    case 3:
                                        year += 1;
                                        if (year == 10000) {
                                            year = 0;
                                        }
                                        update_date_disp();
                                        break;
                                    case 4:
                                        year += 100;
                                        if (year >= 10000) {
                                            year -= 9000;
                                        }
                                        update_date_disp();
                                        break;
                                }
                                break;
                            case 3:
                                // adjust alarm
                                switch (bits_selected) {
                                    case 0:
                                        break;
                                    case 1:
                                        ui32Alarm += 1;
                                        update_alarm_disp();
                                        break;
                                    case 2:
                                        ui32Alarm += 100;
                                        update_alarm_disp();
                                        break;
                                    case 3:
                                        ui32Alarm += 100 * 60;
                                        update_alarm_disp();
                                        break;
                                    case 4:
                                        ui32Alarm += 100 * 60 * 60;
                                        if (ui32Alarm >= 100 * 60 * 60 * 24)
                                            ui32Alarm -= 100 * 60 * 60 * 24;
                                        update_alarm_disp();
                                        break;
                                }
                                break;
                            case 4:
                                // adjust stopwatch
                                switch (bits_selected) {
                                    case 0:
                                        break;
                                    case 1:
                                        ui32Stopwatch += 1;
                                        break;
                                    case 2:
                                        ui32Stopwatch += 100;
                                        break;
                                    case 3:
                                        ui32Stopwatch += 100 * 60;
                                        break;
                                    case 4:
                                        ui32Stopwatch += 100 * 60 * 60;
                                        if (ui32Stopwatch >= 100 * 60 * 60 * 24)
                                            ui32Stopwatch -= 100 * 60 * 60 * 24;
                                        break;
                                }
                                break;
                            default:
                                break;
                        }
                        break;
                        case 8:
                            // run alarm
                            if (note_index ==
                                    sizeof(music_note) / sizeof(uint16_t) &&
                                note_delay == 0) {  // music is not playing,
                                                    // play music
                                note_index = 0;
                                note_delay = music_time[note_index] / 100;
                            } else {  // music is playing, stop music
                                note_index =
                                    sizeof(music_note) / sizeof(uint16_t);
                                note_delay = 0;
                                PWMOutputState(PWM0_BASE, PWM_OUT_7_BIT, false);
                            }
                            break;
                    }
            }
            return;
        }
    }
}

// 将 year、month、day 和 ui32Time 写入 Flash
void WriteToFlash(uint32_t year,
                  uint8_t month,
                  uint8_t day,
                  uint32_t ui32Time) {
    uint32_t ui32FlashAddr = FLASH_USER_DATA_ADDR;
    uint32_t ui32Data[2];

    ui32Data[0] = (year << 16) | (month << 8) | day;
    ui32Data[1] = ui32Time;

    FlashErase(ui32FlashAddr);
    FlashProgram(ui32Data, ui32FlashAddr, sizeof(ui32Data));
}

// 从 Flash 中读取 year、month、day 和 ui32Time
bool ReadFromFlash(uint32_t* year,
                   uint8_t* month,
                   uint8_t* day,
                   uint32_t* ui32Time) {
    uint32_t ui32FlashAddr = FLASH_USER_DATA_ADDR;
    uint32_t ui32Data[2];

    // 从 Flash 中读取数据
    ui32Data[0] = HWREG(ui32FlashAddr);
    ui32Data[1] = HWREG(ui32FlashAddr + sizeof(uint32_t));

    if (ui32Data[0] == 0xFFFFFFFF || ui32Data[1] == 0xFFFFFFFF) {
        return false;
    }

    *year = (ui32Data[0] >> 16) & 0xFFFF;
    *month = (ui32Data[0] >> 8) & 0xFF;
    *day = ui32Data[0] & 0xFF;
    *ui32Time = ui32Data[1];

    return true;
}