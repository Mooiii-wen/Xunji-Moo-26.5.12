/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */


/* Includes ------------------------------------------------------------------*/

#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include <stdio.h>
#include <string.h>

/* Private includes ----------------------------------------------------------*/


/* USER CODE BEGIN Includes */

/*
 * delay.h：
 *   提供 AX_Delayms()、AX_Delayus()。
 *
 * key.h：
 *   提供 AX_KEY_Scan()，用于按键启动机器人。
 *
 * line.h：
 *   提供 AX_LINE_GetData()，用于读取四路循迹传感器。
 *
 * motor.h：
 *   提供 AX_MOTOR_A/B/C/D_SetSpeed()，用于控制四个底盘电机。
 *
 * led.h / beep.h：
 *   用于状态提示，例如启动时蜂鸣器响一下、LED 点亮。
 *
 * servo.h：
 *   控制水平舵机和释放舵机。
 *
 * extra_motor.h：
 *   控制 TB6612 上的三个卷线电机。
 */

#include "delay.h"
#include "key.h"
#include "line.h"
#include "motor.h"
#include "led.h"
#include "beep.h"
#include "servo.h"
#include "extra_motor.h"

/*
 * 如果要加入 OpenMV 摄像头，
 * 需要在 CubeMX 里启用 USART1，
 * 并生成 usart.h / usart.c。
 */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/

/* USER CODE BEGIN PTD */
/*
 * AppState_t：
 *   机器人主状态机枚举。
 *
 * 状态机的作用：
 *   把复杂任务拆成多个步骤，每次只执行当前步骤。
 *
 * 当前流程：
 *   等待按键
 *     -> 循迹
 *     -> 找球
 *     -> 对准球
 *     -> 收球
 *     -> 找篮筐
 *     -> 对准篮筐
 *     -> 卷线送球
 *     -> 释放
 *     -> 结束
 */

typedef enum
{
    APP_WAIT = 0,
    APP_LINE,

    APP_FIND_BALL,
    APP_ALIGN_BALL,
    APP_GRAB_BALL,

    APP_FIND_BASKET,
    APP_ALIGN_BASKET,

    APP_WINCH_PAIR,
    APP_RELEASE,
    APP_FINISH
} AppState_t;

/*
 * OpenMV_Data_t：
 *   保存 OpenMV 发给 STM32 的视觉识别数据。
 *
 * ball_ok：
 *   是否识别到球。
 *
 * ball_x / ball_y：
 *   球在图像中的中心坐标。
 *
 * ball_area：
 *   球的像素面积。
 *   面积越大，一般说明球越近。
 *
 * basket_ok：
 *   是否识别到篮筐。
 *
 * basket_x / basket_y：
 *   篮筐在图像中的中心坐标。
 *
 * basket_area：
 *   篮筐的像素面积。
 *
 * last_update_tick：
 *   最近一次收到有效数据的时间。
 *   用来判断 OpenMV 数据是否过期。
 */
typedef struct
{
    uint8_t ball_ok;
    int16_t ball_x;
    int16_t ball_y;
    int32_t ball_area;

    uint8_t basket_ok;
    int16_t basket_x;
    int16_t basket_y;
    int32_t basket_area;

    uint32_t last_update_tick;
} OpenMV_Data_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/*
 * APP_SPEED_MAX：
 *   主程序中底盘速度的最大限制。
 */
#define APP_SPEED_MAX       3000

/*
 * 循迹速度参数：
 *   FORWARD_FAST：正常直行速度；
 *   FORWARD_SLOW：偏差较大时降低前进速度。
 */
#define FORWARD_FAST        1200
#define FORWARD_SLOW        800

/*
 * 麦克纳姆轮平移修正速度：
 *   SHIFT_SMALL：小偏差修正；
 *   SHIFT_BIG：大偏差修正。
 */
#define SHIFT_SMALL         350
#define SHIFT_BIG           650

/*
 * 原地转向修正速度：
 *   TURN_SMALL：小角度修正；
 *   TURN_BIG：大角度修正。
 */
#define TURN_SMALL          250
#define TURN_BIG            500

/*
 * 舵机角度：
 *   YAW_CENTER：水平舵机中位；
 *   RELEASE_LOCK：释放机构关闭；
 *   RELEASE_OPEN：释放机构打开。
 */
#define YAW_CENTER          90
#define RELEASE_LOCK        20
#define RELEASE_OPEN        90

/*
 * 卷线电机速度：
 *   WINCH_MAIN_SPEED：
 *      单独卷线电机速度。
 *
 *   WINCH_PAIR_SPEED：
 *      两个配合卷线电机速度。
 */
#define WINCH_MAIN_SPEED    1600
#define WINCH_PAIR_SPEED    1500

/*
 * OpenMV 图像参数：
 *   QVGA 图像宽度 320，所以中心点 x = 160。
 */
#define IMG_CENTER_X        160

/*
 * 视觉死区：
 *   如果目标 x 坐标在 160 ± 18 以内，
 *   就认为已经对准，不再修正。
 */
#define IMG_DEAD_ZONE       18

/*
 * 面积阈值：
 *   用来粗略判断距离。
 *
 *   BALL_CLOSE_AREA：
 *      球面积大于这个值时，认为已经接近球。
 *
 *   BASKET_CLOSE_AREA：
 *      篮筐面积大于这个值时，认为篮筐距离合适。
 */
#define BALL_CLOSE_AREA     2500
#define BASKET_CLOSE_AREA   2200

/*
 * BASKET_CLOSE_AREA：
 *
 * 功能：
 *   篮筐接近判断阈值。
 *
 * 参数含义：
 *   OpenMV 会把识别到的篮筐区域面积发送给 STM32，
 *   也就是 openmv_data.basket_area。
 *
 *   basket_area 越大，通常说明篮筐在画面中占得越大，
 *   也就是机器人距离篮筐越近。
 *
 * 使用位置：
 *   APP_AlignBasket_ByYawServo() 函数中使用。
 *
 * 判断逻辑：
 *   如果 openmv_data.basket_area < BASKET_CLOSE_AREA，
 *      说明篮筐还比较远，底盘继续慢速前进。
 *
 *   如果 openmv_data.basket_area >= BASKET_CLOSE_AREA，
 *      说明机器人已经接近篮筐，可以停止并进入释放流程。
 *
 * 调试方法：
 *   如果机器人离篮筐还很远就停止，说明这个值太小，可以调大。
 *
 *   如果机器人已经撞到篮筐还不停，说明这个值太大，可以调小。
 *
 * 注意：
 *   2200 不是固定值，需要根据 OpenMV 实际画面、篮筐大小、摄像头安装高度调试。
 */
#define BASKET_CLOSE_AREA   2200

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/*
 * app_state：
 *   当前机器人处于哪个任务步骤。
 */
static AppState_t app_state = APP_WAIT;

/*
 * last_line_dir：
 *   记录上一次循迹偏移方向。
 *
 *   -1：上一次线偏左；
 *    0：不知道或居中；
 *    1：上一次线偏右。
 *
 * 用途：
 *   当四路传感器全白、丢线时，
 *   根据上一次偏移方向慢速找线。
 */
static int8_t last_line_dir = 0;

/*
 * openmv_data：
 *   保存 OpenMV 识别到的球和篮筐信息。
 */
static OpenMV_Data_t openmv_data;

/*
 * openmv_rx_buf：
 *   串口接收缓冲区。
 *
 * OpenMV 每次发送一行数据，例如：
 *   @,1,150,120,2800,1,170,80,3000,#
 */
static char openmv_rx_buf[80];

/*
 * openmv_rx_index：
 *   当前接收到缓冲区的第几个字符。
 */
static uint8_t openmv_rx_index = 0;

/*
 * yaw_angle：
 *   当前水平舵机角度。
 *
 * 用途：
 *   对准篮筐时，每次小幅度增加或减小角度。
 */
static uint8_t yaw_angle = YAW_CENTER;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */

static void APP_Init(void);
static void APP_Task(void);

static int16_t APP_LimitSpeed(int16_t speed);

static void APP_Chassis_Stop(void);
static void APP_Chassis_Mecanum_Run(int16_t vx, int16_t vy, int16_t wz);

static uint8_t APP_Line_Follow_NoPID(void);

static void APP_OpenMV_ParseLine(char *buf);
static void APP_OpenMV_Poll(void);
static uint8_t APP_OpenMV_DataFresh(void);

static uint8_t APP_AlignBall_ByChassis(void);
static uint8_t APP_AlignBasket_ByYawServo(void);

static void APP_DelayWithOpenMV(uint32_t ms);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/

/* USER CODE BEGIN 0 */

/*
 * 函数名：APP_LimitSpeed
 *
 * 功能：
 *   限制底盘速度，防止速度值超过最大允许范围。
 *
 * 参数：
 *   speed：
 *      需要限制的速度值。
 *
 * 返回值：
 *   限幅后的速度。
 */
static int16_t APP_LimitSpeed(int16_t speed)
{
    if (speed > APP_SPEED_MAX)
        return APP_SPEED_MAX;

    if (speed < -APP_SPEED_MAX)
        return -APP_SPEED_MAX;

    return speed;
}


/*
 * 函数名：APP_Chassis_Mecanum_Run
 *
 * 功能：
 *   根据 vx、vy、wz 计算四个麦克纳姆轮的速度。
 *
 * 参数：
 *   vx：
 *      前后方向速度。
 *      正数：前进；
 *      负数：后退。
 *
 *   vy：
 *      左右平移速度。
 *      正数：右移；
 *      负数：左移。
 *
 *   wz：
 *      原地旋转速度。
 *      正数：顺时针；
 *      负数：逆时针。
 *
 * 对应模块：
 *   四轮麦克纳姆底盘。
 */
static void APP_Chassis_Mecanum_Run(int16_t vx, int16_t vy, int16_t wz)
{
    int16_t fl;
    int16_t fr;
    int16_t bl;
    int16_t br;
    int16_t max_abs = 0;

    /*
     * 麦克纳姆轮速度分解。
     *
     * 默认：
     *   fl：左前轮；
     *   fr：右前轮；
     *   bl：左后轮；
     *   br：右后轮。
     */
    fl = vx - vy - wz;
    fr = vx + vy + wz;
    bl = vx + vy - wz;
    br = vx - vy + wz;

    /*
     * 找出四个轮子中速度绝对值最大的一个，
     * 如果超过 APP_SPEED_MAX，就整体等比例缩小。
     */
    if (fl < 0) { if (-fl > max_abs) max_abs = -fl; }
    else        { if ( fl > max_abs) max_abs =  fl; }

    if (fr < 0) { if (-fr > max_abs) max_abs = -fr; }
    else        { if ( fr > max_abs) max_abs =  fr; }

    if (bl < 0) { if (-bl > max_abs) max_abs = -bl; }
    else        { if ( bl > max_abs) max_abs =  bl; }

    if (br < 0) { if (-br > max_abs) max_abs = -br; }
    else        { if ( br > max_abs) max_abs =  br; }

    if (max_abs > APP_SPEED_MAX)
    {
        fl = fl * APP_SPEED_MAX / max_abs;
        fr = fr * APP_SPEED_MAX / max_abs;
        bl = bl * APP_SPEED_MAX / max_abs;
        br = br * APP_SPEED_MAX / max_abs;
    }

    /*
     * 把计算出的速度发送给四个底盘电机。
     *
     * 如果实车某个轮子方向反了，
     * 不要改上面的麦轮公式，
     * 只改下面对应电机前面的正负号。
     */
    AX_MOTOR_A_SetSpeed(APP_LimitSpeed(fl));
    AX_MOTOR_B_SetSpeed(APP_LimitSpeed(fr));
    AX_MOTOR_C_SetSpeed(APP_LimitSpeed(bl));
    AX_MOTOR_D_SetSpeed(APP_LimitSpeed(br));
}


/*
 * 函数名：APP_Chassis_Stop
 *
 * 功能：
 *   停止底盘四个电机。
 *
 * 参数：
 *   无。
 *
 * 对应模块：
 *   麦克纳姆轮底盘。
 */
static void APP_Chassis_Stop(void)
{
    AX_MOTOR_A_SetSpeed(0);
    AX_MOTOR_B_SetSpeed(0);
    AX_MOTOR_C_SetSpeed(0);
    AX_MOTOR_D_SetSpeed(0);
}


/*
 * 函数名：APP_Line_Follow_NoPID
 *
 * 功能：
 *   不使用 PID，根据四路循迹状态表进行固定速度修正。
 *
 * 返回值：
 *   0：
 *      继续循迹。
 *
 *   1：
 *      检测到全黑，认为到达路口、终点或任务区域。
 */
static uint8_t APP_Line_Follow_NoPID(void)
{
    uint8_t line;

    line = AX_LINE_GetData();

    switch (line)
    {
        case 0x06:
            /*
             * L2 + L3 检测到黑线。
             * 车身基本居中，直行。
             */
            APP_Chassis_Mecanum_Run(FORWARD_FAST, 0, 0);
            last_line_dir = 0;
            break;

        case 0x02:
            /*
             * 只有 L2 检测到黑线。
             * 黑线稍微偏左，车向左小幅修正。
             */
            APP_Chassis_Mecanum_Run(FORWARD_FAST, -SHIFT_SMALL, -TURN_SMALL);
            last_line_dir = -1;
            break;

        case 0x01:
        case 0x03:
            /*
             * L1 或 L1+L2 检测到黑线。
             * 黑线偏左较多，车慢速左移并左转修正。
             */
            APP_Chassis_Mecanum_Run(FORWARD_SLOW, -SHIFT_BIG, -TURN_BIG);
            last_line_dir = -1;
            break;

        case 0x04:
            /*
             * 只有 L3 检测到黑线。
             * 黑线稍微偏右，车向右小幅修正。
             */
            APP_Chassis_Mecanum_Run(FORWARD_FAST, SHIFT_SMALL, TURN_SMALL);
            last_line_dir = 1;
            break;

        case 0x08:
        case 0x0C:
            /*
             * L4 或 L3+L4 检测到黑线。
             * 黑线偏右较多，车慢速右移并右转修正。
             */
            APP_Chassis_Mecanum_Run(FORWARD_SLOW, SHIFT_BIG, TURN_BIG);
            last_line_dir = 1;
            break;

        case 0x0F:
            /*
             * 四路全黑。
             * 认为到达路口或任务点。
             */
            APP_Chassis_Stop();
            return 1;

        case 0x00:
            /*
             * 四路全白。
             * 认为丢线。
             * 根据上一次偏移方向寻找黑线。
             */
            if (last_line_dir < 0)
            {
                APP_Chassis_Mecanum_Run(500, -SHIFT_BIG, -TURN_BIG);
            }
            else if (last_line_dir > 0)
            {
                APP_Chassis_Mecanum_Run(500, SHIFT_BIG, TURN_BIG);
            }
            else
            {
                APP_Chassis_Mecanum_Run(500, 0, 0);
            }
            break;

        default:
            /*
             * 其他复杂状态，例如 0x05、0x0A。
             * 为避免车剧烈乱动，先慢速直行。
             */
            APP_Chassis_Mecanum_Run(FORWARD_SLOW, 0, 0);
            break;
    }

    return 0;
}


/*
 * 函数名：APP_OpenMV_ParseLine
 *
 * 功能：
 *   解析 OpenMV 通过串口发来的一行数据。
 *
 * OpenMV 数据格式：
 *   @,ball_ok,ball_x,ball_y,ball_area,basket_ok,basket_x,basket_y,basket_area,#
 *
 * 示例：
 *   @,1,150,120,2800,1,170,80,3000,#
 *
 * 参数：
 *   buf：
 *      串口接收到的一整行字符串。
 */
static void APP_OpenMV_ParseLine(char *buf)
{
    int ball_ok;
    int ball_x;
    int ball_y;
    int ball_area;

    int basket_ok;
    int basket_x;
    int basket_y;
    int basket_area;

    int ret;

    ret = sscanf(buf,
                 "@,%d,%d,%d,%d,%d,%d,%d,%d,#",
                 &ball_ok,
                 &ball_x,
                 &ball_y,
                 &ball_area,
                 &basket_ok,
                 &basket_x,
                 &basket_y,
                 &basket_area);

    if (ret == 8)
    {
        openmv_data.ball_ok = ball_ok;
        openmv_data.ball_x = ball_x;
        openmv_data.ball_y = ball_y;
        openmv_data.ball_area = ball_area;

        openmv_data.basket_ok = basket_ok;
        openmv_data.basket_x = basket_x;
        openmv_data.basket_y = basket_y;
        openmv_data.basket_area = basket_area;

        openmv_data.last_update_tick = HAL_GetTick();
    }
}


/*
 * 函数名：APP_OpenMV_Poll
 *
 * 功能：
 *   轮询读取 USART1 中 OpenMV 发来的数据。
 *
 * 参数：
 *   无。
 *
 * 返回值：
 *   无。
 *
 * 注意：
 *   这个函数是非阻塞的。
 *   HAL_UART_Receive timeout 设置为 0，
 *   没有数据时会立即返回，不会卡住主循环。
 */
static void APP_OpenMV_Poll(void)
{
    uint8_t ch;

    while (HAL_UART_Receive(&huart1, &ch, 1, 0) == HAL_OK)
    {
        if (ch == '\n')
        {
            openmv_rx_buf[openmv_rx_index] = '\0';
            APP_OpenMV_ParseLine(openmv_rx_buf);
            openmv_rx_index = 0;
        }
        else
        {
            if (openmv_rx_index < sizeof(openmv_rx_buf) - 1)
            {
                openmv_rx_buf[openmv_rx_index++] = ch;
            }
            else
            {
                openmv_rx_index = 0;
            }
        }
    }
}


/*
 * 函数名：APP_OpenMV_DataFresh
 *
 * 功能：
 *   判断 OpenMV 数据是否新鲜。
 *
 * 返回值：
 *   1：
 *      最近 300ms 内收到过有效 OpenMV 数据。
 *
 *   0：
 *      数据超时，不可信。
 */
static uint8_t APP_OpenMV_DataFresh(void)
{
    if (openmv_data.last_update_tick == 0)
    {
        return 0;
    }

    if (HAL_GetTick() - openmv_data.last_update_tick < 300)
    {
        return 1;
    }

    return 0;
}


/*
 * 函数名：APP_AlignBall_ByChassis
 *
 * 功能：
 *   根据 OpenMV 识别到的球坐标，用底盘对准并靠近球。
 *
 * 返回值：
 *   0：
 *      还没有完成对准或靠近。
 *
 *   1：
 *      已经接近球，可以进入收球步骤。
 */
static uint8_t APP_AlignBall_ByChassis(void)
{
    if (!APP_OpenMV_DataFresh())
    {
        APP_Chassis_Stop();
        return 0;
    }

    if (!openmv_data.ball_ok)
    {
        /*
         * 没看到球，原地慢速搜索。
         */
        APP_Chassis_Mecanum_Run(0, 0, 350);
        return 0;
    }

    if (openmv_data.ball_x < IMG_CENTER_X - IMG_DEAD_ZONE)
    {
        /*
         * 球在画面左边，车向左平移。
         */
        APP_Chassis_Mecanum_Run(0, -450, 0);
        return 0;
    }
    else if (openmv_data.ball_x > IMG_CENTER_X + IMG_DEAD_ZONE)
    {
        /*
         * 球在画面右边，车向右平移。
         */
        APP_Chassis_Mecanum_Run(0, 450, 0);
        return 0;
    }
    else
    {
        /*
         * 左右已经对准。
         * 如果球面积还不够大，说明距离还远，继续慢速前进。
         */
        if (openmv_data.ball_area < BALL_CLOSE_AREA)
        {
            APP_Chassis_Mecanum_Run(500, 0, 0);
            return 0;
        }
        else
        {
            APP_Chassis_Stop();
            return 1;
        }
    }
}


/*
 * 函数名：
 *   APP_AlignBasket_ByYawServo
 *
 * 功能：
 *   使用 OpenMV 识别到的篮筐位置，控制水平舵机对准篮筐。
 *
 *   当篮筐位于画面左侧时，水平舵机向左微调；
 *   当篮筐位于画面右侧时，水平舵机向右微调；
 *   当篮筐位于画面中心附近时，认为方向已经对准。
 *
 *   方向对准后，再根据 basket_area 判断机器人是否足够接近篮筐。
 *   如果还不够近，底盘慢速前进；
 *   如果已经足够近，底盘停止，并返回 1，表示对准篮筐任务完成。
 *
 * 对应模块：
 *   1. OpenMV 摄像头：
 *      提供篮筐识别结果，包括 basket_ok、basket_x、basket_area。
 *
 *   2. 水平舵机：
 *      通过 YawServo_SetAngle(yaw_angle) 调整摄像头或机构朝向。
 *
 *   3. 麦克纳姆轮底盘：
 *      当篮筐已经居中但距离还不够近时，底盘慢速前进。
 *
 * 使用的主要变量：
 *   openmv_data.basket_ok：
 *      是否识别到篮筐。
 *      1 表示识别到；
 *      0 表示没有识别到。
 *
 *   openmv_data.basket_x：
 *      篮筐中心点在图像中的 x 坐标。
 *      QVGA 图像宽度为 320，所以中心点大约是 160。
 *
 *   openmv_data.basket_area：
 *      篮筐在图像中的面积。
 *      面积越大，通常说明篮筐越近。
 *
 *   yaw_angle：
 *      当前水平舵机角度。
 *      每次根据篮筐偏差增加或减小 2 度。
 *
 *   IMG_CENTER_X：
 *      图像中心 x 坐标，这里是 160。
 *
 *   IMG_DEAD_ZONE：
 *      视觉死区。
 *      如果篮筐 x 坐标在 160 ± IMG_DEAD_ZONE 范围内，
 *      认为篮筐已经居中，不再继续左右调整。
 *
 *   BASKET_CLOSE_AREA：
 *      篮筐接近阈值。
 *      basket_area 达到这个值后，认为机器人已经接近篮筐。
 *
 * 返回值：
 *   0：
 *      还没有完成篮筐对准或还没有接近篮筐。
 *
 *   1：
 *      篮筐已经对准，并且距离已经合适，可以进入释放步骤。
 *
 * 注意事项：
 *   1. 如果舵机调整方向反了，
 *      把 yaw_angle += 2 和 yaw_angle -= 2 的位置对调。
 *
 *   2. 如果底盘靠近方向反了，
 *      把 APP_Chassis_Mecanum_Run(400, 0, 0) 改成
 *      APP_Chassis_Mecanum_Run(-400, 0, 0)。
 *
 *   3. 如果不需要底盘靠近篮筐，
 *      可以删除 basket_area 判断部分，居中后直接 return 1。
 */
static uint8_t APP_AlignBasket_ByYawServo(void)
{
    /*
     * 第一步：判断 OpenMV 数据是否新鲜。
     *
     * 如果长时间没有收到 OpenMV 的有效数据，
     * 说明当前视觉数据不可靠，不能继续根据旧数据动作。
     */
    if (!APP_OpenMV_DataFresh())
    {
        /*
         * 视觉数据失效时，先让底盘停止，防止误动作。
         */
        APP_Chassis_Stop();

        /*
         * 返回 0，表示篮筐对准任务还没有完成。
         */
        return 0;
    }

    /*
     * 第二步：判断 OpenMV 是否识别到篮筐。
     */
    if (!openmv_data.basket_ok)
    {
        /*
         * 没有识别到篮筐时，底盘先停止。
         * 这里不让车乱走，只让水平舵机进行搜索。
         */
        APP_Chassis_Stop();

        /*
         * 舵机搜索逻辑：
         *   yaw_angle 从当前角度逐渐增加；
         *   如果增加到 140 度还没有找到篮筐，
         *   就跳回 50 度重新扫描。
         *
         * 这样可以让摄像头或机构在一个水平范围内反复搜索篮筐。
         */
        if (yaw_angle < 140)
        {
            yaw_angle += 2;
        }
        else
        {
            yaw_angle = 50;
        }

        /*
         * 输出新的舵机角度。
         */
        YawServo_SetAngle(yaw_angle);

        /*
         * 给舵机一点转动时间。
         *
         * 如果你已经写了 APP_DelayWithOpenMV()，
         * 建议使用 APP_DelayWithOpenMV(30)，
         * 这样延时期间仍然能接收 OpenMV 数据。
         */
        APP_DelayWithOpenMV(30);

        return 0;
    }

    /*
     * 第三步：篮筐已经识别到，开始根据篮筐 x 坐标调整水平舵机。
     *
     * 如果 basket_x 小于图像中心减去死区，
     * 说明篮筐在画面左边。
     */
    if (openmv_data.basket_x < IMG_CENTER_X - IMG_DEAD_ZONE)
    {
        /*
         * 篮筐在画面左边：
         *   让水平舵机向左微调。
         *
         * 这里默认 yaw_angle 增大表示向左。
         * 如果实车方向反了，就把这里改成 yaw_angle -= 2。
         */
        if (yaw_angle < 170)
        {
            yaw_angle += 2;
        }

        YawServo_SetAngle(yaw_angle);

        /*
         * 舵机小步调整后，短暂等待。
         */
        APP_DelayWithOpenMV(30);

        /*
         * 还没有完成对准，返回 0。
         */
        return 0;
    }

    /*
     * 如果 basket_x 大于图像中心加死区，
     * 说明篮筐在画面右边。
     */
    else if (openmv_data.basket_x > IMG_CENTER_X + IMG_DEAD_ZONE)
    {
        /*
         * 篮筐在画面右边：
         *   让水平舵机向右微调。
         *
         * 这里默认 yaw_angle 减小表示向右。
         * 如果实车方向反了，就把这里改成 yaw_angle += 2。
         */
        if (yaw_angle > 10)
        {
            yaw_angle -= 2;
        }

        YawServo_SetAngle(yaw_angle);

        APP_DelayWithOpenMV(30);

        return 0;
    }

    /*
     * 第四步：
     * 篮筐已经在画面中心附近，说明水平角度基本对准。
     */
    else
    {
        /*
         * 判断篮筐面积是否达到接近阈值。
         *
         * 如果 basket_area 小于 BASKET_CLOSE_AREA，
         * 说明篮筐在画面中还比较小，也就是机器人还不够近。
         */
        if (openmv_data.basket_area < BASKET_CLOSE_AREA)
        {
            /*
             * 底盘慢速前进，靠近篮筐。
             *
             * 参数说明：
             *   vx = 400：慢速前进；
             *   vy = 0：不左右平移；
             *   wz = 0：不旋转。
             */
            APP_Chassis_Mecanum_Run(400, 0, 0);

            /*
             * 还没到合适距离，返回 0。
             */
            return 0;
        }
        else
        {
            /*
             * 篮筐已经居中，并且面积足够大，
             * 说明机器人已经接近篮筐，可以停止。
             */
            APP_Chassis_Stop();

            /*
             * 返回 1，表示篮筐对准和靠近任务完成。
             * 状态机可以进入 APP_WINCH_PAIR 或 APP_RELEASE。
             */
            return 1;
        }
    }
}


/*
 * 函数名：APP_Init
 *
 * 功能：
 *   初始化机器人所有用户模块。
 *
 * 包含模块：
 *   延时、按键、循迹、电机、LED、蜂鸣器、舵机、卷线电机。
 */
static void APP_Init(void)
{
    AX_DELAY_Init();

    AX_KEY_Init();
    AX_LINE_Init();
    AX_MOTOR_Init();

    AX_LED_Init();
    AX_BEEP_Init();

    Servo_Init();
    ExtraMotor_Init();

    APP_Chassis_Stop();

    YawServo_SetAngle(YAW_CENTER);
    ReleaseServo_SetAngle(RELEASE_LOCK);

    Winch_Main_SetSpeed(0);
    Winch_Pair_SetSpeed(0);

    AX_LED_Green_On();

    app_state = APP_WAIT;
}


/*
 * 函数名：APP_Task
 *
 * 功能：
 *   机器人主状态机。
 *
 * 调用位置：
 *   main.c 的 while(1) 主循环中反复调用。
 */
static void APP_Task(void)
{
    /*
     * 每次循环先读取 OpenMV 数据。
     */
    APP_OpenMV_Poll();

    switch (app_state)
    {
        case APP_WAIT:
        {
            APP_Chassis_Stop();

            /*
             * 等待按键启动。
             */
            if (AX_KEY_Scan())
            {
                AX_Delayms(20);

                if (AX_KEY_Scan())
                {
                    AX_BEEP_On();
                    AX_Delayms(100);
                    AX_BEEP_Off();

                    AX_LED_Red_On();

                    app_state = APP_LINE;
                }

                while (AX_KEY_Scan());
            }

            break;
        }

        case APP_LINE:
        {
            /*
             * 循迹行驶。
             * 检测到全黑后进入找球状态。
             */
            if (APP_Line_Follow_NoPID())
            {
                APP_Chassis_Stop();
                AX_Delayms(300);

                app_state = APP_FIND_BALL;
            }

            break;
        }

        case APP_FIND_BALL:
        {
            /*
             * 找球。
             */
            if (APP_OpenMV_DataFresh() && openmv_data.ball_ok)
            {
                app_state = APP_ALIGN_BALL;
            }
            else
            {
                APP_Chassis_Mecanum_Run(0, 0, 300);
            }

            break;
        }

        case APP_ALIGN_BALL:
        {
            /*
             * 对准并靠近球。
             */
            if (APP_AlignBall_ByChassis())
            {
                APP_Chassis_Stop();
                AX_Delayms(200);

                app_state = APP_GRAB_BALL;
            }

            break;
        }

        case APP_GRAB_BALL:
        {
            /*
             * 单独卷线电机收球。
             */
            Winch_Main_SetSpeed(WINCH_MAIN_SPEED);
            APP_DelayWithOpenMV(1500);
            Winch_Main_SetSpeed(0);

            APP_DelayWithOpenMV(300);

            app_state = APP_FIND_BASKET;
            break;
        }

        case APP_FIND_BASKET:
        {
            /*
             * 找对方篮筐。
             */
            if (APP_OpenMV_DataFresh() && openmv_data.basket_ok)
            {
                app_state = APP_ALIGN_BASKET;
            }
            else
            {
                if (yaw_angle < 140)
                {
                    yaw_angle += 2;
                }
                else
                {
                    yaw_angle = 50;
                }

                YawServo_SetAngle(yaw_angle);
                AX_Delayms(30);
            }

            break;
        }

        case APP_ALIGN_BASKET:
        {
            /*
             * 水平舵机对准篮筐。
             */
            if (APP_AlignBasket_ByYawServo())
            {
                AX_Delayms(300);
                app_state = APP_WINCH_PAIR;
            }

            break;
        }

        case APP_WINCH_PAIR:
        {
            /*
             * 两个配合卷线电机运行。
             * 一个正转，一个反转。
             */
            Winch_Pair_SetSpeed(WINCH_PAIR_SPEED);
            APP_DelayWithOpenMV(1200);
            Winch_Pair_SetSpeed(0);

            APP_DelayWithOpenMV(300);

            app_state = APP_RELEASE;
            break;
        }

        case APP_RELEASE:
        {
            /*
             * 打开释放舵机，把球放入篮筐。
             */
            ReleaseServo_SetAngle(RELEASE_OPEN);
            APP_DelayWithOpenMV(800);

            ReleaseServo_SetAngle(RELEASE_LOCK);
            APP_DelayWithOpenMV(300);

            app_state = APP_FINISH;
            break;
        }

        case APP_FINISH:
        {
            /*
             * 结束状态。
             * 所有执行机构停止。
             */
            APP_Chassis_Stop();

            Winch_Main_SetSpeed(0);
            Winch_Pair_SetSpeed(0);

            AX_LED_Red_Off();
            AX_LED_Green_On();

            break;
        }

        default:
        {
            app_state = APP_FINISH;
            break;
        }
    }
}

static void APP_DelayWithOpenMV(uint32_t ms)
{
    uint32_t start = HAL_GetTick();

    while (HAL_GetTick() - start < ms)
    {
        APP_OpenMV_Poll();
        AX_Delayms(1);
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM8_Init();
  MX_TIM9_Init();
  MX_TIM10_Init();
  MX_TIM11_Init();
  MX_TIM12_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */
  APP_Init();
  /* USER CODE END 2 */

  /* Infinite loop */

  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  APP_Task();
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
