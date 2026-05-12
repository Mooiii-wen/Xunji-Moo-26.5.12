#include "robot_app.h"

#include "delay.h"
#include "key.h"
#include "line.h"
#include "motor.h"
#include "led.h"
#include "beep.h"
#include "servo.h"
#include "extra_motor.h"

/* =========================
   速度参数
   不是 PID，只是固定速度档位
   ========================= */

#define SPEED_MAX       3000

#define FORWARD_FAST    1200
#define FORWARD_SLOW    800

#define SHIFT_SMALL     350
#define SHIFT_BIG       650

#define TURN_SMALL      250
#define TURN_BIG        500

/* 舵机角度 */
#define YAW_CENTER          90
#define YAW_TARGET          120

#define RELEASE_LOCK        20
#define RELEASE_OPEN        90

/* 卷线速度 */
#define WINCH_MAIN_SPEED    1600
#define WINCH_PAIR_SPEED    1500

/*
   AX_LINE_GetData() 的返回值：
   L1(B15) = 0x01
   L2(B14) = 0x02
   L3(B13) = 0x04
   L4(B12) = 0x08

   常用：
   0x06 = L2 + L3，中间压线
   0x0F = 四个全黑，路口/终点
   0x00 = 全白，丢线
*/

typedef enum
{
    ROBOT_WAIT = 0,
    ROBOT_LINE,
    ROBOT_YAW,
    ROBOT_WINCH_MAIN,
    ROBOT_WINCH_PAIR,
    ROBOT_RELEASE,
    ROBOT_FINISH
} RobotState_t;

static RobotState_t robot_state = ROBOT_WAIT;

/*
   丢线时用上一次偏移方向找线：
   -1：上次偏左
    1：上次偏右
    0：不知道
*/
static int8_t last_line_dir = 0;


/* =========================
   限幅
   ========================= */

static int16_t LimitSpeed(int16_t speed)
{
    if (speed > SPEED_MAX)
        return SPEED_MAX;

    if (speed < -SPEED_MAX)
        return -SPEED_MAX;

    return speed;
}


/* =========================
   麦克纳姆轮运动函数

   vx：前后，正数前进
   vy：左右，正数右平移
   wz：旋转，正数顺时针

   默认电机分配：
   A：左前轮
   B：右前轮
   C：左后轮
   D：右后轮
   ========================= */

void Chassis_Mecanum_Run(int16_t vx, int16_t vy, int16_t wz)
{
    int16_t fl;
    int16_t fr;
    int16_t bl;
    int16_t br;

    int16_t max_abs = 0;

    fl = vx - vy - wz;
    fr = vx + vy + wz;
    bl = vx + vy - wz;
    br = vx - vy + wz;

    if (fl < 0)
    {
        if (-fl > max_abs) max_abs = -fl;
    }
    else
    {
        if (fl > max_abs) max_abs = fl;
    }

    if (fr < 0)
    {
        if (-fr > max_abs) max_abs = -fr;
    }
    else
    {
        if (fr > max_abs) max_abs = fr;
    }

    if (bl < 0)
    {
        if (-bl > max_abs) max_abs = -bl;
    }
    else
    {
        if (bl > max_abs) max_abs = bl;
    }

    if (br < 0)
    {
        if (-br > max_abs) max_abs = -br;
    }
    else
    {
        if (br > max_abs) max_abs = br;
    }

    if (max_abs > SPEED_MAX)
    {
        fl = fl * SPEED_MAX / max_abs;
        fr = fr * SPEED_MAX / max_abs;
        bl = bl * SPEED_MAX / max_abs;
        br = br * SPEED_MAX / max_abs;
    }

    /*
       如果实车某个轮子方向反了，只改这里的正负号。

       例如 A 轮反了：
       AX_MOTOR_A_SetSpeed(-LimitSpeed(fl));
    */
    AX_MOTOR_A_SetSpeed( LimitSpeed(fl));
    AX_MOTOR_B_SetSpeed( LimitSpeed(fr));
    AX_MOTOR_C_SetSpeed( LimitSpeed(bl));
    AX_MOTOR_D_SetSpeed( LimitSpeed(br));
}


void Chassis_Stop(void)
{
    AX_MOTOR_A_SetSpeed(0);
    AX_MOTOR_B_SetSpeed(0);
    AX_MOTOR_C_SetSpeed(0);
    AX_MOTOR_D_SetSpeed(0);
}


/* =========================
   无 PID 循迹

   返回值：
   0：继续循迹
   1：检测到路口/终点
   ========================= */

uint8_t Line_Follow_NoPID(void)
{
    uint8_t line;

    line = AX_LINE_GetData();

    switch (line)
    {
        case 0x06:
            /*
               L2 + L3 压线，车身比较正，直走
            */
            Chassis_Mecanum_Run(FORWARD_FAST, 0, 0);
            last_line_dir = 0;
            break;

        case 0x02:
            /*
               只有 L2 检测到，线偏左一点
               麦轮可以边前进边左移，同时轻微左转
            */
            Chassis_Mecanum_Run(FORWARD_FAST, -SHIFT_SMALL, -TURN_SMALL);
            last_line_dir = -1;
            break;

        case 0x01:
        case 0x03:
            /*
               L1 或 L1+L2，偏左较多
            */
            Chassis_Mecanum_Run(FORWARD_SLOW, -SHIFT_BIG, -TURN_BIG);
            last_line_dir = -1;
            break;

        case 0x04:
            /*
               只有 L3 检测到，线偏右一点
            */
            Chassis_Mecanum_Run(FORWARD_FAST, SHIFT_SMALL, TURN_SMALL);
            last_line_dir = 1;
            break;

        case 0x08:
        case 0x0C:
            /*
               L4 或 L3+L4，偏右较多
            */
            Chassis_Mecanum_Run(FORWARD_SLOW, SHIFT_BIG, TURN_BIG);
            last_line_dir = 1;
            break;

        case 0x0F:
            /*
               四路全黑，认为是路口、停止线或任务点
            */
            Chassis_Stop();
            return 1;

        case 0x00:
            /*
               全白，丢线。
               不用 PID，直接按上次方向慢速找线。
            */
            if (last_line_dir < 0)
            {
                Chassis_Mecanum_Run(500, -SHIFT_BIG, -TURN_BIG);
            }
            else if (last_line_dir > 0)
            {
                Chassis_Mecanum_Run(500, SHIFT_BIG, TURN_BIG);
            }
            else
            {
                Chassis_Mecanum_Run(500, 0, 0);
            }
            break;

        case 0x07:
            /*
               左中区域较黑，略向左修正
            */
            Chassis_Mecanum_Run(FORWARD_SLOW, -SHIFT_SMALL, -TURN_SMALL);
            last_line_dir = -1;
            break;

        case 0x0E:
            /*
               右中区域较黑，略向右修正
            */
            Chassis_Mecanum_Run(FORWARD_SLOW, SHIFT_SMALL, TURN_SMALL);
            last_line_dir = 1;
            break;

        default:
            /*
               其他复杂状态，先慢速直行，避免剧烈乱转
            */
            Chassis_Mecanum_Run(FORWARD_SLOW, 0, 0);
            break;
    }

    return 0;
}


/* =========================
   舵机接口

   你目前上传的头文件里还没有舵机库。
   所以这里先保留接口。
   等你确定舵机接在哪个 TIM 的哪个通道后，
   再把这里补成真正 PWM 输出。
   ========================= */

//void YawServo_SetAngle(uint8_t angle)
//{
//    /*
//       TODO：水平舵机
//       例如：
//       TIM_SetCompare1(TIM3, 500 + angle * 2000 / 180);
//    */
//    (void)angle;
//}
//
//
//void ReleaseServo_SetAngle(uint8_t angle)
//{
//    /*
//       TODO：释放舵机
//       例如：
//       TIM_SetCompare2(TIM3, 500 + angle * 2000 / 180);
//    */
//    (void)angle;
//}


/* =========================
   卷线电机接口

   目前 motor.h 只有 A/B/C/D 四路，
   已经全部给麦克纳姆底盘用了。
   三个卷线电机需要新增电机驱动函数。
   ========================= */

//void Winch_Main_SetSpeed(int16_t speed)
//{
//    /*
//       TODO：单独卷线电机
//       正数：收线
//       负数：放线
//
//       后面可以改成：
//       AX_MOTOR_E_SetSpeed(speed);
//    */
//    (void)speed;
//}
//
//
//void Winch_Pair_SetSpeed(int16_t speed)
//{
//    /*
//       TODO：两个配合卷线电机
//       一个正转，一个反转，速度相同
//
//       后面可以改成：
//       AX_MOTOR_F_SetSpeed(speed);
//       AX_MOTOR_G_SetSpeed(-speed);
//    */
//    (void)speed;
//}


/* =========================
   机器人初始化
   ========================= */

void Robot_Init(void)
{
    AX_DELAY_Init();

    AX_KEY_Init();
    AX_LINE_Init();
    AX_MOTOR_Init();

    AX_LED_Init();
    AX_BEEP_Init();

    Chassis_Stop();

    Servo_Init();
    ExtraMotor_Init();

    YawServo_SetAngle(90);
    ReleaseServo_SetAngle(20);

    AX_LED_Green_On();

    robot_state = ROBOT_WAIT;
}


/* =========================
   主任务状态机
   ========================= */

void Robot_Task(void)
{
    switch (robot_state)
    {
        case ROBOT_WAIT:
        {
            Chassis_Stop();

            if (AX_KEY_Scan())
            {
                AX_Delayms(20);

                if (AX_KEY_Scan())
                {
                    AX_BEEP_On();
                    AX_Delayms(100);
                    AX_BEEP_Off();

                    AX_LED_Red_On();

                    robot_state = ROBOT_LINE;
                }

                while (AX_KEY_Scan());
            }

            break;
        }

        case ROBOT_LINE:
        {
            if (Line_Follow_NoPID())
            {
                Chassis_Stop();
                AX_Delayms(300);

                robot_state = ROBOT_YAW;
            }

            break;
        }

        case ROBOT_YAW:
        {
            YawServo_SetAngle(YAW_TARGET);
            AX_Delayms(800);

            robot_state = ROBOT_WINCH_MAIN;
            break;
        }

        case ROBOT_WINCH_MAIN:
        {
            /*
               单独卷线电机：
               这里示例为正转收线 1.5 秒。
            */
            Winch_Main_SetSpeed(WINCH_MAIN_SPEED);
            AX_Delayms(1500);
            Winch_Main_SetSpeed(0);

            AX_Delayms(300);

            robot_state = ROBOT_WINCH_PAIR;
            break;
        }

        case ROBOT_WINCH_PAIR:
        {
            /*
               两个卷线电机：
               一正一反，同速运行。
            */
            Winch_Pair_SetSpeed(WINCH_PAIR_SPEED);
            AX_Delayms(1200);
            Winch_Pair_SetSpeed(0);

            AX_Delayms(300);

            robot_state = ROBOT_RELEASE;
            break;
        }

        case ROBOT_RELEASE:
        {
            ReleaseServo_SetAngle(RELEASE_OPEN);
            AX_Delayms(800);

            ReleaseServo_SetAngle(RELEASE_LOCK);
            AX_Delayms(300);

            robot_state = ROBOT_FINISH;
            break;
        }

        case ROBOT_FINISH:
        {
            Chassis_Stop();
            Winch_Main_SetSpeed(0);
            Winch_Pair_SetSpeed(0);

            AX_LED_Red_Off();
            AX_LED_Green_On();

            break;
        }

        default:
        {
            robot_state = ROBOT_FINISH;
            break;
        }
    }
}
/*
 * robot_app.c
 *
 *  Created on: May 11, 2026
 *      Author: Lenovo
 */


