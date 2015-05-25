/*
 *     SocialLedge.com - Copyright (C) 2013
 *
 *     This file is part of free software framework for embedded processors.
 *     You can use it and/or distribute it as long as this copyright header
 *     remains unmodified.  The code is free for personal use and requires
 *     permission to use in a commercial product.
 *
 *      THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 *      OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 *      MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 *      I SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR
 *      CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 *     Authors: Noe Quintero and Son Nguyen
 */

#include "tasks.hpp"
#include "examples/examples.hpp"
#include <stdio.h>
#include "uart0.hpp"
#include "I2C2.hpp"
#include "LPC17xx.h"
#include "utilities.h"
#include "eint.h"
#include "lpc_rit.h"
#include "stdint.h"
#include "uart3.hpp"
#include "uart2.hpp"
#include "string.h"
#include "soft_timer.hpp"
#include "PID.hpp"

// Used to wrap
#define wrap_180(x) (x < -180 ? x+360 : (x > 180 ? x - 360: x))

// Global Queue Handle
QueueHandle_t qpid[3]   = {0,0,0};
QueueHandle_t qimu[3]   = {0,0,0};
QueueHandle_t qrx[4]    = {0,0,0,0};

// Map the input to given ranges
int32_t map(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

union uint16_2bytes
{
    int16_t  temp_in16;     // 16-bit signed integer to be converted to two bytes
    uint16_t temp_uint16;   // 16-bit unsigned integer to be converted to two bytes
    uint8_t  temp_uint8[2]; // 2 bytes (unsigned 8-bit integers) to be converted to a 16-bit signed or unsigned integer
};

void set_pwm(I2C2 * p,uint8_t addr, uint8_t ch, uint16_t duty)
{
    uint16_2bytes data;
    data.temp_uint16 = duty;
    p->writeReg(addr, ch*4+6, 0x0);                 // On reg low
    p->writeReg(addr, ch*4+7, 0x0);                 // On reg high
    p->writeReg(addr, ch*4+8, data.temp_uint8[0]);  // Off reg low
    p->writeReg(addr, ch*4+9, data.temp_uint8[1]);  // Off reg high
}

void pwm_exp_task(void * p)
{
    // Motor config
    //
    //         F
    //   FL    |    FR
    //     \   |   /
    //      \  |  /
    //       \ | /
    //L--------+--------R
    //       / | \
    //      /  |  \
    //     /   |   \
    //   BL    |    BR
    //         B
    // Motor channels
    const uint8_t MOTOR_FL  = 0;
    const uint8_t MOTOR_F   = 1;
    const uint8_t MOTOR_FR  = 2;
    const uint8_t MOTOR_R   = 3;
    const uint8_t MOTOR_BR  = 4;
    const uint8_t MOTOR_B   = 5;
    const uint8_t MOTOR_BL  = 6;
    const uint8_t MOTOR_L   = 7;

    // Reg locations
    const uint8_t mode1_reg     = 0x0;
    const uint8_t prescale_reg  = 0xFE;
    const uint8_t pwm_exp_addr  = 0x80;     // Pre-shifted 7-bit address

    float pidypr[3] = {0,0,0};           // Yaw, pitch, roll pid variables
    float lastpidypr[3] = {0,0,0};
    int32_t rcthr = 0;
    int16_t motor_fl    = 0;
    int16_t motor_f     = 0;
    int16_t motor_fr    = 0;
    int16_t motor_r     = 0;
    int16_t motor_br    = 0;
    int16_t motor_b     = 0;
    int16_t motor_bl    = 0;
    int16_t motor_l     = 0;

    I2C2& pwm_exp = I2C2::getInstance();    // Get I2C driver instance
    pwm_exp.init(100);                      // 100 kHz

    // Set freq for the PWM expander
    pwm_exp.writeReg(pwm_exp_addr, mode1_reg, 0x10);    // Sleep
    pwm_exp.writeReg(pwm_exp_addr, prescale_reg, 0x80); // Set clock
    delay_ms(5);
    pwm_exp.writeReg(pwm_exp_addr, mode1_reg, 0xA1);    // Auto increment
    delay_ms(1);

    // Init ESC
    // Set ch0-7 to lowest pulse width
    for(uint8_t i = 0; i<8; i+=4)
    {
        set_pwm(&pwm_exp, pwm_exp_addr, i, 164);
    }
    delay_ms(1000);

    // Set ch0-7 to highest pulse width
    for(uint8_t i = 0; i<8; i+=4)
    {
        set_pwm(&pwm_exp, pwm_exp_addr, i, 430);
    }
    delay_ms(1000);

    // Set ch0-7 to lowest pulse width
    for(uint8_t i = 0; i<8; i+=4)
    {
        set_pwm(&pwm_exp, pwm_exp_addr, i, 164);
    }
    delay_ms(1000);

    while(1)
    {
        //printf("PWM Task \n");
        for(uint8_t i = 0; i<3 ;i++)
            lastpidypr[i] = pidypr[i];

        if(!xQueueReceive(qpid[0], &pidypr[0], 1))
        {
            pidypr[0] = lastpidypr[0];
        }
        if(!xQueueReceive(qpid[1], &pidypr[1], 1))
        {
            pidypr[1] = lastpidypr[1];
        }
        if(!xQueueReceive(qpid[2], &pidypr[2], 1))
        {
            pidypr[2] = lastpidypr[2];
        }
        if(!xQueueReceive(qrx[0], &rcthr, 1000))
        {
            // Turn off motors
            for(uint8_t i = 0; i<8; i+=4)
            {
                set_pwm(&pwm_exp, pwm_exp_addr, i, 164);
            }
        }

//      printf("Thr PWM: %d \n", rcthr);
//      printf("rpwm: %f \n", pidypr[2]);
//      printf("p: %d , r: %d \n", (int16_t)pidypr[1],(int16_t)pidypr[2]);

        motor_fl = rcthr - (int16_t)pidypr[2] - (int16_t)pidypr[1];
        if(motor_fl>430)
            motor_fl = 430;
        if(motor_fl<130)
            motor_fl = 160;

        motor_bl = rcthr - (int16_t)pidypr[2] + (int16_t)pidypr[1];
        if(motor_bl>430)
            motor_bl = 430;
        if(motor_bl<130)
            motor_bl = 160;

        motor_fr = rcthr + (int16_t)pidypr[2] - (int16_t)pidypr[1];
        if(motor_fr>430)
            motor_fr = 430;
        if(motor_fr<130)
            motor_fr = 160;

        motor_br = rcthr + (int16_t)pidypr[2] + (int16_t)pidypr[1];
        if(motor_br>430)
            motor_br = 430;
        if(motor_br<130)
            motor_br = 160;

        motor_f = rcthr + pidypr[2]*0.707 - pidypr[1]*0.707;
        if(motor_f>430)
            motor_f = 430;
        if(motor_f<130)
            motor_f = 160;

        motor_l = rcthr - pidypr[2]*0.707 - pidypr[1]*0.707;
        if(motor_l>430)
            motor_l = 430;
        if(motor_l<130)
            motor_l = 160;

        motor_b = rcthr - pidypr[2]*0.707 + pidypr[1]*0.707;
        if(motor_b>430)
            motor_b = 430;
        if(motor_b<130)
            motor_b = 160;

        motor_r = rcthr + pidypr[2]*0.707 + pidypr[1]*0.707;
        if(motor_r>430)
            motor_r = 430;
        if(motor_r<130)
            motor_r = 160;

        if(rcthr > 150)
        {
            set_pwm(&pwm_exp, pwm_exp_addr, MOTOR_FL, motor_fl);
            set_pwm(&pwm_exp, pwm_exp_addr, MOTOR_BL, motor_bl);
            set_pwm(&pwm_exp, pwm_exp_addr, MOTOR_FR, motor_fr);
            set_pwm(&pwm_exp,pwm_exp_addr, MOTOR_BR, motor_br);
            set_pwm(&pwm_exp,pwm_exp_addr, MOTOR_F, motor_f);
            set_pwm(&pwm_exp,pwm_exp_addr, MOTOR_L, motor_l);
            set_pwm(&pwm_exp,pwm_exp_addr, MOTOR_B, motor_b);
            set_pwm(&pwm_exp,pwm_exp_addr, MOTOR_R, motor_r);
        }
        else
        {
            // Turn off motors
            for(uint8_t i = 0; i<8; i+=4)
            {
                set_pwm(&pwm_exp, pwm_exp_addr, i, 164);
            }
        }
        //vTaskDelay(50);
    }
}

void imu_task(void * p)
{
    char data[65];
    char* pEnd;
    float ypr[3];

    Uart3& imu = Uart3::getInstance();  // Get uart3 driver instance
    imu.init(57600);

    while(1)
    {
        //printf("IMU Task \n");
        if(imu.scanf("%s", data))
        {
            pEnd = strtok(data,"=");    // Find the '=' in the string
            pEnd = strtok(NULL,",");    // Find the first comma
            ypr[0] = atof(pEnd);           // Convert str to float

            pEnd = strtok(NULL,",");    // Find the second comma
            ypr[1] = atof(pEnd);         // Convert str to float

            pEnd = strtok(NULL,",");    // Find the third comma
            ypr[2] = atof(pEnd);          // Convert str to float

            ypr[2] = ypr[2] - 3.1;
           //printf("y: %f, p: %f, r: %f \n", ypr[0], ypr[1], ypr[2]);
            xQueueSend(qimu[0],&ypr[0], 0);
            xQueueSend(qimu[1],&ypr[1], 0);
            xQueueSend(qimu[2],&ypr[2], 0);
            imu.flush();
        }
        vTaskDelay(10); // Wait 10 ms
    }
}

struct rx
{
    uint16_t thro;
    uint16_t aile;
    uint16_t elev;
    uint16_t rudd;
    uint16_t gear;
    uint16_t aux1;
};

rx parse_rx(char * data)
{
    uint16_2bytes ch_val;
    rx controls;
    uint8_t ch_id;
    for(uint8_t i = 0; i <14; i = i + 2)
    {
        ch_id = ((uint8_t)data[2+i])>>3;
        //printf("ch: %d  ", ch_id);
        // 11 bits of data for 2048 mode
        ch_val.temp_uint8[0] = (uint8_t)data[3+i];
        ch_val.temp_uint8[1] = (uint8_t)data[2+i] & 0x7;
        //printf("val: %d \n", ch_val.temp_uint16);
        switch(ch_id)
        {
            case 0:
                // Throttle
                controls.thro = ch_val.temp_uint16;
                break;
            case 1:
                // Aileron
                controls.aile = ch_val.temp_uint16;
                break;
            case 2:
                // Elevator
                controls.elev = ch_val.temp_uint16;
                break;
            case 3:
                // Rudder
                controls.rudd = ch_val.temp_uint16;
                break;
            case 4:
                // Gear
                controls.gear = ch_val.temp_uint16;
                break;
            case 5:
                // Aux1
                controls.aux1 = ch_val.temp_uint16;
                break;
            default:
                break;
        }
    }
    return controls;
}

void radio_rx_task(void * p)
{

    char data[16];
    rx control;
    int32_t typr[4];
    Uart2& spektrum = Uart2::getInstance();  // Get uart2 driver instance
    // Get uart2 driver instance
    spektrum.init(115200);  // Spektrum receiver uses 115200 baud
    spektrum.flush();       // Clear buffer
    while(1)
    {

        //printf("RX Task \n");
        if(LPC_UART2->LSR & 0x01)   // Buffer is not empty
        {
            // read 16 bytes
            spektrum.gets(data,16,10);
//          printf("0x%x%x 0x%x%x ",data[0],data[1],data[2],data[3]);
//          printf("0x%x%x 0x%x%x ",data[4],data[5],data[6],data[7]);
//          printf("0x%x%x 0x%x%x ",data[8],data[9],data[10],data[11]);
//          printf("0x%x%x 0x%x%x\n",data[12],data[13],data[14],data[15]);

            control = parse_rx(data);   // Parse the packet

           // printf("%s \n",data);
            typr[0] = map(control.thro,130,1740, 100,400);
            typr[3] = map(control.aile,130,1740,-45,45);
            typr[2] = map(control.elev,130,1740,-45,45);
            typr[1] = map(control.rudd,130,1740,-150,150);

//          printf("Thro: %d, Aile: %d, Elev: %d, Rudd: %d \n",
//                    control.thro, control.aile, control.elev, control.rudd);
//          printf("Thr Rx: %d \n", control.thro);
            xQueueSend(qrx[0], &typr[0], 1);
            xQueueSend(qrx[1], &typr[1], 1);
            xQueueSend(qrx[2], &typr[2], 1);
            xQueueSend(qrx[3], &typr[3], 1);
            spektrum.flush();   // Clear buffer
        }
    }
}

void pid_task(void * p)
{
    const uint8_t delay = 100; // ms
    sPID y_spid =
    {
        50,    // P
        0,      // I
        0,      // D
        1,      // Direction
        1740,   // max
        0,    // min
        1       // auto
    };

    sPID p_spid =
    {
        50,     // P
        0,      // I
        0,      // D
        1,      // Direction
        1740,   // max
        0,    // min
        1       // auto
    };

    sPID r_spid =
    {
        50,     // P
        0,      // I
        0,      // D
        1,      // Direction
        1740,   // max
        0,      // min
        1       // auto
    };

    float ypr[3];
    float ypr_out[3];
    double ysetpoint = 0.0;
    double y_input = 0.0;
    double y_output = 0.0;

    double psetpoint = 0.0;
    double p_input = 0.0;
    double p_output = 0.0;

    double rsetpoint = 0.0;
    double r_input = 0.0;
    double r_output = 0.0;

    PID y_pid(&y_input,&y_output,&ysetpoint,y_spid,delay);
    y_pid.set_mode(y_spid);

    PID p_pid(&p_input,&p_output,&psetpoint,p_spid,delay);
    p_pid.set_mode(p_spid);

    PID r_pid(&r_input,&r_output,&rsetpoint,r_spid,delay);
    r_pid.set_mode(r_spid);

    while(1)
    {
        //printf("PID Task \n");
        if(xQueueReceive(qimu[0], &ypr,10))
        {
            xQueueReceive(qimu[1], &ypr[1],10);
            xQueueReceive(qimu[2], &ypr[2],10);

            y_input = ypr[0];
            p_input = ypr[1];
            r_input = ypr[2];

            y_pid.compute();
            p_pid.compute();
            r_pid.compute();

            ypr_out[0] = (float)y_output;
            ypr_out[1] = (float)p_output;
            ypr_out[2] = (float)r_output;

//          printf("p: %f  ", p_output);
//          printf("r: %f\n", ypr_out[2]);
            xQueueSend(qpid[0], &ypr_out[0], 0);
            xQueueSend(qpid[1], &ypr_out[1], 0);
            xQueueSend(qpid[2], &ypr_out[2], 0);
        }
        vTaskDelay(delay);
    }
}

int main(void)
{
    for(uint8_t i = 0; i<3; i++)
        qpid[i] = xQueueCreate(1, sizeof(float));
    for(uint8_t i = 0; i<3; i++)
        qimu[i] = xQueueCreate(1, sizeof(float));
    for(uint8_t i = 0; i<4; i++)
        qrx[i] = xQueueCreate(1, sizeof(int32_t));

    xTaskCreate(pwm_exp_task, "pwm_task", STACK_BYTES(2048), 0, PRIORITY_HIGH, 0);
    xTaskCreate(imu_task, "imu_task",STACK_BYTES(2048), 0, PRIORITY_HIGH, 0);
    xTaskCreate(radio_rx_task, "radio",STACK_BYTES(2048), 0, PRIORITY_CRITICAL, 0);
    xTaskCreate(pid_task,"pid_task",STACK_BYTES(2048),0,PRIORITY_HIGH, 0);
    vTaskStartScheduler();
    return -1;
}
