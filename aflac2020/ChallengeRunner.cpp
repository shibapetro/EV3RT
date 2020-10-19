//
//  ChallengeRunner.cpp
//  aflac2020
//
//  Copyright © 2020 Ahiruchan Koubou. All rights reserved.
//

#include "app.h"
#include "ChallengeRunner.hpp"

ChallengeRunner::ChallengeRunner(Motor* lm, Motor* rm, Motor* tm, Motor* am) : LineTracer(lm, rm, tm){
    _debug(syslog(LOG_NOTICE, "%08u, ChallengeRunner constructor", clock->now()));
    leftMotor   = lm;
    rightMotor  = rm;
    armMotor =am;
    pwm_L = 20;
    pwm_R = 20;
    pwmMode = 1;
    count = 0;
    procCount = 1;
    traceCnt = 0;
    frozen = false;
}

void ChallengeRunner::haveControl() {
    activeNavigator = this;
    syslog(LOG_NOTICE, "%08u, ChallengeRunner has control", clock->now());
}

void ChallengeRunner::operate() {

    if (frozen) {
        //printf("Stop");
        pwm_L = 0;
        pwm_R = 0;

    }else{
        if (pwmMode != Mode_speed_constant){
            if (++count && count == procCount){
                switch (pwmMode) {
                    case Mode_speed_increaseL:
                        ++pwm_L;
                        break;
                    case Mode_speed_decreaseL:
                        --pwm_L;
                        break;
                    case Mode_speed_increaseR:
                        ++pwm_R;
                        break;
                    case Mode_speed_decreaseR:
                        --pwm_R;
                        break;
                    case Mode_speed_increaseLR:
                        ++pwm_L;
                        ++pwm_R;
                        break;
                    case Mode_speed_decreaseLR:
                        --pwm_L;
                        --pwm_R;
                        break;
                    case Mode_speed_incrsLdcrsR:
                        ++pwm_L;
                        --pwm_R;
                        break;
                    case Mode_speed_incrsRdcrsL:
                        --pwm_L;
                        ++pwm_R;
                        break;
                    default:
                        break;
                }
                count = 0; //初期化
            }
        }
    }
    
    leftMotor->setPWM(pwm_L);
    rightMotor->setPWM(pwm_R);

    // if (++traceCnt && traceCnt > 50) {
    //     printf(",pwm_L=%d, pwm_R=%d, count=%d, procCount=%d\n", pwm_L,pwm_R,count,procCount);
    //     traceCnt = 0;
    // }
}

//　Activate challengeRunner PWM control according to g_challenge_stepNo
void ChallengeRunner::runChallenge() {

    switch (g_challenge_stepNo) {
        //スラローム専用処理
        case 0:
            printf("ぶつかり\n");
            //haveControl();
            setPwmLR(25,25,Mode_speed_constant,1);
            clock->sleep(880);
            setPwmLR(10,10,Mode_speed_constant,1);
            clock->sleep(1000);
            rest(200);
            break;
        case 1:
            setPwmLR(-20,-20,Mode_speed_constant,1);
            clock->sleep(500);
            rest(200);
            if (_LEFT == 1){
                setPwmLR(42,40,Mode_speed_decreaseLR,30);
            }else{
                setPwmLR(40,42,Mode_speed_decreaseLR,30);
            }
            break;
        case 10:
            rest(400);
            setPwmLR(6,23,Mode_speed_constant,1);
            break;
        case 11:
            rest(200);
            setPwmLR(-3,-15,Mode_speed_constant,1);
            break;            
        case 12:
            rest(200);
            setPwmLR(15,3,Mode_speed_constant,1);
            break;
        case 21:
            rest(200);
            setPwmLR(-10,10,Mode_speed_constant,1);
            break;
        case 22:
            rest(200);
            setPwmLR(10,-10,Mode_speed_constant,1);
            break;  
        case 23:
            rest(200);
            setPwmLR(30,30,Mode_speed_decreaseLR,30);
            break; 
        case 24:
            rest(200);
            setPwmLR(10,-10,Mode_speed_constant,1);
            break;
        case 30:
            rest(200);
            setPwmLR(30,30,Mode_speed_constant,1);
            break;
        case 31:
            rest(200);  
            pwm_L = _EDGE * -9;
            pwm_R = _EDGE * 10;
            setPwmLR(pwm_L,pwm_R,Mode_speed_constant,1);
            break;
        case 32:
            rest(200);  
            pwm_L = _EDGE * 10;
            pwm_R = _EDGE * -9;
            setPwmLR(pwm_L,pwm_R,Mode_speed_constant,1);
            break;
        case 33:
            if (_LEFT == 1){
                setPwmLR(27,24,Mode_speed_constant,1);
            }else{
                setPwmLR(24,27,Mode_speed_constant,1);
            }
            break;
        case 40:
            rest(200);
            setPwmLR(15,-13,Mode_speed_constant,1);
            break;
        case 50:
            rest(200);            
            if (_LEFT == 1){
                setPwmLR(20,25,Mode_speed_increaseR,75);
            }else{
                setPwmLR(25,20,Mode_speed_increaseL,75);
            }
            break;
        case 60:
            rest(200);  
            if (_LEFT == 1){
                setPwmLR(-3,22,Mode_speed_constant,1);
                clock->sleep(200);
                setPwmLR(10,27,Mode_speed_constant,1);
            }else{
                setPwmLR(15,2,Mode_speed_decreaseL,1);
            }
            break;
        case 61:
            rest(200);  
            if (_LEFT == 1){
                setPwmLR(30,0,Mode_speed_constant,1);
                clock->sleep(500);
                setPwmLR(25,15,Mode_speed_constant,1);
            }else{
                setPwmLR(0,30,Mode_speed_constant,1);
                clock->sleep(500);
                setPwmLR(15,25,Mode_speed_constant,1);
            }
            break;       
        case 70:
            rest(200);
            setPwmLR(25,25,Mode_speed_constant,1);
            break;
        case 71:
            rest(200);
            if (_LEFT == 1){
                setPwmLR(-12,18,Mode_speed_constant,1);
                //clock->sleep(1000);
            }else{
                setPwmLR(18,-12,Mode_speed_constant,1);
                //clock->sleep(1000);
            }
            break;
        case 80:
            if (_LEFT == 1){
                setPwmLR(28,28,Mode_speed_constant,1);
            }else{
                setPwmLR(28,28,Mode_speed_constant,1);
            }
            break;
        case 90:
            pwm_L = _EDGE * 12;
            pwm_R = _EDGE * -11;
            setPwmLR(pwm_L,pwm_R,Mode_speed_constant,1);
            break;
        case 100:
            if (_LEFT == 1){
                setPwmLR(25,18,Mode_speed_decreaseL,180);
            }else{
                setPwmLR(18,25,Mode_speed_decreaseR,180);
            }
            break;
        case 101:
            if (_LEFT == 1){
                rest(200);
                setPwmLR(8,-8,Mode_speed_constant,1);
                clock->sleep(500);
                setPwmLR(25,18,Mode_speed_decreaseL,200);
            }else{
                rest(200);
                setPwmLR(-8,8,Mode_speed_constant,1);
                clock->sleep(500);
                setPwmLR(18,25,Mode_speed_decreaseR,200);
            }
            break;
        case 102:
            if (_LEFT == 1){
                rest(200);
                setPwmLR(-8,8,Mode_speed_constant,1);
                clock->sleep(460);
                setPwmLR(25,18,Mode_speed_decreaseL,200);
            }else{
                rest(200);
                setPwmLR(8,-8,Mode_speed_constant,1);
                clock->sleep(460);
                setPwmLR(18,25,Mode_speed_decreaseR,200);
            }
            break;
        case 112:
            rest(200);
            if (_LEFT == 1){
                setPwmLR(-25,-25,Mode_speed_constant,1);
            }else{
                setPwmLR(-25,-25,Mode_speed_constant,1);
            }
            break;
        case 114:
            rest(200);
            pwm_L = _EDGE * 11;
            pwm_R = _EDGE * -8;
            setPwmLR(pwm_L,pwm_R,Mode_speed_constant,1);
            //setPwmLR(2,2,Mode_speed_constant,1);
            break;
        case 121:
            rest(200);
            if (_LEFT == 1){
                setPwmLR(26,30,Mode_speed_increaseL,120);
            }else{
                setPwmLR(30,26,Mode_speed_increaseR,120);
            }
            break;
        case 130:
            rest(200);
            if (_LEFT == 1){
                setPwmLR(-14,15,Mode_speed_constant,1);
            }else{
                setPwmLR(15,-14,Mode_speed_constant,1);
            }
            break;
        case 131:
            rest(200);
            if (_LEFT == 1){
                setPwmLR(17,16,Mode_speed_constant,1);
            }else{
                setPwmLR(16,17,Mode_speed_constant,1);
            }
            break;
        case 132:
            rest(200);
            if (_LEFT == 1){
                setPwmLR(40,10,Mode_speed_increaseR,10);
            }else{
                setPwmLR(10,40,Mode_speed_increaseL,10);
            }
            break;
        case 133:
            pwm_L = _EDGE * 10;
            pwm_R = _EDGE * -10;
            setPwmLR(pwm_L,pwm_R,Mode_speed_constant,1);
            break;
        case 141:
            if (_LEFT == 1){
                setPwmLR(35,33,Mode_speed_constant,1);
            }else{
                setPwmLR(33,35,Mode_speed_constant,1);
            }
            break;
        case 150:
            setPwmLR(30,30,Mode_speed_constant,1);
            break;
        //ボーナスブロック専用処理
        case 151:
            rest(500);
            pwm_L = _EDGE * 10;
            pwm_R = _EDGE * -10;
            setPwmLR(pwm_L,pwm_R,Mode_speed_constant,1);
            break;
        case 152:
            rest(200);
            pwm_L = _EDGE * 12;
            pwm_R = _EDGE * -12;
            setPwmLR(pwm_L,pwm_R,Mode_speed_constant,1);
            break;
        case 160:
            rest(200);
            pwm_L = _EDGE * 8;
            pwm_R = _EDGE * -8;
            setPwmLR(pwm_L,pwm_R,Mode_speed_constant,1);
            break;
        case 161:
            rest(200);
            pwm_L = _EDGE * -8;
            pwm_R = _EDGE * 8;
            setPwmLR(pwm_L,pwm_R,Mode_speed_constant,1);
            break;
        case 170:
            setPwmLR(50,50,Mode_speed_constant,1);
            break;
        case 180:
            setPwmLR(30,30,Mode_speed_decreaseLR,100);
            break;
        case 190:
            rest(500);
            if (_LEFT == 1){
                setPwmLR(15,-17,Mode_speed_constant,1);
            }else{
                setPwmLR(-17,15,Mode_speed_constant,1);
            }
            //clock->sleep(680);
            //rest(900);
            break;
        case 193:
            rest(700);
            setPwmLR(30,30,Mode_speed_constant,1);
            break;
        case 200:
            rest(600);
            if (_LEFT == 1){
                setPwmLR(10,0,Mode_speed_constant,1);
            }else{
                setPwmLR(0,10,Mode_speed_constant,1);
            }
            break;
        case 201:
            setPwmLR(30,30,Mode_speed_constant,1);
            break;
        case 210:
            if (_LEFT == 1){
                setPwmLR(31,24,Mode_speed_constant,1);
            }else{
                setPwmLR(24,31,Mode_speed_constant,1);
            }
            break;
        case 211:
            setPwmLR(30,30,Mode_speed_constant,1);
            break;
        case 213:
            setPwmLR(30,29,Mode_speed_constant,1);
            break;
        case 230:
            rest(200);
            if (_LEFT == 1){
                setPwmLR(-9,20,Mode_speed_constant,1);
            }else{
                setPwmLR(20,-9,Mode_speed_constant,1);
            }
            break;
        case 231:
            rest(200);
            setPwmLR(30,30,Mode_speed_constant,1);
            break;
        case 233:
            rest(200);
            setPwmLR(-20,20,Mode_speed_constant,1);
            break;
        case 240:
            setPwmLR(30,30,Mode_speed_constant,1);
            break;
        case 244:
            if (_LEFT == 1){
                setPwmLR(4,30,Mode_speed_constant,1);
            }else{
                setPwmLR(30,4,Mode_speed_constant,1);
            }
            break;
        case 260:
            if (_LEFT == 1){
                setPwmLR(4,30,Mode_speed_constant,1);
            }else{
                setPwmLR(30,4,Mode_speed_constant,1);
            }
            break;
         case 261:
            if (_LEFT == 1){
                setPwmLR(2,20,Mode_speed_constant,1);
            }else{
                setPwmLR(20,2,Mode_speed_constant,1);
            }
            break;
        case 263:
            setPwmLR(50,50,Mode_speed_decreaseLR,300);
            break;
        case 264:
            setPwmLR(-20,20,Mode_speed_decreaseLR,300);
            break;
        case 265:
            setPwmLR(50,50,Mode_speed_decreaseLR,300);
            break;
         case 271:
            if (_LEFT == 1){
                rest(300);
                setPwmLR(10,2,Mode_speed_constant,1);
            }else{
                rest(300);
                setPwmLR(2,10,Mode_speed_constant,1);
            }
            break;
        case 272:
            setPwmLR(50,50,Mode_speed_decreaseLR,300);
            break;
        case 280:
            setPwmLR(30,30,Mode_speed_decreaseLR,180);
            break;
        case 282:
            if (_LEFT == 1){
                setPwmLR(22,20,Mode_speed_decreaseLR,80);
            }else{
                setPwmLR(22,20,Mode_speed_decreaseLR,80);
            }
            break;
        case 284:
            //rest(200);
            if (_LEFT == 1){
                setPwmLR(11,-9,Mode_speed_constant,1);
            }else{
                setPwmLR(-9,11,Mode_speed_constant,1);
            }
            break;
        case 286:
            setPwmLR(25,25,Mode_speed_constant,1);
            break;
        case 290:
            setPwmLR(0,0,Mode_speed_constant,1);
            freeze();
            break;
        default:
            break;
    }
}

//　左右の車輪に駆動にそれぞれ値を指定する
void ChallengeRunner::setPwmLR(int p_L,int p_R,int mode,int proc_count) {
    pwm_L = p_L;
    pwm_R = p_R;
    pwmMode = mode;
    procCount = proc_count;
    count = 0;
}

// rest for a while
void ChallengeRunner::rest(int16_t rest_time) {
    freeze();
    clock->sleep(rest_time);
    unfreeze();
}

int8_t ChallengeRunner::getPwmL() {
    return pwm_L;
}

int8_t ChallengeRunner::getPwmR() {
    return pwm_R;
}

ChallengeRunner::~ChallengeRunner() {
    _debug(syslog(LOG_NOTICE, "%08u, ChallengeRunner destructor", clock->now()));
}