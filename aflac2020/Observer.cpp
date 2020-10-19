//
//  Observer.cpp
//  aflac2020
//
//  Copyright © 2019 Ahiruchan Koubou. All rights reserved.
//

#include "app.h"
#include "Observer.hpp"
#include "StateMachine.hpp"

//DataLogger angLLogger("angL",10);
//DataLogger angRLogger("angR",10);

// global variables to pass FIR-filtered color from Observer to Navigator and its sub-classes
rgb_raw_t g_rgb;
hsv_raw_t g_hsv;
int16_t g_grayScale, g_grayScaleBlueless;
// global variables to gyro sensor output from Observer to  Navigator and its sub-classes
int16_t g_angle, g_anglerVelocity;
int16_t g_challenge_stepNo, g_color_brightness;


Observer::Observer(Motor* lm, Motor* rm, Motor* am, Motor* tm, TouchSensor* ts, SonarSensor* ss, GyroSensor* gs, ColorSensor* cs) {
    _debug(syslog(LOG_NOTICE, "%08u, Observer constructor", clock->now()));
    leftMotor   = lm;
    rightMotor  = rm;
    armMotor = am;
    tailMotor = tm;
    touchSensor = ts;
    sonarSensor = ss;
    gyroSensor  = gs;
    colorSensor = cs;

    distance = azimuth = locX = locY = aveDiffAng = deltaDiff = prevDeltaDiff = 0.0;
    prevAngL = prevAngR = 0;
    integD = integDL = integDR = 0.0; // temp

    notifyDistance = 0;
    traceCnt = 0;
    diffAng = 0;
    sumDiffAng = 0;
    countAng = 0;
    prevGS = INT16_MAX;
    touch_flag = false;
    sonar_flag = false;
    backButton_flag = false;
    lost_flag = false;
    blue_flag = false;
    
    // For Challenge Run
    line_over_flg = false;
    move_back_flg = false;
    slalom_flg = false;
    curRgbSum = 0;
    prevRgbSum = 0;
    curAngle = 0;
    prevAngle = 0;
    curDegree180 = 0;
    prevDegree180 = 0;
    curDegree360 = 0;
    prevDegree360 = 0;
    accumuDegree = 0;
    g_challenge_stepNo = 0;
    curSonarDis = 0;
    prevSonarDis = 0;
    prevDis = 0;
    prevDisX = 0;
    prevDisY = 0;
    root_no = 0;
    turnDegree=0;
    tempDis = 0;
    line_on_stat=0;
    exception_stat=0;
    gyroSensor->setOffset(0);

    fir_r = new FIR_Transposed<FIR_ORDER>(hn);
    fir_g = new FIR_Transposed<FIR_ORDER>(hn);
    fir_b = new FIR_Transposed<FIR_ORDER>(hn);
    ma = new MovingAverage<int32_t, MA_CAP>();
}

void Observer::activate() {
    // register cyclic handler to EV3RT
    sta_cyc(CYC_OBS_TSK);
    //clock->sleep() seems to be still taking milisec parm
    clock->sleep(PERIOD_OBS_TSK/2/1000); // wait a while
    _debug(syslog(LOG_NOTICE, "%08u, Observer handler set", clock->now()));
}

void Observer::reset() {
    distance = azimuth = locX = locY = 0.0;
    prevAngL = leftMotor->getCount();
    prevAngR = rightMotor->getCount();
    integD = integDL = integDR = 0.0; // temp
}

void Observer::notifyOfDistance(int32_t delta) {
    notifyDistance = delta + distance;
}

int32_t Observer::getDistance() {
    return (int32_t)distance;
}

int16_t Observer::getAzimuth() {
    // degree = 360.0 * radian / M_2PI;
    int16_t degree = (360.0 * azimuth / M_2PI);
    return degree;
}

int16_t Observer::getDegree() {
    // degree = 360.0 * radian / M_2PI;
    int16_t degree = (360.0 * azimuth / M_2PI);
    if (degree > 180){
        degree -= 360;
    }
    return degree;
}

int32_t Observer::getLocX() {
    return (int32_t)locX;
}

int32_t Observer::getLocY() {
    return (int32_t)locY;
}

void Observer::operate() {
    colorSensor->getRawColor(cur_rgb);
    // process RGB by the Low Pass Filter
    cur_rgb.r = fir_r->Execute(cur_rgb.r);
    cur_rgb.g = fir_g->Execute(cur_rgb.g);
    cur_rgb.b = fir_b->Execute(cur_rgb.b);
    curRgbSum = cur_rgb.r + cur_rgb.g + cur_rgb.b;
    rgb_to_hsv(cur_rgb, cur_hsv);
    // save filtered color variables to the global area
    g_rgb = cur_rgb;
    g_hsv = cur_hsv;
    // calculate gray scale and save them to the global area
    if(!garage_flg){
        g_grayScale = (cur_rgb.r * 77 + cur_rgb.g * 150 + cur_rgb.b * 29) / 256;
        g_grayScaleBlueless = (cur_rgb.r * 77 + cur_rgb.g * 150 + (cur_rgb.b - cur_rgb.g) * 29) / 256; // B - G cuts off blue
    }else{
        g_grayScale = (cur_rgb.r * 200 + cur_rgb.g * 10 + cur_rgb.b * 29) / 239;
        //g_grayScaleBlueless = (cur_rgb.r * 200 + cur_rgb.g * 10 + (cur_rgb.b - cur_rgb.g) * 29) / 239; // B - G cuts off blue
        //g_grayScaleBlueless = (cur_rgb.r * 77 + cur_rgb.g * 120 + (cur_rgb.b - cur_rgb.g) * 29) / 226; // B - G cuts off blue        g_color_brightness = colorSensor->getBrightness();
        g_grayScaleBlueless = (cur_rgb.r * 77 + cur_rgb.g * 140 + (cur_rgb.b - cur_rgb.g) * 29) / 246; // B - G cuts off blue        g_color_brightness = colorSensor->getBrightness();
    }

    // save gyro sensor output to the global area
    g_angle = gyroSensor->getAngle();
    g_anglerVelocity = gyroSensor->getAnglerVelocity();

    // accumulate distance
    int32_t curAngL = leftMotor->getCount();
    // angLLogger.logging(curAngL);
    int32_t curAngR = rightMotor->getCount();
    // angRLogger.logging(curAngR);
    double deltaDistL = M_PI * TIRE_DIAMETER * (curAngL - prevAngL) / 360.0;
    double deltaDistR = M_PI * TIRE_DIAMETER * (curAngR - prevAngR) / 360.0;
    double deltaDist = (deltaDistL + deltaDistR) / 2.0;
    distance += deltaDist;
    prevAngL = curAngL;
    prevAngR = curAngR;
    // calculate azimuth
    double deltaAzi = atan2((deltaDistL - deltaDistR), WHEEL_TREAD);
    azimuth += deltaAzi;
    if (azimuth > M_2PI) {
        azimuth -= M_2PI;
    } else if (azimuth < 0.0) {
        azimuth += M_2PI;
    }
    // estimate location
    locX += (deltaDist * sin(azimuth));
    locY += (deltaDist * cos(azimuth));

// modify start by Furuta 2020.09.23
    // monitor good timing to swith to BlindRunner
    if (countAng != -1) {
        if ((curAngL + curAngR) < AVERAGE_START) {
            // cutoff initial noize
            // syslog(LOG_NOTICE, "%08u, initial cutting off", clock->now());         
        } else if (((curAngL + curAngR) >= AVERAGE_START) && ((curAngL + curAngR) < AVERAGE_END)) {
            // calculate average of delta of angL and andR
            diffAng = curAngL - curAngR;
            sumDiffAng += diffAng;
            countAng ++;
            // syslog(LOG_NOTICE, "%08u, diffAng = %d, sum = %d, cnt = %d", clock->now(), diffAng, sumDiffAng, countAng);
        } else if (((curAngL + curAngR) >= AVERAGE_END)) {
            // judge whether should switch to BlindRunner
            aveDiffAng = (double)sumDiffAng / countAng;
            diffAng = curAngL - curAngR;
            prevDeltaDiff = deltaDiff;
            deltaDiff = diffAng - aveDiffAng;
            // syslog(LOG_NOTICE, "%08u, diffAng = %d, sum = %d, cnt = %d, ave = %f", clock->now(), diffAng, sumDiffAng, countAng, aveDiffAng);
           if (((deltaDiff < 0.0) && (prevDeltaDiff > 0.0)) ||
               ((deltaDiff > 0.0) && (prevDeltaDiff < 0.0))) {
                // Delta has been across Average
                syslog(LOG_NOTICE, "%08u, diffAng = %d, sum = %d, cnt = %d", clock->now(), diffAng, sumDiffAng, countAng);
                syslog(LOG_NOTICE, "%08u, Pass control to BlindRunner, prev = %d, delta = %d", clock->now(), (int)(prevDeltaDiff*1000), (int)(deltaDiff*1000)); 
                countAng = -1;  // event to be sent only once
                stateMachine->sendTrigger(EVT_robot_aligned);
            }
        }
    }

    // monitor distance
    if ((notifyDistance != 0.0) && (distance > notifyDistance)) {
         syslog(LOG_NOTICE, "%08u, distance reached", clock->now());
         notifyDistance = 0.0; // event to be sent only once
         stateMachine->sendTrigger(EVT_dist_reached);
    }
    
    // monitor touch sensor
    bool result = check_touch();
    if (result && !touch_flag) {
        syslog(LOG_NOTICE, "%08u, TouchSensor flipped on", clock->now());
        touch_flag = true;
        stateMachine->sendTrigger(EVT_touch_On);
    } else if (!result && touch_flag) {
        syslog(LOG_NOTICE, "%08u, TouchSensor flipped off", clock->now());
        touch_flag = false;
        stateMachine->sendTrigger(EVT_touch_Off);
    }
    
    // monitor sonar sensor
    result = check_sonar();
    if (result && !sonar_flag) {
        syslog(LOG_NOTICE, "%08u, SonarSensor flipped on", clock->now());
        sonar_flag = true;
        stateMachine->sendTrigger(EVT_sonar_On);
    } else if (!result && sonar_flag) {
        syslog(LOG_NOTICE, "%08u, SonarSensor flipped off", clock->now());
        sonar_flag = false;
        stateMachine->sendTrigger(EVT_sonar_Off);
    }
    
    // monitor Back Button
    result = check_backButton();
    if (result && !backButton_flag) {
        syslog(LOG_NOTICE, "%08u, Back button flipped on", clock->now());
        backButton_flag = true;
        stateMachine->sendTrigger(EVT_backButton_On);
    } else if (!result && backButton_flag) {
        syslog(LOG_NOTICE, "%08u, Back button flipped off", clock->now());
        backButton_flag = false;
        stateMachine->sendTrigger(EVT_backButton_Off);
    }

    if (!frozen) { // these checks are meaningless thus bypassed when frozen
        // determine if still tracing the line
        // result = check_lost();
        // if (result && !lost_flag) {
        //     syslog(LOG_NOTICE, "%08u, line lost", clock->now());
        //     lost_flag = true;
        //     stateMachine->sendTrigger(EVT_line_lost);
        // } else if (!result && lost_flag) {
        //     syslog(LOG_NOTICE, "%08u, line found", clock->now());
        //     lost_flag = false;
        //     stateMachine->sendTrigger(EVT_line_found);
        // }

        // temporary dirty logic to detect the second black to blue change
        int32_t ma_gs;
        if (prevGS == INT16_MAX) {
            prevTime = clock->now();
            prevGS = g_grayScale;
            ma_gs = ma->add(0);
        } else {
            curTime = clock->now();
            gsDiff = g_grayScale - prevGS;
            timeDiff = curTime - prevTime;
            ma_gs = ma->add(gsDiff * 1000000 / timeDiff);
            prevTime = curTime;
            prevGS = g_grayScale;
        }

        if ( (ma_gs > 150) || (ma_gs < -150) ){
            //syslog(LOG_NOTICE, "gs = %d, MA = %d, gsDiff = %d, timeDiff = %d", g_grayScale, ma_gs, gsDiff, timeDiff);
            if ( !blue_flag && ma_gs > 150 && g_rgb.b - g_rgb.r > 60 && g_rgb.b <= 255 && g_rgb.r <= 255 ) {
                blue_flag = true;
                syslog(LOG_NOTICE, "%08u, line color changed black to blue", clock->now());
                stateMachine->sendTrigger(EVT_bk2bl);
            } else if ( blue_flag && ma_gs < -150 && g_rgb.b - g_rgb.r < 40 ) {
                blue_flag = false;
                syslog(LOG_NOTICE, "%08u, line color changed blue to black", clock->now());
                stateMachine->sendTrigger(EVT_bl2bk);
            }
        }

        // determine if tilt
        // if ( check_tilt() ) {
        //     stateMachine->sendTrigger(EVT_tilt);
        // }
    }
    
    curDegree180 = getDegree();
    curDegree360 = getAzimuth();
    curSonarDis = sonarSensor->getDistance();


    //ゴールからスラロームに移行する前のエラー処理
     if(g_challenge_stepNo == 900 ){
         prevDis=distance;
         g_challenge_stepNo = 901;
     }else if(g_challenge_stepNo == 901){

        //ライントレースせず、緑に落ちるぽたーん
         if(cur_rgb.r <= 13 && cur_rgb.b <= 50 && cur_rgb.g > 60){
             printf("緑来ました prevDis=%lf distance=%lf sa=%lf\n",prevDis,distance,distance-prevDis);
             //stateMachine->sendTrigger(EVT_green_on);
             if(own_abs(distance-prevDis)>130){
                 root_no=1;
             }else{
                 root_no=2;
             }
             g_challenge_stepNo = 902;
         }
     }else if(g_challenge_stepNo == 902){
        //黒または青をとらえたら
        printf("g_challenge_stepNo = 903 r=%d,g=%d,b=%d,state=%d\n",cur_rgb.r,cur_rgb.g, cur_rgb.b,state);

        if((cur_rgb.g + cur_rgb.b <= 100 && cur_rgb.r <=50 && cur_rgb.g <=40 && cur_rgb.b <=60) || cur_rgb.b - cur_rgb.r >= 60 ){
            printf("黒来ました prevDis=%lf distance=%lf sa=%lf\n",prevDis,distance,distance-prevDis);
            //stateMachine->sendTrigger(EVT_line_on);
            g_challenge_stepNo = 903;
        }
    }else if(g_challenge_stepNo ==903){
        if(root_no == 2){
            clock->sleep(600);
        }else if(root_no==1){
            clock->sleep(700);
        }
        printf("state=%d\n",state);
        stateMachine->sendTrigger(EVT_black_found);
        prevDis=distance;
        g_challenge_stepNo = 904;
    }else if(g_challenge_stepNo == 904 && distance - prevDis > 80){
        stateMachine->sendTrigger(EVT_distance_over);
        g_challenge_stepNo = 905;
    }

    // //ゴール後に脱線して緑に行ってしまった時の例外処理
    // if(g_challenge_stepNo == 910 && exception_stat==0){
    //     exception_stat = 1;
    // }else if( exception_stat == 1 && cur_rgb.r <= 13 && cur_rgb.b <= 50 && cur_rgb.g > 60){
    //     printf("例外緑です\n");
    //     g_challenge_stepNo = 920;
    //     exception_stat=2;

    // //ライントレース後、一定距離走行したら速度を上げる
    // }else if( g_challenge_stepNo == 930 && g_challenge_stepNo == 0){
    //     printf("ライントレース入りました\n");
    //     prevDis = distance;
    //     g_challenge_stepNo = 0;
    //     line_on_stat =1;
    // }else if (line_on_stat == 1 && own_abs(distance-prevDis) > 100){
    //     printf("速度速めました\n");
    //     g_challenge_stepNo = 940;
    //     line_on_stat=2;
    // }
    
    // Preparation for slalom climbing
    if(!slalom_flg && !garage_flg){
        if ((g_challenge_stepNo >= 900 && g_challenge_stepNo <= 905 ) && check_sonar(1,8) && !move_back_flg){
            //state = ST_slalom; // cause of defect - removed on Oct.17g_challenge_stepNo
            g_challenge_stepNo = 0;
            printf("スラロームに近い\n");
            armMotor->setPWM(-50);
            stateMachine->sendTrigger(EVT_slalom_reached);
            g_challenge_stepNo = 1;

            locX = 0;
            locY = 0; 
            distance = 0;
            prevAngL =0;
            prevAngR =0;
            leftMotor->reset();
            rightMotor->reset();
            azimuth = 0;

            stateMachine->sendTrigger(EVT_slalom_reached);
            armMotor->setPWM(60);
            g_challenge_stepNo = 10;
            move_back_flg = true;
        }

        // Robot tilts whenn
        curAngle = g_angle;
        if(curAngle < -9){
            prevAngle = curAngle;
        }
        if (prevAngle < -9 && curAngle >= 0){
            printf("スラロームオン\n");
            slalom_flg = true;
            curAngle = 0;
            prevAngle = 0;
            prevDis = distance;
            prevDisY = locY;
            prevDegree180=getDegree();
            armMotor->setPWM(-100);
        }
    }

    //スラローム専用処理
    if(slalom_flg && !garage_flg){
        //ログ出力
        // if ((g_challenge_stepNo >= 30 && g_challenge_stepNo <= 32) || (g_challenge_stepNo >= 300 && g_challenge_stepNo <= 300)){
        //      if (++traceCnt && traceCnt > 1) {
        //          printf(",locX=%lf,prevDisX=%lf,locX - prevDisX=%lf,locY=%lf,locY - prevDisY=%lf,distance=%lf,prevDis=%lf,g_angle=%d,curDegree180=%d, curSonarDis=%d, g_challenge_stepNo=%d,r+g+b=%d\n",locX,prevDisX,locX - prevDisX,locY,locY - prevDisY,distance,prevDis,g_angle,curDegree180, curSonarDis,g_challenge_stepNo,curRgbSum);
        //          traceCnt = 0;
        //      }
        // }

        //初期位置の特定
        if(g_challenge_stepNo == 10 && distance - prevDis > 30){
            prevDisX = locX;
            prevRgbSum = curRgbSum;
            if(curRgbSum < 100){
                //もともと黒の上にいる場合、ライン下方面に回転
                prevDisX = locX;
                printf("もともと黒の上\n");
                g_challenge_stepNo = 20;
            }else{
                //左カーブ
                stateMachine->sendTrigger(EVT_slalom_challenge);
                g_challenge_stepNo = 11;
            }
        }else if(g_challenge_stepNo == 11){
            //センサーで黒を検知した場合
            if(curRgbSum < 100 ){
                //その場でライン下方面に回転
                prevDisX = locX;
                g_challenge_stepNo = 20;
            }
            //左カーブで黒を見つけられなかった場合、リバース、元の位置まで戻る
            else if(own_abs(prevDisX-locX) > 180 ){
                if(prevRgbSum - curRgbSum > 20){
                    //黒に近づいていたら、もう少し頑張る
                }else{
                    //黒の片鱗も見えなければ逆走する
                    stateMachine->sendTrigger(EVT_slalom_challenge);
                    g_challenge_stepNo = 12;
                }
            }
        //左カーブから戻る
        }else if(g_challenge_stepNo == 12){
            if(prevDisX - locX <= 0){
                //右カーブへ移行
                stateMachine->sendTrigger(EVT_slalom_challenge);
                g_challenge_stepNo = 13;
            
            //途中で黒を検知した場合、左下へ移動
            }else if(curRgbSum < 100 ){
                //その場でライン下方面に回転
                prevDisX = locX;
                g_challenge_stepNo = 20;
            }

        }else if(g_challenge_stepNo == 13){
            //センサーで黒を検知した場合
            if(curRgbSum < 100 ){
                //その場でライン下方面に回転
                prevDisX = locX;
                stateMachine->sendTrigger(EVT_slalom_challenge);
                g_challenge_stepNo = 20;
            }        

        }else if(g_challenge_stepNo == 20 && curDegree180 >= -43){
            //その場で左回転
            g_challenge_stepNo = 21;
            stateMachine->sendTrigger(EVT_slalom_challenge);//21
            root_no = 1;
            printf("上から回転 %d\n",curDegree180);

        }else if(g_challenge_stepNo == 20 && curDegree180 < -43){
            //その場で右回転
            g_challenge_stepNo = 22;
            stateMachine->sendTrigger(EVT_slalom_challenge);//22
            root_no = 2;
            printf("下から回転　%d\n",curDegree180);

        }else if((g_challenge_stepNo == 21 && curDegree180 <= -43) || (g_challenge_stepNo == 22 && curDegree180 >= -43)){
            //角度が一定角度になったら、停止、直進
            printf("角度　%d\n",curDegree180);
            g_challenge_stepNo = 23;
            stateMachine->sendTrigger(EVT_slalom_challenge); //23
            
            g_challenge_stepNo = 24;
        //進行方向に対して横軸に距離進んだら、角度を0度に戻す
        } else if(g_challenge_stepNo == 24){
            if((root_no == 1 && own_abs(prevDisX - locX) > 128) || (root_no ==2 && own_abs(prevDisX - locX) > 123)){
                printf(",進行方向に対して横軸に距離進んだら、角度を0度に戻す\n");
                stateMachine->sendTrigger(EVT_slalom_challenge); //24
                g_challenge_stepNo = 30;
            }
        //進行方向に対して横軸に進んだら、角度を戻す
        }else if(g_challenge_stepNo == 30){
 //           if((root_no == 1 && curDegree180 > prevDegree180 + 1 ) || (root_no == 2 && curDegree180 > prevDegree180 + 5)){
            if((root_no == 1 && curDegree180 > prevDegree180 ) || (root_no == 2 && curDegree180 > prevDegree180)){

                printf(",進行方向に対して横軸に進んだら、角度を戻すroot_no=%d prevDegree180=%d curDegree180=%d\n",root_no,prevDegree180,curDegree180);
                stateMachine->sendTrigger(EVT_slalom_challenge);
                g_challenge_stepNo = 40;
                prevSonarDis = curSonarDis;
                prevDisX = locX;
            }
        }else if(g_challenge_stepNo == 40 && locX - prevDisX > 15 && locY - prevDisY < 400 && curSonarDis > prevSonarDis ){
                printf(",方向を下に調整する\n");
                g_challenge_stepNo = 31;
                stateMachine->sendTrigger(EVT_slalom_challenge);
                prevDegree180 = curDegree180;
        }else if(g_challenge_stepNo == 40 && locX - prevDisX < -3 && locY - prevDisY < 450){
                printf(",方向を上に調整する\n");
                g_challenge_stepNo = 32;
                stateMachine->sendTrigger(EVT_slalom_challenge);
                prevDegree180 = curDegree180;
//        }else if((g_challenge_stepNo == 31 || g_challenge_stepNo == 32) && own_abs(prevDegree180 - curDegree180) >= 10 && check_sonar(0,20)){
        }else if((g_challenge_stepNo == 31 || g_challenge_stepNo == 32) && ((own_abs(prevDegree180 - curDegree180) >= 15 && check_sonar(0,20)) || own_abs(prevDegree180 - curDegree180) >= 20)){
                printf(",調整完了\n");
                g_challenge_stepNo = 33;
                stateMachine->sendTrigger(EVT_slalom_challenge);
                g_challenge_stepNo = 34;
        //  ２つ目の障害物に接近したら向きを変える
        }else if ((g_challenge_stepNo == 34 || g_challenge_stepNo == 40) && (locY - prevDisY > 525 || check_sonar(0,5))){
            printf(",２つ目の障害物に接近したら向きを変える,g_challenge_stepNo=%d,locY = %lf,prevDisY = %lf ,curSonarDis=%d,locX=%lf,prevDisX=%lf\n",g_challenge_stepNo,locY,prevDisY,curSonarDis,locX,prevDisX);
            g_challenge_stepNo = 40;
            stateMachine->sendTrigger(EVT_slalom_challenge);
            g_challenge_stepNo = 50;
            prevRgbSum = curRgbSum;
            line_over_flg = false;
            prevRgbSum = curRgbSum;
        //  視界が晴れたら左上に前進する
        }else if(g_challenge_stepNo == 50  && (check_sonar(50,255)|| own_abs(curDegree180 - prevDegree180) > 55)){
            printf(",視界が晴れたところで前進する ソナー=%d curDegree180=%d prevDegree180=%d",curSonarDis,curDegree180,prevDegree180);
            stateMachine->sendTrigger(EVT_slalom_challenge);
            g_challenge_stepNo = 60;
            prevRgbSum = curRgbSum;
            line_over_flg = false;
        //  黒ラインを超えるまで前進し、超えたら向きを調整し３つ目の障害物に接近する
        }else if (g_challenge_stepNo == 60 && !line_over_flg){
            if (curRgbSum < 100) {
                prevRgbSum = curRgbSum;
            }
            if(prevRgbSum < 100 && curRgbSum > 125){
                printf(",黒ラインを超えたら向きを調整し障害物に接近する\n");
                stateMachine->sendTrigger(EVT_slalom_challenge);
                //prevDis=distance;
                prevDisX = locX;
                g_challenge_stepNo = 61;
                prevDegree180 = curDegree180;
                line_over_flg = true;
                g_challenge_stepNo = 71;
            }else if(locY - prevDisY > 1042){
                //２ピン手前から黒ラインを超えずに横に進んだ場合の例外処理
                g_challenge_stepNo = 61;
                stateMachine->sendTrigger(EVT_slalom_challenge);
                g_challenge_stepNo = 110;
            }
            
        }else if(g_challenge_stepNo == 71 && (locY - prevDisY > 700 || check_sonar(0,5))){
                printf("３ピン手前 locY=%lf, prevDisY=%lf,curSonarDis=%d\n",locY, prevDisY,curSonarDis);
                stateMachine->sendTrigger(EVT_slalom_challenge);
                g_challenge_stepNo = 80;
                line_over_flg = true;
                prevDegree180 = curDegree180;
        // 視界が晴れたら左下に前進する
        }else if (g_challenge_stepNo == 80  && own_abs(prevDegree180 - curDegree180) > 60){
                printf(",視界が晴れたら左下に前進する curDegree180=%d\n",curDegree180);
                stateMachine->sendTrigger(EVT_slalom_challenge);
                accumuDegree = own_abs(-33 - curDegree180); 
                g_challenge_stepNo = 90;
                prevRgbSum = curRgbSum;
                line_over_flg = false;
        // 黒ラインを超えるまで前進し、超えたら向きを調整する
        }else if (g_challenge_stepNo == 90 && !line_over_flg){
            if (curRgbSum < 100) {
                prevRgbSum = curRgbSum;
                prevDisX=locX;
                tempDis = locX;
            }
            if(prevRgbSum < 100 && curRgbSum > 150 && own_abs(prevDisX - locX) > 16){
                printf(",黒ラインを超えたら向きを調整するprevDisX=%lf  locX = %lf \n",prevDisX,locX);
                stateMachine->sendTrigger(EVT_slalom_challenge); //90
                prevDegree180 = curDegree180;
                g_challenge_stepNo = 100;
                line_over_flg = true;
            }

        //３ピン回避中に黒ラインに行ってしまうパターン
        }else if( (g_challenge_stepNo == 80 && curRgbSum < 100) || (g_challenge_stepNo == 71 && curRgbSum < 100) ){
                g_challenge_stepNo = 90;
                clock->sleep(300);
                printf(",例外処理黒ラインを超えたら向きを調整するprevDisX=%lf  locX = %lf \n",prevDisX,locX);
                stateMachine->sendTrigger(EVT_slalom_challenge); //90
                prevDegree180 = curDegree180;
                g_challenge_stepNo = 100;
                prevDisX=locX;
                tempDis = locX;
                line_over_flg = true;

        //  ４つ目の障害物に接近する
        }else if(g_challenge_stepNo == 100 && (own_abs(curDegree180 - prevDegree180) > 52 + accumuDegree || curDegree180 > 5)){ 
            printf(",４つ目の障害物に接近する。locY=%lf,prevDisY=%lf,prevDegree180=%d ,curDegree180=%d,locX=%lf,prevDisX=%lf, curSonarDis=%d\n",locY,prevDisY,prevDegree180,curDegree180,locX,prevDisX,curSonarDis);
            stateMachine->sendTrigger(EVT_slalom_challenge);
            prevDisX=locX;
            prevDis=distance;
            g_challenge_stepNo = 101;
        
        }else if(g_challenge_stepNo == 101 && own_abs(distance - prevDis) > 25 ){
            if(locX - prevDisX < -1.5){ // 下向き
                printf("傾きチェック1　prevDisX =%lf locX=%lf\n",prevDisX,locX);
                stateMachine->sendTrigger(EVT_slalom_challenge); //101               
            }else if(locX - prevDisX > 4){ // 上向き
                printf("傾きチェック2　prevDisX =%lf locX=%lf\n",prevDisX,locX);
                g_challenge_stepNo = 102;
                stateMachine->sendTrigger(EVT_slalom_challenge); //102          
            }
            printf("傾きチェック3　prevDisX =%lf locX=%lf\n",prevDisX,locX);
            g_challenge_stepNo = 110;
        
        //４ピン手前で下に行き過ぎる例外処理 110 tempDis-locX
        //}else if(){

        // ４つ目の障害物に接近したら向きを変える
        }else if (g_challenge_stepNo == 110 && (check_sonar(0,5) || (locY - prevDisY >= 1040 && check_sonar(255,255)))){
            printf(",４つ目の障害物に接近したら向きを変えるlocX=%lf,tempDis=%lf,prevDisX=%lf,locY=%lf,prevDisY=%lf,curDegree180=%d,curSonarDis=%d\n",locX,tempDis,prevDisX,locY,prevDisY,curDegree180,curSonarDis);
            //チェックソナーとcurSonarDisの値がずれるための措置
            int16_t temp_dis = sonarSensor->getDistance();
            if( check_sonar(0,5) && temp_dis - 4 > 0){
                g_challenge_stepNo = 111;
                prevDisY = locY;
                prevDis = 8 * (temp_dis-4);
                printf("ソナーで４以上、ちょっと進んで調整 temp_dis=%d curDegree180=%d\n",temp_dis,curDegree180);
            }else if(temp_dis == 3){
                g_challenge_stepNo = 112;
                prevDisY = locY;
                printf("ソナーで４以上、ちょっと戻って調整 temp_dis=%d curDegree180=%d\n",temp_dis,curDegree180);
                stateMachine->sendTrigger(EVT_slalom_challenge);//112
                
            }else{
                g_challenge_stepNo = 113;
                printf("距離調整なし temp_dis=%d curDegree180=%d\n",temp_dis,curDegree180);
            }

        }else if (g_challenge_stepNo == 113 ||  (g_challenge_stepNo == 112 && locY - prevDisY <= -10) || (g_challenge_stepNo == 111 && locY - prevDisY >= prevDis )){
                g_challenge_stepNo = 114;
                stateMachine->sendTrigger(EVT_slalom_challenge);
                prevDegree180 = curDegree180;

        //step110直後に視界が晴れる可能性を回避
        }else if(g_challenge_stepNo == 114 && own_abs(curDegree180 - prevDegree180) > 40){
            g_challenge_stepNo = 120;
        
        // 視界が晴れたら左上に前進する
        }else if (g_challenge_stepNo == 120 && check_sonar(15,255)){
            g_challenge_stepNo = 121;
            printf("ソナーに引っかかった。\n");
            prevDegree180 = curDegree180;
        }else if (g_challenge_stepNo == 121 && own_abs(curDegree180 - prevDegree180) > 45){
            printf(",視界が晴れたら左上に前進する,ソナー=%d,角度=%d curDegree180=%d\n",curSonarDis,own_abs(curDegree180 - prevDegree180) ,curDegree180);
            stateMachine->sendTrigger(EVT_slalom_challenge);
            g_challenge_stepNo = 130;
            prevRgbSum = curRgbSum;
            line_over_flg = false;
        // 黒ラインを２つ目まで前進し、２つ目に載ったら向きを調整する
        }else if (g_challenge_stepNo == 130){
            if (!line_over_flg){
                if (curRgbSum < 100) {
                    prevRgbSum = curRgbSum;
                    prevDisX = locX;
                }
                if(prevRgbSum < 100 && curRgbSum > 160){
                    line_over_flg = true;
                
                //黒ラインを超えなかった場合の例外処理
                }else if(locX - prevDisX >= 150){
                    line_over_flg = true;
                }
            }else{
                if(locX - prevDisX >= 55){
                    printf(",左に向きを調整する curDegree180=%d,locX=%lf prevDisX=%lf\n",curDegree180,locX,prevDisX);
                    stateMachine->sendTrigger(EVT_slalom_challenge); //130
                    prevDis=distance;
                    g_challenge_stepNo = 131;
                    prevDegree180 = curDegree180;
                }
            }
        }else if(g_challenge_stepNo == 131 && own_abs(prevDegree180 - curDegree180) > 32){
                    printf(",向きを元に戻す locY=%lf \n",locY);
                    stateMachine->sendTrigger(EVT_slalom_challenge); //131
                    prevDis=distance;   
                    g_challenge_stepNo = 132;

        }else if (g_challenge_stepNo == 132 && (distance - prevDis > 70 || curRgbSum < 110)) {
                printf(",黒ラインに向かう途中で、やや上向きに方向修正 distance=%lf prevDis=%lf curDegree180=%d\n",locY,prevDisY,curDegree180);
                stateMachine->sendTrigger(EVT_slalom_challenge);
                prevDis=distance;
                prevDegree180=curDegree180;
                g_challenge_stepNo = 133;

        }else if (g_challenge_stepNo == 133 && (curRgbSum < 120 || own_abs(prevDis-distance) > 200 )){
                printf(",黒ラインを２つ目まで前進し、２つ目に載ったら向きを調整するprevDis=%lf,distance=%lf,prevDegree180=%d,curDegree180=%d\n",prevDis,distance,prevDegree180,curDegree180);
                stateMachine->sendTrigger(EVT_slalom_challenge);
                g_challenge_stepNo = 140;
                //prevDegree180 = curDegree180;
        // 直進しスラロームを降りる 色判断追加する
     //   }else if(g_challenge_stepNo == 140 && curRgbSum <= 150){
     //           printf(",直進しスラロームを降りる 色判断追加する\n");
                prevDegree180 = curDegree180;
     //           g_challenge_stepNo = 141;
        }else if(g_challenge_stepNo ==140 && (check_sonar(85,254) || own_abs(curDegree180 - prevDegree180) > 30)){
            printf(",ソナーに引っかかった,g_challenge_stepNo=%d,ソナー=%d,角度=%d, curRgbSum=%d\n",g_challenge_stepNo,curSonarDis,own_abs(curDegree180 - prevDegree180),curRgbSum);
            g_challenge_stepNo = 141;
            prevDegree180 = curDegree180;
        // 直進しスラロームを降りる
//        }else if(g_challenge_stepNo == 141 && (own_abs(curDegree180 - prevDegree180) > 75){
//        }else if((g_challenge_stepNo == 141 && own_abs(curDegree180 - prevDegree180) > 56)||(g_challenge_stepNo == 140 && own_abs(curDegree180 - prevDegree180) > 65)){
        }else if(g_challenge_stepNo == 141 && own_abs(curDegree180 - prevDegree180) > 5){
            printf(",直進しスラロームを降りる,g_challenge_stepNo=%d,ソナー=%d,角度=%d, curRgbSum=%d\n",g_challenge_stepNo,curSonarDis,own_abs(curDegree180 - prevDegree180),curRgbSum);
            stateMachine->sendTrigger(EVT_slalom_challenge);
            armMotor->setPWM(30);
            g_challenge_stepNo = 150;
        }

        // スラローム降りてフラグOff
        if(g_angle > 6 && g_challenge_stepNo <= 150 && g_challenge_stepNo >= 130){
            printf("スラロームオフ\n");
            state = ST_block;
            slalom_flg = false;
            garage_flg = true;
            locX = 0; //初期化
            locY = 0; //初期化 
            distance = 0;//初期化
            prevAngL =0; //初期化 
            prevAngR =0; //初期化 
            azimuth = 0; //初期化 
            leftMotor->reset(); //初期化 
            rightMotor->reset(); //初期化 
            armMotor->setPWM(-100); //初期化 
            curAngle = 0;//初期化
            prevAngle = 0;//初期化
            root_no=0;
            g_challenge_stepNo = 150;
            stateMachine->sendTrigger(EVT_slalom_challenge);
        }
    }

    //ボーナスブロック＆ガレージ専用処理
    if (garage_flg && !slalom_flg && g_challenge_stepNo < 260 ){
        // if (g_challenge_stepNo >= 151 && g_challenge_stepNo <= 153){
        //     if (++traceCnt && traceCnt > 10) {
        //            printf(",garage_flg=%d,slalom_flg=%d,distance=%lf,g_angle=%d,curDegree360=%d, curSonarDis=%d, g_challenge_stepNo=%d,r=%d,g=%d,b=%d,locX=%lf,locY=%lf\n",garage_flg,slalom_flg,distance,g_angle,curDegree360, curSonarDis,g_challenge_stepNo,cur_rgb.r,cur_rgb.g,cur_rgb.b,locX,locY);
        //         traceCnt = 0;
        //     }
        // }

        //ガレージON後、距離が一部残るようであるため、距離がリセットされたことを確認
        if(g_challenge_stepNo == 150 && distance < 100 && distance > 1){
            g_challenge_stepNo = 151;
        }

        else if( g_challenge_stepNo == 151 && distance > 60){
            // ソナー稼働回転、方向を調整
            printf("ソナー稼働回転、方向を調整\n");
            stateMachine->sendTrigger(EVT_block_challenge); //151
            g_challenge_stepNo = 152;
        }else if(g_challenge_stepNo == 152 && check_sonar(80,255)){
            printf("ソナーガレージ外\n");
            prevDegree360 = curDegree360;
            stateMachine->sendTrigger(EVT_block_challenge); //152
            g_challenge_stepNo = 154;
        }else if (g_challenge_stepNo == 152 && check_sonar(1,80)){
            printf("ソナーガレージ内 curSonarDis=%d,curDegree360=%d\n",curSonarDis,curDegree360);
            prevSonarDis = curSonarDis;
            prevDegree360 = curDegree360;
            stateMachine->sendTrigger(EVT_block_challenge); //152
            g_challenge_stepNo = 153;
        }else if (g_challenge_stepNo == 153 && (curSonarDis - prevSonarDis > 10 || own_abs(curDegree360 - prevDegree360) > 18)){
            printf("ソナーガレージから外れた curSonarDis=%d,prevSonarDis=%d,curDegree360=%d\n",curSonarDis,prevSonarDis,curDegree360);
            g_challenge_stepNo = 154;
            prevDegree360 = curDegree360;
        }else if (g_challenge_stepNo == 153){
            prevSonarDis = curSonarDis;
        //一定角度回転
        }else if(g_challenge_stepNo == 154 && curDegree360 - prevDegree360 > 80){
            stateMachine->sendTrigger(EVT_block_challenge); //160
            g_challenge_stepNo = 170;

        }else if(g_challenge_stepNo == 161 && curDegree360 - prevDegree360 > 120 ){  
            printf("一定角度以上255を発見できなければ１２０度で進行\n");      
            //stateMachine->sendTrigger(EVT_block_challenge); //161
            g_challenge_stepNo = 162;

        //ソナーの値から進行方向確認
        }else if((g_challenge_stepNo == 161 && check_sonar(255,255)) ||  g_challenge_stepNo == 162){
            printf("進行方向発見,prevDegree360=%d,curDegree360=%d\n",prevDegree360,curDegree360);
            prevDegree360 = curDegree360;
            g_challenge_stepNo = 170;
        }

        //升目ライン方向へ進行
        else if(g_challenge_stepNo == 170 ){
            printf("升目ライン方向へ進行\n");
            prevDis = distance;
            // 升目ラインに接近
            stateMachine->sendTrigger(EVT_block_challenge); //170
            clock->sleep(2100);//スラローム後、大きくラインを左に外しても、手前の黒ラインに引っかからないためにスリープ
            g_challenge_stepNo = 180;
            stateMachine->sendTrigger(EVT_block_challenge); //180 減速して接近
        }

        //升目ラインの大外枠とクロス。黒、赤、黄色の３パターンのクロスがある
        else if(g_challenge_stepNo == 180){
        
            //黒を見つけたら、下向きのライントレース
            if(cur_rgb.g + cur_rgb.b <= 100 && cur_rgb.r <=50 && cur_rgb.g <=40 && cur_rgb.b <=60){
                printf("黒を見つけたら、下向きのライントレース\n");
                g_challenge_stepNo = 190;
                stateMachine->sendTrigger(EVT_block_challenge);//190
                prevDegree360 = curDegree360;
                prevDisY=locY;
                tempDis = locX;
                g_challenge_stepNo = 191;
            }
            //赤を見つけたら、赤からブロックへ  直進
            else if(cur_rgb.r - cur_rgb.b >=40 && cur_rgb.g <65 && cur_rgb.r - cur_rgb.g >30){
                printf("赤を見つけたら、赤からブロックへ  直進\n");
                g_challenge_stepNo = 200;
                stateMachine->sendTrigger(EVT_block_challenge); //200
                g_challenge_stepNo = 201;
                prevDegree360 = curDegree360;
                prevDisY=locY;
                tempDis = locX;
                printf("赤色超えました\n");
            }
            
            //黄色を見つけたら、右に直進のライントレース
            else if(cur_rgb.r + cur_rgb.g - cur_rgb.b >= 160 &&  cur_rgb.r - cur_rgb.g <= 30){
                printf("黄色を見つけたら、右に直進のライントレース\n");
                g_challenge_stepNo = 210;
                stateMachine->sendTrigger(EVT_block_challenge); //210
                g_challenge_stepNo = 211;

                clock->sleep(800);
                stateMachine->sendTrigger(EVT_block_challenge); //211
                g_challenge_stepNo = 212;
                
                printf("黄色超えました1,distance=lf\n",distance);
                prevDis = distance;
                prevDisY=locY;
                tempDis = locX;
            }
         //黒ライン進入の続き
        }else if(g_challenge_stepNo == 191 && own_abs(curDegree360 - prevDegree360) > 40){
                g_challenge_stepNo = 192;
                root_no = 1;

        //回転中、黒ではなく黄色に接触した場合、回転角度は固定        
        //}else if(g_challenge_stepNo == 192 && cur_rgb.r + cur_rgb.g - cur_rgb.b >= 160 &&  cur_rgb.r - cur_rgb.g <= 30 ){
        //        g_challenge_stepNo = 194;

        //回転中、黒を見つけた場合
        }else if(g_challenge_stepNo == 192 && (curRgbSum <= 150 || own_abs(curDegree360 - prevDegree360) >= 90)){
                g_challenge_stepNo = 193;
        
        }else if(g_challenge_stepNo == 193 && own_abs(curDegree360 - prevDegree360) >= 125){
                //ここにストレートをいれる
                printf("黄色のほうへ,角度=%d,%d　計算=%d",curDegree360,prevDegree360,own_abs(curDegree360 - prevDegree360));
                stateMachine->sendTrigger(EVT_block_challenge);
                prevDis=distance;
                prevDegree360 = curDegree360;
                g_challenge_stepNo = 195;

        }else if(g_challenge_stepNo == 195 && own_abs(distance - prevDis) >= 90){
                printf("ここからライントレース\n");
                stateMachine->sendTrigger(EVT_line_on_p_cntl);
                g_challenge_stepNo = 220;
                //prevDegree360 = curDegree360;

        //}else if(g_challenge_stepNo >= 191 && g_challenge_stepNo <= 193 ){
                //printf("r=%d,g=%d,b=%d,curRgbSum=%d\n",cur_rgb.r,cur_rgb.g,cur_rgb.b,curRgbSum);

        //赤ライン進入の続き
        }else if(g_challenge_stepNo == 201 && curDegree360 - prevDegree360 >= 75){
                printf("赤色超えました2\n");
                stateMachine->sendTrigger(EVT_block_challenge); //201
                g_challenge_stepNo = 250;
                root_no = 2;
        //黄色ライン進入の続き
        }else if(g_challenge_stepNo==212 && cur_rgb.r + cur_rgb.g - cur_rgb.b <= 130){
                g_challenge_stepNo = 213;
                printf("黄色超えました2 r=%d,g=%d,b=%d\n",cur_rgb.r , cur_rgb.g , cur_rgb.b);
                root_no = 3;
        }else if((g_challenge_stepNo==213 && cur_rgb.r + cur_rgb.g - cur_rgb.b >= 160)|| ((g_challenge_stepNo==212 || g_challenge_stepNo==213) && distance-prevDis > 130)){
                printf("黄色超えました3 r=%d,g=%d,b=%d\n",cur_rgb.r , cur_rgb.g , cur_rgb.b);
                g_challenge_stepNo = 213;
                stateMachine->sendTrigger(EVT_block_challenge); //213
                g_challenge_stepNo = 214;

        }else if(g_challenge_stepNo==214 && cur_rgb.r + cur_rgb.g - cur_rgb.b <= 130){
                g_challenge_stepNo = 250;

        //黒ラインからの黄色を見つけたらブロック方向へターン
//        }else if(g_challenge_stepNo == 220 && cur_rgb.r + cur_rgb.g - cur_rgb.b >= 160 &&  cur_rgb.r - cur_rgb.g <= 30){
        }else if(((g_challenge_stepNo == 220 || g_challenge_stepNo == 221 || g_challenge_stepNo == 195 ) && cur_rgb.r + cur_rgb.g - cur_rgb.b >= 160 ) || (g_challenge_stepNo == 194 && own_abs(curDegree360 - prevDegree360) >= 120)){
            
            printf("ここのprevDegree360=%d,azi=%d,sa=%d,locX=%lf,distance-prev=%lf\n",prevDegree360,curDegree360,prevDegree360-curDegree360,locX,distance-prevDis);
            //prevDegree180は黒ライン侵入時、回転後のもの
            if(distance-prevDis >= 230){
               accumuDegree = prevDegree360 - curDegree360 + 5; 
            }else if(distance-prevDis>=150){
               accumuDegree = prevDegree360 - curDegree360 + 3; 
            }else if(distance-prevDis<80){
                accumuDegree = prevDegree360 - curDegree360 - 20; 
            }else if(distance-prevDis<107){
                accumuDegree = prevDegree360 - curDegree360 - 6; 
            }else {
                accumuDegree = prevDegree360 - curDegree360;
            }
            prevDegree360 = curDegree360;
            g_challenge_stepNo = 230;
            stateMachine->sendTrigger(EVT_block_area_in); //230
            g_challenge_stepNo = 231;

        }else if(g_challenge_stepNo==231 && prevDegree360 - curDegree360 + accumuDegree > 111 ){
            prevDis = distance;
            stateMachine->sendTrigger(EVT_block_challenge); //231
            g_challenge_stepNo = 232;
            printf("ここまで黄色エリア１ locX=%lf,locY=%lf,prevDegree360=%d,azi=%d,sa=%d,gosa=%d\n",locX,locY,prevDegree360,curDegree360,prevDegree360 -curDegree360,accumuDegree);
        
        
        }else if(g_challenge_stepNo == 232 && distance - prevDis > 200 && cur_rgb.r + cur_rgb.g - cur_rgb.b <= 130 ){
            printf("ここまで黄色エリア２ distance=%lf prevDis=%lf locX=%lf,locY=%lf,\n",distance,prevDis,locX,locY);
            stateMachine->sendTrigger(EVT_line_on_pid_cntl); //232
            g_challenge_stepNo = 250;

        //ここから、現在調整中
        //黒を外してしまった判断
        // }else if ((g_challenge_stepNo == 220 || g_challenge_stepNo == 195) && cur_rgb.r + cur_rgb.g + cur_rgb.b > 300 ){
        //     //黒のラインなのに、一定時間白を経過した場合
        //     g_challenge_stepNo == 221;

        // //いろいろ外して、黒に到達してしまった場合
        // }else if((g_challenge_stepNo == 221 || (g_challenge_stepNo == 250 && root_no == 2)) && cur_rgb.r + cur_rgb.g - cur_rgb.b <= 130 && locX < 2690){
        //     //printf("いろいろ外して黒へ,garage_flg=%d,slalom_flg=%d,distance=%lf,g_angle=%d,curDegree360=%d, curSonarDis=%d, g_challenge_stepNo=%d,r=%d,g=%d,b=%d,locX=%lf,locY=%lf\n",garage_flg,slalom_flg,distance,g_angle,curDegree360, curSonarDis,g_challenge_stepNo,cur_rgb.r,cur_rgb.g,cur_rgb.b,locX,locY);
        //     accumuDegree = 90;
        //     if(g_challenge_stepNo == 221 ){
        //         accumuDegree = 70;
        //     }else if(g_challenge_stepNo == 250){
        //         accumuDegree = 70;
        //     }
        //     prevDegree360 = curDegree360;
        //     g_challenge_stepNo = 233;
        //     stateMachine->sendTrigger(EVT_block_area_in); //233
        //     g_challenge_stepNo = 234;

        // }else if(g_challenge_stepNo == 234 && own_abs(prevDegree360 - curDegree360)  > accumuDegree ){
        //     printf("黒から回転\n");
        //     stateMachine->sendTrigger(EVT_line_on_pid_cntl); //234
        //     g_challenge_stepNo = 250;        

        // //ここまで、現在調整中

        //赤を見つけたら黒を見つけるまで直進、その後ライントレース
        }else if(g_challenge_stepNo == 220 && cur_rgb.r - cur_rgb.b >= 40 && cur_rgb.g < 60 && cur_rgb.r - cur_rgb.g > 30){
            g_challenge_stepNo = 240;
            //直前までライントレース
            stateMachine->sendTrigger(EVT_block_area_in); //240
            g_challenge_stepNo = 241;
        
        //赤を離脱するために以下の分岐にあるカラーを順番にたどる
        }else if( cur_rgb.r - cur_rgb.b < 20 && g_challenge_stepNo == 241){
            g_challenge_stepNo = 242;
        }else if( cur_rgb.r - cur_rgb.b >=40 && g_challenge_stepNo == 242){
            g_challenge_stepNo = 243;
        
        //赤を通過時、大きくラインを外れたら、カーブして戻る
        }else if(g_challenge_stepNo == 243 && cur_rgb.r + cur_rgb.g + cur_rgb.b > 300){
            g_challenge_stepNo = 244;
            stateMachine->sendTrigger(EVT_block_challenge); //244
            g_challenge_stepNo = 245;
            clock->sleep(10);
            stateMachine->sendTrigger(EVT_line_on_pid_cntl); //245
            g_challenge_stepNo = 220;

        //ラインを外れていなければ、黒のライントレースへ
        }else if(g_challenge_stepNo == 243 && cur_rgb.r + cur_rgb.g + cur_rgb.b <= 100){
            g_challenge_stepNo = 246;
            stateMachine->sendTrigger(EVT_line_on_pid_cntl); //246 
            g_challenge_stepNo = 220;
        
        }else if(g_challenge_stepNo==250){    
            //printf(",garage_flg=%d,slalom_flg=%d,distance=%lf,g_angle=%d,curDegree360=%d, curSonarDis=%d, g_challenge_stepNo=%d,r=%d,g=%d,b=%d,locX=%lf,locY=%lf\n",garage_flg,slalom_flg,distance,g_angle,curDegree360, curSonarDis,g_challenge_stepNo,cur_rgb.r,cur_rgb.g,cur_rgb.b,locX,locY);
            
            //ブロックに直進、ブロックの黄色を見つけたら
            if(cur_rgb.r + cur_rgb.g - cur_rgb.b >= 150){
                    printf("ブロックGETしてほしい tempDis=%lf,locY=%lf,prevDisY=%lf,locX=%lf,prevDisX=%lf\n",tempDis,locY,prevDisY,locX,prevDisX);                 
                    g_challenge_stepNo = 260;
                    prevDegree360=curDegree360;
            }
        }
    
    }


    //ボーナスブロック後半
    else if (garage_flg && g_challenge_stepNo >= 260 && !slalom_flg){

        // if (g_challenge_stepNo >= 260 && g_challenge_stepNo <= 270){
        //     if (++traceCnt && traceCnt > 10) {
        //            printf(",garage_flg=%d,slalom_flg=%d,distance=%lf,g_angle=%d,curDegree360=%d, curSonarDis=%d, g_challenge_stepNo=%d,r=%d,g=%d,b=%d,locX=%lf,locY=%lf\n",garage_flg,slalom_flg,distance,g_angle,curDegree360, curSonarDis,g_challenge_stepNo,cur_rgb.r,cur_rgb.g,cur_rgb.b,locX,locY);
        //         traceCnt = 0;
        //     }
        // }

        //ガレージに戻るべく、ターン        
        if(g_challenge_stepNo == 260){
            //走行体回頭
            stateMachine->sendTrigger(EVT_block_area_in); //260
            accumuDegree =  prevDegree360 - curDegree360;
            prevDegree360 = curDegree360;
            g_challenge_stepNo = 261;

            //赤〇ポイントからのブロック到達の場合
            if(root_no==2){ 
                //回転角度に210度をセット
                turnDegree = 230;
            //黒ライン、黄色直進からのブロック到達の場合
            }else{
                //回転角度に80度をセット
                turnDegree = 135;
            }
        }else if(g_challenge_stepNo == 261 && accumuDegree > turnDegree){
            stateMachine->sendTrigger(EVT_block_challenge); //261
            g_challenge_stepNo = 262;

        }else if(g_challenge_stepNo == 261){ //角度が条件に該当しない場合、差分を累積していく。
            accumuDegree += getTurnDgree(prevDegree360,curDegree360);
            prevDegree360 = curDegree360;
        //前方に何もない状況になったら進行
        }else if(g_challenge_stepNo == 262){
            if(check_sonar(255,255) || curDegree360 <= 140){
//            if(check_sonar(0,150)){
                printf("調整中。前方に何もない状況になったら進行 curSonarDis=%d,curDegree360=%d\n",curSonarDis,curDegree360);
                g_challenge_stepNo = 263;
                accumuDegree = 0;
                prevDegree360 = curDegree360;
            }
        }else if(g_challenge_stepNo == 263){
            accumuDegree += getTurnDgree(prevDegree360,curDegree360);
            prevDegree360 = curDegree360;

            if(accumuDegree >= 16){ //255を発見してから何度曲がるかを調整
//                if(accumuDegree >= 55){
                printf("角度調整後に前進する curSonarDis=%d,curDegree360=%d\n",curSonarDis,curDegree360);
                stateMachine->sendTrigger(EVT_block_challenge); //263
                g_challenge_stepNo = 270;
                prevDis = distance;
                prevDisX=locX;
            }
        }
        //進行方向が正しいか、一定距離走行、上に進んでいたら下に回転、走行で、下の方向に進むまで繰り返す
        // else if(g_challenge_stepNo == 264 && distance - prevDis > 30){
        //         printf("３０進んで、どちらに進んでいるか確認locX = %lf, prevDisX = %lf\n", locX,prevDisX);
        //         if( locX - prevDisX < 0){
        //             g_challenge_stepNo = 270;
        //         }else {
        //             stateMachine->sendTrigger(EVT_block_challenge);//264
        //             prevDegree180 = curDegree180;
        //             g_challenge_stepNo = 265;
        //         }
        // } //30度回転したら、まっすぐ進む
        // else if(g_challenge_stepNo == 265 && own_abs( prevDegree180 - curDegree180) > 30){
        //     stateMachine->sendTrigger(EVT_block_challenge); //265
        //     prevDis = distance;
        //     prevDisX=locX;
        //     g_challenge_stepNo = 264;
        // }

        else if(g_challenge_stepNo == 270 && distance - prevDis > 380){
            //黒線ブロックを超えるまで、一定距離を走行
            g_challenge_stepNo = 271;
        }

        else if(g_challenge_stepNo == 271 && check_sonar(255,255)){
            g_challenge_stepNo = 280;
        }
        else if(g_challenge_stepNo == 271 && check_sonar(1,254)){
                //視界が開けてなければ逆回転
                stateMachine->sendTrigger(EVT_block_challenge);//271
                prevDegree180=curDegree180;
                printf("曲がる前は角度：=%d,curDegree360=%d,curSonarDis=%d\n",prevDegree180,curDegree360,curSonarDis);
                g_challenge_stepNo = 272;
        }
        else if(g_challenge_stepNo == 272 && (check_sonar(255,255)|| own_abs(prevDegree180 - curDegree180) > 30 )){
                printf("曲がった後は角度：=%d,curDegree360=%d,curSonarDis=%d\n",curDegree180,curDegree360,curSonarDis);
                stateMachine->sendTrigger(EVT_block_challenge);//272
                g_challenge_stepNo = 280;                                
        }
        //緑を見つけたら、速度固定
        else if(g_challenge_stepNo == 280 && cur_rgb.r <= 13 && cur_rgb.b <= 50 && cur_rgb.g > 60){ 
            stateMachine->sendTrigger(EVT_block_challenge); //280
            g_challenge_stepNo = 281;
 
        //一度白を通過
        }else if(g_challenge_stepNo == 281 && curRgbSum > 300){
            g_challenge_stepNo = 282;

        //緑をみつけたらカーブ開始
        }else if(g_challenge_stepNo == 282 && cur_rgb.r <= 13 && cur_rgb.b <= 50 && cur_rgb.g > 60 ){
            stateMachine->sendTrigger(EVT_block_challenge); //282
            g_challenge_stepNo = 283;
        
        //一度白を通過
        }else if(g_challenge_stepNo == 283 && curRgbSum > 300){
            g_challenge_stepNo = 284;

        }else if(g_challenge_stepNo == 284 && cur_rgb.r + cur_rgb.g <= 150){  //青または黒をみつけたら、完全にターン)
            g_challenge_stepNo = 284;
            stateMachine->sendTrigger(EVT_block_challenge); //284
            g_challenge_stepNo = 285;
            clock->sleep(500);
        // ガレージの奥の距離を捉えたら直進
        }else if(g_challenge_stepNo == 285 && check_sonar(35,250)){
            prevDegree360 = curDegree360;
            accumuDegree = 0;
            g_challenge_stepNo = 286;
        }else if(g_challenge_stepNo == 286){
            
            accumuDegree += getTurnDgree(prevDegree360,curDegree360);
            prevDegree360 = curDegree360;

            if(accumuDegree > 23){
                stateMachine->sendTrigger(EVT_block_challenge); //286
                g_challenge_stepNo = 290;
            }

        }else if(g_challenge_stepNo == 290 && check_sonar(0,15)){
                stateMachine->sendTrigger(EVT_block_challenge); //290
                garage_flg = false;
        }
    }//ガレージ終了


    /*
    int32_t iDeltaD  = (int32_t)(1000.0 * deltaDist );
    int32_t iDeltaDL = (int32_t)(1000.0 * deltaDistL);
    int32_t iDeltaDR = (int32_t)(1000.0 * deltaDistR);
    if (iDeltaDL != 0 && iDeltaDR != 0) {
        _debug(syslog(LOG_NOTICE, "%08u, %06d, %06d, %06d, %06d", clock->now(), getDistance(), iDeltaD, iDeltaDL, iDeltaDR));
    }
    */

    integD  += deltaDist;  // temp
    integDL += deltaDistL; // temp
    integDR += deltaDistR; // temp
    // display trace message in every PERIOD_TRACE_MSG ms
    if (++traceCnt * PERIOD_OBS_TSK >= PERIOD_TRACE_MSG) {
        traceCnt = 0;
        /*
        // temp from here
        int32_t iD  = (int32_t)integD;
        int32_t iDL = (int32_t)integDL;
        int32_t iDR = (int32_t)integDR;
        _debug(syslog(LOG_NOTICE, "%08u, %06d, %06d, %06d, %06d", clock->now(), getDistance(), iD, iDL, iDR));
        integD = integDL = integDR = 0.0;
        // temp to here
        */
        /*
        _debug(syslog(LOG_NOTICE, "%08u, Observer::operate(): distance = %d, azimuth = %d, x = %d, y = %d", clock->now(), getDistance(), getAzimuth(), getLocX(), getLocY()));
        _debug(syslog(LOG_NOTICE, "%08u, Observer::operate(): hsv = (%03u, %03u, %03u)", clock->now(), g_hsv.h, g_hsv.s, g_hsv.v));
        _debug(syslog(LOG_NOTICE, "%08u, Observer::operate(): rgb = (%03u, %03u, %03u)", clock->now(), g_rgb.r, g_rgb.g, g_rgb.b));
        _debug(syslog(LOG_NOTICE, "%08u, Observer::operate(): angle = %d, anglerVelocity = %d", clock->now(), g_angle, g_anglerVelocity));
        */
        //_debug(syslog(LOG_NOTICE, "%08u, Observer::operate(): sensor = %d, target = %d, distance = %d", clock->now(), g_grayScale, GS_TARGET, getDistance()));
    }
}

void Observer::deactivate() {
    // deregister cyclic handler from EV3RT
    stp_cyc(CYC_OBS_TSK);
    //clock->sleep() seems to be still taking milisec parm
    clock->sleep(PERIOD_OBS_TSK/2/1000); // wait a while
    _debug(syslog(LOG_NOTICE, "%08u, Observer handler unset", clock->now()));
}

bool Observer::check_touch(void) {
    if (touchSensor->isPressed()) {
        return true;
    } else {
        return false;
    }
}

bool Observer::check_sonar(void) {
    int32_t distance = sonarSensor->getDistance();
    if ((distance <= SONAR_ALERT_DISTANCE) && (distance >= 0)) {
        return true; // obstacle detected - alert
    } else {
        return false; // no problem
    }
}

bool Observer::check_sonar(int16_t sonar_alert_dist_from, int16_t sonar_alert_dist_to) {
    int32_t distance = sonarSensor->getDistance();
    //printf(",distance2=%d, sonar_alert_dist_from=%d, sonar_alert_dist_to=%d\n",distance, sonar_alert_dist_from, sonar_alert_dist_to );
    if (distance >= sonar_alert_dist_from && distance <= sonar_alert_dist_to) {
        return true; // obstacle detected - alert
    } else {
        return false; // no problem
    }
}

bool Observer::check_backButton(void) {
    if (ev3_button_is_pressed(BACK_BUTTON)) {
        return true;
    } else {
        return false;
    }
}

bool Observer::check_lost(void) {
    if (g_grayScale > GS_LOST) {
        return true;
    } else {
        return false;
    }
    /*
    int8_t otRes_r, otRes_g, otRes_b;
    otRes_r = ot_r->test(cur_rgb.r);
    otRes_g = ot_g->test(cur_rgb.g);
    otRes_b = ot_b->test(cur_rgb.b);
    if ((otRes_r == POS_OUTLIER && otRes_g == POS_OUTLIER) ||
        (otRes_g == POS_OUTLIER && otRes_b == POS_OUTLIER) ||
        (otRes_b == POS_OUTLIER && otRes_r == POS_OUTLIER)) {
        return true;
    } else {
        return false;
    }
    */
}

// bool Observer::check_tilt(void) {
//     int16_t anglerVelocity = gyroSensor->getAnglerVelocity();
//     if (anglerVelocity < ANG_V_TILT && anglerVelocity > (-1) * ANG_V_TILT) {
//         return false;
//     } else {
//         _debug(syslog(LOG_NOTICE, "%08u, Observer::operate(): TILT anglerVelocity = %d", clock->now(), anglerVelocity));
//         return true;
//     }
// }

void Observer::freeze() {
    frozen = true;
}

void Observer::unfreeze() {
    frozen = false;
}

//角度累積分を計算
int16_t Observer::getTurnDgree(int16_t prev_x,int16_t x){

    int16_t hoge = prev_x-x;
    if(hoge < 0){
        hoge = hoge * -1;
    }
    if(hoge > 180){
        hoge = 360-hoge;
    }

    return hoge;    
}

Observer::~Observer() {
    _debug(syslog(LOG_NOTICE, "%08u, Observer destructor", clock->now()));
}