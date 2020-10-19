//
//  BlindRunner.cpp
//  aflac2020
//
//  Copyright © 2020 Ahiruchan Koubou. All rights reserved.
//

#include "app.h"
#include "BlindRunner.hpp"
#include "Observer.hpp"
#include "StateMachine.hpp"
#include <string.h>
#include <stdlib.h>

BlindRunner::BlindRunner(Motor* lm, Motor* rm, Motor* tm) : LineTracer(lm, rm, tm) {
#if defined(MAKE_SIM)
    _debug(syslog(LOG_NOTICE, "%08lu, BlindRunner attempts to read profile in simulator environment", clock->now()));
	readPropFile("BlindRunner_prop.txt");
#else
	readPropFile("/ev3rt/res/BlindRunner_prop.txt");
#endif
	leftMotor  = lm;
	rightMotor = rm;
	tailMotor  = tm;
	s_trace_counter = 0;
    _debug(syslog(LOG_NOTICE, "%08lu, BlindRunner constructor", clock->now()));
}

void BlindRunner::haveControl() {
    activeNavigator = this;
    // private変数の初期化
	courseMapSize = sizeof(courseMap) / sizeof(*courseMap);
	currentSection = 0;
	speedChgCnt = 0;
	forward = speed;
	turn = 0;
	d_offset = d_cv01_line_lost = d_cv01_line_found = __INT32_MAX__;
	stopping = false;
    // ログ出力
    syslog(LOG_NOTICE, "%08lu, BlindRunner has control", clock->now());
	syslog(LOG_NOTICE, "%08lu, section %s entered", clock->now(), courseMap[currentSection].id);
	ev3_led_set_color(LED_GREEN);
}

void BlindRunner::operate() {
	int32_t d = observer->getDistance();
	if (currentSection < courseMapSize - 1 && d >= (courseMap[currentSection].sectionEnd - (d_offset==__INT32_MAX__?0:d_offset))) {
		currentSection++;
		syslog(LOG_NOTICE, "%08lu, section %s entered at %d", clock->now(), courseMap[currentSection].id, d);
	} else if (currentSection == courseMapSize  - 1 && d >= (courseMap[currentSection].sectionEnd - d_offset)) {
		if (!stopping) {
			stopping = true;
			_debug(syslog(LOG_NOTICE, "%08lu, BlindRunner course map exhausted", clock->now()));
			observer->notifyOfDistance(0); // give control back to LineTracer
			g_challenge_stepNo = 900; //sano_t
		}
	}

    if (d_cv01_line_lost == __INT32_MAX__ && g_grayScale > GS_LOST && strcmp(sBcv01, courseMap[currentSection].id) == 0) {
		d_cv01_line_lost = d;
	} else if (d_cv01_line_lost != __INT32_MAX__ && d_cv01_line_found == __INT32_MAX__ && g_grayScale <= GS_LOST && strcmp(sBcv01, courseMap[currentSection].id) == 0) {
		d_cv01_line_found = d;
	}
	if (d_offset == __INT32_MAX__ && d_cv01_line_lost != __INT32_MAX__ && d_cv01_line_found != __INT32_MAX__) {
		int32_t d_actual_midpoint = (d_cv01_line_lost + d_cv01_line_found) / 2;
		d_offset = d_cv01_midpoint - d_actual_midpoint;
		syslog(LOG_NOTICE, "%08lu, cv01 midpoint plan %d vs actual %d: distance to be offset by %d", clock->now(), d_cv01_midpoint, d_actual_midpoint, d_offset);
	}

	if (courseMap[currentSection].id[0] == 'L') {
		if (++speedChgCnt * PERIOD_NAV_TSK >= PERIOD_SPEED_CHG) {
			speedChgCnt = 0;
			int s = LineTracer::getSpeed();
			if (s < SPEED_NORM) {
				LineTracer::setSpeed(++s);
			} else if (s > SPEED_NORM) {
				LineTracer::setSpeed(--s);
			}
		}
		LineTracer::operate();
	} else {
		if (courseMap[currentSection].id[0] == 'B') {
			forward = SPEED_BLIND;
		} else if (courseMap[currentSection].id[0] == 'R') {
			LineTracer::setSpeed(SPEED_SLOW);
			forward = SPEED_SLOW;
			if (g_grayScale <= GS_LOST) { // line found
				LineTracer::setSpeed(SPEED_RECOVER);
				currentSection++;  // switch to LineTracer entry
				syslog(LOG_NOTICE, "%08lu, section %s entered at %d", clock->now(), courseMap[currentSection].id, d);
			}
		} else {
			_debug(syslog(LOG_NOTICE, "%08lu, illegal course map id", clock->now()));
			stateMachine->sendTrigger(EVT_cmdStop);				
		}
		turn = _EDGE * forward * courseMap[currentSection].curvature / 2;
		/* 左右モータでロボットのステアリング操作を行う */
    	pwm_L = forward - turn;
    	pwm_R = forward + turn;

    	leftMotor->setPWM(pwm_L);
    	rightMotor->setPWM(pwm_R);
	}
}

int BlindRunner::readLine( FILE* file, char* dst, size_t len ){
    int c = 0;
    unsigned int i = 0;

    while ((c = fgetc(file)) != EOF) {
      if (c < 0) {
        return c;        // error handling
      }
      if ( i + 1 == len ) {  // reached end of buffer
        dst[i] = '\0';
        break;
      }
      if (c == '\n') {   // reached end of line
        dst[i] = '\0';
        break;
      }
      dst[i] = (char)c;
      i++;
    }
    return i;
}

void BlindRunner::readPropFile( const char* filename ){
	FILE* prop_file = NULL;
	prop_file = fopen( filename, "r" );
	if( prop_file==NULL){
		_debug(syslog(LOG_NOTICE, "%08lu, BlindRunner readProfile() file not found", clock->now()));
		return;
	}

	memset( &props, 0, sizeof(struct property) * NUM_PROPS );

	int retval = 0;
	int i = 0;
	char buf[256];
	memset( &buf, 0, sizeof(buf) );

	while( ( retval = readLine(prop_file, buf, 256)) > 0 ){
		char* comptr = strstr( buf, "," );
		if( i < NUM_PROPS && NULL != comptr ){
			*comptr = '\0';
			strcpy( props[i].name, buf );
			props[i].value = atoi(comptr+1);
			i++;
		}else
			break;
	}
	fclose( prop_file );
}

int BlindRunner::getProp( const char* propname ){
	if( props == NULL ){
		return 0;
	}

	for ( int i = 0; i < NUM_PROPS; i++ ){
		if( strcmp( propname, props[i].name ) == 0 ){
			return props[i].value;
		}
	}
	return 0;
}

BlindRunner::~BlindRunner() {
    _debug(syslog(LOG_NOTICE, "%08lu, BlindRunner destructor", clock->now()));
}