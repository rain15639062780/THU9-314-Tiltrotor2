/****************************************************************************
 *
 *   Copyright (c) 2015 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file tiltrotor.cpp
 *
 * @author Roman Bapst 		<bapstroman@gmail.com>
 * @author Andreas Antener 	<andreas@uaventure.com>
 *
*/

#include "tiltrotor.h"
#include "vtol_att_control_main.h"
/***********************************/

#include <px4_config.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>

#include <systemlib/err.h>

#include <uORB/uORB.h>
#include <uORB/topics/debug_key_value.h>

/************************************/

#define ARSP_YAW_CTRL_DISABLE 7.0f	// airspeed at which we stop controlling yaw during a front transition
/*************************************/
//rain 2018-4-9 10:56:30
//用于限制油门的最大输出（1+THROTTLE_TRANSITION_MAX）
#define THROTTLE_TRANSITION_MAX 0.25f	// maximum added thrust above last value in transition

#define VT_TILT_TRANS_L  0.3f     //用于辅助trans_back设计控制律 FW > TRANS > VT_TILT_TRANS_L > MC
#define VT_THRUST_HOVER  0.6f     //mc下悬停的油门值,用于trans_back阶段和thrust比较

/*************************************/
Tiltrotor::Tiltrotor(VtolAttitudeControl *attc) :
	VtolType(attc),
	_rear_motors(ENABLED),
	_tilt_control(0.0f),
	_min_front_trans_dur(0.5f),
	/**********************************/
	//rain 2018-4-9 18:31:27
	//对应.h文件赋初值
	//_thrust_transition_start(0.0f),
	_failsafe_flag(false),
	_failsafe_trans_dur(0.0f),
	_tilt_failsafe(0.0f),
	pub_dbg(nullptr),
	pub_dbg_fs(nullptr),
	pub_dbg_roll_weight(nullptr),
	pub_dbg_pitch_weight(nullptr),
	pub_dbg_yaw_weight(nullptr),
	pub_dbg_roll(nullptr)
	//pub_dbg_array(nullptr)
	/***********************************/
{
	_vtol_schedule.flight_mode = MC_MODE;
	_vtol_schedule.transition_start = 0;

	_mc_roll_weight = 1.0f;
	_mc_pitch_weight = 1.0f;
	_mc_yaw_weight = 1.0f;

	_flag_was_in_trans_mode = false;

	_params_handles_tiltrotor.front_trans_dur = param_find("VT_F_TRANS_DUR");
	_params_handles_tiltrotor.back_trans_dur = param_find("VT_B_TRANS_DUR");
	_params_handles_tiltrotor.tilt_mc = param_find("VT_TILT_MC");
	_params_handles_tiltrotor.tilt_transition = param_find("VT_TILT_TRANS");
	_params_handles_tiltrotor.tilt_fw = param_find("VT_TILT_FW");
	_params_handles_tiltrotor.airspeed_trans = param_find("VT_ARSP_TRANS");
	_params_handles_tiltrotor.airspeed_blend_start = param_find("VT_ARSP_BLEND");
	_params_handles_tiltrotor.elevons_mc_lock = param_find("VT_ELEV_MC_LOCK");
	_params_handles_tiltrotor.front_trans_dur_p2 = param_find("VT_TRANS_P2_DUR");
	_params_handles_tiltrotor.fw_motors_off = param_find("VT_FW_MOT_OFFID");
	_params_handles_tiltrotor.airspeed_mode = param_find("FW_ARSP_MODE");
	_params_handles_tiltrotor.diff_thrust = param_find("VT_FW_DIFTHR_EN");
	_params_handles_tiltrotor.diff_thrust_scale = param_find("VT_FW_DIFTHR_SC");


}

Tiltrotor::~Tiltrotor()
{

}

void
Tiltrotor::parameters_update()
{
	float v;
	int l;

	/* motors that must be turned off when in fixed wing mode */
	param_get(_params_handles_tiltrotor.fw_motors_off, &l);
	_params_tiltrotor.fw_motors_off = get_motor_off_channels(l);


	/* vtol duration of a front transition */
	param_get(_params_handles_tiltrotor.front_trans_dur, &v);
	_params_tiltrotor.front_trans_dur = math::constrain(v, 1.0f, 5.0f);

	/* vtol duration of a back transition */
	param_get(_params_handles_tiltrotor.back_trans_dur, &v);
	_params_tiltrotor.back_trans_dur = math::constrain(v, 0.0f, 5.0f);

	/* vtol tilt mechanism position in mc mode */
	param_get(_params_handles_tiltrotor.tilt_mc, &v);
	_params_tiltrotor.tilt_mc = v;

	/* vtol tilt mechanism position in transition mode */
	param_get(_params_handles_tiltrotor.tilt_transition, &v);
	_params_tiltrotor.tilt_transition = v;

	/* vtol tilt mechanism position in fw mode */
	param_get(_params_handles_tiltrotor.tilt_fw, &v);
	_params_tiltrotor.tilt_fw = v;

	/* vtol airspeed at which it is ok to switch to fw mode */
	param_get(_params_handles_tiltrotor.airspeed_trans, &v);
	_params_tiltrotor.airspeed_trans = v;

	/* vtol airspeed at which we start blending mc/fw controls */
	param_get(_params_handles_tiltrotor.airspeed_blend_start, &v);
	_params_tiltrotor.airspeed_blend_start = v;

	/* vtol lock elevons in multicopter */
	param_get(_params_handles_tiltrotor.elevons_mc_lock, &l);
	_params_tiltrotor.elevons_mc_lock = l;

	/* vtol front transition phase 2 duration */
	param_get(_params_handles_tiltrotor.front_trans_dur_p2, &v);
	_params_tiltrotor.front_trans_dur_p2 = v;

	/* avoid parameters which will lead to zero division in the transition code */
	_params_tiltrotor.front_trans_dur = math::max(_params_tiltrotor.front_trans_dur, _min_front_trans_dur);

	if (_params_tiltrotor.airspeed_trans < _params_tiltrotor.airspeed_blend_start + 1.0f) {
		_params_tiltrotor.airspeed_trans = _params_tiltrotor.airspeed_blend_start + 1.0f;
	}

	/* airspeed mode */
	param_get(_params_handles_tiltrotor.airspeed_mode, &l);
	_params_tiltrotor.airspeed_mode = math::constrain(l, 0, 2);

	param_get(_params_handles_tiltrotor.diff_thrust, &_params_tiltrotor.diff_thrust);

	param_get(_params_handles_tiltrotor.diff_thrust_scale, &v);
	_params_tiltrotor.diff_thrust_scale = math::constrain(v, -1.0f, 1.0f);
}

int Tiltrotor::get_motor_off_channels(int channels)
{
	int channel_bitmap = 0;

	int channel;

	for (int i = 0; i < _params->vtol_motor_count; ++i) {
		channel = channels % 10;

		if (channel == 0) {
			break;
		}

		channel_bitmap |= 1 << (channel - 1);
		channels = channels / 10;
	}

	return channel_bitmap;
}

void Tiltrotor::update_vtol_state()
{

	/* simple logic using a two way switch to perform transitions.
	 * after flipping the switch the vehicle will start tilting rotors, picking up
	 * forward speed. After the vehicle has picked up enough speed the rotors are tilted
	 * forward completely. For the backtransition the motors simply rotate back.
	*/

	if (!_attc->is_fixed_wing_requested()) {

		// plane is in multicopter mode
		switch (_vtol_schedule.flight_mode) {
		case MC_MODE:
			break;

		case FW_MODE:
			_vtol_schedule.flight_mode 	= TRANSITION_BACK;
			_vtol_schedule.transition_start = hrt_absolute_time();
			//rain 2018-4-9 22:06:15
			//转换阶段如果需要在增温模式下完成，在此处加入模式切换控制程序
			break;

		case TRANSITION_FRONT_P1:
			// failsafe into multicopter mode
			//_vtol_schedule.flight_mode = MC_MODE;
		/**********************************************************/
			//rain 2018-4-9 09:19:18
			//进入FAILSAFE_MODE
			_vtol_schedule.flight_mode 	= FAILSAFE_MODE;
			_vtol_schedule.transition_start = hrt_absolute_time();
		_flag_was_in_trans_mode = false;
		/*************************************************************/
			break;

		case TRANSITION_FRONT_P2:
				// failsafe into multicopter mode
			//_vtol_schedule.flight_mode = MC_MODE;
		/**********************************************************/
			//rain 2018-4-9 09:19:18
			//进入FAILSAFE_MODE
			_vtol_schedule.flight_mode 	= FAILSAFE_MODE;
			_vtol_schedule.transition_start = hrt_absolute_time();
		_flag_was_in_trans_mode = false;
		/*************************************************************/
			break;

		case TRANSITION_BACK:
			if (_tilt_control <= _params_tiltrotor.tilt_mc) {
				_vtol_schedule.flight_mode = MC_MODE;
				//rain 2018-4-9 22:03:06
				//trans_back刚完成阶段是不是要程序中设定为定高模式，如需在此处加入
			}

			break;
			/************************************************/
			// rain 2018-4-9 08:45:59
		case FAILSAFE_MODE://FAILSAFE_MODE的完成判断条件
			if (_tilt_control <= _params_tiltrotor.tilt_mc) {
				_vtol_schedule.flight_mode = MC_MODE;
				_failsafe_flag = false;//清除失败保护进入标志
			}

			break;
		/*************************************************/
		}
		

	} else {

		switch (_vtol_schedule.flight_mode) {
		case MC_MODE:
			// initialise a front transition
			_vtol_schedule.flight_mode 	= TRANSITION_FRONT_P1;
			_vtol_schedule.transition_start = hrt_absolute_time();
			break;

		case FW_MODE:
			break;

		case TRANSITION_FRONT_P1: {
				// allow switch if we are not armed for the sake of bench testing
				bool transition_to_p2 = can_transition_on_ground();

				// check if we have reached airspeed to switch to fw mode
				transition_to_p2 |= _params_tiltrotor.airspeed_mode != control_state_s::AIRSPD_MODE_DISABLED &&
						    _airspeed->indicated_airspeed_m_s >= _params_tiltrotor.airspeed_trans &&
						    (float)hrt_elapsed_time(&_vtol_schedule.transition_start) > (_params->front_trans_time_min * 1e6f);

				//add by rain 2018-4-8
				// check if airspeed is estimate and transition by time
				transition_to_p2 |= _params_tiltrotor.airspeed_mode == control_state_s::AIRSPD_MODE_EST &&
						    _tilt_control > _params_tiltrotor.tilt_transition &&
						    (float)hrt_elapsed_time(&_vtol_schedule.transition_start) > (_params->front_trans_time_openloop * 1e6f);
				
				// check if airspeed is invalid and transition by time
				transition_to_p2 |= _params_tiltrotor.airspeed_mode == control_state_s::AIRSPD_MODE_DISABLED &&
						    _tilt_control > _params_tiltrotor.tilt_transition &&
						    (float)hrt_elapsed_time(&_vtol_schedule.transition_start) > (_params->front_trans_time_openloop * 1e6f);

				if (transition_to_p2) {
					_vtol_schedule.flight_mode = TRANSITION_FRONT_P2;
					_vtol_schedule.transition_start = hrt_absolute_time();
				}

				break;
			}

		case TRANSITION_FRONT_P2:

			// if the rotors have been tilted completely we switch to fw mode
			if (_tilt_control >= _params_tiltrotor.tilt_fw) {
				_vtol_schedule.flight_mode = FW_MODE;
				_tilt_control = _params_tiltrotor.tilt_fw;
			}

			break;

		case TRANSITION_BACK:
			 //貌似永远进不到这里 rain 2018-4-9 08:54:37
			// failsafe into fixed wing mode
			_vtol_schedule.flight_mode = FW_MODE;
			break;
			/************************************************/
			// rain 2018-4-9 08:45:59
			//疑问：哪种情况下会进入这里？
		case FAILSAFE_MODE://FAILSAFE_MODE的失控处理程序
			if (_tilt_control <= _params_tiltrotor.tilt_transition) {
				_vtol_schedule.flight_mode = MC_MODE;
			}
			else if (_tilt_control >= _params_tiltrotor.tilt_transition) {
				_vtol_schedule.flight_mode = FW_MODE;
			}

			break;
			/*************************************************/
		}
	}

	// map tiltrotor specific control phases to simple control modes
	switch (_vtol_schedule.flight_mode) {
	case MC_MODE:
		_vtol_mode = ROTARY_WING;
		_vtol_vehicle_status->vtol_in_trans_mode = false;
		_flag_was_in_trans_mode = false;
		break;

	case FW_MODE:
		_vtol_mode = FIXED_WING;
		_vtol_vehicle_status->vtol_in_trans_mode = false;
		_flag_was_in_trans_mode = false;
		break;

	case TRANSITION_FRONT_P1:
	case TRANSITION_FRONT_P2:
		_vtol_mode = TRANSITION_TO_FW;
		_vtol_vehicle_status->vtol_in_trans_mode = true;
		break;

	case TRANSITION_BACK:
		_vtol_mode = TRANSITION_TO_MC;
		_vtol_vehicle_status->vtol_in_trans_mode = true;
		break;
		/******************************/
	//rain 2018-4-9 09:00:46
	//无论_vtol_mode = TRANSITION_TO_MC还是TRANSITION_TO_FW
	//最终都会进入转换阶段处理程序void Tiltrotor::update_transition_state()
	case FAILSAFE_MODE:
	_vtol_mode = TRANSITION_TO_MC;
	_vtol_vehicle_status->vtol_in_trans_mode = true;
		break;
	/*******************************/
	}
}

void Tiltrotor::update_mc_state()
{
	VtolType::update_mc_state();

	// make sure motors are not tilted
	_tilt_control = _params_tiltrotor.tilt_mc;

	// enable rear motors
	if (_rear_motors != ENABLED) {
		set_rear_motor_state(ENABLED);
	}

	// set idle speed for rotary wing mode
	if (!flag_idle_mc) {
		set_idle_mc();
		flag_idle_mc = true;
	}
}

void Tiltrotor::update_fw_state()
{
	VtolType::update_fw_state();

	// make sure motors are tilted forward
	_tilt_control = _params_tiltrotor.tilt_fw;

	// disable rear motors
	if (_rear_motors != DISABLED) {
		set_rear_motor_state(DISABLED);
	}

	// adjust idle for fixed wing flight
	if (flag_idle_mc) {
		set_idle_fw();
		flag_idle_mc = false;
	}
}

void Tiltrotor::update_transition_state()
{
	VtolType::update_transition_state();


		/*******************************************/
	//static struct debug_key_value_s dbg_roll_weight = { 0, 0, 0.0f, "mc_roll_w" };
	//static struct debug_key_value_s dbg_pitch_weight = { 0, 0, 0.0f, "mc_pit_w" };
	static struct debug_key_value_s dbg_yaw_weight = { 0, 0, 0.0f, "mc_yaw_w" };
	//if(pub_dbg_roll_weight == nullptr){
	//if(pub_dbg_pitch_weight == nullptr){
	if(pub_dbg_yaw_weight == nullptr){
	//	pub_dbg_roll_weight = orb_advertise(ORB_ID(debug_key_value), &dbg_roll_weight);
	//	pub_dbg_pitch_weight = orb_advertise(ORB_ID(debug_key_value), &dbg_pitch_weight);
		pub_dbg_yaw_weight = orb_advertise(ORB_ID(debug_key_value), &dbg_yaw_weight);
	}else{
		//推送需要查看的变量值
	//	dbg_roll_weight.value = _mc_roll_weight * 1000 + 1000;
	//	dbg_pitch_weight.value = _mc_pitch_weight * 1000 + 1000;
		dbg_yaw_weight.value = _mc_yaw_weight * 1000 + 1000;
	//	orb_publish(ORB_ID(debug_key_value), pub_dbg_roll_weight, &dbg_roll_weight);
	//	orb_publish(ORB_ID(debug_key_value), pub_dbg_pitch_weight, &dbg_pitch_weight);
		orb_publish(ORB_ID(debug_key_value), pub_dbg_yaw_weight, &dbg_yaw_weight);
		}
		/*******************************************/

	if (!_flag_was_in_trans_mode) {
		// save desired heading for transition and last thrust value
		_flag_was_in_trans_mode = true;
			/*******************************************/
	//rain 2018-4-9 10:59:47
	//考虑一下是不是要换成_mc_virtual_att_sp？
	//_thrust_transition_start = _mc_virtual_att_sp->thrust;
	_thrust_transition_start = _v_att_sp->thrust;

	/********************************************/
	}

	if (_vtol_schedule.flight_mode == TRANSITION_FRONT_P1) {
		// for the first part of the transition the rear rotors are enabled
		if (_rear_motors != ENABLED) {
			set_rear_motor_state(ENABLED);
		}

		// tilt rotors forward up to certain angle
		if (_tilt_control <= _params_tiltrotor.tilt_transition) {
			_tilt_control = _params_tiltrotor.tilt_mc +
					fabsf(_params_tiltrotor.tilt_transition - _params_tiltrotor.tilt_mc) * (float)hrt_elapsed_time(
						&_vtol_schedule.transition_start) / (_params_tiltrotor.front_trans_dur * 1000000.0f);
		}

		bool use_airspeed = _params_tiltrotor.airspeed_mode != control_state_s::AIRSPD_MODE_DISABLED;
		/*****************************************************************************
		//rain 2018-4-9 23:01:06
		//当飞行速度小于airspeed_trans时，增加油门值
		// create time dependant throttle signal higher than  in MC and growing untill  P2 switch speed reached 
		if (use_airspeed &&_airspeed->indicated_airspeed_m_s <= _params_tiltrotor.airspeed_trans) {// 飞行器油门设置点的控制
			_thrust_transition = _thrust_transition_start + (fabsf(THROTTLE_TRANSITION_MAX * _thrust_transition_start) *
						 (float)hrt_elapsed_time(&_vtol_schedule.transition_start) / (_params_tiltrotor.front_trans_dur * 1000000.0f));
			_thrust_transition = math::constrain(_thrust_transition, _thrust_transition_start,
								 (1.0f + THROTTLE_TRANSITION_MAX) * _thrust_transition_start);
		}
		//当不使用空速计时(为方便测试，暂时先去掉!)，以倾转角为依据，随时间增加油门值
		else if(!use_airspeed && _tilt_control <= _params_tiltrotor.tilt_transition){
		_thrust_transition = _thrust_transition_start + (fabsf(THROTTLE_TRANSITION_MAX * _thrust_transition_start) *
								 (float)hrt_elapsed_time(&_vtol_schedule.transition_start) / (_params_tiltrotor.front_trans_dur * 1000000.0f));
		_thrust_transition = math::constrain(_thrust_transition, _thrust_transition_start,
										 (1.0f + THROTTLE_TRANSITION_MAX) * _thrust_transition_start);
		}
		
		*****************************************************************************/
		//当不使用空速计时(为方便测试，暂时先去掉!)，以倾转角为依据，随时间增加油门值
		if(use_airspeed && _tilt_control <= _params_tiltrotor.tilt_transition){
		_thrust_transition = _thrust_transition_start + (fabsf(THROTTLE_TRANSITION_MAX * _thrust_transition_start) *
								 (float)hrt_elapsed_time(&_vtol_schedule.transition_start) / (_params_tiltrotor.front_trans_dur * 1000000.0f));
		_thrust_transition = math::constrain(_thrust_transition, _thrust_transition_start,
										 (1.0f + THROTTLE_TRANSITION_MAX) * _thrust_transition_start);
		}
		//rain 2018-4-9 23:01:06
		
		// at low speeds give full weight to MC
		//_mc_roll_weight = 1.0f;
		//_mc_yaw_weight = 1.0f;
	/**********************************************/
		//rain 2018-4-9 22:33:02
		//对于试验机型，感觉应该加入pitch的控制律分配
		//_mc_pitch_weight = 1.0f;
		/***********************************************/

		/***********************************************/
		//rain 2018-4-9 22:40:08
		//对于倾转机翼的飞机来说有点类似尾垂式飞机，应该根据倾转角分配控制律
	
		//MC-VT_TILT_TRANS_L阶段使用MC控制律
		//MC-trans阶段   混控
		if (_tilt_control <= VT_TILT_TRANS_L) {
		_mc_roll_weight = 1.0f;
		_mc_pitch_weight = 1.0f;
		_mc_yaw_weight = 1.0f;
		}
		else if(_tilt_control <= _params_tiltrotor.tilt_transition){
		//change _mc_control_weight
		//根据倾转角函数逐渐减小mc控制权重（1.0到0.0）
		_mc_roll_weight = (float)(_params_tiltrotor.tilt_transition -_tilt_control )/(_params_tiltrotor.tilt_transition - VT_TILT_TRANS_L);

		_mc_pitch_weight = _mc_roll_weight;
		_mc_yaw_weight = _mc_roll_weight;
		}	
		/*************************************************/		
		
		
		//当速度大于ARSP_YAW_CTRL_DISABLE时，，禁止偏航控制		
		// reduce MC controls once the plane has picked up speed
		if (use_airspeed && _airspeed->indicated_airspeed_m_s > ARSP_YAW_CTRL_DISABLE) {
			_mc_yaw_weight = 0.0f;
		}

			
		//_thrust_transition = _mc_virtual_att_sp->thrust;
			   //rain 2018-4-9 23:17:45
	   //P1阶段结束以后，转换过程中MC的控制律已经完全减到0，即P2阶段完全由FW控制律控制

	} else if (_vtol_schedule.flight_mode == TRANSITION_FRONT_P2) {
		// the plane is ready to go into fixed wing mode, tilt the rotors forward completely
		_tilt_control = _params_tiltrotor.tilt_transition +
				fabsf(_params_tiltrotor.tilt_fw - _params_tiltrotor.tilt_transition) * (float)hrt_elapsed_time(
					&_vtol_schedule.transition_start) / (_params_tiltrotor.front_trans_dur_p2 * 1000000.0f);

		_mc_roll_weight = 0.0f;
		_mc_yaw_weight = 0.0f;
//rain 2018-4-9 23:20:50
		//在这里仍然要提的是关于pitch是否要加入控制
		_mc_pitch_weight = 0.0f;
		// ramp down rear motors (setting MAX_PWM down scales the given output into the new range)
		int rear_value = (1.0f - (float)hrt_elapsed_time(&_vtol_schedule.transition_start) /
				  (_params_tiltrotor.front_trans_dur_p2 *
				   1000000.0f)) * (float)(PWM_DEFAULT_MAX - PWM_DEFAULT_MIN) + (float)PWM_DEFAULT_MIN;

		set_rear_motor_state(VALUE, rear_value);
//实验中如果油门值thrust曲线进入P2时骤降，原因很可能在这里
		//_thrust_transition = _mc_virtual_att_sp->thrust;
	    //_thrust_transition = _v_att_sp->thrust;

	} else if (_vtol_schedule.flight_mode == TRANSITION_BACK) {
			/**************************************/
		//rain 2018-4-9 20:13:17
		//fw模式下后尾电机处于关闭DISABLE状态,trans_back阶段需使用其实现配平
		//主推力电机在FW模式下脉宽基础上增加，由油门量控制
		// enable rear motors
		if (_rear_motors != IDLE) {
			set_rear_motor_state(IDLE);
		}

		if (!flag_idle_mc) {
			set_idle_mc();
			flag_idle_mc = true;
		}

		/*******************************************
	static struct debug_key_value_s dbg = { 0, 0, 0.0f, "trans_b" };
	if(pub_dbg == nullptr){
		pub_dbg = orb_advertise(ORB_ID(debug_key_value), &dbg);
	}else{
		//推送需要查看的变量值
		dbg.value = fabsf(VT_THRUST_HOVER - _thrust_transition_start);
		orb_publish(ORB_ID(debug_key_value), pub_dbg, &dbg);
		}
		*******************************************/

		//rain 2018-4-9 21:38:10
		//疑问：正常情况下巡航油门值应该小于悬停油门值，这样的话下面的处理没问题
		//如果，巡航油门值大于悬停，飞机的爬升速度会很大
		if (fabsf((VT_THRUST_HOVER - _thrust_transition_start)) <= 0.2f){
		_thrust_transition_start = math::max(_thrust_transition_start, VT_THRUST_HOVER);
		}else{
			if((VT_THRUST_HOVER - _thrust_transition_start) >= 0.0f){
			//_thrust_transition_start增加到VT_THRUST_HOVER
			_thrust_transition_start += 0.1f;  //避免飞机转速加速太快，
			}

		}

			//转换前油门值已经大于悬停油门设定值，直接 *0.9f
			_thrust_transition = _thrust_transition_start * 0.9f;

		/**************************************/
		
		// tilt rotors back
		if (_tilt_control > _params_tiltrotor.tilt_mc) {
			_tilt_control = _params_tiltrotor.tilt_fw -
					fabsf(_params_tiltrotor.tilt_fw - _params_tiltrotor.tilt_mc) * (float)hrt_elapsed_time(
						&_vtol_schedule.transition_start) / (_params_tiltrotor.back_trans_dur * 1000000.0f);
		}
		/************************************************************************/	
		//FW-trans阶段使用FW控制律
		//trans-VT_TILT_TRANS_L阶段混控
		//VT_TILT_TRANS_L-MC阶段MC控制律
		//VT_TILT_TRANS_L在本.cpp文件开头宏定义 
		if (_tilt_control >= _params_tiltrotor.tilt_transition) {
		_mc_roll_weight = 0.0f;
		_mc_pitch_weight = 0.0f;
		_mc_yaw_weight = 0.0f;
		}
		else if(_tilt_control >= VT_TILT_TRANS_L){
		//change _mc_control_weight
		//根据倾转角函数逐渐增加mc控制权重
		_mc_roll_weight = (float)(_params_tiltrotor.tilt_transition -_tilt_control )/(_params_tiltrotor.tilt_transition - VT_TILT_TRANS_L);

		_mc_pitch_weight = _mc_roll_weight;
		_mc_yaw_weight = _mc_roll_weight;

		}	
		else if(_tilt_control >= _params_tiltrotor.tilt_mc){
		_mc_roll_weight = 1.0f;
		_mc_pitch_weight = 1.0f;
		_mc_yaw_weight = 1.0f;
		}
		
		/**************************************************************************/
		//rain 2018-4-9 21:56:32
		//一下两行需要屏蔽
		//疑问：trans_back阶段要不要加入_mc_yaw_weight = 0.0f;的限制（源程序中是有限制的）

		// set zero throttle for backtransition otherwise unwanted moments will be created
		//_actuators_mc_in->control[actuator_controls_s::INDEX_THROTTLE] = 0.0f;

		//_mc_roll_weight = 0.0f;

	}
	/***************************************************************************/
	//rain 2018-4-9 09:07:15
	//终止转换下，根据舵面所处位置分配控制权重比，某些过程类似tailsitter的trans_back阶段
	else if (_vtol_schedule.flight_mode == FAILSAFE_MODE) {
	//油门转速：加速
	//rain 2018-4-9 20:13:17
	//fw模式下后尾电机处于关闭DISABLE状态,trans_back阶段需使用其实现配平
	//主推力电机在FW模式下脉宽基础上增加，由油门量控制
	// enable rear motors
	if (_rear_motors == DISABLED) {
		set_rear_motor_state(ENABLED);
	   }

	if (!flag_idle_mc) {
		set_idle_mc();
		flag_idle_mc = true;
	}


		/*******************************************
		//用于查看hover与但前值的比较结果
	static struct debug_key_value_s dbg_fs = { 0, 0, 0.0f, "trans_fs" };
	if(pub_dbg_fs == nullptr){
		//rain 2018-4-12 09:51:07
		// advertise debug value 
		pub_dbg_fs = orb_advertise(ORB_ID(debug_key_value), &dbg_fs);
	}else{
		//推送需要查看的变量值
		dbg_fs.value =  _thrust_transition_start;
		orb_publish(ORB_ID(debug_key_value), pub_dbg_fs, &dbg_fs);
		}
		*******************************************/
	//rain 2018-4-9 21:38:10
	//疑问：正常情况下巡航油门值应该小于悬停油门值，这样的话下面的处理没问题
	//如果，巡航油门值大于悬停，飞机的爬升速度会很大
	if(fabsf(VT_THRUST_HOVER - _thrust_transition_start) <= 0.2f){
	_thrust_transition_start = math::max(_thrust_transition_start, VT_THRUST_HOVER);
		}
	else{
		if((VT_THRUST_HOVER - _thrust_transition_start) >= 0.0f){//_thrust_transition_start小于悬停值0.2以上
		//_thrust_transition_start增加到VT_THRUST_HOVER
		_thrust_transition_start += 0.1f;  //避免飞机转速加速太快，
			}
	}
		//转换前油门值已经大于悬停油门设定值，直接 *0.9f
		_thrust_transition = _thrust_transition_start * 0.9f;

	//rain 2018-4-10 00:09:46
	//记录失败转换时的倾转角并计算所需要的转换时间，保持与trans_back阶段的变化率相同
	if(!_failsafe_flag){
		_failsafe_flag = true;//避免多次进入，造成转换时间变化	
		_tilt_failsafe = _tilt_control;
		_failsafe_trans_dur = ((_tilt_control - _params_tiltrotor.tilt_mc)/_params_tiltrotor.tilt_fw) * _params_tiltrotor.back_trans_dur;

		//_failsafe_trans_dur = 3.0f;
		
		_tilt_failsafe =  math::constrain(_tilt_failsafe, 0.0f, 1.0f);
		_failsafe_trans_dur  = math::constrain(_failsafe_trans_dur, 0.0f, _params_tiltrotor.back_trans_dur);
	}
	
	// tilt rotors back
	if (_tilt_control > _params_tiltrotor.tilt_mc) {
		_tilt_control = _tilt_failsafe - 
				fabsf(_tilt_failsafe - _params_tiltrotor.tilt_mc) * (float)hrt_elapsed_time(
					&_vtol_schedule.transition_start) / (_failsafe_trans_dur * 1000000.0f);
	}

	//根据倾转角分配控制律权重
	//fw(1.0f) > trans > TRANS_L >mc(0.0f)
	//FW-trans阶段使用FW控制律
	//trans-VT_TILT_TRANS_L阶段混控
	//VT_TILT_TRANS_L-MC阶段MC控制律
	//VT_TILT_TRANS_L在本.cpp文件开头宏定义 
	if (_tilt_control >= _params_tiltrotor.tilt_transition) {
	_mc_roll_weight = 0.0f;
	_mc_pitch_weight = 0.0f;
	_mc_yaw_weight = 0.0f;
	}
	else if(_tilt_control >= VT_TILT_TRANS_L){
	//change _mc_control_weight
	//根据倾转角函数逐渐增加mc控制权重由0.0f增加到1.0f
	_mc_roll_weight = (float)(_params_tiltrotor.tilt_transition -_tilt_control )/(_params_tiltrotor.tilt_transition - VT_TILT_TRANS_L);

	_mc_pitch_weight = _mc_roll_weight;
	_mc_yaw_weight = _mc_roll_weight;
	//与trans_front对比，这里是不是也要考虑空速对yaw分量的限制

	}	
	else if(_tilt_control >= _params_tiltrotor.tilt_mc){
	_mc_roll_weight = 1.0f;
	_mc_pitch_weight = 1.0f;
	_mc_yaw_weight = 1.0f;
	}	


	}
	
	/****************************************************************************/

	_mc_roll_weight = math::constrain(_mc_roll_weight, 0.0f, 1.0f);
	_mc_pitch_weight = math::constrain(_mc_pitch_weight, 0.0f, 1.0f);
	_mc_yaw_weight = math::constrain(_mc_yaw_weight, 0.0f, 1.0f);

	// copy virtual attitude setpoint to real attitude setpoint (we use multicopter att sp)
	_mc_virtual_att_sp->thrust = _thrust_transition;
	memcpy(_v_att_sp, _mc_virtual_att_sp, sizeof(vehicle_attitude_setpoint_s));
}

void Tiltrotor::waiting_on_tecs()
{
	// keep multicopter thrust until we get data from TECS
	_v_att_sp->thrust = _thrust_transition;
}

/********************************/
//rain 2018-4-12
void Tiltrotor::publish_mc_weight()
{
	if(pub_dbg_roll != nullptr) {
		// publish the attitude setpoint 
		dbg_roll->value = _mc_roll_weight;
		orb_publish(ORB_ID(debug_key_value), pub_dbg_roll, &dbg_roll);

	} else {
		// advertise and publish 
		//dbg_roll->key = "velx";
		memcpy(&(dbg_roll->key), "velx",sizeof(dbg_roll->key));
		pub_dbg_roll = orb_advertise(ORB_ID(debug_key_value), &dbg_roll);
	}
}

/**********************************/



/**
* Write data to actuator output topic.
*/
void Tiltrotor::fill_actuator_outputs()
{
	_actuators_out_0->timestamp = _actuators_mc_in->timestamp;
	_actuators_out_0->control[actuator_controls_s::INDEX_ROLL] = _actuators_mc_in->control[actuator_controls_s::INDEX_ROLL]
			* _mc_roll_weight;
	_actuators_out_0->control[actuator_controls_s::INDEX_PITCH] =
		_actuators_mc_in->control[actuator_controls_s::INDEX_PITCH] * _mc_pitch_weight;
	_actuators_out_0->control[actuator_controls_s::INDEX_YAW] = _actuators_mc_in->control[actuator_controls_s::INDEX_YAW] *
			_mc_yaw_weight;

	if (_vtol_schedule.flight_mode == FW_MODE) {
		_actuators_out_0->control[actuator_controls_s::INDEX_THROTTLE] =
			_actuators_fw_in->control[actuator_controls_s::INDEX_THROTTLE];

		/* allow differential thrust if enabled */
		if (_params_tiltrotor.diff_thrust == 1) {
			_actuators_out_0->control[actuator_controls_s::INDEX_ROLL] =
				_actuators_fw_in->control[actuator_controls_s::INDEX_YAW] * _params_tiltrotor.diff_thrust_scale;
		}

	} else {
		_actuators_out_0->control[actuator_controls_s::INDEX_THROTTLE] =
			_actuators_mc_in->control[actuator_controls_s::INDEX_THROTTLE];;
	}

	_actuators_out_1->timestamp = _actuators_fw_in->timestamp;

	if (_vtol_schedule.flight_mode != MC_MODE) {
		_actuators_out_1->control[actuator_controls_s::INDEX_ROLL] =
			-_actuators_fw_in->control[actuator_controls_s::INDEX_ROLL] * (1 - _mc_roll_weight);
		_actuators_out_1->control[actuator_controls_s::INDEX_PITCH] =
			(_actuators_fw_in->control[actuator_controls_s::INDEX_PITCH] * (1 - _mc_pitch_weight) + _params->fw_pitch_trim);
		_actuators_out_1->control[actuator_controls_s::INDEX_YAW] =
			_actuators_fw_in->control[actuator_controls_s::INDEX_YAW] * (1 - _mc_yaw_weight);	// yaw
	}
	else {
		_actuators_out_1->control[actuator_controls_s::INDEX_ROLL] = 0.0f;
		_actuators_out_1->control[actuator_controls_s::INDEX_PITCH] = 0.0f;
		_actuators_out_1->control[actuator_controls_s::INDEX_YAW] = 0.0f;	
	}

	_actuators_out_1->control[4] = _tilt_control;
}


/**
* Set state of rear motors.
*/

void Tiltrotor::set_rear_motor_state(rear_motor_state state, int value)
{
	int pwm_value = PWM_DEFAULT_MAX;

	// map desired rear rotor state to max allowed pwm signal
	switch (state) {
	case ENABLED:
		pwm_value = PWM_DEFAULT_MAX;
		_rear_motors = ENABLED;
		break;

	case DISABLED:
		pwm_value = PWM_MOTOR_OFF;
		_rear_motors = DISABLED;
		break;

	case IDLE:
		pwm_value = _params->idle_pwm_mc;
		_rear_motors = IDLE;
		break;

	case VALUE:
		pwm_value = value;
		_rear_motors = VALUE;
		break;
	}

	int ret;
	unsigned servo_count;
	const char *dev = PWM_OUTPUT0_DEVICE_PATH;
	int fd = px4_open(dev, 0);

	if (fd < 0) {
		PX4_WARN("can't open %s", dev);
	}

	ret = px4_ioctl(fd, PWM_SERVO_GET_COUNT, (unsigned long)&servo_count);
	struct pwm_output_values pwm_max_values;
	memset(&pwm_max_values, 0, sizeof(pwm_max_values));

	for (int i = 0; i < _params->vtol_motor_count; i++) {
		if (is_motor_off_channel(i)) {
			pwm_max_values.values[i] = pwm_value;

		} else {
			pwm_max_values.values[i] = PWM_DEFAULT_MAX;
		}

		pwm_max_values.channel_count = _params->vtol_motor_count;
	}

	ret = px4_ioctl(fd, PWM_SERVO_SET_MAX_PWM, (long unsigned int)&pwm_max_values);

	if (ret != OK) {
		PX4_WARN("failed setting max values");
	}

	px4_close(fd);
}

bool Tiltrotor::is_motor_off_channel(const int channel)
{
	return (_params_tiltrotor.fw_motors_off >> channel) & 1;
}
