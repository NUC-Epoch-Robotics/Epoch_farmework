#include "pid.h"
#include "stm32f4xx.h"
#include "math.h"

#define ABS(x)		((x>0)? x: -x)
#define RAMP_YawUp_Acc 150
#define RAMP_YawDown_Acc 150

PID_TypeDef pid_pitch, pid_pithch_speed, pid_roll, pid_roll_speed, pid_yaw_speed;

/**
  * @brief          PID参数初始化
  * @param[in]      none
  * @retval         none
  */
static void pid_param_init( PID_TypeDef *pid,
							PID_ID   id,
							float maxout,
							float intergral_limit,
							float deadband,
							float period,
							float  max_err,
							float  target,
							float 	kp,
							float 	ki,
							float 	kd,
							PID_Improvement_e Improve)
{
	pid->id = id;

	pid->ControlPeriod = period;             //没用到
	pid->DeadBand = deadband;
	pid->IntegralLimit = intergral_limit;
	pid->MaxOutput = maxout;
	pid->Max_Err = max_err;
	pid->target = target;

	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
	pid->output = 0;
	pid->Improve = Improve;

}


/**
  * @brief          中途更改参数设定
  */
static void pid_reset(PID_TypeDef *pid, float kp, float ki, float kd)
{
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
}


/**
  * @brief          pid计算
  */
static float pid_calculate(PID_TypeDef *pid, float measure)//, int16_t target)
{
	//	uint32_t time,lasttime;

	pid->lasttime = pid->thistime;
	pid->thistime = HAL_GetTick();
	pid->dtime = pid->thistime - pid->lasttime;
	pid->measure = measure;
	//	pid->target = target;

	pid->last_err  = pid->err;
	pid->last_output = pid->output;

	pid->err = pid->target - pid->measure;

	//是否进入死区
	if((ABS(pid->err) > pid->DeadBand))
	{
		pid->pout = pid->kp * pid->err;
		pid->iout += (pid->ki * pid->err);


		pid->dout =  pid->kd * (pid->err - pid->last_err);

		//积分是否超出限制
		if(pid->iout > pid->IntegralLimit)
			pid->iout = pid->IntegralLimit;
		if(pid->iout < - pid->IntegralLimit)
			pid->iout = - pid->IntegralLimit;
		//pid输出和
		pid->output = pid->pout + pid->iout + pid->dout;
		
  if(pid->Improve & PID_Ramp)
	{
	float desiredVel = 0.0f;
	float signVel =	 1.0f;
		if (pid->err>1000)
		{
			if((pid->err) < 0.0f)signVel = -1.0f;
			desiredVel = signVel*__sqrtf(2.0f*0.7f*RAMP_YawDown_Acc*signVel*(pid->err));//斜坡函数
			if(fabsf(desiredVel) < fabsf(pid->output)) pid->output = desiredVel;			
		}
		if(pid->err<-1000)
		{	
    		if((pid->err) < 0.0f)signVel = -1.0f;
			desiredVel = signVel*__sqrtf(2.0f*0.7f*RAMP_YawUp_Acc*signVel*(pid->err));//抬升斜坡函数，由于抬升力量需要较大，所以写大点。
			if(fabsf(desiredVel) < fabsf(pid->output)) pid->output = desiredVel;
		}
	}



		//pid->output = pid->output*0.7f + pid->last_output*0.3f;  //滤波？
		if(pid->output > pid->MaxOutput)
		{
			pid->output = pid->MaxOutput;
		}
		if(pid->output < -(pid->MaxOutput))
		{
			pid->output = -(pid->MaxOutput);
		}

	}


	return pid->output;
}


/**
  * @brief          pid结构体初始化，每一个pid参数需要调用一次
  */
void pid_init(PID_TypeDef *pid)
{
	pid->f_param_init = pid_param_init;
	pid->f_pid_reset = pid_reset;
	pid->f_cal_pid = pid_calculate;
}