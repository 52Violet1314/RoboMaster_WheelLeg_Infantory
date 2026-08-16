#include "Self_Rescue.h"

//INCLUDE部分
#include "Parameter.h"
#include "Chassis_Task.h"
#include "Some_Functions.h"
//全局变量定义部分        
Self_Rescue_t Self_Rescue; 

/*******************************************************************************************************
翻倒自救(未翻倒)
********************************************************************************************************/
void Up_Save(Flag_Bit_t *flag,
						 Self_Rescue_t *self_re,
						 Leg_Current_Situation_t *leg[2])
{
	switch(self_re->step[0][2]) //左腿
	{
		case 0:
		{
			if(self_re->goal_pos[0]>=-1.0f && self_re->goal_pos[0]<=1.26f)
			{
				
			}
			else
			{
				if(self_re->goal_pos[0]>=-1.0f && self_re->goal_pos[0]<=1.26f) 
				{
					self_re->goal_pos[0]=1.26f;
				}
				self_re->goal_pos[0] = Half_Circle_RADIAN(self_re->goal_pos[0] - FRONT_SELF_RESCUE_SPEED);
			}
			if(leg[0]->abs_leg_theta>=-1.05f && leg[0]->abs_leg_theta<=1.40f)
			{
				self_re->step[0][2] = 1;
			}
			break;
		}
		case 1:
		{
			if(self_re->goal_pos[0] >= 0)
			{
				self_re->goal_pos[0] = Half_Circle_RADIAN(self_re->goal_pos[0] - FRONT_SELF_RESCUE_SPEED);
			}
			else if(self_re->goal_pos[0] < 0)
			{
				self_re->goal_pos[0] = Half_Circle_RADIAN(self_re->goal_pos[0] + FRONT_SELF_RESCUE_SPEED);
			}
			if(self_re->goal_pos[0]>=-0.2f && self_re->goal_pos[0]<=0.2f)
			{
				self_re->goal_pos[0] = 0;
			}
			self_re->col_flag[0] = 1;
			break;
		}
		default:
		{
			break;
		}
	}
	
	switch(self_re->step[1][2]) //右腿
	{
		case 0:
		{
			if(self_re->goal_pos[1]>=-1.0f && self_re->goal_pos[1]<=1.26f)
			{
				
			}
			else
			{
				if(self_re->goal_pos[1]>=-1.0f && self_re->goal_pos[1]<=1.26f) 
				{
					self_re->goal_pos[1]=1.26f;
				}
				self_re->goal_pos[1] = Half_Circle_RADIAN(self_re->goal_pos[1] - FRONT_SELF_RESCUE_SPEED);
			}
			if(leg[1]->abs_leg_theta>=-1.05f && leg[1]->abs_leg_theta<=1.40f)
			{
				self_re->step[1][2] = 1;
			}
			break;
		}
		case 1:
		{
			if(self_re->goal_pos[1] >= 0)
			{
				self_re->goal_pos[1] = Half_Circle_RADIAN(self_re->goal_pos[1] - FRONT_SELF_RESCUE_SPEED);
			}
			else if(self_re->goal_pos[1] < 0)
			{
				self_re->goal_pos[1] = Half_Circle_RADIAN(self_re->goal_pos[1] + FRONT_SELF_RESCUE_SPEED);
			}
			if(self_re->goal_pos[1]>=-0.2f && self_re->goal_pos[1]<=0.2f)
			{
				self_re->goal_pos[1] = 0;
			}
			self_re->col_flag[1] = 1;
			break;
		}
		default:
		{
			break;
		}
	}
	
	//自救完成
	if( (self_re->step[0][2] == 1     && self_re->step[1][2] == 1)    &&
			(leg[0]->abs_leg_theta>=-0.4f && leg[0]->abs_leg_theta<=0.4f) &&
			(leg[1]->abs_leg_theta>=-0.4f && leg[1]->abs_leg_theta<=0.4f)    )
	{
		flag->fall_flag = 0;
	}
}

/*******************************************************************************************************
翻倒自救(翻倒)
********************************************************************************************************/
void Save_Self(Flag_Bit_t *flag,
							 Self_Rescue_t *self_re,
							 Leg_Current_Situation_t *leg[2])
{
	if(self_re->run_flag == 1)
	{
		if(self_re->downed_state == 1 || self_re->downed_state == 2) //车体翻倒
		{
			if(self_re->downed_state == 1) //头着地
			{
				if(self_re->run_first == 0) //先动左腿
				{
					switch(self_re->step[0][0]) //左腿
					{
						case 0:
						{
							if(self_re->goal_pos[0]>=-2.6f && self_re->goal_pos[0]<=-1.57f)
							{
								
							}
							else
							{
								if(self_re->goal_pos[0]>=-2.6f && self_re->goal_pos[0]<=-1.57f)
								{
									self_re->goal_pos[0] = -2.6f;
								}
								self_re->goal_pos[0] = Half_Circle_RADIAN(self_re->goal_pos[0] + BACK_SELF_RESCUE_SPEED);
							}
							if(leg[0]->abs_leg_theta>=-2.9f && leg[0]->abs_leg_theta<=-1.57f)
							{
								self_re->step[0][0] = 1;
							}
							break;
						}
						case 1:
						{
							if(self_re->step[1][0] != 0)
							{
								self_re->goal_pos[0] = Half_Circle_RADIAN(self_re->goal_pos[0] + BACK_SELF_RESCUE_SPEED);
								if(self_re->goal_pos[0]>=-1.57f && self_re->goal_pos[0]<=0.0f)
								{
									self_re->goal_pos[0] = -1.57f;
								}
								if(leg[0]->abs_leg_theta>=-1.77f && leg[0]->abs_leg_theta<=-1.0f)
								{
									self_re->step[0][0] = 2;
								}
							}
							break;
						}
						default:
						{
							break;
						}
					}
					
					switch(self_re->step[1][0]) //右腿
					{
						case 0:
						{
							if(self_re->step[0][0] != 0)
							{
								if(self_re->goal_pos[1]>=-2.6f && self_re->goal_pos[1]<=-1.57f)
								{
									
								}
								else
								{
									if(self_re->goal_pos[1]>=-2.6f && self_re->goal_pos[1]<=-1.57f)
									{
										self_re->goal_pos[1] = -2.6f;
									}
									self_re->goal_pos[1] = Half_Circle_RADIAN(self_re->goal_pos[1] + BACK_SELF_RESCUE_SPEED);
								}
								if(leg[1]->abs_leg_theta>=-2.9f && leg[1]->abs_leg_theta<=-1.57f)
								{
									self_re->step[1][0] = 1;
								}
							}
							break;
						}
						case 1:
						{
							self_re->goal_pos[1] = Half_Circle_RADIAN(self_re->goal_pos[1] + BACK_SELF_RESCUE_SPEED);
							if(self_re->goal_pos[1]>=-1.57f && self_re->goal_pos[1]<=0.0f)
							{
								self_re->goal_pos[1] = -1.57f;
							}
							if(leg[1]->abs_leg_theta>=-1.77f && leg[1]->abs_leg_theta<=-1.0f)
							{
								self_re->step[1][0] = 2;
							}
							break;
						}
						default:
						{
							break;
						}
					}
					
					if(self_re->step[0][0]==2 && self_re->step[1][0]==2)
					{
						Up_Save(flag,self_re,leg);
					}
				}
				else //先动右腿
				{
					switch(self_re->step[0][0]) //左腿
					{
						case 0:
						{
							if(self_re->step[1][0] != 0)
							{
								if(self_re->goal_pos[0]>=-2.6f && self_re->goal_pos[0]<=-1.57f)
								{
									
								}
								else
								{
									if(self_re->goal_pos[0]>=-2.6f && self_re->goal_pos[0]<=-1.57f)
									{
										self_re->goal_pos[0] = -2.6f;
									}
									self_re->goal_pos[0] = Half_Circle_RADIAN(self_re->goal_pos[0] + BACK_SELF_RESCUE_SPEED);
								}
								if(leg[0]->abs_leg_theta>=-2.9f && leg[0]->abs_leg_theta<=-1.57f)
								{
									self_re->step[0][0] = 1;
								}
							}
							break;
						}
						case 1:
						{
							self_re->goal_pos[0] = Half_Circle_RADIAN(self_re->goal_pos[0] + BACK_SELF_RESCUE_SPEED);
							if(self_re->goal_pos[0]>=-1.57f && self_re->goal_pos[0]<=0.0f)
							{
								self_re->goal_pos[0] = -1.57f;
							}
							if(leg[0]->abs_leg_theta>=-1.77f && leg[0]->abs_leg_theta<=-1.0f)
							{
								self_re->step[0][0] = 2;
							}
							break;
						}
						default:
						{
							break;
						}
					}
					
					switch(self_re->step[1][0]) //右腿
					{
						case 0:
						{
							if(self_re->goal_pos[1]>=-2.6f && self_re->goal_pos[1]<=-1.57f)
							{
								
							}
							else
							{
								if(self_re->goal_pos[1]>=-2.6f && self_re->goal_pos[1]<=-1.57f)
								{
									self_re->goal_pos[1] = -2.6f;
								}
								self_re->goal_pos[1] = Half_Circle_RADIAN(self_re->goal_pos[1] + BACK_SELF_RESCUE_SPEED);
							}
							if(leg[1]->abs_leg_theta>=-2.9f && leg[1]->abs_leg_theta<=-1.57f)
							{
								self_re->step[1][0] = 1;
							}
							break;
						}
						case 1:
						{
							if(self_re->step[0][0] != 0)
							{
								self_re->goal_pos[1] = Half_Circle_RADIAN(self_re->goal_pos[1] + BACK_SELF_RESCUE_SPEED);
								if(self_re->goal_pos[1]>=-1.57f && self_re->goal_pos[1]<=0.0f)
								{
									self_re->goal_pos[1] = -1.57f;
								}
								if(leg[1]->abs_leg_theta>=-1.85f && leg[1]->abs_leg_theta<=-1.0f)
								{
									self_re->step[1][0] = 2;
								}
							}
							break;
						}
						default:
						{
							break;
						}
					}
					
					if(self_re->step[0][0]==2 && self_re->step[1][0]==2)
					{
						Up_Save(flag,self_re,leg);
					}
				}
			}
			else //屁股着地
			{
				if(self_re->run_first == 0) //先动左腿
				{
					switch(self_re->step[0][1]) //左腿
					{
						case 0:
						{
							if(self_re->goal_pos[0]>=1.57f && self_re->goal_pos[0]<=2.6f)
							{
								
							}
							else
							{
								if(self_re->goal_pos[0]>=1.57f && self_re->goal_pos[0]<=2.6f)
								{
									self_re->goal_pos[0] = 2.6f;
								}
								self_re->goal_pos[0] = Half_Circle_RADIAN(self_re->goal_pos[0] - BACK_SELF_RESCUE_SPEED);
							}
							if(leg[0]->abs_leg_theta>=1.57f && leg[0]->abs_leg_theta<=2.9f)
							{
								self_re->step[0][1] = 1;
							}
							break;
						}
						case 1:
						{
							if(self_re->step[1][1] != 0)
							{
								self_re->goal_pos[0] = Half_Circle_RADIAN(self_re->goal_pos[0] - BACK_SELF_RESCUE_SPEED);
								if(self_re->goal_pos[0]>=0.0f && self_re->goal_pos[0]<=1.57f)
								{
									self_re->goal_pos[0] = 1.57f;
								}
								if(leg[0]->abs_leg_theta>=1.0f && leg[0]->abs_leg_theta<=1.85f)
								{
									self_re->step[0][1] = 2;
								}
							}
							break;
						}
						default:
						{
							break;
						}
					}
					
					switch(self_re->step[1][1]) //右腿
					{
						case 0:
						{
							if(self_re->step[0][1] != 0)
							{
								if(self_re->goal_pos[1]>=1.57f && self_re->goal_pos[1]<=2.6f)
								{
									
								}
								else
								{
									if(self_re->goal_pos[1]>=1.57f && self_re->goal_pos[1]<=2.6f)
									{
										self_re->goal_pos[1] = 2.6f;
									}
									self_re->goal_pos[1] = Half_Circle_RADIAN(self_re->goal_pos[1] - BACK_SELF_RESCUE_SPEED);
								}
								if(leg[1]->abs_leg_theta>=1.57f && leg[1]->abs_leg_theta<=2.9f)
								{
									self_re->step[1][1] = 1;
								}
							}
							break;
						}
						case 1:
						{
							self_re->goal_pos[1] = Half_Circle_RADIAN(self_re->goal_pos[1] - BACK_SELF_RESCUE_SPEED);
							if(self_re->goal_pos[1]>=0.0f && self_re->goal_pos[1]<=1.57f)
							{
								self_re->goal_pos[1] = 1.57f;
							}
							if(leg[1]->abs_leg_theta>=1.0f && leg[1]->abs_leg_theta<=1.85f)
							{
								self_re->step[1][1] = 2;
							}
							break;
						}
						default:
						{
							break;
						}
					}
					
					if(self_re->step[0][1]==2 && self_re->step[1][1]==2)
					{
						Up_Save(flag,self_re,leg);
					}
				}
				else //先动右腿
				{
					switch(self_re->step[0][1]) //左腿
					{
						case 0:
						{
							if(self_re->step[1][1] != 0)
							{
								if(self_re->goal_pos[0]>=1.57f && self_re->goal_pos[0]<=2.6f)
								{
									
								}
								else
								{
									if(self_re->goal_pos[0]>=1.57f && self_re->goal_pos[0]<=2.6f)
									{
										self_re->goal_pos[0] = 2.6f;
									}
									self_re->goal_pos[0] = Half_Circle_RADIAN(self_re->goal_pos[0] - BACK_SELF_RESCUE_SPEED);
								}
								if(leg[0]->abs_leg_theta>=1.57f && leg[0]->abs_leg_theta<=2.9f)
								{
									self_re->step[0][1] = 1;
								}
							}
							break;
						}
						case 1:
						{
							self_re->goal_pos[0] = Half_Circle_RADIAN(self_re->goal_pos[0] - BACK_SELF_RESCUE_SPEED);
							if(self_re->goal_pos[0]>=0.0f && self_re->goal_pos[0]<=1.57f)
							{
								self_re->goal_pos[0] = 1.57f;
							}
							if(leg[0]->abs_leg_theta>=1.0f && leg[0]->abs_leg_theta<=1.85f)
							{
								self_re->step[0][1] = 2;
							}
							break;
						}
						default:
						{
							break;
						}
					}
					
					switch(self_re->step[1][1]) //右腿
					{
						case 0:
						{
							if(self_re->goal_pos[1]>=1.57f && self_re->goal_pos[1]<=2.6f)
							{
								
							}
							else
							{
								if(self_re->goal_pos[1]>=1.57f && self_re->goal_pos[1]<=2.6f)
								{
									self_re->goal_pos[1] = 2.6f;
								}
								self_re->goal_pos[1] = Half_Circle_RADIAN(self_re->goal_pos[1] - BACK_SELF_RESCUE_SPEED);
							}
							if(leg[1]->abs_leg_theta>=1.57f && leg[1]->abs_leg_theta<=2.9f)
							{
								self_re->step[1][1] = 1;
							}
							break;
						}
						case 1:
						{
							if(self_re->step[0][1] != 0)
							{
								self_re->goal_pos[1] = Half_Circle_RADIAN(self_re->goal_pos[1] - BACK_SELF_RESCUE_SPEED);
								if(self_re->goal_pos[1]>=0.0f && self_re->goal_pos[1]<=1.57f)
								{
									self_re->goal_pos[1] = 1.57f;
								}
								if(leg[1]->abs_leg_theta>=1.0f && leg[1]->abs_leg_theta<=1.85f)
								{
									self_re->step[1][1] = 2;
								}
							}
							break;
						}
						default:
						{
							break;
						}
					}
					
					if(self_re->step[0][1]==2 && self_re->step[1][1]==2)
					{
						Up_Save(flag,self_re,leg);
					}
				}
			}
		}
		else if(self_re->downed_state == 0) //车体正
		{
			Up_Save(flag,self_re,leg);
		}
	}
}

