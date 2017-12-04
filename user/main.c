#include "include.h"


system_t sys; 

int main()
{

	Stm32_Clock_Init();		//系统时钟初始化
	delay_init(72);			//延时函数初始化
	HW_GPIO_Init();			//GPIO初始化
	HW_EEPROM_Init();		//flash做EEPROM初始化
	HW_USART_Init(BAUD_RATE);	//串口初始化
	HW_EXTI_Init();			//中断初始化
	HW_TIM_Init();			//定时器初始化


	serial_init(); // Setup serial baud rate and interrupts
	settings_init(); // Load grbl settings from EEPROM
	st_init(); // Setup stepper pins and interrupt timers
	sei(); // Enable interrupts
	
	memset(&sys, 0, sizeof(sys));  // Clear all system variables
	sys.abort = true;   // Set abort to complete initialization
	sys.state = STATE_INIT;  // Set alarm state to indicate unknown initial position
	
	for(;;) {
	
	// Execute system reset upon a system abort, where the main program will return to this loop.
	// Once here, it is safe to re-initialize the system. At startup, the system will automatically
	// reset to finish the initialization process.
		if (sys.abort) {
		// Reset system.
			serial_reset_read_buffer(); // Clear serial read buffer
			plan_init(); // Clear block buffer and planner variables
			gc_init(); // Set g-code parser to default state
			protocol_init(); // Clear incoming line data and execute startup lines清除输入的行数据并执行启动行
			spindle_init();       //置主轴的方向为0，并关闭。
			coolant_init();       //冷却液开关初始化
			limits_init();        //限位开关引脚的初始化
			st_reset(); // Clear stepper subsystem variables.清除的步进系统变量。
			
			// Sync cleared gcode and planner positions to current system position, which is only
			// cleared upon startup, not a reset/abort. 同步清除gcode和planner位置到当前系统位置，这只是在启动时清除，而不是重置/中止。名词
			sys_sync_current_position();
			
			// Reset system variables.
			sys.abort = false;
			sys.execute = 0;
			if (bit_istrue(settings.flags,BITFLAG_AUTO_START)) { sys.auto_start = true; }
			
			// Check for power-up and set system alarm if homing is enabled to force homing cycle
			// by setting Grbl's alarm state. Alarm locks out all g-code commands, including the
			// startup scripts, but allows access to settings and internal commands. Only a homing
			// cycle '$H' or kill alarm locks '$X' will disable the alarm.
			// NOTE: The startup script will run after successful completion of the homing cycle, but
			// not after disabling the alarm locks. Prevents motion startup blocks from crashing into
			// things uncontrollably. Very bad.
			#ifdef HOMING_INIT_LOCK
			if (sys.state == STATE_INIT && bit_istrue(settings.flags,BITFLAG_HOMING_ENABLE)) { sys.state = STATE_ALARM; }
			#endif
			
			// Check for and report alarm state after a reset, error, or an initial power up.在重置、错误或初始电源后检查并报告警报状态。
			if (sys.state == STATE_ALARM) {
				report_feedback_message(MESSAGE_ALARM_LOCK); 
			} else {
			// All systems go. Set system to ready and execute startup script.所有系统。设置系统准备并执行启动脚本。
				sys.state = STATE_IDLE;
				protocol_execute_startup(); 
			}
		}
		
		protocol_execute_runtime();
		protocol_process(); // ... process the serial protocol处理串行协议
		
	}
}


/*	测试用代码
		HW_GPIO_OUT(GPIOB,8,0);
		delay_ms(10);
		HW_GPIO_OUT(GPIOB,8,1);	
*/




