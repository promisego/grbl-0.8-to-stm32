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


	serial_init(); // Setup serial baud rate and interrupts 全部注释
	settings_init(); // Load grbl settings from EEPROM
	st_init(); // Setup stepper pins and interrupt timers设置步进引脚和中断定时器
	sei(); // Enable interrupts启用中断
	
	memset(&sys, 0, sizeof(sys));  // Clear all system variables清除所有系统变量
	sys.abort = true;   // Set abort to complete initialization设置abort以完成初始化
	sys.state = STATE_INIT;  // Set alarm state to indicate unknown initial position
	                            //设置报警状态以显示未知的初始位置
	for(;;) {
	
	// Execute system reset upon a system abort, where the main program will return to this loop.
	// Once here, it is safe to re-initialize the system. At startup, the system will automatically
	// reset to finish the initialization process.
		/*在系统中止时执行系统重置，主程序将返回到此循环。
		在这里，重新初始化系统是安全的。在启动时，系统将自动运行
		重置以完成初始化过程。*/
		
		if (sys.abort) {
		// Reset system.
			serial_reset_read_buffer(); // Clear serial read buffer清除串口读取缓冲区
			plan_init(); // Clear block buffer and planner variables清除块缓冲区和计划变量
			gc_init(); // Set g-code parser to default state将g - code解析器设置为默认状态
			protocol_init(); // Clear incoming line data and execute startup lines清除输入的行数据并执行启动行
			spindle_init();       //置主轴的方向为0，并关闭。
			coolant_init();       //冷却液开关初始化
			limits_init();        //限位开关引脚的初始化
			st_reset(); // Clear stepper subsystem variables.清除的步进系统变量。
			
			// Sync cleared gcode and planner positions to current system position, which is only
			// cleared upon startup, not a reset/abort. 
			 //同步清除gcode和planner位置到当前系统位置，这只是在启动时清除，而不是重置/中止。名词
			
			sys_sync_current_position();   //将所有内部位置向量同步到当前系统位置。
			
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
			/*检查电源，并设置系统警报，如果使寻的能够强制回家循环
			设置Grbl的告警状态。警报锁定所有g代码命令，包括
			启动脚本，但允许访问设置和内部命令。只有一个导航
			循环“$ H”或“杀死闹钟”锁定“$ X”将禁用警报。
			注意:启动脚本将在成功完成homing周期后运行，但是
			在禁用警报锁之后。防止启动块进入
			不受控制的事情。非常糟糕。*/
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




