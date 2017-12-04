/*
  nuts_bolts.h - Header file for shared definitions, variables, and functions
  Part of Grbl

  The MIT License (MIT)

  GRBL(tm) - Embedded CNC g-code interpreter and motion-controller
  Copyright (c) 2009-2011 Simen Svale Skogsrud
  Copyright (c) 2011-2012 Sungeun K. Jeon

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in
  all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
  THE SOFTWARE.
*/

#ifndef nuts_bolts_h
#define nuts_bolts_h

//#include <string.h>
//#include <stdint.h>
//#include <stdbool.h>
//#include "config.h"
//#include "defaults.h"
//#include "pin_map.h"

#include "include.h"

//#define false 0
//#define true 1

#define N_AXIS 3 // Number of axes
#define X_AXIS 0 // Axis indexing value
#define Y_AXIS 1
#define Z_AXIS 2

#define MM_PER_INCH (25.40)
#define INCH_PER_MM (0.0393701)

// Useful macros
#define clear_vector(a) memset(a, 0, sizeof(a))
#define clear_vector_float(a) memset(a, 0.0, sizeof(float)*N_AXIS)
#define max(a,b) (((a) > (b)) ? (a) : (b))
#define min(a,b) (((a) < (b)) ? (a) : (b))

// Bit field and masking macros
#define bit(n) (1 << n) 
#define bit_true(x,mask) (x |= mask)
#define bit_false(x,mask) (x &= ~mask)
#define bit_toggle(x,mask) (x ^= mask)
#define bit_istrue(x,mask) ((x & mask) != 0)
#define bit_isfalse(x,mask) ((x & mask) == 0)

// Define system executor bit map. Used internally by runtime protocol as runtime command flags, 
// which notifies the main program to execute the specified runtime command asynchronously.
// NOTE: The system executor uses an unsigned 8-bit volatile variable (8 flag limit.) The default
// flags are always false, so the runtime protocol only needs to check for a non-zero value to 
// know when there is a runtime command to execute.
#define EXEC_STATUS_REPORT  bit(0) // bitmask 00000001
#define EXEC_CYCLE_START    bit(1) // bitmask 00000010
#define EXEC_CYCLE_STOP     bit(2) // bitmask 00000100
#define EXEC_FEED_HOLD      bit(3) // bitmask 00001000
#define EXEC_RESET          bit(4) // bitmask 00010000
#define EXEC_ALARM          bit(5) // bitmask 00100000
#define EXEC_CRIT_EVENT     bit(6) // bitmask 01000000
// #define                  bit(7) // bitmask 10000000

// Define system state bit map. The state variable primarily tracks the individual functions
// of Grbl to manage each without overlapping. It is also used as a messaging flag for
// critical events.
#define STATE_IDLE       0 // Must be zero.
#define STATE_INIT       1 // Initial power up state.初始上电状态。
#define STATE_QUEUED     2 // Indicates buffered blocks, awaiting cycle start.指示缓冲块，等待循环启动。
#define STATE_CYCLE      3 // Cycle is running循环运行
#define STATE_HOLD       4 // Executing feed hold执行进给保持
#define STATE_HOMING     5 // Performing homing cycle执行归航周期
#define STATE_ALARM      6 // In alarm state. Locks out all g-code processes. Allows settings access.在报警状态。锁定所有的g代码进程。允许设置访问。
#define STATE_CHECK_MODE 7 // G-code check mode. Locks out planner and motion only.刀位点检查模式。锁定计划和动作。
// #define STATE_JOG     8 // Jogging mode is unique like homing.

// Define global system variables
typedef struct {
  uint8_t abort;                 // System abort flag. Forces exit back to main loop for reset.系统中止标志。强制退出返回主循环，重新设置
  uint8_t state;                 // Tracks the current state of Grbl.跟踪Grbl的当前状态。
  volatile uint8_t execute;      // Global system runtime executor bitflag variable. See EXEC bitmasks.全局系统运行时执行位标记变量。看到EXEC位掩码
  int32_t position[N_AXIS];      // Real-time machine (aka home) position vector in steps. 实时机器(aka home)位置矢量的步骤。
                                 // NOTE: This may need to be a volatile variable, if problems arise.   注意:如果出现问题，这可能需要是一个volatile变量
  uint8_t auto_start;            // Planner auto-start flag. Toggled off during feed hold. Defaulted by settings.规划师自动启动标志。在喂料时将其关闭。违约的设置。（看门狗）
} system_t;
extern system_t sys;

// Read a floating point value from a string. Line points to the input buffer, char_counter 
// is the indexer pointing to the current character of the line, while float_ptr is 
// a pointer to the result variable. Returns true when it succeeds
int read_float(char *line, uint8_t *char_counter, float *float_ptr);

// Delays variable-defined milliseconds. Compiler compatibility fix for _delay_ms().
void delay_ms(uint16_t ms);

// Delays variable-defined microseconds. Compiler compatibility fix for _delay_us().
void delay_us(uint32_t us);

// Syncs Grbl's gcode and planner position variables with the system position.
void sys_sync_current_position(void);

#endif
