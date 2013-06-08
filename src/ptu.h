/*************************************************************************
*****                PTU BINARY OPCODES INCLUDE FILE                 *****
*****                                                                *****
*****               (C)1997, Directed Perception, Inc.               *****
*****                     All Rights Reserved.                       *****
*****                                                                *****
*****   Licensed users may freely distribute compiled code including *****
*****   this code and data. Source data and code may NOT be          *****
*****   distributed without the prior written consent from           *****
*****   Directed Perception, Inc.                                    *****
*****	      Directed Perception, Inc. reserves the right to make   *****
*****   changes without further notice to any content herein to      *****
*****   improve reliability, function or design. Directed Perception *****
*****   shall not assume any liability arising from the application  *****
*****   or use of this code, data or function.                       *****
*****                                                                *****
**************************************************************************

CHANGE HISTORY:
    8/10/98:    v1.08.09.  In firmware_version_OK, removed addressing to string constant.
    7/15/97:    v1.08.00.  Compiles with MSVC v1.52. Unified with Win16/32
                           PTU interface calls.
    6/20/97:    #define of ALL misdefined. Fixed to equal PAN+TILT
    11/2/95:    v1.07.07d. Firmware version check bug fixed.
    7/11/95:    v1.07.06d. Updated opcode structure and added new support.
    2/19/95:	v1.07.04d. Generalized for Windows, DOS, & UNIX.
	 		   Added networking.
    10/12/94:	v1.07.03d. Pre-release working version.
	  	  	   XON/XOFF removed from PTU firmware to allow for binary mode.


**************************************************************************/




#include "linuxser.h"
#include "opcodes.h"


/* return status codes */
#define PTU_OK					0
#define PTU_ILLEGAL_COMMAND_ARGUMENT    	1
#define PTU_ILLEGAL_COMMAND			2
#define PTU_ILLEGAL_POSITION_ARGUMENT	        3
#define PTU_ILLEGAL_SPEED_ARGUMENT		4
#define PTU_ACCEL_TABLE_EXCEEDED		5
#define PTU_DEFAULTS_EEPROM_FAULT		6
#define PTU_SAVED_DEFAULTS_CORRUPTED	        7
#define PTU_LIMIT_HIT        			8
#define PTU_CABLE_DISCONNECTED			9
#define PTU_ILLEGAL_UNIT_ID	       		10
#define PTU_ILLEGAL_POWER_MODE			11
#define PTU_RESET_FAILED	       		12
#define PTU_NOT_RESPONDING			13
#define PTU_FIRMWARE_VERSION_TOO_LOW	        14

/********************************************************************
 *****                                                          *****
 *****		For all of these commands, a non-zero return status *****
 ***** 		indicates an error, and it returns that error code. *****
 *****															*****
 ********************************************************************/

/* open_host_port(<portname>) ==> <portstream> */
extern portstream_fd open_host_port(char *);

/* close_host_port(<portstream>) ==> <status> */
extern char close_host_port(portstream_fd);



typedef short int PTU_PARM_PTR;

/* reset_PTU_parser(<timeout_in_msec>) ==> [PTU_OK|PTU_NOT_RESPONDING] */
extern char reset_PTU_parser(long);

/* set_desired([PAN|TILT],
					[POSITION|SPEED|ACCELERATION|BASE|UPPER|LOWER],
					[<position>|<speed>|<acceleration>],
					[RELATIVE|ABSOLUTE]) ==> <status>
	set_desired([PAN|TILT],
					HOLD_POWER_LEVEL,
					<power mode>,
					NULL) ==> <status>
	set_desired([PAN|TILT],
					[HOLD_POWER_LEVEL,MOVE_POWER_LEVEL],
					[PTU_REG_POWER|PTU_LOW_POWER|PTU_OFF_POWER],
					NULL) ==> <status>                              */
extern char set_desired(char, char, PTU_PARM_PTR *, char);


/* get_current([PAN|TILT],
					[POSITION|SPEED|ACCELERATION|BASE|UPPER|LOWER|
					 HOLD_POWER_LEVEL|MOVE_POWER_LEVEL|RESOLUTION]) ==> <value> */
extern long get_current(char, char);


/* get_desired([PAN|TILT],
					[POSITION|SPEED|ACCELERATION|BASE|UPPER|LOWER|
					 HOLD_POWER_LEVEL|MOVE_POWER_LEVEL|RESOLUTION]) ==> <value> */
extern long get_desired(char, char);


/* set_mode(COMMAND_EXECUTION_MODE,
				[EXECUTE_IMMEDIATELY|EXECUTE_UPON_IMMEDIATE_OR_AWAIT]) ==> <status>
   set_mode(ASCII_VERBOSE_MODE, [VERBOSE|TERSE|QUERY_MODE]) ==> <status>
   set_mode(ASCII_ECHO_MODE, [ON_MODE|OFF_MODE|QUERY_MODE]) ==> <status>
   set_mode(POSITION_LIMITS_MODE, [ON_MODE|OFF_MODE|QUERY_MODE]) ==> <status>
   set_mode(DEFAULTS,[SAVE_CURRENT_SETTINGS|RESTORE_SAVED_SETTINGS|
		 RESTORE_FACTORY_SETTINGS]) ==> <status> */
extern char set_mode(char,char);


/* halt([ALL|PAN|TILT]) ==> <status>	*/
extern char halt(char);


/* await_completion() ==> <status> */
extern char await_completion(void);


/* reset_PTU() ==> <status> */
extern char reset_ptu(void);


/* firmware_version() ==> <version ID string> */
extern char* firmware_version(void);


/*** multiple unit support ***/
typedef unsigned short int UID_fd;

/* in general, should not be used or required... */
extern char select_host_port(portstream_fd);

/* select_unit(<portstream>, <unit ID>) ==> <status> */
extern char select_unit(portstream_fd, UID_fd);

extern char set_unit_id(UID_fd);   // This call should be made only
								   // when one unit is on the current
								   // host serial port


/********************* function call constants ***********************/

#define PAN		1
#define TILT	        2

#define POSITION		1
#define SPEED			2
#define ACCELERATION	        3
#define BASE			4
#define UPPER_SPEED_LIMIT	5
#define LOWER_SPEED_LIMIT	6
#define MINIMUM_POSITION  	7
#define MAXIMUM_POSITION        8
#define HOLD_POWER_LEVEL	9
#define MOVE_POWER_LEVEL	10
#define RESOLUTION	  	11

/* specifies changes relative to current position */
/* (Had to add conditional compilation since WIN32 already defines these values */
#ifndef RELATIVE
#define RELATIVE	1
#endif
#ifndef ABSOLUTE
#define ABSOLUTE	2
#endif


#define QUERY		NULL

/* power modes */
#define PTU_HI_POWER	1
#define PTU_REG_POWER	2
#define PTU_LOW_POWER	3
#define PTU_OFF_POWER	4

/* PTU mode types */
#define COMMAND_EXECUTION_MODE               1
#define ASCII_VERBOSE_MODE                   2
#define ASCII_ECHO_MODE                      3
#define POSITION_LIMITS_MODE                 4
#define DEFAULTS                             5
#define SPEED_CONTROL_MODE                   6 /* v1.9.7 and higher */

#define EXECUTE_IMMEDIATELY                  1  /* default */
#define EXECUTE_UPON_IMMEDIATE_OR_AWAIT      2

#define VERBOSE				1
#define TERSE				0	/* default */

#define ON_MODE				1  /* default */
#define OFF_MODE                      	0

#define SAVE_CURRENT_SETTINGS		0
#define RESTORE_SAVED_SETTINGS		1
#define RESTORE_FACTORY_SETTINGS	2

#define QUERY_MODE			3

#define ALL		3
