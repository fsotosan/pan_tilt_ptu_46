/*****************************************************************
 *****           PAN-TILT ENCODED COMMAND CODE               *****
 *****                 C include file                        *****
 *****                                                       *****
 *****     (C) 1994, 1998, Directed Perception, Inc.         *****
 *****               All rights reserved.                    *****
 *****************************************************************/

#define OPCODE_VERSION		"1.07.06"


/********* REVISION HISTORY *********/
/*

1.07.06:  9/16/98: Added signed speed special opcodes. Added them at
                   end of opcodes to maintain compatibility with prior
                   firmware versions.
1.07.05d: 2/19/95: reorganized the opcodes. Working on networking.
1.07.04d: 1/26/95: added DEFINEs for reset enable disable opcodes.
  	 	   But unique opcodes not assigned.
10/06/94:	Added and reorganized opcodes
9/18/94:	Initial version

*/



/******************************************************/
/**********  ASYNCHRONOUS EVENT RETURN CODES **********/
/******************************************************/

#define PAN_LIMIT_HIT			220	/*   !P   */
#define TILT_LIMIT_HIT			221	/*   !T   */
#define CABLE_DISCONNECT_DETECTED	222	/*   !D   */
#define PAN_POSITION_TRIGGER_HIT	223	/*   #P   */
#define TILT_POSITION_TRIGGER_HIT	224	/*   #T   */
#define PAN_SPEED_TRIGGER_HIT		225	/*   $P   */
#define TILT_SPEED_TRIGGER_HIT		226	/*   $T   */



/*****************************************/
/**********  RETURN STATE CODES **********/
/*****************************************/

/* power modes */
#define PTU_HI_POWER	1
#define PTU_REG_POWER	2
#define PTU_LOW_POWER	3
#define PTU_OFF_POWER	4

/* speed control modes */
#define PTU_INDEPENDENT_SPEED_CONTROL_MODE      1
#define PTU_PURE_VELOCITY_SPEED_CONTROL_MODE    2

/* return status codes */
#define PTU_OK				0
#define PTU_ILLEGAL_COMMAND		2
#define PTU_ILLEGAL_POSITION_ARGUMENT	3
#define PTU_ILLEGAL_SPEED_ARGUMENT	4
#define PTU_ACCEL_TABLE_EXCEEDED	5
#define PTU_DEFAULTS_EEPROM_FAULT	6
#define PTU_SAVED_DEFAULTS_CORRUPTED	7
#define PTU_LIMIT_HIT        		8
#define PTU_CABLE_DISCONNECTED		9
#define PTU_ILLEGAL_UNIT_ID	       10
#define PTU_ILLEGAL_POWER_MODE	       11
#define PTU_RESET_FAILED	       12


/*******************************************/
/********** INTERNAL RETURN CODES **********/
/*******************************************/

#define INTERACTIVE_MAIN_MENU		 	1
#define DISPLAY_PAN_MOTOR_COMMANDS		2
#define DISPLAY_TILT_MOTOR_COMMANDS		3
#define DISPLAY_MENU_OPTIONS		 	4
#define ILLEGAL_COMMAND_WITH_DELIMITER	        5
#define ASCII_COMMAND_WITH_NO_BINARY_EQUIV	6
#define ILLEGAL_OPCODE		      		7
#define ILLEGAL_ARGUMENT		 	8

/**********************************/
/********** PTU OP CODES **********/
/**********************************/

#define OPCODE_BASE	129
#define MAX_OPCODE	205
#define	NUM_OPCODES	(MAX_OPCODE - OPCODE_BASE + 1)

/*****/								  /*****/
/*****  OPCODES with a 2 byte int ARGUMENT (returns a status byte) *****/
/*****/								  /*****/

#define	PAN_SET_ABS_POS 	      129
#define	TILT_SET_ABS_POS	      130
#define	PAN_SET_REL_POS 	      131
#define	TILT_SET_REL_POS	      132

#define	PAN_SET_REL_SPEED	      133
#define	TILT_SET_REL_SPEED	      134
#define	PAN_SET_ABS_SPEED	      135 /* unsigned in independent speed control */
#define	TILT_SET_ABS_SPEED	      136 /* mode; signed in pure velocity SCM */
                                          /* change made 9/22/98 */

#define LAST_SIGNED_2BYTE_INT_OPCODE	      136

/* unsigned 2 byte int argument */

#define	PAN_SET_BASE_SPEED	      137
#define	TILT_SET_BASE_SPEED	      138

#define	PAN_SET_UPPER_SPEED_LIMIT     139
#define	TILT_SET_UPPER_SPEED_LIMIT    140
#define	PAN_SET_LOWER_SPEED_LIMIT     141
#define	TILT_SET_LOWER_SPEED_LIMIT    142

#define SET_UNIT_ID		              143
#define SELECT_UNIT_ID				  144
#define LAST_2BYTE_INT_OPCODE	      144

/*****/                        /*****/
/***** OPCODES with 0 ARGUMENTS *****/
/*****/                        /*****/

/*** returns 2 byte signed int ***/

#define	PAN_CURRENT_POS_QUERY        145
#define	TILT_CURRENT_POS_QUERY       146
#define	PAN_DESIRED_POS_QUERY        147
#define	TILT_DESIRED_POS_QUERY 	     148

#define	PAN_MIN_POSITION_QUERY       149
#define	TILT_MIN_POSITION_QUERY      150
#define	PAN_MAX_POSITION_QUERY       151
#define	TILT_MAX_POSITION_QUERY      152

/*** returns 2 byte unsigned int ***/

#define	PAN_CURRENT_SPEED_QUERY	     153
#define	TILT_CURRENT_SPEED_QUERY     154
#define	PAN_DESIRED_SPEED_QUERY	     155
#define	TILT_DESIRED_SPEED_QUERY     156

#define	PAN_BASE_SPEED_QUERY	     157
#define	TILT_BASE_SPEED_QUERY	     158

#define	PAN_UPPER_SPEED_LIMIT_QUERY  159
#define	TILT_UPPER_SPEED_LIMIT_QUERY 160
#define	PAN_LOWER_SPEED_LIMIT_QUERY  161
#define	TILT_LOWER_SPEED_LIMIT_QUERY 162

/*** returns 4 byte unsigned int ***/

#define	PAN_ACCEL_QUERY	     		163
#define	TILT_ACCEL_QUERY		164

/* value is 60x arc secs */
#define	PAN_RESOLUTION_QUERY         	165
#define	TILT_RESOLUTION_QUERY        	166

/*** returns signed char (a status byte unless otherwise noted) ***/

#define	AWAIT_COMMAND_COMPLETION     	167

#define	HALT	 			168
#define	HALT_PAN			169
#define	HALT_TILT			170

/* 1 if position limits enabled, 0 otherwise */
#define POSITION_LIMITS_QUERY	     	171
#define	ENABLE_POSITION_LIMITS       	172
#define	DISABLE_POSITION_LIMITS      	173

#define	SET_IMMEDIATE_COMMAND_MODE   	174

#define	SET_SLAVED_COMMAND_MODE      	175

#define	UNIT_RESET  		     	176
#define UNIT_RESET_ON_POWERUP		177	/***** NOT YET SUPPORTED *****/
#define UNIT_RESET_ON_POWERUP_DISABLED	178	/***** NOT YET SUPPORTED *****/

/* returns an unsigned char with the unit_id */
#define UNIT_ID_QUERY		   	179	/***** NOT YET SUPPORTED *****/

/* 1 if echo, 0 otherwise */
#define ECHO_QUERY		     	180
#define	ENABLE_ECHO  		     	181
#define	DISABLE_ECHO  		     	182

#define	SAVE_DEFAULTS  		     	183
#define	RESTORE_SAVED_DEFAULTS          184
#define	RESTORE_FACTORY_DEFAULTS        185

/* returns power mode byte */
#define	PAN_HOLD_POWER_QUERY  	        186
#define	TILT_HOLD_POWER_QUERY  	        187
#define	PAN_MOVE_POWER_QUERY  	        188
#define	TILT_MOVE_POWER_QUERY  	        189

/* 1 if verbose, 0 otherwise */
#define VERBOSE_QUERY		        190
#define SET_VERBOSE_ASCII_ON	        191
#define SET_VERBOSE_ASCII_OFF	        192

/* 1 if joystick enabled, 0 otherwise */
#define JOYSTICK_QUERY		        193		/***** NOT YET SUPPORTED *****/
#define ENABLE_JOYSTICK		        194		/***** NOT YET SUPPORTED *****/
#define DISABLE_JOYSTICK	        195		/***** NOT YET SUPPORTED *****/

/*** other return opcodes ***/

/* returns string ending with '\n' */
#define	FIRMWARE_VERSION_QUERY	        196

#define LAST_0ARG_OPCODE		196

/*****/				     /*****/
/***** OPCODES with a 1 byte ARGUMENT *****/
/*****/			             /*****/

/*** returns signed char (a status byte) ***/

/* ARG1: power mode byte */
#define	PAN_SET_HOLD_POWER  	        197
#define	TILT_SET_HOLD_POWER 	        198
#define	PAN_SET_MOVE_POWER  	        199
#define	TILT_SET_MOVE_POWER	        200

#define LAST_1BYTE_ARG_OPCODE		200

/*****/				         /*****/
/***** OPCODES with a 4 byte int ARGUMENT *****/
/*****/			                 /*****/

/*** returns signed char (a status byte) ***/

/* a 4 byte unsigned integer argument */
#define	PAN_SET_ACCEL	 		201
#define	TILT_SET_ACCEL	 		202

#define LAST_4BYTE_ARG_OPCODE		202


/*****/				            /*****/
/***** SPECIAL OPCODES with custom ARGUMENTS *****/
/*****/			                    /*****/

#define SPECIAL_OPCODE                  203


/*** (Added 9/16/98 at end to maintain backward compatibility.) ***/
/*** 0 byte argument, returns signed char                       ***/
#define	QUERY_SPEED_CONTROL_MODE        203
#define	SET_INDEPENDENT_CONTROL_MODE    204
#define SET_PURE_VELOCITY_CONTROL_MODE  205

