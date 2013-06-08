/*************************************************************************
*****                  PTU BINARY DRIVER CODE FILE                   ****/
#define PTU_CPI_CODE_VERSION   "v1.09.07"
/****                                                                *****
*****               (C)1998, Directed Perception, Inc.               *****
*****                     All Rights Reserved.                       *****
*****                                                                *****
*****   Licensed users may freely distribute compiled code including *****
*****   this code and data. Source data and code may NOT be          *****
*****   distributed without the prior written consent from           *****
*****   Directed Perception, Inc.                                    *****
*****	        Directed Perception, Inc. reserves the right to make *****
*****   changes without further notice to any content herein to      *****
*****   improve reliability, function or design. Directed Perception *****
*****   shall not assume any liability arising from the application  *****
*****   or use of this code, data or function.                       *****
*****                                                                *****
**************************************************************************

CHANGE HISTORY:
    9/23/98: v1.09.07.   Added additional commands supported in firmware version v1.9.7
		         and above. Unit commands modified to work (unsigned short int).
    8/10/98: v1.08.09.   In firmware_version_OK, removed addressing to string constant.
    9/27/97: v1.08.08.   Win32. Removed writestring in openserial.
                         Set 0 read timeout in setserial. Peek improved.
    7/10/97: v1.08.00.   Firmware version check modified so illegal version
	                 numbers (e.g., from bad read operations) are detected
    11/2/95:  v1.07.07d. Firmware version check bug fixed.
    7/11/95:  v1.07.06d. Updated opcode structure and added new support.
    2/19/95:  v1.07.04d. Generalized for Windows, DOS, & UNIX.
  	                 Added networking.
    10/12/94: v1.07.03d. Pre-release working version.
				   	     XON/XOFF removed from PTU firmware to allow for binary mode.


**************************************************************************/

#define PTU_OPCODE_VERSION					"v1.07.07d"
/* this code supports controller firmware versions equal or above */
#define PTU_modelVersion							1
#define PTU_codeVersion						     	7
#define PTU_revision 								6

/* The binary PTU driver code */
/* Assumes serial support for read_byte and write_byte */


//#include <windows.h>


// #include <dos.h>
//#include <conio.h>
#include <stdio.h>
#include <ctype.h>
//#include <string.h>
//#include <stdlib.h>
//#include <time.h>
// #include <dir.h>

#include "ptu.h"


static char err;

static portstream_fd current_host_port;

static char speed_control_mode = PTU_INDEPENDENT_SPEED_CONTROL_MODE;


/* open_host_port(<portname>) ==> <portstream> */
portstream_fd open_host_port(char *portname)
{  current_host_port = openserial(portname);
   return current_host_port;
}


/* close_host_port(<portstream>) ==> <status> */
char close_host_port(portstream_fd portstream)
	{   current_host_port = PORT_NOT_OPENED;
		return( closeserial(portstream) );
	}


unsigned char GetSerialChar(char await_char)
	{  unsigned char c;

		for (;;) {
			err = SerialBytesIn(current_host_port, &c, 1, -1);
			if (err < 0) return err;
			else if (err > 0) return c;
				  else if (await_char != TRUE)
							 return 0;
			}
	}

char SerialOut(unsigned char outchar)
{    return SerialBytesOut(current_host_port, &outchar, 1);
}


/* use this function to switch the host port currently being controlled */
char select_host_port(portstream_fd portstream)
	{  current_host_port = portstream;
		return(0);
	}



/* reset_PTU() ==> <status> */
char reset_ptu(void)
	{  unsigned char c;

		SerialOut(UNIT_RESET);
		while ( ((c = GetSerialChar(TRUE)) == PAN_LIMIT_HIT) ||
				  (c == TILT_LIMIT_HIT) );
		return(c);
	}



/* an internal function that verifies that the PTU controller firmware
	supports this binary interface. TRUE when OK, otherwise error code. */
char firmware_version_OK(void)
{   unsigned char *tmpStr;
    char c1, c2;
	int modelVersion, codeVersion, revision;
	unsigned char versionSTR[256];
	int charsRead=0;
	unsigned char initString[] = "    v ";

	FlushInputBuffer(current_host_port);
	tmpStr = initString;
	if ( SerialBytesOut(current_host_port, tmpStr, 6) != TRUE )
	   { printf ("\nERROR(firmware_version_OK): SerialBytesOut error\n");
	     return(FALSE);
	}
	do_delay(1000);

	switch ( ReadSerialLine(current_host_port, versionSTR, -1, &charsRead) ) {
		case TRUE:
				break;
		case TIMEOUT_CHAR_READ:
				printf("\nERROR(firmware_version_OK): ReadSerialLine TIMEOUT_CHAR_READ (%d read)\n", charsRead);
				return(TIMEOUT_CHAR_READ);
		default:
				printf("\nERROR(firmware_version_OK): ReadSerialLine error\n");
				return(FALSE);
	}
	/* parse to the beginning of the version ID */
	tmpStr = versionSTR;
	while ( tolower(*tmpStr) != 'v' ) tmpStr++;
	if ( *(tmpStr + 2) == '*' )
		{ tmpStr += 2;
		  while ( tolower(*tmpStr) != 'v' ) tmpStr++;
		  tmpStr++;
		  }
	sscanf((char *) tmpStr, "%1d %c %2d %c %2d",
						&modelVersion, &c1, &codeVersion, &c2, &revision);

	if ( /* ensure the version numbers seem reasonable as version numbers, and
		    the version number is high enough */
		 ((modelVersion < 999)  && (modelVersion >= PTU_modelVersion)) ||
		 ((codeVersion  < 999)  && (codeVersion  >= PTU_codeVersion))  ||
		 ((revision     < 9999) && (revision     >= PTU_revision)) )
		  { printf("\n\nController firmware v%d.%d.%d is compatible\n\n",
		  			  modelVersion, codeVersion, revision);
			 return(TRUE); /* the controller firmware is compatible */
		  }
	else { printf("\n\nPTU Controller firmware version v%d.%d.%d is NOT compatible:\n\tversion v%d.%d.%d and higher is required\n\n",
					  modelVersion, codeVersion, revision,
					  PTU_modelVersion, PTU_codeVersion, PTU_revision);
			 return(FALSE);
		  }

}



/* Flushes the PTU parser so that pending command parses are terminated,
	and the call blocks until the PTU is ready to accept the next command
	or it times out before the PTU responds that it is OK. */
/* reset_PTU_parser(<timeout_in_msec>) ==> [PTU_OK|PTU_NOT_RESPONDING] */
char reset_PTU_parser(long timeout_in_msec)
	{ long elapsed_time = 250;
	  char status;

	  FlushInputBuffer(current_host_port);      /* flushes the PTU return info */
	  SerialOut(' '); SerialOut(' '); SerialOut(' ');  /* terminates any pending parses */
	  do_delay(250);    /* allows the return of any PTU info from the termination */

	  if ( firmware_version_OK() != TRUE )
	     return(PTU_FIRMWARE_VERSION_TOO_LOW);

	  /* now make sure the PTU is responding to commands */
	  for (;;)
		 { /* issue a command and ensure it looks legal */
			SerialOut(PAN_HOLD_POWER_QUERY);
			/* do_delay(250);  */
			status = GetSerialChar(FALSE);
			if ( (status >= PTU_REG_POWER) & (status <= PTU_OFF_POWER) )
				{ /* things look OK, so flush and unblock */
				  FlushInputBuffer(current_host_port); /* flushes the PTU return info */
				  return(PTU_OK);
				}
			else
				{ /* there's a problem, so flush, delay, and retry */
				  FlushInputBuffer(current_host_port); /* flushes the PTU return info */
				  do_delay(500);
				  elapsed_time += 750;
				  if (elapsed_time > timeout_in_msec)
					  return(PTU_NOT_RESPONDING);
				}}
	}


/* set_desired([PAN|TILT],
					[POSITION|SPEED|ACCELERATION|BASE|UPPER|LOWER],
					[<position>|<speed>|<acceleration>],
					[RELATIVE|ABSOLUTE]) ==> <status>
	set_desired([PAN|TILT],
					[HOLD_POWER_LEVEL,MOVE_POWER_LEVEL],
					<power mode>,
					NULL) ==> <status>                              */
char set_desired(char axis, char kinematic_property,
 	             PTU_PARM_PTR *value, char movement_mode)
	{ unsigned short int uvalue;
	  long lvalue;
	  char cvalue;

	  switch (kinematic_property)	{
			case POSITION:
					switch (axis)  {
						case PAN:
							 switch (movement_mode) {
								case RELATIVE: SerialOut(PAN_SET_REL_POS);
													break;
								case ABSOLUTE: SerialOut(PAN_SET_ABS_POS);
													break;
								default: return(PTU_ILLEGAL_COMMAND_ARGUMENT);
							 }
							 break;
						case TILT:
							 switch (movement_mode) {
								case RELATIVE: SerialOut(TILT_SET_REL_POS);
													break;
								case ABSOLUTE: SerialOut(TILT_SET_ABS_POS);
													break;
								default: return(PTU_ILLEGAL_COMMAND_ARGUMENT);
							 }
							 break;
						default: return(PTU_ILLEGAL_COMMAND_ARGUMENT);		}
					PutSignedShort(current_host_port, (signed short *) value);
					break;
			case SPEED:
					switch (axis)  {
						case PAN:
							 switch (movement_mode) {
								case RELATIVE: SerialOut(PAN_SET_REL_SPEED);
											   PutSignedShort(current_host_port, (signed short *) value);
											   break;
								case ABSOLUTE: SerialOut(PAN_SET_ABS_SPEED);
											   if (speed_control_mode == PTU_INDEPENDENT_SPEED_CONTROL_MODE)
												  { uvalue = *((unsigned short *) value);
												    PutUnsignedShort(current_host_port, &uvalue);		}
											   else
												  { PutSignedShort(current_host_port, (signed short *) value);		}
											   break;
								default: return(PTU_ILLEGAL_COMMAND_ARGUMENT);
							 }
							 break;
						case TILT:
							 switch (movement_mode) {
								case RELATIVE: SerialOut(TILT_SET_REL_SPEED);
											   PutSignedShort(current_host_port, (signed short *) value);
											   break;
								case ABSOLUTE: SerialOut(TILT_SET_ABS_SPEED);
											   if (speed_control_mode == PTU_INDEPENDENT_SPEED_CONTROL_MODE)
												  { uvalue = *((unsigned short *) value);
													PutUnsignedShort(current_host_port, &uvalue);		}
											   else
												  { PutSignedShort(current_host_port, (signed short *) value);		}
											   break;
								default: return(PTU_ILLEGAL_COMMAND_ARGUMENT);
							 }
							 break;
						default: return(PTU_ILLEGAL_COMMAND_ARGUMENT);		}
						break;
			case ACCELERATION:
					lvalue = *((long *)value);
					switch (axis)  {
						case PAN:
							 switch (movement_mode) {
								case RELATIVE: lvalue += get_current(PAN,ACCELERATION);
													SerialOut(PAN_SET_ACCEL);
													break;
								case ABSOLUTE: SerialOut(PAN_SET_ACCEL);
													break;
								default: return(PTU_ILLEGAL_COMMAND_ARGUMENT);
							 }
							 break;
						case TILT:
							 switch (movement_mode) {
								case RELATIVE: lvalue += get_current(TILT,ACCELERATION);
													SerialOut(TILT_SET_ACCEL);
													break;
								case ABSOLUTE: SerialOut(TILT_SET_ACCEL);
													break;
								default: return(PTU_ILLEGAL_COMMAND_ARGUMENT);
							 }
							 break;
						default: return(PTU_ILLEGAL_COMMAND_ARGUMENT);		}
					PutSignedLong(current_host_port, &lvalue);
				  break;
			case BASE:
					switch (axis)  {
						case PAN:
							 switch (movement_mode) {
								case RELATIVE:
								case ABSOLUTE: SerialOut(PAN_SET_BASE_SPEED);
													break;
								default: return(PTU_ILLEGAL_COMMAND_ARGUMENT);
							 }
							 break;
						case TILT:
							 switch (movement_mode) {
								case RELATIVE:
								case ABSOLUTE: SerialOut(TILT_SET_BASE_SPEED);
													break;
								default: return(PTU_ILLEGAL_COMMAND_ARGUMENT);
							 }
							 break;
						default: return(PTU_ILLEGAL_COMMAND_ARGUMENT);		}
					uvalue = *((unsigned short int*) value);
					PutUnsignedShort(current_host_port, &uvalue);
				  break;
			case UPPER_SPEED_LIMIT:
					switch (axis)  {
						case PAN:
							 switch (movement_mode) {
								case RELATIVE:
								case ABSOLUTE: SerialOut(PAN_SET_UPPER_SPEED_LIMIT);
											   break;
								default: return(PTU_ILLEGAL_COMMAND_ARGUMENT);
							 }
							 break;
						case TILT:
							 switch (movement_mode) {
								case RELATIVE:
								case ABSOLUTE: SerialOut(TILT_SET_UPPER_SPEED_LIMIT);
													break;
								default: return(PTU_ILLEGAL_COMMAND_ARGUMENT);
							 }
							 break;
						default: return(PTU_ILLEGAL_COMMAND_ARGUMENT);		}
					uvalue = *((unsigned short int*) value);
					PutUnsignedShort(current_host_port, &uvalue);
				  break;
			case LOWER_SPEED_LIMIT:
					switch (axis)  {
						case PAN:
							 switch (movement_mode) {
								case RELATIVE:
								case ABSOLUTE: SerialOut(PAN_SET_LOWER_SPEED_LIMIT);
													break;
								default: return(PTU_ILLEGAL_COMMAND_ARGUMENT);
							 }
							 break;
						case TILT:
							 switch (movement_mode) {
								case RELATIVE:
								case ABSOLUTE: SerialOut(TILT_SET_LOWER_SPEED_LIMIT);
													break;
								default: return(PTU_ILLEGAL_COMMAND_ARGUMENT);
							 }
							 break;
						default: return(PTU_ILLEGAL_COMMAND_ARGUMENT);		}
					uvalue = *((unsigned short int*) value);
					PutUnsignedShort(current_host_port, &uvalue);
				  break;
			case HOLD_POWER_LEVEL:
					switch (axis)  {
						case PAN:
							 switch (movement_mode) {
								case RELATIVE:
								case ABSOLUTE: SerialOut(PAN_SET_HOLD_POWER);
													break;
								default: return(PTU_ILLEGAL_COMMAND_ARGUMENT);
							 }
							 break;
						case TILT:
							 switch (movement_mode) {
								case RELATIVE:
								case ABSOLUTE: SerialOut(TILT_SET_HOLD_POWER);
													break;
								default: return(PTU_ILLEGAL_COMMAND_ARGUMENT);
							 }
							 break;
						default: return(PTU_ILLEGAL_COMMAND_ARGUMENT);		}
					cvalue = *((unsigned char*) value);
					SerialOut((unsigned char) cvalue);
				  break;
			case MOVE_POWER_LEVEL:
					switch (axis)  {
						case PAN:
							 switch (movement_mode) {
								case RELATIVE:
								case ABSOLUTE: SerialOut(PAN_SET_MOVE_POWER);
													break;
								default: return(PTU_ILLEGAL_COMMAND_ARGUMENT);
							 }
							 break;
						case TILT:
							 switch (movement_mode) {
								case RELATIVE:
								case ABSOLUTE: SerialOut(TILT_SET_MOVE_POWER);
													break;
								default: return(PTU_ILLEGAL_COMMAND_ARGUMENT);
							 }
							 break;
						default: return(PTU_ILLEGAL_COMMAND_ARGUMENT);		}
					cvalue = *((unsigned char*) value);
					SerialOut((unsigned char) cvalue);
				  break;
			default: return(PTU_ILLEGAL_COMMAND_ARGUMENT);
			}


	return(GetSerialChar(TRUE));    /* return the command execution status */

	}


/* await_completion() ==> <status> */
char await_completion(void)
	{ SerialOut(AWAIT_COMMAND_COMPLETION);
	  return( GetSerialChar(TRUE) );
	  }


/* get_current([PAN|TILT],
					[POSITION|SPEED|ACCELERATION|BASE|
					 UPPER_SPEED_LIMIT|LOWER_SPEED_LIMIT|
					 MINIMUM_POSITION|MAXIMUM_POSITION|
					 RESOLUTION|
					 HOLD_POWER_LEVEL|MOVE_POWER_LEVEL|]) ==> <value> */
long get_current(char axis, char kinematic_property)
	{ unsigned short int uvalue;
	  signed short int value;
     long long_value;

	  switch (kinematic_property)	{
			case POSITION:
					switch (axis)  {
						case PAN:	SerialOut(PAN_CURRENT_POS_QUERY);
									goto get_and_return_signed_short_int;
						case TILT:  SerialOut(TILT_CURRENT_POS_QUERY);
									goto get_and_return_signed_short_int;
						default: 	return(PTU_ILLEGAL_COMMAND_ARGUMENT);
						}
			case SPEED:
					switch (axis)  {
						case PAN:   SerialOut(PAN_CURRENT_SPEED_QUERY);
									if (speed_control_mode == PTU_INDEPENDENT_SPEED_CONTROL_MODE)
									   goto get_and_return_unsigned_short_int;
									else
									   goto get_and_return_signed_short_int;
						case TILT:  SerialOut(TILT_CURRENT_SPEED_QUERY);
									if (speed_control_mode == PTU_INDEPENDENT_SPEED_CONTROL_MODE)
									   goto get_and_return_unsigned_short_int;
									else
									   goto get_and_return_signed_short_int;
						default: 	return(PTU_ILLEGAL_COMMAND_ARGUMENT);
						}
			case ACCELERATION:
					switch (axis)  {
						case PAN:	SerialOut(PAN_ACCEL_QUERY);
									goto get_and_return_long;
						case TILT:  SerialOut(TILT_ACCEL_QUERY);
									goto get_and_return_long;
						default: 	return(PTU_ILLEGAL_COMMAND_ARGUMENT);
						}
			case BASE:
					switch (axis)  {
						case PAN: 	SerialOut(PAN_BASE_SPEED_QUERY);
									goto get_and_return_unsigned_short_int;
						case TILT:  SerialOut(TILT_BASE_SPEED_QUERY);
									goto get_and_return_unsigned_short_int;
						default: 	return(PTU_ILLEGAL_COMMAND_ARGUMENT);
						}
			case UPPER_SPEED_LIMIT:
					switch (axis)  {
						case PAN: 	SerialOut(PAN_UPPER_SPEED_LIMIT_QUERY);
									goto get_and_return_unsigned_short_int;
						case TILT: 	SerialOut(TILT_UPPER_SPEED_LIMIT_QUERY);
									goto get_and_return_unsigned_short_int;
						default: 	return(PTU_ILLEGAL_COMMAND_ARGUMENT);
						}
			case LOWER_SPEED_LIMIT:
					switch (axis)  {
						case PAN:	SerialOut(PAN_LOWER_SPEED_LIMIT_QUERY);
									goto get_and_return_unsigned_short_int;
						case TILT:  SerialOut(TILT_LOWER_SPEED_LIMIT_QUERY);
									goto get_and_return_unsigned_short_int;
						default:    return(PTU_ILLEGAL_COMMAND_ARGUMENT);
						}
			case MINIMUM_POSITION:
					switch (axis)  {
						case PAN:	SerialOut(PAN_MIN_POSITION_QUERY);
									goto get_and_return_signed_short_int;
						case TILT:  SerialOut(TILT_MIN_POSITION_QUERY);
									goto get_and_return_signed_short_int;
						default: 	return(PTU_ILLEGAL_COMMAND_ARGUMENT);
						}
			case MAXIMUM_POSITION:
					switch (axis)  {
						case PAN:	SerialOut(PAN_MAX_POSITION_QUERY);
									goto get_and_return_signed_short_int;
						case TILT:  SerialOut(TILT_MAX_POSITION_QUERY);
									goto get_and_return_signed_short_int;
						default: 	return(PTU_ILLEGAL_COMMAND_ARGUMENT);
						}
			case RESOLUTION:
					switch (axis)  {
						case PAN:	SerialOut(PAN_RESOLUTION_QUERY);
									goto get_and_return_long;
						case TILT:  SerialOut(TILT_RESOLUTION_QUERY);
									goto get_and_return_long;
						default: 	return(PTU_ILLEGAL_COMMAND_ARGUMENT);
						}
			case HOLD_POWER_LEVEL:
					switch (axis)  {
						case PAN:	SerialOut(PAN_HOLD_POWER_QUERY);
									goto get_and_return_char;
						case TILT:  SerialOut(TILT_HOLD_POWER_QUERY);
									goto get_and_return_char;
						default: 	return(PTU_ILLEGAL_COMMAND_ARGUMENT);
						}
			case MOVE_POWER_LEVEL:
					switch (axis)  {
						case PAN:	SerialOut(PAN_MOVE_POWER_QUERY);
									goto get_and_return_char;
						case TILT:  SerialOut(TILT_MOVE_POWER_QUERY);
									goto get_and_return_char;
						default: 	return(PTU_ILLEGAL_COMMAND_ARGUMENT);
						}
			default: return(PTU_ILLEGAL_COMMAND_ARGUMENT);
		}

	get_and_return_unsigned_short_int:
		err = GetUnsignedShort(current_host_port, &uvalue, -1);
		long_value = uvalue;
		return(long_value);

	get_and_return_signed_short_int:
		err = GetSignedShort(current_host_port, &value, -1);
		long_value = value;
		return(long_value);

	get_and_return_long:
		err = GetSignedLong(current_host_port, &long_value, -1);
		return(long_value);

	get_and_return_char:
		long_value = (long) GetSerialChar(TRUE);
		return(long_value);

}



/* get_desired([PAN|TILT],
					[POSITION|SPEED|ACCELERATION|BASE|
					 UPPER_SPEED_LIMIT|LOWER_SPEED_LIMIT|
					 MINIMUM_POSITION|MAXIMUM_POSITION|
					 RESOLUTION|
					 HOLD_POWER_LEVEL|MOVE_POWER_LEVEL]) ==> <ptr to value> */
long get_desired(char axis, char kinematic_property)
	{ unsigned short int uvalue;
	  signed short int value;
      long long_value;

	  switch (kinematic_property)	{
			case POSITION:
					switch (axis)  {
						case PAN:   SerialOut(PAN_DESIRED_POS_QUERY);
									goto get_and_return_signed_short_int;
						case TILT:  SerialOut(TILT_DESIRED_POS_QUERY);
					 				goto get_and_return_signed_short_int;
						default: 	return(PTU_ILLEGAL_COMMAND_ARGUMENT);
						}
			case SPEED:
					switch (axis)  {
						case PAN:   SerialOut(PAN_DESIRED_SPEED_QUERY);
							//		if (speed_control_mode == PTU_INDEPENDENT_SPEED_CONTROL_MODE)
									   goto get_and_return_unsigned_short_int;
							//		else
							//		   goto get_and_return_signed_short_int;
						case TILT:  SerialOut(TILT_DESIRED_SPEED_QUERY);
									if (speed_control_mode == PTU_INDEPENDENT_SPEED_CONTROL_MODE)
									   goto get_and_return_unsigned_short_int;
									else
									   goto get_and_return_signed_short_int;
						default:    return(PTU_ILLEGAL_COMMAND_ARGUMENT);
						}
			case ACCELERATION:
					switch (axis)  {
						case PAN:   SerialOut(PAN_ACCEL_QUERY);
									goto get_and_return_long;
						case TILT:  SerialOut(TILT_ACCEL_QUERY);
						  			goto get_and_return_long;
						default:    return(PTU_ILLEGAL_COMMAND_ARGUMENT);
						}
			case BASE:
					switch (axis)  {
						case PAN: 	SerialOut(PAN_BASE_SPEED_QUERY);
									goto get_and_return_unsigned_short_int;
						case TILT:  SerialOut(TILT_BASE_SPEED_QUERY);
									goto get_and_return_unsigned_short_int;
						default: 	return(PTU_ILLEGAL_COMMAND_ARGUMENT);
						}
			case UPPER_SPEED_LIMIT:
					switch (axis)  {
						case PAN: 	SerialOut(PAN_UPPER_SPEED_LIMIT_QUERY);
									goto get_and_return_unsigned_short_int;
						case TILT: 	SerialOut(TILT_UPPER_SPEED_LIMIT_QUERY);
									goto get_and_return_unsigned_short_int;
						default: 	return(PTU_ILLEGAL_COMMAND_ARGUMENT);
						}
			case LOWER_SPEED_LIMIT:
					switch (axis)  {
						case PAN:	SerialOut(PAN_LOWER_SPEED_LIMIT_QUERY);
									goto get_and_return_unsigned_short_int;
						case TILT:  SerialOut(TILT_LOWER_SPEED_LIMIT_QUERY);
									goto get_and_return_unsigned_short_int;
						default: 	return(PTU_ILLEGAL_COMMAND_ARGUMENT);
						}
			case MINIMUM_POSITION:
					switch (axis)  {
						case PAN:	SerialOut(PAN_MIN_POSITION_QUERY);
									goto get_and_return_signed_short_int;
						case TILT:  SerialOut(TILT_MIN_POSITION_QUERY);
									goto get_and_return_signed_short_int;
						default: break;
						}
			case MAXIMUM_POSITION:
					switch (axis)  {
						case PAN:	SerialOut(PAN_MAX_POSITION_QUERY);
									goto get_and_return_signed_short_int;
						case TILT:  SerialOut(TILT_MAX_POSITION_QUERY);
									goto get_and_return_signed_short_int;
						default: 	return(PTU_ILLEGAL_COMMAND_ARGUMENT);
						}
			case RESOLUTION:
					switch (axis)  {
						case PAN:   SerialOut(PAN_RESOLUTION_QUERY);
									goto get_and_return_long;
						case TILT:  SerialOut(TILT_RESOLUTION_QUERY);
									goto get_and_return_long;
						default:    return(PTU_ILLEGAL_COMMAND_ARGUMENT);
						}
			case HOLD_POWER_LEVEL:
					switch (axis)  {
						case PAN:	SerialOut(PAN_HOLD_POWER_QUERY);
									goto get_and_return_char;
						case TILT:  SerialOut(TILT_HOLD_POWER_QUERY);
									goto get_and_return_char;
						default: 	return(PTU_ILLEGAL_COMMAND_ARGUMENT);
						}
			case MOVE_POWER_LEVEL:
					switch (axis)  {
						case PAN:	SerialOut(PAN_MOVE_POWER_QUERY);
									goto get_and_return_char;
						case TILT:  SerialOut(TILT_MOVE_POWER_QUERY);
									goto get_and_return_char;
						default: 	return(PTU_ILLEGAL_COMMAND_ARGUMENT);
						}
			default: return(PTU_ILLEGAL_COMMAND_ARGUMENT);

		}

	get_and_return_unsigned_short_int:
		err = GetUnsignedShort(current_host_port, &uvalue,-1);
		long_value = uvalue;
		return(long_value);

	get_and_return_signed_short_int:
		err = GetSignedShort(current_host_port, &value,-1);
		long_value = value;
		return(long_value);

	get_and_return_long:
		err = GetSignedLong(current_host_port, &long_value,-1);
		return(long_value);

	get_and_return_char:
		long_value = (long) GetSerialChar(TRUE);
		return(long_value);

}


/* set_mode(COMMAND_EXECUTION_MODE,
				[EXECUTE_IMMEDIATELY|EXECUTE_UPON_IMMEDIATE_OR_AWAIT]) ==> <status>
   set_mode(ASCII_VERBOSE_MODE, [VERBOSE|TERSE|QUERY_MODE]) ==> <status>
   set_mode(ASCII_ECHO_MODE, [ON_MODE|OFF_MODE|QUERY_MODE] ==> <status>
   set_mode(POSITION_LIMITS_MODE, [ON_MODE|OFF_MODE|QUERY_MODE] ==> <status>
   set_mode(DEFAULTS,[SAVE_CURRENT_SETTINGS|RESTORE_SAVED_SETTINGS|RESTORE_FACTORY_SETTINGS]) ==> <status>
   *** below is only supported by PTU firmware versions 1.9.7 and higher.                        ***
   *** This call must be made before pure velocity speed control mode made be used by CPI calls. ***
   set_mode(SPEED_CONTROL_MODE, [PTU_INDEPENDENT_SPEED_CONTROL_MODE |
								 PTU_PURE_VELOCITY_SPEED_CONTROL_MODE | QUERY_MODE] ==> <status>
*/
char set_mode(char mode_type, char mode_parameter)
	{ switch (mode_type) {
		case DEFAULTS:
				{switch (mode_parameter) {
					case SAVE_CURRENT_SETTINGS:
								SerialOut(SAVE_DEFAULTS);
								goto return_status;
					case RESTORE_SAVED_SETTINGS:
								SerialOut(RESTORE_SAVED_DEFAULTS);
								goto return_status;
					case RESTORE_FACTORY_SETTINGS:
								SerialOut(RESTORE_FACTORY_DEFAULTS);
								goto return_status;
					default: return(PTU_ILLEGAL_COMMAND_ARGUMENT);
					}}
		case ASCII_ECHO_MODE:
				{switch (mode_parameter) {
					case ON_MODE:
								SerialOut(ENABLE_ECHO);
								goto return_status;
					case OFF_MODE:
								SerialOut(DISABLE_ECHO);
								goto return_status;
					case QUERY_MODE:
								SerialOut(ECHO_QUERY);
								goto return_status;
					default: return(PTU_ILLEGAL_COMMAND_ARGUMENT);
					}}
		case COMMAND_EXECUTION_MODE:
				switch (mode_parameter) {
					case EXECUTE_IMMEDIATELY:
								SerialOut(SET_IMMEDIATE_COMMAND_MODE);
								break;
					case EXECUTE_UPON_IMMEDIATE_OR_AWAIT:
								SerialOut(SET_SLAVED_COMMAND_MODE);
								break;
					default: return(PTU_ILLEGAL_COMMAND_ARGUMENT);
					}
				break;
		case ASCII_VERBOSE_MODE:
				{switch (mode_parameter) {
					case VERBOSE:
								SerialOut(SET_VERBOSE_ASCII_ON);
								break;
					case TERSE:
								SerialOut(SET_VERBOSE_ASCII_OFF);
								break;
					case QUERY_MODE:
								SerialOut(VERBOSE_QUERY);
								break;
					default: return(PTU_ILLEGAL_COMMAND_ARGUMENT);
					}}
				break;
		case POSITION_LIMITS_MODE:
				{switch (mode_parameter) {
					case ON_MODE:
								SerialOut(ENABLE_POSITION_LIMITS);
								break;
					case OFF_MODE:
								SerialOut(DISABLE_POSITION_LIMITS);
								break;
					case QUERY_MODE:
								SerialOut(POSITION_LIMITS_QUERY);
								break;
					default: return(PTU_ILLEGAL_COMMAND_ARGUMENT);
					}}
				break;
		case SPEED_CONTROL_MODE:
				{switch (mode_parameter) {
					case PTU_INDEPENDENT_SPEED_CONTROL_MODE:
								speed_control_mode = PTU_INDEPENDENT_SPEED_CONTROL_MODE;
						        SerialOut(SET_INDEPENDENT_CONTROL_MODE);
								break;
					case PTU_PURE_VELOCITY_SPEED_CONTROL_MODE:
								speed_control_mode = PTU_PURE_VELOCITY_SPEED_CONTROL_MODE;
						        SerialOut(SET_PURE_VELOCITY_CONTROL_MODE);
								break;
					case QUERY_MODE:
								SerialOut(QUERY_SPEED_CONTROL_MODE);
								break;
					default: return(PTU_ILLEGAL_COMMAND_ARGUMENT);
					}}
				break;
		default: return(PTU_ILLEGAL_COMMAND_ARGUMENT);
	}
  return_status:
	return( GetSerialChar(TRUE) );	/* return <status> */
}


/* halt([ALL|PAN|TILT]) ==> <status>	*/
char halt(char halt_type)
	{ switch(halt_type) {
		case PAN:   SerialOut(HALT_PAN);
						break;
		case TILT:  SerialOut(HALT_TILT);
						break;
		default:    SerialOut(HALT);
						break;
		}
	  return( GetSerialChar(TRUE) );
	}


/* firmware_version() ==> <version ID string> */
char* firmware_version(void)
{  static unsigned char version_ID_string[256];
	int charsRead;

	SerialOut(FIRMWARE_VERSION_QUERY);
	do_delay(1000);
	ReadSerialLine(current_host_port, version_ID_string,0,&charsRead);
	return((char *) version_ID_string);
}



/* modified 10/19/98  */
char select_unit(portstream_fd portstream, UID_fd unit_ID)
	{   current_host_port = portstream;
		SerialOut(SELECT_UNIT_ID);
		PutUnsignedShort(portstream, &unit_ID);
		return GetSerialChar(TRUE);
	}


/*  modified 10/19/98  */
char set_unit_id(UID_fd unit_ID)
	{   SerialOut(SET_UNIT_ID);
		PutUnsignedShort(current_host_port, &unit_ID);
		return GetSerialChar(TRUE);
	}

