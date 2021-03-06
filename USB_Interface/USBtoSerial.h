/*
             LUFA Library
     Copyright (C) Dean Camera, 2014.

  dean [at] fourwalledcubicle [dot] com
           www.lufa-lib.org
*/

/*
  Copyright 2014  Dean Camera (dean [at] fourwalledcubicle [dot] com)

  Permission to use, copy, modify, distribute, and sell this
  software and its documentation for any purpose is hereby granted
  without fee, provided that the above copyright notice appear in
  all copies and that both that the copyright notice and this
  permission notice and warranty disclaimer appear in supporting
  documentation, and that the name of the author not be used in
  advertising or publicity pertaining to distribution of the
  software without specific, written prior permission.

  The author disclaims all warranties with regard to this
  software, including all implied warranties of merchantability
  and fitness.  In no event shall the author be liable for any
  special, indirect or consequential damages or any damages
  whatsoever resulting from loss of use, data or profits, whether
  in an action of contract, negligence or other tortious action,
  arising out of or in connection with the use or performance of
  this software.
*/

/** \file
 *
 *  Header file for USBtoSerial.c.
 */

#ifndef _USB_SERIAL_H_
#define _USB_SERIAL_H_

	/* Includes: */
		#include <avr/io.h>
		#include <avr/wdt.h>
		#include <avr/interrupt.h>
		#include <avr/power.h>

		#include <string.h>
		#include <avr/pgmspace.h>
		#include <avr/sleep.h>
		#include <util/delay.h>
		#include <stdlib.h>
		#include <stdio.h>
		#include "sd/fat.h"
		#include "sd/fat_config.h"
		#include "sd/partition.h"
		#include "sd/sd_raw.h"
		#include "sd/sd_raw_config.h"
		#include "uart.h"

		#include "Descriptors.h"

		#include <LUFA/Drivers/Board/LEDs.h>
		#include <LUFA/Drivers/Peripheral/Serial.h>
		#include <LUFA/Drivers/Misc/RingBuffer.h>
		#include <LUFA/Drivers/USB/USB.h>
		#include <LUFA/Platform/Platform.h>

		//#include <Lufa/Common/Common.h>				// Contains some useful basic function
		//#include <Lufa/Drivers/Peripheral/SPI.h>

	/* Macros: */
		/** LED mask for the library LED driver, to indicate that the USB interface is not ready. */
		#define LEDMASK_USB_NOTREADY      (LEDS_LED1 | LEDS_LED2)

		/** LED mask for the library LED driver, to indicate that the USB interface is enumerating. */
		#define LEDMASK_USB_ENUMERATING  (LEDS_LED1 | LEDS_LED2)

		/** LED mask for the library LED driver, to indicate that the USB interface is ready. */
		#define LEDMASK_USB_READY        (LEDS_LED1)

		/** LED mask for the library LED driver, to indicate that an error has occurred in the USB interface. */
		#define LEDMASK_USB_ERROR        (LEDS_LED2)

	/* Function Prototypes: */
		void SetupHardware(void);

		void EVENT_USB_Device_Connect(void);
		void EVENT_USB_Device_Disconnect(void);
		void EVENT_USB_Device_ConfigurationChanged(void);
		void EVENT_USB_Device_ControlRequest(void);

		void EVENT_CDC_Device_LineEncodingChanged(USB_ClassInfo_CDC_Device_t* const CDCInterfaceInfo);
		
		static uint8_t read_line(char* buffer, uint8_t buffer_length);
		static uint32_t strtolong(const char* str);
		static uint8_t find_file_in_dir(struct fat_fs_struct* fs, struct fat_dir_struct* dd, const char* name, struct fat_dir_entry_struct* dir_entry);
		static struct fat_file_struct* open_file_in_dir(struct fat_fs_struct* fs, struct fat_dir_struct* dd, const char* name);
// 		static uint8_t print_disk_info(const struct fat_fs_struct* fs);

		char wait_for_answer();
		char make_file(struct fat_fs_struct* fs, struct fat_dir_struct* dd,char* command);
		char exec_cmd(struct fat_fs_struct* fs, struct fat_dir_struct* dd,char* command);
		void cmd_cd(struct fat_fs_struct* fs, struct fat_dir_struct* dd,char* command);
		void cmd_ls(struct fat_fs_struct* fs, struct fat_dir_struct* dd,char* command);
		void cmd_cat(struct fat_fs_struct* fs, struct fat_dir_struct* dd,char* command);
		void cmd_rm(struct fat_fs_struct* fs, struct fat_dir_struct* dd,char* command);
		void cmd_touch(struct fat_fs_struct* fs, struct fat_dir_struct* dd,char* command);
		void cmd_write(struct fat_fs_struct* fs, struct fat_dir_struct* dd,char* command);
		void cmd_mkdir(struct fat_fs_struct* fs, struct fat_dir_struct* dd,char* command);
		void cmd_test(struct fat_fs_struct* fs, struct fat_dir_struct* dd,char* command);
		//void cmd_cd(struct fat_fs_struct* fs, struct fat_dir_struct* dd,char* command);
		//void cmd_cd(struct fat_fs_struct* fs, struct fat_dir_struct* dd,char* command);

#endif

