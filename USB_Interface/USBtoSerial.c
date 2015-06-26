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
 *  Main source file for the USBtoSerial project. This file contains the main tasks of
 *  the project and is responsible for the initial application hardware configuration.
 */

#include "USBtoSerial.h"

#define DEBUG 0

#define soft_reset()        \
do                          \
{                           \
	wdt_enable(WDTO_15MS);  \
	for(;;)                 \
	{                       \
	}                       \
} while(0)

void startup_seq();
/* Receive a byte from USB interface and store in Buffer */
void receive_usb_byte(RingBuffer_t* const Buffer);
/* Sends all the bytes stored in Buffer to USB endpoint */
void flush_buffer_to_usb(RingBuffer_t* const Buffer);
/* Sends one byte from USBtoUART_Buffer to UART */
void flush_byte_to_uart(RingBuffer_t* const Buffer);


const char* CRLF = "\r\n";
char buffer[20];
static RingBuffer_t Buffer_Rx;
static uint8_t      Buffer_Rx_Data[256];

/** Circular buffer to hold data from the host before it is sent to the device via the serial port. */
static RingBuffer_t Buffer_Rx_USB;
static uint8_t      Buffer_Rx_USB_Data[128];

/** Circular buffer to hold data from the serial port before it is sent to the host. */
static RingBuffer_t Buffer_Tx_USB;
static uint8_t      Buffer_Tx_USB_Data[128];

/** LUFA CDC Class driver interface configuration and state information. This structure is
 *  passed to all CDC Class driver functions, so that multiple instances of the same class
 *  within a device can be differentiated from one another.
 */
USB_ClassInfo_CDC_Device_t VirtualSerial_CDC_Interface =
	{
		.Config =
			{
				.ControlInterfaceNumber         = INTERFACE_ID_CDC_CCI,
				.DataINEndpoint                 =
					{
						.Address                = CDC_TX_EPADDR,
						.Size                   = CDC_TXRX_EPSIZE,
						.Banks                  = 1,
					},
				.DataOUTEndpoint                =
					{
						.Address                = CDC_RX_EPADDR,
						.Size                   = CDC_TXRX_EPSIZE,
						.Banks                  = 1,
					},
				.NotificationEndpoint           =
					{
						.Address                = CDC_NOTIFICATION_EPADDR,
						.Size                   = CDC_NOTIFICATION_EPSIZE,
						.Banks                  = 1,
					},
			},
	};

// #define SOCKIT

#ifdef SOCKIT
    #define configure_pin_ss() DDRB |= (1 << DDB6)

    #define select_card() PORTB &= ~(1 << PORTB6)
    #define unselect_card() PORTB |= (1 << PORTB6)
#endif
		
/** Main program entry point. This routine contains the overall program flow, including initial
 *  setup of all components and the main program loop.
 */
int main(void)
{
	SetupHardware();
	DDRB |= (1<<7) ;
	PORTB |= (1<<7) ;

	startup_seq();
// 	uart_init();
	/* we will just use ordinary idle mode */
	set_sleep_mode(SLEEP_MODE_IDLE);
	
	RingBuffer_InitBuffer(&Buffer_Rx_USB, Buffer_Rx_USB_Data, sizeof(Buffer_Rx_USB_Data));
	RingBuffer_InitBuffer(&Buffer_Tx_USB, Buffer_Tx_USB_Data, sizeof(Buffer_Tx_USB_Data));
	RingBuffer_InitBuffer(&Buffer_Rx, Buffer_Rx_Data, sizeof(Buffer_Rx_Data));
	
	LEDs_TurnOffLEDs(LEDS_ALL_LEDS);
	GlobalInterruptEnable();

	while(1)
	{
#ifdef TESTER
		if(!sd_raw_init())
			continue;

		/* open first partition */
		struct partition_struct* partition = partition_open(sd_raw_read, sd_raw_read_interval, sd_raw_write, sd_raw_write_interval, 0);

		if(!partition)
		{
			/* If the partition did not open, assume the storage device is a "superfloppy", i.e. has no MBR. */
			partition = partition_open(sd_raw_read, sd_raw_read_interval, sd_raw_write, sd_raw_write_interval, -1);
			if(!partition)
				soft_reset();
		}

		/* open file system */
		struct fat_fs_struct* fs = fat_open(partition);
		if(!fs)		
			soft_reset();		// Sometimes this causes hanging. Reset the device

		/* open root directory */
		struct fat_dir_entry_struct directory;
		fat_get_dir_entry_of_path(fs, "/", &directory);

		struct fat_dir_struct* dd = fat_open_dir(fs, &directory);
		if(!dd)
			continue;

		char success = 0;
		char filename_available = 0;
		volatile uint8_t  errors = 0;
		RingBuffer_Flush(&Buffer_Rx);

		uart_putc('t');

		if(wait_for_answer() == 's')
		{
			char filename[10];
			char filenum[4];
	
			for(char i=0; i<100; i++)
			{
				strcpy(filename, "dump");
				strcat(filename, itoa(i, filenum, 10));
				struct fat_dir_entry_struct subdir_entry;
				if(!find_file_in_dir(fs, dd, filename, &subdir_entry))
				{
					filename_available = 1;
					break;
				}
			}
			if(filename_available)
			{
				filename_available = 0;
				// Create the file
				if(!make_file(fs, dd, filename))
				continue;
		
				// Check if slave is ready for memory transfer
				uart_putc('r');
				if(wait_for_answer() == 'a')
				{
					// Open the file
					struct fat_file_struct* fd = open_file_in_dir(fs, dd, filename);
					if(fd)
					{
						uart_putc('m');
						if(wait_for_answer() == 'a')
						{
							uint8_t data_len;
							volatile int i;
							for(i=0; i<512; i++)
							{	
								data_len = read_line(buffer, sizeof(buffer)-3);
								if(!data_len)
								{
									errors++;
									continue;
								}
								strcat(buffer, CRLF);
								data_len += 2;
								/* write text to file */
								if(fat_write_file(fd, (uint8_t*) buffer, data_len) != data_len)
								{
									break;
								}
							}
							fat_close_file(fd);
							if(i == 512)
							success = 1;
						}
					}
				}
			}
		}
		if(success)
			uart_putc('s');
		_delay_ms(5000);
		fat_close(fs);
		partition_close(partition);

#endif



		receive_usb_byte(&Buffer_Rx_USB);
		
		flush_buffer_to_usb(&Buffer_Tx_USB);			
		
		/* Load the next byte from the USART transmit buffer into the USART if transmit buffer space is available */
		flush_byte_to_uart(&Buffer_Rx_USB);
		
		/* These functions should be called frequently for proper USB operation */
		CDC_Device_USBTask(&VirtualSerial_CDC_Interface);
		USB_USBTask();
	}
}

/** Configures the board hardware and chip peripherals for the demo's functionality. */
void SetupHardware(void)
{
	/* Disable watchdog if enabled by bootloader/fuses */
	MCUSR &= ~(1 << WDRF);
	wdt_disable();

	/* Disable clock division */
	clock_prescale_set(clock_div_1);

	/* Hardware Initialization */
	LEDs_Init();
// 	USB_Init();
}

void startup_seq()
{
	LEDs_SetAllLEDs(LEDS_ALL_LEDS);
	_delay_ms(500);
	LEDs_SetAllLEDs(LEDS_LED1);
	_delay_ms(500);
	LEDs_SetAllLEDs(LEDS_LED2);
	_delay_ms(500);
	LEDs_TurnOffLEDs(LEDS_ALL_LEDS);
}

/* Receive a byte from USB interface and store in Buffer */
void receive_usb_byte(RingBuffer_t* const Buffer)
{
	/* Only try to read in bytes from the CDC interface if the transmit buffer is not full */
	if (!(RingBuffer_IsFull(Buffer)))
	{
		int16_t ReceivedByte = CDC_Device_ReceiveByte(&VirtualSerial_CDC_Interface);

		/* Store received byte into the USART transmit buffer */
		if (!(ReceivedByte < 0))
			RingBuffer_Insert(Buffer, ReceivedByte);
	}
}

/* Sends all the bytes stored in Buffer to USB endpoint */
void flush_buffer_to_usb(RingBuffer_t* const Buffer)
{
	uint16_t BufferCount = RingBuffer_GetCount(Buffer);
	//bool done = false;
	if (BufferCount)
	{
		Endpoint_SelectEndpoint(VirtualSerial_CDC_Interface.Config.DataINEndpoint.Address);

		/* Check if a packet is already enqueued to the host - if so, we shouldn't try to send more data
			* until it completes as there is a chance nothing is listening and a lengthy timeout could occur */
		if (Endpoint_IsINReady())
		{
			/* Never send more than one bank size less one byte to the host at a time, so that we don't block
				* while a Zero Length Packet (ZLP) to terminate the transfer is sent if the host isn't listening */
			uint8_t BytesToSend = MIN(BufferCount, (CDC_TXRX_EPSIZE - 1));

			/* Read bytes from the USART receive buffer into the USB IN endpoint */
			while (BytesToSend--)
			{
				/* Try to send the next byte of data to the host, abort if there is an error without dequeuing */
				if (CDC_Device_SendByte(&VirtualSerial_CDC_Interface,
										RingBuffer_Peek(Buffer)) != ENDPOINT_READYWAIT_NoError)
				{
					break;
				}

				/* Dequeue the already sent byte from the buffer now we have confirmed that no transmission error occurred */
				RingBuffer_Remove(Buffer);
			}
		}
	}
	//return done;
}

/* Sends one byte from USBtoUART_Buffer to UART */
void flush_byte_to_uart(RingBuffer_t* const Buffer)
{
	/* Load the next byte from the transmit buffer into the USART if transmit buffer space is available */
	if (Serial_IsSendReady() && !(RingBuffer_IsEmpty(Buffer)))
		Serial_SendByte(RingBuffer_Remove(Buffer));
}

/** Event handler for the library USB Connection event. */
void EVENT_USB_Device_Connect(void)
{
	LEDs_SetAllLEDs(LEDMASK_USB_ENUMERATING);
}

/** Event handler for the library USB Disconnection event. */
void EVENT_USB_Device_Disconnect(void)
{
	LEDs_TurnOffLEDs(LEDMASK_USB_NOTREADY);
}

/** Event handler for the library USB Configuration Changed event. */
void EVENT_USB_Device_ConfigurationChanged(void)
{
	bool ConfigSuccess = true;

	ConfigSuccess &= CDC_Device_ConfigureEndpoints(&VirtualSerial_CDC_Interface);

	LEDs_SetAllLEDs(ConfigSuccess ? LEDMASK_USB_READY : LEDMASK_USB_ERROR);
}

/** Event handler for the library USB Control Request reception event. */
void EVENT_USB_Device_ControlRequest(void)
{
	CDC_Device_ProcessControlRequest(&VirtualSerial_CDC_Interface);
}

/** ISR to manage the reception of data from the serial port, placing received bytes into a circular buffer
 *  for later transmission to the host.
 */
ISR(USART1_RX_vect, ISR_BLOCK)
{
// 	uint8_t ReceivedByte = UDR1;
// 
// 	if ((USB_DeviceState == DEVICE_STATE_Configured) && !(RingBuffer_IsFull(&Buffer_Tx_USB)))
// 	  RingBuffer_Insert(&Buffer_Tx_USB, ReceivedByte);

	uint8_t ReceivedByte = UDR1;

	if ( !RingBuffer_IsFull(&Buffer_Rx))
	RingBuffer_Insert(&Buffer_Rx, ReceivedByte);
}

/** Event handler for the CDC Class driver Line Encoding Changed event.
 *
 *  \param[in] CDCInterfaceInfo  Pointer to the CDC class interface configuration structure being referenced
 */
void EVENT_CDC_Device_LineEncodingChanged(USB_ClassInfo_CDC_Device_t* const CDCInterfaceInfo)
{
	uint8_t ConfigMask = 0;

	switch (CDCInterfaceInfo->State.LineEncoding.ParityType)
	{
		case CDC_PARITY_Odd:
			ConfigMask = ((1 << UPM11) | (1 << UPM10));
			break;
		case CDC_PARITY_Even:
			ConfigMask = (1 << UPM11);
			break;
	}

	if (CDCInterfaceInfo->State.LineEncoding.CharFormat == CDC_LINEENCODING_TwoStopBits)
	  ConfigMask |= (1 << USBS1);

	switch (CDCInterfaceInfo->State.LineEncoding.DataBits)
	{
		case 6:
			ConfigMask |= (1 << UCSZ10);
			break;
		case 7:
			ConfigMask |= (1 << UCSZ11);
			break;
		case 8:
			ConfigMask |= ((1 << UCSZ11) | (1 << UCSZ10));
			break;
	}

	/* Keep the TX line held high (idle) while the USART is reconfigured */
	PORTD |= (1 << 3);

	/* Must turn off USART before reconfiguring it, otherwise incorrect operation may occur */
	UCSR1B = 0;
	UCSR1A = 0;
	UCSR1C = 0;

	/* Set the new baud rate before configuring the USART */
	UBRR1  = SERIAL_2X_UBBRVAL(CDCInterfaceInfo->State.LineEncoding.BaudRateBPS);

	/* Reconfigure the USART in double speed mode for a wider baud rate range at the expense of accuracy */
	UCSR1C = ConfigMask;
	UCSR1A = (1 << U2X1);
	UCSR1B = ((1 << RXCIE1) | (1 << TXEN1) | (1 << RXEN1));

	/* Release the TX line after the USART has been reconfigured */
	PORTD &= ~(1 << 3);
}

// Waits 10ms for an answer from slave
char wait_for_answer()
{
	for(char i=0; i<100; i++)
	{
		_delay_ms(100);
		if(!RingBuffer_IsEmpty(&Buffer_Rx))
		return RingBuffer_Remove(&Buffer_Rx);
	}
	return 0;
}

char make_file(struct fat_fs_struct* fs, struct fat_dir_struct* dd,char* filename)
{
	struct fat_dir_entry_struct file_entry;
	if(!fat_create_file(dd, filename, &file_entry))
	return 0;
	return 1;
}

uint8_t read_line(char* buffer, uint8_t buffer_length)
{
	memset(buffer, 0, buffer_length);

	uint8_t read_length = 0;
	while(read_length < buffer_length - 1)
	{
		uint8_t c;
		uint16_t i=0;
		// If nothing is received for a while report failure
		while(RingBuffer_IsEmpty(&Buffer_Rx))
		if(i++>1000)
		return 0;
		c = RingBuffer_Remove(&Buffer_Rx);

		//if(c == 0x08 || c == 0x7f)
		//{
		//if(read_length < 1)
		//continue;
		//--read_length;
		//buffer[read_length] = '\0';
		//continue;
		//}

		if(c == '\n')
		{
			buffer[read_length] = '\0';
			break;
		}
		else
		{
			buffer[read_length] = c;
			++read_length;
		}
	}

	return read_length;
}

// uint32_t strtolong(const char* str)
// {
// 	uint32_t l = 0;
// 	while(*str >= '0' && *str <= '9')
// 	l = l * 10 + (*str++ - '0');
// 
// 	return l;
// }

uint8_t find_file_in_dir(struct fat_fs_struct* fs, struct fat_dir_struct* dd, const char* name, struct fat_dir_entry_struct* dir_entry)
{
	while(fat_read_dir(dd, dir_entry))
	{
		if(strcmp(dir_entry->long_name, name) == 0)
		{
			fat_reset_dir(dd);
			return 1;
		}
	}

	return 0;
}

struct fat_file_struct* open_file_in_dir(struct fat_fs_struct* fs, struct fat_dir_struct* dd, const char* name)
{
	struct fat_dir_entry_struct file_entry;
	if(!find_file_in_dir(fs, dd, name, &file_entry))
	return 0;

	return fat_open_file(fs, &file_entry);
}
