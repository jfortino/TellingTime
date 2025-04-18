/*
 * audio_player.c
 *
 *  Created on: Mar 9, 2025
 *  Author: Joseph Fortino
 */


#include "watch_config.h"
#ifdef AUDIO

#include "audio_player.h"
#include "fatfs.h"
#include <string.h>
#include <stdio.h>

EAudioPlayerFSMState audio_player_state;

static uint16_t* audio_buffer;
static uint8_t buffer_num;
static EAudioFile* audio_file_list;
static int8_t audio_file_list_size;
static uint8_t audio_file_list_index;
static FSIZE_t remaining_file_size;
static uint8_t primed = 0;


static void interleave_zeros(uint16_t* buf)
{
	for (int i = 1; i < AUDIO_BUFFER_SAMPLES; i++)
	{
		buf[AUDIO_BUFFER_SAMPLES * 2 - i*2] = buf[AUDIO_BUFFER_SAMPLES- i];
		buf[AUDIO_BUFFER_SAMPLES - i] = 0;
	}
}



void generate_number_audio(EAudioFile* file_list, uint8_t* file_list_size, unsigned int num)
{
	EAudioFile audio_file;

	// We only accept 3 digit numbers
	if (num > 999)
	{
		return;
	}

	int temp = num;
	int digits[3];	// We should only ever have to work with 3 digit numbers
	int num_digits = 0;

	// Stores the digits of num into the digits array in ascending order (1s, 10s, 100s, etc.)
	// Also counts the number of digits
	while (temp > 0)
	{
		digits[num_digits] = temp % 10;
		temp /= 10;
		num_digits++;
	}

	for (int i = num_digits; i > 0; i--)
	{
		if (digits[i-1] != 0)	// We don't verbally say 0 in a number like 504
		{
			if (i == 2)	// Special case to handle ten place and teens
			{
				if (digits[1] == 1)		// Handles 10 and the teens
				{
					audio_file = TEN_AUDIO + digits[0];
					i--;	// Prevents another loop iteration
				}
				else	// Handles 20, 30, 40, ...
				{
					audio_file = TWENTY_AUDIO + digits[1];
				}
			}
			else	// Handles hundreds and ones (and every other place if we weren't stopping at 999)
			{
				audio_file = digits[i-1];
			}

			file_list[*file_list_size++] = audio_file;
		}

		if (*file_list_size > MAX_AUDIO_LIST_SIZE)
		{
			return;
		}
	}
}



EAudioPlayerStatus PlayerFSM_Prime(EAudioFile* file_list, uint8_t file_list_size, uint16_t* buf)
{
	if (!primed)
	{
		if (f_mount(&USERFatFS, USERPath, 1) != FR_OK)
		{
			return AUDIOPLAYER_ERROR;
		}

		audio_file_list = file_list;
		audio_file_list_size = file_list_size;
		audio_buffer = buf;
		buffer_num = 0;

		audio_file_list_index = 0;
		audio_player_state = OPEN_FILE;

		primed = 1;

		// Handles initial file open and filling both halves of the double buffer
		for (int i = 0; i < NUM_AUDIO_BUFFERS+1; i++)
		{
			if (PlayerFSM_Run() != AUDIOPLAYER_OK)
			{
				primed = 0;
				return AUDIOPLAYER_ERROR;
			}
		}

		return AUDIOPLAYER_OK;
	}
	else
	{
		return AUDIOPLAYER_ERROR;
	}
}


EAudioPlayerStatus PlayerFSM_Run()
{
	if (primed)
	{
		switch (audio_player_state)
		{
			case FILL_BUFFER:
				UINT br;	// I have no clue what this does, but it is needed for the f_read() function call
				uint8_t* buffer_pointer = (uint8_t*) (&audio_buffer[buffer_num * AUDIO_BUFFER_SAMPLES*2]);
				UINT bytes_to_read = (remaining_file_size >= AUDIO_BUFFER_SAMPLES*2) ? AUDIO_BUFFER_SAMPLES*2 : remaining_file_size;

				// Copies the next chunk of data from the file into the audio buffer
				if (f_read(&USERFile, buffer_pointer, bytes_to_read, &br) != FR_OK)
				{
					return AUDIOPLAYER_ERROR;
				}

				buffer_num = (buffer_num + 1) % NUM_AUDIO_BUFFERS;
				remaining_file_size -= bytes_to_read;

				// If there is still data we haven't read, break out of the switch statement
				if (remaining_file_size > 0)
				{
					interleave_zeros((uint16_t*) buffer_pointer);
					break;
				}

				// Fills the rest of the audio buffer with 0s if it wasn't filled all the way
				memset(&buffer_pointer[bytes_to_read], 0, AUDIO_BUFFER_SAMPLES*2 - bytes_to_read);
				interleave_zeros((uint16_t*) buffer_pointer);

				// Closes the file
				if (f_close(&USERFile) != FR_OK)
				{
					return AUDIOPLAYER_ERROR;
				}

				audio_file_list_index++;
				// Break statement intentionally omitted so that the next file is opened immediately instead of when the next DMA IRQ happens


			case OPEN_FILE:
				char audio_file_name[MAX_FILENAME_LENGTH + 4];

				// If there are no more files in the list to read, we unmount the file system and go to the AUDIO_DONE state
				if (audio_file_list_index >= audio_file_list_size)
				{
					// Unmounts the file system since we are done using it
					if (f_mount(NULL, USERPath, 1) != FR_OK)
					{
						return AUDIOPLAYER_ERROR;
					}

					audio_player_state = AUDIO_DONE;
					primed = 0;
					return AUDIOPLAYER_FINISHED;
				}

				// Generates the complete file name of the file
				sprintf(audio_file_name, "%s.raw", AUDIO_FILENAMES[audio_file_list[audio_file_list_index]]);

				// Checks if the next file in the list exists
				if (f_stat(audio_file_name, NULL) != FR_OK)
				{
					return AUDIOPLAYER_ERROR;
				}

				// Opens the next file in the list
				if (f_open(&USERFile, audio_file_name, FA_READ) != FR_OK)
				{
					return AUDIOPLAYER_ERROR;
				}

				// Initializes the remaining file size to the total size of the file
				remaining_file_size = f_size(&USERFile);

				audio_player_state = FILL_BUFFER;
				break;

			default:
				break;
		}
	}
	else
	{
		return AUDIOPLAYER_ERROR;
	}

	return AUDIOPLAYER_OK;
}

#endif
