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

EAudioPlayerFSMState audio_player_state;

static uint16_t* audio_buffer;
static uint8_t buffer_sel;
static EAudioFile* audio_file_list;
static int8_t audio_file_list_size;
static uint8_t audio_file_list_index;
static FSIZE_t remaining_file_size;
static uint8_t primed = 0;

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

		audio_file_list_index = 0;
		buffer_sel = 0;
		audio_player_state = OPEN_FILE;

		primed = 1;

		// Handles initial file open and filling both halves of the double buffer
		for (int i = 0; i < 3; i++)
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
				uint8_t* buffer_pointer = (uint8_t*) (audio_buffer + (buffer_sel ? AUDIO_BUFFER_SAMPLES : 0));
				UINT bytes_to_read = (remaining_file_size >= AUDIO_BUFFER_SAMPLES) ? AUDIO_BUFFER_SAMPLES : remaining_file_size;

				// Copies next chunk of data from the file into the audio buffer
				if (f_read(&USERFile, buffer_pointer, bytes_to_read, &br) != FR_OK)
				{
					return AUDIOPLAYER_ERROR;
				}

				buffer_sel = !buffer_sel;
				remaining_file_size -= bytes_to_read;

				// If there is still data we haven't read, break out of the switch statement
				if (remaining_file_size > 0)
				{
					break;
				}

				// Fills the rest of the audio buffer with 0s if it wasn't filled all the way
				memcpy(buffer_pointer + bytes_to_read, 0, AUDIO_BUFFER_SAMPLES - bytes_to_read);

				// Closes the file
				if (f_close(&USERFile) != FR_OK)
				{
					return AUDIOPLAYER_ERROR;
				}

				audio_file_list_index++;
				// Break statement intentionally omitted so that the next file is opened immediately instead of when the next DMA IRQ happens


			case OPEN_FILE:
				char audio_file_name[MAX_FILENAME_LENGTH];

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
