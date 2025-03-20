/*
 * audio_player.h
 *
 *  Created on: Mar 9, 2025
 *  Author: Joseph Fortino
 */

#ifndef INC_AUDIO_PLAYER_H_
#define INC_AUDIO_PLAYER_H_

#include <stdint.h>
#include "watch_config.h"
#include "audio_library.h"

typedef enum
{
	AUDIOPLAYER_OK,
	AUDIOPLAYER_ERROR,
	AUDIOPLAYER_FINISHED
} EAudioPlayerStatus;

typedef enum
{
	OPEN_FILE,
	FILL_BUFFER,
	AUDIO_DONE
} EAudioPlayerFSMState;


extern EAudioPlayerFSMState audio_player_state;

EAudioPlayerStatus PlayerFSM_Prime(EAudioFile* file_list, uint8_t file_list_size, uint16_t* buf);
EAudioPlayerStatus PlayerFSM_Run();

#endif /* INC_AUDIO_PLAYER_H_ */
