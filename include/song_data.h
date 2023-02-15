#pragma once
#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#define PLAYBACK_STEP 75680 /* About 0.4 of second */
_Static_assert(PLAYBACK_STEP % 11 == 0, "Playback adjustment step: must a be multiple of 11, "
    "otherwise pointer to bit packed data will end up in wrong state.");

void MoveSongBackward(void);
void MoveSongForward(void);
void ResetSongPosition(void);
void SetSongPlayback(void);
void SetupSongPlayback(const unsigned char* song_data, size_t song_data_size);

bool FetchNextSongSample(uint16_t* write_to);
