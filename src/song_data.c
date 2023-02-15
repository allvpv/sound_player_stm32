#include "song_data.h"
#include "configuration.h"
#include <stm32.h>
#include <gpio.h>

static const unsigned char* gSongData = NULL;
static uint32_t gSongDataSize = 0;
static uint32_t gSongIndexByte = 0;
static uint32_t gSongIndexBitFromMsb = 0;

/*
 * Setting up song
 */
void MoveSongBackward(void) {
    if (gSongIndexByte >= PLAYBACK_STEP)
        gSongIndexByte -= PLAYBACK_STEP;
    else {
        gSongIndexByte = 0;
        gSongIndexBitFromMsb = 0;
    }
}

void MoveSongForward(void) {
    if (gSongIndexByte < gSongDataSize)
        gSongIndexByte += PLAYBACK_STEP;
}

void ResetSongPosition(void) {
    gSongIndexByte = 0;
    gSongIndexBitFromMsb = 0;
}

void SetupSongPlayback(const unsigned char* song_data, size_t song_data_size) {
    gSongData = song_data;
    gSongDataSize = song_data_size;

    ResetSongPosition();
}

/*
 * Playing song
 */

/* Unpack bit-packed data */
bool FetchNextSongSample(uint16_t* write_to) {
    if (gSongIndexByte + 2 >= gSongDataSize)
        return false;

    uint16_t sample = *(uint16_t*) &gSongData[gSongIndexByte];

    sample = (sample >> 8) | (sample << 8);
    sample <<= gSongIndexBitFromMsb;
    sample >>= 5;

    if (gSongIndexBitFromMsb < 5) {
        gSongIndexBitFromMsb += 11;

    } else if (gSongIndexBitFromMsb == 5) {
        gSongIndexByte += 2;
        gSongIndexBitFromMsb = 0;

    } else {
        uint32_t shift = gSongIndexBitFromMsb - 5;

        gSongIndexByte += 2;
        gSongIndexBitFromMsb = shift;

        if (gSongIndexByte >= gSongDataSize)
            return false;

        uint16_t sample_part2 = *(uint16_t*) &gSongData[gSongIndexByte];

        sample_part2 = (sample_part2 >> 8) | (sample_part2 << 8);
        sample_part2 >>= (16 - shift);

        sample |= sample_part2;
    }

    *write_to = sample;
    return true;
}
