#include "boot_audio.h"
#include "audio.h"

static bool s_splash_done = false;
static bool s_ble_adv_ready = false;
static bool s_ready_to_pair_played = false;

static void try_ready_to_pair(void)
{
    if (!s_splash_done || !s_ble_adv_ready || s_ready_to_pair_played) {
        return;
    }
    s_ready_to_pair_played = true;
    audio_play_ready_to_pair();
}

void boot_audio_on_splash_done(void)
{
    s_splash_done = true;
    try_ready_to_pair();
}

void boot_audio_on_ble_adv_ready(void)
{
    s_ble_adv_ready = true;
    try_ready_to_pair();
}
