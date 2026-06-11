#ifndef BOOT_AUDIO_H
#define BOOT_AUDIO_H

/**
 * Coordinates boot voice prompts that depend on splash + BLE readiness.
 */
void boot_audio_on_splash_done(void);
void boot_audio_on_ble_adv_ready(void);

#endif // BOOT_AUDIO_H
