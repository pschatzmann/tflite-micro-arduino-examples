/* Copyright 2018 The TensorFlow Authors. All Rights Reserved.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
==============================================================================*/

#if defined(ARDUINO) && defined(ARDUINO_ARDUINO_NANO33BLE)
#define ARDUINO_EXCLUDE_CODE
#endif  // defined(ARDUINO) && !defined(ARDUINO_ARDUINO_NANO33BLE)

#ifndef ARDUINO_EXCLUDE_CODE

#include <algorithm>
#include <cmath>
#include "audio_provider.h"
#include "micro_features_micro_model_settings.h"
#include "test_over_serial/test_over_serial.h"
#include "AudioTools.h"
#include "AudioLibs/AudioKit.h"

AudioKitStream i2s;
bool g_is_audio_initialized = false;
int16_t g_audio_output_buffer[kMaxAudioSampleSize];
int32_t g_latest_audio_timestamp = 0;
tflite::ErrorReporter* g_error_reporter;

/// Setup I2S
TfLiteStatus InitAudioRecording(tflite::ErrorReporter* error_reporter) {
  AudioLogger::instance().begin(Serial, AudioLogger::Info);

  if (!g_is_audio_initialized) {
    g_error_reporter = error_reporter;
    // Start listening for audio: MONO @ 16KHz
    auto cfg = i2s.defaultConfig(RX_MODE);
    cfg.channels = 1;
    cfg.use_apll = false;
    cfg.auto_clear = false;
    cfg.sample_rate = kAudioSampleFrequency;
    cfg.input_device = AUDIO_HAL_ADC_INPUT_LINE2;
    cfg.buffer_size = 512;
    cfg.buffer_count = 16;
    i2s.begin(cfg);
    g_is_audio_initialized = true;
  }

  return kTfLiteOk;
}

/// Just return the data using readBytes
TfLiteStatus GetAudioSamples(tflite::ErrorReporter* error_reporter,
                             int start_ms, int duration_ms,
                             int* audio_samples_size, int16_t** audio_samples) {
  LOGD(LOG_METHOD);
  // Determine how many samples we want in total
  const int duration_sample_count = duration_ms * (kAudioSampleFrequency / 1000);
  const int duration_byte_count = duration_sample_count * 2;
  LOGD("start: %d - duration: %d -> samples: %d", start_ms, duration_ms, duration_sample_count);
  // blocking read to provide the requested data
  int num_read = i2s.readBytes((uint8_t*)g_audio_output_buffer, duration_byte_count);
  if (num_read!=duration_byte_count){
    LOGE("readBytes: %d->%d",duration_byte_count, num_read);
    return kTfLiteError;
  }
  // Set pointers to provide access to the audio
  *audio_samples_size = duration_sample_count;
  *audio_samples = g_audio_output_buffer; 

  return kTfLiteOk;
}

// We return the to be time for  2-4 frames (1 frame=20ms)
int32_t LatestAudioTimestamp() {
  LOGD(LOG_METHOD);
  g_latest_audio_timestamp += 40; // time for 2 frames
  LOGD("g_latest_audio_timestamp: %d",g_latest_audio_timestamp);
  return g_latest_audio_timestamp;
}

#endif  // ARDUINO_EXCLUDE_CODE
