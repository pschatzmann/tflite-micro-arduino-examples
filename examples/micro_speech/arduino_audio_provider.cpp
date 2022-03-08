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

#define DEFAULT_PDM_BUFFER_SIZE 1024

I2SStream i2s;

using namespace test_over_serial;

namespace {
bool g_is_audio_initialized = false;
// An internal buffer able to fit 16x our sample size
// A buffer that holds our output
int16_t g_audio_output_buffer[kMaxAudioSampleSize];
// Mark as volatile so we can check in a while loop to see if
// any samples have arrived yet.
volatile int32_t g_latest_audio_timestamp = 0;
// error reporter
tflite::ErrorReporter* g_error_reporter;
// test_over_serial silence insertion flag
}  // namespace


TfLiteStatus InitAudioRecording(tflite::ErrorReporter* error_reporter) {
  AudioLogger::instance().begin(Serial, AudioLogger::Info);

  if (!g_is_audio_initialized) {
    g_error_reporter = error_reporter;
    g_latest_audio_timestamp = millis();
    // Start listening for audio: MONO @ 16KHz
    auto cfg = i2s.defaultConfig(RX_MODE);
    cfg.channels = 1;
    cfg.sample_rate = kAudioSampleFrequency;
    i2s.begin(cfg);
    g_is_audio_initialized = true;

  }

  return kTfLiteOk;
}

// This next part should only be called when the main thread notices that the
// latest audio sample data timestamp has changed, so that there's new data
TfLiteStatus GetAudioSamples(tflite::ErrorReporter* error_reporter,
                             int start_ms, int duration_ms,
                             int* audio_samples_size, int16_t** audio_samples) {

  LOGI(LOG_METHOD);
  // Determine how many samples we want in total
  const int duration_sample_count =  duration_ms * (kAudioSampleFrequency / 1000);
  const int duration_byte_count = duration_sample_count * 2;
  // blocking read to provide the requested data
  int num_read = i2s.readBytes((uint8_t*)g_audio_output_buffer, duration_byte_count);
  LOGI("num_read: %d",num_read);

  // Set pointers to provide access to the audio
  *audio_samples_size = duration_sample_count;
  *audio_samples = g_audio_output_buffer;

  return kTfLiteOk;
}

int32_t LatestAudioTimestamp() {
  LOGI(LOG_METHOD);
  // trigger next call to GetAudioSamples
  if (i2s.available()>1024){
    g_latest_audio_timestamp = millis();
  }
  return g_latest_audio_timestamp;
}

#endif  // ARDUINO_EXCLUDE_CODE
