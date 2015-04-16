/*
 *  Copyright (c) 2012 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef WEBRTC_MODULES_AUDIO_PROCESSING_NOISE_SUPPRESSION_IMPL_H_
#define WEBRTC_MODULES_AUDIO_PROCESSING_NOISE_SUPPRESSION_IMPL_H_

#include "webrtc/modules/audio_processing/include/audio_processing.h"
#include "webrtc/modules/audio_processing/processing_component.h"

namespace webrtc {

class AudioBuffer;
class CriticalSectionWrapper;

class NoiseSuppressionImpl : public NoiseSuppression,
                             public ProcessingComponent {
 public:
  NoiseSuppressionImpl(const AudioProcessing* apm,
                       CriticalSectionWrapper* crit);
  virtual ~NoiseSuppressionImpl();

  int AnalyzeCaptureAudio(AudioBuffer* audio);
  int ProcessCaptureAudio(AudioBuffer* audio);

  // NoiseSuppression implementation.
  virtual bool is_enabled() const override;
  virtual float speech_probability() const override;

 private:
  // NoiseSuppression implementation.
  virtual int Enable(bool enable) override;
  virtual int set_level(Level level) override;
  virtual Level level() const override;

  // ProcessingComponent implementation.
  virtual void* CreateHandle() const override;
  virtual int InitializeHandle(void* handle) const override;
  virtual int ConfigureHandle(void* handle) const override;
  virtual void DestroyHandle(void* handle) const override;
  virtual int num_handles_required() const override;
  virtual int GetHandleError(void* handle) const override;

  const AudioProcessing* apm_;
  CriticalSectionWrapper* crit_;
  Level level_;
};

}  // namespace webrtc

#endif  // WEBRTC_MODULES_AUDIO_PROCESSING_NOISE_SUPPRESSION_IMPL_H_
