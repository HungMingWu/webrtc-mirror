/*
 *  Copyright (c) 2012 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef WEBRTC_MODULES_AUDIO_CODING_MAIN_ACM2_AUDIO_CODING_MODULE_IMPL_H_
#define WEBRTC_MODULES_AUDIO_CODING_MAIN_ACM2_AUDIO_CODING_MODULE_IMPL_H_

#include <vector>

#include "webrtc/base/thread_annotations.h"
#include "webrtc/common_types.h"
#include "webrtc/engine_configurations.h"
#include "webrtc/modules/audio_coding/main/acm2/acm_codec_database.h"
#include "webrtc/modules/audio_coding/main/acm2/acm_receiver.h"
#include "webrtc/modules/audio_coding/main/acm2/acm_resampler.h"
#include "webrtc/system_wrappers/interface/scoped_ptr.h"

namespace webrtc {

class CriticalSectionWrapper;

namespace acm2 {

class ACMDTMFDetection;
class ACMGenericCodec;

class AudioCodingModuleImpl : public AudioCodingModule {
 public:
  explicit AudioCodingModuleImpl(const AudioCodingModule::Config& config);
  ~AudioCodingModuleImpl();

  // Change the unique identifier of this object.
  virtual int32_t ChangeUniqueId(const int32_t id) override;

  // Returns the number of milliseconds until the module want a worker thread
  // to call Process.
  virtual int32_t TimeUntilNextProcess() override;

  // Process any pending tasks such as timeouts.
  virtual int32_t Process() override;

  /////////////////////////////////////////
  //   Sender
  //

  // Initialize send codec.
  virtual int InitializeSender() override;

  // Reset send codec.
  virtual int ResetEncoder() override;

  // Can be called multiple times for Codec, CNG, RED.
  virtual int RegisterSendCodec(const CodecInst& send_codec) override;

  // Register Secondary codec for dual-streaming. Dual-streaming is activated
  // right after the secondary codec is registered.
  virtual int RegisterSecondarySendCodec(const CodecInst& send_codec) override;

  // Unregister the secondary codec. Dual-streaming is deactivated right after
  // deregistering secondary codec.
  virtual void UnregisterSecondarySendCodec() override;

  // Get the secondary codec.
  virtual int SecondarySendCodec(CodecInst* secondary_codec) const override;

  // Get current send codec.
  virtual int SendCodec(CodecInst* current_codec) const override;

  // Get current send frequency.
  virtual int SendFrequency() const override;

  // Get encode bit-rate.
  // Adaptive rate codecs return their current encode target rate, while other
  // codecs return there long-term average or their fixed rate.
  virtual int SendBitrate() const override;

  // Set available bandwidth, inform the encoder about the
  // estimated bandwidth received from the remote party.
  virtual int SetReceivedEstimatedBandwidth(int bw) override;

  // Register a transport callback which will be
  // called to deliver the encoded buffers.
  virtual int RegisterTransportCallback(
      AudioPacketizationCallback* transport) override;

  // Add 10 ms of raw (PCM) audio data to the encoder.
  virtual int Add10MsData(const AudioFrame& audio_frame) override;

  /////////////////////////////////////////
  // (RED) Redundant Coding
  //

  // Configure RED status i.e. on/off.
  virtual int SetREDStatus(bool enable_red) override;

  // Get RED status.
  virtual bool REDStatus() const override;

  /////////////////////////////////////////
  // (FEC) Forward Error Correction (codec internal)
  //

  // Configure FEC status i.e. on/off.
  virtual int SetCodecFEC(bool enabled_codec_fec) override;

  // Get FEC status.
  virtual bool CodecFEC() const override;

  // Set target packet loss rate
  virtual int SetPacketLossRate(int loss_rate) override;

  /////////////////////////////////////////
  //   (VAD) Voice Activity Detection
  //   and
  //   (CNG) Comfort Noise Generation
  //

  virtual int SetVAD(bool enable_dtx = true,
                     bool enable_vad = false,
                     ACMVADMode mode = VADNormal) override;

  virtual int VAD(bool* dtx_enabled,
                  bool* vad_enabled,
                  ACMVADMode* mode) const override;

  virtual int RegisterVADCallback(ACMVADCallback* vad_callback) override;

  /////////////////////////////////////////
  //   Receiver
  //

  // Initialize receiver, resets codec database etc.
  virtual int InitializeReceiver() override;

  // Reset the decoder state.
  virtual int ResetDecoder() override;

  // Get current receive frequency.
  virtual int ReceiveFrequency() const override;

  // Get current playout frequency.
  virtual int PlayoutFrequency() const override;

  // Register possible receive codecs, can be called multiple times,
  // for codecs, CNG, DTMF, RED.
  virtual int RegisterReceiveCodec(const CodecInst& receive_codec) override;

  // Get current received codec.
  virtual int ReceiveCodec(CodecInst* current_codec) const override;

  // Incoming packet from network parsed and ready for decode.
  virtual int IncomingPacket(const uint8_t* incoming_payload,
                             int payload_length,
                             const WebRtcRTPHeader& rtp_info) override;

  // Incoming payloads, without rtp-info, the rtp-info will be created in ACM.
  // One usage for this API is when pre-encoded files are pushed in ACM.
  virtual int IncomingPayload(const uint8_t* incoming_payload,
                              int payload_length,
                              uint8_t payload_type,
                              uint32_t timestamp) override;

  // Minimum playout delay.
  virtual int SetMinimumPlayoutDelay(int time_ms) override;

  // Maximum playout delay.
  virtual int SetMaximumPlayoutDelay(int time_ms) override;

  // Smallest latency NetEq will maintain.
  virtual int LeastRequiredDelayMs() const override;

  // Impose an initial delay on playout. ACM plays silence until |delay_ms|
  // audio is accumulated in NetEq buffer, then starts decoding payloads.
  virtual int SetInitialPlayoutDelay(int delay_ms) override;

  // TODO(turajs): DTMF playout is always activated in NetEq these APIs should
  // be removed, as well as all VoE related APIs and methods.
  //
  // Configure Dtmf playout status i.e on/off playout the incoming outband Dtmf
  // tone.
  virtual int SetDtmfPlayoutStatus(bool enable) override { return 0; }

  // Get Dtmf playout status.
  virtual bool DtmfPlayoutStatus() const override { return true; }

  // Estimate the Bandwidth based on the incoming stream, needed
  // for one way audio where the RTCP send the BW estimate.
  // This is also done in the RTP module .
  virtual int DecoderEstimatedBandwidth() const override;

  // Set playout mode voice, fax.
  virtual int SetPlayoutMode(AudioPlayoutMode mode) override;

  // Get playout mode voice, fax.
  virtual AudioPlayoutMode PlayoutMode() const override;

  // Get playout timestamp.
  virtual int PlayoutTimestamp(uint32_t* timestamp) override;

  // Get 10 milliseconds of raw audio data to play out, and
  // automatic resample to the requested frequency if > 0.
  virtual int PlayoutData10Ms(int desired_freq_hz,
                              AudioFrame* audio_frame) override;

  /////////////////////////////////////////
  //   Statistics
  //

  virtual int NetworkStatistics(ACMNetworkStatistics* statistics) override;

  // GET RED payload for iSAC. The method id called when 'this' ACM is
  // the default ACM.
  // TODO(henrik.lundin) Not used. Remove?
  int REDPayloadISAC(int isac_rate,
                     int isac_bw_estimate,
                     uint8_t* payload,
                     int16_t* length_bytes);

  virtual int ReplaceInternalDTXWithWebRtc(bool use_webrtc_dtx) override;

  virtual int IsInternalDTXReplacedWithWebRtc(bool* uses_webrtc_dtx) override;

  virtual int SetISACMaxRate(int max_bit_per_sec) override;

  virtual int SetISACMaxPayloadSize(int max_size_bytes) override;

  virtual int ConfigISACBandwidthEstimator(
      int frame_size_ms,
      int rate_bit_per_sec,
      bool enforce_frame_size = false) override;

  // If current send codec is Opus, informs it about the maximum playback rate
  // the receiver will render.
  virtual int SetOpusMaxPlaybackRate(int frequency_hz) override;

  virtual int UnregisterReceiveCodec(uint8_t payload_type) override;

  virtual int EnableNack(size_t max_nack_list_size) override;

  virtual void DisableNack() override;

  virtual std::vector<uint16_t> GetNackList(
      int round_trip_time_ms) const override;

  virtual void GetDecodingCallStatistics(
      AudioDecodingCallStats* stats) const override;

 private:
  int UnregisterReceiveCodecSafe(int payload_type);

  ACMGenericCodec* CreateCodec(const CodecInst& codec);

  int InitializeReceiverSafe() EXCLUSIVE_LOCKS_REQUIRED(acm_crit_sect_);

  bool HaveValidEncoder(const char* caller_name) const
      EXCLUSIVE_LOCKS_REQUIRED(acm_crit_sect_);

  // Set VAD/DTX status. This function does not acquire a lock, and it is
  // created to be called only from inside a critical section.
  int SetVADSafe(bool enable_dtx, bool enable_vad, ACMVADMode mode)
      EXCLUSIVE_LOCKS_REQUIRED(acm_crit_sect_);

  // Process buffered audio when dual-streaming is not enabled (When RED is
  // enabled still this function is used.)
  int ProcessSingleStream();

  // Process buffered audio when dual-streaming is enabled, i.e. secondary send
  // codec is registered.
  int ProcessDualStream();

  // Preprocessing of input audio, including resampling and down-mixing if
  // required, before pushing audio into encoder's buffer.
  //
  // in_frame: input audio-frame
  // ptr_out: pointer to output audio_frame. If no preprocessing is required
  //          |ptr_out| will be pointing to |in_frame|, otherwise pointing to
  //          |preprocess_frame_|.
  //
  // Return value:
  //   -1: if encountering an error.
  //    0: otherwise.
  int PreprocessToAddData(const AudioFrame& in_frame,
                          const AudioFrame** ptr_out)
      EXCLUSIVE_LOCKS_REQUIRED(acm_crit_sect_);

  // Change required states after starting to receive the codec corresponding
  // to |index|.
  int UpdateUponReceivingCodec(int index);

  int EncodeFragmentation(int fragmentation_index,
                          int payload_type,
                          uint32_t current_timestamp,
                          ACMGenericCodec* encoder,
                          uint8_t* stream)
      EXCLUSIVE_LOCKS_REQUIRED(acm_crit_sect_);

  void ResetFragmentation(int vector_size)
      EXCLUSIVE_LOCKS_REQUIRED(acm_crit_sect_);

  // Get a pointer to AudioDecoder of the given codec. For some codecs, e.g.
  // iSAC, encoding and decoding have to be performed on a shared
  // codec-instance. By calling this method, we get the codec-instance that ACM
  // owns, then pass that to NetEq. This way, we perform both encoding and
  // decoding on the same codec-instance. Furthermore, ACM would have control
  // over decoder functionality if required. If |codec| does not share an
  // instance between encoder and decoder, the |*decoder| is set NULL.
  // The field ACMCodecDB::CodecSettings.owns_decoder indicates that if a
  // codec owns the decoder-instance. For such codecs |*decoder| should be a
  // valid pointer, otherwise it will be NULL.
  int GetAudioDecoder(const CodecInst& codec, int codec_id,
                      int mirror_id, AudioDecoder** decoder)
      EXCLUSIVE_LOCKS_REQUIRED(acm_crit_sect_);

  CriticalSectionWrapper* acm_crit_sect_;
  int id_;  // TODO(henrik.lundin) Make const.
  uint32_t expected_codec_ts_ GUARDED_BY(acm_crit_sect_);
  uint32_t expected_in_ts_ GUARDED_BY(acm_crit_sect_);
  CodecInst send_codec_inst_ GUARDED_BY(acm_crit_sect_);

  uint8_t cng_nb_pltype_ GUARDED_BY(acm_crit_sect_);
  uint8_t cng_wb_pltype_ GUARDED_BY(acm_crit_sect_);
  uint8_t cng_swb_pltype_ GUARDED_BY(acm_crit_sect_);
  uint8_t cng_fb_pltype_ GUARDED_BY(acm_crit_sect_);

  uint8_t red_pltype_ GUARDED_BY(acm_crit_sect_);
  bool vad_enabled_ GUARDED_BY(acm_crit_sect_);
  bool dtx_enabled_ GUARDED_BY(acm_crit_sect_);
  ACMVADMode vad_mode_ GUARDED_BY(acm_crit_sect_);
  ACMGenericCodec* codecs_[ACMCodecDB::kMaxNumCodecs]
      GUARDED_BY(acm_crit_sect_);
  int mirror_codec_idx_[ACMCodecDB::kMaxNumCodecs] GUARDED_BY(acm_crit_sect_);
  bool stereo_send_ GUARDED_BY(acm_crit_sect_);
  int current_send_codec_idx_ GUARDED_BY(acm_crit_sect_);
  bool send_codec_registered_ GUARDED_BY(acm_crit_sect_);
  ACMResampler resampler_ GUARDED_BY(acm_crit_sect_);
  AcmReceiver receiver_;  // AcmReceiver has it's own internal lock.

  // RED.
  bool is_first_red_ GUARDED_BY(acm_crit_sect_);
  bool red_enabled_ GUARDED_BY(acm_crit_sect_);

  // TODO(turajs): |red_buffer_| is allocated in constructor, why having them
  // as pointers and not an array. If concerned about the memory, then make a
  // set-up function to allocate them only when they are going to be used, i.e.
  // RED or Dual-streaming is enabled.
  uint8_t* red_buffer_ GUARDED_BY(acm_crit_sect_);

  // TODO(turajs): we actually don't need |fragmentation_| as a member variable.
  // It is sufficient to keep the length & payload type of previous payload in
  // member variables.
  RTPFragmentationHeader fragmentation_ GUARDED_BY(acm_crit_sect_);
  uint32_t last_red_timestamp_ GUARDED_BY(acm_crit_sect_);

  // Codec internal FEC
  bool codec_fec_enabled_ GUARDED_BY(acm_crit_sect_);

  // This is to keep track of CN instances where we can send DTMFs.
  uint8_t previous_pltype_ GUARDED_BY(acm_crit_sect_);

  // Used when payloads are pushed into ACM without any RTP info
  // One example is when pre-encoded bit-stream is pushed from
  // a file.
  // IMPORTANT: this variable is only used in IncomingPayload(), therefore,
  // no lock acquired when interacting with this variable. If it is going to
  // be used in other methods, locks need to be taken.
  WebRtcRTPHeader* aux_rtp_header_;

  bool receiver_initialized_ GUARDED_BY(acm_crit_sect_);

  AudioFrame preprocess_frame_ GUARDED_BY(acm_crit_sect_);
  CodecInst secondary_send_codec_inst_ GUARDED_BY(acm_crit_sect_);
  scoped_ptr<ACMGenericCodec> secondary_encoder_ GUARDED_BY(acm_crit_sect_);
  uint32_t codec_timestamp_ GUARDED_BY(acm_crit_sect_);
  bool first_10ms_data_ GUARDED_BY(acm_crit_sect_);

  CriticalSectionWrapper* callback_crit_sect_;
  AudioPacketizationCallback* packetization_callback_
      GUARDED_BY(callback_crit_sect_);
  ACMVADCallback* vad_callback_ GUARDED_BY(callback_crit_sect_);
};

}  // namespace acm2

class AudioCodingImpl : public AudioCoding {
 public:
  AudioCodingImpl(const Config& config) {
    AudioCodingModule::Config config_old = config.ToOldConfig();
    acm_old_.reset(new acm2::AudioCodingModuleImpl(config_old));
    acm_old_->RegisterTransportCallback(config.transport);
    acm_old_->RegisterVADCallback(config.vad_callback);
    acm_old_->SetDtmfPlayoutStatus(config.play_dtmf);
    if (config.initial_playout_delay_ms > 0) {
      acm_old_->SetInitialPlayoutDelay(config.initial_playout_delay_ms);
    }
    playout_frequency_hz_ = config.playout_frequency_hz;
  }

  virtual ~AudioCodingImpl() override {};

  virtual bool RegisterSendCodec(AudioEncoder* send_codec) override;

  virtual bool RegisterSendCodec(int encoder_type,
                                 uint8_t payload_type,
                                 int frame_size_samples = 0) override;

  virtual const AudioEncoder* GetSenderInfo() const override;

  virtual const CodecInst* GetSenderCodecInst() override;

  virtual int Add10MsAudio(const AudioFrame& audio_frame) override;

  virtual const ReceiverInfo* GetReceiverInfo() const override;

  virtual bool RegisterReceiveCodec(AudioDecoder* receive_codec) override;

  virtual bool RegisterReceiveCodec(int decoder_type,
                                    uint8_t payload_type) override;

  virtual bool InsertPacket(const uint8_t* incoming_payload,
                            int32_t payload_len_bytes,
                            const WebRtcRTPHeader& rtp_info) override;

  virtual bool InsertPayload(const uint8_t* incoming_payload,
                             int32_t payload_len_byte,
                             uint8_t payload_type,
                             uint32_t timestamp) override;

  virtual bool SetMinimumPlayoutDelay(int time_ms) override;

  virtual bool SetMaximumPlayoutDelay(int time_ms) override;

  virtual int LeastRequiredDelayMs() const override;

  virtual bool PlayoutTimestamp(uint32_t* timestamp) override;

  virtual bool Get10MsAudio(AudioFrame* audio_frame) override;

  virtual bool NetworkStatistics(
      ACMNetworkStatistics* network_statistics) override;

  virtual bool EnableNack(size_t max_nack_list_size) override;

  virtual void DisableNack() override;

  virtual bool SetVad(bool enable_dtx,
                      bool enable_vad,
                      ACMVADMode vad_mode) override;

  virtual std::vector<uint16_t> GetNackList(
      int round_trip_time_ms) const override;

  virtual void GetDecodingCallStatistics(
      AudioDecodingCallStats* call_stats) const override;

 private:
  // Temporary method to be used during redesign phase.
  // Maps |codec_type| (a value from the anonymous enum in acm2::ACMCodecDB) to
  // |codec_name|, |sample_rate_hz|, and |channels|.
  // TODO(henrik.lundin) Remove this when no longer needed.
  static bool MapCodecTypeToParameters(int codec_type,
                                       std::string* codec_name,
                                       int* sample_rate_hz,
                                       int* channels);

  int playout_frequency_hz_;
  // TODO(henrik.lundin): All members below this line are temporary and should
  // be removed after refactoring is completed.
  scoped_ptr<acm2::AudioCodingModuleImpl> acm_old_;
  CodecInst current_send_codec_;
};

}  // namespace webrtc

#endif  // WEBRTC_MODULES_AUDIO_CODING_MAIN_ACM2_AUDIO_CODING_MODULE_IMPL_H_
