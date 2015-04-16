/*
 *  Copyright (c) 2012 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef WEBRTC_MODULES_RTP_RTCP_SOURCE_RTP_RTCP_IMPL_H_
#define WEBRTC_MODULES_RTP_RTCP_SOURCE_RTP_RTCP_IMPL_H_

#include <list>
#include <vector>

#include "webrtc/modules/rtp_rtcp/interface/rtp_rtcp.h"
#include "webrtc/modules/rtp_rtcp/source/rtcp_receiver.h"
#include "webrtc/modules/rtp_rtcp/source/rtcp_sender.h"
#include "webrtc/modules/rtp_rtcp/source/rtp_sender.h"
#include "webrtc/system_wrappers/interface/scoped_ptr.h"
#include "webrtc/test/testsupport/gtest_prod_util.h"

namespace webrtc {

class ModuleRtpRtcpImpl : public RtpRtcp {
 public:
  explicit ModuleRtpRtcpImpl(const RtpRtcp::Configuration& configuration);

  virtual ~ModuleRtpRtcpImpl();

  // Returns the number of milliseconds until the module want a worker thread to
  // call Process.
  virtual int32_t TimeUntilNextProcess() override;

  // Process any pending tasks such as timeouts.
  virtual int32_t Process() override;

  // Receiver part.

  // Called when we receive an RTCP packet.
  virtual int32_t IncomingRtcpPacket(const uint8_t* incoming_packet,
                                     uint16_t incoming_packet_length) override;

  virtual void SetRemoteSSRC(const uint32_t ssrc) override;

  // Sender part.

  virtual int32_t RegisterSendPayload(const CodecInst& voice_codec) override;

  virtual int32_t RegisterSendPayload(const VideoCodec& video_codec) override;

  virtual int32_t DeRegisterSendPayload(const int8_t payload_type) override;

  int8_t SendPayloadType() const;

  // Register RTP header extension.
  virtual int32_t RegisterSendRtpHeaderExtension(
      const RTPExtensionType type,
      const uint8_t id) override;

  virtual int32_t DeregisterSendRtpHeaderExtension(
      const RTPExtensionType type) override;

  // Get start timestamp.
  virtual uint32_t StartTimestamp() const override;

  // Configure start timestamp, default is a random number.
  virtual int32_t SetStartTimestamp(const uint32_t timestamp) override;

  virtual uint16_t SequenceNumber() const override;

  // Set SequenceNumber, default is a random number.
  virtual int32_t SetSequenceNumber(const uint16_t seq) override;

  virtual void SetRtpStateForSsrc(uint32_t ssrc,
                                  const RtpState& rtp_state) override;
  virtual bool GetRtpStateForSsrc(uint32_t ssrc, RtpState* rtp_state) override;

  virtual uint32_t SSRC() const override;

  // Configure SSRC, default is a random number.
  virtual void SetSSRC(const uint32_t ssrc) override;

  virtual int32_t CSRCs(uint32_t arr_of_csrc[kRtpCsrcSize]) const override;

  virtual int32_t SetCSRCs(const uint32_t arr_of_csrc[kRtpCsrcSize],
                           const uint8_t arr_length) override;

  virtual int32_t SetCSRCStatus(const bool include) override;

  RTCPSender::FeedbackState GetFeedbackState();

  int CurrentSendFrequencyHz() const;

  virtual void SetRTXSendStatus(const int mode) override;

  virtual void RTXSendStatus(int* mode, uint32_t* ssrc,
                             int* payloadType) const override;

  virtual void SetRtxSsrc(uint32_t ssrc) override;

  virtual void SetRtxSendPayloadType(int payload_type) override;

  // Sends kRtcpByeCode when going from true to false.
  virtual int32_t SetSendingStatus(const bool sending) override;

  virtual bool Sending() const override;

  // Drops or relays media packets.
  virtual int32_t SetSendingMediaStatus(const bool sending) override;

  virtual bool SendingMedia() const override;

  // Used by the codec module to deliver a video or audio frame for
  // packetization.
  virtual int32_t SendOutgoingData(
      const FrameType frame_type,
      const int8_t payload_type,
      const uint32_t time_stamp,
      int64_t capture_time_ms,
      const uint8_t* payload_data,
      const uint32_t payload_size,
      const RTPFragmentationHeader* fragmentation = NULL,
      const RTPVideoHeader* rtp_video_hdr = NULL) override;

  virtual bool TimeToSendPacket(uint32_t ssrc,
                                uint16_t sequence_number,
                                int64_t capture_time_ms,
                                bool retransmission) override;
  // Returns the number of padding bytes actually sent, which can be more or
  // less than |bytes|.
  virtual int TimeToSendPadding(int bytes) override;

  virtual bool GetSendSideDelay(int* avg_send_delay_ms,
                                int* max_send_delay_ms) const override;

  // RTCP part.

  // Get RTCP status.
  virtual RTCPMethod RTCP() const override;

  // Configure RTCP status i.e on/off.
  virtual int32_t SetRTCPStatus(const RTCPMethod method) override;

  // Set RTCP CName.
  virtual int32_t SetCNAME(const char c_name[RTCP_CNAME_SIZE]) override;

  // Get remote CName.
  virtual int32_t RemoteCNAME(const uint32_t remote_ssrc,
                              char c_name[RTCP_CNAME_SIZE]) const override;

  // Get remote NTP.
  virtual int32_t RemoteNTP(uint32_t* received_ntp_secs,
                            uint32_t* received_ntp_frac,
                            uint32_t* rtcp_arrival_time_secs,
                            uint32_t* rtcp_arrival_time_frac,
                            uint32_t* rtcp_timestamp) const override;

  virtual int32_t AddMixedCNAME(const uint32_t ssrc,
                                const char c_name[RTCP_CNAME_SIZE]) override;

  virtual int32_t RemoveMixedCNAME(const uint32_t ssrc) override;

  // Get RoundTripTime.
  virtual int32_t RTT(const uint32_t remote_ssrc,
                      uint16_t* rtt,
                      uint16_t* avg_rtt,
                      uint16_t* min_rtt,
                      uint16_t* max_rtt) const override;

  // Reset RoundTripTime statistics.
  virtual int32_t ResetRTT(const uint32_t remote_ssrc) override;

  // Force a send of an RTCP packet.
  // Normal SR and RR are triggered via the process function.
  virtual int32_t SendRTCP(uint32_t rtcp_packet_type = kRtcpReport) override;

  virtual int32_t ResetSendDataCountersRTP() override;

  // Statistics of the amount of data sent and received.
  virtual int32_t DataCountersRTP(uint32_t* bytes_sent,
                                  uint32_t* packets_sent) const override;

  // Get received RTCP report, sender info.
  virtual int32_t RemoteRTCPStat(RTCPSenderInfo* sender_info) override;

  // Get received RTCP report, report block.
  virtual int32_t RemoteRTCPStat(
      std::vector<RTCPReportBlock>* receive_blocks) const override;

  // Set received RTCP report block.
  virtual int32_t AddRTCPReportBlock(
      const uint32_t ssrc, const RTCPReportBlock* receive_block) override;

  virtual int32_t RemoveRTCPReportBlock(const uint32_t ssrc) override;

  virtual void GetRtcpPacketTypeCounters(
      RtcpPacketTypeCounter* packets_sent,
      RtcpPacketTypeCounter* packets_received) const override;

  // (REMB) Receiver Estimated Max Bitrate.
  virtual bool REMB() const override;

  virtual int32_t SetREMBStatus(const bool enable) override;

  virtual int32_t SetREMBData(const uint32_t bitrate,
                              const uint8_t number_of_ssrc,
                              const uint32_t* ssrc) override;

  // (IJ) Extended jitter report.
  virtual bool IJ() const override;

  virtual int32_t SetIJStatus(const bool enable) override;

  // (TMMBR) Temporary Max Media Bit Rate.
  virtual bool TMMBR() const override;

  virtual int32_t SetTMMBRStatus(const bool enable) override;

  int32_t SetTMMBN(const TMMBRSet* bounding_set);

  virtual uint16_t MaxPayloadLength() const override;

  virtual uint16_t MaxDataPayloadLength() const override;

  virtual int32_t SetMaxTransferUnit(const uint16_t size) override;

  virtual int32_t SetTransportOverhead(
      const bool tcp,
      const bool ipv6,
      const uint8_t authentication_overhead = 0) override;

  // (NACK) Negative acknowledgment part.

  virtual int SelectiveRetransmissions() const override;

  virtual int SetSelectiveRetransmissions(uint8_t settings) override;

  // Send a Negative acknowledgment packet.
  virtual int32_t SendNACK(const uint16_t* nack_list,
                           const uint16_t size) override;

  // Store the sent packets, needed to answer to a negative acknowledgment
  // requests.
  virtual int32_t SetStorePacketsStatus(
      const bool enable, const uint16_t number_to_store) override;

  virtual bool StorePackets() const override;

  // Called on receipt of RTCP report block from remote side.
  virtual void RegisterSendChannelRtcpStatisticsCallback(
      RtcpStatisticsCallback* callback) override;
  virtual RtcpStatisticsCallback*
      GetSendChannelRtcpStatisticsCallback() override;

  // (APP) Application specific data.
  virtual int32_t SetRTCPApplicationSpecificData(
      const uint8_t sub_type,
      const uint32_t name,
      const uint8_t* data,
      const uint16_t length) override;

  // (XR) VOIP metric.
  virtual int32_t SetRTCPVoIPMetrics(const RTCPVoIPMetric* VoIPMetric) override;

  // (XR) Receiver reference time report.
  virtual void SetRtcpXrRrtrStatus(bool enable) override;

  virtual bool RtcpXrRrtrStatus() const override;

  // Audio part.

  // Set audio packet size, used to determine when it's time to send a DTMF
  // packet in silence (CNG).
  virtual int32_t SetAudioPacketSize(
      const uint16_t packet_size_samples) override;

  virtual bool SendTelephoneEventActive(int8_t& telephone_event) const override;

  // Send a TelephoneEvent tone using RFC 2833 (4733).
  virtual int32_t SendTelephoneEventOutband(const uint8_t key,
                                            const uint16_t time_ms,
                                            const uint8_t level) override;

  // Set payload type for Redundant Audio Data RFC 2198.
  virtual int32_t SetSendREDPayloadType(const int8_t payload_type) override;

  // Get payload type for Redundant Audio Data RFC 2198.
  virtual int32_t SendREDPayloadType(int8_t& payload_type) const override;

  // Store the audio level in d_bov for header-extension-for-audio-level-
  // indication.
  virtual int32_t SetAudioLevel(const uint8_t level_d_bov) override;

  // Video part.

  virtual int32_t SendRTCPSliceLossIndication(
      const uint8_t picture_id) override;

  // Set method for requestion a new key frame.
  virtual int32_t SetKeyFrameRequestMethod(
      const KeyFrameRequestMethod method) override;

  // Send a request for a keyframe.
  virtual int32_t RequestKeyFrame() override;

  virtual int32_t SetCameraDelay(const int32_t delay_ms) override;

  virtual void SetTargetSendBitrate(
      const std::vector<uint32_t>& stream_bitrates) override;

  virtual int32_t SetGenericFECStatus(
      const bool enable,
      const uint8_t payload_type_red,
      const uint8_t payload_type_fec) override;

  virtual int32_t GenericFECStatus(
      bool& enable,
      uint8_t& payload_type_red,
      uint8_t& payload_type_fec) override;

  virtual int32_t SetFecParameters(
      const FecProtectionParams* delta_params,
      const FecProtectionParams* key_params) override;

  bool LastReceivedNTP(uint32_t* NTPsecs,
                       uint32_t* NTPfrac,
                       uint32_t* remote_sr) const;

  bool LastReceivedXrReferenceTimeInfo(RtcpReceiveTimeInfo* info) const;

  virtual int32_t BoundingSet(bool& tmmbr_owner, TMMBRSet*& bounding_set_rec);

  virtual void BitrateSent(uint32_t* total_rate,
                           uint32_t* video_rate,
                           uint32_t* fec_rate,
                           uint32_t* nackRate) const override;

  uint32_t SendTimeOfSendReport(const uint32_t send_report);

  bool SendTimeOfXrRrReport(uint32_t mid_ntp, int64_t* time_ms) const;

  // Good state of RTP receiver inform sender.
  virtual int32_t SendRTCPReferencePictureSelection(
      const uint64_t picture_id) override;

  virtual void RegisterSendChannelRtpStatisticsCallback(
      StreamDataCountersCallback* callback) override;
  virtual StreamDataCountersCallback*
      GetSendChannelRtpStatisticsCallback() const override;

  void OnReceivedTMMBR();

  // Bad state of RTP receiver request a keyframe.
  void OnRequestIntraFrame();

  // Received a request for a new SLI.
  void OnReceivedSliceLossIndication(const uint8_t picture_id);

  // Received a new reference frame.
  void OnReceivedReferencePictureSelectionIndication(
      const uint64_t picture_id);

  void OnReceivedNACK(const std::list<uint16_t>& nack_sequence_numbers);

  void OnRequestSendReport();

 protected:
  void RegisterChildModule(RtpRtcp* module);

  void DeRegisterChildModule(RtpRtcp* module);

  bool UpdateRTCPReceiveInformationTimers();

  uint32_t BitrateReceivedNow() const;

  // Get remote SequenceNumber.
  uint16_t RemoteSequenceNumber() const;

  // Only for internal testing.
  uint32_t LastSendReport(uint32_t& last_rtcptime);

  RTPSender                 rtp_sender_;

  RTCPSender                rtcp_sender_;
  RTCPReceiver              rtcp_receiver_;

  Clock*                    clock_;

 private:
  FRIEND_TEST_ALL_PREFIXES(RtpRtcpImplTest, Rtt);
  FRIEND_TEST_ALL_PREFIXES(RtpRtcpImplTest, RttForReceiverOnly);
  int64_t RtcpReportInterval();
  void SetRtcpReceiverSsrcs(uint32_t main_ssrc);

  void set_rtt_ms(uint32_t rtt_ms);
  uint32_t rtt_ms() const;

  bool IsDefaultModule() const;

  int32_t             id_;
  const bool                audio_;
  bool                      collision_detected_;
  int64_t             last_process_time_;
  int64_t             last_bitrate_process_time_;
  int64_t             last_rtt_process_time_;
  uint16_t            packet_overhead_;

  scoped_ptr<CriticalSectionWrapper> critical_section_module_ptrs_;
  scoped_ptr<CriticalSectionWrapper> critical_section_module_ptrs_feedback_;
  ModuleRtpRtcpImpl*            default_module_;
  std::vector<ModuleRtpRtcpImpl*> child_modules_;
  size_t padding_index_;

  // Send side
  NACKMethod            nack_method_;
  uint32_t        nack_last_time_sent_full_;
  uint16_t        nack_last_seq_number_sent_;

  bool                  simulcast_;
  VideoCodec            send_video_codec_;
  KeyFrameRequestMethod key_frame_req_method_;

  RemoteBitrateEstimator* remote_bitrate_;

  RtcpRttStats* rtt_stats_;

  // The processed RTT from RtcpRttStats.
  scoped_ptr<CriticalSectionWrapper> critical_section_rtt_;
  uint32_t rtt_ms_;
};

}  // namespace webrtc

#endif  // WEBRTC_MODULES_RTP_RTCP_SOURCE_RTP_RTCP_IMPL_H_
