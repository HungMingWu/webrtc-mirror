/*
 * libjingle
 * Copyright 2014, Google Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *  3. The name of the author may not be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
 * EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.appspot.apprtc;

import android.app.Activity;
import android.util.Log;

import org.appspot.apprtc.AppRTCClient.AppRTCSignalingParameters;
import org.webrtc.DataChannel;
import org.webrtc.IceCandidate;
import org.webrtc.MediaConstraints;
import org.webrtc.MediaStream;
import org.webrtc.MediaStreamTrack;
import org.webrtc.PeerConnection;
import org.webrtc.PeerConnection.IceConnectionState;
import org.webrtc.PeerConnectionFactory;
import org.webrtc.SdpObserver;
import org.webrtc.SessionDescription;
import org.webrtc.StatsObserver;
import org.webrtc.VideoCapturer;
import org.webrtc.VideoRenderer;
import org.webrtc.VideoSource;
import org.webrtc.VideoTrack;

import java.util.LinkedList;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

public class PeerConnectionClient {
  private static final String TAG = "RTCClient";
  private final Activity activity;
  private PeerConnectionFactory factory;
  private PeerConnection pc;
  private VideoSource videoSource;
  private boolean videoSourceStopped;
  private final PCObserver pcObserver = new PCObserver();
  private final SDPObserver sdpObserver = new SDPObserver();
  private final MediaConstraints videoConstraints;
  private final VideoRenderer.Callbacks localRender;
  private final VideoRenderer.Callbacks remoteRender;
  private LinkedList<IceCandidate> queuedRemoteCandidates =
      new LinkedList<IceCandidate>();
  private final MediaConstraints sdpMediaConstraints;
  private final PeerConnectionEvents events;
  private boolean isInitiator;
  private boolean useFrontFacingCamera = true;
  private SessionDescription localSdp = null; // either offer or answer SDP
  private MediaStream videoMediaStream = null;

  public PeerConnectionClient(
      Activity activity,
      VideoRenderer.Callbacks localRender,
      VideoRenderer.Callbacks remoteRender,
      AppRTCSignalingParameters appRtcParameters,
      PeerConnectionEvents events) {
    this.activity = activity;
    this.videoConstraints = appRtcParameters.videoConstraints;
    this.localRender = localRender;
    this.remoteRender = remoteRender;
    this.events = events;
    isInitiator = appRtcParameters.initiator;

    sdpMediaConstraints = new MediaConstraints();
    sdpMediaConstraints.mandatory.add(new MediaConstraints.KeyValuePair(
        "OfferToReceiveAudio", "true"));
    sdpMediaConstraints.mandatory.add(new MediaConstraints.KeyValuePair(
        "OfferToReceiveVideo", "true"));

    factory = new PeerConnectionFactory();

    MediaConstraints pcConstraints = appRtcParameters.pcConstraints;
    pcConstraints.optional.add(
        new MediaConstraints.KeyValuePair("RtpDataChannels", "true"));
    pc = factory.createPeerConnection(appRtcParameters.iceServers,
        pcConstraints, pcObserver);
    isInitiator = false;

    // Uncomment to get ALL WebRTC tracing and SENSITIVE libjingle logging.
    // NOTE: this _must_ happen while |factory| is alive!
    // Logging.enableTracing(
    //     "logcat:",
    //     EnumSet.of(Logging.TraceLevel.TRACE_ALL),
    //     Logging.Severity.LS_SENSITIVE);

    if (videoConstraints != null) {
      videoMediaStream = factory.createLocalMediaStream("ARDAMSVideo");
      videoMediaStream.addTrack(createVideoTrack(useFrontFacingCamera));
      pc.addStream(videoMediaStream, new MediaConstraints());
    }

    if (appRtcParameters.audioConstraints != null) {
      MediaStream lMS = factory.createLocalMediaStream("ARDAMSAudio");
      lMS.addTrack(factory.createAudioTrack(
          "ARDAMSa0",
          factory.createAudioSource(appRtcParameters.audioConstraints)));
      pc.addStream(lMS, new MediaConstraints());
    }
  }

  public boolean getStats(StatsObserver observer, MediaStreamTrack track) {
    return pc.getStats(observer, track);
  }

  public void createOffer() {
    isInitiator = true;
    pc.createOffer(sdpObserver, sdpMediaConstraints);
  }

  public void createAnswer() {
    isInitiator = false;
    pc.createAnswer(sdpObserver, sdpMediaConstraints);
  }


  public void addRemoteIceCandidate(IceCandidate candidate) {
    if (queuedRemoteCandidates != null) {
      queuedRemoteCandidates.add(candidate);
    } else {
      pc.addIceCandidate(candidate);
    }
  }

  public void setRemoteDescription(SessionDescription sdp) {
    SessionDescription sdpISAC = new SessionDescription(
        sdp.type, preferISAC(sdp.description));
    Log.d(TAG, "Set remote SDP");
    pc.setRemoteDescription(sdpObserver, sdpISAC);
  }

  public void stopVideoSource() {
    if (videoSource != null) {
      videoSource.stop();
      videoSourceStopped = true;
    }
  }

  public void startVideoSource() {
    if (videoSource != null && videoSourceStopped) {
      videoSource.restart();
      videoSourceStopped = false;
    }
  }

  public void close() {
    if (pc != null) {
      pc.dispose();
      pc = null;
    }
    if (videoSource != null) {
      videoSource.dispose();
      videoSource = null;
    }
    if (factory != null) {
      factory.dispose();
      factory = null;
    }
  }

  /**
   * SDP/ICE ready callbacks.
   */
  public static interface PeerConnectionEvents {
    /**
     * Callback fired once offer is created and local SDP is set.
     */
    public void onLocalDescription(final SessionDescription sdp);

    /**
     * Callback fired once local Ice candidate is generated.
     */
    public void onIceCandidate(final IceCandidate candidate);

    /**
     * Callback fired once connection is established (IceConnectionState is
     * CONNECTED).
     */
    public void onIceConnected();
  }

  // Cycle through likely device names for the camera and return the first
  // capturer that works, or crash if none do.
  private VideoCapturer getVideoCapturer(boolean useFrontFacing) {
    String[] cameraFacing = { "front", "back" };
    if (!useFrontFacing) {
      cameraFacing[0] = "back";
      cameraFacing[1] = "front";
    }
    for (String facing : cameraFacing) {
      int[] cameraIndex = { 0, 1 };
      int[] cameraOrientation = { 0, 90, 180, 270 };
      for (int index : cameraIndex) {
        for (int orientation : cameraOrientation) {
          String name = "Camera " + index + ", Facing " + facing +
              ", Orientation " + orientation;
          VideoCapturer capturer = VideoCapturer.create(name);
          if (capturer != null) {
            Log.d(TAG, "Using camera: " + name);
            return capturer;
          }
        }
      }
    }
    throw new RuntimeException("Failed to open capturer");
  }

  private VideoTrack createVideoTrack(boolean frontFacing) {
      VideoCapturer capturer = getVideoCapturer(frontFacing);
      if (videoSource != null) {
        videoSource.stop();
        videoSource.dispose();
      }

      videoSource = factory.createVideoSource(
          capturer, videoConstraints);
      String trackExtension = frontFacing ? "frontFacing" : "backFacing";
      VideoTrack videoTrack =
          factory.createVideoTrack("ARDAMSv0" + trackExtension, videoSource);
      videoTrack.addRenderer(new VideoRenderer(localRender));
      return videoTrack;
  }

  // Poor-man's assert(): die with |msg| unless |condition| is true.
  private static void abortUnless(boolean condition, String msg) {
    if (!condition) {
      throw new RuntimeException(msg);
    }
  }

  // Mangle SDP to prefer ISAC/16000 over any other audio codec.
  private static String preferISAC(String sdpDescription) {
    String[] lines = sdpDescription.split("\r\n");
    int mLineIndex = -1;
    String isac16kRtpMap = null;
    Pattern isac16kPattern =
        Pattern.compile("^a=rtpmap:(\\d+) ISAC/16000[\r]?$");
    for (int i = 0;
         (i < lines.length) && (mLineIndex == -1 || isac16kRtpMap == null);
         ++i) {
      if (lines[i].startsWith("m=audio ")) {
        mLineIndex = i;
        continue;
      }
      Matcher isac16kMatcher = isac16kPattern.matcher(lines[i]);
      if (isac16kMatcher.matches()) {
        isac16kRtpMap = isac16kMatcher.group(1);
        continue;
      }
    }
    if (mLineIndex == -1) {
      Log.d(TAG, "No m=audio line, so can't prefer iSAC");
      return sdpDescription;
    }
    if (isac16kRtpMap == null) {
      Log.d(TAG, "No ISAC/16000 line, so can't prefer iSAC");
      return sdpDescription;
    }
    String[] origMLineParts = lines[mLineIndex].split(" ");
    StringBuilder newMLine = new StringBuilder();
    int origPartIndex = 0;
    // Format is: m=<media> <port> <proto> <fmt> ...
    newMLine.append(origMLineParts[origPartIndex++]).append(" ");
    newMLine.append(origMLineParts[origPartIndex++]).append(" ");
    newMLine.append(origMLineParts[origPartIndex++]).append(" ");
    newMLine.append(isac16kRtpMap);
    for (; origPartIndex < origMLineParts.length; ++origPartIndex) {
      if (!origMLineParts[origPartIndex].equals(isac16kRtpMap)) {
        newMLine.append(" ").append(origMLineParts[origPartIndex]);
      }
    }
    lines[mLineIndex] = newMLine.toString();
    StringBuilder newSdpDescription = new StringBuilder();
    for (String line : lines) {
      newSdpDescription.append(line).append("\r\n");
    }
    return newSdpDescription.toString();
  }

  private void drainRemoteCandidates() {
    for (IceCandidate candidate : queuedRemoteCandidates) {
      pc.addIceCandidate(candidate);
    }
    queuedRemoteCandidates = null;
  }

  public void switchCamera() {
    if (videoConstraints == null)
      return;  // No video is sent.

    if (pc.signalingState() != PeerConnection.SignalingState.STABLE) {
      Log.e(TAG, "Switching camera during negotiation is not handled.");
      return;
    }

    pc.removeStream(videoMediaStream);
    VideoTrack currentTrack = videoMediaStream.videoTracks.get(0);
    videoMediaStream.removeTrack(currentTrack);

    String trackId = currentTrack.id();
    // On Android, there can only be one camera open at the time and we
    // need to release our implicit references to the videoSource before the
    // PeerConnectionFactory is released. Since createVideoTrack creates a new
    // videoSource and frees the old one, we need to release the track here.
    currentTrack.dispose();

    useFrontFacingCamera = !useFrontFacingCamera;
    VideoTrack newTrack = createVideoTrack(useFrontFacingCamera);
    videoMediaStream.addTrack(newTrack);
    pc.addStream(videoMediaStream, new MediaConstraints());

    SessionDescription remoteDesc = pc.getRemoteDescription();
    if (localSdp == null || remoteDesc == null) {
      Log.d(TAG, "Switching camera before the negotiation started.");
      return;
    }

    localSdp = new SessionDescription(localSdp.type,
        localSdp.description.replaceAll(trackId, newTrack.id()));

    if (isInitiator) {
      pc.setLocalDescription(new SwitchCameraSdbObserver(), localSdp);
      pc.setRemoteDescription(new SwitchCameraSdbObserver(), remoteDesc);
    } else {
      pc.setRemoteDescription(new SwitchCameraSdbObserver(), remoteDesc);
      pc.setLocalDescription(new SwitchCameraSdbObserver(), localSdp);
    }
  }

  // Implementation detail: observe ICE & stream changes and react accordingly.
  private class PCObserver implements PeerConnection.Observer {
    @Override
    public void onIceCandidate(final IceCandidate candidate){
      activity.runOnUiThread(new Runnable() {
          public void run() {
            events.onIceCandidate(candidate);
          }
        });
    }

    @Override
    public void onError(){
      activity.runOnUiThread(new Runnable() {
          public void run() {
            throw new RuntimeException("PeerConnection error!");
          }
        });
    }

    @Override
    public void onSignalingChange(
        PeerConnection.SignalingState newState) {
      Log.d(TAG, "SignalingState: " + newState);
    }

    @Override
    public void onIceConnectionChange(
        PeerConnection.IceConnectionState newState) {
      Log.d(TAG, "IceConnectionState: " + newState);
      if (newState == IceConnectionState.CONNECTED) {
        activity.runOnUiThread(new Runnable() {
          public void run() {
            events.onIceConnected();
          }
        });
      }
    }

    @Override
    public void onIceGatheringChange(
        PeerConnection.IceGatheringState newState) {
    }

    @Override
    public void onAddStream(final MediaStream stream){
      activity.runOnUiThread(new Runnable() {
          public void run() {
            abortUnless(stream.audioTracks.size() <= 1 &&
                stream.videoTracks.size() <= 1,
                "Weird-looking stream: " + stream);
            if (stream.videoTracks.size() == 1) {
              stream.videoTracks.get(0).addRenderer(
                  new VideoRenderer(remoteRender));
            }
          }
        });
    }

    @Override
    public void onRemoveStream(final MediaStream stream){
      activity.runOnUiThread(new Runnable() {
          public void run() {
            stream.videoTracks.get(0).dispose();
          }
        });
    }

    @Override
    public void onDataChannel(final DataChannel dc) {
      activity.runOnUiThread(new Runnable() {
          public void run() {
            throw new RuntimeException(
                "AppRTC doesn't use data channels, but got: " + dc.label() +
                " anyway!");
          }
        });
    }

    @Override
    public void onRenegotiationNeeded() {
      // No need to do anything; AppRTC follows a pre-agreed-upon
      // signaling/negotiation protocol.
    }
  }

  // Implementation detail: handle offer creation/signaling and answer setting,
  // as well as adding remote ICE candidates once the answer SDP is set.
  private class SDPObserver implements SdpObserver {
    @Override
    public void onCreateSuccess(final SessionDescription origSdp) {
      abortUnless(localSdp == null, "multiple SDP create?!?");
      final SessionDescription sdp = new SessionDescription(
          origSdp.type, preferISAC(origSdp.description));
      localSdp = sdp;
      activity.runOnUiThread(new Runnable() {
          public void run() {
            Log.d(TAG, "Set local SDP from " + sdp.type);
            pc.setLocalDescription(sdpObserver, sdp);
          }
        });
    }

    @Override
    public void onSetSuccess() {
      activity.runOnUiThread(new Runnable() {
          public void run() {
            if (isInitiator) {
              // For offering peer connection we first create offer and set
              // local SDP, then after receiving answer set remote SDP.
              if (pc.getRemoteDescription() == null) {
                // We've just set our local SDP so time to send it.
                Log.d(TAG, "Local SDP set succesfully");
                events.onLocalDescription(localSdp);
              } else {
                // We've just set remote description,
                // so drain remote ICE candidates.
                Log.d(TAG, "Remote SDP set succesfully");
                drainRemoteCandidates();
              }
            } else {
              // For answering peer connection we set remote SDP and then
              // create answer and set local SDP.
              if (pc.getLocalDescription() != null) {
                // We've just set our local SDP so time to send it and drain
                // remote ICE candidates.
                Log.d(TAG, "Local SDP set succesfully");
                events.onLocalDescription(localSdp);
                drainRemoteCandidates();
              } else {
                // We've just set remote SDP - do nothing for now -
                // answer will be created soon.
                Log.d(TAG, "Remote SDP set succesfully");
              }
            }
          }
        });
    }

    @Override
    public void onCreateFailure(final String error) {
      activity.runOnUiThread(new Runnable() {
          public void run() {
            throw new RuntimeException("createSDP error: " + error);
          }
        });
    }

    @Override
    public void onSetFailure(final String error) {
      activity.runOnUiThread(new Runnable() {
          public void run() {
            throw new RuntimeException("setSDP error: " + error);
          }
        });
    }
  }

  private class SwitchCameraSdbObserver implements SdpObserver {
    @Override
    public void onCreateSuccess(SessionDescription sdp) {
    }

    @Override
    public void onSetSuccess() {
      Log.d(TAG, "Camera switch SDP set succesfully");
    }

    @Override
    public void onCreateFailure(final String error) {
    }

    @Override
    public void onSetFailure(final String error) {
      activity.runOnUiThread(new Runnable() {
        public void run() {
          throw new RuntimeException("setSDP error while switching camera: " + error);
        }
      });
    }
  }
}
