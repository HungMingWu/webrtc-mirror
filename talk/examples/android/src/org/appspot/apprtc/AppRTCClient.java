/*
 * libjingle
 * Copyright 2013, Google Inc.
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

import org.webrtc.IceCandidate;
import org.webrtc.MediaConstraints;
import org.webrtc.PeerConnection;
import org.webrtc.SessionDescription;

import java.util.List;

public interface AppRTCClient {
  /**
   * Asynchronously connect to an AppRTC room URL, e.g.
   * https://apprtc.appspot.com/?r=NNN. Once connection is established
   * onConnectedToRoom() callback with room parameters is invoked.
   */
  public void connectToRoom(String url);

  /**
   * Send local SDP (offer or answer, depending on role) to the
   * other participant.
   */
  public void sendLocalDescription(final SessionDescription sdp);

  /**
   * Send Ice candidate to the other participant.
   */
  public void sendLocalIceCandidate(final IceCandidate candidate);

  /**
   * Disconnect from the channel.
   */
  public void disconnect();

  /**
   * Struct holding the signaling parameters of an AppRTC room.
   */
  public class AppRTCSignalingParameters {
    public final List<PeerConnection.IceServer> iceServers;
    public final boolean initiator;
    public final MediaConstraints pcConstraints;
    public final MediaConstraints videoConstraints;
    public final MediaConstraints audioConstraints;

    public AppRTCSignalingParameters(
        List<PeerConnection.IceServer> iceServers,
        boolean initiator, MediaConstraints pcConstraints,
        MediaConstraints videoConstraints, MediaConstraints audioConstraints) {
      this.iceServers = iceServers;
      this.initiator = initiator;
      this.pcConstraints = pcConstraints;
      this.videoConstraints = videoConstraints;
      this.audioConstraints = audioConstraints;
    }
  }

  /**
   * Signaling callbacks.
   */
  public static interface AppRTCSignalingEvents {
    /**
     * Callback fired once the room's signaling parameters
     * AppRTCSignalingParameters are extracted.
     */
    public void onConnectedToRoom(final AppRTCSignalingParameters params);

    /**
     * Callback fired once channel for signaling messages is opened and
     * ready to receive messages.
     */
    public void onChannelOpen();

    /**
     * Callback fired once remote SDP is received.
     */
    public void onRemoteDescription(final SessionDescription sdp);

    /**
     * Callback fired once remote Ice candidate is received.
     */
    public void onRemoteIceCandidate(final IceCandidate candidate);

    /**
     * Callback fired once channel is closed.
     */
    public void onChannelClose();

    /**
     * Callback fired once channel error happened.
     */
    public void onChannelError(int code, String description);
  }
}
