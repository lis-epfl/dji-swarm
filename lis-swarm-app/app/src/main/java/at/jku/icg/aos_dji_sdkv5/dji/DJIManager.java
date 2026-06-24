package at.jku.icg.aos_dji_sdkv5.dji;

import android.content.Context;
import at.jku.icg.aos_dji_sdkv5.core.DroneSwarmStreamData;
import dji.sdk.keyvalue.key.FlightControllerKey;
import dji.v5.common.callback.CommonCallbacks;
import dji.v5.common.error.IDJIError;
import dji.v5.common.video.channel.VideoChannelType;
import dji.v5.common.video.interfaces.IVideoChannel;
import dji.v5.common.video.interfaces.IVideoDecoder;
import dji.v5.common.video.stream.PhysicalDeviceCategory;
import dji.v5.common.video.stream.PhysicalDevicePosition;
import dji.v5.common.video.stream.StreamSource;
import dji.v5.manager.datacenter.MediaDataCenter;
import java.util.List;

/* JADX INFO: loaded from: classes.dex */
public class DJIManager {
    private CommonCallbacks.KeyListener baroListener;
    private FlightControllerKey barokey;
    private CommonCallbacks.KeyListener compassListener;
    private FlightControllerKey compasskey;
    public DroneSwarmStreamData droneSwarmStreamData;
    private CommonCallbacks.KeyListener locationListener;
    private FlightControllerKey locationkey;
    private CommonCallbacks.KeyListener pitchListener;
    private FlightControllerKey pitchkey;
    public IVideoChannel primarychannel;
    private CommonCallbacks.KeyListener rollListener;
    private FlightControllerKey rollkey;
    public List<StreamSource> sourcelist;
    private FlightControllerKey velocityxkey;
    private CommonCallbacks.KeyListener velocityxlistener;
    private FlightControllerKey velocityykey;
    private CommonCallbacks.KeyListener velocityylistener;
    private FlightControllerKey velocityzkey;
    private CommonCallbacks.KeyListener velocityzlistener;
    public IVideoDecoder videodecoder;
    private CommonCallbacks.KeyListener yawListener;
    private FlightControllerKey yawkey;
    private final int videoViewWidth = 1280;
    private final int videoViewHeight = 720;

    public static boolean isTranscodedVideoFeedNeeded() {
        return false;
    }

    public DJIManager(Context context) {
        List<StreamSource> list;
        if (MediaDataCenter.getInstance().getVideoStreamManager() == null || MediaDataCenter.getInstance().getVideoStreamManager().getAvailableStreamSources() == null) {
            return;
        }
        this.sourcelist = MediaDataCenter.getInstance().getVideoStreamManager().getAvailableStreamSources();
        IVideoChannel availableVideoChannel = MediaDataCenter.getInstance().getVideoStreamManager().getAvailableVideoChannel(VideoChannelType.PRIMARY_STREAM_CHANNEL);
        this.primarychannel = availableVideoChannel;
        if (availableVideoChannel == null || (list = this.sourcelist) == null) {
            return;
        }
        for (StreamSource streamSource : list) {
            if (streamSource.getPhysicalDeviceCategory() == PhysicalDeviceCategory.CAMERA && streamSource.getPhysicalDevicePosition() == PhysicalDevicePosition.DEFAULT) {
                this.primarychannel.startChannel(streamSource, new CommonCallbacks.CompletionCallback() { // from class: at.jku.icg.aos_dji_sdkv5.dji.DJIManager.1
                    public void onFailure(IDJIError iDJIError) {
                    }

                    public void onSuccess() {
                    }
                });
            }
        }
    }

    public void onDestroy() {
        IVideoChannel iVideoChannel = this.primarychannel;
        if (iVideoChannel != null) {
            iVideoChannel.closeChannel(new CommonCallbacks.CompletionCallback() { // from class: at.jku.icg.aos_dji_sdkv5.dji.DJIManager.2
                public void onFailure(IDJIError iDJIError) {
                }

                public void onSuccess() {
                }
            });
        }
        IVideoDecoder iVideoDecoder = this.videodecoder;
        if (iVideoDecoder != null) {
            iVideoDecoder.destroy();
            this.videodecoder = null;
        }
    }
}
