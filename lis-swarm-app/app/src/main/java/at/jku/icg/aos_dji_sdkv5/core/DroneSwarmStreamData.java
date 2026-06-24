package at.jku.icg.aos_dji_sdkv5.core;

import android.media.MediaCodec;
import android.media.MediaCrypto;
import android.media.MediaFormat;
import android.os.Handler;
import android.os.HandlerThread;
import android.os.Message;
import android.util.Log;
import android.view.Surface;
import at.jku.icg.aos_dji_sdkv5.dji.DJIManager;
import dji.sdk.keyvalue.key.CameraKey;
import dji.sdk.keyvalue.key.FlightControllerKey;
import dji.sdk.keyvalue.key.GimbalKey;
import dji.sdk.keyvalue.key.KeyTools;
import dji.sdk.keyvalue.value.camera.CameraVideoStreamSourceType;
import dji.sdk.keyvalue.value.camera.VideoFrameRate;
import dji.sdk.keyvalue.value.camera.VideoResolutionFrameRate;
import dji.sdk.keyvalue.value.common.Attitude;
import dji.sdk.keyvalue.value.common.CameraLensType;
import dji.sdk.keyvalue.value.common.ComponentIndexType;
import dji.sdk.keyvalue.value.common.LocationCoordinate3D;
import dji.sdk.keyvalue.value.common.Velocity3D;
import dji.v5.common.callback.CommonCallbacks;
import dji.v5.common.error.IDJIError;
import dji.v5.common.video.channel.VideoChannelType;
import dji.v5.common.video.interfaces.IVideoChannel;
import dji.v5.common.video.interfaces.IVideoFrame;
import dji.v5.common.video.interfaces.StreamDataListener;
import dji.v5.common.video.stream.VideoStreamFormat;
import dji.v5.manager.KeyManager;
import dji.v5.manager.aircraft.rtk.RTKCenter;
import dji.v5.manager.aircraft.rtk.RTKLocationInfo;
import dji.v5.manager.aircraft.rtk.RTKLocationInfoListener;
import dji.v5.manager.datacenter.MediaDataCenter;
import io.moquette.BrokerConstants;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.util.LinkedList;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;

/* JADX INFO: loaded from: classes.dex */
public class DroneSwarmStreamData implements StreamDataListener {
    static final /* synthetic */ boolean $assertionsDisabled = false;
    private static final int BUF_QUEUE_SIZE = 30;
    private static final String H264 = "video/avc";
    private static final String HEVC = "video/hevc";
    private static final int MSG_DECODE_FRAME = 2;
    private static final int MSG_FRAME_QUEUE_IN = 1;
    private static final int MSG_INIT_CODEC = 0;
    private static final int MSG_PACKET_2_PARSER = 4;
    private static final int MSG_TELEMETRY_DATA = 5;
    private static final int MSG_YUV_DATA = 3;
    public static final String TAG;
    private static YuvDecoder dec;
    public boolean EnableRtspStream;
    public AOSManager aosManager;
    private MediaCodec codec;
    private Handler dataHandler;
    private HandlerThread dataHandlerThread;
    private final DJIManager djiMan;
    private int frameRate;
    private Handler handlerNew;
    private HandlerThread handlerThreadNew;
    private int height;
    private int iFrameRate;
    public MQTTEmbedded mqttEmbedded;
    private IVideoChannel primarychannel;
    private VideoResolutionFrameRate rat;
    private int resWH;
    private Surface surface;
    private VideoResolutionFrameRate vidResFrameRate;
    private int width;
    private Attitude aircraft_attitude = null;
    private LocationCoordinate3D curr_loc = null;
    private LocationCoordinate3D curr_loc_rtk = null;
    private LocationCoordinate3D curr_loc_rtk1 = null;
    private boolean hasIFrameInQueue = false;
    MediaCodec.BufferInfo bufferInfo = new MediaCodec.BufferInfo();
    LinkedList<Long> bufferChangedQueue = new LinkedList<>();
    private Double curr_compass_rtk = Double.valueOf(0.0d);
    private Boolean rtk_mode = false;
    private boolean gotPackage = false;
    private boolean once = true;
    private long frameIndex = 0;
    private long rtspFramesFed = 0;
    private long parseCount = 0;
    private double curr_lat_dj = 0.0d;
    private double curr_lon_dj = 0.0d;
    private double curr_alt_dj = 0.0d;
    private double curr_comp = 0.0d;
    private double curr_tilt = 0.0d;
    private double gimbal_pan = 0.0d;
    private double gimbal_yaw = 0.0d;
    private double drone_pitch = 0.0d;
    private double drone_roll = 0.0d;
    private double drone_yaw = 0.0d;
    private double velocity_x = 0.0d;
    private double velocity_y = 0.0d;
    private double velocity_z = 0.0d;
    private Integer satelliteCount = 0;
    private Integer targetWayPointDone = 0;
    private Integer targetWayPointDoneset = 0;
    private Integer withlimit = 0;
    private Integer counter = 0;
    private Integer receivedwaydata = 0;
    public double targetWayPointDonelat = 0.0d;
    public double targetWayPointDonelong = 0.0d;
    public double virtualstickonoff = 0.0d;
    public double setvaluetocheck = 0.0d;
    private RTKLocationInfoListener live_rtkLocationInfoListener1 = new RTKLocationInfoListener() { // from class: at.jku.icg.aos_dji_sdkv5.core.DroneSwarmStreamData.6
        /* JADX DEBUG: Method merged with bridge method: onUpdate(Ljava/lang/Object;)V */
        public void onUpdate(RTKLocationInfo rTKLocationInfo) {
            if (rTKLocationInfo != null) {
                if (rTKLocationInfo.getReal3DLocation() != null && DroneSwarmStreamData.this.rtk_mode.booleanValue()) {
                    DroneSwarmStreamData.this.curr_loc_rtk = rTKLocationInfo.getRtkLocation().getMobileStationLocation();
                    DroneSwarmStreamData.this.curr_loc_rtk1 = rTKLocationInfo.getReal3DLocation();
                }
                if (rTKLocationInfo.getRealHeading() == null || !DroneSwarmStreamData.this.rtk_mode.booleanValue()) {
                    return;
                }
                KeyManager.getInstance().getValue(KeyTools.createKey(FlightControllerKey.KeyAircraftAttitude), new CommonCallbacks.CompletionCallbackWithParam<Attitude>() { // from class: at.jku.icg.aos_dji_sdkv5.core.DroneSwarmStreamData.6.1
                    public void onFailure(IDJIError iDJIError) {
                    }

                    /* JADX DEBUG: Method merged with bridge method: onSuccess(Ljava/lang/Object;)V */
                    public void onSuccess(Attitude attitude) {
                        DroneSwarmStreamData.this.aircraft_attitude = attitude;
                        DroneSwarmStreamData.this.curr_compass_rtk = DroneSwarmStreamData.this.aircraft_attitude.getYaw();
                    }
                });
            }
        }
    };
    private long sessionId = 0;
    public boolean RtspIsRunning = false;
    private final Message pkg_in = Message.obtain();
    private long createTime = System.currentTimeMillis();
    private Queue<DJIFrame> frameQueue = new ArrayBlockingQueue(30);

    private native String codecinfotest();

    private native void feedFrame(byte[] bArr, int i, int i2, int i3, long j);

    private native boolean init();

    /* JADX INFO: Access modifiers changed from: private */
    public native boolean parse(byte[] bArr, int i);

    private native boolean release();

    public static native void setTelemetryData(double d, double d2, double d3, double d4, double d5, double d6, double d7, int i, double d8, double d9, double d10, double d11, double d12, double d13, int i2, double d14, double d15);

    private static native long startRTSPServer(String str, int i, String str2, String str3, int i2);

    private static native boolean stopRTSPServer(long j);

    static {
        System.loadLibrary("RtspServer");
        System.loadLibrary("ffmpeg_ext");
        TAG = "DroneSwarmStreamData";
    }

    public DroneSwarmStreamData(DJIManager dJIManager) {
        this.djiMan = dJIManager;
        startDataHandler();
        this.width = 1920;
        this.height = 1080;
        initCodec();
        HandlerThread handlerThread = new HandlerThread("Feed packages to parser thread");
        this.handlerThreadNew = handlerThread;
        handlerThread.start();
        this.handlerNew = new Handler(this.handlerThreadNew.getLooper(), new Handler.Callback() { // from class: at.jku.icg.aos_dji_sdkv5.core.DroneSwarmStreamData.1
            @Override // android.os.Handler.Callback
            public boolean handleMessage(Message message) {
                int i = message.what;
                if (i == 4) {
                    DroneSwarmStreamData.this.parseCount++;
                    if (DroneSwarmStreamData.this.parseCount <= 5 || DroneSwarmStreamData.this.parseCount % 300 == 0) {
                        Log.v(TAG, "parse() call #" + DroneSwarmStreamData.this.parseCount
                                + " size=" + message.arg1);
                    }
                    DroneSwarmStreamData.this.parse((byte[]) message.obj, message.arg1);
                    return false;
                }
                if (i != 5) {
                    return false;
                }
                DroneSwarmStreamData.this.getTelemetryData();
                return false;
            }
        });
    }

    public void onReceive(IVideoFrame iVideoFrame) {
        byte[] data = iVideoFrame.getData();
        if (!this.gotPackage) {
            String nalInfo = inspectNalUnits(data);
            Log.v(TAG, "Got first frame, w=" + iVideoFrame.getWidth() + " h=" + iVideoFrame.getHeight()
                    + " fps=" + iVideoFrame.getFps() + " sequenceNr=" + iVideoFrame.getSeqNumber()
                    + " dataLen=" + data.length + " NAL=" + nalInfo);
            this.gotPackage = true;
        }
        // Log whenever VPS/SPS/PPS (HEVC types 32,33,34) or IDR (19,20) appear
        if (containsHevcParamSets(data)) {
            Log.v(TAG, ">>> VPS/SPS/PPS or IDR detected in frame! len=" + data.length
                    + " NAL=" + inspectNalUnits(data));
        }
        this.width = iVideoFrame.getWidth();
        this.height = iVideoFrame.getHeight();
        Message messageObtain = Message.obtain();
        messageObtain.what = 4;
        messageObtain.obj = data;
        messageObtain.arg1 = data.length;
        this.handlerNew.sendMessage(messageObtain);
        Message messageObtain2 = Message.obtain();
        messageObtain2.what = 5;
        this.handlerNew.sendMessage(messageObtain2);
        // Feed raw SDK data directly to RTSP server (matches old app exactly)
        if (this.RtspIsRunning && this.EnableRtspStream) {
            feedFrame(iVideoFrame.getData(), iVideoFrame.getWidth(), iVideoFrame.getHeight(), iVideoFrame.getFps(), iVideoFrame.getSeqNumber());
            this.rtspFramesFed++;
            if (this.rtspFramesFed == 1 || this.rtspFramesFed % 300 == 0) {
                Log.v(TAG, "RTSP feedFrame #" + this.rtspFramesFed
                        + " size=" + data.length + " w=" + iVideoFrame.getWidth()
                        + " h=" + iVideoFrame.getHeight() + " fps=" + iVideoFrame.getFps()
                        + " seq=" + iVideoFrame.getSeqNumber());
            }
        }
    }

    /** Returns the number of frames fed to the RTSP server so far. */
    public long getRtspFramesFed() {
        return this.rtspFramesFed;
    }

    /**
     * Inspects raw frame data to identify NAL unit types and determine actual codec.
     * H264 NAL types: 1=non-IDR, 5=IDR, 7=SPS, 8=PPS
     * HEVC NAL types: 32=VPS, 33=SPS, 34=PPS, 19=IDR_W_RADL, 20=IDR_N_LP
     */
    private String inspectNalUnits(byte[] data) {
        if (data == null || data.length < 5) return "null/short";
        StringBuilder sb = new StringBuilder();
        // Log first 16 bytes as hex
        sb.append("hex[");
        for (int i = 0; i < Math.min(16, data.length); i++) {
            sb.append(String.format("%02x", data[i] & 0xFF));
            if (i < Math.min(15, data.length - 1)) sb.append(" ");
        }
        sb.append("] ");
        // Find NAL start codes and identify types
        int nalCount = 0;
        for (int i = 0; i < data.length - 4 && nalCount < 5; i++) {
            boolean is4byte = (data[i] == 0 && data[i+1] == 0 && data[i+2] == 0 && data[i+3] == 1);
            boolean is3byte = (data[i] == 0 && data[i+1] == 0 && data[i+2] == 1);
            if (is4byte || is3byte) {
                int nalByte = data[is4byte ? i+4 : i+3] & 0xFF;
                // H264: type = nalByte & 0x1F
                int h264type = nalByte & 0x1F;
                // HEVC: type = (nalByte >> 1) & 0x3F
                int hevctype = (nalByte >> 1) & 0x3F;
                sb.append("@").append(i).append(":");
                sb.append("h264t=").append(h264type);
                sb.append("/hevct=").append(hevctype);
                sb.append(" ");
                nalCount++;
                i += (is4byte ? 4 : 3);
            }
        }
        if (nalCount == 0) sb.append("NO_NAL_START_CODES");
        return sb.toString();
    }

    /** Returns true if the data contains HEVC VPS(32), SPS(33), PPS(34), or IDR(19,20) NAL units. */
    private boolean containsHevcParamSets(byte[] data) {
        if (data == null || data.length < 5) return false;
        for (int i = 0; i < data.length - 4; i++) {
            boolean is4byte = (data[i] == 0 && data[i+1] == 0 && data[i+2] == 0 && data[i+3] == 1);
            boolean is3byte = !is4byte && (data[i] == 0 && data[i+1] == 0 && data[i+2] == 1);
            if (is4byte || is3byte) {
                int nalByte = data[is4byte ? i+4 : i+3] & 0xFF;
                int hevcType = (nalByte >> 1) & 0x3F;
                if (hevcType == 32 || hevcType == 33 || hevcType == 34 || hevcType == 19 || hevcType == 20) {
                    return true;
                }
            }
        }
        return false;
    }

    private static class DJIFrame {
        public long codecOutputTime;
        public long fedIntoCodecTime;
        public long frameIndex;
        public int frameNum;
        public int height;
        public long incomingTimeMs;
        public boolean isKeyFrame;
        public long pts;
        public int size;
        public byte[] videoBuffer;
        public int width;

        public DJIFrame(byte[] bArr, int i, long j, long j2, boolean z, int i2, long j3, int i3, int i4) {
            this.videoBuffer = bArr;
            this.size = i;
            this.pts = j;
            this.incomingTimeMs = j2;
            this.isKeyFrame = z;
            this.frameNum = i2;
            this.frameIndex = j3;
            this.width = i3;
            this.height = i4;
        }

        public long getQueueDelay() {
            return this.fedIntoCodecTime - this.incomingTimeMs;
        }

        public long getDecodingDelay() {
            return this.codecOutputTime - this.fedIntoCodecTime;
        }

        public long getTotalDelay() {
            return this.codecOutputTime - this.fedIntoCodecTime;
        }
    }

    /* JADX INFO: Access modifiers changed from: private */
    public void initCodec() {
        if (this.width == 0 || this.height == 0) {
            return;
        }
        if (this.codec != null) {
            releaseCodec();
        }
        String str = TAG;
        Log.v(str, "initVideoDecoder----------------------------------------------------------");
        Log.v(str, "initVideoDecoder video width = " + this.width + "  height = " + this.height);
        MediaFormat mediaFormatCreateVideoFormat = MediaFormat.createVideoFormat(HEVC, this.width, this.height);
        Log.v(str, "initVideoDecoder: video/hevc yuv output");
        mediaFormatCreateVideoFormat.setInteger("color-format", 21);
        try {
            this.codec = MediaCodec.createDecoderByType(HEVC);
            StringBuilder sb = new StringBuilder();
            sb.append("initVideoDecoder create: ");
            sb.append(this.codec != null);
            Log.v(str, sb.toString());
            this.codec.configure(mediaFormatCreateVideoFormat, (Surface) null, (MediaCrypto) null, 0);
            Log.v(str, "initVideoDecoder configure");
            MediaCodec mediaCodec = this.codec;
            if (mediaCodec == null) {
                Log.v(str, "Can't find video info!");
            } else {
                mediaCodec.start();
            }
        } catch (Exception e) {
            Log.v(TAG, "init codec failed, do it again: " + e);
            e.printStackTrace();
        }
    }

    private void releaseCodec() {
        Queue<DJIFrame> queue = this.frameQueue;
        if (queue != null) {
            queue.clear();
            this.hasIFrameInQueue = false;
        }
        MediaCodec mediaCodec = this.codec;
        if (mediaCodec != null) {
            try {
                mediaCodec.flush();
            } catch (Exception e) {
                Log.v(TAG, "flush codec error: " + e.getMessage());
            }
            try {
                try {
                    this.codec.stop();
                    this.codec.release();
                } catch (Exception e2) {
                    Log.v(TAG, "close codec error: " + e2.getMessage());
                }
            } finally {
                this.codec = null;
            }
        }
    }

    private void startDataHandler() {
        HandlerThread handlerThread = this.dataHandlerThread;
        if (handlerThread == null || !handlerThread.isAlive()) {
            HandlerThread handlerThread2 = new HandlerThread("Frame data after parser handler thread");
            this.dataHandlerThread = handlerThread2;
            handlerThread2.start();
            this.dataHandler = new Handler(this.dataHandlerThread.getLooper()) { // from class: at.jku.icg.aos_dji_sdkv5.core.DroneSwarmStreamData.2
                /* JADX DEBUG: Another duplicated slice has different insns count: {[IGET, INVOKE, INVOKE]}, finally: {[IGET, INVOKE, INVOKE, INVOKE, IF] complete} */
                @Override // android.os.Handler
                public void handleMessage(Message message) {
                    int i = message.what;
                    if (i == 0) {
                        try {
                            DroneSwarmStreamData.this.initCodec();
                        } catch (Exception e) {
                            Log.v(DroneSwarmStreamData.TAG, "init codec error: " + e.getMessage());
                            e.printStackTrace();
                        }
                        Log.v(DroneSwarmStreamData.TAG, "MSG_INIT_CODEC");
                        removeCallbacksAndMessages(null);
                        sendEmptyMessageDelayed(2, 1L);
                        return;
                    }
                    if (i != 1) {
                        try {
                            if (i != 2) {
                                if (i != 3) {
                                    return;
                                }
                                DroneSwarmStreamData.this.aosManager.getYuvDecoder().onReceive(DroneSwarmStreamData.this.codec.getOutputFormat().getInteger("color-format"), (byte[]) message.obj, DroneSwarmStreamData.this.width, DroneSwarmStreamData.this.height);
                                return;
                            }
                            try {
                                DroneSwarmStreamData.this.decodeFrame();
                                if (DroneSwarmStreamData.this.frameQueue.size() <= 0) {
                                    return;
                                }
                            } catch (Exception e2) {
                                Log.v(DroneSwarmStreamData.TAG, "handle frame error: " + e2);
                                if (e2 instanceof MediaCodec.CodecException) {
                                    e2.printStackTrace();
                                }
                                DroneSwarmStreamData.this.initCodec();
                                if (DroneSwarmStreamData.this.frameQueue.size() <= 0) {
                                    return;
                                }
                            }
                            sendEmptyMessage(2);
                            return;
                        } catch (Throwable th) {
                            if (DroneSwarmStreamData.this.frameQueue.size() > 0) {
                                sendEmptyMessage(2);
                            }
                            throw th;
                        }
                    }
                    try {
                        DroneSwarmStreamData.this.onFrameQueueIn(message);
                    } catch (Exception e3) {
                        Log.v(DroneSwarmStreamData.TAG, "queue in frame error: " + e3);
                        e3.printStackTrace();
                    }
                    if (hasMessages(2)) {
                        return;
                    }
                    sendEmptyMessage(2);
                }
            };
        }
    }

    /* JADX INFO: Access modifiers changed from: private */
    public void onFrameQueueIn(Message message) {
        DJIFrame dJIFrame = (DJIFrame) message.obj;
        if (dJIFrame == null) {
            return;
        }
        if (!this.hasIFrameInQueue) {
            if (dJIFrame.frameNum != 1 && !dJIFrame.isKeyFrame) {
                Log.v(TAG, "the timing for setting iframe has not yet come.");
                return;
            }
            if (dJIFrame != null) {
                DJIFrame dJIFrame2 = new DJIFrame(dJIFrame.videoBuffer, dJIFrame.videoBuffer.length, dJIFrame.pts, System.currentTimeMillis(), dJIFrame.isKeyFrame, 0, dJIFrame.frameIndex - 1, dJIFrame.width, dJIFrame.height);
                this.frameQueue.clear();
                this.frameQueue.offer(dJIFrame2);
                Log.v(TAG, "add iframe success!!!!");
                this.hasIFrameInQueue = true;
            } else if (dJIFrame.isKeyFrame) {
                Log.v(TAG, "onFrameQueueIn no need add i frame!!!!");
                this.hasIFrameInQueue = true;
            } else {
                Log.v(TAG, "input key frame failed");
            }
        }
        if (dJIFrame.width != 0 && dJIFrame.height != 0 && (dJIFrame.width != this.width || dJIFrame.height != this.height)) {
            this.width = dJIFrame.width;
            this.height = dJIFrame.height;
            Log.v(TAG, "init decoder for the 1st time or when resolution changes");
            Handler handler = this.dataHandler;
            if (handler != null && !handler.hasMessages(0)) {
                this.dataHandler.sendEmptyMessage(0);
            }
        }
        if (this.frameQueue.offer(dJIFrame)) {
            return;
        }
        DJIFrame dJIFramePoll = this.frameQueue.poll();
        this.frameQueue.offer(dJIFrame);
        Log.v(TAG, "Drop a frame with index=" + dJIFramePoll.frameIndex + " and append a frame with index=" + dJIFrame.frameIndex);
    }

    /* JADX INFO: Access modifiers changed from: private */
    public void decodeFrame() throws Exception {
        DJIFrame dJIFramePoll = this.frameQueue.poll();
        if (dJIFramePoll == null) {
            return;
        }
        MediaCodec mediaCodec = this.codec;
        if (mediaCodec == null) {
            Handler handler = this.dataHandler;
            if (handler == null || handler.hasMessages(0)) {
                return;
            }
            this.dataHandler.sendEmptyMessage(0);
            return;
        }
        int iDequeueInputBuffer = mediaCodec.dequeueInputBuffer(0L);
        if (iDequeueInputBuffer >= 0) {
            this.codec.getInputBuffer(iDequeueInputBuffer).put(dJIFramePoll.videoBuffer);
            dJIFramePoll.fedIntoCodecTime = System.currentTimeMillis();
            dJIFramePoll.getQueueDelay();
            this.codec.queueInputBuffer(iDequeueInputBuffer, 0, dJIFramePoll.size, dJIFramePoll.pts, 0);
            int iDequeueOutputBuffer = this.codec.dequeueOutputBuffer(this.bufferInfo, 0L);
            if (iDequeueOutputBuffer >= 0) {
                if (this.surface == null) {
                    ByteBuffer outputBuffer = this.codec.getOutputBuffer(iDequeueOutputBuffer);
                    if (outputBuffer == null) {
                        Log.d(TAG, "decodeFrame: yuvDataBuf was null");
                    }
                    outputBuffer.position(this.bufferInfo.offset);
                    outputBuffer.limit(this.bufferInfo.size - this.bufferInfo.offset);
                    if (this.bufferInfo.size > 0) {
                        byte[] bArr = new byte[outputBuffer.limit()];
                        outputBuffer.get(bArr);
                        Message messageObtain = Message.obtain();
                        messageObtain.what = 3;
                        messageObtain.obj = bArr;
                        messageObtain.arg1 = this.bufferInfo.size - this.bufferInfo.offset;
                        this.dataHandler.sendMessage(messageObtain);
                    }
                }
                this.codec.releaseOutputBuffer(iDequeueOutputBuffer, true);
                return;
            }
            if (iDequeueOutputBuffer != -3) {
                if (iDequeueOutputBuffer == -2) {
                    Log.v(TAG, "format changed, color: " + this.codec.getOutputFormat().getInteger("color-format"));
                    return;
                }
                return;
            }
            long jCurrentTimeMillis = System.currentTimeMillis();
            this.bufferChangedQueue.addLast(Long.valueOf(jCurrentTimeMillis));
            if (this.bufferChangedQueue.size() < 10 || jCurrentTimeMillis - this.bufferChangedQueue.pollFirst().longValue() >= 1000) {
                return;
            }
            Log.e(TAG, "Reset decoder. Get INFO_OUTPUT_BUFFERS_CHANGED more than 10 times within a second.");
            this.bufferChangedQueue.clear();
            this.dataHandler.removeCallbacksAndMessages(null);
            this.dataHandler.sendEmptyMessage(0);
            return;
        }
        this.codec.flush();
    }

    public void getVideoFramerate() {
        KeyManager.getInstance().getValue(KeyTools.createCameraKey(CameraKey.KeyVideoResolutionFrameRate, ComponentIndexType.LEFT_OR_MAIN, CameraLensType.CAMERA_LENS_WIDE), new CommonCallbacks.CompletionCallbackWithParam<VideoResolutionFrameRate>() { // from class: at.jku.icg.aos_dji_sdkv5.core.DroneSwarmStreamData.3
            /* JADX DEBUG: Method merged with bridge method: onSuccess(Ljava/lang/Object;)V */
            public void onSuccess(VideoResolutionFrameRate videoResolutionFrameRate) {
                DroneSwarmStreamData.this.iFrameRate = Integer.parseInt(videoResolutionFrameRate.getFrameRate().name().replaceAll("[^-?0-9]+", ""));
                DroneSwarmStreamData.this.vidResFrameRate = videoResolutionFrameRate;
                Log.d(DroneSwarmStreamData.TAG, "Initial Camera FrameRate=" + DroneSwarmStreamData.this.iFrameRate + "fps and " + videoResolutionFrameRate.getResolution().name());
            }

            public void onFailure(IDJIError iDJIError) {
                Log.d(DroneSwarmStreamData.TAG, "Get Camera FrameRate error=" + iDJIError.description());
            }
        });
    }

    public void setVideoResolution(VideoResolutionFrameRate videoResolutionFrameRate) {
        KeyManager.getInstance().setValue(KeyTools.createCameraKey(CameraKey.KeyVideoResolutionFrameRate, ComponentIndexType.LEFT_OR_MAIN, CameraLensType.CAMERA_LENS_WIDE), videoResolutionFrameRate, new CommonCallbacks.CompletionCallback() { // from class: at.jku.icg.aos_dji_sdkv5.core.DroneSwarmStreamData.4
            public void onSuccess() {
                Log.d(DroneSwarmStreamData.TAG, "Successfully set video resolution");
            }

            public void onFailure(IDJIError iDJIError) {
                Log.d(DroneSwarmStreamData.TAG, "Camera resolution switch didn't work=" + iDJIError.description());
            }
        });
    }

    public boolean setCamera(CameraVideoStreamSourceType cameraVideoStreamSourceType) {
        KeyManager.getInstance().setValue(KeyTools.createKey(CameraKey.KeyCameraVideoStreamSource), cameraVideoStreamSourceType, new CommonCallbacks.CompletionCallback() { // from class: at.jku.icg.aos_dji_sdkv5.core.DroneSwarmStreamData.5
            public void onFailure(IDJIError iDJIError) {
            }

            public void onSuccess() {
                Log.d(DroneSwarmStreamData.TAG, "CAM Setting successfully applied");
            }
        });
        return true;
    }

    public void setCameraDewarping(boolean z, ComponentIndexType componentIndexType, CameraLensType cameraLensType) {
        KeyManager.getInstance().setValue(KeyTools.createCameraKey(CameraKey.KeyDewarpingEnabled, componentIndexType, cameraLensType), Boolean.valueOf(z), (CommonCallbacks.CompletionCallback) null);
    }

    public void switchCameraResolution() throws InterruptedException {
        VideoFrameRate videoFrameRate;
        getVideoFramerate();
        Thread.sleep(200L, 0);
        switch (AnonymousClass13.$SwitchMap$dji$sdk$keyvalue$value$camera$VideoFrameRate[this.vidResFrameRate.getFrameRate().ordinal()]) {
            case 1:
            case 2:
                videoFrameRate = VideoFrameRate.RATE_25FPS;
                break;
            case 3:
                videoFrameRate = VideoFrameRate.RATE_24FPS;
                break;
            case 4:
                videoFrameRate = VideoFrameRate.RATE_50FPS;
                break;
            case 5:
                videoFrameRate = VideoFrameRate.RATE_48FPS;
                break;
            case 6:
                videoFrameRate = VideoFrameRate.RATE_50FPS;
                break;
            default:
                videoFrameRate = VideoFrameRate.RATE_30FPS;
                break;
        }
        String str = TAG;
        Log.d(str, "Switch camera to " + videoFrameRate.name().replaceAll("[^-?0-9]+", "") + "fps");
        setVideoResolution(new VideoResolutionFrameRate(this.vidResFrameRate.getResolution(), videoFrameRate));
        Thread.sleep(200L, 0);
        Log.d(str, "Switch camera back to " + this.iFrameRate + "fps");
        setVideoResolution(this.vidResFrameRate);
    }

    /* JADX INFO: renamed from: at.jku.icg.aos_dji_sdkv5.core.DroneSwarmStreamData$13, reason: invalid class name */
    static /* synthetic */ class AnonymousClass13 {
        static final /* synthetic */ int[] $SwitchMap$dji$sdk$keyvalue$value$camera$VideoFrameRate;

        static {
            int[] iArr = new int[VideoFrameRate.values().length];
            $SwitchMap$dji$sdk$keyvalue$value$camera$VideoFrameRate = iArr;
            try {
                iArr[VideoFrameRate.RATE_24FPS.ordinal()] = 1;
            } catch (NoSuchFieldError unused) {
            }
            try {
                $SwitchMap$dji$sdk$keyvalue$value$camera$VideoFrameRate[VideoFrameRate.RATE_30FPS.ordinal()] = 2;
            } catch (NoSuchFieldError unused2) {
            }
            try {
                $SwitchMap$dji$sdk$keyvalue$value$camera$VideoFrameRate[VideoFrameRate.RATE_25FPS.ordinal()] = 3;
            } catch (NoSuchFieldError unused3) {
            }
            try {
                $SwitchMap$dji$sdk$keyvalue$value$camera$VideoFrameRate[VideoFrameRate.RATE_48FPS.ordinal()] = 4;
            } catch (NoSuchFieldError unused4) {
            }
            try {
                $SwitchMap$dji$sdk$keyvalue$value$camera$VideoFrameRate[VideoFrameRate.RATE_50FPS.ordinal()] = 5;
            } catch (NoSuchFieldError unused5) {
            }
            try {
                $SwitchMap$dji$sdk$keyvalue$value$camera$VideoFrameRate[VideoFrameRate.RATE_60FPS.ordinal()] = 6;
            } catch (NoSuchFieldError unused6) {
            }
        }
    }

    public void onFrameDataRecv(byte[] bArr, long j, int i, int i2, boolean z, int i3, int i4) {
        this.frameIndex++;
        int i5 = i2 + 1;
        if (i3 > 0 || i4 > 0) {
            DJIFrame dJIFrame = new DJIFrame(bArr, i, j, System.currentTimeMillis(), z, i5 + 1, this.frameIndex, i3, i4);
            Message messageObtain = Message.obtain();
            messageObtain.what = 1;
            messageObtain.obj = dJIFrame;
            this.dataHandler.sendMessage(messageObtain);
            if (this.once) {
                Log.v(TAG, "Parsed paket size=" + i + " frameNum=" + i5 + " width x height=" + i3 + "x" + i4
                        + " isKey=" + z + " NAL=" + inspectNalUnits(bArr));
                this.once = false;
            }
        }
    }

    /**
     * Sets up the video channel and camera. Does NOT add the stream data listener
     * — that's deferred to {@link #startRtspServer()} to prevent parse() from
     * auto-initializing FFmpeg with the wrong HEVC flag before startRTSPServer()
     * has set is_HEVC=1.
     */
    public void start() throws InterruptedException, IOException {
        if (MediaDataCenter.getInstance().getVideoStreamManager() != null && MediaDataCenter.getInstance().getVideoStreamManager().getAvailableStreamSources() != null) {
            IVideoChannel availableVideoChannel = MediaDataCenter.getInstance().getVideoStreamManager().getAvailableVideoChannel(VideoChannelType.PRIMARY_STREAM_CHANNEL);
            if (setCamera(CameraVideoStreamSourceType.WIDE_CAMERA)) {
                Log.v(TAG, "Set Camera to wide OK");
            }
            setCameraDewarping(true, ComponentIndexType.LEFT_OR_MAIN, CameraLensType.CAMERA_LENS_WIDE);
            this.primarychannel = availableVideoChannel;
            if (availableVideoChannel.getVideoStreamFormat().value() == 0) {
                Log.v(TAG, "Stream is H264");
            } else {
                Log.v(TAG, "Stream is HEVC");
            }
        } else {
            Log.e(TAG, "VideoStreamManager or StreamSources is null — video channel NOT started");
        }
        Log.v(TAG, "Video channel ready. Press Start RTSP to begin streaming.");
    }

    /**
     * Matches the old app's exact working sequence:
     * startRTSPServer(HEVC) → init() → addStreamDataListener
     * This ensures init() sees is_HEVC=1 (set by startRTSPServer) before any
     * frames arrive, preventing parse() from auto-initializing with is_HEVC=0.
     * @return true if RTSP server started successfully
     */
    public boolean startRtspServer() {
        if (this.primarychannel == null) {
            Log.e(TAG, "startRtspServer: primarychannel is null — video channel not started");
            return false;
        }
        int streamFormat = this.primarychannel.getVideoStreamFormat().value();
        Log.v(TAG, "startRtspServer: format=" + (streamFormat == 0 ? "H264" : "HEVC")
                + " gotPackage=" + this.gotPackage
                + " width=" + this.width + " height=" + this.height);
        // Step 1: startRTSPServer — sets native is_HEVC=1
        this.sessionId = startRTSPServer(BrokerConstants.HOST, 8554, "video", "video", streamFormat);
        Log.v(TAG, "startRTSPServer returned sessionID=" + this.sessionId);
        if (this.sessionId != 0) {
            this.RtspIsRunning = true;
            // Don't enable feedFrame yet — let parse() process frames first
            // to populate native VPS/SPS/PPS state. Enable after a short delay.
            Log.v(TAG, "RTSP server running on 0.0.0.0:8554");
        } else {
            Log.e(TAG, "startRTSPServer FAILED (returned 0)");
            return false;
        }
        // Step 2: init() — reads is_HEVC=1, sets up HEVC parser
        init();
        Log.v(TAG, "Native init() complete (HEVC mode)");
        // Step 3: add stream listener — frames flow to parse()
        this.primarychannel.addStreamDataListener(this);
        Log.v(TAG, "Stream data listener added — frames flowing to parse()");
        // Enable feedFrame immediately
        this.EnableRtspStream = true;
        Log.v(TAG, "feedFrame enabled — RTSP streaming active");
        // Step 4: Switch camera resolution back and forth to force new IDR/VPS/SPS/PPS
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                Log.v(TAG, "Triggering resolution switch to force VPS/SPS/PPS...");
                switchCameraResolution();
            } catch (InterruptedException e) {
                Log.e(TAG, "Resolution switch interrupted: " + e.getMessage());
            }
        }).start();
        return true;
    }

    /** Enable feedFrame — call this after the PC client has connected to RTSP. */
    public void enableFeedFrame() {
        this.EnableRtspStream = true;
        Log.v(TAG, "EnableRtspStream=true — feedFrame() now active");
    }

    public void stop() {
        if (this.RtspIsRunning) {
            stopRTSPServer(this.sessionId);
        }
        if (this.mqttEmbedded != null) {
            this.mqttEmbedded.stop();
        }
        this.RtspIsRunning = false;
        this.EnableRtspStream = false;
    }

    protected void finalize() {
        stop();
        release();
        releaseCodec();
    }

    public void SetArriveOnTargetWaypoint(int i) {
        this.targetWayPointDone = Integer.valueOf(i);
    }

    public void SetArriveOnTargetWaypointvaluetocheck(double d) {
        this.setvaluetocheck = d;
    }

    public void SetArriveOnTargetWaypointvirtualstick(double d) {
        this.virtualstickonoff = d;
    }

    /* JADX INFO: Access modifiers changed from: private */
    public void getTelemetryData() {
        if (this.aosManager.isRunning()) {
            KeyManager.getInstance().getValue(KeyTools.createKey(GimbalKey.KeyGimbalAttitude), new CommonCallbacks.CompletionCallbackWithParam<Attitude>() { // from class: at.jku.icg.aos_dji_sdkv5.core.DroneSwarmStreamData.7
                public void onFailure(IDJIError iDJIError) {
                }

                /* JADX DEBUG: Method merged with bridge method: onSuccess(Ljava/lang/Object;)V */
                public void onSuccess(Attitude attitude) {
                    DroneSwarmStreamData.this.curr_tilt = attitude.getPitch().doubleValue();
                    DroneSwarmStreamData.this.gimbal_pan = attitude.getRoll().doubleValue();
                    DroneSwarmStreamData.this.gimbal_yaw = attitude.getYaw().doubleValue();
                }
            });
            KeyManager.getInstance().getValue(KeyTools.createKey(FlightControllerKey.KeyGPSSatelliteCount), new CommonCallbacks.CompletionCallbackWithParam<Integer>() { // from class: at.jku.icg.aos_dji_sdkv5.core.DroneSwarmStreamData.8
                public void onFailure(IDJIError iDJIError) {
                }

                /* JADX DEBUG: Method merged with bridge method: onSuccess(Ljava/lang/Object;)V */
                public void onSuccess(Integer num) {
                    DroneSwarmStreamData.this.satelliteCount = num;
                }
            });
            KeyManager.getInstance().getValue(KeyTools.createKey(FlightControllerKey.KeyAircraftAttitude), new CommonCallbacks.CompletionCallbackWithParam<Attitude>() { // from class: at.jku.icg.aos_dji_sdkv5.core.DroneSwarmStreamData.9
                public void onFailure(IDJIError iDJIError) {
                }

                /* JADX DEBUG: Method merged with bridge method: onSuccess(Ljava/lang/Object;)V */
                public void onSuccess(Attitude attitude) {
                    DroneSwarmStreamData.this.drone_pitch = attitude.getPitch().doubleValue();
                    DroneSwarmStreamData.this.drone_roll = attitude.getRoll().doubleValue();
                    DroneSwarmStreamData.this.drone_yaw = attitude.getYaw().doubleValue();
                }
            });
            KeyManager.getInstance().getValue(KeyTools.createKey(FlightControllerKey.KeyAircraftVelocity), new CommonCallbacks.CompletionCallbackWithParam<Velocity3D>() { // from class: at.jku.icg.aos_dji_sdkv5.core.DroneSwarmStreamData.10
                public void onFailure(IDJIError iDJIError) {
                }

                /* JADX DEBUG: Method merged with bridge method: onSuccess(Ljava/lang/Object;)V */
                public void onSuccess(Velocity3D velocity3D) {
                    DroneSwarmStreamData.this.velocity_x = velocity3D.getX().doubleValue();
                    DroneSwarmStreamData.this.velocity_y = velocity3D.getY().doubleValue();
                    DroneSwarmStreamData.this.velocity_z = velocity3D.getZ().doubleValue();
                }
            });
        }
        if (this.rtk_mode.booleanValue()) {
            this.curr_lat_dj = this.curr_loc_rtk.getLatitude().doubleValue();
            this.curr_lon_dj = this.curr_loc_rtk.getLongitude().doubleValue();
            this.curr_alt_dj = this.curr_loc_rtk1.getAltitude().doubleValue();
            this.curr_comp = this.curr_compass_rtk.doubleValue();
        } else {
            KeyManager.getInstance().getValue(KeyTools.createKey(FlightControllerKey.KeyAircraftLocation3D), new CommonCallbacks.CompletionCallbackWithParam<LocationCoordinate3D>() { // from class: at.jku.icg.aos_dji_sdkv5.core.DroneSwarmStreamData.11
                public void onFailure(IDJIError iDJIError) {
                }

                /* JADX DEBUG: Method merged with bridge method: onSuccess(Ljava/lang/Object;)V */
                public void onSuccess(LocationCoordinate3D locationCoordinate3D) {
                    DroneSwarmStreamData.this.curr_lat_dj = locationCoordinate3D.getLatitude().doubleValue();
                    DroneSwarmStreamData.this.curr_lon_dj = locationCoordinate3D.getLongitude().doubleValue();
                    DroneSwarmStreamData.this.curr_alt_dj = locationCoordinate3D.getAltitude().doubleValue();
                }
            });
            KeyManager.getInstance().getValue(KeyTools.createKey(FlightControllerKey.KeyCompassHeading), new CommonCallbacks.CompletionCallbackWithParam<Double>() { // from class: at.jku.icg.aos_dji_sdkv5.core.DroneSwarmStreamData.12
                public void onFailure(IDJIError iDJIError) {
                }

                /* JADX DEBUG: Method merged with bridge method: onSuccess(Ljava/lang/Object;)V */
                public void onSuccess(Double d) {
                    DroneSwarmStreamData.this.curr_comp = d.doubleValue();
                }
            });
        }
        setTelemetryData(this.curr_lat_dj, this.curr_lon_dj, this.curr_alt_dj, this.curr_comp, this.curr_tilt, this.gimbal_pan, this.gimbal_yaw, this.satelliteCount.intValue(), this.drone_pitch, this.drone_roll, this.drone_yaw, this.velocity_x, this.velocity_y, this.velocity_z, this.targetWayPointDone.intValue(), this.setvaluetocheck, this.virtualstickonoff);
    }

    public void setRtk_mode(boolean z) {
        this.rtk_mode = Boolean.valueOf(z);
    }

    public void addRTKLocationInfoListener() {
        RTKCenter.getInstance().addRTKLocationInfoListener(this.live_rtkLocationInfoListener1);
    }

    public void removeRTKLocationInfoListener() {
        RTKCenter.getInstance().removeRTKLocationInfoListener(this.live_rtkLocationInfoListener1);
    }
}
