package at.jku.icg.aos_dji_sdkv5.core;

import android.content.res.AssetManager;
import android.view.Surface;

/* JADX INFO: loaded from: classes.dex */
public class NativeLib {
    public static native void aos_view_create(AssetManager assetManager);

    public static native void aos_view_destroy();

    public static native void aos_view_render();

    public static native void aos_view_resize(int i, int i2);

    public static native void clearImageFiles();

    public static native void cloneBackupImages();

    public static native void createUserStudyFiles();

    public static native float getAnomalyDetectionThreshold();

    public static native float getCompassCorrection();

    public static native float getFocalPlaneDistance();

    public static native float getFocalPlanePitch();

    public static native float getFocalPlaneRoll();

    public static native int getIntegrationWindow();

    public static native float getMinPoseDistance();

    public static native int getStereoModeDisparity();

    public static native int getStereoModeMaxDisparity();

    public static native int getStereoModeMaxPairShift();

    public static native int getStereoModeMaxSASize();

    public static native int getStereoModeMinDisparity();

    public static native int getStereoModeMinPairShift();

    public static native int getStereoModeMinSASize();

    public static native int getStereoModePairShift();

    public static native int getStereoModeSASize();

    public static native int getStereoSyntheticAperture();

    public static native float getcontrastdetectionThreshold();

    public static native float getcontrastthreshold();

    public static native float getinital_altitude();

    public static native float getmaxFocalPlaneDistance();

    public static native float getmaxFocalPlanePitch();

    public static native float getmaxFocalPlaneRoll();

    public static native float getmaxcompasscorrection();

    public static native float getminFocalPlaneDistance();

    public static native float getminFocalPlanePitch();

    public static native float getminFocalPlaneRoll();

    public static native float getmincompasscorrection();

    public static native float getmincontrasthreshold();

    public static native float getminrxthreshold();

    public static native float getrxthreshold();

    public static native void intializeagain(boolean z);

    public static native boolean isInitiaalize_set();

    public static native boolean isLRStereoMode();

    public static native boolean isShowAnomalyDetection();

    public static native boolean isShowStereoMode();

    public static native boolean is_add_frames();

    public static native boolean is_pinhole_perspective();

    public static native void live_surface_view_receive_imageinfo(Surface surface, int i, int i2, byte[] bArr, double d, double d2, double d3, double d4, double d5, boolean z, boolean z2, String str, int i3);

    public static native void live_surface_view_receive_imageinfo_withgimbal(Surface surface, int i, int i2, byte[] bArr, double d, double d2, double d3, double d4, double d5, double d6, double d7, boolean z, boolean z2, String str, int i3);

    public static native void process_images();

    public static native void render_pinhole_perspective(boolean z);

    public static native void setAnomalyDetectionThreshold(float f);

    public static native void setCompassCorrection(float f);

    public static native void setFocalPlaneDistance(float f);

    public static native void setFocalPlanePitch(float f);

    public static native void setFocalPlaneRoll(float f);

    public static native void setIntegrationWindow(int i);

    public static native void setLRStereoMode(boolean z);

    public static native void setLoadImagesFromFile(boolean z);

    public static native void setMinPoseDistance(float f);

    public static native void setShowCenterImage(boolean z);

    public static native void setStereoModeDisparity(int i);

    public static native void setStereoModePairShift(int i);

    public static native void setStereoModeSASize(int i);

    public static native void set_add_frames(boolean z);

    public static native void setcapture(boolean z);

    public static native void setcenterrenderimage(boolean z);

    public static native void setcontrastdetectionThreshold(float f);

    public static native void setmaxFocalPlaneDistance(float f);

    public static native void setmaxFocalPlanePitch(float f);

    public static native void setmaxFocalPlaneRoll(float f);

    public static native void setmaxcompasscorrection(float f);

    public static native void setmaxcontrastthreshold(float f);

    public static native void setmaxrxthreshold(float f);

    public static native void setminFocalPlaneDistance(float f);

    public static native void setminFocalPlanePitch(float f);

    public static native void setminFocalPlaneRoll(float f);

    public static native void setmincompasscorrection(float f);

    public static native void setmincontrastthreshold(float f);

    public static native void setminrxthreshold(float f);

    public static native void showAnomalyDetection(boolean z);

    public static native void showStereoMode(boolean z);

    public static native void switchbetintegralandsingleanomaly(boolean z);

    public static native void test();

    static {
        System.loadLibrary("lf_interface");
    }
}
