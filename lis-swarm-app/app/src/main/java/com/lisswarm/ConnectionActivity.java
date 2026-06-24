package com.lisswarm;

import android.app.Activity;
import android.content.Intent;
import android.os.AsyncTask;
import android.os.Build;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.TextView;

import androidx.core.app.ActivityCompat;
import androidx.core.content.ContextCompat;

import dji.sdk.keyvalue.key.FlightControllerKey;
import dji.sdk.keyvalue.key.KeyTools;
import dji.sdk.keyvalue.key.ProductKey;
import dji.sdk.keyvalue.value.product.ProductType;
import dji.v5.common.callback.CommonCallbacks;
import dji.v5.common.error.IDJIError;
import dji.v5.common.register.DJISDKInitEvent;
import dji.v5.manager.KeyManager;
import dji.v5.manager.SDKManager;
import dji.v5.manager.interfaces.SDKManagerCallback;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;

public class ConnectionActivity extends Activity implements View.OnClickListener {

    private static final String TAG = "LIS_Connection";
    private static final int REQUEST_PERMISSION_CODE = 12345;
    private static final String[] REQUIRED_PERMISSIONS = {
        "android.permission.BLUETOOTH",
        "android.permission.BLUETOOTH_ADMIN",
        "android.permission.VIBRATE",
        "android.permission.INTERNET",
        "android.permission.ACCESS_WIFI_STATE",
        "android.permission.ACCESS_COARSE_LOCATION",
        "android.permission.ACCESS_NETWORK_STATE",
        "android.permission.ACCESS_FINE_LOCATION",
        "android.permission.CHANGE_WIFI_STATE",
        "android.permission.WRITE_EXTERNAL_STORAGE",
        "android.permission.READ_EXTERNAL_STORAGE",
        "android.permission.READ_PHONE_STATE"
    };

    private TextView tvConnectionStatus;
    private TextView tvProduct;
    private TextView tvSdkVersion;
    private Button btnOpen;

    private List<String> missingPermissions = new ArrayList<>();
    private AtomicBoolean isRegistrationInProgress = new AtomicBoolean(false);
    private boolean productConnected = false;
    private boolean fcConnected = false;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_connection);
        initUI();
        checkAndRequestPermissions();
        if (missingPermissions.isEmpty()) {
            startSDKRegistration();
        }
    }

    private void initUI() {
        tvConnectionStatus = findViewById(R.id.tv_connection_status);
        tvProduct = findViewById(R.id.tv_product);
        tvSdkVersion = findViewById(R.id.tv_sdk_version);
        btnOpen = findViewById(R.id.btn_open);
        btnOpen.setOnClickListener(this);
        btnOpen.setEnabled(false);

        try {
            tvSdkVersion.setText("SDK: " + SDKManager.getInstance().getSDKVersion());
        } catch (Exception e) {
            tvSdkVersion.setText("SDK: initializing...");
        }
    }

    private void checkAndRequestPermissions() {
        for (String perm : REQUIRED_PERMISSIONS) {
            if (ContextCompat.checkSelfPermission(this, perm) != 0) {
                missingPermissions.add(perm);
            }
        }
        if (!missingPermissions.isEmpty() && Build.VERSION.SDK_INT >= 23) {
            ActivityCompat.requestPermissions(this,
                missingPermissions.toArray(new String[0]), REQUEST_PERMISSION_CODE);
        }
    }

    @Override
    public void onRequestPermissionsResult(int requestCode, String[] permissions, int[] grantResults) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults);
        if (requestCode == REQUEST_PERMISSION_CODE) {
            for (int i = grantResults.length - 1; i >= 0; i--) {
                if (grantResults[i] == 0) {
                    missingPermissions.remove(permissions[i]);
                }
            }
        }
        if (missingPermissions.isEmpty()) {
            startSDKRegistration();
        }
    }

    private void startSDKRegistration() {
        if (!isRegistrationInProgress.compareAndSet(false, true)) return;

        AsyncTask.execute(() -> {
            SDKManager.getInstance().init(getApplicationContext(), new SDKManagerCallback() {
                @Override
                public void onRegisterSuccess() {
                    new Handler(Looper.getMainLooper()).post(() -> {
                        Log.i(TAG, "SDK registered successfully");
                        try {
                            tvSdkVersion.setText("SDK: " + SDKManager.getInstance().getSDKVersion());
                        } catch (Exception ignored) {}
                        registerKeyListeners();
                        refreshUI();
                    });
                }

                @Override
                public void onRegisterFailure(IDJIError error) {
                    Log.e(TAG, "SDK registration failed: " + error.description());
                    runOnUiThread(() -> tvConnectionStatus.setText("SDK Registration Failed"));
                }

                @Override
                public void onInitProcess(DJISDKInitEvent event, int percent) {
                    if (event == DJISDKInitEvent.INITIALIZE_COMPLETE) {
                        SDKManager.getInstance().registerApp();
                    }
                }

                @Override
                public void onProductConnect(int productId) {
                    Log.i(TAG, "Product connected: " + productId);
                }

                @Override
                public void onProductDisconnect(int productId) {
                    Log.i(TAG, "Product disconnected: " + productId);
                    runOnUiThread(() -> {
                        btnOpen.setEnabled(false);
                        tvConnectionStatus.setText("Disconnected");
                    });
                }

                @Override
                public void onProductChanged(int productId) {}

                @Override
                public void onDatabaseDownloadProgress(long current, long total) {}
            });
        });

        // Key listeners are registered after onRegisterSuccess callback
    }

    private void registerKeyListeners() {
        KeyManager.getInstance().listen(KeyTools.createKey(ProductKey.KeyConnection), this,
            (Boolean oldVal, Boolean newVal) -> {
                if (newVal != null) {
                    productConnected = newVal;
                    runOnUiThread(() -> {
                        btnOpen.setEnabled(productConnected && fcConnected);
                        tvConnectionStatus.setText(productConnected ? "Product Connected" : "Disconnected");
                    });
                }
            });

        KeyManager.getInstance().listen(KeyTools.createKey(FlightControllerKey.KeyConnection), this,
            (Boolean oldVal, Boolean newVal) -> {
                if (newVal != null) {
                    fcConnected = newVal;
                    runOnUiThread(() -> btnOpen.setEnabled(productConnected && fcConnected));
                }
            });

        KeyManager.getInstance().listen(KeyTools.createKey(ProductKey.KeyProductType), this,
            (ProductType oldVal, ProductType newVal) -> {
                if (newVal != null) {
                    runOnUiThread(() -> tvProduct.setText(newVal.name()));
                }
            });
    }

    private void refreshUI() {
        KeyManager.getInstance().getValue(KeyTools.createKey(ProductKey.KeyConnection),
            new CommonCallbacks.CompletionCallbackWithParam<Boolean>() {
                @Override
                public void onSuccess(Boolean connected) {
                    if (connected != null && connected) {
                        productConnected = true;
                        runOnUiThread(() -> {
                            tvConnectionStatus.setText("Product Connected");
                            btnOpen.setEnabled(true);
                        });
                    }
                }
                @Override
                public void onFailure(IDJIError error) {}
            });
    }

    @Override
    public void onClick(View v) {
        if (v.getId() == R.id.btn_open) {
            startActivity(new Intent(this, SwarmActivity.class));
        }
    }

    @Override
    protected void onDestroy() {
        KeyManager.getInstance().cancelListen(this);
        super.onDestroy();
    }
}
