package at.jku.icg.aos_dji_sdkv5.core;

import android.util.Log;
import io.moquette.broker.Server;
import io.moquette.broker.config.IConfig;
import io.moquette.broker.config.MemoryConfig;
import io.moquette.interception.AbstractInterceptHandler;
import io.moquette.interception.messages.InterceptPublishMessage;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.util.Collections;
import java.util.Properties;

public final class MQTTEmbedded {
    public static final String TAG = "MQTTEmbedded";
    private static AOSManager aosManager;
    private static String dataDir;
    public static Server mqttBroker;

    /** Listener for incoming MQTT commands (joystick or waypoint). */
    public interface CommandListener {
        void onCommandReceived(String command);
    }

    private static CommandListener commandListener;

    public void setCommandListener(CommandListener listener) {
        commandListener = listener;
    }

    static class PublisherListener extends AbstractInterceptHandler {
        @Override
        public String getID() {
            return "MQTTEmbeddedPublishListener";
        }

        PublisherListener() {
        }

        @Override
        public void onPublish(InterceptPublishMessage interceptPublishMessage) {
            String payload = interceptPublishMessage.getPayload().toString(StandardCharsets.UTF_8);
            if (MQTTEmbedded.commandListener != null) {
                MQTTEmbedded.commandListener.onCommandReceived(payload);
            }
        }

        public void onSessionLoopError(Throwable th) {
            Log.e(TAG, "Session event loop reported error: " + th);
        }
    }

    public void run() throws InterruptedException, IOException {
        Properties props = new Properties();
        props.setProperty("port", "1883");
        props.setProperty("host", "0.0.0.0");
        props.setProperty("allow_anonymous", "true");
        if (dataDir != null) {
            props.setProperty("persistent_store", dataDir + "/moquette_store.h2");
        }
        IConfig config = new MemoryConfig(props);
        Server server = new Server();
        mqttBroker = server;
        server.startServer(config, Collections.singletonList(new PublisherListener()));
        Log.v(TAG, "MQTT moquette Broker started on 0.0.0.0:1883");
    }

    public void stop() {
        if (mqttBroker != null) {
            mqttBroker.stopServer();
        }
    }

    public MQTTEmbedded(AOSManager aOSManager, String appDataDir) {
        aosManager = aOSManager;
        dataDir = appDataDir;
    }
}
