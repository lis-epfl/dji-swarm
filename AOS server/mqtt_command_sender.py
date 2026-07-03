"""
LIS_Swarm direct MQTT command sender
====================================
Publishes command strings (VS:..., ENABLE_VS, TAKEOFF, ...) straight to each
RC's embedded Moquette broker (tcp://<rc-ip>:1883) over ONE persistent
connection per drone.

This bypasses DroneSwarmServer's send path, which builds and tears down a full
MQTT connection per command (Dialog1Dlg.cpp SendWayPoint2Drone2: create ->
TCP connect -> publish -> waitForCompletion -> disconnect x2 -> destroy,
~220 ms each). That capped commands at ~4.5 Hz with one drone and ~1.5 Hz per
drone with three. A persistent connection pays the connect cost once; each
publish is then a sub-millisecond socket write, so 20 Hz per drone is trivial.

The app side needs no changes: its Moquette intercept handler fires
onCommandReceived for ANY publish from ANY client regardless of topic
(MQTTEmbedded.java), and the payload is the same raw command string.

Design notes:
- paho-mqtt 2.x API (CallbackAPIVersion.VERSION2), already installed for the
  project's Python 3.7.
- One client + network thread per drone via loop_start(); connect_async +
  reconnect_delay_set give automatic reconnection, so an RC power-cycle or
  cable pull self-heals.
- Unique client ids (LIS_PC_d<N>_<pid>) so connections can't kick each other
  or the server's transient "MQTTDroneSwarm" client off the broker.
- QoS policy (chosen by the caller per send): QoS 0 for the 20 Hz VS stream
  (latest-wins; drops while disconnected are correct), QoS 1 for one-shot
  commands (ENABLE_VS/DISABLE_VS/TAKEOFF/LAND) — paho queues QoS 1 publishes
  while disconnected and delivers them on reconnect.
- Best-effort like the other feed publishers: send() never raises and never
  blocks the control loops; failures print throttled warnings.

No ds_wrapper import — telemetry/video still flow through DroneSwarmServer.
"""

import os
import time

import paho.mqtt.client as mqtt

# Topic the server's send path also uses (Dialog1Dlg.cpp TOPIC). The app's
# intercept handler ignores the topic, but keeping it consistent aids sniffing.
COMMAND_TOPIC = "MQTTWayPoints"
BROKER_PORT = 1883
KEEPALIVE_S = 10

# Minimum seconds between repeated "publish failed" warnings per drone.
_WARN_INTERVAL_S = 1.0


class MqttCommandSender:
    """Persistent per-drone MQTT publishers for drone command strings."""

    def __init__(self, ip_by_drone, port=BROKER_PORT, topic=COMMAND_TOPIC,
                 keepalive=KEEPALIVE_S):
        """
        Args:
            ip_by_drone: {drone_id (1-based int): "a.b.c.d"} RC broker IPs.
        """
        self._topic = topic
        self._ips = dict(ip_by_drone)
        self._clients = {}
        self._connected = {did: False for did in self._ips}
        self._last_warn = {did: 0.0 for did in self._ips}
        self._stopping = False

        for did, ip in self._ips.items():
            client = mqtt.Client(
                mqtt.CallbackAPIVersion.VERSION2,
                client_id="LIS_PC_d{}_{}".format(did, os.getpid()))
            client.on_connect = self._make_on_connect(did)
            client.on_disconnect = self._make_on_disconnect(did)
            client.reconnect_delay_set(min_delay=0.5, max_delay=2)
            # connect_async + loop_start: the network thread owns connecting,
            # keepalive pings, and reconnection; never blocks this thread.
            client.connect_async(ip, port, keepalive)
            client.loop_start()
            self._clients[did] = client

    def _make_on_connect(self, drone_id):
        def on_connect(client, userdata, flags, reason_code, properties):
            self._connected[drone_id] = True
            print("[mqtt-cmd {}] connected to {} ({})".format(
                drone_id, self._ips[drone_id], reason_code), flush=True)
        return on_connect

    def _make_on_disconnect(self, drone_id):
        def on_disconnect(client, userdata, flags, reason_code, properties):
            self._connected[drone_id] = False
            if not self._stopping:
                print("[mqtt-cmd {}] disconnected from {} ({}); "
                      "auto-reconnecting".format(
                          drone_id, self._ips[drone_id], reason_code),
                      flush=True)
        return on_disconnect

    def connected(self, drone_id):
        return self._connected.get(drone_id, False)

    def send(self, drone_id, command, qos=0):
        """Publish a command string to drone_id's broker. Never raises/blocks.

        Returns True if the publish was accepted locally (QoS 0: written to
        the socket buffer; QoS 1: queued for delivery), False otherwise.
        """
        client = self._clients.get(drone_id)
        if client is None:
            self._warn(drone_id, "no MQTT client for drone {}".format(drone_id))
            return False
        try:
            info = client.publish(self._topic, command, qos=qos)
            if info.rc == mqtt.MQTT_ERR_SUCCESS:
                return True
            # QoS 0 while disconnected lands here (MQTT_ERR_NO_CONN): the VS
            # stream is latest-wins, so dropping is the right behaviour.
            self._warn(drone_id, "publish rc={} (dropped: {!r})".format(
                info.rc, command))
        except Exception as e:
            self._warn(drone_id, "publish error: {}".format(e))
        return False

    def _warn(self, drone_id, msg):
        now = time.monotonic()
        if now - self._last_warn.get(drone_id, 0.0) >= _WARN_INTERVAL_S:
            self._last_warn[drone_id] = now
            print("[mqtt-cmd {}] {}".format(drone_id, msg), flush=True)

    def stop(self):
        self._stopping = True
        for did, client in self._clients.items():
            try:
                client.disconnect()
                client.loop_stop()
            except Exception:
                pass
        self._clients = {}
