# gborges VHAL

Replacement for `android.hardware.automotive.vehicle@V4-default-service`. Soong displaces upstream via `overrides:` on the `cc_binary` — never patched in `hardware/interfaces/automotive/`.

For the bigger picture (where this fits in the tree, SELinux exemption, build commands), see the *Architecture: the gborges VHAL* section in the top-level [`CLAUDE.md`](../../../CLAUDE.md).

## Provider architecture

`VehicleHardware` holds a `PropertyProviderRegistry`. A provider implements `IPropertyProvider` and **claims** `(propId, areaId)` pairs at registration time:

- **Claimed pairs** — `setValues()` updates the in-memory cache **and** fans out to `provider.writeValue()` (e.g. MQTT publish). The cache is the source of truth: the framework sees the new value immediately whether or not any external client is listening or echoing. When an external source mutates the property out-of-band (e.g. an MQTT `/cmd` from an HMI or sensor sim), the provider feeds the update back through `registry.onUpdate`, which also writes the cache and fires `mOnPropertyChangeCallback`. Either path keeps the cache consistent.
- **Unclaimed pairs** — fall through to the cache, synchronously, exactly as upstream `@V4-default-service` would.

`seedProperties()` populates VIN / make / model / `PERF_VEHICLE_SPEED` / `GEAR_SELECTION` / `PARKING_BRAKE_ON` / `IGNITION_STATE` / HVAC / heartbeat etc. with sane defaults at boot.

The contract is in [`include/IPropertyProvider.h`](include/IPropertyProvider.h); registry behaviour in [`src/PropertyProviderRegistry.cpp`](src/PropertyProviderRegistry.cpp).

### Current providers

| Provider | Path | Claims |
|---|---|---|
| MQTT | `providers/mqtt/` | `PERF_VEHICLE_SPEED`, `IGNITION_STATE`, `GEAR_SELECTION`, `PARKING_BRAKE_ON`, `NIGHT_MODE`, `HVAC_POWER_ON`, `DISPLAY_BRIGHTNESS` (see `mqttClaims()` in `src/VehicleService.cpp`) |

The MQTT provider talks to the on-device NanoMQ broker at `mqtt-tcp://127.0.0.1:1883`, links statically against `libnng`, and uses `<prefix>/0x<propIdHex>/<areaId>` for reads and `…/cmd` for writes with a JSON envelope.

### Topic directions

The two suffixes are direction-specific. **Topic prefix is `gborges/vhal`.**

| Direction | Topic | Who publishes | What it means |
|---|---|---|---|
| Outbound | `gborges/vhal/0x<prop>/<area>` | The provider (i.e. the VHAL itself) | Canonical current value. Published when the provider echoes a value (after a `/cmd`, or after a `setValues` from CarService for a claimed prop) |
| Inbound | `gborges/vhal/0x<prop>/<area>/cmd` | External clients (HMI, sim, host scripts) | Request to set the property. Decoded by the provider, fed into VHAL, then echoed on the read topic |

**To set a value from an external MQTT client, you must publish to the `/cmd` topic.** Publishing on the bare read topic does nothing — the provider only subscribes to `<prefix>/+/+/cmd`.

### JSON envelope

Populate exactly one typed array (or `stringValue` / `byteValues`) — whichever matches the property's declared type. Optional fields: `status` (`"AVAILABLE"` | `"UNAVAILABLE"` | `"ERROR"`, defaults to `AVAILABLE`), `timestamp` (ns; auto-filled if 0).

```json
{ "int32Values": [1] }
{ "floatValues": [27.7] }
{ "int64Values": [1700000000000000000] }
{ "stringValue": "GBORGES000000001" }
```

### Examples (claimed properties)

Speed = 100 km/h (~27.7 m/s):

```
topic:   gborges/vhal/0x11600207/0/cmd
payload: {"floatValues":[27.7]}
```

Gear → DRIVE (`VehicleGear::GEAR_DRIVE` = 8):

```
topic:   gborges/vhal/0x11400400/0/cmd
payload: {"int32Values":[8]}
```

Parking brake off:

```
topic:   gborges/vhal/0x11200402/0/cmd
payload: {"int32Values":[0]}
```

Ignition → ON (`VehicleIgnitionState::ON` = 4):

```
topic:   gborges/vhal/0x11400409/0/cmd
payload: {"int32Values":[4]}
```

Night mode on:

```
topic:   gborges/vhal/0x11200407/0/cmd
payload: {"int32Values":[1]}
```

HVAC power on (zoned — areaId is `HVAC_ALL` = `0x117`):

```
topic:   gborges/vhal/0x11200510/279/cmd
payload: {"int32Values":[1]}
```

### Tools

Per project rules, **no mosquitto clients** on the gborges system — use [paho-mqtt](https://pypi.org/project/paho-mqtt/) (Python), an `nng`-based client, or MQTT Explorer.

```python
import json, paho.mqtt.publish as pub
pub.single("gborges/vhal/0x11600207/0/cmd",
           json.dumps({"floatValues": [27.7]}),
           hostname="<pi-ip>")
```

To watch the round-trip, subscribe to `gborges/vhal/#` and you'll see your `/cmd` publish followed by the provider's echo on the bare read topic.

### Verifying on-device

```sh
adb shell logcat -b all *:V | grep GborgesVHAL-MQTT
# expect, per /cmd:
#   rx topic=gborges/vhal/0x11600207/0/cmd len=23 payload={"floatValues":[27.7]}
#   dispatch prop=0x11600207 area=0 float[1]={27.7000}
adb shell dumpsys android.hardware.automotive.vehicle.IVehicle/default | grep -A2 11600207
```

## Roadmap

Real vehicle signals will eventually move to **CAN** (via the MCP2515 HAT — pinout in `device/gborges/README.md`). MQTT will stay as a **userdebug / eng dev backplane only** — useful for injecting test signals, scripting integration tests from the host, and stubbing properties no CAN frame is mapped to yet. On `user` builds the MQTT provider should be compiled out (or its claims set should be empty), leaving only the CAN provider for production traffic.

The provider registry already supports this split — multiple providers can register, each claiming a disjoint set of properties. Adding a `CanPropertyProvider` is a sibling of `providers/mqtt/`, not a rewrite.

## Known gaps — MQTT security

The current MQTT setup is intentionally minimal and **not suitable for a production network**:

- **No TLS.** Plain `mqtt-tcp://`. NanoMQ on this tree is built without a TLS engine (see `external/nanonng/Android.bp`); WSS / QUIC / SQLite / SCRAM are also intentionally excluded.
- **No authentication.** `nanomq.conf.android` has `allow_anonymous = true` and `acl_nomatch = allow`. Any LAN client that can reach port 1883 can publish to claim topics and drive VHAL state.
- **No client certificates.** No mTLS / device identity. The `clientId` (`gborges-vhal`) is informational only.
- **No on-the-wire integrity.** No payload signing; topics and JSON envelopes are trusted as-is.
- **Broadcast on the LAN.** The broker binds `0.0.0.0:1883` and advertises itself via mDNS (`_mqtt._tcp:1883`).

These are acceptable on a closed bench network for development. Before this ships on a vehicle, at minimum: enable TLS in NanoMQ (requires re-introducing a TLS engine to `external/nanonng/Android.bp`), switch `allow_anonymous` off, provision per-device credentials, and either firewall the broker to localhost or restrict listeners by interface. By that point the CAN provider should be carrying the real signals and MQTT can be dropped from `user` builds entirely.

## Layout

```
vendor/gborges/vhal/
├── Android.bp                             cc_binary "@V4-gborges-service" + libgborges-vhal-headers
├── manifest/vhal-gborges-service.xml      VINTF: IVehicle/default v4
├── rc/vhal-gborges-service.rc             init: class early_hal, user vehicle_network
├── include/                               IPropertyProvider, PropertyProviderRegistry, VehicleHardware
├── src/                                   VehicleService.cpp (main), VehicleHardware.cpp, PropertyProviderRegistry.cpp
└── providers/mqtt/                        cc_library_static "libgborges-vhal-provider-mqtt"
    ├── include/                           MqttPropertyProvider.h, PropertyTopicCodec.h
    └── src/                               MqttPropertyProvider.cpp, PropertyTopicCodec.cpp
```
