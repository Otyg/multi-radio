# gRPC PositionService

`PositionService` är ett separat gRPC-interface som enbart publicerar
positionsdata (AIS-fartyg och ADS-B-luftfartyg). Audio, IQ-data, avkodade
meddelanden och kontroll-RPC:er är aldrig tillgängliga på detta interface.

Avsedd användning: externa klienter (kartor, dashboards, loggningssystem)
som ska ta emot positioner utan åtkomst till resten av backend.

---

## Konfiguration

Interfacet aktiveras via miljövariabler. Om `MR_POSITION_BIND_ADDRESS` är
tom (standard) startas det inte alls.

| Variabel                    | Standardvärde | Beskrivning |
|-----------------------------|---------------|-------------|
| `MR_POSITION_BIND_ADDRESS`  | *(tom)*        | Adress och port, t.ex. `0.0.0.0:50052`. Tom = inaktivt. |
| `MR_POSITION_AUTH_TOKEN`    | *(tom)*        | Bearer-token som klienten måste skicka. Tom = ingen autentisering. |

Servern loggar vid uppstart:

```
Position-only gRPC endpoint on 0.0.0.0:50052 (no auth)
```

eller

```
Position-only gRPC endpoint on 0.0.0.0:50052 (auth enabled)
```

---

## Proto-definition

Källfil: [`proto/radio.proto`](../proto/radio.proto)

```protobuf
// Begäran saknar parametrar — strömmar alla spårade mål.
message StreamPositionsRequest {}

service PositionService {
  rpc StreamPositions(StreamPositionsRequest) returns (stream RadarSnapshot);
}
```

Svarsmeddelanden återanvänder de befintliga typerna `RadarSnapshot` och
`RadarTarget` (samma som `TelemetryService.StreamRadarSnapshots`).

### RadarSnapshot

```protobuf
message RadarSnapshot {
  repeated RadarTarget targets     = 1;  // alla aktiva mål
  repeated string      removed_ids = 2;  // mål borttagna sedan förra snapshot
  uint64               snapshot_ms = 3;  // Unix-tid i millisekunder
}
```

Servern skickar en ny snapshot ungefär var **2:a sekund**.
`removed_ids` innehåller `id`-värden för mål som har försvunnit ur trackern
sedan föregående snapshot — klienten kan använda listan för att ta bort
markörer från kartan utan att behöva jämföra hela mållistan.

### RadarTarget

| Fält           | Typ      | Innehåll |
|----------------|----------|----------|
| `id`           | string   | MMSI (fartyg) eller ICAO hex (luftfartyg) |
| `label`        | string   | Namn/callsign; faller tillbaka på `id` om okänt |
| `kind`         | string   | `"SEA"`, `"AIR"` eller `"?"` |
| `lat`          | double   | Latitud i decimal grader (WGS-84) |
| `lon`          | double   | Longitud i decimal grader (WGS-84) |
| `sog_knots`    | double   | Fart över grund (knop) |
| `cog_degrees`  | double   | Kurs över grund (grader, 0–360) |
| `has_altitude` | bool     | Sant om `altitude_ft` är giltig |
| `altitude_ft`  | double   | Höjd i fot (enbart luftfartyg) |
| `last_seen_ms` | uint64   | Senaste mottagning (Unix ms) |

---

## Autentisering

Om `MR_POSITION_AUTH_TOKEN` är satt måste klienten skicka en
`Authorization`-header med Bearer-token i varje anrop:

```
Authorization: Bearer <token>
```

Servern svarar med `UNAUTHENTICATED` om token saknas eller är felaktig.

Om variabeln är tom accepteras alla anslutningar utan autentisering —
lämpligt i ett isolerat nät men bör undvikas mot internet.

---

## Exempel: Python-klient

### Generera Python-stubs

`radio_pb2` och `radio_pb2_grpc` är Python-moduler som genereras av `protoc`
från `proto/radio.proto`. De ingår inte i repot och måste genereras en gång
innan klienten kan köras.

```bash
pip install grpcio grpcio-tools

python3 -m grpc_tools.protoc \
  --proto_path=proto \
  --python_out=. \
  --grpc_python_out=. \
  proto/radio.proto
```

Det skapar `radio_pb2.py` (meddelanden) och `radio_pb2_grpc.py` (stubs för
alla tre tjänster). Kör kommandot från projektets rotkatalog eller anpassa
sökvägarna.

### Klientkod

```python
import grpc
from radio_pb2 import StreamPositionsRequest
from radio_pb2_grpc import PositionServiceStub

# Utan autentisering
channel = grpc.insecure_channel('localhost:50052')

# Med token
# metadata = [('authorization', 'Bearer min-token')]
# channel = grpc.insecure_channel('localhost:50052')

stub    = PositionServiceStub(channel)
for snapshot in stub.StreamPositions(StreamPositionsRequest()):
    print(f't={snapshot.snapshot_ms}  {len(snapshot.targets)} mål  '
          f'{len(snapshot.removed_ids)} borttagna')
    for t in snapshot.targets:
        print(f'  {t.kind:3}  {t.id:10}  {t.label:20}  '
              f'{t.lat:.4f},{t.lon:.4f}  {t.sog_knots:.1f} kn')
```

Med token:

```python
creds = grpc.metadata_call_credentials(
    lambda ctx, cb: cb([('authorization', 'Bearer min-token')], None))
channel = grpc.secure_channel(
    'localhost:50052',
    grpc.composite_channel_credentials(grpc.local_channel_credentials(), creds))
```

---

## Skillnad mot TelemetryService

| Egenskap              | `PositionService`           | `TelemetryService`                 |
|-----------------------|-----------------------------|------------------------------------|
| Port                  | Konfigurerbar (`50052`)     | Huvud-port (`50051`)               |
| Auth-token            | Separat, valfritt           | Delad med `RadioControlService`    |
| Positionsdata         | Ja (`StreamPositions`)      | Ja (`StreamRadarSnapshots`)        |
| Audio                 | Nej                         | Ja (`StreamAudioFrames`)           |
| IQ                    | Nej                         | Ja (`StreamIqFrames`)              |
| Avkodade meddelanden  | Nej                         | Ja (`StreamDecodedMessages`)       |
| Mottagarstatus/-event | Nej                         | Ja (`StreamReceiverEvents`)        |
| Mottagarfilter        | Nej (alla mottagare)        | Ja (`receiver_id`-fält i request)  |

`StreamPositions` har inget `receiver_id`-fält — alla mottagares mål slås
samman av `TargetTracker` innan snapshot skickas, på samma sätt som
`StreamRadarSnapshots` med `include_all_receivers = true`.
