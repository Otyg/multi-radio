# Server-side miljövariabler

Den här listan gäller server-sidan: `multi_radio_server` och `multi_radio_radio`.

Prioritet:
- miljövariabel vinner över värdet i `backend.ini`
- om ingen miljövariabel finns används värdet från configfilen
- om inget värde finns där används inbyggt standardvärde

## Grundkonfiguration

| Variabel | Standardvärde | Gäller | Beskrivning |
| --- | --- | --- | --- |
| `MR_BACKEND_CONFIG` | `backend.ini` | båda | Sökväg till backend-konfigfilen. |
| `MR_BIND_ADDRESS` | `0.0.0.0:50051` | båda | gRPC-adress som processen lyssnar på. |
| `MR_AUTH_TOKEN` | `multi-radio-dev-token` | båda | Delad auth-token för huvud-API:t. |
| `MR_PLUGIN_DIR` | `./backend/plugins` | främst tjock backend | Katalog med plugin-`.so`. På tunn radio-host är den normalt tom i split-läge. |
| `MR_LOG_DIR` | `./logs` | båda | Katalog för JSONL-loggar, track-db och plugin state. |
| `MR_POSITION_BIND_ADDRESS` | tom | båda | Extra position-only gRPC-endpoint. Tomt värde stänger av endpointen. |
| `MR_POSITION_AUTH_TOKEN` | tom | båda | Auth-token för position-only endpointen. Tomt värde betyder ingen auth där. |
| `MR_BACKEND_MODE` | `local` | båda | `local` eller `remote`. `remote` används för tjock backend som hämtar IQ från radio-host. |
| `MR_REMOTE_DSP_HOST` | tom | tjock backend i `remote` | Adress till tunn radio-host, t.ex. `192.168.128.6:50051`. Krävs i `remote`-läge. |
| `MR_IQ_TRANSPORT` | `grpc_stream` | båda | Val av IQ-transport. Nuvarande giltiga värde är `grpc_stream`. |
| `MR_HARDWARE_CONFIG_PATH` | `$HOME/.config/multi-radio/hardware.conf` | båda | Sökväg till persistent hårdvarukonfig, bl.a. ppm-korrektion. |

## Signalhälsa / IQ-trösklar

De här läses vid processstart och styr `IQ_THRESHOLDS` / `IQ_STATS`.

| Variabel | Standardvärde | Beskrivning |
| --- | --- | --- |
| `MR_SIGNAL_SR_ABS_MAX_HZ` | `2000.0` | Max tillåtet absolut samplingsfel i Hz. |
| `MR_SIGNAL_SR_REL_MAX` | `0.02` | Max tillåtet relativt samplingsfel. |
| `MR_SIGNAL_LEVEL_MIN_DBFS` | `-55.0` | Lägre gräns för godkänd signalnivå. |
| `MR_SIGNAL_LEVEL_MAX_DBFS` | `-8.0` | Övre gräns för godkänd signalnivå. |
| `MR_SIGNAL_CLIP_MAX_PCT` | `2.5` | Max tillåten clipping i procent. |
| `MR_SIGNAL_SNR_MIN_DB` | `12.0` | Minsta godkända SNR i dB. |
| `MR_SIGNAL_STABLE_MAX_SNR_DELTA_DB` | `20.0` | Max tillåten SNR-variation mellan fönster. |
| `MR_SIGNAL_STABLE_MAX_LEVEL_DELTA_DB` | `18.0` | Max tillåten nivåvariation mellan fönster. |
| `MR_SIGNAL_STABLE_WINDOWS` | `2` | Antal stabila fönster som krävs innan råstatus blir OK. |
| `MR_SIGNAL_HYST_ON_WINDOWS` | `2` | Antal bra fönster för att slå på `signal_ok`. |
| `MR_SIGNAL_HYST_OFF_WINDOWS` | `3` | Antal dåliga fönster för att slå av `signal_ok`. |

## Debug / felsökning

| Variabel | Standardvärde | Beskrivning |
| --- | --- | --- |
| `MR_PLUGIN_DEBUG` | av | Skriver plugin-load, vald kedja och vissa plugin-händelser till stderr. Alla värden utom tomt eller `0` aktiverar. |
| `MR_AIS_DEBUG` | av | Extra debug för AIS/ASM/NRZI-relaterade plugins. Alla värden utom tomt eller `0` aktiverar. |

## Plugin-specifika variabler

De här används bara om motsvarande plugin faktiskt är laddat och valt i kedjan.

### Demodulatorer / dekodrar

| Variabel | Standardvärde | Plugin | Beskrivning |
| --- | --- | --- | --- |
| `MR_FSK_BAUD_RATE` | `4800` | `fsk_demod` | Symbolhastighet för FSK-demod. |
| `MR_FSK_DEVIATION_HZ` | `2400` | `fsk_demod` | Frekvensdeviation för FSK-demod. |
| `MR_GMSK_BAUD_RATE` | `9600` | `gmsk_demod` | Symbolhastighet för GMSK-demod. |
| `MR_GMSK_BT` | `0.3` | `gmsk_demod` | BT-produkt för GMSK-filtret. |
| `MR_DSC_BAUD_RATE` | `1200` | `dsc_afsk_demod` | Symbolhastighet för DSC AFSK. |
| `MR_DSC_MARK_HZ` | `1300` | `dsc_afsk_demod` | Mark-ton för DSC AFSK. |
| `MR_DSC_SPACE_HZ` | `2100` | `dsc_afsk_demod` | Space-ton för DSC AFSK. |
| `MR_DSC_BIT1_IS_MARK` | `1` | `dsc_afsk_demod` | Sätter om logisk etta motsvarar mark-ton. |
| `MR_NRZI_INVERT` | `0` | `nrzi_decoder` | Inverterar NRZI-konventionen när satt till ett sant heltal. |
| `MR_PPM_BIT_DURATION_US` | `10` | `ppm_demod` | Bitlängd i mikrosekunder för PPM-demod. |
| `MR_VDES_PHY_DIAG_INTERVAL_BLOCKS` | `20` | `vdes_phy_demod` | Hur ofta PHY-diagnostik loggas, i block. `0` tillåts. |
| `MR_VDES_PHY_SQUELCH_DB` | `10.0` | `vdes_phy_demod` | Squelch-tröskel i dB för VDES PHY-demod. |

### IQ-inspelning

| Variabel | Standardvärde | Plugin | Beskrivning |
| --- | --- | --- | --- |
| `MR_IQ_RECORD_ENABLED` | på | `iq_recorder` | Sätt till `0` för att starta pluginen avstängd. |
| `MR_IQ_RECORD_DIR` | `recordings/` | `iq_recorder` | Utkatalog för `.iq16` och metadata. |
| `MR_IQ_RECORD_PREFIX` | `raw` | `iq_recorder` | Filprefix för inspelningar. |
| `MR_IQ_RECORD_MAX_BYTES` | `0` | `iq_recorder` | Säkerhetsgräns. `0` betyder ingen storleksgräns. |

## Praktiska profiler

### Tunn radio-host

Vanlig minimiuppsättning:

```ini
MR_BIND_ADDRESS=192.168.128.6:50051
MR_AUTH_TOKEN=...
MR_PLUGIN_DIR=
MR_LOG_DIR=./logs-radio
MR_BACKEND_MODE=local
```

### Tjock backend-host

Vanlig minimiuppsättning:

```ini
MR_BIND_ADDRESS=192.168.128.2:50051
MR_AUTH_TOKEN=...
MR_PLUGIN_DIR=./backend/plugins
MR_LOG_DIR=./logs
MR_BACKEND_MODE=remote
MR_REMOTE_DSP_HOST=192.168.128.6:50051
MR_IQ_TRANSPORT=grpc_stream
```
