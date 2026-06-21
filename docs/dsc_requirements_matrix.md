# VHF-DSC Steg 1 - Kravmatris (M.493/M.541/M.585)

Datum: 2026-05-26  
Status: Initial baseline (RX-only, VHF kanal 70)

## Scope

- Endast mottagning (`RX-only`), ingen sändningslogik.
- Fokus på VHF-DSC fysik/protokollkedja från IQ/audio till bit-/ordnivå.
- Första iterationen täcker syntetiska testvektorer + demodulatorpipeline.

## Standardbaseline

- ITU-R M.493-16 (12/2023): DSC signalering, ordstruktur, call sequence.
- ITU-R M.541-11 (11/2023): operativa regler/procedurer.
- ITU-R M.585-10 (04/2026): MMSI/identiteter.
- ETSI EN 300 338-serien: testmetodik/conformance (senare certifieringsfas).

## Kravmatris

| ID | Krav | Källa | Verifiering |
|---|---|---|---|
| DSC-RQ-001 | Demodulatorn ska stödja 1200 baud för VHF-DSC. | ITU-R M.493 | Enhetstest + syntetisk replay |
| DSC-RQ-002 | VHF-DSC tonpar ska hanteras med center 1700 Hz och mark/space 1300/2100 Hz. | ITU-R M.493 | Syntvektor med tonmappning + spektrumanalys |
| DSC-RQ-003 | Pipeline ska kunna konsumera både IQ och diskriminator/audio-path (via normaliserad intern kanal). | Arkitekturkrav | Integrations-/replaytest |
| DSC-RQ-004 | Demodulatorn ska exponera kvalitetsmått per burst (SNR/energi/bit confidence). | Intern design | JSON fält i replay-utdata |
| DSC-RQ-005 | Bitström ska produceras med reproducerbar timing trots icke-heltal samples/symbol. | Intern design | Deterministiskt test med seed |
| DSC-RQ-006 | Burst-detektering ska tåla frekvensoffset och AWGN inom definierade testprofiler. | ITU-R M.493 + testplan | SNR/offset sweep |
| DSC-RQ-007 | Ordtolkning ska följa DSC 10-bit ord (7 info + 3 zero-count check). | ITU-R M.493 | Enhetstest ordkod/ordvalidering |
| DSC-RQ-008 | RX ska stödja time-diversity-sammanslagning (DX/RX par) i parserfas. | ITU-R M.493 | Sekvens-/parsertest |
| DSC-RQ-009 | Parser ska validera EOS/ECC och markera felorsak explicit. | ITU-R M.493 | Parsertest med korrupta ramar |
| DSC-RQ-010 | MMSI-tolkning ska följa M.585 och returnera strukturerat fält. | ITU-R M.585 | Enhetstest på kända MMSI-exempel |
| DSC-RQ-011 | Operativ deduplicering/alerting ska följa M.541-flöden för repetitioner. | ITU-R M.541 | Scenario-/integrationstest |
| DSC-RQ-012 | Testvektorformatet ska vara maskinläsbart och spårbart till krav-ID. | Intern QA | JSON manifest + CI-check |

## Definition av klart för steg 1

- Scope och baseline fastlåsta enligt ovan.
- Krav-ID etablerade och refererade i testplan.
- Initial verifieringsstrategi definierad per krav.
