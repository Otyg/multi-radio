#!/usr/bin/env python3
import sys

def encode_ais_nmea(hex_str):
    """
    Konverterar en rå hex-sträng (AIS-ram) till en !AIVDM NMEA-mening.
    Antar att hex-strängen avslutas med 2 bytes HDLC FCS (som i multi-radio-projektet).
    """
    try:
        # Konvertera hex till bytes
        data = bytes.fromhex(hex_str)
    except ValueError:
        return "Fel: Ogiltig hex-sträng."

    if len(data) < 2:
        return "Fel: För kort data (behöver minst 2 bytes för FCS)."

    # AIS-ramar i NMEA-format inkluderar inte de sista 2 byten (HDLC FCS)
    payload = data[:-2]
    
    encoded = ""
    bit_acc = 0
    bit_count = 0

    # Loopa igenom payload och paketera om 8-bitars bytes till 6-bitars värden
    for byte in payload:
        bit_acc = (bit_acc << 8) | byte
        bit_count += 8
        while bit_count >= 6:
            val = (bit_acc >> (bit_count - 6)) & 0x3F
            bit_count -= 6
            # AIS 6-bit ASCII-tabell: 0-39 -> +48, 40-63 -> +56
            char_code = val + 48 if val < 40 else val + 56
            encoded += chr(char_code)

    # Hantera kvarvarande bitar (fill bits)
    fill_bits = 0
    if bit_count > 0:
        fill_bits = 6 - bit_count
        val = (bit_acc << fill_bits) & 0x3F
        char_code = val + 48 if val < 40 else val + 56
        encoded += chr(char_code)

    # Bygg NMEA-meningen (AIVDM, 1 fragment, frag 1, tom sekvens-ID, kanal A)
    sentence = f"AIVDM,1,1,,A,{encoded},{fill_bits}"

    # Beräkna NMEA-kontrollsumman (XOR av alla tecken mellan ! och *)
    checksum = 0
    for char in sentence:
        checksum ^= ord(char)

    return f"!{sentence}*{checksum:02X}"

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(f"Användning: {sys.argv[0]} <hex_sträng>")
        print("Exempel: 10C4D0A0... (inklusive FCS)")
        sys.exit(1)

    print(encode_ais_nmea(sys.argv[1]))