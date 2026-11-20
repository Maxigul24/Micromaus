/* Gemeinsame Mechanik-, Motor- und Encoderparameter. */
#ifndef MICROMAUS_CONFIG_H
#define MICROMAUS_CONFIG_H

/* AMT102-V sitzt hinter dem Getriebe direkt auf der Radwelle. */
#define ENCODER_PPR                2048
#define ENCODER_COUNTS_PER_REV     (ENCODER_PPR * 4)

/* JGB37-3525, Variante 960 RPM: Getriebe ca. 1:4,4 (aus Leerlaufdrehzahl geschaetzt).
 * Nur fuer die Feldfrequenz relevant; mit dem Testmodus kalibrieren:
 * GETRIEBE_NEU = GETRIEBE_ALT * Soll / Ist. */
#define GETRIEBE_UNTERSETZUNG_X10  250
#define POLPAARE                   3   /* am Motor geprueft */

#define RAMPE_ABTRIEB_RPM_PRO_S    120 /* Beschleunigung und Bremsen am Rad */

/* Gegen-EMK-Ausgleich (Closed Loop): Leerlauf 960 Rad-RPM bei 12 V -> Strang-Amplitude
 * 12 V/sqrt(3) bei 960 RPM. Bei VM = 12 V: 1000 * 12 / (sqrt(3) * 12 * 960) = 0,6 pro Rad-RPM.
 * Unabhaengig vom Getriebe-Schaetzwert, da auf Rad-RPM bezogen. */
#define STAERKE_PRO_RPM_X10        6

#define RAD_DURCHMESSER_MM         60
#define SPURWEITE_MM               100
#define ZELLE_MM                   200

#endif
