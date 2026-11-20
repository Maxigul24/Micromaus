#include "floodfill_fahrt.h"
#include "distance_sensors.h"
#include "encoder.h"
#include "lsm6dsox.h"
#include "micromaus_config.h"
#include "micromouse.h"
#include "motor_control.h"
#include "platform.h"

#define FAHR_RPM               60   /* ~190 mm/s bei 60 mm Rad */
#define DREH_RPM               30
#define WAND_SCHWELLE_MM       150  /* Zelle 200 mm: Wand von der Mitte ~100 mm entfernt */
#define SENSOR_WARTEZEIT_MS    400  /* alle drei ToF liefern in der Zeit neue Werte */

/* PD fuer Geradeausfahrt. Fehler in mm, positiv = Maus zu weit rechts.
 * Werte sind Startwerte, am Gang einstellen: erst KP bis sie leicht pendelt, dann KD hoch. */
#define PD_INTERVALL_MS        100  /* ~ Aktualisierungsrate der ToF (3 Sensoren nacheinander) */
#define KP_X10                 3    /* 0,3 RPM pro mm */
#define KD_X100                5    /* 0,05 RPM pro mm/s */
#define PD_MAX_RPM             20   /* max. Drehzahldifferenz je Rad */
#define SEITE_SOLL_MM          70   /* Abstand zur Wand wenn mittig; wird am Start eingelernt */

/* Start unten links mit Blick nach Norden (Zeile 0 ist der Nordrand). */
#define START_X                0U
#define START_Y                (MICROMOUSE_MAZE_HEIGHT - 1U)
/* Testgang: Ziel = Zelle hinter der seitlichen Oeffnung. Anpassen! */
#define ZIEL_X                 1U
#define ZIEL_Y                 (MICROMOUSE_MAZE_HEIGHT - 3U)

/* Encoder sitzt auf der Radwelle: Ticks = Weg / Radumfang * Counts. */
#define TICKS_ZELLE   ((int32_t)((int64_t)ZELLE_MM * ENCODER_COUNTS_PER_REV * 10000 / \
                                 (31416LL * RAD_DURCHMESSER_MM)))
/* 90 Grad auf der Stelle: Bogen pi*Spur/4, pi kuerzt sich heraus. */
#define TICKS_90GRAD  ((int32_t)((int64_t)SPURWEITE_MM * ENCODER_COUNTS_PER_REV / \
                                 (4 * RAD_DURCHMESSER_MM)))

static const int8_t schritt_x[4] = { 0, 1, 0, -1 };
static const int8_t schritt_y[4] = { -1, 0, 1, 0 };
static int32_t seite_soll_mm = SEITE_SOLL_MM;

static int32_t betrag(int32_t v)
{
    return v < 0 ? -v : v;
}

static bool wand_abstand(distance_sensor_id_t id, int32_t *mm)
{
    distance_reading_t r;
    if (!distance_sensors_get(id, &r) || !r.initialized || !r.valid ||
        r.distance_mm >= WAND_SCHWELLE_MM) {
        return false;
    }
    *mm = r.distance_mm;
    return true;
}

static bool wand(distance_sensor_id_t id)
{
    int32_t mm;
    return wand_abstand(id, &mm);
}

typedef struct {
    uint32_t letzte_ms;
    int32_t fehler_alt;
    uint8_t quelle_alt;     /* 0 = keine, 1 = beide Waende, 2 = links, 3 = rechts, 4 = Encoder */
    int32_t ref_l;          /* Encoder-Bezug fuer Kurshalten ohne Waende */
    int32_t ref_r;
} pd_t;

/* Waende wenn vorhanden, sonst Encoder-Differenz. Liefert Drehzahlkorrektur (+ = nach links). */
static int32_t pd_korrektur(pd_t *pd, uint32_t now, int32_t ticks_l, int32_t ticks_r)
{
    int32_t links, rechts, fehler;
    uint8_t quelle;
    bool wl = wand_abstand(DISTANCE_SENSOR_LEFT, &links);
    bool wr = wand_abstand(DISTANCE_SENSOR_RIGHT, &rechts);

    if (wl && wr) {
        quelle = 1;
        fehler = links - rechts;
    } else if (wl) {
        quelle = 2;
        fehler = 2 * (links - seite_soll_mm);
    } else if (wr) {
        quelle = 3;
        fehler = 2 * (seite_soll_mm - rechts);
    } else {
        quelle = 4;
        if (pd->quelle_alt != 4) {
            /* Kurs ab hier halten; fruehere Korrekturen nicht rueckgaengig machen. */
            pd->ref_l = ticks_l;
            pd->ref_r = ticks_r;
        }
        /* Betrag, weil das Vorzeichen des rechten Encoders gespiegelt ist. */
        int32_t diff_ticks = betrag(ticks_l - pd->ref_l) - betrag(ticks_r - pd->ref_r);
        fehler = (int32_t)((int64_t)diff_ticks * 31416 * RAD_DURCHMESSER_MM /
                           (10000LL * ENCODER_COUNTS_PER_REV));
    }

    uint32_t dt = now - pd->letzte_ms;
    if (quelle != pd->quelle_alt || dt == 0U) {
        pd->fehler_alt = fehler; /* kein D-Sprung beim Quellenwechsel */
    }
    int32_t korrektur = fehler * KP_X10 / 10 +
                        (dt ? (fehler - pd->fehler_alt) * KD_X100 * 10 / (int32_t)dt : 0);
    pd->fehler_alt = fehler;
    pd->quelle_alt = quelle;
    pd->letzte_ms = now;

    if (korrektur > PD_MAX_RPM) korrektur = PD_MAX_RPM;
    if (korrektur < -PD_MAX_RPM) korrektur = -PD_MAX_RPM;
    return korrektur;
}

static void warte_ms(uint32_t ms)
{
    uint32_t start = millis();
    while (millis() - start < ms) {
        distance_sensors_update(millis());
    }
}

static void stoppen(void)
{
    motor_control_set_speed(MOTOR_1, 0);
    motor_control_set_speed(MOTOR_2, 0);
    motor_control_status_t l, r;
    do {
        distance_sensors_update(millis());
        motor_control_get_status(MOTOR_1, &l);
        motor_control_get_status(MOTOR_2, &r);
    } while (l.target_rpm != 0 || r.target_rpm != 0);
}

static bool gyro_ok;
static int32_t gyro_bias;

/* Summiert Rohwerte; jeder Wert steht fuer 1/ODR Sekunden. */
static bool gyro_integrieren(int64_t *summe)
{
    int16_t raw;
    bool neu;
    if (!lsm6dsox_gyro_z(&raw, &neu)) {
        return false;
    }
    if (neu) {
        *summe += raw - gyro_bias;
    }
    return true;
}

/* Betrag, weil die Einbaulage (Z nach oben/unten) das Vorzeichen bestimmt. */
static int32_t gyro_winkel_mdeg(int64_t summe)
{
    int64_t mdeg = summe * LSM6DSOX_MDPS_PRO_LSB_X10 / (10 * LSM6DSOX_ODR_HZ);
    return (int32_t)(mdeg < 0 ? -mdeg : mdeg);
}

/* Stillstand: Nullpunkt-Offset ueber ~1 s mitteln. */
static void gyro_kalibrieren(void)
{
    int64_t summe = 0;
    int32_t n = 0;
    uint32_t start = millis();
    while (n < LSM6DSOX_ODR_HZ) {
        int16_t raw;
        bool neu;
        if (!lsm6dsox_gyro_z(&raw, &neu) || millis() - start > 3000U) {
            gyro_ok = false;
            return;
        }
        if (neu) {
            summe += raw;
            n++;
        }
    }
    gyro_bias = (int32_t)(summe / n);
}

/* Faehrt bis der mittlere Radweg 'ticks' erreicht. false bei Fehler.
 * Geradeaus lenkt der PD-Regler; Drehungen misst der Gyro (sonst Encoder). */
static bool fahre(int32_t links_rpm, int32_t rechts_rpm, int32_t ticks)
{
    bool gerade = links_rpm == rechts_rpm;
    bool mit_gyro = !gerade && gyro_ok;
    int64_t gyro_summe = 0;
    pd_t pd = { millis(), 0, 0, 0, 0 };
    int32_t v = betrag(links_rpm);
    /* Bremsweg der Rampe v^2/(2a), in Ticks */
    int32_t brems_ticks = (int32_t)((int64_t)v * v * ENCODER_COUNTS_PER_REV /
                                    (2 * RAMPE_ABTRIEB_RPM_PRO_S * 60));
    /* doppelte Sollzeit + 2 s, falls die Encoder nicht zaehlen */
    uint32_t max_ms = (uint32_t)((int64_t)ticks * 60000 * 2 /
                                 ((int64_t)v * ENCODER_COUNTS_PER_REV)) + 2000U;
    /* Drehwinkel aus den Ticks: TICKS_90GRAD entspricht 90 Grad. */
    int32_t ziel_mdeg = (int32_t)((int64_t)ticks * 90000 / TICKS_90GRAD);
    int32_t brems_mdeg = (int32_t)((int64_t)brems_ticks * 90000 / TICKS_90GRAD);
    int32_t l0 = encoder_left_get_ticks();
    int32_t r0 = encoder_right_get_ticks();
    uint32_t start = millis();

    motor_control_set_speed(MOTOR_1, links_rpm);
    motor_control_set_speed(MOTOR_2, rechts_rpm);
    while (1) {
        int32_t tl = encoder_left_get_ticks();
        int32_t tr = encoder_right_get_ticks();
        if (mit_gyro) {
            if (!gyro_integrieren(&gyro_summe)) {
                stoppen();
                uart_print("Fehler: Gyro antwortet nicht, Fahrt abgebrochen\r\n");
                return false;
            }
            if (gyro_winkel_mdeg(gyro_summe) >= ziel_mdeg - brems_mdeg) {
                break;
            }
        } else {
            int32_t weg = (betrag(tl - l0) + betrag(tr - r0)) / 2;
            if (weg >= ticks - brems_ticks) {
                break;
            }
        }
        uint32_t now = millis();
        if (gerade && now - pd.letzte_ms >= PD_INTERVALL_MS) {
            int32_t k = pd_korrektur(&pd, now, tl, tr);
            motor_control_set_speed(MOTOR_1, links_rpm - k);
            motor_control_set_speed(MOTOR_2, rechts_rpm + k);
        }
        motor_control_status_t l, r;
        motor_control_get_status(MOTOR_1, &l);
        motor_control_get_status(MOTOR_2, &r);
        if (l.blocked || r.blocked) {
            stoppen();
            uart_print("Fehler: Rad blockiert/Takt verloren, Fahrt abgebrochen\r\n");
            return false;
        }
        if (millis() - start > max_ms) {
            stoppen();
            uart_print(mit_gyro ? "Fehler: Drehung dauert zu lange, abgebrochen\r\n"
                                : "Fehler: Encoder zaehlen nicht, Fahrt abgebrochen\r\n");
            return false;
        }
        if (!mit_gyro) {
            distance_sensors_update(millis()); /* Gyro-Schleife schnell halten, keine Werte verpassen */
        }
    }

    if (!mit_gyro) {
        stoppen();
        return true;
    }

    /* Beim Bremsen weiter integrieren, damit der Ist-Winkel zum Einstellen stimmt. */
    motor_control_set_speed(MOTOR_1, 0);
    motor_control_set_speed(MOTOR_2, 0);
    motor_control_status_t l, r;
    uint32_t still_ms = 0;
    do {
        gyro_integrieren(&gyro_summe);
        motor_control_get_status(MOTOR_1, &l);
        motor_control_get_status(MOTOR_2, &r);
        if (l.target_rpm == 0 && r.target_rpm == 0 && still_ms == 0) {
            still_ms = millis();
        }
    } while (still_ms == 0 || millis() - still_ms < 200U);
    uart_print("Drehung Ist=");
    uart_print_number((uint32_t)gyro_winkel_mdeg(gyro_summe) / 1000U);
    uart_print(" Soll=");
    uart_print_number((uint32_t)ziel_mdeg / 1000U);
    uart_print(" Grad\r\n");
    return true;
}

void floodfill_fahren(void)
{
    uint8_t x = START_X;
    uint8_t y = START_Y;
    uint8_t blick = MICROMOUSE_NORTH;

    micromouse_init(ZIEL_X, ZIEL_Y);
    uart_print("Floodfill: Start\r\n");

    /* Maus muss dabei stillstehen. */
    gyro_ok = lsm6dsox_init();
    if (gyro_ok) {
        gyro_kalibrieren();
    }
    if (gyro_ok) {
        uart_print("Gyro LSM6DSOX ok, Bias=");
        uart_print_int(gyro_bias);
        uart_print("\r\n");
    } else {
        uart_print("Gyro nicht gefunden: Drehungen ueber Encoder\r\n");
    }

    /* Maus mittig in den Gang stellen: Wandabstand fuer den PD einlernen. */
    warte_ms(SENSOR_WARTEZEIT_MS);
    int32_t links, rechts;
    if (wand_abstand(DISTANCE_SENSOR_LEFT, &links) &&
        wand_abstand(DISTANCE_SENSOR_RIGHT, &rechts)) {
        seite_soll_mm = (links + rechts) / 2;
    }
    uart_print("Seitenabstand Soll=");
    uart_print_number((uint32_t)seite_soll_mm);
    uart_print("mm\r\n");

    while (x != ZIEL_X || y != ZIEL_Y) {
        warte_ms(SENSOR_WARTEZEIT_MS);
        /* Rueckseite bleibt offen: dort sind wir hergekommen (am Start setzt der Rand die Wand). */
        uint8_t waende = 0;
        if (wand(DISTANCE_SENSOR_FRONT)) waende |= (uint8_t)(1U << blick);
        if (wand(DISTANCE_SENSOR_RIGHT)) waende |= (uint8_t)(1U << ((blick + 1U) % 4U));
        if (wand(DISTANCE_SENSOR_LEFT))  waende |= (uint8_t)(1U << ((blick + 3U) % 4U));
        micromouse_set_walls(x, y, waende);

        micromouse_direction_t ziel_richtung;
        if (!micromouse_plan_next(x, y, &ziel_richtung)) {
            uart_print("Floodfill: kein Weg zum Ziel\r\n");
            return;
        }

        uint8_t drehung = (uint8_t)((ziel_richtung + 4U - blick) % 4U);
        bool ok = true;
        if (drehung == 1U) {
            ok = fahre(DREH_RPM, -DREH_RPM, TICKS_90GRAD);
        } else if (drehung == 2U) {
            ok = fahre(DREH_RPM, -DREH_RPM, 2 * TICKS_90GRAD);
        } else if (drehung == 3U) {
            ok = fahre(-DREH_RPM, DREH_RPM, TICKS_90GRAD);
        }
        if (!ok || !fahre(FAHR_RPM, FAHR_RPM, TICKS_ZELLE)) {
            return;
        }

        blick = (uint8_t)ziel_richtung;
        x = (uint8_t)(x + schritt_x[blick]);
        y = (uint8_t)(y + schritt_y[blick]);
        uart_print("Zelle x=");
        uart_print_number(x);
        uart_print(" y=");
        uart_print_number(y);
        uart_print(" | Waende der vorigen Zelle (Bit N/O/S/W)=");
        uart_print_number(waende);
        uart_print("\r\n");
    }
    uart_print("Floodfill: Ziel erreicht\r\n");
}
