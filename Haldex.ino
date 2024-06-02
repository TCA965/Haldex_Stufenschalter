#include <TMC2209.h>
#include <AccelStepper.h>
#include <Arduino.h>
#include <U8g2lib.h>
#include <EEPROM.h>

// Hardware Pins
// Eingänge:
#define IO_Temperatur A0
#define IO_Eingabe_Poti A1
#define IO_Eingabe_25 A1
#define IO_Eingabe_50 2
#define IO_Eingabe_75 3
#define IO_Eingabe_100 A3
#define IO_Diagnose_Pumpe 9
#define IO_Hauptschalter 10
#define IO_Handbremse 11
#define IO_Bremse 12

// Ausgänge:
#define IO_Enable 4
#define IO_Direction 5
#define IO_Step 6
#define IO_LED_Sperrgrad 13
#define IO_Pumpe 23

// TMC2209
HardwareSerial &serial_stream = Serial;
TMC2209 stepper_driver;
TMC2209::Status TMCStatus;

AccelStepper stepper(1, IO_Step, IO_Direction);
// Motorparameter
const uint8_t STALL_GUARD_THRESHOLD = 5;
const uint32_t COOL_STEP_DURATION_THRESHOLD = 2000;

// Variablen zur dynamischen Ermittlung der Schritte
int SchrittzahlMaximum, Schrittzahl100Prozent, Schrittzahl75Prozent, Schrittzahl50Prozent, Schrittzahl25Prozent, Schrittzahl0Prozent;

// u8g2 OLED
U8G2_SH1106_128X64_NONAME_2_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE);

// Steinhart-Hart-Werte und Variablen zum Thermistor lesen
float R1 = 4700;
float logR2, R2, Temperatur;
float c1 = 0.239796837e-03, c2 = 3.742907175e-04, c3 = -1.462932313e-07;

// Timer
unsigned long MillisLeseEingaenge = 0;
const long IntervalLeseEingaenge = 50;

unsigned long MillisSchreibe = 0;
const long IntervalSchreibe = 100;

unsigned long MillisLED = 0;
const long IntervalLED = 500;

// Globale Variablen
int Sperrgrad, SperrgradKompensiert, TempOffset;

// Status Variablen
volatile bool ZustandBremse = false;
volatile bool ZustandHandbremse = false;
volatile bool ZustandHauptschalter = false;
volatile bool ZustandTemperatur = false;
volatile bool ZustandBearbeitet = true;
bool TMCFehler = false;
bool PumpeFehler = false;

// Interrupt-Routine
// Interrupt Vector für Port B (PCINT0_vect)
ISR(PCINT0_vect) {
  // Wenn die Bremse von 0 auf 12 V wechselt
  if (digitalRead(IO_Bremse) && ZustandBremse == false) {
    ZustandBremse = true;
    ZustandBearbeitet = false;
  } else if (!digitalRead(IO_Bremse) && ZustandBremse == true) {
    ZustandBremse = false;
    ZustandBearbeitet = false;
  }

  // Wenn die Handbremse von 0 auf 12 V wechselt
  if (!digitalRead(IO_Handbremse) && ZustandHandbremse == false) {
    ZustandHandbremse = true;
    ZustandBearbeitet = false;
  } else if (digitalRead(IO_Handbremse) && ZustandHandbremse == true) {
    ZustandHandbremse = false;
    ZustandBearbeitet = false;
  }

  // Wenn der Hauptschalter von 12 auf 0 V wechselt
  if (!digitalRead(IO_Hauptschalter) && ZustandHauptschalter == false) {
    ZustandHauptschalter = true;
    ZustandBearbeitet = false;
  } else if (digitalRead(IO_Hauptschalter) && ZustandHauptschalter == true) {
    ZustandHauptschalter = false;
    ZustandBearbeitet = false;
  }
}

void setup() {
  // Willkommensbotschaft
  u8g2.begin();
  u8g2.setFont(u8g2_font_helvR08_tf);
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_helvR08_tf);
    u8g2.setCursor(0, 12);
    u8g2.print(F("Haldex Controller"));
    u8g2.drawHLine(0, 12, 128);
    u8g2.setCursor(0, 40);
    u8g2.print(F("TCA965"));
    u8g2.setCursor(0, 60);
    u8g2.print(F("allradbus.com"));

  } while (u8g2.nextPage());

  // Konfiguriere Stepper
  stepper_driver.setup(serial_stream);
  stepper_driver.setRunCurrent(40);
  stepper_driver.setHoldCurrent(40);
  stepper_driver.enableAutomaticCurrentScaling();
  stepper_driver.enableAutomaticGradientAdaptation();
  stepper_driver.setStallGuardThreshold(STALL_GUARD_THRESHOLD);
  stepper_driver.setMicrostepsPerStep(4);
  stepper_driver.setStandstillMode(TMC2209::BRAKING);
  stepper_driver.setCoolStepDurationThreshold(COOL_STEP_DURATION_THRESHOLD);
  stepper.setMaxSpeed(4000);
  stepper.setAcceleration(4000);

  // Konfiguriere I/O
  pinMode(IO_Enable, OUTPUT);
  pinMode(IO_Step, OUTPUT);
  pinMode(IO_Direction, OUTPUT);
  pinMode(IO_Eingabe_Poti, INPUT);
  pinMode(IO_Temperatur, INPUT);
  pinMode(IO_Bremse, INPUT);
  pinMode(IO_Handbremse, INPUT_PULLUP);
  pinMode(IO_Hauptschalter, INPUT);
  pinMode(IO_Diagnose_Pumpe, INPUT);
  pinMode(IO_Pumpe, OUTPUT);
  pinMode(IO_LED_Sperrgrad, OUTPUT);

  if (digitalRead(IO_Bremse))
    ZustandBremse = true;
  if (!digitalRead(IO_Handbremse))
    ZustandHandbremse = true;
  if (!digitalRead(IO_Hauptschalter))
    ZustandHauptschalter = true;

  // PinChange Interrupt konfigurieren
  PCICR = (1 << PCIE0);                                    // enable PCINT[23:16] interrupts
  PCMSK0 = (1 << PCINT4) | (1 << PCINT3) | (1 << PCINT2);  // PB4 = PCINT4 | PB2 = PCINT2

  // Aktiviere Endstufen in TMC2209
  digitalWrite(IO_Enable, LOW);
  // Aktiviere TMC2209
  stepper_driver.enable();

  delay(250);
  LeseTMCStatus(true);
  delay(250);

  // Prüfe, ob AutoTune durchgeführt werden soll
  if (EEPROM.read(0) != 123) {
    // Lasse Stepper 400 Steps in Positive Richtung laufen und erwarte Rückgabewert
    SchrittzahlMaximum = autoHome(400);

    // Wenn der Rückgabewert kleiner als 50 Steps ist, scheint die Drehrichtung falsch zu sein
    if (SchrittzahlMaximum < 50) {
      // Lasse den Stepper nun also 400 Steps in negative Richtung laufen
      SchrittzahlMaximum = autoHome(-400);
    }
    if (SchrittzahlMaximum < -50) {
      Schrittzahl0Prozent = -100;
      Schrittzahl100Prozent = SchrittzahlMaximum + 40;
      Schrittzahl50Prozent = (Schrittzahl100Prozent - Schrittzahl0Prozent) / 2 + Schrittzahl0Prozent;
      Schrittzahl75Prozent = (Schrittzahl100Prozent - Schrittzahl50Prozent) / 2 + Schrittzahl50Prozent;
      Schrittzahl25Prozent = (Schrittzahl50Prozent - Schrittzahl0Prozent) / 2 + Schrittzahl0Prozent;
      EEPROM.write(0, 123);
    } else if (SchrittzahlMaximum > 50) {
      Schrittzahl0Prozent = 100;
      Schrittzahl100Prozent = SchrittzahlMaximum - 40;
      Schrittzahl50Prozent = (Schrittzahl100Prozent - Schrittzahl0Prozent) / 2 + Schrittzahl0Prozent;
      Schrittzahl75Prozent = (Schrittzahl100Prozent - Schrittzahl50Prozent) / 2 + Schrittzahl50Prozent;
      Schrittzahl25Prozent = (Schrittzahl50Prozent - Schrittzahl0Prozent) / 2 + Schrittzahl0Prozent;
      EEPROM.write(0, 123);
    }
    eepromWriteInt(2, Schrittzahl0Prozent);
    eepromWriteInt(4, Schrittzahl25Prozent);
    eepromWriteInt(6, Schrittzahl50Prozent);
    eepromWriteInt(8, Schrittzahl75Prozent);
    eepromWriteInt(10, Schrittzahl100Prozent);
    eepromWriteInt(12, SchrittzahlMaximum);
  } else {
    // Wenn nicht, nehme Werte aus dem EEPROM
    Schrittzahl0Prozent = eepromReadInt(2);
    Schrittzahl25Prozent = eepromReadInt(4);
    Schrittzahl50Prozent = eepromReadInt(6);
    Schrittzahl75Prozent = eepromReadInt(8);
    Schrittzahl100Prozent = eepromReadInt(10);
    SchrittzahlMaximum = eepromReadInt(12);
  }

  digitalWrite(IO_Enable, HIGH);
  stepper.setCurrentPosition(0);
  stepper.setSpeed(1000);

  // Motorstrom auf passende Werte für den Betrieb setzen
  stepper_driver.setRunCurrent(60);
  stepper_driver.setHoldCurrent(40);
}

void loop() {
  // Lese Eingänge alle 100 ms
  if (millis() - MillisLeseEingaenge >= IntervalLeseEingaenge) {
    MillisLeseEingaenge = millis();
    LeseEingaenge();
  }

  // Lese Eingänge alle 250 ms
  if (millis() - MillisSchreibe >= IntervalSchreibe) {
    MillisSchreibe = millis();
    if ((stepper.distanceToGo() * stepper.distanceToGo()) < 4) {
      SchreibeDisplay();
    }
  }

  // LED blinken lassen
  if (millis() - MillisLED >= IntervalLED) {
    MillisLED = millis();
    if (map(Sperrgrad, Schrittzahl0Prozent, Schrittzahl100Prozent, 0, 100) >= 60) {
      if (digitalRead(IO_LED_Sperrgrad))
        digitalWrite(IO_LED_Sperrgrad, LOW);
      else
        digitalWrite(IO_LED_Sperrgrad, HIGH);
    } else {
      digitalWrite(IO_LED_Sperrgrad, LOW);
    }
  }

  // Prüfe ob Abschaltgründe vorliegen
  if (ZustandBremse || ZustandHandbremse || ZustandTemperatur) {
    // Wenn Bremse, Handbremse oder Übertemperatur vorhanden sind, Stepper stromlos machen
    digitalWrite(IO_Enable, HIGH);
    // Sofern der Zustand gewechselt hat, Position des Steppers zurücksetzen
    if (!ZustandBearbeitet) {
      ZustandBearbeitet = true;
      stepper.setCurrentPosition(0);
    }
  } else if (ZustandHauptschalter) {
    // Wenn der Hauptschalter aus ist, Stepper und Pumpe stromlos machen
    digitalWrite(IO_Enable, HIGH);
    digitalWrite(IO_Pumpe, LOW);
    // Sofern der Zustand gewechselt hat, Position des Steppers zurücksetzen
    if (!ZustandBearbeitet) {
      ZustandBearbeitet = true;
      stepper.setCurrentPosition(0);
      stepper.setSpeed(1000);
    }
  }
  // Kein Abschaltgrund liegt vor
  // Stepper aktivieren, Vorladepumpe einschalten
  else if (!ZustandBremse && !ZustandHandbremse && !ZustandHauptschalter && !ZustandTemperatur && !TMCFehler) {
    digitalWrite(IO_Enable, LOW);
    digitalWrite(IO_Pumpe, HIGH);
    stepper.moveTo(SperrgradKompensiert);
    stepper.setSpeed(1000);
    stepper.runSpeedToPosition();

    // Sofern der Zustand gewechselt hat, Stepper aktivieren und zur alten Position fahren
    if (!ZustandBearbeitet) {
      stepper.setCurrentPosition(0);
      stepper.setSpeed(1000);
      ZustandBearbeitet = true;
      stepper.moveTo(SperrgradKompensiert);
      stepper.setSpeed(1000);
      while (stepper.currentPosition() != SperrgradKompensiert) {
        stepper.runSpeedToPosition();
      }
    }
  }

  // Fehler im Treiber des Schrittmotors
  if (TMCFehler) {
    // Pumpe und Stepper abschalten
    digitalWrite(IO_Enable, HIGH);
    digitalWrite(IO_Pumpe, LOW);
  }

  // Fehlerstatus abfragen
  // Wenn der Diagnosepin der Pumpe Low ist, während die Pumpe angesteuert wird, gibt es einen Kurzschluss
  if (!digitalRead(IO_Diagnose_Pumpe) && digitalRead(IO_Pumpe)) {
    PumpeFehler = true;
    // Pumpe und Stepper abschalten
    digitalWrite(IO_Enable, HIGH);
    digitalWrite(IO_Pumpe, LOW);
    u8g2.firstPage();
    do {

      u8g2.setFont(u8g2_font_helvB14_tr);
      u8g2.setCursor(0, 15);
      u8g2.print(F("Fehler!"));
      u8g2.setCursor(0, 25);
      u8g2.setFont(u8g2_font_helvR08_tf);
      u8g2.print(F("Vorladepumpe"));

    } while (u8g2.nextPage());
  } else {
    PumpeFehler = false;
  }



  // Temperaturkompensation einbauen
  // Beachten, ob Stepper in positive oder negative Richtung läuft
  if (Schrittzahl0Prozent < 0) {
    SperrgradKompensiert = Sperrgrad - TempOffset;
    if (SperrgradKompensiert < (SchrittzahlMaximum + 5))
      SperrgradKompensiert = SchrittzahlMaximum + 5;
  } else {
    SperrgradKompensiert = Sperrgrad + TempOffset;
    if (SperrgradKompensiert > (SchrittzahlMaximum - 5))
      SperrgradKompensiert = SchrittzahlMaximum - 5;
  }
}

void SchreibeDisplay() {
  if (!PumpeFehler && !TMCFehler) {
    int BreiteBox = map(Sperrgrad, Schrittzahl0Prozent, Schrittzahl100Prozent, 0, 118);
    u8g2.firstPage();
    do {
      u8g2.drawHLine(0, 0, 128);
      u8g2.setFont(u8g2_font_helvR08_tf);
      u8g2.setCursor(5, 11);
      u8g2.print(F("HALDEX"));

      u8g2.drawFrame(5, 15, 118, 15);
      u8g2.drawLine(75, 15, 75, 16);
      u8g2.drawLine(75, 19, 75, 20);
      u8g2.drawLine(75, 23, 75, 24);
      u8g2.drawLine(75, 27, 75, 28);

      u8g2.setCursor(5, 47);

      if (!ZustandBremse && !ZustandHandbremse && !ZustandHauptschalter && !ZustandTemperatur) {

        u8g2.drawBox(5, 15, BreiteBox, 15);
        if (BreiteBox >= 71) {
          u8g2.setDrawColor(0);
          u8g2.drawLine(75, 15, 75, 16);
          u8g2.drawLine(75, 19, 75, 20);
          u8g2.drawLine(75, 23, 75, 24);
          u8g2.drawLine(75, 27, 75, 28);
          u8g2.setDrawColor(1);
        }

        u8g2.setFont(u8g2_font_helvB14_tr);
        u8g2.print(map(Sperrgrad, Schrittzahl0Prozent, Schrittzahl100Prozent, 0, 100));
        u8g2.setFont(u8g2_font_helvR08_tf);
        u8g2.print(F(" %"));
      } else {
        if (ZustandHandbremse) {
          u8g2.print(F("Handbremse bet"));
          u8g2.write(0xE4);
          u8g2.print(F("tigt"));
        }
        if (ZustandBremse) {
          u8g2.print(F("Bremse bet"));
          u8g2.write(0xE4);
          u8g2.print(F("tigt"));
        }
        if (ZustandHauptschalter) {
          u8g2.print(F("Hauptschalter aus"));
        }
        if (ZustandTemperatur) {
          u8g2.write(0xDC);
          u8g2.print(F("bertemperatur"));
        }
      }

      u8g2.drawHLine(0, 49, 128);
      u8g2.setFont(u8g2_font_helvR08_tf);
      u8g2.setCursor(5, 61);
      u8g2.write(0xD6);
      u8g2.print(F("ltemperatur "));
      u8g2.print(Temperatur, 0);
      u8g2.print(F(" "));
      u8g2.write(0xB0);
      u8g2.print(F("C"));
      u8g2.drawHLine(0, 63, 128);

    } while (u8g2.nextPage());
  }
}

void LeseEingaenge() {
  // Lese Eingaenge
  BerechneTemperatur(analogRead(IO_Temperatur));
  Sperrgrad = BerechneSperrgradPoti(analogRead(IO_Eingabe_Poti));

  // Anpassung des Sperrgrades über die Öltemperatur:
  // Bei wärmer werdendem Öl, wird das Ventil weiter geschlossen
  TempOffset = (Temperatur - 20) / 4;

  // Sobald das Öl 100°C überschreitet, wird das Ventil jedoch geöffnet
  if (Temperatur > 100 && ZustandTemperatur == false) {
    ZustandTemperatur = true;
    ZustandBearbeitet = false;
  } else if (Temperatur <= 95 && ZustandTemperatur == true) {
    ZustandTemperatur = false;
  }

  LeseTMCStatus(false);
}

void LeseTMCStatus(bool PruefeOpenLoad) {
  TMCStatus = stepper_driver.getStatus();

  if (TMCStatus.over_temperature_shutdown == 1) {
    TMCFehler = true;
    u8g2.firstPage();
    do {
      u8g2.setFont(u8g2_font_helvB14_tr);
      u8g2.setCursor(0, 15);
      u8g2.print(F("Fehler!"));
      u8g2.setCursor(0, 40);
      u8g2.setFont(u8g2_font_helvR08_tf);
      u8g2.print(F("Übertemperatur"));
      u8g2.setCursor(0, 50);
      u8g2.print(F("Motortreiber"));
    } while (u8g2.nextPage());
  }

  if (TMCStatus.short_to_ground_a == 1 || TMCStatus.short_to_ground_b == 1) {
    TMCFehler = true;
    u8g2.firstPage();
    do {
      u8g2.setFont(u8g2_font_helvB14_tr);
      u8g2.setCursor(0, 15);
      u8g2.print(F("Fehler!"));
      u8g2.setCursor(0, 40);
      u8g2.setFont(u8g2_font_helvR08_tf);
      u8g2.print(F("Kurzschluss nach Masse"));
    } while (u8g2.nextPage());
  }

  if (TMCStatus.low_side_short_a == 1 || TMCStatus.low_side_short_b == 1) {
    TMCFehler = true;
    u8g2.firstPage();
    do {
      u8g2.setFont(u8g2_font_helvB14_tr);
      u8g2.setCursor(0, 15);
      u8g2.print(F("Fehler!"));
      u8g2.setCursor(0, 40);
      u8g2.setFont(u8g2_font_helvR08_tf);
      u8g2.print(F("Kurzschluss Stepper"));
    } while (u8g2.nextPage());
  }
  if ((TMCStatus.open_load_a == 1 || TMCStatus.open_load_b == 1) && PruefeOpenLoad) {
    TMCFehler = true;
    u8g2.firstPage();
    do {
      u8g2.setFont(u8g2_font_helvB14_tr);
      u8g2.setCursor(0, 15);
      u8g2.print(F("Fehler!"));
      u8g2.setCursor(0, 40);
      u8g2.setFont(u8g2_font_helvR08_tf);
      u8g2.print(F("Unterbrechung Stepper"));
    } while (u8g2.nextPage());
  }
}

void BerechneTemperatur(int temp) {
  int Vo = map(temp, 0, 1023, 1023, 0);
  R2 = R1 * (1023.0 / (float)Vo - 1.0);
  logR2 = log(R2);
  Temperatur = (1.0 / (c1 + c2 * logR2 + c3 * logR2 * logR2 * logR2));
  Temperatur = Temperatur - 273.15;
}

int BerechneSperrgradPoti(int poti) {
  float SperrgradRAW = map(poti, 0, 1020, 0, 100);
  SperrgradRAW = SperrgradRAW / 100;
  float SperrgradRAWKorrigiert = 0.9 * (SperrgradRAW * SperrgradRAW * SperrgradRAW) - 1.3 * (SperrgradRAW * SperrgradRAW) + 1.35 * SperrgradRAW + 0.03;
  SperrgradRAWKorrigiert = SperrgradRAWKorrigiert * 100;
  int temp = (int)SperrgradRAWKorrigiert;
  if (temp < 0)
    temp = 0;
  temp = map(temp, 3, 98, Schrittzahl0Prozent, Schrittzahl100Prozent);
  return temp;
}

int autoHome(int pos) {
  digitalWrite(IO_Enable, LOW);
  // Fahre zur angegebenen Position
  stepper.moveTo(pos);
  // Fahre langsam
  stepper.setSpeed(400);
  // Die ersten Schritte ohne StallGuard
  while (stepper.currentPosition() != (pos * 0.1)) {
    stepper.runSpeedToPosition();
  }
  // Wenn der Stepper in Bewegung ist, StallGuard laufend auslesen
  while (stepper.currentPosition() != pos) {
    stepper.runSpeedToPosition();
    int stall_guard_result = stepper_driver.getStallGuardResult();
    // Wenn der Rückgabewert des Stallguards höher als die Schwelle ist, blockiert der Motor
    if ((stall_guard_result < (STALL_GUARD_THRESHOLD * 2))) {
      // Ermittelten Wert nach oben geben
      return stepper.currentPosition();
    }
  }
}

void eepromWriteInt(int adr, int wert) {
  byte low, high;
  low = wert & 0xFF;
  high = (wert >> 8) & 0xFF;
  EEPROM.update(adr, low);
  EEPROM.update(adr + 1, high);
  return;
}

int eepromReadInt(int adr) {
  byte low, high;
  low = EEPROM.read(adr);
  high = EEPROM.read(adr + 1);
  return low + ((high << 8) & 0xFF00);
}