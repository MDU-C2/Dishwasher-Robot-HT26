# Dishwasher Robot – Gripper Control (version 1.4)

Modulär styrning av nio servon på PCA9685 kanal **0–8**, via en Arduino UNO R4 WiFi och ett modernt PySide6-gränssnitt på macOS. Kanalerna 0–5 är de befintliga servona; kanal 6–8 är tre kontinuerliga 360°-servon för fingrarna. Den manuella `SET`-vägen behåller 0–180°-kommandon, medan joystickvägen är proportionell hastighetsstyrning.

## 1. Koppla hårdvaran

- Arduino UNO R4 WiFi: **SDA → SDA**, **SCL → SCL** på PCA9685.
- Arduino **5V → VCC (logic)** på PCA9685, **GND → GND**.
- Externt servoaggregat (korrekt spänning för dina servon): **plus → V+**, **minus → GND** på PCA9685.
- **Gemensam jord** mellan Arduino, PCA9685 och servoaggregatet.
- Servo 1–6 → PCA9685 kanal 0–5: signal på PWM, röd på V+, svart/brun på GND (kontrollera servoledningarnas färger).
- Finger-servo 1–3 → PCA9685 kanal 6–8. För kontinuerliga servon stängs PCA9685-utgången av helt vid stopp; riktning och hastighet styrs från stoppets centerreferens mot min/max-puls.
- FSR: koppla sensorn via lämplig spänningsdelare till **A1**. GUI:t visar spänningen beräknad från ADC-värdet med 5,00 V referens; använd gemensam GND och överskrid inte Arduino-ingångens spänning.
- USB-kabel mellan Arduino och dator. **Mata inte servona via Arduinons 5V-pin.**
- Joystick: **VRX → A2**, **VRY → A3**, **SW → D2**, VCC → 5V, GND → GND.

### Säkerhet

Arduino skickar alla nio kanaler till 90° vid start. För kanal 6–8 betyder 90°-kommandot stopp. Testa först utan påkopplade tendon-linor eller annan last eftersom rörelsen kan bli abrupt. Håll en fysisk möjlighet att bryta servomatningen tillgänglig. Programmets frånkopplingsknapp stoppar inte nödvändigtvis en positionsservo; den kan fortsätta hålla sin senast begärda position.

**Vinklar:** 0–180° är kommandovärden. Verklig position kan avvika utan en encoder/positionssensor. Varje kanal har tre kalibreringspunkter: 0°/min, 90°/center och 180°/max. Arduino använder styckvis linjär interpolation mellan punkterna. Värdena sparas individuellt på Mac med Qt:s QSettings och skickas automatiskt till Arduino nästa gång du ansluter.

**Varning:** Programmet tillåter 600–2400 µs som ett begränsat kalibreringsfönster, men detta är inte en tillverkargaranti för säkra ändlägen. Positionsservon börjar på 1000/1500/2000 µs och 360°-servon på 900/1500/2100 µs. Ändra bara ett fält i taget i 10 µs-steg. Ett mekaniskt stopp, surrande eller tydligt ökad ström är signal att backa omedelbart. Klicka **Avbryt / 90°** för att lämna ett testläge.

Den [Waveshare MG996R-produktsida](https://www.waveshare.com/product/robotics/motors-servos/servos/mg996r-servo.htm) och [Electrokit-sidan för exakt produkt](https://www.electrokit.com/analogt-servo-11kg-0.18s-metall-stort-moment) anger 4,8–6,0 V, 180° och servoets moment/hastighet, men anger ingen verifierad 0°/180°-pulsbredd. Därför behandlar projektet 500–2500 µs som en vanlig hobbyservo-uppgift, inte som en säker specifikation för just dessa exemplar. [Adafruits PCA9685-exempel](https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library/blob/master/examples/servo/servo.ino) visar 600–2400 µs som ett justerbart exempel och säger att ändarna måste anpassas per servo.

PCA9685 körs med 50 Hz. `writeMicroseconds()` omvandlar mikrosekunder till 12-bitars ticks med den nominella 25 MHz-oscillatorn. Oscillatorn varierar mellan exemplar; högsta precision kräver mätning av PWM-signalen med oscilloskop och uppdatering av `PCA9685_OSCILLATOR_HZ` i firmware.

## 2. Ladda upp Arduino-programmet (en gång)

1. Öppna **Arduino IDE** och välj **Arduino UNO R4 WiFi** och rätt port.
2. Installera biblioteket **Adafruit PWM Servo Driver Library** via Library Manager (inklusive dess beroenden).
3. Öppna `firmware/GripperController/GripperController.ino` och välj **Upload**.
4. Stäng **Serial Monitor** innan du ansluter från Python. Båda programmen kan inte använda samma seriella port samtidigt.

**Om du har äldre firmware installerad:** Ladda upp den nya `.ino`-filen en gång. Den nya GUI-versionen skickar `CAL` med tre puls-punkter och kräver den uppdaterade firmware-versionen.

## 3. Installera Python-paket på Mac

Kör detta i Terminal i projektmappen (där `main.py` ligger):

```bash
python3 --version
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
python main.py
```

Använd en modern Python-version som stöds av PySide6, exempelvis Python 3.11–3.13. Efter installation kan du starta via `python main.py` eller dubbelklicka på `start_mac.command` (macOS kan be dig bekräfta första öppningen). **Ingen Arduino IDE behövs när du kör GUI:t efter första uppladdningen.**

## 4. Så använder du appen

1. Välj USB-port i listan (på Mac oftast `/dev/cu.usbmodem...`).
2. Klicka på **Anslut**. Ge kortet en stund: Arduino kan starta om när porten öppnas.
3. Dra reglagen på positionsservona kanal 0–5, eller skriv en vinkel direkt i rutorna. För Finger 1–3 håller du inne **Bakåt** eller **Framåt**; när knappen släpps skickas stopp.
4. Använd snabbvalen **0° / 90° / 180°**, eller **Alla till 90°**.
5. Det gröna ACK-meddelandet betyder att **Arduino accepterade målvinkeln**, inte att den verkliga servovinkeln är uppmätt.
6. FSR-rutan under **READY FOR EXPANSION** visar spänningen live från A1, tillsammans med det underliggande råvärdet. Det är en spänning, inte en kraftenhet.

## 4.1 Joystickstyrning

Joysticken läses lokalt av Arduino för låg latens. Vid uppstart kalibreras X/Y-neutralen automatiskt medan joysticken lämnas orörd. Standard-deadzonen är ±40 ADC-räkningar och knappen avstudsas i 200 ms. Knappen är aktiv låg med `INPUT_PULLUP`; om den fysiska testkopplingen faktiskt ger 0=släppt och 1=tryckt ändras `JOYSTICK_BUTTON_ACTIVE_LOW` till `false` i firmware.

Växla **Fingerläge (kanal 6–8)** för att välja joystickgrupp. I läget **Övriga** finns tre standardlägen:

| Läge | X-axel | Y-axel |
|---|---|---|
| 1 · Gripper group | kanal 0 | kanal 1 |
| 2 · Middle group | kanal 2 | kanal 3 |
| 3 · End group | kanal 4 | kanal 5 |

I **Fingerläge** används joystickens X-axel och Y-axeln är avsiktligt inaktiv:

| Läge | X-axel | Y-axel |
|---|---|---|
| 1 · Finger 1 | kanal 6 | ej använd |
| 2 · Finger 2 | kanal 7 | ej använd |
| 3 · Finger 3 | kanal 8 | ej använd |
| 4 · Alla fingrar | kanal 6 + 7 + 8 | ej använd |

Joysticken styr kontinuerliga servon proportionellt: för kanal 6–8 stängs PCA9685-signalen av helt vid stopp och hastigheten ökar linjärt ju längre spaken förs från centerreferensen. Maximal hastighet använder kalibreringens min/max-puls och kan justeras med `JOYSTICK_MAX_SPEED` i firmware. Lägesbyte stoppar först den gamla gruppen och kräver sedan att joysticken återgår till neutral innan det nya läget aktiveras.

Tryck på joystick-knappen för nästa läge inom den valda gruppen. Manuell styrning och joystickstyrning konkurrerar inte: det senaste manuella slider- eller hållknappskommandot stänger av joystickläget. Aktivera **Joystick aktiv** igen och släpp joysticken till neutral för att återta kontrollen. Kanalgrupperna finns i `JOYSTICK_GROUPS` i `robot_control/config.py` och motsvarande `joystickModes` i firmware.

## 4.2 Kalibrera servonas rörelseomfång

Under servo-sliders finns panelen **SERVO CALIBRATION / PWM** med tre individuella pulsbreddspunkter per kanal. Kalibreringen tillämpas inte när kanalen står i en annan målvinkel än 90°.

1. Koppla helst loss mekanisk last och tendon-linor.
2. Anslut Arduino. Vid ny anslutning går alla kanaler till den säkra standardpositionen **90°**; tidigare GUI-målvinklar spelas inte upp automatiskt. För Finger 1–3 stängs utgången av.
3. Välj rätt kanal och kontrollera startvärdena. Positionsservon använder normalt **1000 / 1500 / 2000 µs**; Finger 1–3 använder **900 / 1500 / 2100 µs** som starkare startområde.
4. Tryck **Testa 90°**. Ändra endast ett värde, exempelvis min från 1000 till 990 µs, och tryck **Spara / tillämpa**. Kalibreringen skickas först när servot står på 90°.
5. Tryck **Testa 0°** för positionsservo eller **Testa bakåt** för 360°-servo. Kontrollera rörelsen utan att vänta på ett mekaniskt stopp. Justera min-värdet i 10 µs-steg och tillämpa igen vid behov.
6. Tryck **Avbryt / 90°** mellan ändpunkterna. Upprepa motsvarande försiktigt för **Testa 180°** eller **Testa framåt**.
7. Justera centerpulsen separat om positionsservots mittläge blir fel eller om 360°-servot kryper i stopp-läget. Centerpulsen är inte längre tvingad till mittpunkten mellan min och max.
8. När varje punkt är godkänd, lämna kanalen på 90° och kalibrera nästa kanal. Värdena sparas individuellt för kanal 0–8 och återanvänds efter omstart.

Arduino får meddelanden som `CAL,0,900,1500,2100`, bekräftar `ACKCAL,0,900,1500,2100` och fortsätter därefter ta emot `SET,0,90`. Dessa gränser ligger i RAM på Arduino och återställs vid omstart tills datorn ansluter på nytt; GUI:t skickar dem igen från QSettings.


## Felsökning

- **Porten saknas:** kontrollera USB-kabel, rätt kort, och att USB fungerar för data. Tryck ”Uppdatera portar”.
- **Ingen READY / bara CONNECTING:** kontrollera I²C-kopplingen och att PCA9685 svarar på adress `0x40`. Arduino-programmet förblir i felläge om PCA9685 inte hittas.
- **Servot rör sig inte:** kontrollera extern V+, gemensam GND och kanalsignal. För kanal 6–8 ska 90° vara stopp; små avvikelser runt mittpulsen kan behöva kalibreras bort.
- **Servot når inte 180°:** det kan vara normalt för ett individuellt MG996R-exemplar eller för mekaniken. Kontrollera först 50 Hz, matningen och att inget mekaniskt stopp nås. Kalibrera sedan 0° och 180° separat.
- **Kalibrering ändrar inget:** ladda upp den nya `.ino`-filen, kontrollera att Arduino svarar med fyra värdefält i `ACKCAL` (`kanal,min,center,max`), och tryck **Spara / tillämpa** när kanalen står på 90°.
- **macOS-dialog om Python/Qt:** PySide6-applikationen körs lokalt och kräver ingen server.

## Projektstruktur

```text
gripper-control/
├── main.py                         startar desktop-GUI
├── start_mac.command               genväg på Mac efter installation
├── requirements.txt
├── robot_control/
│   ├── config.py                   kanaler, namn, standardvärden
│   ├── calibration.py              trepunktsmodell och validering
│   ├── protocol.py                 gemensamt meddelandeformat
│   ├── serial_worker.py            seriell kommunikation separat från GUI
│   ├── servo_card.py              återanvändbar servo-widget
│   ├── theme.py                    utseende/färger
│   └── window.py                   layout, knappar, sensorsektion
└── firmware/
    └── GripperController/
        └── GripperController.ino     Arduino/PCA9685
```

## Senare: fler servon och sensorer

- **Fler servon:** lägg till kanal/namn i `robot_control/config.py`, ändra `SERVO_COUNT`/`SERVO_CHANNELS` och lägg till kalibreringsposter i Arduino-skissen. UI:t genererar kort från konfigurationen och visar två kolumner automatiskt.
- **FSR:** den första sensorn läses redan på A1 via lämplig spänningsdelare och skickas som `FSR,1,512,2501` (råvärde och millivolt). Spänningsreferensen är 5,00 V i firmwarekonstanten `FSR_ADC_REFERENCE_MV`; lägg senare till kraftkalibrering om värdena ska motsvara gram eller newton.
- **LiDAR:** anslut SDA/SCL till samma I²C-buss som PCA9685, kontrollera rätt matning/logicnivåer och unik I²C-adress för ert LiDAR-breakout. Lägg LiDAR-kod i Arduino och skicka exempelvis `DIST,43` för 43 mm. Val av bibliotek och mätintervall beror på exakt VL53-modell.
- **Slip detection:** använd samma meddelandestruktur som för de andra sensorerna.

**Rekommenderat på sikt:** lägg snabba säkerhetsgränser och reglering som måste fungera oberoende av USB-förbindelsen på Arduino-sidan. Använd Python för högre nivåns logik och visualisering.
