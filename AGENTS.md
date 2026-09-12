# AGENTS.md

Leitfaden für KI-Agenten, die in diesem Repository arbeiten.

## Projekt

`TPUart` ist eine PlatformIO-C++-Library, die den Datalink Layer für die
TP-UART-Kommunikation in KNX-Anlagen umsetzt. Zielplattformen sind RP2040 und ESP32, und
die Interfaces sprechen dort im Wesentlichen das jeweilige Hersteller-SDK an - pico-sdk
(`uart_*`, `dma_*`, `critical_section_t`) beziehungsweise ESP-IDF (`uart_driver_install`
und Verwandte). Vom Arduino-Framework kommen nur die Zeitfunktionen und, im generischen
Adapter `ArduinoSerial<T>`, die `Stream`-artige Klasse des Aufrufers. Welche Cores und
Plattformversionen dafür gezogen werden, steht in `platformio.ini` und nur dort.

In **`docs/`** liegen drei Herstellerdokumente, und sie sind die Autorität für
Protokollzeiten und Opcodes: **`docs/Onsemi_NCN5130.pdf`** (OnSemi, der Chip, der
tatsächlich auf der Testhardware sitzt), **`docs/Siemens_TPUART.pdf`** (Siemens
TPUART2, technisches Handbuch, 2012) und **`docs/Siemens_TPUART2.pdf`** (Siemens
TP-UART **2+**, 2013). Sie sind sich nicht in allem einig - wo sie abweichen, ist das
ein echter Unterschied zwischen den Chips, kein Dokumentationsfehler (siehe
`BcuType`). Zwei Dinge zum 2+ sind wissenswert: seine Servicetabelle ist mit der des
älteren TPUART2 identisch (dieselben Grenzen `81-BE` / `47-7F`, kein
`U_L_DataOffset`), seine **Host-Baudrate ist aber 115200 oder 19200, ausgewählt über
den BDS-Pin** - nicht 38400. `begin(BcuType::Tpuart2)` probiert nur 19200, ein auf
115200 gestrappter 2+ würde also nie erkannt.

## Aktueller Stand (Interface-Schicht)

`src/TPUart/Interface/` enthält ein abstraktes UART-Interface und vier
Implementierungen, alle bisher ausgebaut:

- **`Abstract.h`** - die gemeinsame Schnittstelle. Rein pollend
  (`begin/end/flush/available/availableForWrite/read/write/overflow`).
  **`available()` liefert eine Anzahl, keinen Wahrheitswert** - die Schicht darüber muss
  wissen, wie weit sie hinter dem Bus liegt (siehe die Quittungsunterdrückung weiter
  unten), nicht nur, ob überhaupt etwas da ist.
  **Auf der Leseseite darf ein Interface nicht STAPELN.** Die Schicht darüber misst Pausen
  daran, wann ein Byte *sichtbar* wird, nicht daran, wann es auf der Leitung war - ein
  zurückgehaltenes Byte lügt also über seine Ankunftszeit, und die Pausenerkennung ist das,
  was ein Telegramm vom nächsten trennt. Hardware-FIFOs sind die Falle: der UART-Treiber der
  ESP-IDF lässt Empfangenes im 128-Byte-Hardware-FIFO liegen, bis 120 Bytes beisammen sind
  oder die Leitung 10 Symbolzeiten still war (5,7ms bei 19200 - mehr als die 2,6ms-Schwelle
  fürs Frame-Ende), was ein 263-Byte-Echo bei genau 120 Bytes abschnitt. `ESP32::begin()`
  setzt deshalb `uart_set_rx_full_threshold(1)` und `uart_set_rx_timeout(1)`. Auf dem RP2040
  ergibt sich das von selbst, weil DMA jedes Byte sofort ablegt - weshalb die Annahme so
  lange unbemerkt blieb. **Für `ArduinoSerial` ist das nie geprüft worden** - ob der
  eingepackte Kern beim Empfang stapelt, ist offen, und die Pausenerkennung hängt daran.
  Das Fehlerbild ist wissenswert, denn es meldet sich nicht selbst: die Pausenerkennung ist
  keine Prüfung, sondern das *Axiom* - sie legt überhaupt erst fest, wo ein Frame endet, und
  dahinter gibt es kein zweites, unabhängiges Signal, das einer falschen Grenze widersprechen
  könnte. Wo Redundanz vorhanden war, hat sie gehalten (jedes Phantomtelegramm kam als
  `INVALID` heraus, die Prüfsumme hat sie alle erwischt), aber **Steuerbytes haben keine
  Prüfsumme** - ein einzelnes Byte ist seine eigene Nachricht. Durchgekommen sind deshalb
  erfundene Chip-Zustände und, der scharfe Fall, ein erfundenes positives `L_Data.con`
  (`0x8B & 0x7F == 0x0B`), das `Transmitter::confirmed()` aufrief und den Sendeweg für einen
  Versand freigab, der keine Bestätigung hatte. Nachrüsten lässt sich das weiter unten nicht:
  nach einem abgeschnittenen Frame in den Resync statt in den Leerlauf zu gehen würde genau
  den Fall zerstören, für den die Schwelle auf 2600 gesenkt wurde (das `L_Data.con`, das nach
  einem verstümmelten Echo hinter einer Pause eintrifft, abgesichert durch zwei Testfälle).
  Der Schutz gehört dorthin, wo er jetzt liegt - das Interface darf nicht lügen.
  Beachte die Asymmetrie zu *unserer eigenen* Verspätung, denn sie erklärt, warum nur eines von
  beidem zerstörerisch ist. Auf der Leseseite in Rückstand zu geraten ist überhaupt erst
  einmal schwierig: was der Chip weiterreicht, kommt im Bustakt, ein Oktett je 1,354ms bei
  9600 Baud, gegen einen 500µs-Tick - und das schließt das Echo unseres eigenen Versands ein
  (263 Oktetts gemessen als ~355ms, also Busgeschwindigkeit, nicht Host-Geschwindigkeit). Nur
  Steuerbytes kommen im Takt der Host-Strecke, und sie kommen zu zweit, nicht zu Hunderten.
  Die Zeichenzeit des Hosts ist die Grenze der *Schreib*seite (siehe `Timer`). Und selbst
  wenn der Tick zurückfällt, meldet `available()` weiterhin den Rückstand, es wird also keine
  Stille vorgetäuscht und kein Frame zerschnitten: die Quittung kommt lediglich zu spät und
  wird unterdrückt - eingeschränkte Funktion, ehrliche Daten. Ein stapelndes Interface
  erfindet dagegen Stille, und das macht aus Nutzlast Steuerbytes.
  **`availableForWrite()` liefert eine Anzahl, keinen Wahrheitswert**, und `write()` darf
  nicht blockieren - der Aufrufer fragt vorher nach Platz. Die Anzahl gibt es, damit mehrere
  zusammengehörende Bytes reserviert und unmittelbar hintereinander abgesetzt werden können,
  ohne dass eine Quittung dazwischenrutscht; jede Implementierung muss deshalb die
  Schreibreihenfolge wahren.
  **Die Zahl muss in BEIDE Richtungen stimmen**, und beide Fehler sind hier gemacht worden.
  Die Zahl, die sie definiert, ist **`TPUART_TX_INTERFACE_BUFFER` (4)**, und sie steht in
  `Transmitter.h` - sie ist eine Anforderung des Protokolls, keine Eigenschaft irgendeiner
  Hardware, und sie ist aus den beiden Konstanten abgeleitet (`max`), aus denen sie folgt
  (`TPUART_CTRL_MAX_GROUP`, `TPUART_TX_ATOMIC_BYTES`). Die Interfaces leiten ihre Puffergröße
  davon ab, sie kann also nicht an einer Stelle wachsen und an der anderen zurückbleiben.
  - **Zu klein**: 0 oder 1 zu melden bricht eine 4-Byte-Steuersequenz *dauerhaft*, nicht nur
    verlangsamt - sie darf nicht zerteilt werden, bleibt also am Kopf der Warteschlange stehen
    und blockiert alles dahinter. Siehe `ArduinoSerial`.
  - **Zu groß**: ein Interface darf nie mehr als so viele Bytes *unter sich* zulassen, welche
    Puffer es auch hat. Was in einem tiefen FIFO liegt, verzögert alles Folgende, und das
    nächste Byte ist womöglich ein `U_Ackn.req` mit 2,8ms Frist (Siemens TP-UART 2+ S. 25).
    32 Byte in einem TX-FIFO sind 18ms bei 19200 - die Quittung käme nicht nur zu spät, der
    Chip hängte sie ans *nächste* Telegramm. Tiefer zu puffern bringt ohnehin nichts: der Bus
    nimmt ein Byte je 1,354ms, und was länger wartet, ist zu spät.
  Zwei Wege erfüllen beides, beide sind im Einsatz: ein kleiner Ring plus ein exaktes
  Hardware-Signal (`RP2040` - FIFO aus, das Halteregister ist damit ein Byte tief und
  `uart_is_writable` ist genau; Tiefe beweisbar <= 5), oder **Buchführung über die Leitung**
  (`ArduinoSerial`, `ESP32` - Bytezeit aus der Baudrate, 8E1 = 11 Bit, jedes übergebene Byte
  schreibt den Fahrplan fort, und was noch aussteht, folgt aus der Uhr). Das zweite braucht
  man, wenn die Schicht darunter ihren Füllstand verbirgt, was beide tun. Eine
  Callback-/Benachrichtigungs-API gibt es bewusst **nicht**: Interrupts können global
  vorübergehend gesperrt werden, und eine IRQ-Zustellung genau einmal je Byte ist nicht
  garantiert (besonders mit DMA), der Datenpfad muss also immer pollend sein, nie schiebend.
  Interrupts werden ausschließlich als interne Implementierungssache innerhalb eines einzelnen
  Interfaces benutzt (siehe `RP2040`), nie um Verarbeitung in höheren Schichten anzustoßen.
- **`RP2040`** - Hardware-UART mit DMA-getriebenem Empfang in einen Ringpuffer, dessen Größe
  `TPUART_RP2040_RX_BUFFER_EXP` bestimmt (Vorgabe 8 = 256 Byte). Ein Gerät, dessen Hauptloop
  eine ganze Sekunde stehenbleiben kann, will mehr: der Bus liefert höchstens 738 Byte/s, mit
  `-D TPUART_RP2040_RX_BUFFER_EXP=11` (2048) ist das mit Reserve abgedeckt. Die frühere
  Diagnose-Env stellte genau das ein. Achtung: die Zahl, auf der diese Auslegung ursprünglich
  beruhte (~1745 Byte/s, die gesättigte 19200er Host-Strecke), ist der falsche Bezug - zitiere
  sie nicht als Anforderung.
  DMA ist der *einzige* Empfangspfad (kein reiner IRQ-Modus).
  **Zwei verschiedene Arten von Überlauf, und sie brauchen verschiedene Erkennung.** Ein
  Hardware-Overrun des UART (`UART_UARTRSR_OE_BITS`) heißt, dass DMA das Datenregister nicht
  rechtzeitig geleert hat. Ein *Ring*-Überlauf heißt, dass DMA unseren Leser überholt hat - die
  Hardware kam dort bestens mit, `OE` bleibt also sauber und taugt dafür nicht.
  Der Ringfall wird in `read()` gerastet (`_ringOverflow`) und von `overflow()` abgeholt. Die
  Rastung trägt: `read()` muss `_dmaReaderCount` korrigieren, wenn es das Überholen bemerkt,
  und zerstört damit genau die Zählerdifferenz, die die Bedingung geprüft hat - und `processRx()`
  fragt `overflow()` erst *nach* `read()`. Dort neu zu rechnen ergibt deshalb immer falsch, und
  genau so blieben Ringüberläufe früher vollständig unberichtet.
  Wenn `read()` korrigiert, setzt es beim **ältesten noch gültigen** Byte wieder auf
  (`dmaTransferCount() - BUFFER_SIZE + 1`), nicht beim neuesten. Zum neuesten zu springen wirft
  einen ganzen Ring voll noch lesbarer Bytes weg und vervielfacht den Verlust um die Puffergröße.
  Der DMA-Transferzähler (`TPUART_RP2040_TRANSFER_COUNT`) läuft ab, und die Maschine bliebe
  danach für immer stehen, er muss also neu gestartet werden. Früher war er der größte Wert, den
  die Hardware nimmt (`UINT32_MAX >> 1`, über einen Monat Dauerlast) - **bewusst klein gemacht**:
  `TPUART_RP2040_TRANSFER_COUNT_EXP` steht auf 20, also rund 24 Minuten bei voller Buslast, und
  `-D …_EXP=12` provoziert einen Neustart binnen Sekunden. Ein Pfad, der einmal im Monat läuft,
  ist ein Pfad, den nie jemand ausprobiert, und dieser muss Schreibposition und Zähler über den
  Neustart hinweg stimmig halten.
  Das ist gefahrlos, weil das Neustartfenster kurz ist: die DMA steht zwischen dem Ablauf des
  Zählers und dem nächsten `checkRestart()`, also höchstens einen Tick (500µs), und in der Zeit
  liefert der Bus höchstens **ein** Byte (ein TP1-Zeichen dauert 1,354ms) - genau das, was das
  Halteregister des UART puffert. Für einen Overrun bräuchte es zwei Bytes im Fenster.
  Ablauf: der DMA-Fertig-Interrupt (`onDmaComplete()`) setzt nur ein Flag - keine Verarbeitung im
  ISR-Kontext - und der eigentliche Neustart (`restartDma()`) passiert aus dem normalen Pollpfad
  über `checkRestart()` in `available()`.
  Der Neustart ist **zeigererhaltend**: `_dmaReaderCount` wird *nicht* zurückgesetzt; stattdessen
  wird `_dmaTransferBase` auf die aktuelle absolute Summe gesetzt, damit die Zähler stimmig
  bleiben, und `TPUART_RP2040_TRANSFER_COUNT` wird auf ein Vielfaches der Puffergröße abgerundet,
  damit die Schreibposition im Ring nahtlos weiterläuft. Das ist wesentlich - den Lesezeiger
  zurückzusetzen würde stillschweigend jedes empfangene, aber noch ungelesene Byte im Ring
  verwerfen.
  Beachte `_dmaTransferBase = total` statt `+= TRANSFER_COUNT`: im Normalfall identisch (Zähler
  bei 0 abgelaufen), aber es übersteht auch einen Neustart, der ausgelöst wird, während der
  Zähler noch *läuft*. Die Additionsform würde den logischen Zähler vorausspringen lassen,
  während die physische Schreibposition stehen bleibt, und damit den Ringindex dauerhaft gegen
  die Wirklichkeit verschieben, sodass `read()` für immer Bytes von der falschen Stelle lieferte.
  `end()` benutzt aus demselben Grund `dma_channel_cleanup()` statt eines nackten
  `dma_channel_abort()` - ein Abort kann den Fertig-IRQ-Status des Kanals gesetzt lassen, was
  nach dem nächsten `begin()` einen unechten Neustart auslöste. Dieser Pfad läuft bei jedem
  Baudratenkandidaten.
  **Der Sendeweg läuft über einen Software-Ring** (`TPUART_RP2040_TX_BUFFER_SIZE`), nicht direkt
  in die Hardware. Zwei Gründe: `write()` rief früher `uart_tx_wait_blocking()`, was bis zu eine
  Bytezeit blockiert und in einem ISR nicht hinnehmbar ist; und die Hardware kann den mehrbytigen
  Vorlauf nicht anbieten, den `availableForWrite()` melden muss. Am PL011 schaltet
  `uart_set_fifo_enabled()` RX- und TX-FIFO gemeinsam, und der FIFO ist hier aus, das
  TX-Halteregister ist also ein Byte tief - daher der Software-Ring.
  Er ist **fest auf 4 Byte und bewusst nicht einstellbar**: mehr kann ein einzelner Tick nicht
  erzeugen - drei für ein Telegrammoktett (Offset, Position, Daten) plus eines für die Quittung
  zum gerade eintreffenden Telegramm. Alles darüber läge nur herum; die Hardware nimmt bei 19200
  Baud ohnehin ein Byte je ~0,52ms, und ein Byte, das länger im Puffer wartet, erreicht den Bus
  zu spät. Ein größerer Ring würde nur den Rückstand verstecken, den `availableForWrite()` melden
  soll.
  Das ist es, was die Vorfahrt der Steuercodes tragend macht statt kosmetisch: die längste
  Steuersequenz ist 4 Byte (`TPUART_CTRL_MAX_GROUP`), sie braucht also den Ring vollständig leer -
  und der Telegrammpfad greift sich drei Bytes, sobald drei frei sind, womit in einem 4-Byte-Ring
  nie vier frei würden. `processCtrlQueue()` liefert deshalb wahr ("der Sendeweg gehört mir in
  diesem Tick") **auch dann, wenn die Gruppe noch nicht passt**, damit der Telegrammpfad
  zurücksteht und der Ring leerläuft. Vorfahrt heißt hier Platz schaffen, nicht bloß zuerst
  dranzukommen.
  **Geleert wird der Ring von einem TX-Interrupt** (`onTxInterrupt()`), und deshalb gibt es ihn
  in dieser Form. Früher lief `pumpTx()` (davor `drainTx()`) nur aus `write()`/
  `availableForWrite()`, also nur zum Tick, und zwischen den Ticks hielten nur die zwei Bytes in
  der Hardware die Leitung beschäftigt - und je Tick passt nur *eines* nach, weil das
  Halteregister erst frei wird, wenn das Schieberegister das vorige übernommen hat. Die Messung
  dazu steht im Abschnitt zum Tick-Intervall. **Vertieft wird dadurch nichts**: der Ring bleibt
  4, die Hardware bleibt 2, das Quittungsbudget in `Transmitter.h` bleibt unangetastet. Wer
  daraus "Interrupt, also machen wir auch den FIFO an" folgert, bricht diese Zusage.
  Nebenläufigkeit: der Ring hat jetzt **zwei** Kontexte - der Tick füllt, der ISR leert - daher
  `critical_section_t _txSection`. Ein sperrfreies "ISR scharf"-Flag geht hier nicht: beide Seiten
  müssen `uart_is_writable()` *prüfen* und abhängig vom Ergebnis schreiben, und der M0+ hat dafür
  kein CAS. Es muss eine Critical Section sein und nicht `save_and_disable_interrupts()`, weil im
  `Loop1`-Modus der Erzeuger auf dem anderen Kern läuft. Ist die IRQ-Leitung schon belegt
  (arduino-pico installiert in `SerialUART::begin()` einen exklusiven Handler auf demselben UART),
  wird die Beschleunigung stillschweigend übersprungen und `TXIM` nie gesetzt - das Interface
  verhält sich dann genau wie vorher.
  **Eine Korrektur an der Begründung, die hier früher stand**: behauptet wurde, der RX-FIFO
  *müsse* aus bleiben, weil die DMA-Anforderung sonst erst an der IFLS-Schwelle käme und die
  letzten Bytes eines Frames liegen blieben. Das ist nicht belegt - das Datenblatt beschreibt
  beide Signale (§4.2.5 S. 424: `uartrxdmasreq` ab einem Zeichen, `uartrxdmabreq` ab der
  Wassermarke), sagt aber nirgends, welches als `DREQ_UARTx_RX` nach außen geführt ist, und das
  SDK schweigt ebenfalls. FIFO-Betrieb mit DMA ist ausdrücklich vorgesehen; nirgends steht eine
  Anweisung, ihn abzuschalten. Geändert wurde trotzdem nichts: die Entscheidung fürs Abschalten
  stammt aus einer Beobachtung am laufenden Bus, und sie ohne erneute Prüfung an echter Hardware
  umzudrehen ist das Risiko nicht wert. Der TX-Interrupt **beantwortet** diese Frage nicht, er
  macht sie nur weniger dringend.
- **`ESP32`** - kein DMA. Recherchiert und bestätigt: UHCI (der einzige DMA-Weg für UART auf dem
  ESP32) hat auf dem klassischen ESP32 keine offizielle ESP-IDF-Treiberunterstützung (nur
  C3/C6/S3/P4). Benutzt das übliche `uart_driver_install`; die Ereigniswarteschlange des Treibers
  wird beim Pollen nicht-blockierend geleert, um FIFO-/Pufferüberläufe zu erkennen.
  `availableForWrite()` **meldete früher `uart_get_tx_buffer_free_size()` - bis zu 512 Byte**, der
  Fehler "zu groß" in Reinform: der Treiber hätte ein ganzes Telegramm an Host-Bytes geschluckt,
  und jede Quittung dahinter wäre hoffnungslos zu spät gekommen. Der Sendepuffer des Treibers
  lässt sich nicht auf 4 verkleinern (`uart_driver_install` erlaubt nur 0 oder mehr als eine
  FIFO-Länge), er wird deshalb nicht als Puffer, sondern als Durchlauf benutzt: dieselbe
  Buchführung über die Leitung wie im `ArduinoSerial` begrenzt, was übergeben wird, und der freie
  Platz des Treibers geht als zweite Schranke ein, damit `uart_write_bytes()` trotzdem nie wartet.
  **ISR-tauglich ist es weiterhin nicht**: `uart_write_bytes()` nimmt den TX-Mutex des Treibers.
  Heute unbestritten (ein einziger Schreiber), aber `tick()` auf dem ESP32 aus einem Interrupt zu
  treiben bräuchte einen eigenen Ringpuffer plus einen FreeRTOS-Task.
  **Die Pins sind hier die Falle, und sie haben einen Abend gekostet.** `uart_set_pin()` prüft
  nicht, ob ein Pin frei ist - es legt die Pin-Matrix um, und wenn dort der eingebaute Flash oder
  PSRAM hängt (ESP32-PICO-V3-02: GPIO 6-11 **sowie 16 und 17**; WROVER: 16/17), findet der nächste
  Cache-Miss keinen Flash mehr. Der Kern bleibt stehen, und da der Panic-Handler selbst erst aus
  dem Flash gelesen werden müsste, **kommt kein einziges Zeichen heraus**: nach 300ms setzt der
  Interrupt-Watchdog stillschweigend zurück (`rst:0x8 TG1WDT_SYS_RESET`). Ein Bootloop ganz ohne
  Ausgabe auf dem ESP32 heißt "zuerst die Pins prüfen" - im Quelltext ist nichts Falsches zu
  finden. Erschwerend kommt hinzu, dass der hängende Aufruf der unverdächtige ist:
  `uart_param_config()` kehrt zurück, `uart_driver_install()` meldet brav `rc=0`, und
  **`uart_set_pin()` kommt nie wieder**. Das frühere Diagnosewerkzeug hatte 16/17 fest verdrahtet,
  weil das die Arduino-Vorgabepins für `Serial2` sind (auf einem WROOM-32 auch richtig); die
  Verdrahtung des Entwicklungsboards ist **RX 37 / TX 5**. Die beiden Rollen sind nicht tauschbar:
  GPIO 34-39 sind beim klassischen ESP32 reine Eingänge, 37 kann also nur RX sein - vertauscht
  bleibt der Sendeweg stumm, ohne dass irgendwo ein Fehler auftaucht. Ein Aufrufer muss also Pins
  übergeben, die auf seinem Modul tatsächlich frei sind; der Konstruktor nimmt sie entgegen, und
  der Kommentar an `uart_set_pin()` beschreibt die Falle, weil die Quelle selbst nichts zeigt.
  Was daran über den Einzelfall hinausgeht: jene Env hatte monatelang warnungsfrei gebaut, ohne je
  auf echter Hardware zu laufen, und der Fehler war durch Lesen nicht zu finden - im Quelltext
  steht nichts Falsches, die Zahl 16 ist erst auf diesem Modul verkehrt. Genau dafür ist die Zeile
  "nur kompiliert, nie gelaufen" in den offenen Punkten da.
- **`ArduinoSerial<T>`** - allgemeiner Wrapper um beliebige `Stream`-artige Arduino-Klassen (z.B.
  `HardwareSerial`). Ein Template, damit es auch mit Kernen funktioniert, bei denen `Serial1`
  nicht wörtlich `HardwareSerial` heißt (z.B. arduino-pico) - daher das Muster
  `ArduinoSerial<decltype(Serial1)>`, das `test/test_tpuart/interface_check.cpp` auch ausdrücklich
  instanziiert.
  **`availableForWrite()` ist der schwierige Teil dieses Adapters, und er hat beide Fehler
  gemacht.** Zuerst zu klein: durchgereicht, was die Serial-Klasse meldet.
  `Print::availableForWrite()` liefert als Vorgabe 0, und `SerialUART` von arduino-pico liefert
  `(uart_is_writable(_uart)) ? 1 : 0`, also höchstens 1. Eine 4-Byte-Steuersequenz bekam damit nie
  ihren Platz, blieb am Kopf der Steuerwarteschlange stehen und blockierte den Telegrammpfad
  gleich mit. Beobachtet auf dem RP2040 als **Dauerlauf von `CTRL OVERFLOW`**, sobald eine eigene
  Adresse gesetzt war - `U_SetAddress.req` ist die erste Sequenz, die mehr als ein Byte braucht,
  weshalb dieser Adapter bis dahin unauffällig aussah.
  Zu groß ist der unsichtbare Gegenfehler: `SerialUART::write()` ruft `uart_putc_raw()` direkt in
  die Hardware, und `uart_init()` des SDK lässt den **TX-FIFO an - 32 Byte tief**,
  `uart_is_writable` bleibt also wahr, bis 32 Bytes drinliegen.
  Beides löst dieselbe Buchführung: Bytezeit aus der Baudrate, jedes übergebene Byte schreibt
  `_lineBusyUntil` fort, und was noch aussteht, folgt aus der Uhr. Gemeldet wird der Platz bis zur
  erlaubten Tiefe.
  **Dazu ein vier Byte tiefer eigener Sendepuffer**, und hier stand einmal das Gegenteil ("kein eigener
  Ring - die FIFO darunter ist der Ring"). Das widersprach der Anforderung in `Transmitter.h`, die einen
  solchen Puffer ausdrücklich verlangt, und es ging schief: `availableForWrite()` ist eine RESERVIERUNG,
  deren Einhaltung ohne Puffer an der eingepackten Klasse hängt - und die kann ablehnen, ohne dass diese
  Ebene den Grund kennt. `SerialPIO` tut es, wenn seine `CoreMutex` belegt ist (Reentranz zwischen Tick und
  Hauptkontext). Weil der Aufrufer Gruppen unteilbar absetzt, lag dann eine halbe Sequenz auf der
  Hostleitung: am Bus gemessen meldete der Chip `PE` in Serie und liess etwa jedes zwanzigste Telegramm
  unbestaetigt, jedes davon 10s Wachhund plus BCU-Reset. Gezaehlt hat es nichts - kein Überlauf, kein
  Verlust, keine Meldung. Der Puffer nimmt jetzt an, was zugesagt war, und schiebt es nach; die erlaubte
  Tiefe bleibt dieselbe, weil "noch bei uns" und "geschätzt noch unter uns" zusammen gezählt werden. Vor
  einer zugelassenen Quittung liegen damit nie mehr als drei Bytes, die 2,8ms-Frist bleibt also gedeckt.
  **Der Sendeweg dieses Adapters ist damit erstmals an echter Hardware gelaufen** - über `SerialPIO` in
  `tpbridge`; empfangen hatte er dort vorher schon fehlerfrei. Ob die eingepackte Klasse überhaupt freien Sendeplatz meldet, wird **in
  `begin()`** festgestellt (unmittelbar danach ist der Sendepuffer leer, eine funktionierende
  Implementierung muss dort also mehr als 0 melden); tut sie es nicht, bremst allein die Uhr.
- **`Dummy`** - simuliertes Interface für Tests, ohne echte Hardware (aber trotzdem über das
  Arduino-Framework gebaut, nicht nativ - siehe "Build-Umgebungen").
  **Es liegt nicht in `src/`, sondern in `test/test_tpuart/`**, neben den Tests, die sein einziger
  Nutzer sind. Ein Testdoppel in der Library wäre totes Gewicht in jeder Produktionsfirmware und
  bräuchte eine `#ifdef`-Klammer, die jemand vergessen kann; aus dem Testordner heraus ist es
  zugleich der Beleg, dass `Interface::Abstract` von außen implementierbar ist, genau wie ein
  Aufrufer es täte.
  Empfangene Bytes werden einzeln über `addByte(char data, uint32_t pauseUs)` eingereiht. Die
  Pause ist in **Mikrosekunden**, nicht Millisekunden, und sie ist selbst bedeutungstragende
  Angabe - der Abstand zwischen Bytes wird von der Protokollschicht ausgewertet (Zeitüberschreitung,
  Frame-Ende-Erkennung), Tests müssen ihn also je Byte genau setzen können. `overflow()` ist
  einmalig; Bytes, die die Schicht darüber schreibt (z.B. `U_Ackn.req`), werden mitgeschrieben und
  sind über `writtenBytes()` einsehbar. `setWriteCapacity()` täuscht einen beengten Sendepuffer
  vor, damit Tests den Fall "nicht genug Platz für eine zusammengehörende Bytegruppe" abdecken
  können - und die Kapazität *schrumpft* tatsächlich mit jedem `write()`, freigegeben wird sie
  wieder von `drainWritten()`. Ohne das könnte ein Aufrufer, der mehr Bytes schreibt als er
  reserviert hat, in keinem Test scheitern. `available()` zählt *jedes* Byte, dessen Ankunftszeit
  bereits vorbei ist, nicht nur das nächste - ohne das ließe sich "die Verarbeitung ist in
  Rückstand geraten und das Telegramm liegt schon vollständig im Puffer" in keinem Test
  nachstellen, und genau das treibt die Quittungsunterdrückung.
  **`begin()` spult das Skript bewusst nicht zurück.** Echte Interfaces verwerfen über ein
  `end()`/`begin()`-Paar hinweg alles Gepufferte. `_pos` zurückzusetzen würde die Warteschlange ab
  Byte 0 erneut abspielen - und da `searchBaudRate()` je Baudratenkandidat ein `end()`/`begin()`
  macht, sähe die Erkennungslogik bei jedem Versuch dasselbe `U_Reset.ind` wieder, eines, das auf
  echter Hardware längst weg wäre. Das ist falsches Zutrauen für genau den Teil, den der Anwender
  als den heiklen benannt hat. Zum absichtlichen Neuscharfmachen eines Skripts gibt es
  `clearData()`.

## Aktueller Stand (DataLinkLayer)

`src/TPUart/Types.h` + `DataLinkLayer.{h,cpp}` + `Receiver.{h,cpp}` +
`Transmitter.{h,cpp}` setzen die Zustandsmaschine auf Byteebene um, aufgesetzt auf
`Interface::Abstract`.

**Die Teilung in `Receiver`/`Transmitter`** trägt sich über die Art, wie die beiden verbunden
sind: jeder hält eine `DataLinkLayer &` und ist dessen `friend`, damit Interface,
Statistik und Chip-Zustand an einer Stelle liegen statt in beiden Hälften doppelt. Die einzige
umgekehrte Freundschaft ist `Receiver` innerhalb von `Transmitter` (Quittung und Echovergleich);
der `DataLinkLayer` kommt mit deren öffentlichen Methoden aus.
**Genau je eine Referenz, `_dll`** - die Hälften halten *keine* eigenen
`Interface::Abstract &` / `Statistics &` vor. Das sparte eine Indirektion im Bytepfad, erkaufte
aber eine Zusage über die Reihenfolge der Memberdeklarationen im `DataLinkLayer` (die
zwischengespeicherten Referenzen mussten aus bereits konstruierten Membern initialisiert werden),
und für den Transmitter war diese Zusage ohnehin nicht haltbar, weil die beiden Hälften einander
brauchen. Gegen einen 2-3µs-Tick ist die Indirektion nicht messbar. Die Trennlinie ist
**Richtung, nicht Kontext**: alles Lesende liegt im `Receiver`, alles Schreibende im
`Transmitter`, und beide haben eine Tick-Hälfte und eine Loop-Hälfte.
Der eine Fall, der quer liegt, ist die **Quittung**, und sie ist an der Naht geteilt: die
*Entscheidung* liegt in `Receiver::sendAcknowledge()` (sie fällt bei Byte 6 eines noch
eintreffenden Frames, und nur der Empfänger weiß, wie viel davon noch aussteht), das *Byte* geht
über `Transmitter::sendAcknowledge()` hinaus.
Gewachsen ist das aus einer flachen Klasse, die selbst einmal `Receiver` hieß und umbenannt wurde,
als `tick()` auch das Senden treiben musste. Die Namen kamen zurück, als der echte Transmitter
(Sendequeue, `U_L_DataStart/Cont/End`, Wachhund) die Klasse groß genug machte, um sie zu
zerteilen.

**Jede Klasse wird in einer `.h` deklariert und in einer `.cpp` definiert** - einschließlich der
trivialen Getter. Die eine Ausnahme ist `Interface::ArduinoSerial`, und
sie ist unvermeidlich: ein Template, das mit dem Serial-Typ des Aufrufers instanziiert wird,
braucht seine Rümpfe an der Instanziierungsstelle. `Interface::Abstract` hat eine `.cpp` für genau
zwei Funktionen (Destruktor und die Vorgabe für `overflow()`), damit seine vtable einmal statt in
jeder Übersetzungseinheit entsteht. Die `Statistics`-Inkremente je Byte liegen inzwischen außerhalb
der Header; bei 2-3µs je Tick gegen ein 500µs-Intervall ist das nicht messbar.

- **Zeitvergleiche über die Tick-/Loop-Grenze hinweg brauchen eine Momentaufnahme *und* einen
  vorzeichenbehafteten Vergleich.** `_lastReceivedAt` wird vom Tick geschrieben (einem Interrupt)
  und von `loop()` gelesen. Schreibt man `now = millis(); if ((uint32_t)(now - _lastReceivedAt) >=
  TIMEOUT)`, macht ein Byte, das zwischen den beiden Lesevorgängen eintrifft, den Zeitstempel
  *neuer* als das bereits eingefrorene `now`; die vorzeichenlose Differenz läuft auf ~4,29
  Milliarden über und die Frist schlägt sofort zu. Das erzeugte im IP-Router eine Flut unechter
  `BCU disconnected`-Meldungen, nur unter Last (bei 38400 Baud landet alle ~286µs ein Byte in
  diesem Fenster) und unmöglich vor dem `Timer`, als beide Hälften in einem Kontext liefen.
  Die Regel: **erst den Zeitstempel lesen, dann die Uhr**, und trotzdem als `int32_t` vergleichen -
  der Tick kann gleich danach erneut schreiben, und ein leicht vorauseilender Zeitstempel bedeutet
  "gerade eben", nicht "vor Ewigkeiten". Jeder andere Zeitvergleich der Library bleibt innerhalb
  eines Kontexts (`_awaitSince`, `_emptySince`, `_detectRequestSentAt` im Tick;
  `_lastStateRequestAt`, `_lastBusLoadTime`, der Wiederholungsfilter im Hauptkontext); die beiden
  `_detectNextAttemptAt`-Prüfungen waren bereits vorzeichenbehaftet.
- **Die Library treibt `tick()` selbst an** (`Timer.{h,cpp}`, eingetragen aus `begin()`, ausgetragen
  in `end()`, Vorgabe 500µs). Das gibt es, weil `tick()` **ein Byte je Richtung und
  Aufruf** bewegt: aus einem Host-`loop()` getrieben hängen Durchsatz und die 2,8ms-Quittungsfrist
  beide an der Loopfrequenz der Anwendung. In der echten Firmware gemessen, wo der knx-Wrapper
  `process()` einmal je Durchlauf ruft, reichte das nicht.
  **`TPUart::Timer` ist ein SINGLETON - ein Timer für alle BCUs**, und das ist keine Stilfrage. Die
  Library unterstützt mehrere BCUs an einem Gerät, und ein Timer je Instanz trägt das nicht: der RP2040
  hat genau **vier** Hardware-Alarme (`NUM_ALARMS`), von denen der Default-Pool einen belegt - noch im
  Runtime-Init, also vor `main()`, weshalb unser `claimOwnPool()` ihm nichts wegnehmen kann und nichts
  abstürzt. Frei sind damit drei, und die vierte Instanz fiel **stillschweigend** auf den Default-Pool
  zurück: Priorität 0x80 statt 0x40, serialisiert mit fremden Callbacks - genau der Zustand, dessen
  Beseitigung den eigenen Pool überhaupt gerechtfertigt hat, und nichts meldete es.
  Getrennte Timer brächten auch nichts ein: mehrere Alarme auf 0x40 verdrängen sich **nicht** gegenseitig
  (gleiche Priorität verdrängt nicht), sie serialisieren genauso wie ein gemeinsamer Timer - nur
  unvorhersehbar statt in fester Reihenfolge.
  Der Preis ist, dass die Ticks aller Instanzen hintereinander in **einem** Aufruf laufen. Gemessener
  Höchstwert eines einzelnen Ticks: 660µs, davon 269µs im Quittungs-Callback des Aufrufers; Median 1-2µs.
  Drei Instanzen im schlimmsten Fall wären also rund 2ms gegen ein Quittungsfenster von 2,8ms - praktisch
  unkritisch, aber `getTickDurationMaxUs()` gibt es weiterhin **je `DataLinkLayer`**, damit es sichtbar
  bleibt.
  **`TPUART_TIMER_MAX_CLIENTS` ist 1**, weil eine BCU der Normalfall ist; zwei oder drei kosten ein `-D`.
  Der Grund ist **nicht** die Laufzeit - hier stand einmal, ein größeres Array belaste den heißesten Code,
  und das war überzogen: vier Plätze sind grob 25 Takte von den 66.500, die bei 133MHz in einem 500µs-Tick
  stecken, also 0,04%. Der Grund ist, dass eine Grenze sichtbar sein soll: wer eine zweite BCU anschließt,
  schreibt das einmal ausdrücklich hin.
  Ist kein Platz frei, wird die Instanz **abgewiesen** und die bereits eingetragene läuft weiter.
  **Der Fehlfall ist laut**, und das trägt die Entscheidung: die abgewiesene Instanz wird gar nicht
  getickt, `checkTickRate()` sieht in ihrem `loop()` null Ticks im Fenster und meldet
  `Tick stopped - no tick in ... ms`; die Konsole zeigt zusätzlich `(no slot)`. Ein vergessenes `-D` fällt
  damit binnen zwei Sekunden auf. Dass dieser Zweig existiert, war übrigens Glück und nicht Absicht - er ist
  da, weil `delta == 0` sonst eine Division durch null wäre.
  **Das Verzeichnis ist plattformunabhängig, nur der Timer ist es nicht**: `add()` trägt auch dort ein, wo
  `supported()` falsch ist, und nur der Rückgabewert sagt, ob wirklich getickt wird. Ohne diese Trennung
  wäre die Buchführung im nativen Testlauf gar nicht prüfbar.
  **Der Destruktor des `DataLinkLayer` trägt aus**, und das ist kein Schmuck: der Timer hält einen
  *Zeiger*, und der überlebte die Instanz sonst - der nächste Tick liefe in freigegebenen Speicher, auf dem
  RP2040 aus einem Interrupt. `end()` zu verlangen genügt nicht, weil niemand verpflichtet ist, es vor dem
  Wegwerfen zu rufen. `test_timer_slot_is_released_on_destruction` hält das fest (mutationsgeprüft).
  - **Die Vorgabe ist 500µs, und auf manchen Interfaces sollte ein Gerät, das viel sendet, sie
    senken** (`-D TPUART_TIMER_INTERVAL_US=…`). Zum Empfangen ist 500µs bequem (der Bus liefert
    höchstens alle 1,354ms ein Byte), aber **Senden kann die Leitung aushungern**, und das ist
    gemessen, nicht geraten. Die sendende Hardware ist flach: der RP2040-UART läuft mit
    abgeschaltetem FIFO (die DMA will jedes empfangene Byte sofort), er hält also genau zwei Bytes -
    Halte- plus Schieberegister, 572µs bei 38400 Baud. Werden die nur aus dem Tick nachgefüllt,
    **kommt je Tick höchstens ein Byte hinein**, weil das Halteregister erst frei wird, wenn das
    Schieberegister das vorige übernommen hat. Das pendelt sich auf 3 Byte je 1000µs statt 3,5 ein -
    14% Verlust, und es deckt sich mit der Messung: 436 Host-Bytes einzuspeisen dauerte 143ms gegen
    die 125ms, die die Leitung zulässt (`loop1` mit ~12µs Pollrate maß 123ms). Der Software-TX-Ring
    deckte das *nicht* ab, weil er sich nur zum Tick in die Hardware entleerte.
    **Das `RP2040`-Interface ist ausgenommen, seit es einen TX-Interrupt hat**: die Hardware füllt
    sich selbst nach, sobald ein Platz frei wird, ihr Durchsatz hängt also gar nicht mehr am
    Tick-Intervall (siehe `RP2040.h`). **Die Regel gilt nur für `ArduinoSerial`**, und was sie
    auslöst, ist nicht "nur der Tick füllt nach", sondern *wie viel* je Aufruf hineinkommt: mit der
    oben beschriebenen Zwei-Byte-Hardware genau eines, was der Transmitter auch anbietet - daher
    **muss das Intervall unter einer Zeichenzeit der Host-Strecke bleiben** (286µs bei 38400, 573µs
    bei 19200), ein Router dort will also 250µs, zum Preis von rund 1% CPU.
    **`ESP32` braucht das nicht.** Sein Treiberring nimmt beide Bytes eines Oktetts im selben Tick
    (`Transmitter::process()` fragt nach 2, mit Offsetbyte nach 3, und schreibt sie zusammen), ein
    Tick liefert also ein ganzes Oktett - 1146µs Leitungszeit bei 19200, 572µs bei 38400, beides
    länger als der 500µs-Tick. Die Leitung bleibt der Engpass, und das ist der Sinn.
  - **RP2040**: Hardware-Timer, `tick()` läuft **im Interrupt**. Nur
    zulässig, weil beide brauchbaren Interfaces ISR-sicher schreiben (`RP2040` über seinen
    Software-TX-Ring, `ArduinoSerial` über `uart_putc_raw()`). Flash-Schreibvorgänge sind keine
    Gefahr: `rp2040_arduino_platform.cpp` klammert sie in `noInterrupts()`/`idleOtherCore()`, der
    Tick pausiert also, statt aus abgeschaltetem XIP zu laufen.
    **Er hängt in einem EIGENEN Alarmpool, nicht im Vorgabepool**, und das aus zwei Gründen, die beide
    gemessen sind: die Callbacks *eines* Pools laufen aus einem gemeinsamen IRQ-Handler und damit
    serialisiert, und nur ein eigener Pool hat einen eigenen Hardware-Alarm, dessen Priorität sich
    anheben lässt. Sie steht auf `TPUART_RP2040_TIMER_IRQ_PRIORITY` (0x40) statt auf den 0x80, die das SDK
    beim Hochlauf jedem IRQ gibt.
    **Der Grund dafür ist eine Feinheit des Cortex-M, die leicht falsch erinnert wird**: ein Interrupt
    verdrängt dort sehr wohl einen anderen (dafür steht das N in NVIC), aber **nur bei echt höherer
    Priorität - gleiche Priorität verdrängt nicht**. Solange der Tick auf 0x80 mitschwimmt, wartet er auf
    jeden anderen laufenden Handler, und das sind alle. 0x40 genügt und 0x00 wäre falsch: im gesamten
    SDK-Quellbaum gibt es genau *einen* expliziten `irq_set_priority()`-Aufruf, und der setzt die
    Hintergrundarbeit des Netzwerkstacks nach **unten** (`async_context_threadsafe_background` auf
    `PICO_LOWEST_IRQ_PRIORITY`). Über 0x80 sitzt also niemand; 0x00 nähme nur USB und DMA die Luft.
    Beachte, dass daraus zugleich folgt, dass **der Netzwerkstack den Tick gar nicht blockieren kann** -
    eine Vermutung, die hier ausführlich verfolgt und am Quelltext widerlegt wurde.
    **Keine der bequemen SDK-Funktionen ist für eine Library benutzbar**: `alarm_pool_create()` macht ein
    *hard assert*, wenn der Alarm schon vergeben ist, und
    `alarm_pool_create_with_unused_hardware_alarm()` ebenso, wenn gar keiner frei ist - ein Absturz statt
    eines meldbaren Fehlschlags. `hardware_alarm_claim_unused(false)` ist der einzige Weg, der einen
    Misserfolg zurückgibt, deshalb wird damit geprüft, sofort wieder freigegeben und erst dann erzeugt
    (`claimOwnPool()`). Schlägt irgendetwas davon fehl, bleibt `_pool` auf `nullptr` und es läuft wie
    vorher über den Vorgabepool - der Umbau kann also nicht schlechter sein als sein Vorgänger.
    **Was Priorität NICHT löst, ist `PRIMASK`**: `noInterrupts()`, `save_and_disable_interrupts()` und
    `critical_section_enter_blocking()` sperren unabhängig davon. Dagegen hülfe nur ein Tick auf dem
    zweiten Kern, denn PRIMASK ist pro Kern - und selbst der nicht gegen `idleOtherCore()`, das der
    Flash-Pfad zusätzlich benutzt.
  - **ESP32**: `esp_timer`, `tick()` läuft im **Timer-Task**. Ein echter Interrupt scheidet aus -
    `uart_write_bytes()` nimmt einen Treiber-Mutex. Bewusst auch kein festgenagelter Task mit
    `delayMicroseconds()` (so machte es das frühere Diagnosewerkzeug): das kostet einen ganzen Kern,
    und auf einem echten Gerät trägt Kern 0 WLAN und Bluetooth.
  - **Sonst überall**: `Timer::supported()` ist falsch und der Timer startet nicht. Dann tickt **nichts**,
    bis der Aufrufer `Timer::trigger()` aus einem eigenen Kontext ruft - einen Rückfall gibt es nicht, siehe
    den nächsten Punkt.
  - **DER HAUPTLOOP TICKT NIE.** `process()` ruft ausschließlich `loop()`. Das ist eine ausdrückliche
    Festlegung des Anwenders und keine Auslegungsfrage - und sie ist die Lehre aus dem Fall, der diese
    ganze Messung ausgelöst hat: aus dem Hauptloop getrieben lief der Router mit 457 Ticks/s statt 2000 und
    verlor 12% der Quittungen. Ein Rückfall auf den Hauptloop macht genau diesen Zustand zum **stillen
    Normalfall** - er sieht aus wie "es läuft", und niemand erfährt, dass der Antrieb fehlt. Fehlt der
    Timer, tickt lieber nichts: das verlangt eine Entscheidung des Entwicklers statt einer Notlösung.
  - **KEINE STELLSCHRAUBE JE INSTANZ.** Der Timer wird immer benutzt, `begin()` trägt die Instanz ohne
    Nachfrage ein, `end()` und der Destruktor tragen sie aus. Es gab dafür zwischenzeitlich ein
    `setUseTimer(bool)`, und es ist bewusst wieder weg: ein Schalter mit der Bedeutung "ich mache es
    selbst" ist genau die Zuständigkeitsübergabe, die niemand nachlesen kann - und was `process()` dann
    tun soll, war für jeden Leser eine andere Antwort. Ebenso weg ist `setTickInterval()` an der Instanz:
    das Intervall ist global, und eine Methode am Objekt, die alle Instanzen umstellt, wäre eine Falle.
    **Was bleibt, ist genau zweierlei**: `Timer::setInterval(0)` hält den Takt an, und `Timer::trigger()`
    treibt ihn von Hand. Wer beides kombiniert - Timer läuft UND jemand ruft `trigger()` -, hat zwei
    Tick-Kontexte auf einer Schnittstelle. Im IP-Router ausprobiert (Kern 0 gegen Kern 1), und der Chip
    meldete es sofort: `Unknown control byte` für die gestohlenen Empfangsbytes, `PE` (Protokollfehler) in
    `U_State.ind` für die halbierten Sequenzen.
    `usesTimer()` meldet, ob **diese** Instanz vom Timer getickt wird, **dynamisch** und nicht bei
    `begin()` festgeschrieben (der Timer kann später anlaufen, wenn eine andere Instanz ihren Platz
    freigibt). Es ist der einzige Diagnosewert zum Antrieb, und er ist wichtiger als er aussieht: liefert
    er falsch, ohne dass jemand `trigger()` ruft, steht die Schicht still - und das sieht von außen aus wie
    "die BCU antwortet nicht".
  - **`Timer::trigger()` treibt den Takt von Hand** - ein `tick()` je eingetragener Instanz, genau das,
    was der plattformeigene Callback tut. Das ist der **einzige** Weg neben dem Timer, und damit die
    Antwort auf "Plattform ohne Timer-Unterstützung": kein Rückfall, sondern ein ausdrücklicher Aufruf aus
    einem Kontext, den der Entwickler selbst stellt. Es gilt dieselbe Anforderung wie an den
    Callback: der Kontext muss dürfen, was `tick()` tut - auf dem ESP32 also kein ISR, weil
    `uart_write_bytes()` einen Mutex nimmt.
  - **Der Timer-Antrieb gibt eine Zusage, die zwei Kerne nicht geben.** Ein Interrupt läuft immer zu
    Ende und wird nie vom Hauptloop verdrängt, `tick()` und `loop()` konnten sich also nie
    verschränken. Mit `tick()` auf Kern 1 ist das weg. Die SPSC-Ringe überstehen es (der Kopf wird
    erst veröffentlicht, wenn der Eintrag vollständig geschrieben ist), und die Sendewarteschlange
    berührt es gar nicht mehr - sie gehört seit dem Umbau dem Hauptkontext allein, der Tick sieht nur
    die Vorlage. Jedes ab hier neu hinzugefügte geteilte Feld muss trotzdem gegen echte Parallelität
    geprüft werden, nicht nur gegen Verdrängung.
  - **Folge für Callbacks**: auf dem RP2040 läuft der Quittungs-Callback
    (`registerCheckAcknowledge`) jetzt im Interrupt-Kontext. Er musste ohnehin kurz und
    allokationsfrei sein; das ist nun keine Empfehlung mehr.
- **Verbindungsaufbau** (`begin(BcuType)` / `isConnected()` / `connectedBaudRate()`): aus
  nicht blockierend. Solange `isConnected()` nicht
  wahr ist, laufen die beiden Hälften überhaupt nicht - `RxState`/`TxState` bleiben unangetastet,
  denn Bytes, die bei unbestätigter Baudrate gelesen werden, sind als Protokoll gar nicht deutbar.
  **Wer das Interface anfassen darf, hängt an `_everConnected`**, und das ist die Invariante, die zu
  erhalten ist: solange es falsch ist, gehört das Interface dem **Hauptkontext**, der
  `searchBaudRate()` aus `loop()` fährt; ab der ersten erfolgreichen Verbindung gehört es für immer
  dem **Tick**, einschließlich `reconnect()` nach einem Verlust. Die Übergabe passiert genau einmal
  und wird vom Hauptkontext veröffentlicht (`_everConnected` vor `_connected`); danach fasst der
  Hauptkontext das Interface nie wieder an, es braucht also weder Sperre noch Rückgabe.
  Deshalb sind die beiden Fälle getrennte Funktionen: **die Suche konfiguriert das Interface je
  Kandidat neu (`end()`/`begin()`), und das darf niemals in einem Interrupt passieren** -
  `uart_driver_install()` allokiert auf dem ESP32, und `ArduinoSerial` reicht an ein beliebiges
  fremdes `begin()` weiter. **Der Reconnect braucht nichts davon**: die Baudrate steht fest und kann
  sich nicht ändern, es ist also derselbe `U_Reset.req` und dieselbe Antwort auf einem bereits
  offenen Interface - genau das, was der Tick ohnehin tut. Er verwirft dabei auch nicht mehr den
  Empfangspuffer und initialisiert keine korrekt konfigurierte Hardware für nichts neu.
  Folge für Aufrufer: **`loop()` muss laufen, damit überhaupt eine Verbindung zustande kommt**;
  `tick()` allein kommt nie dorthin. Dafür lässt sich der Tick-Antrieb jederzeit umschalten,
  verbunden oder nicht - die Library erzwingt keine Regel mehr, um die die Anwendung herumbauen
  müsste.
  Je `tick()` wird höchstens ein Byte betrachtet, dieselbe Disziplin wie überall sonst in dieser
  Klasse. Je Baudratenkandidat blockierend zu warten scheidet damit aus - die Erkennung ist ein
  Zustand über mehrere `tick()`-Aufrufe hinweg, getragen von `_detectAwaitingResponse` /
  `_detectRequestSentAt`.
  **Die Kandidaten hängen am `BcuType`**: `Tpuart2` probiert immer nur 19200; `Ncn5120` (deckt
  NCN5120/5121/5130 ab) probiert 19200, dann 38400. Je Kandidat: `interface.end()` +
  `interface.begin(baud)` (ein sauberer Neustart ist für einen Baudratenwechsel nötig - besonders
  das RP2040-Interface würde sonst versuchen, einen zweiten DMA-Kanal zu beanspruchen) plus ein
  einzelnes `U_Reset.req`.
  **Die Zuordnung ist hier der Kern der Korrektheit** (ausdrückliche Anforderung des Anwenders):
  ein `U_Reset.ind`, das zu irgendeinem unbeteiligten Zeitpunkt eintrifft, darf nie als Bestätigung
  missverstanden werden. Es zählen deshalb nur Bytes, die *nach* dem Absenden der Anforderung
  eintreffen, solange `_detectAwaitingResponse` gesetzt ist - und das erste Byte in diesem Fenster
  entscheidet sofort über den Kandidaten: `0x00` wird übersprungen (der Chip schickt am Anfang
  Nullbytes), alles, was nicht `U_RESET_IND` (`0x03`) ist, lässt den
  Kandidaten sofort scheitern (kein Aussitzen der restlichen Frist auf einen späten Treffer), und
  `U_RESET_IND` gelingt - womit Baudrate und der initiale Reset in einem Schritt bestätigt sind.
  Beides ist ein Ereignis, nicht zwei.
  Ein gescheiterter oder abgelaufener Kandidat rückt sofort zum nächsten weiter; ist die ganze Liste
  durch, pausieren die Versuche `TPUART_DETECT_RETRY_INTERVAL_MS` (1000),
  bevor von vorn begonnen wird - **die BCU antwortet erst, wenn Busspannung anliegt**, und das kann
  dem eigenen Hochlauf deutlich hinterherhinken; dass die Versuche eine Weile laufen, ist also der
  Normalfall und kein Notnagel für seltenes Versagen.
  **Eine Baudratenumschaltung nach dem Verbinden gibt es nicht und ist nicht geplant** - laut
  Anwender müsste dafür die physische Bus-Transceiver-Hardware im Betrieb umkonfiguriert werden,
  wofür dieses Projekt keinen Weg hat.
- **`tick()` und `loop()` sind zwei bewusst getrennte Hälften, mit dem RX-Ringpuffer als
  Trennlinie.** `tick()` ist die zeitkritische Seite (ein Byte je Aufruf, blockiert nie, dafür
  bestimmt, aus einem Timer-ISR zu laufen): sie zerlegt, sendet die Quittung und schiebt alles
  Fertige in `_rxQueue`. `loop()` ist die entspannte Seite, aus dem Hauptloop gerufen: sie leert die
  Warteschlange, reicht **Telegramme** an einen registrierten Callback und verarbeitet
  **Steuerbytes** intern. Ohne diese Teilung liefe Anwendungscode im Interrupt-Kontext und könnte
  das schmale Quittungsfenster blockieren.
  Für empfangene Daten gibt es keine Poll-API - kein `hasFrame()`/`popFrame()`, kein `rxBuffer()`.
  Telegramme kommen ausschließlich über `registerFrameCallback(std::function<void(Frame &frame)>)`,
  gerufen aus `loop()`. Das `Frame` ist eine Sicht auf einen internen Puffer und nur für die Dauer
  des Aufrufs gültig; es ist nicht `const`, damit der Verbraucher eigene Flags setzen kann (z.B.
  `setFiltered()`).
  **Aus der `tick()`-Hälfte darf nichts Beobachtbares heraussickern.** Ein ISR darf nicht nach
  `Serial` schreiben und nicht allokieren, es gibt also keinen Diagnosehaken je Byte. Ein früheres
  Paar `lastByte()`/`ackSent()` existierte genau für eine bytweise Konsolenspur und wurde aus
  diesem Grund entfernt - diese Form nicht wieder einführen.
  Bekannter Preis, vom Anwender akzeptiert: der Zeitpunkt, zu dem das `U_Ackn.req` abging (früher
  das cyanfarbene `*` hinter Byte 6), ist nicht mehr beobachtbar, weil er nicht Teil des Telegramms
  und damit nicht Teil des Warteschlangeneintrags ist. Ein Flag dafür kann mit der nächsten Stufe
  der Frame-Flags kommen.
- **Politik bei voller Warteschlange: den NEUEN Eintrag verwerfen, das bereits Angenommene
  behalten.** `pushRxEntry()` verweigert, wenn der Eintrag nicht passt, und setzt ein einmaliges
  Flag, lesbar über `queueOverflow()`; die Bytes werden stillschweigend verworfen. Ausdrückliche
  Entscheidung des Anwenders - ändere das nicht "hilfsbereit" in Überschreiben des Ältesten.
- **Aufbau des Ringpuffers**: `[len_lo][len_hi][flags][data...]`. Die Länge braucht zwei Bytes, weil
  263 nicht in eines passt. Die Flags stehen **vor** den Daten, damit ein Leser entscheiden kann, ob
  ihn der Eintrag überhaupt interessiert, bevor er ihn anfasst. Ein Erzeuger, ein Verbraucher: `_rxQueueHead` schreibt nur `tick()`, `_rxQueueTail` nur
  `loop()`, und der Kopf rückt erst weiter, *nachdem* der Eintrag vollständig geschrieben ist, ein
  halb geschriebener Eintrag wird also nie sichtbar. Kopf und Ende laufen monoton mit
  Modulo-Indizierung (derselbe Kniff wie beim TX-Ring des RP2040), damit voll und leer nicht
  verwechselbar sind.
  Telegramm oder Steuerbyte steht **nicht** in einem Flag - es wird aus `data[0]` neu abgeleitet,
  mit derselben Prüfung, die der Parser benutzt hat, also genau mit dem Byte, das die Entscheidung
  ursprünglich getroffen hat.
  `loop()` **prüft die rekonstruierte Länge gegen `TPUART_BUFFER_SIZE`**, bevor es kopiert. In
  geordnetem Betrieb unerreichbar (der Erzeuger schreibt nie mehr als 263), aber das Längenfeld ist
  16 Bit breit, während `_deliverBuffer` ein 263-Byte-*Member* ist - ein Wert außerhalb des Bereichs
  überschriebe die Zustände, den Sendepuffer und die `std::function`-Callbacks. Ein einziger
  beschädigter Eintrag darf nicht das ganze Objekt zerlegen können, es wird also der Ring verworfen
  und weitergemacht.
- **`TP_FRAME_FLAG_*`** - das Flag-Byte ist die API, die der Rest des KNX-Stacks spricht, **die
  Bitbelegung liegt damit fest und darf nicht umsortiert werden**: ein Verbraucher, der das Byte
  numerisch liest statt über die Zugriffsmethoden von `Frame`, bekäme sonst stillschweigend etwas
  anderes. Bit 4 war lange als `ECHO` reserviert und unbenutzt, dort sitzt jetzt `INVALID`.
  **Das Byte ist damit voll** -
  ein weiteres Flag bräuchte ein zweites Byte im Warteschlangeneintrag. Beachte, dass
  `Repeated`/`Extended`/`GroupAddress` keines brauchen: `Frame` leitet sie direkt aus den Daten ab.
  | Bit | Flag | heute gesetzt? |
  |---|---|---|
  | 7 | `TX` | ja - das Echo eines von uns gesendeten Telegramms |
  | 6 | `DATA_CON` | ja - eine Bestätigung für unser eigenes Telegramm kam an |
  | 5 | `FILTERED` | ja - der Wiederholungsfilter hat es als Dublette markiert |
  | 4 | `INVALID` | ja - CRC-Fehler, abgeschnitten, kaputte Länge |
  | 3 | `ADDRESSED` | ja - wir sind für dieses Telegramm zuständig (der Quittungs-Callback sagte ja) |
  | 2 | `ACK_BUSY` | ja - nur im Busmonitor |
  | 1 | `ACK_NACK` | ja - nur im Busmonitor |
  | 0 | `ACK` | ja - nur im Busmonitor |
- **`ACK` heißt "dieses Telegramm wurde quittiert"** - von wem auch immer. Bekannt sein kann das in
  drei Lagen: wir haben selbst quittiert, wir haben die Quittung im Busmonitor auf dem Bus gesehen,
  oder sie kam als Antwort auf ein Telegramm, das *wir* gesendet haben.
  **`ADDRESSED` heißt "wir sind für dieses Telegramm zuständig"** - und das ist ausdrücklich NICHT
  dasselbe wie "wir haben quittiert". Es wird gesetzt, sobald der Quittungs-Callback ja sagt, und
  *bevor* die Rückstandsprüfung greift: ein Telegramm, dessen Quittung unterdrückt wurde, ist trotzdem
  an uns gerichtet, und der Aufrufer soll es als solches erkennen. Umgekehrt trägt unser eigenes Echo
  auch mit Bestätigung **kein** `ADDRESSED` - für ein Telegramm, das wir selbst gesendet haben, sind wir
  der Absender und nicht der zuständige Empfänger. Genau daran fallen die beiden Flags auseinander, und
  `test_own_echo_with_confirmation_is_acked_but_not_addressed` nagelt es fest.
  `setAcknowledge(AcknowledgeType)` setzt entsprechend `ADDRESSED | ACK`. **Verenge `ACK` nicht auf
  "bei uns ist eine Quittung angekommen"**. Diese Lesart wurde ausprobiert und zurückgenommen: sie
  wirkt einleuchtend,
  weil der Chip die sofortige Quittung selbst erzeugt und nie zurückspiegelt, aber sie bricht die
  eine einheitliche Bedeutung, die das Flag über alle drei Fälle hat, und lässt keinen Weg, ein
  eigenes NACK oder BUSY festzuhalten.
- **Zwei Bytepuffer neben den Warteschlangen**: `_rxBuffer` (die gerade eintreffende Sequenz - ein
  Telegramm *oder* eine Steuerbytefolge, nie beides zugleich, ein Puffer deckt also beides ab) und
  `_txBuffer` (das zu sendende Telegramm samt Prüfsumme). Beide `TPUART_BUFFER_SIZE` (263) Byte.
  Steuercodes laufen ausdrücklich *nicht* über `_txBuffer` (siehe die Steuercode-Warteschlange
  unten). Dazu `_deliverBuffer` (263), eine zusammenhängende Kopie für den Callback, weil ein
  Ringeintrag umbrechen kann.
- **Steuercodes haben ihre eigene Warteschlange** (`_ctrlQueue`, `TPUART_CTRL_QUEUE_EXP`, 32 Byte),
  Eintragsformat `[len][bytes...]`. Das ist der zweite SPSC-Ring dieser Klasse, Spiegelbild des
  RX-Rings in die Gegenrichtung: Erzeuger ist der Hauptkontext, Verbraucher ist `tick()`, der Kopf
  wird zuletzt veröffentlicht.
  **Warum eine Warteschlange und nicht Besitz über `TxState`**: Steuercodes müssen absetzbar sein,
  während ein Telegramm gesendet wird. Sie sind keine Telegrammübertragung - genau dasselbe Argument
  galt bereits für die Quittung. Der frühere Entwurf ließ `queueControl()` *verweigern*, sobald
  `TxState != Idle`, was zudem der Bedeutung des Rückgabewerts widerspricht: das `bool` jeder
  Steuerbytemethode heißt "Vorbedingung erfüllt" (nicht initialisiert / falscher Chiptyp), nie
  "gerade beschäftigt".
  Eine Gruppe geht **ganz oder gar nicht** hinaus: `processCtrlQueue()` prüft zuerst, ob das
  Interface Platz für die vollständige Gruppe hat. Eine zu zerteilen ließe die BCU den Rest als
  eigenen Befehl lesen. `TPUART_CTRL_MAX_GROUP` ist 4, die längste Sequenz aus Tabelle 12 des
  Datenblatts (`U_SetAddress.req`, `U_SetRepetition.req`, `U_PollingState.req`) - beachte, dass das
  `TPUART_TX_ATOMIC_BYTES` (3) übersteigt, was nur den Telegrammpfad abdeckt.
  Steuercodes haben in `processTx()` **Vorfahrt** vor Telegrammbytes, und je Tick geht höchstens
  eine Gruppe hinaus. Ein Überlauf wird einmalig über `controlOverflow()` gemeldet; der Code wird
  verworfen, nichts bereits Eingereihtes wird gestört.
  **Ein Einplatz-Zwischenspeicher für die Quittung wäre der falsche Weg** und ist keine Vereinfachung,
  die noch aussteht: einer, der nur aus dem Telegramm-Sendepfad geleert wird, lässt eine Quittung
  beliebig lange liegen und setzt sie dann weit außerhalb ihres ~1,7ms-Fensters ab. Eine Quittung ist
  entweder rechtzeitig oder sie gehört unterdrückt.
- **`RxState` beschreibt nur, was gerade *eintrifft***: `Idle` / `Frame` / `FrameAck` / `Control` /
  `Resync`. Es gibt bewusst **keine "fertig"-Zustände** - sobald etwas fertig ist, geht es in den
  Ringpuffer und der Zustand rückt sofort weiter. Frühere Entwürfe hatten
  `FrameComplete`/`ControlComplete`/`Invalid` als Übergangszustände, die innerhalb eines `tick()`
  gelesen werden mussten; der Ringpuffer macht das alles überflüssig, und `releaseBuffer()`,
  `isCompleteState()` und das aufgeschobene `_resyncAfterRelease`-Flag sind damit entfallen.
  Bewusst *nicht* als Zustände modelliert: "Ziel bekannt" und "Größe bekannt"
  ("Ziel bekannt", "Größe bekannt") - Positionen innerhalb eines Telegramms
  müssen nicht unterschieden werden.
- **`Invalid` ist ein Auslöser, kein Zustand** (ausdrückliche Formulierung des Anwenders und der
  Grund, dass das alte `RxState::Invalid` weg ist). Ein kaputtes Telegramm wird immer gleich
  gemeldet: als normaler Warteschlangeneintrag mit `TP_FRAME_FLAG_INVALID`. Unterschiedlich ist nur,
  was *danach* passiert:
  | Ursache | gemeldet | danach |
  |---|---|---|
  | CRC stimmt nicht | kaputtes Telegramm | `Resync` - dem Frame-Ende ist nicht zu trauen, das Längenoktett kann selbst beschädigt sein |
  | Pause mitten im Telegramm (Abschnitt) | kaputtes Telegramm, so weit es kam | `Idle` - **die Pause ist bereits der Synchronisationspunkt**, den ein Resync suchen ginge |
  | Längenoktett beschädigt (LG=255) | kaputtes Telegramm, so viel wie empfangen | `Resync` - Frame-Ende unbekannt, weiterzusammeln wäre Raten |
  `Resync` bedeutet damit genau eines: alles ab hier ist bedeutungslos bis zu einer bestätigten
  Pause. Bytes werden währenddessen nicht einmal gepuffert.
- **`TxState`**: `Idle` / `Transmit` / `Await` - es geht um das Senden eines **Telegramms** und um
  nichts sonst. Er dient zugleich als Besitzmarke für `_txBuffer`: der Hauptkontext füllt ihn nur
  bei `Idle` und veröffentlicht `Transmit` zuletzt.
  **Weder die Quittung noch ein Steuercode sind ein `TxState`** - beides sind Bytegruppen, die
  zwischendurch hinausgehen, ohne eine Telegrammübertragung anzufassen. Für Steuercodes gilt das
  erst, seit sie ihre eigene Warteschlange haben; davor belegte ein anstehender Steuercode diesen
  Zustand und blockierte alles andere.
  RX und TX laufen nie gleichzeitig, die beiden Zustandsmaschinen können also nicht kollidieren.
- **Steuerbefehle an die BCU** - `startMonitoring()`, `reset()`, `requestState()`,
  `stopMode(bool)`, `busyMode(bool)`, `powerControl(bool)`. Namen und Bedeutung sind API und
  liegen fest, weil Aufrufercode sie so anspricht.
  Alle liefern `bool` im Sinne von **"Vorbedingung erfüllt"**, nicht "das Byte liegt auf dem Bus" -
  `false` heißt nur: keine Verbindung, falscher Chiptyp oder Steuerwarteschlange voll. Ein Aufrufer,
  der den Wert ignoriert, bleibt damit gültig.
  `stopMode()` und `powerControl()` gibt es nur beim NCN512x und liefern auf einem TPUART2 `false`.
  `busyMode()` funktioniert auf beiden, aber mit verschiedenen Opcodes.
  `requestState()` setzt beim NCN zusätzlich `U_SystemState.req` ab - der *einzige* Weg, auf dem je
  ein `U_SystemStat.ind` eintreffen kann, die eine mehrbytige Steuerantwort, die der Parser
  behandelt. `powerControl()` schreibt ACR0 als 2-Byte-Gruppe und ruft danach `requestState()`,
  damit die Wirkung sichtbar wird.
- **Busmonitor** (`startMonitoring()` / `reset()` / `isBusMonitor()`): `startMonitoring()` reiht
  `U_Busmon.req` (`0x05`) ein; ist der Modus bereits aktiv, wird nichts gesendet und `true`
  geliefert. Es gibt bewusst **kein `setBusMonitor(bool)` und kein
  "aus"** - laut Datenblatt (S. 36, Bild 35) lässt sich der Zustand nur über den Reset-Service
  verlassen, die API bildet das ab: `reset()` sendet `U_Reset.req` (`0x01`), was den Busmonitor als
  Nebenwirkung beendet und den Vorgabe-CRC-Modus wiederherstellt.
  Beachte, dass dieses `reset()` das *nach* dem Verbindungsaufbau ist - es setzt `isConnected()`
  voraus (über `queueControl()`). Der *initiale* Reset, der die Verbindung herstellt, ist ein ganz
  anderer Codepfad: er passiert innerhalb des Verbindungsaufbaus während `begin()`, bevor
  `_connected` überhaupt wahr ist, und ist zugleich bei jedem Start der Reset des CRC-Modus (siehe
  "Verbindungsaufbau" oben).
  Zwei Dinge machen die Umschaltung sicher, und beide sind leicht zu brechen:
  1. `_busMonitor` wird **erst aktualisiert, nachdem** `processCtrlQueue()` das Byte tatsächlich auf
     die Leitung gelegt hat, und es wird **aus dem gerade gesendeten Code abgeleitet**
     (`U_BUSMON_REQ` → wahr, `U_RESET_REQ` → falsch). Ein eigenes "Wunsch"-Feld gibt es bewusst
     nicht: so ein Feld ist Nutzlast und müsste *vor* dem Veröffentlichen des Warteschlangeneintrags
     geschrieben werden. Diese Reihenfolge war einmal falsch - das Veröffentlichen kam zuerst, ein
     Tick im Zwischenraum las also den alten Wert, und Chip-Zustand und `_busMonitor` liefen dauerhaft
     auseinander. Das Ableiten lässt die Reihenfolgefrage verschwinden.
  2. Das Senden des Codes löst `forceResync()` aus, aber nur für `U_BUSMON_REQ` und `U_RESET_REQ` -
     nur die ändern, was der Bytestrom bedeutet (Bus-Quittungen erscheinen bzw. verschwinden).
     Andere Steuercodes lassen das Format unberührt, dort wäre ein Resync nur ein unnötig verworfenes
     Telegramm. Und `forceResync()` handelt selbst nur, wenn wirklich eine Sequenz im Gang war
     (`RxState::Frame`, `Control` oder `FrameAck`). War `RxState` gleich `Idle`, tut es nichts: es
     ist nichts unterwegs, das zu schützen wäre, und die nächsten Bytes sind schlicht die Antwort auf
     das, was wir gerade gesendet haben (z.B. `U_Reset.ind`). Eine frühere Fassung ging bedingungslos
     in den `Resync` und fraß damit nach jedem Moduswechsel stillschweigend diese Antwort plus das
     erste Telegramm danach - ein echter Fehler, keine Entwurfsentscheidung. `Resync` wird ebenfalls
     in Ruhe gelassen (verwirft ohnehin schon).
  **Was im Busmonitor überhaupt noch wirkt, steht in Tabelle 11 des NCN5130-Datenblatts (S. 32), und
  danach richten sich die Wächter im Code.** Die Tabelle führt je Dienst `E` (wird ausgeführt), `I`
  (wird ignoriert, *ohne* Rückmeldung an den Host) oder `R` (abgelehnt mit Protokollfehler). Im
  Busmonitor sind `I`: `U_State.req`, `U_SystemStat.req`, `U_SetBusy.req`/`U_QuitBusy.req`,
  `U_SetAddress.req`, `U_SetRepetition.req`, `U_Configure.req`, `U_Ackn.req`, `U_PollingState.req` und
  `U_L_DataStart/Cont/End.req`. `E` bleiben nur `U_Reset.req`, `U_StopMode.req`/`U_ExitStopMode.req`
  und `U_IntRegWr.req`/`U_IntRegRd.req`. Dazu die Fußnote, die erklärt warum: *"Bus Monitor state is
  not a separate state. It is applied on top of Normal, Stop, Sync or Power-Up State."*
  Daraus folgen fünf Sperren, und alle fünf sind aus demselben Grund da - was der Chip ohnehin
  ignoriert, gehört nicht in eine Spur, die passiv sein soll:
  - `sendAcknowledge()` steigt sofort aus.
  - `requestState()` liefert `false`, ohne etwas einzureihen. Das deckt zugleich `stopMode()` und
    `powerControl()` ab, die es zum Sichtbarmachen ihrer Wirkung rufen - die Dienste selbst wirken
    (beide `E`), nur die Rückmeldung bleibt aus, `SystemState` steht also für die Dauer still.
  - `busyMode()` liefert `false`. Es abzusetzen behauptete sonst über `_busyModeSince` einen Zustand,
    den der Chip gar nicht angenommen hat.
  - `applyConfiguration()` liefert `false`, ohne etwas abzusetzen. Die Konfigurationsepoche bleibt
    damit offen und wird nach dem Verlassen nachgeholt - was sich trifft, denn verlassen wird der
    Modus nur per Reset, und der löscht die Konfiguration im Chip ohnehin.
  - `Transmitter::process()` kehrt um, **vor** dem `Await`-Zweig und vor der Zustandsprüfung.
  **Der gesamte Sendeweg wird beim Umschalten geräumt, nicht eingefroren** - `Transmitter::abort()`,
  gerufen aus `controlByteSent(U_BUSMON_REQ)`. Das umfasst beides: die laufende Übertragung und die
  Warteschlange. Ein halb abgesetztes Telegramm später fortzusetzen ergibt keinen Sinn (auf dem Bus
  hat nie jemand einen Anfang gesehen, und der Chip hat es mit dem Moduswechsel ohnehin verworfen) -
  darin unterscheidet sich `abort()` von `restart()`, das nach einem Reset von vorn beginnt. Und eine
  eingefrorene Warteschlange ginge beim Verlassen auf einen Schlag hinaus, mit Telegrammen, die dann
  längst überholt sind.
  **`abort()` läuft im Tick und verwirft dort NUR DIE VORLAGE**, nicht die Warteschlange. Das ist die
  Aufteilung, die den früheren Wettlauf auflöst: `_txState` behält einen Schreiber (aus dem Hauptkontext
  kollidierte es mit `confirmed()`/`echoReceived()` aus dem Empfangspfad), und die **Warteschlange gehört
  dem Hauptkontext allein** - geräumt wird sie in `stageNextTelegram()`, das im Busmonitor `clear()` ruft.
  Hier stand einmal, `abort()` ziehe `_queueTail` bis an `_queueHead` vor und der Heap werde aus `loop()`
  freigegeben. Beides beschreibt den ersetzten Entwurf: es gibt weder `_queueTail`/`_queueHead` noch Heap
  im Datenpfad, die Sendequeue ist ein linearer Bytepuffer (`TransmitQueue`). **Den Tick dort schreiben zu
  lassen wäre ein echter Fehler** - er fasste `_head` und `_end[]` an, die der Hauptkontext in `push()`
  und `compact()` ebenfalls schreibt, auf dem ESP32 echt parallel. Der Puffer liefe auseinander, und
  `front()` gäbe eine falsche Länge zurück.
  **Busmon an heißt Busy aus, und Auto-Quittung aus.** Der Chip nimmt im Modus keine Telegramme mehr
  an und quittiert nichts; ein Busy-Modus, der das Quittieren nur ersetzt, kann es dort nicht geben,
  und heraus kommt man ohnehin nur per Reset. Der Zustand ist also nicht unbekannt, sondern bekannt
  weg - `controlByteSent(U_BUSMON_REQ)` setzt `_autoAcknowledge` auf falsch und meldet den Busy-Modus
  über `reportBusyModeCancelled()` ab. **Gemeldet statt selbst gelöscht**, weil `_busyModeSince` dem
  Hauptkontext gehört (`busyMode()`, `checkBusyMode()`) und ein zweiter Schreiber aus dem Tick der
  teurere Fehler wäre; den Weg gibt es für genau diesen Zweck schon. Ohne ihn liefe `checkBusyMode()`
  in eine Sackgasse: nach Ablauf der Frist ruft es `busyMode(false)`, und das lehnt im Busmonitor jetzt
  ab - `_busyModeSince` bliebe stehen und der Versuch wiederholte sich endlos.
  **Beim Verlassen wird `_lastReceivedAt` neu aufgezogen** (in `controlByteSent(U_RESET_REQ)`, solange
  `_busMonitor` noch steht). Weil die Überwachung während des Modus ruht, ist der Zeitstempel auf
  einem ruhigen Bus beliebig alt - und Busmonitor auf ruhigem Bus ist der Normalfall. Zwischen dem
  `U_Reset.req` und der `U_Reset.ind`, die ihn beantwortet, liegt zwar nur etwa eine Millisekunde,
  aber ein `loop()` genau dort meldete einen Verbindungsabbruch, den es nie gab. Dieselbe Vorsorge
  steht aus demselben Grund schon in `connectDetected()`.
  **Und die Verbindungsüberwachung ruht** (`processConnectionState()` kehrt früh um). Sie ruht auf
  zwei Beinen - Busverkehr als Lebenszeichen und die sekündliche Statusabfrage -, und im Busmonitor
  bricht das zweite weg, weil `U_State.req` dort `I` ist und keine Antwort erzeugt. Auf einem ruhigen
  Bus liefe die 5s-Frist deshalb *zwangsläufig* ab, `connectionLost()` setzte den Reconnect in Gang,
  und dessen `U_Reset.req` beendete den Busmonitor: er hielte nie länger als fünf Sekunden. Der Preis
  ist, dass eine im Busmonitor ausfallende BCU unbemerkt bleibt, bis der Modus verlassen wird - bei
  einem passiven Modus hinnehmbar.
  Dasselbe Muster beim Sendepfad: der Wächter steht vor dem `Await`-Zweig, weil ein Telegramm, das
  beim Umschalten schon auf die Bestätigung wartet, sonst nach `TPUART_TX_CONFIRM_TIMEOUT_MS` den
  Wachhund auslöste - und der schickt `U_Reset.req`, beendet also wieder den Busmonitor. Angefangene
  Telegramme frieren stattdessen ein und laufen nach dem Reset von vorn.
  Die Steuercode-Warteschlange ist bewusst von keiner Sperre betroffen: über sie geht der Reset
  hinaus, und der ist der einzige Weg heraus.
  Was der Modus einbringt, ist die Quittung *vom Bus*: `RxState::FrameAck` wartet nach einem
  CRC-gültigen Telegramm ein Byte länger und faltet es
  in die Flags des Telegramms, oder meldet es ohne `ACK`, wenn zuerst die bestätigte Pause zuschlägt -
  dann hat niemand quittiert.
  Das Quittungsbyte wird bewusst **nicht** an den Empfangspuffer angehängt; das Telegramm dort bleibt
  genau so, wie es empfangen wurde. Es liegt in **einem** Feld `_acknowledge`, nicht in einem je
  Herkunft: Busmonitor und eigene Quittung schließen einander aus (wir quittieren im Busmonitor nie,
  und außerhalb davon reicht der Chip keine Bus-Quittung weiter - Siemens S. 32), ein zweites Feld
  wäre also nur ein zweiter Weg zum selben Flag. Welche von beiden es war, trägt
  `ADDRESSED`/`DATA_CON`, nicht das Feld. Erkannt wird sie über `(v & L_ACKN_MASK) == L_ACKN_IND`
  (`0x33`/`0x00`, das Muster `x x 0 0 x x 0 0` aus Bild 35), und beide Flagpaare lesen sich
  **invertiert** - ein gesetztes Maskenbit heißt *nicht* busy / *nicht* nack (TP1: `0xCC` ACK, `0x0C`
  NACK, `0xC0` BUSY). Alle drei von Hand nachgeprüft.
  **Das Warten auf ein einzelnes Byte ruht auf Zeiten, und es braucht eine eigene Frist** - deshalb
  ist das Warten auf eine Antwort (`TPUART_FRAME_ACK_US`, 4000) getrennt von der Erkennung einer
  Buspause (`TPUART_FRAME_WAIT_US`, 2600). Durchgerechnet: die Quittung beginnt 15 Bitzeiten nach dem
  Telegramm (1,56ms bei 9600 auf dem Bus), das Oktett selbst dauert 11 Bit (1,15ms), und die
  Weiterleitung an den Host mit 19200 legt weitere 0,57ms drauf - sie landet also rund 3,3ms nach dem
  Telegrammende auf dem Bus. Unser Fenster beginnt aber nicht dort, sondern sobald wir das letzte
  Byte des Telegramms *gelesen* haben, was selbst ~0,57ms später ist - bleiben ~2,7ms. Gegen eine
  Pausenschwelle von 2600 schlüge die Pause zu, *bevor* die Quittung überhaupt eintrifft, ohne die
  Trennung wäre die Antwort also nie zu sehen; 4000 gibt ~1,3ms Reserve.
  **`TPUART_FRAME_WAIT_US` ist 2600, weil beide Datenblätter das so sagen**, und der Wert ist ein
  *Unterscheidungskriterium*, keine Schätzung: der NCN5130 markiert Frame-Enden - beim Senden wie
  beim Empfangen - mit `>= 2,6 ms Stille` (S. 40, Bilder 44-47 und 50-53), und Siemens sagt dem Host,
  er solle das Paketende "by supervising the EOP gap of 2 - 2,5 ms" erkennen (TP-UART 2 S. 32 / 2+
  S. 33). Ab 2,6ms ist es *garantiert* ein Frame-Ende; darunter rät man, darüber wartet man umsonst.
  Früher stand dort 2800, gemessen gegen den 5,2ms-Abstand zum nächsten Telegramm - die Notbremse
  statt des dokumentierten Frame-Endes, und für beide Chips 200µs zu spät.
  Die untere Schranke ist der Abstand *innerhalb* eines Telegramms: am Host sind das 1,354ms abzüglich
  der Übertragungszeit zur BCU, also ~0,78ms bei 19200 und ~1,07ms bei 38400 - bequem unter 2600. Die
  obere Schranke der alten Begründung gilt weiterhin als Rückfall: das nächste Telegramm kann nicht vor
  50 Bitzeiten (~5,2ms) beginnen, am Host beobachtet ~6,36ms. Beachte, dass das Datenblatt außerdem
  eine Quittungstoleranz von **30 Bitzeiten** dokumentiert ("time-out after 30 bit times", S. 38) -
  ein Gerät, das diesen Spielraum ausschöpft, landete bei ~4,8ms und würde selbst vom 4000er Fenster
  verpasst.
  Ein Byte in `FrameAck`, das keine Quittung ist, wird nicht erwartet; es wird defensiv behandelt
  (Telegramm melden, dann Resync, da dieses Byte bereits verbraucht und der Puffer belegt ist).
  Ein CRC-ungültiges Telegramm wartet **nicht** auf eine Quittung - der Telegrammrand selbst ist an
  diesem Punkt nicht vertrauenswürdig.
- **Die Verlustmelder liegen ebenfalls am `DataLinkLayer`**, aus demselben Grund: sie steuern nichts.
  `_interfaceOverflow`, `_rxQueueOverflow`, `_ctrlQueueOverflow` sind Information für die Schicht
  darüber, kein Zustand, auf den die Hälften reagieren - in ihnen waren sie Zustand, der nichts tat.
  Die Hälften rufen aus dem Tick `reportInterfaceOverflow()` / `reportRxQueueOverflow()` /
  `reportControlOverflow()`, und jede davon tut **beides**, was früher an jeder Aufrufstelle
  nebeneinanderstand: das einmalige Flag setzen *und* den Zähler hochzählen ("ist gerade etwas
  passiert" gegenüber "wie oft insgesamt").
- **Alle Callbacks liegen am `DataLinkLayer`, nicht in den Hälften** - Telegramm, Quittung und
  Meldung (`_callbacksReceivedFrame`, `_callbackCheckAcknowledge`,
  `_callbackMessage`). Er ist die Schnittstelle nach außen und die Stelle, an der sie registriert
  werden, also ist er auch die Stelle, die sie aufruft. Der `Receiver` baut das Telegramm und fragt:
  `_dll.checkAcknowledge()` danach, ob und was zu quittieren ist, `_dll.deliverFrame()` für die
  Übergabe. "Kein registrierter Callback heißt gar keine Quittung" sitzt deshalb ebenfalls dort, wie
  das alte `checkAcknowledge()`.
- **Die Pause wird nur gemessen, solange eine Sequenz offen ist.** `checkPause()` kehrt bei
  `RxState::Idle` sofort zurück - eine Pause kann immer nur etwas *beenden* (ein begonnenes
  Telegramm, einen Resync, eine ausbleibende Antwort), und zwischen zwei Telegrammen gibt es nichts
  zu beenden; die Länge kommt aus dem Längenoktett, nicht aus der Zeit. Das ist zugleich der
  billigste verfügbare Leerlaufpfad: auf einem ruhigen Bus ist `Idle` der Normalzustand, ein Tick
  spart dort den `micros()`-Aufruf der Pausenmessung und kostet stattdessen einen Vergleich.
  (Die Abkürzung spart nicht mehr *jeden* Zeitzugriff: seit der Taktmessung liest `tick()` selbst
  einmal `micros()` - siehe den Leerlaufpfad weiter unten.)
  Zwei Felder bleiben: `_emptySince` + `_emptyStarted` - seit wann das Interface nichts mehr liefert,
  und ob diese Messung überhaupt läuft. Das zweite trägt: der Bezugspunkt muss die *erste*
  Beobachtung von Stille sein, nicht das zuletzt gelesene Byte, sonst hielte ein stehengebliebener
  Tick seine eigene Lücke für eine Buspause. Ein drittes Flag "schon ausgelöst" wurde probiert und
  verworfen - die `Idle`-Abkürzung verhindert den Wiedereintritt bereits, weil jeder Zweig von
  `handleVerifiedPause()` in `Idle` endet. Einen Zweig dort einzubauen, der *nicht* in `Idle` endet,
  bricht diese Zusicherung.
- **Zwei Empfangspuffer plus das Telegramm, drei Aufgaben** - die immer wiederkehrende Frage "warum
  nicht einer?" beantworten ihre Besitzer. `_buffer` (263, **Tick**) ist die eintreffende Sequenz:
  zusammenhängend, weil das Zerlegen hineinindiziert und der Echovergleich `memcmp` benutzt - direkt
  in den Ring zu zerlegen hieße also Modulo-Indizierung im zeitkritischsten Pfad. `_queue` (1024,
  **Tick -> Loop**) ist die ISR-Grenze *und der Rückstau* - rund 80 Telegramme, was ihre eigentliche
  Aufgabe ist: ein Stillstand von einer Sekunde lässt den Bus ~740 Byte nachliefern. Das **`Frame`
  selbst** ist die Übergabe: es besitzt seine Bytes (siehe `Frame.h`) und wird je Eintrag auf dem
  **Stack von `processQueue()`** gebaut, der Ringplatz ist also freigegeben, bevor fremder Code
  läuft. Früher war es ein `_deliverBuffer`-Member des `Receiver` - eine Kratzfläche im C-Stil,
  hinter einem kurzlebigen Objekt; der Speicher gehört dem Typ, der die Daten darstellt. Kein Paar
  lässt sich zusammenlegen: `_buffer` gehört dem Tick und das Telegramm der Loop (dieselben Bytes
  hieße, der Tick überschreibt, was der Callback liest), und der Ring ersetzt das Telegramm nicht
  (er bricht um) und wird nicht von ihm ersetzt (er hält ein einzelnes Telegramm).
  **`Frame` besitzt ohne Heap**: ein festes `TPUART_BUFFER_SIZE`-Array, `Frame(length, flags)` und
  dann `data()` zum Füllen. Ein `malloc` je Telegramm im Empfangspfad würde für die Lebensdauer des
  Geräts im Bustakt allokieren und freigeben, neben den längerlebigen Blöcken der Sendeseite. Der
  Preis ist, dass ein Telegramm immer so groß ist wie das größtmögliche - auf dem Stack belanglos,
  wissenswert, wenn ein Aufrufer eines aufbewahrt.
- **Eine Stelle je Zustandsübergang.** `Receiver::resetSequence(nextState)` ist der einzige Code, der
  die Empfangsfelder löscht (Pufferlänge, Telegrammgröße, CRC, Quittung, Adressiertheit) - früher
  stand er am Sequenzbeginn, in `forceResync()` und in `completeSequence()` ausgeschrieben.
  `Transmitter::beginTransmission()` ist derselbe Gedanke für die Sendeseite (geholtes Telegramm und
  Neustart nach einem Reset). Es geht dabei nicht in erster Linie um Länge: ein Feld, das in einer
  von drei Kopien ergänzt und in den anderen vergessen wird, zeigt sich als Zustand, der ins
  *nächste* Telegramm durchsickert, und das ist das übelste Fehlerbild dieser Klasse.
- **Keine bequemen Weiterleitungen am `DataLinkLayer` für das, was die Hälften schon anbieten.**
  `rxState()`, `txState()`, `isTransmitting()`, `transmitQueueUsed/Size()` gab es und sind entfernt -
  `getReceiver()`/`getTransmitter()` erreichen dieselben Werte unmittelbar. Die Überlaufmelder sahen
  ebenfalls nach Weiterleitungen aus; sie sind keine, denn die Flags selbst sind hierher gewandert
  (siehe oben), es gibt sie also genau an einer Stelle und genau einen Weg, sie zu lesen.
- **`tick()` blockiert nie und verweilt nie** - es ist auf den späteren ISR-Einsatz gebaut, jeder
  Schritt steigt also so früh und so billig wie möglich aus. Es ist `processRx()` + `processTx()`,
  und beide haben ihre Wächter vorn: nichts empfangen -> RX übersprungen (nur der Pausenzähler
  läuft); auf der TX-Seite wird zuerst die Steuerwarteschlange geprüft (leer -> ein Indexvergleich),
  dann `_txState == Idle` -> Telegrammpfad ganz übersprungen; nicht genug Platz im Interface für
  `TPUART_TX_ATOMIC_BYTES` (3) -> auf einen späteren Tick verschoben.
  Die 3-Byte-Reservierung gibt es, weil ein Telegrammbyte von bis zu zwei Positionsbytes begleitet
  sein kann, die unmittelbar hintereinander hinausmüssen - vorab nach dem Platz zu fragen ist das,
  was eine gleichzeitig fällige Quittung daran hindert, dazwischenzurutschen. Es funktioniert, weil
  jedes Interface die Schreibreihenfolge wahrt. Steuergruppen reservieren stattdessen ihre eigene
  Länge (bis zu `TPUART_CTRL_MAX_GROUP` = 4).
- **Ein `tick()` = ein Byte je Richtung** (vom Anwender gesetzte Invariante, beibehalten): es wird
  höchstens ein empfangenes Byte zerlegt und höchstens ein *Telegramm*byte abgesetzt. "Ein Byte" auf
  der TX-Seite heißt ein Byte **des Telegramms** - die bis zu zwei Positionsbytes, die es begleiten
  können (`U_L_DataOffset.req` / `U_L_DataCont.req`), gehören dazu und gehen im selben Tick hinaus,
  wofür `TPUART_TX_ATOMIC_BYTES` (3) genau den Platz reserviert. Das Interface wird nie in einer
  Schleife leergezogen. Das macht einen Tick zu einem **Durchlauf mit fester Kostenobergrenze**,
  worauf der ISR-Plan beruht.
- **Zieltakt: alle ~100µs** (0,1ms). **Maßgeblich ist der Bus, nicht die Host-Strecke**: KNX TP1
  läuft immer mit 9600 Baud, und ein Zeichen belegt 13 Bitzeiten (Start, 8 Daten, Parität, Stop, dazu
  2 Bit Abstand), es trifft also höchstens alle **1,354ms** ein Byte ein - 738 Byte/s. Die 19200
  (oder 38400) zwischen Host und BCU sind mehr als das Doppelte; der Überschuss trägt unsere Sendungen
  und die Steuerbytes, er beschleunigt den Empfang nicht. Bei 100µs sind das ~13 Ticks je Byte, und
  ein Byte je Tick hat reichlich Reserve.
- **Der Leerlaufpfad muss so kurz wie möglich bleiben**, denn bei 100µs ist er der mit Abstand am
  häufigsten ausgeführte Code hier. Heutige Gestalt, gemessen auf dem RP2040: zwei bis drei
  MMIO-Lesevorgänge (`micros()` für die Taktmessung, der DMA-Transferzähler in `available()`, und
  `micros()` in `checkPause()`, solange ein Pausenfenster läuft) plus etwa ein Dutzend Vergleiche -
  deutlich unter 100 von den ~13.300 Takten, die bei 133MHz je Tick zur Verfügung stehen, also rund
  0,5% CPU. Die Reihenfolge der Wächter ist bewusst gewählt: `_initialized` und `_connected` zuerst,
  dann `_interface.available()`, und auf der TX-Seite `_txState == Idle` vor allem anderen. Muss das
  je billiger werden, sind die zwei naheliegenden Kandidaten, den `micros()`-Aufruf in `checkPause()`
  durch einen Tickzähler zu ersetzen und dem Leerlaufpfad eine Ja/Nein-Abfrage statt der vollen
  Anzahl von `available()` zu geben.
  **Der erste `micros()`-Aufruf ist eine bewusste Abkehr von "so wenig wie möglich"**, und er steht
  ausdrücklich *vor* allen weiteren Abbrüchen in `tick()`: gemessen werden soll der **Antrieb**, also
  wie oft wir gerufen werden, nicht wie oft wir etwas zu tun hatten. Er kostet rund 0,1% CPU und
  bezahlt damit den einzigen Wert dieser Schicht, der sich vorher überhaupt nicht ablesen ließ. Wie
  teuer das Raten stattdessen war, steht bei `getTicks()` in `Statistics.h`.
- **Das Interface gehört `tick()` allein.** Sobald der Timer-Antrieb läuft, darf nichts von außen
  `Interface::Abstract` anfassen - das wäre ein zweiter Zugriffskontext auf dieselben Zähler und
  Hardwareregister, und `RP2040::overflow()` löscht als Nebenwirkung sogar das Overrun-Flag des UART.
  Deshalb läuft die Überlaufmeldung über die Schicht: `tick()` rastet sie in `_interfaceOverflow`
  (nur im Bytepfad, denn ein Überlauf kann nur entstehen, während Daten fließen - der Leerlaufpfad
  bleibt frei von Hardwarezugriff), und der Hauptkontext liest sie über das einmalige
  `interfaceOverflow()`. Ein Aufrufer ruft folglich nichts mehr am `interface` selbst auf.
- **Pollend, nicht rückrufend**, im Einklang mit der Interface-Schicht: `tick()` ist dafür gedacht,
  wiederholt gerufen zu werden - entweder aus `loop()` oder aus einem Hardwaretimer. Immer nur eines
  von beiden darf es treiben.
- **Geteilter Zustand wird über Warteschlangen und Reihenfolge behandelt, nicht über Sperren.**
  - **Steuerwarteschlange**: ein Erzeuger (Hauptkontext), ein Verbraucher (`tick()`), der Kopf wird
    **nach** dem vollständigen Schreiben des Eintrags veröffentlicht. Das *ersetzte* ein Besitzschema,
    in dem `TxState` die Marke für `_txBuffer` war und `queueControl()` verweigerte, sobald der Tick
    sie hielt. Dieses Schema hatte zwei Probleme: es machte Steuercodes während eines Telegrammversands
    unbenutzbar, und jede Nutzlast, die die Marke nicht abdeckte (das Wunschflag für den Busmonitor),
    musste vor dem Veröffentlichen geschrieben werden - was sie nicht wurde, mit einem echten Fehler
    dauerhafter Divergenz als Folge.
  - **RX-Warteschlange**: dieselbe Form, andere Richtung - ein Erzeuger (`tick()`), ein Verbraucher
    (`loop()`), Kopf nach Fertigstellung des Eintrags veröffentlicht.
  - **TX-Puffer** (`_txBuffer`/`_txLength`): `TxState` ist seine Besitzmarke - `Idle` heißt, der
    Hauptkontext darf den Puffer füllen, alles andere heißt, der Tick besitzt ihn. Die Regel dabei
    ist die Lehre aus dem Steuercodefall: *alle* Nutzlast gehört hinter das Veröffentlichen.
  Eine blockierende Sperre käme ohnehin nicht in Frage: auf dem RP2040 läuft `tick()` in einem
  Interrupt, und ein Interrupt kann nicht blockieren. Ein einzelnes `volatile bool` funktionierte auf
  dem RP2040 (nur der Hauptkontext schreibt es, und ein ISR kann vom Hauptloop nicht unterbrochen
  werden), aber **nicht** auf dem ESP32, wo der Tick-Task wirklich parallel auf Kern 0 läuft - dort
  öffnet sich das klassische Fenster zwischen Prüfen und Handeln.
  - **Callbacks**: die beiden sind *nicht* gleichartig, und das ist wichtig.
    `registerFrameCallback()` ist unkritisch - der Telegramm-Callback wird nur aus `loop()` gerufen,
    demselben Kontext, in dem er gesetzt wird. Genau das bringt der Ringpuffer ein.
    `registerCheckAcknowledge()` ist der **eine Callback, der aus `tick()` gerufen wird**: die
    Quittungsentscheidung fällt bei Byte 6 eines noch eintreffenden Telegramms, innerhalb des
    Quittungsfensters, sie lässt sich also nicht über den Ringpuffer aufschieben - das Telegramm
    existiert als Eintrag noch gar nicht. Zwei Folgen: die Funktion muss kurz sein und darf weder
    blockieren noch allokieren (auf dem RP2040 läuft sie in einem Interrupt), und sie muss **vor dem
    Start des Tick-Antriebs** gesetzt werden. In der Praxis werden beide einmal beim Hochlauf gesetzt,
    es braucht also keine Absicherung; wer je zur Laufzeit tauschen will, beachte, dass eine
    `std::function`-Zuweisung nicht atomar ist - sie gibt intern frei und allokiert neu - und eine
    Critical Section oder einen veröffentlichten Tauschplatz bräuchte.
  **Keine Datei hier enthält überhaupt Sperren oder plattformspezifische Synchronisation** - frühere
  Entwürfe hatten `noInterrupts()` und später einen `portMUX`; beides erwies sich als überflüssig,
  sobald Besitz und Reihenfolge ausdrücklich festgelegt waren. Halte es so.
- **Quittung (im `Receiver` entschieden, vom `Transmitter` geschrieben)**: sie läuft genau in dem
  Moment, in dem `_frameSize` erstmals bekannt wird - 6 Byte hinein bei einem Standardtelegramm, 7
  bei einem erweiterten - und schreibt `U_Ackn.req` (`0x10 | nack<<2 | busy<<1 | addressed`, NCN5130
  Tabelle 12) direkt ins Interface. Das ist der früheste Punkt, an dem alles Nötige vorliegt: Ziel
  (`[3][4]` standard, `[4][5]` erweitert), Adresstyp (Gruppenbit in `[5]` standard, `[1]` erweitert)
  **und die verbleibende Telegrammlänge**.
  Es heißt zugleich, dass kein "schon quittiert"-Flag nötig ist - die Größe wird je Telegramm genau
  einmal bekannt, dieser Zweig läuft also von Bauart her genau einmal.
  Die Quittung muss hinaus, **während das Telegramm noch läuft** - der Chip legt die sofortige
  Quittung unmittelbar hinter dem Prüfsummenoktett auf den Bus (Datenblatt Bilder 50-52), es gibt also
  keine Gelegenheit nach dem Telegramm. Das TPUART2+-Datenblatt beziffert es: der Service "must be sent
  latest 2,8 ms after receiving the address type octet of an addressed frame" (S. 25) - dieselben
  2,8ms, die `TPUART_FRAME_WAIT_US` begrenzen, und eine unabhängige Bestätigung, dass zwei
  Buszeichenzeiten hier die maßgebliche Einheit sind.
- **Die Quittung wird unterdrückt, wenn wir hinter dem Bus liegen.** `sendAcknowledge()` vergleicht
  zuerst `_interface.available()` mit `_frameSize - _rxLength` - der Zahl der noch ungelesenen Bytes
  dieses Telegramms. Warten **mindestens so viele** bereits, dann ist das Telegramm samt
  Prüfsummenoktett vollständig eingetroffen, es ist auf der Leitung also längst vorbei und sein
  Quittungsfenster zu. Dann zu quittieren wäre schlimmer als zu schweigen - der Chip hängte sie ans
  *nächste* Telegramm und bestätigte damit ein Telegramm, das nie geprüft wurde. Deshalb liefert
  `Abstract::available()` eine Anzahl statt eines Wahrheitswerts.
  Der Vergleich ist `>=`, und der Unterschied zählt: mit `>` schlüpfte der Gleichheitsfall durch, und
  dieser Fall ist zeitlich unbegrenzt - da ein folgendes Telegramm weitere 50 Bitzeiten nicht beginnen
  kann, gilt `available() == remaining` auch nach einem *beliebig* langen Stillstand. `>` ließ also
  genau die Quittung durch, die die Prüfung verhindern soll. Der Normalbetrieb ist unberührt (0-1
  wartende Bytes gegen mindestens 2 ausstehende).
  Welche Quittung gesendet wird, entscheidet `registerCheckAcknowledge(std::function<AckType(uint16_t
  destination, bool isGroupAddress)>)`. **Ohne registrierten Callback wird überhaupt nichts
  quittiert** - `checkAcknowledge()` liefert dann `AckType::None`.
  Alles zu quittieren ist die Entscheidung eines Aufrufers, nie eine Vorgabe: ein Gerät, das jedes
  Telegramm quittiert, behauptet, unter jeder Zieladresse auf dem Bus erreichbar zu sein.
  (`std::function` ist hier eine bewusste Ausnahme vom sonst STL-abgeneigten Stil dieses Projekts,
  vom Anwender so entschieden. Eine zuzuweisen ist nicht atomar -
  intern wird freigegeben und neu allokiert -, sie muss also vor dem Start des Tick-Antriebs gesetzt
  werden. Eine `noInterrupts()`-Klammer gibt es bewusst **nicht**: sie hülfe auf dem ESP32 nicht, wo
  der Tick als Task auf dem anderen Kern läuft, und die Vorbedingung deckt beide Plattformen ab. Eine
  frühere Fassung dieser Datei behauptete, es gebe eine solche Klammer; die gab es nie.)
- **Steuerbytes werden roh und unklassifiziert durchgereicht.** Gegen das NCN5130-Datenblatt geprüft
  (`docs/Onsemi_NCN5130.pdf`, Tabelle 13 "Services to Host Controller", S. 34): jede Indikation in der
  Gruppe **Control Services** ist genau 1 Byte lang und aus dem rohen Wert allein verständlich
  (`U_Reset.ind`, `U_State.ind`, `U_FrameState.ind`, `U_Configure.ind`, `U_FrameEnd.ind`,
  `U_StopMode.ind`, `L_Ackn.ind`, `L_Data.con`) **außer `U_SystemStat.ind`, das 2 Byte hat**. Es gibt
  also keine Typisierung `Reset`/`State`/`Configuration`/... - die Schicht behandelt die eine
  2-Byte-Ausnahme intern (über den Zwischenzustand `RxState::Control`) und überlässt die Deutung dem
  Aufrufer. `U_SystemStat.ind` ist zusätzlich an `BcuType::Ncn5120` gebunden, wie in der alten
  Library - auf einem TPUART2 bedeutet `0x4B` etwas anderes und würde sonst das folgende Byte
  verschlucken. Erreichbar ist der Zweig nur über `requestState()`, da der Chip es ausschließlich als
  Antwort auf `U_SystemState.req` sendet.
  **`L_Poll_Data.ind` (`0xF0`) ist hier die Falle, und es hat einen eigenen Zustand.** Es gehört zur
  Gruppe *transparent DLL*, nicht zu den Steuerdiensten. Auf dem **Bus** ist ein Poll mehrbytig (Bild
  56, Siemens Bild 23: Steuerbyte, Quelle ×2, Polladresse ×2, Slotzahl, Prüfsumme, dann ein Byte je
  Slot, bis zu 15). **Auf der Host-Leitung ist er das normalerweise nicht.** Siemens TP-UART 2 S. 32 /
  2+ S. 33, wörtlich: *"From a L_PollData-request only the Controlbyte is transmitted to the host if
  the TP-UART is a polling slave. If the TP-UART is polling master the complete polling frame is
  transmitted to the host as well if a collision is detected during sending the polling master
  frame."* In Bild 56 stehen die übrigen Bytes entsprechend in der Spalte *KNX Bus*, nicht in der des
  Hosts. Was der Chip weiterreicht, wenn er weder Master noch Slave ist, steht nirgends.
  Beide Fälle deckt `RxState::Poll` mit `processPollByte()` ab, gebaut als Spiegelbild von
  `processFrameByte()`: die Kopflänge steht fest, die Slotzahl ergibt die Gesamtlänge, ein voller
  Zyklus wird also **abgezählt und ohne Warten auf eine Pause abgeschlossen**. Folgt nichts - der
  Normalfall -, schließt `handleVerifiedPause()` die Sequenz, und ein einzelnes `0xF0` wird bewusst
  **nicht** als `INVALID` markiert: daran ist nichts kaputt. Nur ein begonnener und dann
  abgeschnittener Zyklus ist es. Nie quittiert: ein Poll trägt keinen Adresstyp, und antworten hieße,
  den eigenen Slot über `U_PollingState.req` zu füllen, was diese Library nicht benutzt.
  Was nicht passieren darf, ist, es als 1-Byte-Steuerbyte zu behandeln: der Parser läse dann die
  Nutzlast als neue Sequenzen - und ein Master an Adresse 1.0.x hat als oberes Quellbyte `0x10`, was
  *ein* gültiger Anfang eines erweiterten Telegramms ist, er baute also ein Phantomtelegramm und
  könnte ein `U_Ackn.req` für eine aus Polldaten zusammengesetzte Adresse absetzen. Eine fremde
  Quittung auf den Bus zu legen ist das übelste Fehlerbild dieser ganzen Schicht. Der naheliegende
  Kurzschluss - `0xF0` byteweise als "unerwartet" verwerfen und die Telegrammerkennung beim ersten
  Pollbyte wieder aufnehmen, das wie ein Anfang aussieht - führt genau dorthin.
  `controlServiceName()` benennt `0xF0`, damit ein erkannter Poll als `L_Poll_Data.ind` gemeldet wird
  statt als Fehler `Unknown control byte F0`. Kein Verbraucher dieser Library (KNX-Stack, OFM-Network,
  OGM-Common) kennt Polldaten überhaupt, weshalb nichts als Telegramm ausgeliefert wird -
  `Frame::isFrame()` ist für `0xF0` falsch, der Eintrag geht also an `handleControlEntry()`. Echten
  Poll-Verkehr gibt es in keiner heutigen Anlage.
- **Die Telegrammgröße** kommt aus `metadataSize() + apduSize()` (standard = 8 + `data[5]&0x0F`,
  erweitert = 9 + `data[6]`), nicht aus der Zeit.
- **Fortlaufende CRC8**: der laufende XOR-Akkumulator wird mit jedem Telegrammbyte fortgeschrieben
  *außer* dem, das an der Prüfsummenposition landen wird (bekannt, sobald der Kopf zerlegt ist) - die
  Prüfung beim Abschluss ist damit ein O(1)-Vergleich (`~crc == data[frameSize-1]`), ohne zweiten Lauf
  über den Puffer.
- **Die Pausenprüfung beruht bewusst NICHT auf "Zeit seit dem letzten gelesenen Byte"**: ein
  Timer/ISR kann ausgehungert werden (pausiert, oder der Hauptloop blockiert, etwa durch ein
  periodisches `delay()` in einer Testumgebung), ohne dass auf dem Bus eine echte Lücke war -
  `interface.available()` gibt den DMA-Puffer wahrheitsgemäß wieder, unabhängig davon, ob *unsere*
  Verarbeitung mitkam. Die Schicht verfolgt deshalb den Zeitstempel der *ersten* Beobachtung von
  "nichts verfügbar" (`_emptySince`); erst wenn dieser Zustand ununterbrochen
  `TPUART_FRAME_WAIT_US` (2600, also 2,6ms - und `TPUART_FRAME_ACK_US`, 4000, während in `FrameAck`
  auf eine Antwort gewartet wird) angehalten hat, gilt er als echte Buspause. Jedes neu eintreffende
  Byte setzt diese Verfolgung sofort zurück.
  **Was tatsächlich herauskommt, hängt am Tick**: die Messung beginnt beim ersten Tick, der "nichts
  da" sieht, und schlägt beim ersten Tick jenseits der Frist zu, die wirksame Pause ist also die
  Schwelle plus bis zu zwei Tickintervalle. Bei 500µs sind das 2,6-3,6ms - eine Lücke von genau 2,6ms
  wird *nicht* erwischt; bei 250µs oder auf `loop1` schon. Der Weg dorthin ist ein schnellerer Tick,
  keine kleinere Schwelle: unter 2,6ms verlässt man, was die Datenblätter zusichern.
- **Ein Moduswechsel löst ebenfalls einen Resync aus** (siehe Busmonitor oben) - dieser verwirft die
  laufende Sequenz, ohne sie zu melden, da er Folge unseres eigenen Handelns ist und nicht eines
  Busproblems.
- **`Frame` ist überall ein VOLLSTÄNDIGES Telegramm einschließlich Prüfsumme**, in beide Richtungen
  und an jedem Einstiegspunkt. Daran hängt mehr, als es aussieht: solange `sendFrame()` es *ohne*
  erwartete und `pushTransmitQueue()` *mit*, hatten dieselben Bytes zwei Bedeutungen, und die
  KOMPAT-Form musste die Länge um eins kürzen. Es gibt kein `+1`/`-1` mehr.
  **`isValid()` rechnet nach und liest zugleich das Flag** - vier Bedingungen: `INVALID` nicht
  gesetzt, Steuerbyte ist L_Data, `length() == size()` (das erledigt Mindestlänge und Vollständigkeit
  in einem), Prüfsumme stimmt. Beide Anteile tragen etwas Eigenes, keiner reicht allein: das Flag
  stützt sich auf Wissen des Empfängers, das den Bytes nicht mehr anzusehen ist (eine Pause mitten im
  Telegramm), und die Rechnung ist das Einzige, was bei einem selbst gebauten *Sende*telegramm etwas
  aussagt, wo die Flags immer 0 sind. Das Flag steht vorn, ein bereits gemeldetes Telegramm kostet
  also nicht einmal den Durchlauf. `isInvalid()` ist die Verneinung davon.
  **Die Längenableitung gibt es genau einmal**: `Frame::sizeOf(data, available)`, statisch, benutzt
  von `Frame::size()`, vom Empfänger (`processFrameByte()`) und von der Sendewarteschlange. Statisch,
  weil `sizeof(Frame)` 263 Byte ist - im Tick je Telegramm und in jedem `front()` wäre ein Objekt der
  falsche Preis. Sie stand einmal zusätzlich inline im Empfänger; zwei Kopien einer Regel laufen
  früher oder später auseinander. Dasselbe gilt für die 9-gegen-8 (`metadataSizeOf()`) und für die
  Prüfsumme (`calcCRC8()`).
  **`Frame::size()` geht dabei über ein nullgefülltes Kopf-Array**, und das ist kein Zierrat: `at()`
  liefert jenseits von `length()` eine 0, `size()` beantwortet damit "wie lang *sollte* es sein" auch
  bei einem Fragment. `sizeOf()` meldet dort 0 ("noch nicht entscheidbar"), und das wäre hier
  gefährlich - `cemiSize()` rechnet `size() - 1`, mit einer 0 entstünde ein 1-Byte-`malloc`, in das
  `cemiData()` dann hineinschriebe.
  `isTruncated()` gab es einmal; es hatte einen einzigen Nutzer (`printFrame()`) und unterschied
  "abgeschnitten" von "Prüfsumme falsch". Beide tragen `INVALID`, und niemand reagiert verschieden
  darauf - kaputt ist kaputt.
- **`Statistics`** (`Statistics.{h,cpp}`, erreichbar über `getStatistics()`). Die Zähler werden aus
  `tick()` hochgezählt und aus dem Hauptkontext gelesen, daher
  `volatile` und einfache Inkremente - ein Schreiber je Zähler. `reset()` aus dem Hauptkontext kann
  mit einem Inkrement kollidieren; für eine Statistik wird das hingenommen statt gesperrt.
  **Das Namensschema ist verbindlich**: Richtungspräfix immer (`getRx…`/`getTx…`), die Einheit steht
  im Namen (`…Frames` zählt Telegramme, `…Bytes` zählt Bytes, `…Overflows`/`…Losses` zählen
  Ereignisse), nichts wird abgekürzt. Wer einen Zähler ergänzt, hält sich daran - sonst ist in einem
  Jahr wieder unklar, ob ein `getInterfaceOverflows` den Empfang oder den Versand meint (es war der
  Empfang, daher heißt er heute `getRxInterfaceOverflows`).
  **Eine Zahl, ein Name.** `getRxFrameBytes()` und `getRxBusBytes()` lieferten dasselbe Feld unter
  zwei Namen - genau die Doppelung, die diese Klasse an ihrem Vorgänger kritisiert. `getRxFrameBytes()`
  ist die echte Methode, `getRxBusBytes()` nur noch KOMPAT.
  **Der KOMPAT-Block enthält ausschließlich Namen, die es in v1 schon gab und die ein Verbraucher
  aufruft.** Neue Namen bekommen dort *nichts*: solange 2.0 in Arbeit ist, kann sich auf sie noch
  niemand stützen, sie dürfen also direkt heißen, wie sie heißen sollen. Nicht nachgerüstet werden die
  drei ungenutzten Dubletten aus v1 (`getRxOverflowInterface`, `getRxOverflowFrameBuffer`,
  `getRxOverflowSearchBuffer`) - dort gab es jedes dieser Ereignisse unter zwei Namen, und die
  Verbraucher rufen durchweg nur die eine Schreibweise.
  **Gesendete Telegrammbytes gibt es bewusst nicht als eigenen Zähler**, sie sind ableitbar - aber
  nicht so einfach, wie es aussieht: die Quittung geht ebenfalls über `Transmitter::writeByte()` und
  steckt damit in `getTxBytes()`. Richtig ist
  `getTxBytes() - getTxControlBytes() - getTxAcknowledges()`, denn eine Quittung ist genau ein Byte.
  **Ereignis und Umfang sind zwei Fragen.** `getRxResyncs()` zählt, wie oft die Position im Bytestrom
  verloren ging, `getRxDroppedBytes()` was es gekostet hat - drei Resyncs zu je 5 Byte und einer zu 15
  ergeben dieselbe Byte-Zahl bei völlig verschiedenem Befund. Gezählt wird an der einen Stelle, an der
  alle Wege in den Resync zusammenlaufen (`resetSequence()`); doppelt zählt dabei nichts, weil
  `forceResync()` bei bereits laufendem Resync früh umkehrt.
  **Die Fehler, die der Chip selbst meldet, werden einzeln gezählt** - `getChipSlaveCollisions()`,
  `getChipReceiveErrors()`, `getChipTransmitErrors()`, `getChipProtocolErrors()`,
  `getChipTemperatureWarnings()`. Nötig ist das, weil `_stateErrors` die Bits aus `U_State.ind` nur
  ODER-akkumuliert und beim Ausgeben löscht: der Chip meldet jedes Ereignis genau einmal, "einmal vor
  Stunden" und "dauernd" sähen dort gleich aus. Einzeln statt als Summe, weil es fünf verschiedene
  Diagnosen sind - eine stehende Übertemperaturwarnung ist etwas anderes als gelegentliche Kollisionen.
  Ein gemeinsamer Zähler über alle fünf fehlt **bewusst** - er sagte nichts, was diese hier nicht
  besser sagen, und wäre nachträglich auch nicht zu bilden, weil ein `U_State.ind` mehrere Bits zugleich
  tragen kann. Die Zuordnung Bit → Zähler steht im `DataLinkLayer`, nicht in `Statistics` - dafür
  braucht es die Protokollkonstanten, und die Zählerklasse soll keine kennen. Kein `Rx`/`Tx`-Präfix:
  das sind Zustände des Chips, keine Richtung von uns aus (TE und SC entstehen beim Senden auf dem Bus,
  RE beim Empfangen - ein gemeinsames Präfix wäre in jedem Fall falsch).
  **Höchststände statt nur Überläufe.** `getRxQueuePeakBytes()`, `getTxControlQueuePeakBytes()` und
  `getTxQueuePeakBytes()` beantworten die Auslegungsfrage, *bevor* etwas überläuft - ein
  Überlaufzähler sagt nur, dass es zu spät war. Sie hängen sich an einen Füllstand an, der an der
  jeweiligen Stelle ohnehin ausgerechnet wird, es kommt also nur ein Vergleich dazu. Alle drei zählen
  **Bytes**; die Sendequeue tat das früher in Telegrammen, seit sie ein Bytepuffer ist, misst sie sich
  wie die beiden anderen. Zu lesen sind sie gegen `TPUART_RX_QUEUE_SIZE`, `TPUART_CTRL_QUEUE_SIZE` und
  `TPUART_TX_BUFFER_SIZE`.
  **Gezählt werden Probleme, nicht Vorgänge.** Deshalb gibt es `getConnectionLosses()` und
  `getTxConfirmTimeouts()`, aber keinen Zähler für Resets und keinen für negative Bestätigungen: ein
  negatives `L_Data.con` sagt nur, dass auf dem Bus niemand quittiert hat - eine Aussage über den Bus,
  kein Fehler der Strecke. `getTxConfirmTimeouts()` zählt dagegen den Fall, in dem **überhaupt keine**
  Bestätigung kam und der Wachhund die BCU zurücksetzen musste; das zeigt auf den Chip oder die
  Verkabelung. Resets zu zählen wurde verworfen, weil allein die Baudratenerkennung mehrere schickt -
  ohne Ursache ist die Zahl wertlos.
  **Der Unterschied, auf den es ankommt: kaputte Telegramme und verworfene Bytes sind nicht
  dasselbe** und werden getrennt gezählt. Ein kaputtes Telegramm *wurde gemeldet* (mit
  `TP_FRAME_FLAG_INVALID` - CRC-Fehler, von einer Pause abgeschnitten, beschädigtes Längenoktett), der
  Verbraucher hat es also gesehen. Verworfene Bytes hat nie jemand gesehen, und dafür gibt es drei
  Quellen: alles, was während `Resync` verbraucht wurde; die Reste einer von einem Moduswechsel
  abgebrochenen Sequenz; und ein fertiger Eintrag, für den im RX-Ring kein Platz mehr war. Ein
  gemeinsamer Zähler "verworfen" für kaputte Telegramme und verlorene Bytes wäre unbrauchbar - führe
  ihn nicht ein.
  **Die Kategorien sind keine Zerlegung**: ein mangels Ringplatz verworfenes Telegramm steht mit
  seinen Bytes auch in `getRxFrameBytes()`, denn über den Bus kam es, und daran hängt die Buslast.
  `getTxAcknowledgesSuppressed()` lohnt die Beobachtung: er zählt den Fall "wir liegen hinter dem Bus",
  was korrektes Verhalten ist und kein Fehler - ein steigender Wert heißt aber, dass der Tick nicht oft
  genug drankommt. **Er ist dabei nicht durch Fremdverkehr aufgebläht**: `sendAcknowledge()` fragt den
  Quittungs-Callback *vor* der Rückstandsprüfung, ein abgelehntes Telegramm kommt also gar nicht bis
  dorthin. Ohne diese Reihenfolge zeigte ein Gerät ohne eigene Adressen Rückstände in Höhe des gesamten
  Busverkehrs an, und ein echter Rückstand wäre darin nicht mehr zu finden - festgehalten in
  `test_unaddressed_frame_is_not_acknowledged`, das den Fall bewusst *mit* vollem Rückstand fährt.
- **Der Takt misst sich selbst** (`getTicks()`, `getTickDeferrals()`, `getTickLastDeferredUs()`,
  `getRxInterfacePeakBytes()`), und
  das ist die Antwort auf ein wiederkehrendes Problem: der Antrieb ist die Voraussetzung für alles in
  dieser Schicht, war aber der einzige Wert, den man **nicht ablesen konnte**. Sichtbar waren nur seine
  Folgen - unterdrückte Quittungen, Interface-Überläufe -, und die Ursache musste geraten werden.
  Die Werte trennen verschiedene Krankheitsbilder, und erst zusammen ergeben sie eine Diagnose:
  - `getTicks()` gegen die Laufzeit ist die **mittlere** Rate. Deutlich unter dem eingestellten Soll
    heißt: der Timer treibt diese Instanz gar nicht, der Hauptloop tickt - dasselbe sagt `usesTimer()`.
  - `getTickDeferrals()` sagt, **wie oft** der Tick aufgehalten wurde (Schwelle
    `TPUART_TICK_DEFERRED_US`, die Zeichenzeit des Busses), `getTickLastDeferredUs()`, **wie lang die
    letzte** dauerte. Ein guter Mittelwert bei vorhandenen Verzögerungen heißt: der Antrieb stimmt, wird
    aber zwischendurch blockiert.
    **Bewusst die letzte und nicht die schlimmste.** Ein Höchstwert SÄTTIGT - nach einem einzigen
    Flash-Schreibvorgang stünde dort für immer eine fünfstellige Zahl, und er sagte danach nichts mehr
    über den aktuellen Zustand. Es gab dafür einmal `getTickGapMaxUs()`; der Wert ist aus genau diesem
    Grund entfernt worden. Wer wissen will, wann es passiert, liest den Zähler vor und nach der
    verdächtigen Aktion ab.
  - `getTickDurationMaxUs()` und `getCheckAcknowledgeMaxUs()` beantworten die **umgekehrte** Frage: nicht,
    wie oft der Tick aufgehalten wurde, sondern wie lange er selbst braucht. Daran hängt die
    Prioritätswahl - wer andere Interrupts verdrängen will, muss belegen können, dass er sie nur kurz
    aufhält. Gemessen wird nur der volle Durchlauf; die frühen Abbrüche sind ein paar Vergleiche und
    würden das Bild beschönigen. Die zweite Zahl ist der Anteil des **Aufrufers**: der Quittungs-Callback
    ist der einzige unbegrenzte Teil des Ticks (alles andere ist ein Byte je Richtung), und liegen beide
    dicht beieinander, gehört die Laufzeit ihm und nicht dieser Schicht. Beides sind Höchstwerte und
    sättigen deshalb - anders als bei `getTickLastDeferredUs()` ist das hier richtig, weil die Frage
    "wie schlimm kann es werden" lautet und nicht "wie steht es gerade".
  - `getRxInterfacePeakBytes()` ist derselbe Befund in der Einheit, in der er entsteht: 0-1 ist gesund
    (der Bus liefert höchstens alle 1,354ms ein Byte, der Tick holt alle 500µs eines ab), ab 2 wird bei
    Byte 6 die Quittung unterdrückt. Kostenlos erhoben, weil `Receiver::process()` `available()` ohnehin
    fragt.
  Die Grenze, die zählt, ist **~2700µs**: darüber ist ein Standardtelegramm vollständig eingetroffen,
  bevor der Tick bei Byte 6 die Entscheidung trifft.
  `_tickLastUs` gehört dem Tick und wird in `begin()` zurückgesetzt - die Pause zwischen `end()` und
  `begin()` ist kein Aussetzer des Antriebs, und ohne das Zurücksetzen stünde sie für immer als
  Höchstwert da, ausgerechnet auf einem Gerät, das die BCU neu verbindet (`test_tick_gap_survives_restart`).
  `DataLinkLayer::tickInterval()` liefert dazu das **eingestellte** Soll. Die drei Auskünfte sind
  bewusst getrennt: `usesTimer()` sagt, *wer* tickt, `Timer::interval()`, wie schnell es *gedacht* war,
  und `getTicks()`, was daraus *geworden* ist.
- **Und der Antrieb meldet sich selbst, wenn er die Untergrenze reißt** (`checkTickRate()`, aus `loop()`,
  Fenster `TPUART_TICK_RATE_WINDOW_MS` = 2000). Das ist die Lehre aus dem Fall, der diese ganze Messung
  ausgelöst hat: im Router lag die Rate bei 457/s statt 2000/s, und **sichtbar war davon ausschließlich der
  Folgeschaden** - 12% unterdrückte Quittungen und ein Rückstand von 7 Byte. Die Ursache stand nirgends;
  sie war nur auf ausdrückliche Nachfrage über `usesTimer()` zu erfahren, und danach fragt im Betrieb
  niemand. Nach dem Umschalten auf den eigenen Timer: 0 unterdrückte Quittungen, unverändert am
  Protokollpfad.
  Zwei Entwurfsentscheidungen daran sind tragend:
  - **Gemessen wird die ERREICHTE Rate, nicht die eingestellte.** Ein Wächter auf `usesTimer()` hätte
    genau den Fall verfehlt, in dem jemand absichtlich selbst tickt (`Timer::trigger()`, eigener Task) und dabei zu langsam
    ist - und das war hier fast der Fall.
  - **Die Schwelle kommt aus dem Bus, nicht aus der Konfiguration**: `TPUART_TICK_DEFERRED_US` ist 1354, die
    Zeichenzeit auf TP1. Ein Tick bewegt ein Byte je Richtung; liegt der mittlere Abstand darüber, holt die
    Schicht weniger Bytes ab, als der Bus im Vollausbau liefert (738 Byte/s), und **keine Puffergröße hilft
    dagegen**. Unterhalb ist alles gut, oberhalb ist es grundsätzlich kaputt - das ist keine Empfehlung,
    sondern eine Untergrenze.
  Gemeldet wird **einmal je Störung**, nicht je Fenster; erholt sich die Rate, wird der Melder wieder
  scharf. Der Fall "gar kein Tick im Fenster" ist ausdrücklich behandelt, er wäre sonst eine Division durch
  null. Zwei Testfälle hängen daran, und der zweite ist der wichtigere:
  `test_deferred_tick_is_reported` und `test_healthy_tick_is_not_reported` - ein Wächter, der grundsätzlich
  meldet, wäre ohne den zweiten genauso "grün" wie der richtige und im Betrieb reines Rauschen.
  **Ein guter Mittelwert schließt das Problem nicht aus**, und im Router steckten dahinter *zwei*
  verschiedene Ursachen, die sich im blossen Höchstwert nicht unterscheiden ließen - erst
  `getTickDeferrals()` hat sie getrennt:
  - **Dauerhaft, 2-4 mal je Sekunde**: ein gleichrangiger Interrupt auf 0x80. Behoben durch den eigenen
    Alarmpool mit angehobener Priorität (siehe `Timer`), Ergebnis 0 Verzögerungen über 26000 Ticks bei einem
    Höchstwert von 520µs gegen 500µs Intervall.
  - **Einmalig und riesig**: ein Flash-Schreibvorgang. **Auf BEIDEN Plattformen gemessen** - RP2040
    53176µs, ESP32 40033µs -, es ist also keine Eigenheit des einen Ports. Der Mechanismus unterscheidet
    sich, die Wirkung nicht: der RP2040 sperrt über `noInterrupts()`/`idleOtherCore()` das XIP (PRIMASK
    wirkt unabhängig von jeder IRQ-Priorität, und `idleOtherCore()` parkt Kern 1 gleich mit - ein Tick
    dort wäre also ebenfalls betroffen), beim ESP32 wird der Instruction-Cache abgeschaltet und der
    `esp_timer`-Task kann nicht aus dem Flash laufen. Aus der Library heraus ist beides nicht behebbar.
    **Die Dauer, die die Anwendung für den Speichervorgang meldet, ist NICHT der Stillstand** - sie
    beantwortet eine andere Frage, und beide Zahlen sind für sich richtig. Gemeldet wird die Dauer des
    integritätskritischen Fensters: so lange muss die Stromversorgung bei einem Ausfall überbrücken,
    damit die Daten vollständig geschrieben sind. Auf dem RP2040 wird der Schreibbereich VORAB gelöscht
    und der Erase des alten Blocks läuft danach - das kritische Fenster ist damit kurz ("2ms"), der
    Tick-Stillstand umfasst aber auch den nachgelagerten Erase (53176µs). Auf dem ESP32 fallen beide
    zusammen ("47ms" gegen 40033µs), weil der Erase dort im Schreibvorgang steckt.
    Für die Auslegung von Fristen dieser Schicht zählt der Stillstand, nicht die gemeldete Schreibdauer.
  **Was ein solcher Stillstand kostet, ist für Puffer und Quittung VERSCHIEDEN**, und die Messung oben
  zeigte nur deshalb keinen Schaden, weil der Bus fast leer war - das ist kein Freibrief:
  - **Der Empfangspuffer hält, und das ist rechenbar.** Die DMA ist eigenständig und füllt den Ring
    weiter, während die CPU steht; sie braucht keine Interrupts. 52ms bei voller Buslast (738 Byte/s)
    sind 38 Byte gegen einen 256-Byte-Ring - überlaufen würde er erst bei rund 350ms.
  - **Die Quittung hält nicht - und mehr als sie kann auch nicht verlorengehen.** Jedes Telegramm, dessen
    Byte 6 in das Fenster fällt, verliert seine Quittung; bei voller Last sind das drei bis vier je
    Stillstand. Dass in der Messung keines betroffen war, lag daran, dass nur 3% des Verkehrs an dieses
    Gerät gerichtet waren und der Bus ruhig lag - Glück, kein Verdienst.
    **Das Telegramm selbst geht dabei nicht verloren**: es liegt im Ring, wird nach dem Stillstand
    zerlegt und ausgeliefert, und die Wiederholungen des Absenders kommen ebenfalls an (als `FILTERED`
    markiert). Die Kosten liegen beim ABSENDER - drei Wiederholungen Busbandbreite und eine Übertragung,
    die er für gescheitert hält, obwohl sie ankam. Empfangsverlust setzt einen Ringüberlauf voraus, und
    der bräuchte die oben gerechneten ~350ms.
  Zugleich ist es der Beleg dafür, dass die Pausenerkennung richtig aufgehängt ist: `_emptySince` misst ab
  der ersten Beobachtung von Stille, ein 52ms-Stillstand des Ticks wird also nicht als Buspause
  missdeutet.
  Der Wächter schweigt zu beidem zu Recht, sobald der Mittelwert stimmt - dafür gibt es
  `getTickDeferrals()` und `getTickLastDeferredUs()` daneben.
  `getBusLoad()` hat die Einheit Byte je Sekunde und beschreibt **genau ein Messintervall** von
  `BUS_LOAD_INTERVAL_MS` (1000) - einen gleitenden Mittelwert gibt es bewusst **nicht** (Entscheidung
  des Anwenders): der Wert soll die letzte Sekunde beschreiben und nicht ein Mittel über mehrere. Die
  Konstante ist ein festes `static constexpr`-Member, kein überschreibbares Makro - sie beschreibt eine
  Anzeige, keine Betriebsart. Der Preis ist bekannt und hingenommen: ein einzelnes 263-Oktett-Telegramm
  belegt den Bus 356ms, die Anzeige springt dadurch stärker als der Bus sich ändert. Wer glätten will,
  mittelt über mehrere dieser fertigen Werte, statt das Fenster zu verbreitern.
  **Gerechnet wird beim MESSEN, nicht beim Ablesen**: `sampleBusLoad()` läuft aus `loop()`, hält
  Zählerstände plus Zeitstempel als Bezugspunkt und schließt die Sekunde ab, sobald das Intervall voll
  ist; `getBusLoad()` liefert das fertige Ergebnis, beliebig oft und ohne es zu verändern. Ein
  Bezugspunkt, der bei jedem Lesen mitzöge, ergäbe Spannen von Millisekunden - ein einzelnes Telegramm
  darin sähe wie ein völlig überlasteter Bus aus. Geteilt wird durch die *gemessene* Spanne, nie durch
  die angenommenen 1000ms, ein verspäteter Hauptloop verfälscht also nichts.
  Was hineinfließt, ist bewusst eng gefasst: **nur Telegrammbytes, Polls eingeschlossen**. Steuerbytes
  kommen von der BCU, nicht vom Bus, und gehören zur Auslastung der Host-Leitung; im Resync verworfene
  Bytes lassen sich nichts zuordnen.
  **`getBusLoadPercent()` ist NICHT dieselbe Zahl in anderer Einheit**, und der Unterschied ist der
  eigentliche Punkt: es ist eine **Zeitrechnung**, nicht ein Verhältnis zur Bytekapazität.
  ```
  belegt = Oktetts × BUS_OCTET_TIME_US (1354)
         + Telegramme × (BUS_FRAME_GAP_US (5208) + BUS_ACK_SLOT_US (2708))
  ```
  **Beide Zuschläge sind FESTE SLOTS**, und daran hängt, dass die Rechnung ohne Kenntnis des Verkehrs
  auskommt. Beim Quittungsslot ist das der entscheidende Punkt: er ist reserviert, nicht bedingt - ob
  jemand quittiert oder nicht, die Zeit vergeht und niemand sonst kann senden. Es muss deshalb *nicht*
  bekannt sein, welche Telegramme quittiert wurden, was der Host außerhalb des Busmonitors auch gar
  nicht wissen kann. 15 Bitzeiten Abstand plus das Quittungsoktett (11 Bit) sind 26 Bitzeiten = 2708µs;
  bleibt die Quittung aus, wartet der Sender die dokumentierten 30 Bitzeiten ab (S. 38) - 417µs mehr,
  was keine Fallunterscheidung wert ist.
  Hier stand zwischenzeitlich, die Quittung sei "nicht zählbar" und der Wert unterschätze die Belegung
  deshalb um rund 13%. Das war falsch, und der Fehler war die Frage: gefragt war nicht, *ob* quittiert
  wurde, sondern nur, *wie lang der Slot ist*.
  **UNABHÄNGIG BESTÄTIGT durch die geläufige Kennzahl "der Bus schafft rund 50 Telegramme je Sekunde".**
  Sie fällt aus dieser Rechnung genau heraus - und nur, wenn alle drei Anteile drinstehen:
  ```
  9 × 1354µs (Oktetts) + 5208µs (Pause) + 2708µs (Quittungsslot) = 20102µs  ->  49,7 Telegramme/s
  ```
  Ohne den Quittungsslot kämen 57,5/s heraus, ohne die Pause 67/s - beides passt nicht. Das ist der
  einzige Prüfstein von außen, den diese Rechnung hat, und er trifft. Zugleich ist die Anzeige damit
  kalibriert: 50 minimale Telegramme je Sekunde sind genau 100%, und bei kleinen Telegrammen liest sich
  der Prozentwert direkt als "Telegramme je Sekunde geteilt durch 50".
  Deshalb steht in der Messprobe neben dem Byte- auch der **Telegrammzähler** (heile plus kaputte - der
  Bus war in beiden Fällen belegt). Aus den Oktetts allein ginge es nicht: 270 Oktetts sind ein großes
  Telegramm oder dreißig kleine, und die dreißig belegen den Bus 156ms länger, weil vor jedem Telegramm
  50 Bitzeiten frei sein müssen.
  Das ist auch der Grund, warum der Nenner **nicht** die rohe Kanalkapazität ist (738 Oktetts/s). Gegen
  die gerechnet wäre 100% unerreichbar, und zwar unterschiedlich weit: ~61% bei 9-Oktett-Telegrammen,
  ~98% bei maximalen - eine Zahl, deren Obergrenze vom Verkehr abhängt, taugt nicht als Füllstand. Über
  die Zeit gerechnet heißt 100% dagegen wirklich "hier passt kein Telegramm mehr hinein".
  **Abgeschnittene Telegramme gehen vollwertig ein**, und das ist kein Zufall: gezählt werden die Bytes,
  die tatsächlich ankamen (`length` am Sequenzende, nicht die Sollgröße), und da ein Fragment als
  `INVALID` gemeldet wird, zählt es auch bei den Telegrammen mit. Ein nach drei Oktetts abgebrochenes
  Telegramm bekommt also 3 × 1354µs plus einmal die Pause - der Bus war für beides belegt.
  **Bytes aus einem Resync fehlen dagegen**, sie laufen nach `incrementRxDroppedBytes()` und tauchen in
  `getRxFrameBytes()` nie auf. Bewusst nicht nachgerüstet (Entscheidung des Anwenders): ein Resync ist so
  selten, dass es die Zahl nicht bewegt, und wenn er häufig wird, zeigt `getRxResyncs()` das deutlicher
  an. **Den Dropped-Zähler dafür einfach zu addieren wäre falsch** - er fasst drei Quellen zusammen, und
  eine davon ist bereits gezählt: ein fertiges Telegramm ohne Platz im RX-Ring erhöht *beide* Zähler.
  Wer den Resync-Anteil doch will, braucht dafür einen eigenen Zähler.
  **Über 100 wird bewusst NICHT gedeckelt** (Entscheidung des Anwenders), und der Rückgabetyp ist deshalb
  16 Bit - ein `uint8_t` liefe bei 256% still über und meldete 0. Der Bus kann nicht voller als voll sein,
  ein Wert über 100 ist also ein Befund und soll sichtbar sein statt weggerundet.
  `test_bus_load_percent_is_not_capped` hält das fest, damit es niemand als offensichtlichen Fehler
  "repariert".
  **Damit das trägt, werden die Telegrammbytes JE BYTE gezählt** (`processFrameByte()` /
  `processPollByte()` / `processControlByte()`), nicht mehr am Sequenzende in einem Rutsch. Der alte Weg
  hatte einen echten Zuordnungsfehler: ein laufendes Telegramm trug 0 bei und brachte beim Abschluss seine
  gesamte Busbelegung mit, auch den Teil, der vor dem Fenster lag - bei einem maximalen Telegramm 356ms,
  also über ein Drittel der Messsekunde. Das hätte ein "über 100" erzeugt, das nichts bedeutet, und damit
  genau die Aussage zerstört, für die der Deckel weggelassen wurde. Die Kategorie steht dabei in jedem Pfad
  schon mit Byte 0 fest, und der Zähler hat weiterhin genau einen Schreiber (den Tick) - es braucht also
  weder Klassifikation im Nachhinein noch Atomarität. Achte auf die Reihenfolge in
  `processControlByte()`: der Zähler steht **hinter** dem Poll-Zweig, weil der sein Byte selbst als
  Telegrammbyte zählt.
  Als Rest bleibt der Zuschlag **je Telegramm**, der weiterhin erst beim Abschluss fällig wird - vorher
  steht nicht fest, ob das Telegramm heil ist. Für ein Telegramm an der Fenstergrenze sind das 7916µs,
  bei 1s also 0,8%.
  Früher legte der Leser selbst das Fenster fest: das Intervall war "Zeit seit dem letzten Aufruf", was
  von der Konsole kommt und damit unbegrenzt ist - die erste Anzeige nach Stunden Laufzeit mittelte
  über die gesamte Laufzeit, und der Zwischenwert (Bytes × 1000) lief über 32 Bit über. Eine Messung
  nach einer Lücke von mehr als zehn Intervallen wird verworfen statt benutzt, da sie nichts über die
  aktuelle Last aussagt.
- **`SystemState`** (`SystemState.{h,cpp}`, erreichbar über `getSystemState()`) hält das Byte hinter
  einem `U_SystemStat.ind`: die Regler-/Oszillatorflags und die Betriebsart des Chips. Es füllt sich
  nur als Antwort auf `requestState()`, `isValid()` beginnt deshalb falsch - was *nicht* dasselbe ist
  wie "Power-UP, alles aus", die Lesart, die ein nacktes Nullbyte sonst bekäme. Kein `volatile`:
  anders als `Statistics` wird es aus `loop()` geschrieben, nicht aus `tick()`.
  Achte auf `SYSTEM_STATE_MODE_NORMAL`: `mode()` maskiert mit `0x03`, die Konstante muss dazu
  passen. Steht dort ein Wert mit gesetzten oberen Bits, kann `normalMode()` nie wahr werden -
  ein Fehler, der lange unbemerkt bleibt, weil `modeString()` direkt gegen `0x03` vergleicht und
  weiterhin das Richtige anzeigt.
- **Ein Telegramm senden** (`pushTransmitQueue()`, drei Überladungen): `const Frame &` ist die
  Umsetzung, `(data, length)` wickelt ein, `Frame *` ist KOMPAT und die einzige, die den **Besitz**
  übernimmt. Ein laufender Versand ist kein Ablehnungsgrund.
  **Alle drei nehmen ein VOLLSTÄNDIGES Telegramm einschließlich Prüfsumme** - es gibt hier keine zwei
  Konventionen mehr. Früher erwartete `sendFrame()` es *ohne* und die KOMPAT-Form kürzte deshalb die
  Länge um eins; dieselben Bytes hatten zwei Bedeutungen. **Die Prüfsumme wird geprüft, nicht neu
  gerechnet**: sie gehört zum Telegramm, und sie stillschweigend zu überschreiben verdeckte einen
  Fehler im Aufrufer - das Telegramm ginge dann mit korrekter CRC über falschem Inhalt hinaus.
  Geprüft wird über `Frame::isValid()`, und zwar **vor jeder Veränderung am Puffer**; ein abgelehntes
  Telegramm hat ihn also nie angefasst, was jedes Rückabwickeln erübrigt.
- **Die Warteschlange ist ein linearer Bytepuffer** (`TransmitQueue`, `TPUART_TX_BUFFER_SIZE`,
  Vorgabe 2048) und **gehört dem Hauptkontext allein**. Das ist die Bedingung, unter der darin nach
  Priorität umsortiert werden darf: der Tick fasst sie nicht an.
  **Die Library hat damit keinen Heap mehr im Datenpfad.** Vorher lag jedes wartende Telegramm in
  einem eigenen `malloc`-Block variabler Größe - die einzige fragmentierende Allokation, im Bustakt
  über die Lebensdauer des Geräts.
  **Bytes statt Telegrammzahl**, und das ist der Grund: ein gewöhnliches Gruppentelegramm ist 9-15
  Oktetts, ein maximales 263. Feste Plätze verschenkten rund 95%; dieselben 2048 Byte fassen ~140
  gewöhnliche Telegramme, als feste Plätze wären es 7.
  **Wie viel man braucht, ergibt die Anwendung, nicht der Bus**: der knx-Stack sendet je `loop()`
  genau *ein* Telegramm (`sendNextGroupTelegram()`), der Loop läuft aber um ein Vielfaches schneller,
  als der Bus abfließt (~50 Telegramme/s). 100 gleichzeitig geänderte KOs sind nach ~200ms alle
  eingereiht, während der Bus ~10 geschafft hat - es warten also ~90. Zusammengefasst wird bereits im
  KO selbst (mehrfaches Schreiben vor dem Senden ergibt *ein* Telegramm mit dem letzten Wert), die
  Obergrenze ist also die Zahl gleichzeitig sendebereiter KOs.
  **Über uns puffert nichts**: lehnt die Queue ab, verwirft der knx-Stack das Telegramm und meldet ein
  negatives `L_Data.con` nach oben. Jede Ablehnung ist unmittelbar ein verlorenes Telegramm.
  **Kein Längenpräfix** - ein Telegramm beschreibt seine eigene Länge (`Frame::sizeOf()`), und hier
  kommt nur Geprüftes hinein. **Der Empfangsring macht es anders und zu Recht**: der bewahrt auch
  `INVALID`-Einträge auf, und bei genau denen ist die Selbstbeschreibung das, was man nicht glauben
  darf.
- **Prioritäten: System > Urgent > Normal > Low**, strikt, ohne Aging - genau wie der Bus selbst das
  Medium vergibt. Innerhalb einer Klasse gilt Eingangsreihenfolge. Die Rohwerte des Steuerbytes
  (Bits 3-2) sind dabei **nicht** sortiert: `0=System, 1=Normal, 2=Urgent, 3=Low`, Normal und Urgent
  tauschen also die Plätze. Verifiziert gegen den Code, der die Bits schreibt (`knx_types.h`,
  `CemiFrame::priority()` mit Maske `0x0C`) - die Datenblätter beschreiben die UART-Strecke, nicht die
  Rahmensemantik.
  **Die Reserve ist die tragende Hälfte, nicht die Kür.** Sortieren hilft nur Telegrammen, die es in
  die Queue *geschafft* haben; ein Low-Ansturm füllt sie sonst, und das System-Telegramm der
  ETS-Verbindung wird an der Tür abgewiesen. `TPUART_TX_PRIORITY_RESERVE` (= `TPUART_BUFFER_SIZE`,
  263) ist deshalb abgeleitet, nicht geraten: ein maximales Telegramm oberhalb von Low passt immer.
- **Der Tick bekommt eine Vorlage gereicht**, er holt sich nichts. `_stagedData`/`_stagedLength` plus
  zwei monotone Zähler, je ein Schreiber: `_stagedSeq` (Hauptkontext), `_takenSeq` (Tick), Nutzlast
  vor Zähler veröffentlicht. Drei Regeln tragen das: die Vorlage nur schreiben, wenn beide gleich
  sind; den Platz erst freigeben, wenn `_takenSeq` nachgezogen hat; und der Tick kopiert in `_buffer`,
  weshalb die Bytes danach weg dürfen - auch `restart()` nach einem Reset arbeitet aus `_buffer`.
  **Der Sendestart hängt damit nicht am Hauptloop**: nach der Übernahme hat er die gesamte Dauer der
  laufenden Übertragung Zeit nachzulegen (20ms und mehr), ein 53ms-Flash-Stillstand ist also gedeckt.
  Der Preis ist eine **auf genau ein Telegramm begrenzte Inversion** - eine bereits vorgelegte
  Sendung lässt sich nicht zurückholen. Schlimmster Fall: ein maximales in Übertragung plus ein
  maximales vorgelegt, also ~712ms, gegen eine T_Ack-Frist von rund 3 Sekunden.
  **Das löst zugleich zwei alte Wettläufe auf**: `abort()` verwirft nur noch die Vorlage, den Puffer
  räumt der Hauptkontext in `stageNextTelegram()` - und weil Einstellen und Räumen nun im selben
  Kontext liegen, schließt sich auch das Fenster, das hier als "ohne Sperre nicht zu schließen"
  dokumentiert war.
  Jedes Oktett geht mit seinem eigenen Positionsbyte hinaus (`U_L_DataStart.req` /
  `U_L_DataCont.req`, beide `0x80 | position`, keine Fallunterscheidung nötig), das letzte - die
  Prüfsumme - mit `U_L_DataEnd.req`, was die Übertragung auf dem Bus überhaupt erst startet. Das
  Positionsfeld hat nur 6 Bit, ein erweitertes Telegramm braucht deshalb `U_L_DataOffset.req` für die
  oberen 3; der Chip behält diesen Offset, bis er geändert wird, er wird also nur bei Änderung erneut
  gesendet. **Diesen Service gibt es nur beim NCN** - die Siemens-Tabelle
  (`docs/Siemens_TPUART.pdf` S. 21) geht von `U_L_DataContinue` (Index 1..62) direkt zu
  `U_L_DataEnd` (Länge 7..63) und vergibt den Opcode `0x08` gar nicht, ihn dort abzusetzen ergäbe
  also ein unbekanntes Steuerbyte. Ein TPUART2 braucht ihn auch nie: er kann nicht mehr als 64
  Oktetts senden, der Index bleibt also unter 64. **Offset 0 wird zu Beginn jedes Telegramms
  ausdrücklich geschrieben**, und das ist kein überflüssiges Byte: der Offset lebt im Register des
  Chips. Ihn nur in einer eigenen Variablen zu führen und die je Telegramm auf 0 zu setzen, ohne das
  Register anzufassen, setzte ein kleines Telegramm nach einem großen am stehengebliebenen Offset ab.
  Ein Oktett je
  `tick()`, weshalb `TPUART_TX_ATOMIC_BYTES` gleich 3 ist (Offset + Position + Daten).
  **Die Busmonitor-Quittung und unser eigenes `L_Data.con` sind derselbe Vorgang**, sie teilen sich
  deshalb einen Zustand (`RxState::FrameAck`) und eine Frist (`TPUART_FRAME_ACK_US`). Beide stammen
  von derselben Quittung auf dem Bus und treffen deshalb gleich spät ein - ~3,3ms nach dem
  Telegrammende. Ein fertiges Telegramm geht in `FrameAck`, wenn `_busMonitor` gesetzt ist *oder* wenn
  es unser eigenes Echo ist (`isOwnEcho()`). **Die Bestätigung setzt die Quittungsflags**, mit
  derselben Bedeutung, die sie überall sonst haben: `ACK` = es gibt eine Quittung (überhaupt eine
  Bestätigung), `ACK_NACK` kommt dazu, wenn sie negativ war - genau die Kombination, die
  `acknowledgeFlags()` für `AckType::Nack` erzeugt. Dazu `DATA_CON` für "eine Bestätigung ist
  eingetroffen", was eine negative Bestätigung von gar keiner unterscheidet (Frist abgelaufen). Ein
  Telegramm ohne Bestätigung trägt deshalb weder `A` noch `N` noch `B`.
  `BUSY` wird aus einer Bestätigung nie gesetzt, und das ist keine Auslassung: die Bestätigung ist
  **ein Bit** (NCN5130 Tabelle 13, Siemens S. 31 - beide ausdrücklich). Der Chip hat seine eigenen
  Wiederholungen gefahren (bis zu 3 nach NACK, bis zu 3 nach BUSY) und meldet nur das Ergebnis, `N`
  heißt hier also "nicht positiv quittiert" und deckt ein NACK ebenso ab wie ein erschöpftes BUSY oder
  schlichtes Schweigen. Die echte Unterscheidung gibt es allein im Busmonitor, wo das rohe
  Quittungsbyte durchkommt.
  Die Reihenfolge zählt dort: `completeSequence()` muss *vor* der Freigabe von `_txState` laufen, weil
  `isOwnEcho()` den Sendeweg noch belegt braucht, um `TX` zu setzen.
  Was in `FrameAck` außer der erwarteten Antwort eintrifft, wird **neu verarbeitet, nicht verworfen**:
  bei Wiederholungen spiegelt der Chip das Telegramm erneut, das nächste Byte ist also meist der
  Anfang dieses Wiederholungsechos. Das alte Verhalten (melden, dann Resync) warf das gesamte
  Wiederholungsecho weg. Ein `U_FrameState.ind` wird geschluckt - im 8-Bit-UART-Modus geht es dem
  `L_Data.con` voraus (NCN5130 S. 42).
  **`TPUART_TX_CONFIRM_TIMEOUT_MS` (10s) ist ein Wachhund, keine Protokollfrist.** Kommt überhaupt
  keine Bestätigung, ist unbekannt, was im Sendepuffer des Chips liegt, und der einzige Weg in einen
  definierten Zustand ist ein Reset - also sendet er `U_Reset.req`. Das Wiederaufnehmen hängt
  **nicht** an dieser Anforderung, sondern am `U_Reset.ind`, und bewusst **ohne zu fragen, wer den
  Reset verursacht hat**: jeder Reset leert den Sendepuffer des Chips, danach beginnt also alles noch
  Offene einfach von vorn (und `_busMonitor` wird gelöscht, auch bei einem Reset, den wir nicht
  gesendet haben). Bleibt auch die Indikation aus, läuft dieselbe Frist erneut ab und der Reset wird
  wiederholt - gegen eine stumme BCU gibt es nichts Besseres, als es weiter zu versuchen.
  **Gemessen wird fehlender FORTSCHRITT, nicht fehlender Abschluss, jedes Echo macht sie also neu
  scharf** (`echoReceived()`). Das Echo beweist, dass der Chip das Telegramm gerade auf den Bus legt -
  er spiegelt jedes gesendete Oktett zurück, bei jeder Wiederholung ebenfalls -, solange also Echos
  kommen, *kann* die Bestätigung noch nicht da sein, und ein Abbruch schnitte in eine gesunde
  Übertragung. Der Unterschied zeigt sich auf einem belegten Bus: der Chip muss vor jeder Wiederholung
  auf eine freie Leitung warten, und eine feste Frist ab dem `U_L_DataEnd.req` schlüge mitten in der
  Übertragung zu und verwürfe ein Telegramm, das unterwegs war. Zu messen ist der Abstand zwischen
  Lebenszeichen, nicht die Gesamtdauer. 10s sind dafür großzügig: ein maximal großes Telegramm belegt
  den Bus 356ms plus das Warten auf eine freie Leitung.
  Die Frist wird auch nach dem Zuschlagen des Wachhunds neu scharf gemacht, eine stumme BCU bekommt
  also alle 10s einen Resetversuch statt einem einzigen und nie wieder.
  **Der Chip spiegelt jedes gesendete Oktett zum Host zurück** (Datenblatt S. 42), unser eigenes
  Telegramm trifft also als normales Telegramm ein. `isEcho()` erkennt es am Vergleich mit dem
  Sendepuffer - alles außer dem Steuerbyte (dessen Wiederholungsbit die BCU bei einer Wiederholung
  löscht) und der davon abhängigen Prüfsumme. Zwei Dinge hängen daran: `completeSequence()` markiert
  das Telegramm mit `TP_FRAME_FLAG_TX`, und der Wachhund des Transmitters wird neu scharf gemacht
  (siehe oben).
  **`sendAcknowledge()` steigt beim eigenen Echo sofort aus** (`Transmitter::isEchoPrefix()`, bei
  Byte 6) - **man quittiert nicht sich selbst.**
  Hier stand einmal das Gegenteil, mit der Begründung, ein Gerät sende nicht an sich selbst, der
  Quittungs-Callback antworte für das eigene Ziel also "nicht meins" und die Kette ende von allein; der
  Fall, in dem er "meins" sagt, galt als exotisch. **Am echten Gerät widerlegt**: ein IP-Router leitet
  auf Gruppenadressen weiter, die in seiner eigenen Filtertabelle stehen, und sagt für sein Echo
  deshalb "meins". Gemessen über 15,7 Stunden: **25196 gesendete Quittungen, exakt 4 je eigenem
  Telegramm** (Original plus die drei Wiederholungen des Chips) und keine einzige für ein fremdes
  Telegramm. Das ist der Normalfall des Hauptverbrauchers dieser Library, nicht ein Sonderfall.
  Auf dem Bus hatte das Byte nie Wirkung (der Chip verwirft ein `U_Ackn.req`, das während seines eigenen
  Sendens eintrifft) - gratis war es trotzdem nicht: es belegt einen der vier Plätze in
  `TPUART_TX_INTERFACE_BUFFER`, genau in dem Moment, in dem der Sendepfad drei davon für das nächste
  Oktett braucht, und es machte `getTxAcknowledges()` als Diagnose wertlos.
  Verloren geht dadurch nichts: das `ACK` am Echo kommt aus dem `L_Data.con` über `RxState::FrameAck`,
  nicht aus unserer eigenen Quittung.
  **Geprüft wird der ANFANG gegen das laufende Telegramm, nicht "läuft gerade eine Übertragung"** - die
  naheliegende Variante wäre aktiv schädlich: `TxState::Await` dauert bis zur Bestätigung, und in dieser
  ganzen Zeit würde kein **fremdes** Telegramm mehr quittiert. Dafür steht
  `test_foreign_frame_is_acknowledged_while_awaiting_confirmation`, und er ist der einzige Fall, der
  diese Verwechslung fängt (mutationsgeprüft). Dass ein 6-Byte-Anfang genügt, liegt an der Quelladresse
  darin: ein fremdes Telegramm mit *unserer* Quelladresse wäre eine doppelt vergebene physikalische
  Adresse, also ein Anlagenfehler.
- **`RepetitionFilter`** (`RepetitionFilter.{h,cpp}`) markiert ein Telegramm, das der Sender ein
  zweites Mal auf den Bus gelegt hat, weil er keine Quittung sah. Der Verbraucher bekommt es weiterhin,
  mit `FILTERED` markiert, und kann die Verarbeitung überspringen, statt zweimal zu handeln. Geprüft
  und gemerkt wird für **jedes gültige Telegramm**, nicht nur für wiederholte - ohne den Wert des
  Originals wäre die Wiederholung nicht erkennbar; markiert wird nur, wenn zusätzlich `isRepeated()`
  gilt.
  **Das eigene Echo läuft mit, und `getRxRepeatedFrames()` zählt es mit** - ausdrücklich so (Entscheidung
  des Anwenders): das Echo kommt real über den Empfangspfad herein, das `Rx` im Namen ist also richtig und
  gilt für beide Richtungen, weil das gesendete Telegramm zum empfangenen wird. Der Chip löscht bei einer
  Wiederholung das Wiederholungsbit und spiegelt jede Wiederholung genauso zurück wie den ersten Versuch,
  `isRepeated()` greift also.
  **Bei der Deutung wissen**: quittiert auf der Linie niemand unsere Telegramme, wiederholt der Chip jedes
  bis zu dreimal, und der Zähler füllt sich mit unserem eigenen Versand. An einem Router gemessen: 18897
  von 30861 Telegrammen als wiederholt gemeldet, also 61% - und davon **alle** eigene Echos (exakt 3 je
  eigenem Telegramm), fremde Wiederholungen 0. Das liest sich wie ein schwer gestörter Bus und ist keiner.
  Getrennt zu zählen wurde erwogen und verworfen; wer es doch braucht, hat an der Zählstelle das Flag
  `TP_FRAME_FLAG_TX` zur Hand.
  Gespeichert wird je Sender ein 16-Bit-Fingerabdruck (CRC-16/SPI-FUJITSU, mit erzwungen gesetztem
  Wiederholungsbit, damit Original und Wiederholung gleich hashen), nicht das
  Telegramm: 50 maximal große Telegramme wären 13KB, das hier sind 400 Byte. Der Preis ist eine
  theoretische Kollision, die 16 Bit vertretbar machen.
  **Einträge verfallen nie, und das ist Absicht.** Ein Eintrag je Quelle; ein Eintrag verschwindet nur
  dadurch, dass er verdrängt wird, wenn eine *neue* Quelle auftaucht und alle
  `TPUART_REPETITION_FILTER_COUNT` (50) Plätze belegt sind - die am längsten nicht gesehene Quelle
  fällt heraus. Eine LRU-Liste also, aber ohne `std::list` + `unordered_map`: ein festes Array mit
  linearer Suche über 50 Einträge, was im Hauptkontext nichts kostet.
  **Baue keine Zeitgrenze ein.** Sie wurde probiert (2s) und entfernt. Es gibt keine Schranke
  abzuleiten: wann die Wiederholung eintrifft, hängt davon ab, wann der Sender den Bus das nächste Mal
  frei sieht, und das können Millisekunden oder zehn Sekunden sein. Eine zu kurze Grenze lässt die
  Wiederholung als vermeintliches Original durch - genau das, was der Filter verhindern soll.
  Der eine Fall, in dem der Filter etwas kostet, wird hingenommen: verpassen wir ein *Original*, trifft
  seine Wiederholung ein, und gehört der gespeicherte Fingerabdruck noch zu einem früheren,
  byteidentischen Telegramm derselben Quelle, dann ist die einzige Kopie, die wir bekommen, als
  Wiederholung markiert. Diese beiden Fälle sind von außen nicht unterscheidbar - zu welchem von zwei
  identischen Telegrammen eine Wiederholung gehört, ist schlicht nicht wissbar -, und danach mit einem
  Zeitgeber zu raten ist schlechter, als den Verlust hinzunehmen.
- **`setOwnAddress()` / `setRepetitions()`** halten Konfiguration, die die BCU bei jedem Reset
  vergisst ("After reset the address evaluation is deactivated again", Siemens S. 23), sie wird danach
  also erneut gesendet. Auslöser ist eine **Konfigurationsepoche**: der Tick zählt sie bei jedem Reset
  hoch, der Hauptkontext zieht nach und ruft `applyConfiguration()` - ein Zähler statt eines Flags,
  damit jede Seite genau einen Schreiber hat. Die Epoche wird nur nachgezogen, wenn wirklich alles in
  die Steuerwarteschlange gekommen ist; `applyConfiguration()` liefert sonst falsch und der nächste
  `loop()` versucht es erneut, denn diese Konfiguration stillschweigend zu verlieren hieße, die
  Quittung für alles an uns Adressierte stillschweigend zu verlieren. Die beiden Setter benutzen
  denselben Weg über `markConfigurationPending()` (die angewandte Epoche um eins zurückdrehen - der
  Hauptkontext ist ihr einziger Schreiber, und der Wert kann nie mit `_configEpoch` kollidieren).
  Gesendet werden nur Abweichungen: keine Adresse heißt kein Service, und die Wiederholungszähler
  gehen nur hinaus, wenn sie von der Vorgabe nach dem Reset (3/3) abweichen.
  Beide Services gibt es auf beiden Chips, sie unterscheiden sich aber in **Opcode, Sequenzlänge und
  Bitbelegung**: `0xF1`+Adr+Adr+Dummy gegen `0x28`+Adr+Adr, und `0xF2`+Zähler+2 Dummies gegen
  `0x24`+Zähler - mit BUSY in den Bits 6-4 (NCN, Bild 37) gegenüber den Bits 7-5 (TPUART2, Bild 20).
  Achtung bei `0x28`: auf einem TPUART2 ist das `U_SetAddress`, auf einem NCN `U_IntRegWr.req` -
  dieselbe Zahl, völlig anderer Dienst. Die Zähler werden als zwei getrennte Werte gehalten und je Chip
  zusammengesetzt, es gibt also kein Zwischenformat, das in das eine oder andere Layout umgerechnet
  werden müsste - genau dort verwechselt man `||` mit `|` und bekommt immer nur 0 oder 1 heraus.
  **Eine Adresse zu setzen aktiviert die Auto-Quittung des Chips**, und beide Datenblätter sagen das
  ausdrücklich - "Sets the physical address of the device and activates the auto-acknowledge function"
  (NCN5130 S. 36), "If the address is set a complete address evaluation in the TP-UART is activated"
  (Siemens S. 23) -, **`applyConfiguration()` setzt `_autoAcknowledge` deshalb selbst**, wenn es die
  Adresse einreiht. Auf die Bestätigung des Chips zu warten funktionierte nicht: `U_Configure.ind`
  **gibt es nur beim NCN**. Die hostgerichteten Dienste des TPUART2 sind Reset-, ProductID-,
  State-Indication und `L_Data.confirm` (Bild 25), mehr nicht. Auf einem NCN trifft die Indikation
  weiterhin ein und behält weiterhin recht, als eigene Meldung des Chips über seinen Zustand. Damit hat
  `_autoAcknowledge` zwei Schreiber (der Hauptkontext setzt, der Tick löscht bei einem Reset und
  korrigiert aus der Indikation), was hinnehmbar ist, weil die Epoche jede Uneinigkeit binnen eines
  `loop()` auflöst.
  **Es ist reine Information und steuert nichts.** Eine frühere Fassung dieser Datei behauptete das
  Gegenteil - dass `sendAcknowledge()` bei aktiver Auto-Quittung schweigen dürfe, da der Chip schneller
  ist und unser `U_Ackn.req` nur einen aktiven BUSY-Modus abbräche ("BUSY mode is deactivated
  immediately if the host controller confirms a frame by sending U_Ackn.req", S. 35). Das war falsch,
  und es hat echte Telegramme gekostet: **die Auto-Quittung des Chips ist ein Rückfall für den Fall,
  dass der Host nicht rechtzeitig antwortet, kein Ersatz für dessen Antwort**, und sie deckt nur die
  eigene physikalische Adresse des Chips ab - keine Gruppenadressen, und auf einem Koppler nicht die
  fremden physikalischen Adressen, die er weiterleitet. Im IP-Router war die Wirkung sofort sichtbar:
  Telegramme kamen mit `ADDRESSED`, aber ohne `ACK`, und der Sender wiederholte sie dreimal, weil
  niemand auf dem Bus quittiert hatte. Quittiert wird hier deshalb bedingungslos. Die Nebenwirkung auf
  den BUSY-Modus ist der hingenommene Preis - BUSY-Modus benutzt nichts.
  **Was ein Reset sonst noch löscht, und was wiederhergestellt wird.** Die Epoche deckt Adresse,
  Wiederholungszähler und - da es sich als zugehörig herausstellte - die **Spannungsregler (ACR0)** ab:
  `Tabelle 16` gibt als Resetwert `0111 0100` an, also alles an, und der RESET-Zustand wird "entered
  after Power On Reset (POR) **or in response to a U_Reset.req**... NCN5130 gets initialized" (S. 30).
  Ein `powerControl(false)` wäre also von jedem Reset stillschweigend rückgängig gemacht worden -
  einschließlich der Resets, die diese Library selbst sendet (der Wachhund des Transmitters). Gesendet
  wird es nur, wenn der Aufrufer es je angefasst hat, und erneutes Senden ist idempotent, die Annahme
  ist also in beide Richtungen harmlos.
  Bewusst **nicht** wiederhergestellt: Busmonitor (ein Reset ist der dokumentierte Weg *heraus*, ihn
  wiederherzustellen machte `reset()` nutzlos), Busy-Modus (von Natur aus flüchtig), Stop-Modus (vom
  Reset beendet). **`U_Configure.req` wird überhaupt nie gesendet** - erweiterte CRC, Auto-Polling und
  der Marker sind allesamt ungenutzt, und die Vorgabe nach einem Reset ist genau das, was der Parser
  erwartet. Wird eines davon je gewollt, ist `applyConfiguration()` die Stelle, und die Epoche deckt es
  automatisch ab.
  **Ein verpasstes `U_Reset.ind` ist ebenfalls abgedeckt**, über ein Signal, das nichts zusätzlich
  kostet: der NCN durchläuft POWER-UP und SYNC nur nach einem Reset ("entered after Reset State",
  S. 30), und der Zustand wird ohnehin sekündlich abgefragt, `checkChipRestart()` markiert die
  Konfiguration also als anstehend, wenn der Chip nach NORMAL zurückkehrt. Das ist der Grund, dass die
  Konfiguration *nicht* periodisch erneut gesendet wird - erkennen ist billiger als wiederholen. STOP
  zählt ebenfalls als "nicht normal"; das kostet je Stop-Zyklus ein überflüssiges erneutes Senden, das
  dieselben Werte schreibt.
  **Die Auto-Quittung lässt sich nicht abschalten** ("Autoacknowledge can only be deactivated by a
  Reset Service", S. 38; auf der Siemens-Seite ebenso, wo sie nach einem Reset weg ist).
  `setOwnAddress(0)` sendet deshalb nichts und der Chip quittiert weiter - `_autoAcknowledge` bleibt
  bewusst gesetzt, da es den Chip beschreibt und nicht unseren Wunsch. Wer sie tatsächlich loswerden
  will, muss `setOwnAddress(0)` ein `reset()` folgen lassen.
- **Verbindungsüberwachung** (`processConnectionState()`, aus `loop()`): der Zustand wird alle
  `TPUART_STATE_INTERVAL_MS` (1s) abgefragt, und die Verbindung gilt nach
  `TPUART_CONNECTION_TIMEOUT_MS` (5s) ohne ein einziges empfangenes Byte als verloren.
  **Erst die Abfrage macht den Unterschied überhaupt erkennbar**: auf einem ruhigen Bus
  kommt minutenlang nichts, Stille allein sagt also nichts. Eine Abfrage muss beantwortet werden
  (`U_State.ind`), und diese Antwort zählt als Lebenszeichen wie jedes andere Byte - ein belebter Bus
  braucht also keinen zusätzlichen Verkehr, um bestätigt zu bleiben.
  Bei Verlust läuft der Verbindungsaufbau erneut, aber **nur mit der bereits gefundenen Baudrate** -
  `_detectCandidateIndex` bleibt unangetastet, und `rejectDetectCandidate()` schaltet nicht mehr
  weiter, sobald `_everConnected` gesetzt ist. Die Baudrate ist eine Hardwareeigenschaft der BCU (der
  BDS-Pin bei einem TP-UART 2+) und kann sich im Betrieb nicht ändern, erneutes Abtasten hieße also
  nur, für nichts mit falschen Raten an die BCU zu schreiben; eine andere zu finden setzt einen
  Neustart voraus. Der Wiederholungsversuch selbst ist geduldig - die Busspannung kann eine Weile weg
  sein -, und `_connectReported` wird gelöscht, damit der Reconnect erneut angekündigt wird.
  Die Reihenfolge zählt: die Detect-Felder werden zuerst geschrieben und `_connected` **zuletzt**, der
  Tick sieht sie also entweder gar nicht oder vollständig vorbereitet.
  `bcuState()` ist **abgeleitet, nicht gespeichert** (`Uninitialized`/`Connected`/`Disconnected`) -
  es zu speichern bräuchte einen zweiten Schreiber für ein Feld, das beide Kontexte anfassen. Ein
  anstehendes Telegramm überlebt: das `U_Reset.ind`, das den Reconnect bestätigt, startet es von vorn,
  dieselbe Regel wie bei jedem anderen Reset.
- **Meldungen verlassen die Schicht über `registerMessage()`**, samt dem Flag `bool error`. Das ist bewusst *kein*
  Steuerbyte-Callback: Steuerbytes bleiben in der Schicht - sie zu deuten ist ihre Aufgabe -, und was
  der Aufrufer bekommt, ist die fertige Meldung.
  Jede empfangene Steuersequenz wird namentlich protokolliert (`controlServiceName()` in `Types.h`).
  **Nur Überraschungen erreichen die Konsole.** Fünf Dienste werden unterdrückt, weil sie lediglich
  etwas beantworten, das wir selbst getan haben: `U_State.ind` und `U_SystemStat.ind` (die Antwort auf
  die sekündliche Abfrage - sie zu protokollieren füllte die Konsole zweimal je Sekunde mit
  "unverändert"), `U_Configure.ind` (bestätigt die Auto-Quittung, die unsere Adresse eingeschaltet
  hat), `U_Reset.ind` / `U_StopMode.ind` (Antworten auf unser eigenes `U_Reset.req` / `stopMode()`) und
  `L_Data.con`. Letzteres erreicht `handleControlEntry()` nur, wenn es *neben* unserem Telegramm
  eintraf statt dahinter: mit erkanntem Echo wartet der Empfänger in `RxState::FrameAck` darauf und
  hängt es als Flag an. Ohne erkanntes Echo landet es hier - was etwas über den Empfangspfad aussagt,
  nicht über die Bestätigung, und was bereits am fehlenden `A` des gemeldeten Telegramms und an
  `getRxInvalidFrames()` ablesbar ist. Als Konsolenzeile war es nur Lärm. Gemeldet wird stattdessen ihr
  *Inhalt*, und nur, wenn er etwas zu sagen hat: aufgelaufene Fehlerbits über `showStateErrors()`, der
  Systemzustand über `showSystemState()` bei Änderung, der Zustand der Auto-Quittung über
  `isAutoAcknowledge()`.
  **Der eine Reset, der etwas bedeutet, meldet sich selbst, mit Grund**: der Wachhund des Transmitters
  rastet `confirmTimeout()`, und `loop()` macht daraus "No L_Data.con for 10000 ms - BCU reset". Ein
  Reset, den die *BCU* auslöst, bleibt ebenfalls sichtbar - über den Verbindungsverlust oder, auf einem
  NCN, über die Statuszeile, die POWER-UP/SYNC durchläuft (siehe `checkChipRestart()`).
  Die Ausgabe zu unterdrücken ändert nichts an der Auswertung: Reset und Configure werden im *Tick*
  verbraucht (`resetIndication()` / `configureIndication()`), lange bevor der Eintrag hier ankommt. Was
  druckbar bleibt, ist genau das Unerwartete: ein `L_Data.con` oder `L_Ackn.ind` außerhalb des
  erwarteten Ablaufs, ein `U_FrameEnd.ind` (Markermodus, den wir nie einschalten), ein
  `U_FrameState.ind` außerhalb einer Übertragung - und unbekannte Bytes. Ein `U_SystemStat.ind`, dessen
  Länge nicht 2 ist, fällt ebenfalls durch; eine halbe Sequenz ist sehenswert.
  Ein Wert, der zu keinem Dienst aus Tabelle 13 passt, wird als **Fehler** protokolliert, denn die BCU
  kann ein solches Byte nicht senden - es heißt, dass der Bytestrom an dieser Stelle falsch gelesen
  wird. Daneben endet `loop()` mit `showSystemState()` (nur bei Änderung) und `showStateErrors()`.
  Die Fehlerbits aus `U_State.ind` werden ODER-akkumuliert, nicht überschrieben:
  der Chip meldet jedes Ereignis einmal, ein späteres sauberes `U_State.ind` löschte sonst einen
  Fehler, den es gab.
  `printMessage()` formatiert in einen 128-Byte-Stackpuffer (der längste Text ist die ~60 Zeichen
  lange Systemzustandszeile). Nur im Hauptkontext.
- **Die Puffergröße ist 263, und die zugehörige Größenprüfung trägt.** Das größte *gültige* Telegramm
  ist erweitert mit maximaler APDU: 9 + 254 = 263. 254, nicht 255 - der Wert 255 ist im Längenoktett
  eines erweiterten Telegramms reserviert, eine APDU wird also nie länger als 254. Damit ist
  `_frameSize > 263` erreichbar (ein Längenoktett von 255 ergibt 264), und das ist genau der Fehlerfall
  "unplausible Größe". Lösche diese Prüfung nicht als unerreichbar - unerreichbar ist sie nur, wenn der
  Puffer auf 264 überdimensioniert wird. Ein `static_assert` in `Receiver.cpp` nagelt `>= 9 + 254`
  fest, den Puffer zu verkleinern lässt also den Bau scheitern, statt still überzulaufen; ein zweites
  sichert zu, dass der Ringpuffer mindestens einen maximal großen Eintrag fasst.

Ausdrücklich zurückgestellt (Entscheidung des Anwenders, nicht vergessen):
- **Auf eine negative Bestätigung zu reagieren.** Sie ist sichtbar (kein `DATA_CON`-Flag), löst aber
  nichts aus. Beachte, dass der Chip von sich aus bereits bis zu (NACK + BUSY + 1) mal wiederholt; eine
  negative Bestätigung heißt, dass *diese* erschöpft sind, ein weiterer Versuch ist also eine
  Entscheidung der Anwendungsebene, nicht des Datalink Layers.
- Die Deutung der Steuerbytes ist nur teilweise fertig. `handleControlEntry()` wertet
  `U_SystemStat.ind` und die Fehlerbits von `U_State.ind` aus; es fehlen noch `U_Configure.ind` (die
  Modusflags des Chips) und `L_Data.con`. Beachte, dass der Haken die Flags des Eintrags nicht
  übergeben bekommt, ein von einer Pause abgeschnittenes `U_SystemStat.ind` (das *sehr wohl* als
  `INVALID` markiert ist) lässt sich dort also noch nicht unterscheiden.
- Paritäts-/Framingfehler werden nirgends sichtbar. Die DMA liest nur das untere Byte des `DR` und
  verwirft damit die Fehlerbits des PL011, und `overflow()` schaut nur auf `OE` - während `rsr = 0`
  `PE`/`FE`/`BE` als Nebenwirkung löscht. Auf dem ESP32 ignoriert `drainEventQueue()` ebenso
  `UART_PARITY_ERR`/`UART_FRAME_ERR`. KNX läuft mit gerader Parität, ein Paritätsfehler zeigt sich
  derzeit also nur als CRC8-Fehler (~1/256 Restrisiko, dass er durchrutscht).
- Unterstützung für erweiterte CRC16 (NCN5120 CRC-16/CCITT oder TPUART2 CRC-16/SPI) - die Schicht geht
  ausschließlich von einfacher CRC8 aus, und **das ist eine Entscheidung, keine Lücke**. Sie sichert
  **den UART zwischen Chip und Host, nicht den Bus** - das Datenblatt ist da deutlich ("Optional CRC on
  UART to the Host", S. 1; "when active, NCN5130 accompanies every received frame with a 2-byte
  CRC-CCITT value", S. 40). Auf dieser Strecke gibt 8E1 bereits Parität je Byte, und was das überlebt,
  müsste immer noch die KNX-Prüfsumme passieren, die diese Schicht selbst prüft, ein verfälschtes
  Telegramm wird also als `INVALID` gemeldet statt verarbeitet. Dagegen stehen: zwei zusätzliche Bytes
  je Telegramm auf der Host-Strecke - genau der Engpass, der beim Tick-Intervall gemessen wurde - plus
  CRC16-Arbeit im Tick und zwei zusätzliche Bytes beim Empfangen.
  Das ist zugleich der Grund, warum der Reset im Verbindungsaufbau (`U_Reset.req`, `0x01`, siehe
  "Verbindungsaufbau" oben) über die Bestätigung der Baudrate hinaus zählt: eine frühere Sitzung oder
  ein ETS-Lauf kann den NCN5130 im Modus für erweiterte CRC hinterlassen, und ohne den Reset scheint
  jedes Telegramm zwei Bytes am Ende zu haben, die nicht dazugehören (genau dieses Bild trat während
  der Entwicklung auf und wurde so aufgeklärt).
- Das **Ergebnis** des `L_Data.con` auszuwerten. Es gibt den Sendeweg frei, aber sein MSB
  (positive/negative Bestätigung) wird nur protokolliert, nicht verarbeitet - eine negative Bestätigung
  sollte irgendwann eine Wiederholung auslösen.

## Build-Umgebungen

`platformio.ini` hat **drei Envs, und alle drei sind Testumgebungen**: `pico_test`, `esp32_test` und
`native_test`. Eine Produktions-Env gibt es bewusst nicht - dieses Repository ist eine Library, kein
Programm. Gebaut und geprüft wird ausschließlich über `pio test`:

```
pio test -e pico_test
pio test -e esp32_test
pio test -e native_test                                       (auf dem PC, ohne Gerät)
pio test -e pico_test --without-uploading --without-testing   (nur übersetzen, ohne Gerät)
```

**`native_test` ist eine Ergänzung, kein Ersatz.** Es fährt dieselben Fälle gegen dieselbe Library,
aber mit **gestellter Uhr**: `pump()` dreht die Zeit in 50µs-Schritten weiter, statt zu warten. Der
ganze Durchlauf kostet damit rund zwei Sekunden statt fünfundzwanzig, und die beiden
Fünf-Sekunden-Fälle kosten gar nichts mehr.
Was dabei **wegfällt, ist das Zeitverhalten selbst** - und daran ist hier schon ein Fehler
aufgefallen, den eine gestellte Uhr wegdefiniert (siehe die Zustellkosten der Testvorrichtung weiter
unten). Die Schrittweite *ist* die Tickrate; 50µs liegen unter dem 500µs-Vorgabetakt und weit unter
der Bytezeit des Busses, die Pausenerkennung sieht nativ also feiner auf als auf jeder echten
Plattform. Ein Fall, der nativ besteht, kann auf Hardware knapp scheitern. **Vor einer Freigabe zählen
die Hardwareläufe.**
Möglich ist das **ohne eine Zeile Änderung in `src/`**: die Library benutzt aus Arduino nur `millis()`
und `micros()`, und die plattformgebundenen Teile schalten sich über ihre eigenen
`ARDUINO_ARCH_*`-Klammern selbst ab (`Interface/RP2040.cpp`, `ESP32.cpp`, und `Timer.cpp` hat
einen `#else`-Zweig, der `supported()` auf `false` setzt). Den Ersatz liefert
`test/test_tpuart/native/Arduino.h`; nur diese Env nimmt den Ordner über `-I` in den Suchpfad, in den
Hardware-Envs liegt das echte `Arduino.h` davor.
Der Schim ist **header-only, und das ist Bedingung**: eine `.cpp` dort würde PlatformIO in *jeder* Env
mitübersetzen (der Testordner wird rekursiv gebaut) und den Hardware-Envs ein zweites `millis()`
verpassen. Der Zählerstand steckt deshalb in einem funktionslokalen `static`.
Was nativ nicht mitläuft, ist `interface_check.cpp` - es instanziiert `ArduinoSerial<decltype(Serial1)>`
und die Plattform-Interfaces, und beides gibt es dort nicht. Die Env braucht einen **Host-Compiler im
`PATH`** (`g++` oder `clang++`); fehlt er, scheitert sie beim Übersetzen, ohne die anderen beiden zu
berühren.

`pio run` ist **kein Einstiegspunkt** und scheitert im Linker mit `undefined reference to 'setup'`:
`setup()` und `loop()` kommen aus `test/test_tpuart/test_main.cpp`, und dieser Ordner wird nur bei
einem Testlauf mitübersetzt. Zum Aufräumen taugt `pio run` trotzdem, das Ziel kommt vor dem Linker:
`pio run -t clean -e pico_test`.

**Dass jedes Interface sich überhaupt übersetzen und linken lässt, prüft
`test/test_tpuart/interface_check.cpp`**, und das läuft in beiden Testumgebungen mit. Dort wird
`ArduinoSerial<decltype(Serial1)>` ausdrücklich instanziiert - als Template entsteht ohne Aufrufer
kein einziger Rumpf, ein Fehler darin fiele sonst erst im Fremdprojekt auf -, und je Plattform werden
`RP2040` bzw. `ESP32` mit einem `DataLinkLayer` verdrahtet. Die Funktion dort wird **nie aufgerufen**:
der Compiler übersetzt sie vollständig und der Linker löst jedes Symbol auf, aber kein Konstruktor
läuft und kein Interface fasst Hardware an. Ein globales Objekt stattdessen beanspruchte auf dem
RP2040 beim Start einen DMA-Kanal, den die echten Tests dann nicht mehr bekämen.

Beide Envs bauen immer mit `framework = arduino` - einen nativen Host-Bau gibt es nicht.


### Tests

`pio test -e pico_test` und `-e esp32_test` fahren die Sammlung in `test/test_tpuart/test_main.cpp`
**auf dem Zielgerät**, gegen das `Dummy`-Interface, das im selben Ordner liegt. Unity ist das
Framework (PlatformIOs Vorgabe, automatisch geholt - im Projekt landet keine Abhängigkeit).

`test_build_src = yes` ist tragend: ein Testbau übersetzt `src/` sonst gar nicht und die Library
fehlte schlicht.

**`test/test_tpuart/unity_config.h` gibt es für genau eine Zeile.** PlatformIO erzeugt diese Datei
sonst selbst und beendet sie mit `void unityOutputComplete(void) { Serial.end(); }`. Auf einem Board,
dessen `Serial` das USB-CDC *ist*, meldet das das Gerät mitten in `UNITY_END()` vom Bus ab: dem
Testläufer fehlt die Schlusszeile (`ClearCommError failed` nach 23 von 24 Fällen), und der nächste
Upload findet überhaupt keinen Port mehr (`Please specify upload_port`), bis das Board neu gesteckt
wird. Unsere Fassung leert stattdessen den Puffer. Die Datei bereitzustellen heißt zugleich, die vier
Ausgabefunktionen bereitzustellen - sie stehen am Kopf von `test_main.cpp`.
`setup()` wartet vor `UNITY_BEGIN()` darauf, dass der Host den Port öffnet (mit einer Frist, damit die
Sammlung auch unbeaufsichtigt läuft); ohne das fehlt der erste Fall regelmäßig im Bericht. Nach dem
Lauf hält `loop()` die Verbindung offen und nimmt zwei Tasten entgegen: `r` fährt die Sammlung erneut,
`b` setzt den RP2040 in den Bootloader.

**Alle drei Umgebungen sind verifiziert**, nicht nur gebaut: 96 von 96 auf `pico_test` und dieselben
96 auf `esp32_test` (~39s), beide auf echter Hardware, dazu `native_test` (~1,5s). Da die
Fälle echte Fristen ausmessen, ist gerade die Übereinstimmung über zwei sehr verschiedene Uhren und
Treiberpuffer hinweg das Interessante am Ergebnis - besonders bei den Buslast-Fällen, deren Schranken
an einer Messspanne von rund 1000ms hängen: nativ ist die deterministisch, auf Hardware nicht.
**WAS DIE SAMMLUNG NICHT PRÜFT, ist der Antrieb selbst.** Die Vorrichtung hält `Timer::setInterval(0)`,
getickt wird ausschließlich aus `pump()`/`tickOnly()`. Geprüft ist damit die Buchführung des Timers
(Eintragen, Austragen, Plätze, Intervall) - dass der Hardware-Timer wirklich mit dem eingestellten Takt
feuert, belegt nur der Betrieb auf einem Gerät. Für das Singleton steht dieser Nachweis noch aus; die
Messungen mit 0 Verzögerungen über 26000 Ticks stammen vom vorherigen, instanzeigenen Antrieb.
**Der native Lauf ersetzt die beiden Hardwareläufe trotzdem nicht**, und dafür gibt es jetzt einen
Beleg statt einer Vermutung: er war grün, als `pico_test` rot war, und konnte den Fehler prinzipiell
nicht sehen - dort hängt die Uhr an den Schleifendurchläufen und nicht an der Wanduhr, eine teure
Testvorrichtung kostet also nichts (siehe die Zustellkosten weiter unten).

**Die Testvorrichtung muss billig zustellen, sonst verfälscht sie Zeitmessungen.** Mit
angehaltenem Timer laufen Tick und Loop serialisiert: was der Telegramm-Callback kostet, liegt
zwischen dem Tick, der das letzte Byte einer Sequenz verbraucht, und dem nächsten, der die Leere
bemerkt und `_emptySince` setzt - die Zustellkosten verschieben also die Pausenerkennung nach hinten.
`_frames` wuchs während eines Falls um und kopierte dabei jeden Eintrag samt Datenvektor neu;
`test_confirmation_after_broken_echo_arrives_after_pause` fiel dadurch auf dem RP2040 in etwa der
Hälfte der Läufe durch - die Pause wurde erst erkannt, nachdem das `L_Data.con` bereits eingetroffen
und im `Resync` verworfen war. Behoben mit `reserve()` und `push_back(std::move(...))`; auf dem ESP32
war die Reserve zufällig groß genug, dort fiel es nie auf.

**Derselbe Fehler ein zweites Mal, an anderer Stelle: `Dummy::available()` war quadratisch.** Es lief
bei jedem Aufruf von `_pos` bis zum Ende der Warteschlange, und `read()` ruft es gleich noch einmal -
bei einem ohne Pause eingespeisten Block sind damit alle Bytes sofort verfügbar und jeder Aufruf zählt
den ganzen Rest erneut ab. `test_rx_queue_overflow_is_counted` speist 900 Bytes ein, das sind rund
810.000 Schleifendurchläufe je Richtung: auf dem PC unsichtbar, auf dem RP2040 über 70ms gegen ein
80ms-Budget. Der Fall verlor seine Ticks also an die Vorrichtung und meldete 0 Ringüberläufe - **nativ
grün, auf Hardware rot**. Behoben, indem der Fahrplan fortgeschrieben statt neu durchgerechnet wird
(`_availableCount`/`_scanArrivesAt`); tragend ist dabei, dass `read()` `_nextAvailableAt` um genau die
Pause weiterschiebt, um die `_pos` vorrückt - die absoluten Ankunftszeiten der verbliebenen Bytes
ändern sich dadurch nicht, ein einmal verfügbares Byte bleibt also verfügbar.
Die Lehre ist dieselbe wie beim `_frames`-Fall und inzwischen zweimal belegt: **Kosten in der
Vorrichtung sind nicht neutral, sie gehen dem Prüfling vom Zeitbudget ab** - und der native Lauf kann
das nicht zeigen, weil dort die Uhr an den Schleifendurchläufen hängt und nicht an der Wanduhr. Genau
deshalb zählen vor einer Freigabe die Hardwareläufe.

**Die neun Busmonitor-Fälle sind gegen Mutation geprüft**, und das war nötig: `abort()`
auszukommentieren ließ zunächst nur *einen* der beiden dafür gedachten Fälle scheitern.
`test_monitor_aborts_running_transmission` bestand weiter, weil schon der Wächter in
`Transmitter::process()` verhindert, dass noch Bytes hinausgehen - der Fall prüfte also den Wächter
und nicht den Abbruch. Erst die zusätzliche Zusicherung auf `TxState::Idle` bindet ihn an
`abort()`. Wer hier einen Fall ergänzt: prüfe, ob er ohne die zugehörige Änderung wirklich scheitert.

`monitor_speed = 115200` steht in `[env]` und muss zum `Serial.begin(115200)` in `test_main.cpp`
passen; PlatformIOs Vorgabe wäre 9600. Das fiel nur auf dem ESP32 je ins Gewicht, wo `Serial` ein
echter UART hinter dem Brückenchip ist - auf dem RP2040 ist `Serial` das USB-CDC und die Baudrate
Zierde, weshalb die fehlende Zeile bis zur ersten ESP32-Monitorsitzung unbemerkt blieb, die dann nur
Buchstabensalat zeigte. Der Testläufer selbst war nie betroffen: er benutzt `test_speed`, dessen
Vorgabe bereits 115200 ist.

**Auf der Hardware läuft alles in Echtzeit - dort gibt es keine stellbare Uhr** (die hat nur
`native_test`, siehe oben). Ein Fall kostet, was die Sache kostet:
2,6ms für die Pausenerkennung, 5s für einen Verbindungsverlust. Das ist der Preis dafür, die echten
Fristen zu prüfen statt heruntergedrehter, und der ganze Lauf bleibt trotzdem im Sekundenbereich.
Wird das je untragbar, ist der Hebel eine Indirektion `TPUart::nowMs()`/`nowUs()` - 14 direkte
`millis()`/`micros()`-Aufrufstellen in 7 Dateien, was zugleich einen nativen Bau möglich machte.
`TPUART_TX_CONFIRM_TIMEOUT_MS` wird in `[env]` von 10s auf 1s gesenkt. Zwei Fälle hängen daran - der
Wachhund, der bei ausbleibendem `L_Data.con` einen Reset auslöst, und die Auffrischung der Frist durch
jedes Echo. Beide brauchen echte Wartezeit; mit dem Vorgabewert dauerte allein der Wachhund-Fall zehn
Sekunden. Geprüft wird damit der Mechanismus, nicht die Zahl.

Jeder Fall bekommt eine **frische Vorrichtung** (`Dummy` + `DataLinkLayer`, auf dem Heap in
`setUp()`/`tearDown()`), weil `RxState`/`TxState` von außen nicht zurücksetzbar sind - ein
liegengebliebener Resync sickerte sonst in den nächsten Fall. Die Vorrichtung setzt
`Timer::setInterval(0)`, damit kein Timer nebenher läuft; getickt wird ausschließlich aus `pump()` bzw.
`tickOnly()`. Einen Fall
hinzuzufügen heißt, eine parameterlose Funktion zu schreiben und sie mit `RUN_TEST()` in `setup()`
aufzuführen - diese Liste ist die einzige Stelle, die von ihm weiß.

Zum RP2040: hier stand zwischenzeitlich eine zweite Env, die über J-Link flashen sollte, weil der
USB-Port am Ende jedes Testlaufs verschwand. Die Ursache lag aber nicht am USB, sondern an der
Unity-Anbindung von PlatformIO (siehe `unity_config.h` oben). Mit eigener `unity_config.h` läuft der
Upload ganz normal über USB, auch mehrfach hintereinander. Die J-Link-Env ist damit weg.

### `library.json`

Version **2.0.0**. `frameworks`/`platforms` sind gesetzt, damit der LDF keine fremden Plattformen
versucht. Einen `srcFilter` braucht es nicht: `src/` enthält ausschließlich Library-Code.
**Legst du eine Datei in `src/`, die nicht Teil der Library ist, brauchst du den Filter wieder.**
Testcode braucht hier keinen Eintrag - er liegt in `test/`, was ein Verbraucher nie sieht.

## Die API gehört den Verbrauchern

Drei Projekte binden diese Library ein - `knx/src/knx/tpuart_data_link_layer.{h,cpp}`, `OGM-Common`
(`Console.cpp`, `Common.cpp`, `Hardware.cpp`) und `OFM-Network` (`TelegramJson.cpp`, `Module.cpp`,
`Webserver/GroupMonitor.cpp`). Sie müssen übersetzen, ihre Aufrufe legen damit einen Teil der
Oberfläche fest. Drei Gruppen, und nur die erste ist schuldenfrei:

1. **Echt und bleibt.** `TPUart.h` (Sammelheader), **mehrere Telegramm-Callbacks**
   (`registerFrameCallback()` hängt an - der IP-Router hat **vier** Zuhörer: den Empfangspfad des
   knx-Stacks, `bcu debug` von OGM-Common, den MQTT-Publisher von OFM-Network und dessen
   Gruppenmonitor; mit nur einem Platz hätte der letzte alle davor stillschweigend abgeschaltet),
   `Frame::data()` mit Rückgabetyp **`const char *`** statt `const uint8_t *` (OFM-Network schreibt
   `const char *d = frame.data()`; da der Rückgabetyp nicht überladbar ist, heißt der Schreibzugriff
   für den, der ein Telegramm füllt, `buffer()`), `Frame::cemiData()`/`cemiSize()` (der Stack braucht
   die cEMI-Sicht; `cemiData()` liefert einen `malloc`-Puffer, **den der Aufrufer freigeben muss**),
   und die Klassennamen `ESP32`/`RP2040` samt Dateinamen (`ESP32.h`, `RP2040.h` - ein `#include`
   unterscheidet unter Linux Groß- und Kleinschreibung).
2. **Im Code mit `KOMPAT` markiert, entfernbar, sobald die Verbraucher nachgezogen sind.** Doppelte
   Schreibweisen (`isMonitoring()`, `registerReceivedFrame()`, `getBcuStateInfo()`, `BcuType::BCU_*`,
   `AcknowledgeType`/`ACK_*`, `Statistics::getRxDiscardedBytes()` und Verwandte) und zusätzliche
   Einstiegspunkte (`DataLinkLayer()` ohne Interface plus `begin(type, interface*)`, `process()` - heute nur noch `loop()`, früher `tick()` + `loop()` -, `end()`, `pushTransmitQueue(Frame*)`, das bei Erfolg den Besitz übernimmt und
   die Länge um eins kürzt, weil die Library die Prüfsumme selbst rechnet) und `Frame(const char*,
   size_t)`.
3. **Platzhalter aus der Zeit des SearchBuffers**, den es hier nicht mehr gibt.
   `Receiver::getSearchBufferPosition()` liefert den Füllstand des Empfangspuffers und
   `Receiver::getAwaitBytes()` die noch ausstehenden Bytes des laufenden Telegramms (0, solange die
   Größe noch nicht feststeht) - beide statt der früheren konstanten 0.
   **`OGM-Common`s `bcu`-Befehl druckt sie inzwischen nicht mehr**; sie stehen damit ohne Aufrufer da
   und bleiben nur auf ausdrückliche Entscheidung des Anwenders erhalten, nicht mehr aus einem
   Kompatibilitätszwang. Wer hier aufräumt, kann sie streichen, sobald das bestätigt ist.
   Bei **0** bleibt allein `Statistics::getRxSearchBufferOverflow()` - dafür gibt es in dieser
   Library nichts Vergleichbares, jeder Ersatzwert wäre eine Falschaussage. Die Parameter `irq`/`dma`
   des `RP2040`-Konstruktors werden aus demselben Grund ignoriert wie früher: es gibt nur den
   DMA-Pfad.

**Nichts in diesem Repository fasst diese Oberfläche an**, kein Test und kein Beispiel - die Gruppen
2 und 3 werden also nirgends übersetzt. Ein Bruch fällt deshalb erst auf, wenn eines der drei Projekte
gegen diese Library gebaut wird. Hier stand einmal eine Datei `src/compat_check.cpp`, die genau dafür
jeden Aufruf der Verbraucher einmal ausschrieb; sie ist entfernt, weil sie eine handgepflegte Kopie
war und der Bau des Verbrauchers die eigentliche Prüfung ist. Wie leicht es passiert, zeigt der eine
Fund, den es dort gab: OFM-Network war gar nicht erfasst, bis der IP-Router zum ersten Mal gegen diese
Library gebaut wurde - und die Signatur von `data()` brach sofort.
**Baue nach einer Änderung an der Oberfläche einen Verbraucher, bevor du sie für fertig hältst.**
Der schnellste ist **`OAM-TestApp`**: dort liegen die OpenKNX-Repositories als relative Symlinks in `lib/`,
diese Library gehört als `lib/tpuart -> ../../tpuart` dazu (unter Windows `mklink /D`, und relativ wie die
übrigen Einträge). Damit übersetzt ein Bau der TestApp `knx`, `OGM-Common` und `OFM-Network` gegen den
Arbeitsstand hier - genau die drei Verbraucher, deren Aufrufe die Gruppen 2 und 3 festlegen.

## Fehler, die hier schon einmal gemacht wurden

Keiner davon steht noch im Code. Sie stehen hier, weil jeder von ihnen beim Lesen plausibel aussah
und erst am laufenden Bus auffiel - wer an denselben Stellen arbeitet, läuft in dieselbe Falle.

- `_busMonitorPending` wurde geschrieben, *nachdem* `queueControl()` den Versand veröffentlicht hatte,
  ein Tick in diesem Fenster rastete also den alten Wert - Chip-Zustand und `_busMonitor` liefen danach
  dauerhaft auseinander (Quittungen in eine passive Spur, oder nach einem Reset gar keine mehr).
  Behoben, indem der Modus aus dem tatsächlich gesendeten Code abgeleitet wird.
- Ringüberläufe im `RP2040` wurden nie gemeldet: die Selbstkorrektur in `read()` löschte die
  Zählerdifferenz, die `overflow()` danach prüfte, der Zweig war also tot und ein stehengebliebener
  Hauptloop verlor Telegramme unsichtbar. Behoben mit einer Rastung.
- Dieselbe Korrektur sprang zum *neuesten* Byte und verwarf damit einen vollen Ring noch gültiger
  Daten, was den Verlust um die Puffergröße vervielfachte.
- `sendAcknowledge()` benutzte `>`, wo der Grenzfall ("Telegramm bereits vollständig gepuffert") `>=`
  braucht.
- `L_Poll_Data.ind` wurde als 1-Byte-Steuerbyte zerlegt, was ein Phantomtelegramm und eine fremde
  Quittung auf dem Bus erzeugen konnte.
- **Eine Korrektur in genau diesem Review brach den Empfang und musste ihrerseits korrigiert werden**:
  `end()` wurde von `dma_channel_abort()` auf `dma_channel_cleanup()` umgestellt, um einen
  liegengebliebenen Fertig-IRQ-Status zu löschen. `cleanup()` fasst zusätzlich das CTRL-Register des
  Kanals an, und die Kanalkonfiguration lebte *nur* im Konstruktor - ab dem zweiten
  Baudratenkandidaten schrieb die DMA also nie wieder, und `isConnected()` blieb für immer falsch.
  Jetzt: `abort()` + `dma_channel_acknowledge_irq0()`, und `begin()` wendet die vollständige
  Kanalkonfiguration erneut an, der Empfang hängt also nicht mehr davon ab, was `end()` in Ruhe lässt.
  Die Lehre verallgemeinert sich: eine SDK-Hilfsfunktion, deren Name zur Absicht passt, hat nicht
  automatisch den richtigen Wirkungsbereich, und eine Änderung im RX-Datenpfad, die nur übersetzt, ist
  nicht getestet.

## Offene Punkte / mögliche nächste Schritte

- Echte Adressfilterung hinter `registerCheckAcknowledge()`. Ohne registrierten Callback wird nichts
  quittiert; welche Ziele das Gerät als seine ansieht, muss der Aufrufer entscheiden.
- `TPUART_FRAME_WAIT_US`: 3000 gegen die 30-Bitzeiten-Toleranz der Quittung abwägen - eine
  Entscheidung, die am echten Bus zu treffen ist.
- RX-Puffer-Asymmetrie: RP2040 bis 2048 Byte, ESP32 512. Derselbe Stresstest verliert auf dem ESP32
  deutlich mehr (wird dort aber als `UART_BUFFER_FULL` gemeldet).
- Die FIFO-Frage im `RP2040`: die ursprüngliche Begründung fürs Abschalten des RX-FIFO ist nicht
  belegt. Mit `FEN=1` wäre das Verlustfenster beim DMA-Neustart ~18ms statt ~570µs. Korrigiert wurde
  nur der Kommentar; die Umstellung braucht eine Messung an echter Hardware, weil die Abschaltung aus
  einer Beobachtung am laufenden Bus stammt.
- Speicherbarrieren für die beiden SPSC-Ringe auf dem ESP32: `volatile` ordnet formal nur volatile
  Zugriffe untereinander, ist also keine Release-Semantik. Auf dem RP2040 folgenlos (ISR desselben
  Kerns), auf dem ESP32 mit echter Parallelität formal nicht garantiert - praktisch mit heutigem GCC
  nicht beobachtet.
- **`ArduinoSerial` ist weiterhin nur kompiliert, nie an echter Hardware gelaufen.** Es ist ein
  Kandidat für genau den Fehler, den der ESP32-Treiber hatte: ob der eingepackte Arduino-Kern
  Empfangenes stapelt, ist nicht geprüft, und die Pausenerkennung hängt daran.
- **`ESP32` ist am echten Bus verifiziert**: empfangen (31 Telegramme ohne Verlust), gesendet (263 Byte
  samt Echo, drei Wiederholungen, NACK), quittiert, Chip-Zustand gelesen. Nicht geprüft sind dort
  Busmonitor, Stop-Modus und die Umschaltung der Spannungsregler. Tick auf dem ESP32: Median 5µs,
  Maximum 44µs (RP2040: Median 1µs, Maximum 2µs) - der Abstand ist deutlich, aber weit innerhalb des
  500µs-Intervalls.
- Die Testsammlung läuft auf beiden Plattformen auf echter Hardware, prüft dort aber gegen den `Dummy`:
  verifiziert sind damit Protokoll und Zeitverhalten der Library, nicht die Interface-Klassen.
- **Der Busmonitor ist nie an einem echten Chip gelaufen.** Die neun Testfälle laufen gegen den `Dummy`,
  und die Wächter selbst sind aus Tabelle 11 des NCN5130-Datenblatts (S. 32) abgeleitet - gelesen, nicht
  gemessen. Offen ist damit vor allem, ob der Chip die dort als `I` geführten Dienste tatsächlich
  folgenlos verwirft, und ob `stopMode()`/`powerControl()` im Modus wirklich durchgreifen.
- Poll-Slave-Betrieb (`U_PollingState.req`, beim NCN zusätzlich Auto-Polling über `U_Configure.req`)
  bewusst nicht umgesetzt - nur damit könnte das Gerät an einem Poll überhaupt teilnehmen.
  Empfangsseitig ist der Fall abgedeckt, und kein Verbraucher der Library kennt Polldaten.
- Kein Umgang mit dem NCN5130-Merkmal "FrameEnd mit Marker" (Verdopplung von `0xCB`-Bytes) - belegbar
  nicht erreichbar, solange `U_Configure.req` nie gesendet wird, aber nicht ausdrücklich abgesichert.
- Tote Codestellen, bewusst stehengelassen: beide Überladungen von `Frame::setAcknowledge()` (dieselbe
  Abbildung wie `acknowledgeFlags()` in der Schicht, aber mit abweichender Bedeutung - die eine löscht
  Bits vorher, die andere ODERt nur), `addFlags()`/`resetFlags()` sowie mehrere unbenutzte Konstanten
  in `Types.h`.

## Codestil

- **Jede Membervariable trägt das Präfix `_`** - ausnahmslos, einschließlich der Felder einfacher
  Datenstrukturen (`Dummy::QueuedByte`, `TPUartDmaCompleteRegistration` in `RP2040.cpp`), nicht nur
  gekapselter Klassenzustand.
- Kommentare sind auf Deutsch, Bezeichner auf Englisch; diese Datei ist auf Deutsch.
- STL und Heap-Allokation grundsätzlich meiden. Drei Ausnahmen, alle bewusst: `std::function` für die
  Callbacks und `malloc`/`free` für die Telegramme der Sendequeue (beides ausdrückliche Entscheidungen
  des Anwenders - siehe den Transmitter oben), dazu `std::vector`/`std::string` innerhalb des
  **`Dummy`-Interfaces und in `Frame::printFrame()`**, was Test- bzw. Diagnosecode ist und nicht
  Datenpfad. In der Schicht selbst gibt es keinen STL-Container, und der Heap wird nur aus dem
  Hauptkontext angefasst.

## Arbeitsweise in diesem Projekt

Der Anwender treibt Entwurfsentscheidungen Schritt für Schritt und will mehrdeutige Verzweigungen
ausdrücklich vor der Umsetzung hinterfragt haben (Rückfragen bündeln statt anzunehmen - ähnlich der
"grill-me"-Methode: einen Entscheidungsbaum der offenen Zweige aufbauen, alles derzeit Beantwortbare
in einem Schwung fragen, eigenständig recherchierbare technische Tatsachen selbst klären statt zu
fragen, und Zweig für Zweig weitermachen, bis nichts mehr offen ist). Wenn etwas wirklich mehrdeutig
oder eine Abwägung ist, frage; wenn es eine sachliche/technische Frage ist, die du selbst prüfen kannst
(z.B. ob eine Plattform eine API unterstützt), recherchiere sie und berichte das Ergebnis, statt den
Anwender nachschlagen zu lassen.
