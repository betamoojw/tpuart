# History.md - Zurückgestellt, frühere Fehler, offene Punkte

Teil von `AGENTS.md`.

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
- **Kein `IRAM_ATTR` für den Tick-Pfad auf dem ESP32.** Getestet mit 85 markierten Funktionen (Timer,
  Tick, Receiver, Transmitter, Statistik-Zähler, `ESP32`-Interface) am Router mit NCN5130, jeweils im
  Release-Build: Max run 344µs (ack 212µs) mit IRAM gegen 353µs (ack 233µs) ohne - kein messbarer
  Gewinn, dafür dauerhaft belegter knapper IRAM. Die Gründe: `uart_get_buffered_data_len()` liegt als
  Treiberfunktion ohnehin im Flash, der Leerlauf-Tick liest also bei jedem Durchlauf von dort; ein
  Pfad, der 2000-mal je Sekunde läuft, fällt nicht aus dem Flash-Cache; und während eines
  Flash-Schreibvorgangs hält ESP-IDF den `esp_timer`-Task trotzdem an. Der Quittungs-Callback des
  Aufrufers bliebe ohnehin im Flash.
  Falls der Leerlauf-Tick doch schlanker werden muss, ist der Ansatz nicht IRAM, sondern den
  Treiberaufruf zu vermeiden: `available()` fragt `uart_get_buffered_data_len()` erst nach einem
  Daten-Event aus der Event-Queue (`xQueueReceive`, im RAM). Das ändert die Pausenerkennung und braucht
  einen Hardwarelauf. Anlass wäre ein Deferred, das im Release-Build über Stunden auffällig bleibt.
  **Tick-Werte nur im Release-Build vergleichen.** Die `develop`-Umgebungen erben aus OGM-Common
  `build_type = debug` und damit `-O0`: dort lagen Max run 1203µs (ack 793µs) und Deferred bei rund
  140 je Stunde - der vermeintliche Rückschritt der Library war ausschließlich der Build-Typ.

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
  **Der Stop-Modus selbst ist inzwischen belegt** - allerdings außerhalb des Busmonitors: der
  Registerzugriff beim Verbinden fährt den Chip hinein, liest dort seine Register und kommt über den Reset
  sauber heraus (am NCN5130 gelaufen). Was das nicht abdeckt, ist der Stop-Modus *im* Busmonitor.
  **`powerControl()` ist am Chip verifiziert**, über den SAVEB-Pfad der Anwendung: im `U_SystemStat.ind`
  verschwinden `V20V` und `VDD2`, während `VBUS` und `VFILT` gesetzt bleiben. Genau diese Kombination
  trennt die beiden Deutungen - die Busspannung war in Ordnung, die zwei Schienen wurden also abgeschaltet
  und sind nicht eingebrochen. Nach `powerControl(true)` stehen beide wieder.
- Poll-Slave-Betrieb (`U_PollingState.req`, beim NCN zusätzlich Auto-Polling über `U_Configure.req`)
  bewusst nicht umgesetzt - nur damit könnte das Gerät an einem Poll überhaupt teilnehmen.
  Empfangsseitig ist der Fall abgedeckt, und kein Verbraucher der Library kennt Polldaten.
- Kein Umgang mit dem NCN5130-Merkmal "FrameEnd mit Marker" (Verdopplung von `0xCB`-Bytes) - belegbar
  nicht erreichbar, solange `U_Configure.req` nie gesendet wird, aber nicht ausdrücklich abgesichert.
- Tote Codestellen, bewusst stehengelassen: beide Überladungen von `Frame::setAcknowledge()` (dieselbe
  Abbildung wie `acknowledgeFlags()` in der Schicht, aber mit abweichender Bedeutung - die eine löscht
  Bits vorher, die andere ODERt nur), `addFlags()`/`resetFlags()` sowie mehrere unbenutzte Konstanten
  in `Types.h`.
