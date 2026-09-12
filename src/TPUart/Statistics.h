#pragma once
#include <stdint.h>

namespace TPUart
{

// Zähler für Diagnose. Das Namensschema ist durchgehend: RICHTUNGSPRÄFIX IMMER (getRx…/getTx…),
// die Einheit steht im Namen (…Frames zählt Telegramme, …Bytes zählt Bytes, …Overflows/…Losses zählen
// Ereignisse), und nichts wird abgekürzt. Wer einen Zähler ergänzt, hält sich daran - sonst ist in einem
// Jahr wieder unklar, ob getInterfaceOverflows den Empfang oder den Versand meint (es war der Empfang).
//
// Der eigentliche Punkt der Aufteilung: **fehlerhafte Telegramme und verworfene Bytes sind zwei
// verschiedene Dinge.**
//   - Ein fehlerhaftes Telegramm ist eines, das gemeldet WURDE, aber kaputt ist (CRC falsch, von einer
//     Pause abgeschnitten, Längenbyte korrupt). Der Aufrufer hat es gesehen, mit INVALID-Flag.
//   - Verworfene Bytes hat nie jemand gesehen. Drei Quellen: alles, was im Resync anfällt, weil nach einem
//     Fehler die Position im Bytestrom unbekannt ist; die Reste einer Sequenz, die ein Moduswechsel
//     abbricht; und ein fertiger Eintrag, für den im RX-Ring kein Platz mehr war.
//
// ACHTUNG BEIM AUFSUMMIEREN: die Kategorien sind KEINE ZERLEGUNG, und sie weichen in BEIDE Richtungen
// von getRxBytes() ab. Wer eine Konsolenzeile "Bytes = frame + ctrl" schreibt, behauptet etwas Falsches -
// genau das ist hier schon passiert und fiel erst am laufenden Gerät auf.
//
//   ÜBERSCHNEIDUNG: ein Telegramm, das mangels Platz im Ring verworfen wird, steht mit seinen Bytes AUCH
//   in getRxFrameBytes() - es kam ja über den Bus, und daran hängt die Buslast.
//
//   DURCHFALL: getRxBytes() zählt JEDES vom Interface gelesene Byte, die beiden Kategorien nur die einer
//   ABGESCHLOSSENEN Sequenz (completeSequence(), mit der gepufferten Länge). Was in RxState::FrameAck
//   verbraucht wird, wird deshalb nirgends kategorisiert: das L_Data.con zu einem selbst gesendeten
//   Telegramm und ein geschlucktes U_FrameState.ind. Beide werden absichtlich nicht an den Puffer
//   angehängt - sie gehen als FLAG ans Telegramm, nicht als Byte. Auf einem viel sendenden Gerät sind das
//   ein bis zwei Bytes je gesendetem Telegramm, also durchaus sichtbare Größenordnungen.
//
// Die Zähler beantworten verschiedene Fragen. Sie summieren sich nicht, und sie sollen es auch nicht.
//
// GESENDETE TELEGRAMMBYTES gibt es bewusst nicht als eigenen Zähler, sie sind ableitbar - aber nicht so
// einfach, wie es aussieht: die Quittung geht ebenfalls über Transmitter::writeByte() und steckt damit in
// getTxBytes(). Richtig ist
//
//     getTxBytes() - getTxControlBytes() - getTxAcknowledges()
//
// denn eine Quittung ist genau ein Byte. Enthalten sind darin auch die Positions- und Offsetbytes, es sind
// also die Bytes des Telegrammpfades und nicht die Oktetts des Telegramms.
//
// Geschrieben wird aus dem Tick (also je nach Betrieb aus einem Interrupt), gelesen aus dem Hauptkontext.
// Deshalb volatile und nur einfache Inkremente - ein Zähler hat genau einen Schreiber. Die drei Ausnahmen
// sind an ihrer Deklaration vermerkt; auch sie haben genau einen Schreiber, nur eben den Hauptkontext.
// reset() aus dem Hauptkontext kann theoretisch mit einem Inkrement kollidieren; bei reiner Statistik ist
// das hinnehmbar und nicht wert, dafür zu sperren.
class Statistics
{
  private:
    volatile uint32_t _rxBytes = 0;
    volatile uint32_t _rxFrames = 0;
    volatile uint32_t _rxInvalidFrames = 0;
    volatile uint32_t _rxFrameBytes = 0;
    volatile uint32_t _rxControlBytes = 0;
    volatile uint32_t _rxDroppedBytes = 0;
    volatile uint32_t _rxRepeatedFrames = 0;

    volatile uint32_t _txControlBytes = 0;
    volatile uint32_t _txFrames = 0;

    // JEDES an die Schnittstelle übergebene Byte, unabhängig wofür - das Gegenstück zu _rxBytes. Der
    // einzige direkte Maßstab für die Auslastung der Hostleitung: bei 38400 Baud und 8E1 (11 Bit je
    // Zeichen) passen dort 3490 Byte/s durch. Gezählt wird in Transmitter::writeByte(), dem einzigen
    // Schreibzugriff dieser Klasse auf das Interface.
    volatile uint32_t _txBytes = 0;
    volatile uint32_t _txAcknowledges = 0;
    volatile uint32_t _txAcknowledgesSuppressed = 0;

    volatile uint32_t _rxInterfaceOverflows = 0;
    volatile uint32_t _rxQueueOverflows = 0;
    volatile uint32_t _txControlQueueOverflows = 0;
    volatile uint32_t _txQueueOverflows = 0;

    // Der Wachhund des Transmitters hat zugeschlagen: zu einem gesendeten Telegramm kam ÜBERHAUPT KEIN
    // L_Data.con, der einzige Weg in einen definierten Zustand war ein Reset der BCU. Das ist etwas anderes
    // als ein negatives L_Data.con - das sagt nur, dass auf dem Bus niemand quittiert hat, und ist damit
    // eine Aussage über den Bus, kein Fehler der Strecke. Hier dagegen hat der Chip gar nicht geantwortet,
    // und das zeigt auf ihn selbst oder auf die Verkabelung. Nur dieser Fall wird gezählt.
    volatile uint32_t _txConfirmTimeouts = 0;

    // Aus dem HAUPTKONTEXT (connectionLost()), nicht aus dem Tick - ein Schreiber bleibt es trotzdem.
    volatile uint32_t _connectionLosses = 0;

    // Wie oft die Position im Bytestrom verloren ging. Das ist eine ANDERE Frage als getRxDroppedBytes():
    // drei Resyncs zu je 5 Byte und einer zu 15 kosten dieselbe Zahl Bytes, sind aber völlig verschiedene
    // Befunde - der eine heißt "es hakt wiederholt", der andere "einmal längere Störung".
    volatile uint32_t _rxResyncs = 0;

    // DIE FEHLER, DIE DER CHIP SELBST MELDET (Fehlerbits im U_State.ind), einzeln gezählt. Die Bits
    // werden im DataLinkLayer nur ODER-akkumuliert und beim Ausgeben gelöscht, weil der Chip jedes
    // Ereignis genau einmal meldet - "einmal vor Stunden" und "dauernd" sehen dort also gleich aus.
    //
    // Einzeln und nicht als Summe, weil die fünf verschiedene Diagnosen sind: eine stehende
    // Übertemperaturwarnung ist etwas anderes als gelegentliche Kollisionen auf dem Bus. Ein
    // gemeinsamer Zähler über alle fünf fehlt bewusst - er sagte nichts, was diese hier nicht besser
    // sagen. Er ist auch nicht nachträglich zu bilden, weil ein U_State.ind mehrere Bits zugleich
    // tragen kann; das ist hier kein Verlust.
    //
    // Kein Rx/Tx-Präfix: das sind Zustände des Chips, keine Richtung von uns aus. TE und SC entstehen
    // beim Senden auf dem Bus, RE beim Empfangen - ein gemeinsames Präfix wäre in jedem Fall falsch.
    volatile uint32_t _chipSlaveCollisions = 0;
    volatile uint32_t _chipReceiveErrors = 0;
    volatile uint32_t _chipTransmitErrors = 0;
    volatile uint32_t _chipProtocolErrors = 0;
    volatile uint32_t _chipTemperatureWarnings = 0;

    // DER TAKT SELBST. Er ist die Voraussetzung für alles andere in dieser Schicht, und bis hierher war er
    // der einzige Wert, der sich NICHT ablesen ließ - man sah nur seine Folgen (getTxAcknowledgesSuppressed,
    // getRxInterfaceOverflows) und musste auf die Ursache raten. Genau diese Rateübung hat einen Abend
    // gekostet, deshalb steht die Messung jetzt hier.
    //
    // _ticks gegen die Laufzeit gerechnet ergibt die MITTLERE Taktrate - der belastbare Wert, denn ein
    // einzelner Ausreißer verzerrt ihn nicht. Die Verzögerungen daneben beantworten die andere Hälfte:
    // ein durchgehend zu langsamer Antrieb (Mittelwert schlecht) gegen einen, der gelegentlich aufgehalten
    // wird (Mittelwert gut, Verzögerungen vorhanden).
    //
    // Bezahlt wird das mit EINEM micros() je Tick. Das verdoppelt den Leerlaufpfad ungefähr - siehe den
    // Kommentar an DataLinkLayer::tick(), wo die Rechnung steht.
    volatile uint32_t _ticks = 0;

    // WIE OFT die Grenze gerissen wurde, nicht nur wie schlimm es einmal war. Ohne diesen Zähler ist der
    // Höchstwert unfalsifizierbar: ein einzelner Ausreißer und ein Dauerzustand sehen identisch aus, und
    // jede Erklärung dazu bleibt eine Vermutung ohne Gegenprobe. Erst beide zusammen sind eine Diagnose -
    // "einmal 12ms" ist ein Ereignis, "300 mal über 1,4ms" ist ein Defekt.
    //
    // Klassifiziert wird im DataLinkLayer, nicht hier: die Schwelle ist eine Protokollkonstante, und diese
    // Klasse kennt keine (dieselbe Trennung wie bei den Fehlerbits des Chips).
    volatile uint32_t _tickDeferrals = 0;

    // DIE ZULETZT GEMESSENE VERZÖGERUNG, nicht die schlimmste je gemessene. Der Höchstwert SÄTTIGT: nach
    // einem einzigen Flash-Schreibvorgang steht dort für immer eine fünfstellige Zahl, und er sagt danach
    // nichts mehr über den aktuellen Zustand. Der letzte Wert wandert mit - wer nach einer verdächtigen
    // Aktion nachsieht, bekommt deren Größenordnung und nicht die Vorgeschichte.
    volatile uint32_t _tickLastDeferredUs = 0;

    // WIE LANGE EIN TICK LÄUFT, im schlechtesten je gemessenen Fall. Das ist die Zahl, die darüber
    // entscheidet, welche IRQ-Priorität dieser Schicht zusteht: wer andere Interrupts verdrängen will,
    // muss belegen können, dass er sie nur kurz aufhält. Ohne die Messung ist jede Prioritätswahl geraten.
    //
    // Die Library selbst ist genügsam - ein Byte je Richtung, im schlimmsten Fall ein memcpy/memcmp über
    // 263 Byte beim Abschluss eines maximal großen Telegramms. Die eine unbekannte Größe darin ist der
    // Quittungs-Callback des Aufrufers: er läuft MIT im Tick (die Entscheidung fällt bei Byte 6, sie lässt
    // sich nicht aufschieben), und was er kostet, bestimmt der Aufrufer. Genau deshalb steht er hier mit
    // drin statt herausgerechnet zu werden.
    volatile uint32_t _tickDurationMaxUs = 0;

    // Der Anteil des AUFRUFERS daran. Der Quittungs-Callback ist der einzige unbegrenzte Teil des Ticks:
    // alles andere ist ein Byte je Richtung und nach oben beschränkt. Ohne diese Zahl lässt sich ein
    // langer Tick nicht zuordnen - man weiss, DASS er lang war, aber nicht, wer ihn lang gemacht hat.
    volatile uint32_t _checkAcknowledgeMaxUs = 0;

    // Wie viele Bytes das Interface im Höchstfall bereithielt, als der Tick es abfragte. Das ist der
    // RÜCKSTAND in seiner ursprünglichen Einheit: 0-1 ist gesund (der Bus liefert höchstens alle 1,354ms
    // ein Byte, der Tick holt alle 500µs eines ab), alles darüber heißt, dass der Tick zwischenzeitlich
    // weg war. Kostenlos erhoben - Receiver::process() fragt available() ohnehin bei jedem Durchlauf.
    volatile uint32_t _rxInterfacePeakBytes = 0;

    // HÖCHSTSTÄNDE DER DREI WARTESCHLANGEN. Sie beantworten die Auslegungsfrage, BEVOR etwas überläuft:
    // ein Überlaufzähler sagt nur, dass es zu spät war, ein Höchststand sagt, wie viel Luft noch ist.
    // Alle drei zählen BYTES - die Sendequeue tat das früher in Telegrammen, seit sie ein Bytepuffer ist,
    // misst sie sich wie die beiden anderen.
    volatile uint32_t _rxQueuePeakBytes = 0;
    volatile uint32_t _txControlQueuePeakBytes = 0;
    volatile uint32_t _txQueuePeakBytes = 0;

    // BUSLAST: gemessen wird im Takt, nicht beim Ablesen. Vorher rechnete getBusLoad() über die Zeit seit
    // dem letzten AUFRUF - und die ist unbegrenzt, weil der von der Konsole kommt. Beim ersten Abruf nach
    // Stunden Laufzeit war das Ergebnis erstens ein Mittel über die gesamte Uptime statt einer aktuellen
    // Last, und zweitens lief die Zwischengröße (Bytes mal 1000) in 32 Bit über. Mit dem festen Fenster
    // sind beide Probleme weg.
    //
    // GEMESSEN WIRD JE SEKUNDE, und was dabei herauskommt, steht bis zur nächsten Messung fest. Es gibt
    // hier bewusst KEINEN gleitenden Mittelwert mehr (Entscheidung des Anwenders): der Wert soll die
    // letzte Sekunde beschreiben und nicht ein Mittel über mehrere.
    //
    // Der Preis ist bekannt und hingenommen: ein einzelnes maximales Telegramm belegt den Bus 356ms, drei
    // davon füllen eine Sekunde fast allein - die Anzeige springt dadurch stärker, als es die Änderung am
    // Bus rechtfertigt. Wer das wieder glätten will, mittelt über mehrere dieser fertigen Werte, statt das
    // Fenster erneut zu verbreitern.
    //
    // WICHTIG dabei: gerechnet wird beim MESSEN, nicht beim Ablesen, und der Bezugspunkt wandert nur im
    // Sekundentakt. Ein Bezugspunkt, der bei jedem Lesen mitzöge, ergäbe Spannen von wenigen Millisekunden
    // - ein einzelnes Telegramm darin sähe wie ein völlig überlasteter Bus aus.
    //
    // Fest und nicht überschreibbar: das beschreibt eine Anzeige, keine Betriebsart.
    static constexpr uint32_t BUS_LOAD_INTERVAL_MS = 1000;

    // Der Bezugspunkt der laufenden Messung: Zählerstände plus Zeitstempel. Gespeichert werden die
    // ZÄHLERSTÄNDE, nicht Differenzen - die Differenz entsteht erst beim Abschluss der Sekunde.
    // Die TELEGRAMMZAHL steht mit drin, weil die Belegung des Busses nicht allein an den Oktetts hängt:
    // vor jedem Telegramm müssen 50 Bitzeiten frei sein (~5,2ms), und diese Pause gehört der Übertragung,
    // nicht der Ruhe. Aus Bytes allein ist sie nicht ableitbar - 90 Oktetts sind ein maximales Telegramm
    // oder zehn kleine, und das ist ein Unterschied von 79ms.
    uint32_t _busLoadRefBytes = 0;
    uint32_t _busLoadRefFrames = 0;
    uint32_t _busLoadRefAt = 0;
    bool _busLoadRefValid = false;

    // Das Ergebnis der zuletzt abgeschlossenen Sekunde. Bis die erste vorbei ist, bleibt beides 0 - das
    // ist ehrlicher als ein aus einer Bruchteilsekunde hochgerechneter Wert.
    uint32_t _busLoadBytesPerSecond = 0;
    uint16_t _busLoadPercent = 0;

  public:
    void reset();

    // --- Empfang ---------------------------------------------------------------------------------------

    void incrementRxBytes(uint32_t increment = 1);
    void incrementRxFrames(uint32_t increment = 1);
    void incrementRxInvalidFrames(uint32_t increment = 1);
    void incrementRxFrameBytes(uint32_t increment = 1);
    void incrementRxControlBytes(uint32_t increment = 1);
    void incrementRxDroppedBytes(uint32_t increment = 1);

    // Aus dem Hauptkontext (loop()), nicht aus dem Tick - der Wiederholungsfilter läuft dort.
    void incrementRxRepeatedFrames(uint32_t increment = 1);

    uint32_t getRxBytes() const;         // jedes vom Interface gelesene Byte
    uint32_t getRxFrames() const;        // gemeldet und in Ordnung
    uint32_t getRxInvalidFrames() const; // gemeldet, aber kaputt

    // Telegramm-Bytes, und die eines Poll-Telegramms zählen mit - beides ist Verkehr auf dem Bus. Ein
    // eigener Poll-Zähler wäre eine Zeile in jeder Statistikausgabe, die praktisch immer null zeigt.
    // Zugleich die Grundlage der Buslast: Steuerbytes kommen von der BCU und nicht vom Bus, sie gehören in
    // die Auslastung der Hostleitung (getTxBytes/getRxBytes), nicht in die des Busses. Und die im Resync
    // verworfenen Bytes lassen sich niemandem zuordnen; sie fehlen damit in der Last, aber sie sind der
    // Ausnahmefall, und ein geschätzter Anteil wäre schlechter als ein sauber definierter Wert.
    uint32_t getRxFrameBytes() const;

    uint32_t getRxControlBytes() const;
    uint32_t getRxDroppedBytes() const; // nie gemeldet: Resync, Moduswechsel, voller RX-Ring

    // Gemeldet, aber als Wiederholung markiert (TP_FRAME_FLAG_FILTERED) - der Aufrufer soll sie sehen und
    // nicht noch einmal verarbeiten.
    uint32_t getRxRepeatedFrames() const;

    // --- Versand ---------------------------------------------------------------------------------------

    void incrementTxBytes(uint32_t increment = 1);
    void incrementTxControlBytes(uint32_t increment = 1);
    void incrementTxFrames(uint32_t increment = 1);
    void incrementTxAcknowledges(uint32_t increment = 1);
    void incrementTxAcknowledgesSuppressed(uint32_t increment = 1);
    void incrementTxConfirmTimeouts(uint32_t increment = 1);

    uint32_t getTxFrames() const;
    uint32_t getTxControlBytes() const;

    // Alles, was je an die Schnittstelle ging - siehe _txBytes.
    uint32_t getTxBytes() const;
    uint32_t getTxAcknowledges() const;

    // Nicht quittiert, weil das Telegramm auf dem Bus schon durch war (Verarbeitung hing hinterher).
    // Kein Fehler, sondern die gewollte Schutzmaßnahme - ein steigender Wert heißt aber: der Tick kommt
    // nicht schnell genug dran.
    uint32_t getTxAcknowledgesSuppressed() const;

    // Zu einem gesendeten Telegramm kam kein L_Data.con, der Wachhund musste die BCU zurücksetzen -
    // siehe _txConfirmTimeouts. Ein steigender Wert zeigt auf den Chip oder die Verkabelung, nicht auf
    // den Bus.
    uint32_t getTxConfirmTimeouts() const;

    // --- Verluste und Verbindung -----------------------------------------------------------------------

    void incrementRxInterfaceOverflows(uint32_t increment = 1);
    void incrementRxQueueOverflows(uint32_t increment = 1);
    void incrementTxControlQueueOverflows(uint32_t increment = 1);

    // Anders als die übrigen Überläufe wird dieser aus dem HAUPTKONTEXT gezählt (pushTransmitQueue()),
    // nicht aus dem Tick - der Zähler hat damit trotzdem genau einen Schreiber.
    void incrementTxQueueOverflows(uint32_t increment = 1);

    // Aus dem Hauptkontext (connectionLost()).
    void incrementConnectionLosses(uint32_t increment = 1);

    void incrementRxResyncs(uint32_t increment = 1);

    // Aus dem Hauptkontext (handleControlEntry()). Welches Bit welchen Zähler trifft, entscheidet dort
    // der DataLinkLayer - diese Klasse kennt keine Protokollkonstanten und soll auch keine kennen.
    void incrementChipSlaveCollisions(uint32_t increment = 1);
    void incrementChipReceiveErrors(uint32_t increment = 1);
    void incrementChipTransmitErrors(uint32_t increment = 1);
    void incrementChipProtocolErrors(uint32_t increment = 1);
    void incrementChipTemperatureWarnings(uint32_t increment = 1);

    // --- Takt ------------------------------------------------------------------------------------------

    // Zählt einen Tick und stempelt beim ersten den Bezugspunkt der mittleren Rate.
    void recordTick();

    // Ein Tickabstand hat die Busgrenze gerissen. Getrennt von recordTick(), weil die Schwelle eine
    // Protokollkonstante ist und diese Klasse keine kennt - siehe _tickDeferrals.
    void recordTickDeferred(uint32_t deferredUs);

    // Höchststand wie die Warteschlangen-Peaks, siehe _rxInterfacePeakBytes.
    void updateRxInterfacePeakBytes(uint32_t pending);

    // Wie oft tick() gelaufen ist. NUR ALS DIFFERENZ ZWEIER STÄNDE zu benutzen - der Zähler läuft bei
    // 500µs nach rund 25 Tagen um, und eine Rechnung gegen die Gesamtlaufzeit bräche damit. Genau so
    // benutzt ihn checkTickRate(), und dort stimmt die vorzeichenlose Differenz auch über den Umlauf.
    uint32_t getTicks() const;



    // Wie oft der Tick aufgehalten wurde - siehe _tickDeferrals. Erst zusammen mit der Dauer ist das eine
    // Aussage: einmal 12ms ist ein Ereignis, hundertmal 2ms ein Defekt. Die Differenz zwischen zwei
    // Abfragen sagt zudem, WANN es passiert - dafür genügt es, den Wert vor und nach einer verdächtigen
    // Aktion abzulesen.
    uint32_t getTickDeferrals() const;

    // Die Dauer der ZULETZT gemessenen Verzögerung - siehe _tickLastDeferredUs. 0, solange nie eine
    // auftrat.
    uint32_t getTickLastDeferredUs() const;

    // Höchststand wie die übrigen Peaks - siehe _tickDurationMaxUs.
    void updateTickDurationMaxUs(uint32_t durationUs);

    // Die längste je gemessene Laufzeit eines Ticks, einschließlich des Quittungs-Callbacks. Gemessen
    // wird nur der VOLLE Durchlauf; die frühen Abbrüche (nicht verbunden, kein Interface) sind ein
    // paar Vergleiche und würden das Bild nur beschönigen.
    uint32_t getTickDurationMaxUs() const;

    // Höchststand des Quittungs-Callbacks - siehe _checkAcknowledgeMaxUs. Gegen getTickDurationMaxUs()
    // zu lesen: liegen beide dicht beieinander, gehört die Laufzeit dem Aufrufer, nicht dieser Schicht.
    void updateCheckAcknowledgeMaxUs(uint32_t durationUs);
    uint32_t getCheckAcknowledgeMaxUs() const;


    // Der größte je beobachtete Rückstand im Interface, in Bytes - siehe _rxInterfacePeakBytes.
    uint32_t getRxInterfacePeakBytes() const;

    // Kein Inkrement, sondern ein Höchststand: der Wert wird nur größer. Gerufen wird das dort, wo der
    // Füllstand ohnehin schon ausgerechnet ist, es kommt also nur ein Vergleich dazu.
    void updateRxQueuePeakBytes(uint32_t used);
    void updateTxControlQueuePeakBytes(uint32_t used);
    void updateTxQueuePeakBytes(uint32_t used);

    uint32_t getRxInterfaceOverflows() const;
    uint32_t getRxQueueOverflows() const;
    uint32_t getTxControlQueueOverflows() const;
    uint32_t getTxQueueOverflows() const;

    // Wie oft die Verbindung zur BCU als verloren galt und ein Reconnect anlief. Auf einem gesunden Gerät
    // steht der Wert auf 0; alles darüber heißt, dass die BCU zeitweise nicht geantwortet hat.
    uint32_t getConnectionLosses() const;

    // Wie oft die Maschine in den Resync gegangen ist - siehe _rxResyncs. Zusammen mit
    // getRxDroppedBytes() ergibt sich daraus die durchschnittliche Länge einer Störung.
    uint32_t getRxResyncs() const;

    // Die Fehlermeldungen des Chips, einzeln - siehe die Felder. Die Kürzel sind dieselben wie in der
    // Konsolenzeile, die showStateErrors() ausgibt: SC, RE, TE, PE, TW.
    uint32_t getChipSlaveCollisions() const;
    uint32_t getChipReceiveErrors() const;
    uint32_t getChipTransmitErrors() const;
    uint32_t getChipProtocolErrors() const;
    uint32_t getChipTemperatureWarnings() const;

    // Höchststände, siehe die Felder. Zu lesen gegen TPUART_RX_QUEUE_SIZE (Receiver.h),
    // TPUART_CTRL_QUEUE_SIZE (Transmitter.h) und TPUART_TX_BUFFER_SIZE (TransmitQueue.h).
    uint32_t getRxQueuePeakBytes() const;
    uint32_t getTxControlQueuePeakBytes() const;
    uint32_t getTxQueuePeakBytes() const;

    // --- Buslast ---------------------------------------------------------------------------------------

    // AUS DEM HAUPTLOOP, bei jedem Durchlauf. Begrenzt sich selbst auf eine Momentaufnahme je Takt -
    // der Aufrufer muss also nicht takten. Bewusst nicht aus dem Tick: der Leerlaufpfad dort soll frei von
    // millis() bleiben, und für einen Sekundentakt ist der Hauptloop schnell genug. Bleibt er einmal länger
    // weg, verzerrt das nichts: gerechnet wird über die Zeit zwischen den Einträgen, und die stimmt dann
    // eben nicht mehr auf die Sekunde.
    void sampleBusLoad();

    // Bytes pro Sekunde der zuletzt abgeschlossenen Sekunde. Einen gleitenden Mittelwert gibt es hier
    // NICHT mehr - der Wert beschreibt genau ein Messintervall (BUS_LOAD_INTERVAL_MS).
    // Das Ablesen verändert nichts und rechnet auch nichts - es liefert das Ergebnis der zuletzt
    // abgeschlossenen Sekunde, beliebig oft. Solange die erste noch läuft: 0.
    uint32_t getBusLoad() const;

    // DIE PHYSIK DES BUSSES als Bezugsgröße der Prozentanzeige: KNX TP1 läuft immer mit 9600 Baud, und
    // ein Zeichen belegt 13 Bitzeiten (Start, 8 Daten, Parität, Stop, dazu 2 Bit Abstand) - 1354µs je
    // Oktett, also 738 Oktetts je Sekunde.
    //
    // Sie steht HIER und nicht in Types.h, weil sie eine Anzeige beschreibt und nicht das Protokoll -
    // dieselbe Trennung, aus der schon die Zuordnung "Fehlerbit -> Zähler" im DataLinkLayer liegt und
    // nicht in dieser Klasse: die Zählerklasse kennt keine Protokollkonstanten.
    //
    // TPUART_TICK_DEFERRED_US in Timer.h trägt dieselbe Zahl aus demselben Grund, bleibt aber ein
    // eigenes, überschreibbares Makro: das ist eine Schwelle, die jemand verstellen können soll, und ein
    // -D darauf darf nicht stillschweigend den Nenner dieser Anzeige mitverschieben.
    static constexpr uint32_t BUS_OCTET_TIME_US = 1354;
    static constexpr uint32_t BUS_MAX_BYTES_PER_SECOND = 1000000UL / BUS_OCTET_TIME_US;

    // Die Pause VOR jedem Telegramm: 50 Bitzeiten bei 9600 Baud. Sie ist keine Ruhe, sondern Teil der
    // Übertragung - in dieser Zeit kann niemand senden.
    static constexpr uint32_t BUS_FRAME_GAP_US = 5208;

    // DER QUITTUNGS-SLOT, und er zählt UNBEDINGT mit - das ist der Kern: er ist fest reserviert, nicht
    // bedingt. Ob in dieser Zeit jemand quittiert oder nicht, ändert nichts daran, dass sie vergeht und
    // niemand sonst senden kann. Deshalb braucht es hier auch keine Kenntnis darüber, welche Telegramme
    // quittiert wurden - eine Kenntnis, die der Host außerhalb des Busmonitors gar nicht hat.
    //
    // 15 Bitzeiten Abstand plus das Quittungsoktett (11 Bit) = 26 Bitzeiten bei 9600 Baud. Bleibt die
    // Quittung aus, wartet der Sender stattdessen die dokumentierten 30 Bitzeiten ab (NCN5130 S. 38) -
    // 417µs mehr, was keine Fallunterscheidung wert ist.
    static constexpr uint32_t BUS_ACK_SLOT_US = 2708;

    // WIE VOLL DER BUS IST, als Anteil der Zeit, in der er belegt war. Anders als getBusLoad() ist das
    // keine Bytezahl in anderer Einheit, sondern eine Zeitrechnung aus Oktetts UND Telegrammzahl:
    //
    //     belegt = Oktetts * BUS_OCTET_TIME_US
    //            + Telegramme * (BUS_FRAME_GAP_US + BUS_ACK_SLOT_US)
    //
    // Erst dadurch heißt 100% wirklich "hier passt kein Telegramm mehr hinein" und ist auch erreichbar.
    // Über die Oktetts allein ginge das nicht: 90 Oktetts sind ein maximales Telegramm oder zehn kleine,
    // und die zehn belegen den Bus 79ms länger.
    //
    // BEIDE ZUSCHLÄGE SIND FESTE SLOTS, keine Schätzung - das ist der Grund, warum diese Rechnung ohne
    // Kenntnis des Verkehrs auskommt. Weder muss bekannt sein, wer quittiert hat, noch ob überhaupt
    // jemand quittiert hat: die Zeit ist so oder so reserviert.
    //
    // ÜBER 100 WIRD BEWUSST NICHT GEDECKELT (Entscheidung des Anwenders), und deshalb ist der Rückgabetyp
    // 16 Bit: ein uint8_t liefe bei 256% still über und meldete 0. Physikalisch kann der Bus nicht voller
    // als voll sein, ein Wert über 100 ist also ein BEFUND und soll sichtbar sein.
    //
    // Damit das trägt, werden die Bytes JE BYTE gezählt (siehe Receiver::processFrameByte()) und nicht am
    // Sequenzende in einem Rutsch. Sonst trüge ein laufendes Telegramm 0 bei und brächte beim Abschluss
    // seine ganze Busbelegung mit - bei einem maximalen Telegramm 356ms, die im falschen Fenster landen,
    // also über ein Drittel der Messsekunde. Genau dieser Fehler hätte ein "über 100" erzeugt, das nichts
    // bedeutet, und damit die Aussage der Zahl zerstört.
    //
    // Was als Rest bleibt, ist der Zuschlag JE TELEGRAMM: der wird weiterhin erst beim Abschluss fällig,
    // weil vorher nicht feststeht, ob das Telegramm heil ist. Das sind 7916µs für ein Telegramm an der
    // Fenstergrenze, bei 1s also 0,8% - vernachlässigbar.
    uint16_t getBusLoadPercent() const;

    // --- KOMPAT: Namen der alten Library ----------------------------------------------------------------
    //
    // AUSSCHLIESSLICH Namen, die es in v1 schon gab und die ein Verbraucher heute aufruft (der bcu-Befehl
    // in OGM-Common). Neue Namen bekommen hier NICHTS - solange 2.0 in Arbeit ist, kann sich auf sie noch
    // niemand stützen, sie dürfen also direkt heißen, wie sie heißen sollen.
    //
    // Nicht nachgerüstet werden die drei ungenutzten Dubletten aus v1 (getRxOverflowInterface,
    // getRxOverflowFrameBuffer, getRxOverflowSearchBuffer): dort gab es jedes dieser Ereignisse unter zwei
    // Namen, und die Verbraucher rufen durchweg nur die eine Schreibweise.
    //
    // Alle hier können weg, sobald OGM-Common umgestellt ist.
    uint32_t getRxReceivedBytes() const;        // -> getRxBytes()
    uint32_t getRxRepetitions() const;          // -> getRxRepeatedFrames()
    uint32_t getRxDiscardedBytes() const;       // -> getRxDroppedBytes()
    uint32_t getRxBusBytes() const;             // -> getRxFrameBytes()
    uint32_t getRxUartOverflow() const;         // -> getRxInterfaceOverflows()
    uint32_t getRxFrameBufferOverflow() const;  // -> getRxQueueOverflows()
    uint32_t getTxOverflowFrameBuffer() const;  // -> getTxQueueOverflows()

    // PLATZHALTER: den SearchBuffer gibt es nicht mehr, hier kann nichts überlaufen. Bleibt bei 0 - jeder
    // Ersatzwert wäre eine Falschaussage, denn es gibt in dieser Library nichts Vergleichbares.
    uint32_t getRxSearchBufferOverflow() const;
};

} // namespace TPUart
