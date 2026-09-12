#pragma once
#include <stddef.h>
#include <stdint.h>

#include <functional>
#include <vector>

#include "TPUart/Frame.h"
#include "TPUart/Interface/Abstract.h"
#include "TPUart/Receiver.h"
#include "TPUart/RepetitionFilter.h"
#include "TPUart/Statistics.h"
#include "TPUart/SystemState.h"
#include "TPUart/Timer.h"
#include "TPUart/Transmitter.h"
#include "TPUart/Types.h"

namespace TPUart
{

// Verbindungsüberwachung. Beides wie in der alten Library: dort wurde jede Sekunde der Status abgefragt
// (processRequestState) und nach 5s ohne ein einziges empfangenes Byte auf BCU_DISCONNECTED umgestellt.
//
// Die Statusabfrage ist der einzige Weg, eine stumme BCU von einem stillen Bus zu unterscheiden: an einem
// ruhigen Bus kommt regulär minutenlang nichts, und ohne eigene Frage sähe ein Ausfall genauso aus. Auf die
// Frage MUSS eine Antwort kommen (U_State.ind), und die zählt als Lebenszeichen wie jedes andere Byte.
#ifndef TPUART_STATE_INTERVAL_MS
#define TPUART_STATE_INTERVAL_MS 1000
#endif

#ifndef TPUART_CONNECTION_TIMEOUT_MS
#define TPUART_CONNECTION_TIMEOUT_MS 5000
#endif

// Wie lange auf die Antwort eines einzelnen Baudraten-Kandidaten gewartet wird (wie in der alten Library).
constexpr uint32_t TPUART_DETECT_RESPONSE_TIMEOUT_MS = 50;

// Wie lange gewartet wird, bevor die komplette Kandidatenliste erneut von vorn probiert wird, nachdem
// keiner der Kandidaten geantwortet hat. Nötig, weil die BCU erst antwortet, sobald am Bus die
// Busspannung anliegt - das kann nach dem eigenen Boot noch beliebig lange dauern.
#ifndef TPUART_DETECT_RETRY_INTERVAL_MS
#define TPUART_DETECT_RETRY_INTERVAL_MS 1000
#endif

// Wie lange der Busy-Modus höchstens anhält, bevor die Library ihn selbst zurücknimmt. 700ms wie in der
// alten Library, und der Wert ist nicht frei gewählt: der TPUART2 verlässt den Modus nach dieser Zeit in
// HARDWARE, der NCN512x nicht. Das Nachziehen macht beide Chips gleich - und verhindert auf dem NCN, dass
// ein einmal gesetzter Busy-Modus für immer stehenbleibt.
#ifndef TPUART_BUSY_MODE_MS
#define TPUART_BUSY_MODE_MS 700
#endif

// Der Datalink Layer selbst. Er hält, was BEIDE Hälften angeht, und die Hälften selbst:
//
//   Receiver    - alles, was einläuft (siehe Receiver.h), inklusive der Acknowledge-Entscheidung
//   Transmitter - alles, was hinausgeht (siehe Transmitter.h): Telegramme, Steuercodes, Acknowledge
//
// Aufgeteilt wie in der alten Library, und wie dort halten beide Hälften eine Referenz hierauf: Interface,
// Statistik, Chip- und Verbindungszustand liegen an einer Stelle, statt in zwei Hälften gedoppelt zu werden.
// Sie sind deshalb `friend` - eine Sichtbarkeit, die bewusst eng gehalten ist. Der DataLinkLayer selbst
// braucht keine Gegen-Freundschaft, er kommt mit den öffentlichen Methoden der beiden aus.
//
// Was hier bleibt:
//   - Verbindungsaufnahme samt Baudratenerkennung (begin(), searchBaudRate(), reconnect())
//   - Verbindungsüberwachung (processConnectionState()) und der abgeleitete BcuState
//   - Chip-Zustand: Busmonitor, Auto-Quittung, Chiptyp
//   - Konfiguration, die die BCU bei jedem Reset vergisst (Adresse, Wiederholungszähler)
//   - die semantischen Steuerbefehle, die daraus Steuercodes machen
//   - Auswertung und Ausgabe der Steuerbyte-Sequenzen, die der Receiver hereinreicht
//
// ZWEI KONTEXTE, und die Trennlinie ist der Ringpuffer im Receiver:
//
//   tick()  - die zeitkritische Seite. Ein Byte je Richtung pro Aufruf, blockiert nie und hält sich
//             nirgends auf, weil sie aus einem Timer läuft (Zielintervall 500µs, siehe Timer.h;
//             der Bus liefert höchstens alle 1,354ms ein Byte - 9600 Baud, 13 Bitzeiten je TP1-Zeichen).
//             "Ein Byte" heißt auf der Sendeseite: ein Byte DES TELEGRAMMS - die bis zu zwei zugehörigen
//             Positionsbytes gehen im selben Tick mit raus. Reserviert wird dafür, was wirklich anfällt:
//             2 Bytes, oder 3 wenn ein Offsetbyte fällig ist (siehe Transmitter::process()).
//             TPUART_TX_ATOMIC_BYTES ist nur noch die Obergrenze, aus der TPUART_TX_INTERFACE_BUFFER folgt.
//             Jeder Tick ist damit ein Durchlauf mit fester Obergrenze an Arbeit.
//   loop()  - die gemütliche Seite, aus dem Hauptloop aufgerufen. Leert den Ringpuffer: Telegramme gehen
//             per Callback nach außen, Steuerbytes verarbeitet der Datalink Layer selbst.
//
// Aus der tick()-Hälfte darf deshalb NICHTS nach außen dringen, was der Aufrufer ausgeben könnte: aus
// einem Interrupt heraus sind weder Serial-Ausgaben noch Speicheranforderungen erlaubt. Eine frühere
// Diagnose-API (lastByte()/ackSent() für eine Byte-für-Byte-Ausgabe) ist genau daran gescheitert und
// wieder entfernt worden - alles Beobachtbare kommt jetzt ausschließlich über den Frame-Callback.
//
// Der Leerlaufpfad von tick() ist der mit Abstand am häufigsten durchlaufene Code hier und soll so kurz
// wie möglich bleiben. Die Reihenfolge der Abbruchbedingungen ist deshalb nicht beliebig: erst die reinen
// Flag-Vergleiche (_initialized, _connected, TxState), danach erst alles, was das Interface anfassen muss.
// Aktuell kostet ein Leerlauf-Tick ZWEI MMIO-Zugriffe - micros() für die Taktmessung und den DMA-Zähler in
// available() - plus rund ein Dutzend Vergleiche. Das micros() in checkPause() kommt nur dazu, solange eine
// Sequenz offen ist; zwischen zwei Telegrammen ist sie das nicht.
class DataLinkLayer
{
    friend class Receiver;
    friend class Transmitter;

  public:
    // Das Frame gehört dem Aufruf: es besitzt seine Daten (siehe Frame.h) und lebt auf dem Stack des
    // Verbrauchers. Nicht const, damit der Aufrufer eigene Flags setzen kann (setFiltered()).
    using FrameCallback = std::function<void(Frame &frame)>;

    // Liefert die Acknowledge-Entscheidung für ein einlaufendes Frame. Nicht gesetzt bedeutet: es wird
    // GAR NICHT quittiert - wie ACK_None in der alten Library, wenn dort kein Callback registriert war.
    using AcknowledgeCallback = std::function<AckType(uint16_t destination, bool isGroupAddress)>;

    // Meldungen des Datalink Layers - wie in der alten Library, inklusive des error-Flags. Der Text gilt
    // nur für die Dauer des Aufrufs. Gerufen wird ausschließlich aus loop().
    using MessageCallback = std::function<void(const char *message, bool error)>;

  private:
    // Was sich beide Hälften teilen. Sie greifen darauf über ihre _dll-Referenz zu (dafür sind sie friend),
    // halten also keine eigenen Referenzen - Begründung in Receiver.h.
    // Zeiger und nicht Referenz, weil die alte Library das Interface erst in begin() bekam - der Aufrufer
    // legt den DataLinkLayer an, bevor die Plattform ihr Interface erzeugt hat (siehe KOMPAT-begin()).
    // Ohne Interface tut tick()/loop() nichts.
    Interface::Abstract *_interface = nullptr;
    Statistics _statistics;
    SystemState _systemState;
    RepetitionFilter _repetitionFilter;

    // Zeitstempel des vorigen tick(), allein für die Taktmessung (siehe Statistics::recordTick). Gehört dem
    // Tick, wird von niemandem sonst angefasst. 0 heißt "noch kein Vorgänger" - begin() setzt ihn zurück,
    // damit die Pause zwischen zwei Betriebsphasen nicht als Aussetzer erscheint.
    uint32_t _tickLastUs = 0;

    // Überwachung der ERREICHTEN Taktrate (siehe checkTickRate). Reiner Hauptkontext - gelesen wird nur
    // der Tickzähler der Statistik, geschrieben hier. _tickRateCheckedAt == 0 heißt "noch kein Bezugspunkt".
    uint32_t _tickRateCheckedAt = 0;
    uint32_t _tickRateLastTicks = 0;
    bool _tickRateReported = false;

    // Seit wann der Busy-Modus läuft, 0 = aus. Siehe checkBusyMode(). Nur Hauptkontext.
    uint32_t _busyModeSince = 0;

    // Der Tick meldet hierüber, dass er ein U_Ackn.req abgesetzt hat - damit ist der Busy-Modus im Chip
    // beendet (S. 35), und _busyModeSince muss nachgezogen werden. Über ein eigenes Flag statt direkt,
    // damit _busyModeSince seinen einzigen Schreiber behält: der Tick setzt hier, loop() räumt dort auf.
    volatile bool _busyModeCancelled = false;

    // VERLUSTMELDUNGEN. Sie liegen hier und nicht in den Hälften, weil sie dort Zustand wären, der nichts
    // steuert: an der Verarbeitung ändert sich durch sie nichts, sie sind reine Auskunft für die Schicht
    // darüber. Gesetzt werden sie aus dem Tick (report*), gelesen und dabei gelöscht im Hauptkontext.
    volatile bool _interfaceOverflow = false;
    volatile bool _rxQueueOverflow = false;
    volatile bool _ctrlQueueOverflow = false;

    // Fehlerbits aus U_State.ind, aufsummiert bis sie jemand abholt. Wie in der alten Library werden sie
    // verodert und nicht überschrieben: der Chip meldet ein Ereignis nur EINMAL, ein späteres, fehlerfreies
    // U_State.ind würde einen zwischenzeitlichen Fehler sonst wieder verschwinden lassen.
    uint8_t _stateErrors = 0;

    // Verbindungsaufnahme (siehe begin() und den Block bei searchBaudRate()). Bis _connected true wird,
    // laufen die beiden Hälften nicht - bei unbestätigter Baudrate sind eingehende Bytes nicht als
    // Protokoll interpretierbar.
    //
    // _connected hat zwei Schreiber, und das ist ein Staffelstab, kein geteilter Zustand: der Hauptkontext
    // setzt true (nach der Suche) bzw. false (bei Zeitüberschreitung), der Tick setzt true (nach dem
    // Wiederaufbau). Jede Seite schreibt nur den Übergang, den sie selbst festgestellt hat, und nur wenn
    // sie den Gegenwert gelesen hat - die beiden können sich deshalb nicht überschreiben.
    bool _initialized = false;
    BcuType _bcuType = BcuType::Ncn5120;
    volatile bool _connected = false;
    uint32_t _connectedBaudRate = 0;

    // Verbindungsüberwachung. _lastReceivedAt schreibt der Tick (jedes empfangene Byte, im Receiver),
    // gelesen wird im Hauptkontext; die beiden anderen gehören dem Hauptkontext allein.
    volatile uint32_t _lastReceivedAt = 0;
    uint32_t _lastStateRequestAt = 0;

    // Der Besitzwechsel am Interface (siehe searchBaudRate). Geschrieben genau einmal, vom Hauptkontext;
    // gelesen bei jedem tick(). Danach unveränderlich.
    volatile bool _everConnected = false;

    bool _connectReported = false; // damit die Verbindungsmeldung aus loop() kommt, nicht aus dem Tick

    // Konfiguration, die die BCU nach jedem Reset vergisst ("After reset the address evaluation is
    // deactivated again", Siemens S. 23). Gehalten wird sie deshalb hier und nach jedem Reset erneut
    // abgesetzt. Die Zähler stehen auf dem Wert, den beide Chips nach einem Reset selbst haben.
    uint16_t _ownAddress = 0;
    uint8_t _repetitionsNack = 3;
    uint8_t _repetitionsBusy = 3;

    // Die Spannungsregler des NCN (ACR0). Drei Zustände, weil "nie angefasst" etwas anderes ist als "an":
    // nur wenn der Aufrufer sich geäußert hat, wird nach einem Reset überhaupt etwas geschrieben. Siehe
    // queuePowerControl() - der Reset-Wert des Registers ist "alles an", ein powerControl(false) wäre also
    // nach jedem Reset still wieder aufgehoben.
    enum class PowerControl : uint8_t
    {
        Unset,
        On,
        Off,
    };
    PowerControl _powerControl = PowerControl::Unset;

    // Der Chip quittiert selbst. Gelesen wird das im Tick (Receiver::sendAcknowledge), geschrieben von
    // BEIDEN Seiten: der Hauptkontext setzt es beim Absetzen der Adresse (applyConfiguration - das ist der
    // Vorgang, der die Auto-Quittung aktiviert), der Tick löscht es beim Reset und korrigiert es aus dem
    // U_Configure.ind. Zwei Schreiber sind hier tragbar, weil jede Uneinigkeit von der Epoche wieder
    // aufgelöst wird: ein Reset erhöht sie, der Hauptkontext setzt die Adresse erneut ab und damit auch
    // dieses Flag.
    volatile bool _autoAcknowledge = false;

    // "Die Konfiguration muss neu abgesetzt werden": der Tick zählt hoch (bei jedem Reset), der
    // Hauptkontext zieht nach. Ein Zähler statt eines Flags, damit jede Seite genau einen Schreiber hat.
    volatile uint32_t _configEpoch = 0;
    uint32_t _configAppliedEpoch = 0;

    // Der Chip hat sich zuletzt in einem anderen Zustand als NORMAL gemeldet - siehe checkChipRestart().
    // Nur Hauptkontext.
    bool _chipOutOfNormal = false;

    bool _detectAwaitingResponse = false;
    uint8_t _detectCandidateIndex = 0;
    uint32_t _detectRequestSentAt = 0;
    uint32_t _detectNextAttemptAt = 0;

    // Der tatsächliche Chip-Zustand - im Busmonitor darf nicht geackt und nicht gesendet werden. Gesetzt
    // wird er ausschließlich aus controlByteSent(), sobald das zugehörige Steuerbyte den Chip erreicht hat,
    // und dort aus dem gesendeten Code abgeleitet. Ein separates "Wunsch"-Feld gibt es bewusst NICHT: es
    // wäre Nutzlast und müsste damit vor der Besitzübergabe geschrieben werden - eine
    // Reihenfolgeabhängigkeit, die schon einmal falsch war (Chip im Busmonitor, Library wusste es nicht).
    volatile bool _busMonitor = false;

    // ALLE CALLBACKS LIEGEN HIER, nicht in den Hälften - wie in der alten Library
    // (_callbacksReceivedFrame, _callbackCheckAcknowledge, _callbackMessage). Der DataLinkLayer ist die
    // Schnittstelle nach außen, bei ihm werden sie registriert, also ruft er sie auch. Der Receiver baut
    // das Frame und fragt hier nach (deliverFrame/checkAcknowledge).
    // MEHRERE Frame-Callbacks, wie in der alten Library (_callbacksReceivedFrame): der knx-Stack
    // registriert seinen Empfangspfad, OGM-Common zusätzlich eine Debug-Ausgabe. Mit nur einem Platz
    // hätte die zweite Registrierung die erste still ersetzt - und damit den Empfang abgeschaltet.
    std::vector<FrameCallback> _frameCallbacks;
    AcknowledgeCallback _acknowledgeCallback;
    MessageCallback _messageCallback;

    // Die beiden Hälften. Nach den geteilten Feldern deklariert, siehe oben.
    Transmitter _transmitter;
    Receiver _receiver;

    // --- Verbindungsaufnahme -----------------------------------------------------------------------
    //
    // WER DAS INTERFACE ANFASSEN DARF, ist die eine Regel, um die sich das hier dreht, und sie ist an
    // _everConnected festgemacht:
    //
    //   solange !_everConnected  - der HAUPTKONTEXT sucht die Baudrate (searchBaudRate() aus loop()).
    //                              tick() kehrt vorher um und rührt das Interface nicht an.
    //   ab _everConnected        - der TICK hat es, für immer. Auch nach einem Verbindungsverlust:
    //                              reconnect() läuft im Tick.
    //
    // Der Übergang findet GENAU EINMAL statt und wird vom Hauptkontext veröffentlicht (_everConnected vor
    // _connected). Danach fasst der Hauptkontext das Interface nie wieder an - deshalb braucht es keine
    // Rückgabe und keine Sperre.
    //
    // Möglich ist das, weil die beiden Fälle unterschiedlich viel tun müssen: die SUCHE konfiguriert das
    // Interface pro Kandidat neu (end()/begin()), und genau das gehört nicht in einen Interrupt - auf dem
    // ESP32 alloziert uart_driver_install() vom Heap, ein ArduinoSerial reicht an eine beliebige fremde
    // begin()-Implementierung weiter. Der WIEDERAUFBAU dagegen braucht davon nichts: die Baudrate steht
    // längst fest und ändert sich nicht mehr (siehe advanceDetectCandidate), es geht nur noch um dasselbe
    // U_Reset.req und dieselbe Antwort - Lesen und Schreiben auf einem offenen Interface, also genau das,
    // was der Tick ohnehin tut.
    //
    // Nebeneffekt, der vorher fehlte: der Wiederaufbau verwirft nicht mehr grundlos den Empfangspuffer und
    // initialisiert keine Hardware neu, die längst richtig konfiguriert ist.
    enum class DetectResult : uint8_t
    {
        Pending,   // noch keine Entscheidung - weiter warten
        Connected, // die BCU hat mit U_Reset.ind geantwortet
        Failed,    // falsches Byte oder Frist abgelaufen
    };

    // Nur aus loop(), und nur bis zur ersten Verbindung. Konfiguriert das Interface pro Kandidat neu.
    void searchBaudRate();

    // Nur aus tick(), nach einem Verbindungsverlust. Fasst das Interface nicht neu an.
    void reconnect();

    // Gemeinsamer Teil beider Wege: höchstens ein Byte ansehen, sonst die Frist prüfen.
    DetectResult pollDetectResponse();

    // Die Verbindung steht.
    void connectDetected();

    // Nächster Baudraten-Kandidat (nur aus der Suche).
    void advanceDetectCandidate();

    // Aus loop(): fragt regelmäßig den Status ab und erkennt daran den Verbindungsverlust.
    void processConnectionState();
    void connectionLost();

    // Setzt Adresse und Wiederholungszähler an die BCU ab. Nur aus dem Hauptkontext (queueControl).
    // Liefert false, wenn dabei etwas nicht in die Steuer-Warteschlange gepasst hat - dann bleibt die
    // Epoche offen und der nächste loop() versucht es erneut, statt die Konfiguration stillschweigend
    // zu verlieren.
    bool applyConfiguration();

    // Sorgt dafür, dass applyConfiguration() erneut läuft - nach einem misslungenen Versuch, oder wenn am
    // gemeldeten Chip-Zustand ein Reset erkennbar wird, den wir nicht mitbekommen haben.
    void markConfigurationPending();

    // Setzt ACR0 ab, sofern der Aufrufer die Regler je angefasst hat. Teil der Konfiguration, weil ein
    // Reset das Register auf "alles an" zurückstellt.
    bool queuePowerControl();

    // Erkennt am gemeldeten Chip-Zustand, dass sich die BCU zurückgesetzt hat - auch ohne die U_Reset.ind
    // gesehen zu haben. Aus loop(), nach jedem U_SystemStat.ind.
    void checkChipRestart();

    // --- Rückmeldungen der beiden Hälften ----------------------------------------------------------
    //
    // Der Chip-Zustand liegt hier, geändert wird er aber durch das, was die Hälften tatsächlich senden und
    // empfangen. Diese drei Haken sind der Weg dafür - die Hälften deuten die Bytes, der Datalink Layer
    // zieht die Folgerungen.

    // Ein Steuercode ist tatsächlich abgesetzt worden (aus dem Tick, Transmitter). Erst jetzt gilt der Chip
    // als umgeschaltet.
    void controlByteSent(uint8_t code);

    // Eine U_Reset.ind ist eingetroffen (aus dem Tick, Receiver) - von wem auch immer ausgelöst.
    void resetIndication();

    // Ein U_Configure.ind ist eingetroffen (aus dem Tick, Receiver): der Chip meldet seine Betriebsarten.
    void configureIndication(uint8_t value);

    // Etwas ist verlorengegangen (aus dem Tick, beide Hälften). Setzt den Einmal-Merker UND zählt - die
    // beiden gehören zusammen und standen vorher an jeder Fundstelle doppelt nebeneinander.
    void reportInterfaceOverflow();
    void reportRxQueueOverflow();
    void reportControlOverflow();

    // Ein fertiges Telegramm aus dem Ringpuffer (aus loop(), Receiver) - geht an den Frame-Callback.
    void deliverFrame(Frame &frame);

    // Die Acknowledge-Entscheidung zu einem gerade EINLAUFENDEN Telegramm (aus dem TICK, Receiver).
    AckType checkAcknowledge(uint16_t destination, bool isGroupAddress);

    // Eine fertige Steuerbyte-Sequenz aus dem Ringpuffer (aus loop(), Receiver). Steuerbytes gehen bewusst
    // NICHT nach außen - ihre Deutung ist Sache des Datalink Layers, der Aufrufer bekommt davon nur die
    // Meldungen zu sehen.
    void handleControlEntry(const uint8_t *data, size_t length);

    // Meldungen an den registrierten Ausgabekanal, Signaturen wie in der alten Library. Nur aus loop()
    // aufrufen: sie formatieren in einen Stack-Puffer und rufen fremden Code, beides gehört nicht in tick().
    void printMessage(const char *format, ...) __attribute__((format(printf, 2, 3)));
    void printError(const char *format, ...) __attribute__((format(printf, 2, 3)));

    // Nimmt den Busy-Modus nach TPUART_BUSY_MODE_MS von selbst zurück - siehe dort und die Implementierung.
    void checkBusyMode();

    // Meldet einen Antrieb, der die Untergrenze des Busses reißt - siehe die Implementierung. Aus loop(),
    // begrenzt sich selbst auf ein Fenster von TPUART_TICK_RATE_WINDOW_MS.
    void checkTickRate();

    // Aus dem Tick: ein U_Ackn.req ist rausgegangen. Siehe _busyModeCancelled.
    void reportBusyModeCancelled();

    // Zeigt Änderungen am System-Status und aufgelaufene Fehlerbits an - aus loop() heraus, wie die
    // showSystemState()/showStateError()-Paare der alten Library.
    void showSystemState();
    void showStateErrors();

  public:
    explicit DataLinkLayer(Interface::Abstract &interface);

    // KOMPAT: die alte Library wurde ohne Interface angelegt und bekam es in begin(). Bis dahin tut
    // tick()/loop() nichts.
    DataLinkLayer();

    // TRÄGT SICH BEIM ZENTRALEN TIMER AUS, und das ist keine Höflichkeit: der Timer hält einen ZEIGER auf
    // diese Instanz, und der überlebte sie sonst - der nächste Tick liefe in freigegebenen Speicher, aus
    // einem Interrupt heraus. In einem echten Gerät lebt ein DataLinkLayer so lange wie das Gerät selbst,
    // die Testvorrichtung legt aber für jeden Fall einen neuen an, und einem Aufrufer ist es ebenfalls
    // erlaubt. end() allein genügt dafür nicht - es muss niemand end() rufen, bevor er das Objekt fallen
    // lässt.
    ~DataLinkLayer();

    // Startet die Verbindungsaufnahme: probiert die für bcuType in Frage kommenden Baudraten durch (siehe
    // BcuType), bis die BCU mit U_Reset.ind auf ein von uns gesendetes U_Reset.req antwortet - das bestätigt
    // Baudrate UND Reset in einem Schritt. NICHT BLOCKIEREND: begin() merkt sich nur den Chiptyp, gesucht
    // wird aus loop() heraus (searchBaudRate) - dort und nur dort darf das Interface neu konfiguriert
    // werden. Der Aufrufer muss also loop() rufen, damit die Verbindung zustande kommt; tick() allein
    // genügt dafür nicht.
    //
    // GESUCHT wird nur EINMAL. Die Baudrate ist eine Hardware-Eigenschaft der BCU und ändert sich im
    // Betrieb nicht; nach einem Verbindungsverlust wird deshalb nur noch mit der bekannten Rate wieder
    // aufgebaut. Eine andere zu finden verlangt einen Neustart.
    void begin(BcuType bcuType);

    // KOMPAT: Signatur der alten Library. Setzt das Interface und ruft dann begin(bcuType).
    void begin(BcuType bcuType, Interface::Abstract *interface);

    // WIRD DIESE INSTANZ GERADE VOM ZENTRALEN TIMER GETICKT? Falsch heißt: sie wird gar nicht getickt,
    // solange nicht jemand Timer::trigger() ruft - denn der Hauptloop tickt NIE (siehe process()). Gründe
    // für falsch: die Plattform hat keinen Timer (Timer::supported()), das Intervall steht auf 0
    // (Timer::setInterval(0)), oder alle Plätze waren belegt (TPUART_TIMER_MAX_CLIENTS).
    //
    // EINEN SCHALTER JE INSTANZ GIBT ES BEWUSST NICHT: der Timer wird IMMER benutzt, und begin() trägt die
    // Instanz ohne Nachfrage ein. Wer den Takt selbst stellen will - eigener Task, fremder Timer, zweiter
    // Kern, oder eine Plattform ohne Timer-Unterstützung -, schaltet ihn global ab
    // (Timer::setInterval(0)) und ruft Timer::trigger() aus seinem eigenen Kontext.
    //
    // NUR AUSKUNFT, KEINE MELDUNG. Dass eine Instanz nicht getickt wird, meldet die Library selbst über
    // den Message-Callback - checkTickRate() sieht null Ticks im Fenster und schreibt "Tick stopped".
    // Diese Methode ist für einen Aufrufer, der den Zustand abfragen will; sie ist ausdrücklich NICHT die
    // Stelle, an der ein Verbraucher die Diagnose selbst zusammensetzen soll. Die `bcu`-Ausgabe von
    // OGM-Common benutzt sie deshalb nicht: eine Diagnose an zwei Stellen läuft auseinander, und die dort
    // wäre die schlechtere.
    //
    // Dynamisch beantwortet und nicht bei begin() festgeschrieben: der Timer kann später anlaufen, etwa
    // wenn eine andere Instanz ihren Platz freigibt.
    bool usesTimer() const;

    // KOMPAT: beendet den Betrieb und schließt das Interface. Der Parameter der alten Fassung
    // (deleteUart) fehlt bewusst - der DataLinkLayer besitzt das Interface nicht und darf es nicht
    // freigeben.
    void end();

    // Zeitkritische Seite: ein Byte pro Aufruf, läuft aus dem zentralen Timer. Siehe Klassenkommentar. Von
    // außen nur zu rufen, wenn der Timer diese Instanz NICHT treibt - zwei Kontexte in einer State-Machine
    // sind ein Defekt, kein Nachteil. Für den Regelfall gibt es Timer::trigger(), das alle eingetragenen
    // Instanzen bedient.
    void tick();

    // Gemütliche Seite: leert den RX-Ringpuffer, ruft den Frame-Callback und verarbeitet Steuerbytes
    // intern. Muss aus dem Hauptloop kommen, nie aus einem Interrupt.
    void loop();

    // KOMPAT: die alte Library kannte nur diesen einen Einstieg. RUFT NUR NOCH loop() - hier wird NICHT
    // getickt, nie. Der Hauptloop ist kein Antrieb für tick(); die Begründung steht an der Definition.
    // Für einen Aufrufer mit laufendem Timer ändert das nichts, dort tat process() schon vorher nur loop().
    void process();

    bool isConnected() const;
    uint32_t connectedBaudRate() const;
    BcuType bcuType() const;

    // Abgeleitet, nicht gespeichert - siehe BcuState.
    BcuState bcuState() const;
    const char *bcuStateName() const;

    // Anders als registerCheckAcknowledge() unkritisch: dieser Callback wird ausschließlich aus loop()
    // gerufen, also im selben Kontext, in dem er gesetzt wird. Genau dafür gibt es den Ringpuffer.
    // Mehrfach aufrufbar: jeder registrierte Callback bekommt jedes Telegramm, in der Reihenfolge der
    // Registrierung. Genau wie in der alten Library - dort hieß die Methode registerReceivedFrame().
    void registerFrameCallback(FrameCallback callback);

    // KOMPAT: alter Name von registerFrameCallback().
    void registerReceivedFrame(FrameCallback callback);

    // ACHTUNG, einziger Callback, der aus tick() heraus gerufen wird: die Acknowledge-Entscheidung fällt
    // mitten im laufenden Frame und lässt sich deshalb nicht über den Ringpuffer in den Hauptkontext
    // verlagern. Folgen: die Funktion muss kurz sein und darf weder blockieren noch Speicher anfordern
    // (auf dem RP2040 läuft sie im Interrupt), und sie muss VOR dem Start des Tick-Antriebs gesetzt werden -
    // eine std::function-Zuweisung ist nicht atomar, ein gleichzeitiger tick() träfe ein halb
    // überschriebenes Objekt an.
    void registerCheckAcknowledge(AcknowledgeCallback callback);

    // Ausgabekanal des Datalink Layers, wie registerMessage() in der alten Library. Ohne ihn passiert
    // dasselbe wie dort: die Meldungen entfallen ersatzlos, am Verhalten ändert sich nichts.
    void registerMessage(MessageCallback callback);

    // Alle drei melden einmalig und setzen sich dabei zurück - reine Statistik, aus dem Hauptkontext
    // abzufragen. Der Umweg über den Datalink Layer ist Absicht: das Interface darf nur aus EINEM Kontext
    // angefasst werden, und das ist tick(). Im Timer-Betrieb wäre ein direktes interface.overflow() aus
    // loop() ein zweiter Zugriff auf dieselben Zählerstände und Hardwareregister.
    bool queueOverflow();     // ein Telegramm wurde verworfen, weil der Ringpuffer voll war
    bool interfaceOverflow(); // das Interface hat Daten verloren (DMA hat den Leser überholt o.ä.)
    bool controlOverflow();   // ein Steuercode wurde verworfen, weil seine Warteschlange voll war

    // Die beiden Hälften direkt - für Diagnose und für alles, was ihren Zustand wissen will: RxState und
    // TxState, ob gerade gesendet wird, wie voll die Sendewarteschlange ist. Genauso hatte die alte Library
    // getReceiver()/getTransmitter().
    //
    // Hier standen einmal Kurzfassungen daneben (rxState(), txState(), isTransmitting(),
    // transmitQueueUsed/Size) - fünf Weiterleitungen auf Methoden, die über diese beiden ohnehin erreichbar
    // sind. Zwei Wege zum selben Wert sind einer zu viel; die Hälften sind der direktere.
    Receiver &getReceiver();
    Transmitter &getTransmitter();

    // Wie in der alten Library eine Referenz auf die Zähler. Die Einmal-Meldungen queueOverflow() &Co.
    // bleiben daneben bestehen: sie sind für "ist gerade etwas passiert" gedacht, die Statistik für
    // "wie oft insgesamt".
    Statistics &getStatistics();

    // Der Wiederholungsfilter arbeitet in loop() von selbst; der Zugriff ist für clear() und die Diagnose.
    RepetitionFilter &getRepetitionFilter();

    // Letzter vom Chip gemeldeter System-Status. Gefüllt wird er erst durch requestState() - und nur beim
    // NCN512x, der TPUART2 kennt den Dienst nicht (isValid() bleibt dort false). Ausgegeben wird er von
    // loop() selbst, sobald er sich ändert; dieser Zugriff ist für Aufrufer, die ihn auswerten wollen.
    SystemState &getSystemState();

    // --- Steuerbefehle an die BCU -------------------------------------------------------------------
    //
    // Namen und Semantik sind aus der alten Library übernommen, damit vorhandener Aufrufercode weiter
    // passt. Der Rückgabewert bedeutet dort wie hier "Vorbedingung erfüllt" - also NICHT "das Byte ist
    // schon auf dem Bus". Alle Befehle wandern in die Steuercode-Warteschlange des Transmitters und gehen
    // von dort aus dem Tick raus; sie funktionieren deshalb auch während eines laufenden Telegrammversands.
    // false heißt: keine Verbindung (siehe begin()), falscher Chiptyp, oder Warteschlange voll
    // (controlOverflow()).
    //
    // Anders als in der alten Library geben die Methoden, die dort void waren, hier bool zurück - für
    // Aufrufer, die den Wert ignorieren, ist das quellkompatibel.

    // Busmonitor einschalten: der Chip reicht danach alles ungefiltert durch, inklusive der Quittungen vom
    // Bus, und bestätigt selbst nichts mehr - entsprechend unterbleibt auch unser eigenes U_Ackn.req.
    // Es gibt bewusst kein Gegenstück: der Zustand lässt sich laut Datenblatt nur per reset() verlassen.
    // Ist der Busmonitor schon aktiv, wird kein Byte gesendet und true zurückgegeben (wie die alte Library).
    bool startMonitoring();

    // Setzt den Chip zurück. Beendet damit auch den Busmonitor und stellt den Default-CRC-Modus her.
    // Betrifft nur den bereits verbundenen Betrieb - für die initiale Verbindungsaufnahme siehe begin().
    bool reset();

    // Fragt den Chip-Status ab (U_State.req). Beim NCN512x zusätzlich den System-Status
    // (U_SystemState.req) - erst dadurch kann überhaupt ein U_SystemStat.ind eintreffen, das der Receiver
    // als einzige mehrbytige Steuerantwort behandelt.
    bool requestState();

    // Stop-Modus betreten/verlassen. Nur NCN512x - beim TPUART2 gibt es den Dienst nicht, dort false.
    bool stopMode(bool state);

    // Busy-Modus: der Chip quittiert adressierte Telegramme mit BUSY statt ACK. Beide Chiptypen, aber mit
    // unterschiedlichen Opcodes.
    //
    // ENDET NACH TPUART_BUSY_MODE_MS VON SELBST (700ms), auch ohne busyMode(false). Das ist kein Komfort,
    // sondern Gleichstand zwischen den Chips: der TPUART2 steigt nach dieser Zeit in Hardware aus, der
    // NCN512x nicht. Wer ihn länger braucht, setzt ihn erneut.
    bool busyMode(bool state);

    // Schaltet die Spannungsregler VCC2/20V des Chips (Schreibzugriff auf ACR0). Nur NCN512x.
    bool powerControl(bool state);

    // Physikalische Adresse des Geräts. Sie zu setzen aktiviert im Chip die AUTO-QUITTUNG: er bestätigt
    // Telegramme an DIESE EINE Adresse notfalls selbst, wenn vom Host nichts rechtzeitig kommt. Das ist
    // ein Fallback und kein Ersatz - unser eigenes Acknowledge geht weiterhin bei jedem Telegramm raus,
    // für das der Callback es verlangt (siehe Receiver::sendAcknowledge). Alles andere - Gruppenadressen,
    // bei einem Koppler auch fremde Einzeladressen - kennt der Chip gar nicht.
    //
    // 0 bedeutet "keine Adresse" und lässt die Auto-Quittung aus. ABSCHALTEN lässt sie sich damit aber
    // nicht mehr: dafür gibt es in beiden Datenblättern keinen Dienst, "Autoacknowledge can only be
    // deactivated by a Reset Service" (NCN5130 S. 38). Nach setOwnAddress(0) behält der Chip also seine
    // bisherige Adresse, bis reset() kommt oder der Strom weg war - isAutoAcknowledge() sagt, was gilt.
    //
    // Der Wert wird gehalten und nach jedem Reset der BCU erneut abgesetzt, denn dort geht er verloren.
    // Liefert false, wenn die Steuer-Warteschlange gerade voll ist; dann versucht es der nächste loop()
    // von selbst weiter.
    bool setOwnAddress(uint16_t address);
    uint16_t ownAddress() const;

    // Wie oft die BCU ein unquittiertes Telegramm wiederholt, getrennt für NACK und BUSY (je 0...7, beide
    // nach einem Reset 3). Reihenfolge der Parameter wie in der alten Library. false bei einem Wert über 7
    // oder wenn die Steuer-Warteschlange voll ist - im zweiten Fall versucht der nächste loop() weiter.
    bool setRepetitions(uint8_t nack, uint8_t busy);

    // Steht im Chip die Auto-Quittung als Fallback bereit? Das ist der Zustand des CHIPS und nicht bloß
    // unsere Absicht - genau wie bei isBusMonitor(): mit dem Absetzen der Adresse übernimmt er, mit jedem
    // Reset gibt er wieder ab, und beim NCN bestätigt er es zusätzlich per U_Configure.ind.
    //
    // REINE AUSKUNFT. Am eigenen Acknowledge ändert die Antwort nichts - wer daraus schließt, selbst
    // schweigen zu dürfen, quittiert am Ende gar nicht. Siehe Receiver::sendAcknowledge().
    bool isAutoAcknowledge() const;

    // Läuft der Busy-Modus gerade? Gemeint ist unser eigener Zeitgeber (siehe busyMode()), nicht eine
    // Rückmeldung des Chips - das Statusbyte führt das Bit zwar mit, aber es wird nur einmal je Sekunde
    // abgefragt und wäre für die Acknowledge-Entscheidung viel zu träge.
    bool isBusyMode() const;

    // Der Zustand, in dem der Chip tatsächlich ist - nicht der zuletzt gewünschte.
    bool isBusMonitor() const;

    // KOMPAT: alte Namen.
    bool isMonitoring() const;      // -> isBusMonitor()
    const char *getBcuStateInfo() const; // -> bcuStateName()

    // --- Telegrammversand ---------------------------------------------------------------------------
    //
    // STELLT EIN TELEGRAMM IN DIE SENDEWARTESCHLANGE. Ein laufender Versand ist KEIN Ablehnungsgrund -
    // eingereiht wird immer, abgearbeitet nach Priorität (System > Urgent > Normal > Low, innerhalb einer
    // Klasse in Eingangsreihenfolge).
    //
    // ÜBERGEBEN WIRD EIN VOLLSTÄNDIGES TELEGRAMM EINSCHLIESSLICH PRÜFSUMME - für alle drei Überladungen
    // dasselbe, es gibt hier keine zwei Konventionen mehr. Die Prüfsumme wird GEPRÜFT, nicht neu gerechnet:
    // sie gehört zum Telegramm, und sie stillschweigend zu überschreiben verdeckte einen Fehler im
    // Aufrufer - das Telegramm ginge dann mit korrekter CRC über falschem Inhalt hinaus.
    //
    // false bedeutet: keine Verbindung, Busmonitor aktiv (dort ist der Chip transparent und sendet nicht),
    // unpassende Länge, kein intaktes Telegramm (Steuerbyte, Länge oder Prüfsumme), oder die Warteschlange
    // ist voll. Jede Ablehnung nennt ihren Grund über den Message-Callback - aus einem bool ist er nicht
    // zu erschließen.
    //
    // Die Daten werden kopiert, der Aufrufer darf seinen Puffer sofort wieder verwenden.
    bool pushTransmitQueue(const Frame &frame);
    bool pushTransmitQueue(const uint8_t *data, size_t length);

    // KOMPAT: Sendeweg der alten Library, und die EINZIGE Überladung, die den Besitz übernimmt - bei Erfolg
    // wird das Frame gelöscht, bei false bleibt es beim Aufrufer. Der knx-Stack verlässt sich darauf.
    bool pushTransmitQueue(Frame *frame);
};

} // namespace TPUart
