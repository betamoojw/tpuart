#pragma once
#include <stddef.h>
#include <stdint.h>

#include "TPUart/Frame.h"
#include "TPUart/Types.h"

namespace TPUart
{

class DataLinkLayer;

// Ab wann Stille als Pause gilt, also als Ende eines Telegramms.
//
// Der Abstand, den es zu unterscheiden gilt, ist kleiner als die Zeichenlänge: der Bus läuft immer mit
// 9600 Baud, ein TP1-Zeichen belegt 13 Bitzeiten = 1,354ms, aber INNERHALB eines Telegramms liegen nur
// die Byte-Anfänge so weit auseinander. Die Stille dazwischen ist am Host 1,354ms minus der
// Übertragungszeit zur BCU, bei 19200 also gut 0,78ms und bei 38400 rund 1,07ms. Das ist die Untergrenze.
//
// Nach oben begrenzt der kürzestmögliche Abstand zum nächsten Telegramm. Vor jeder Übertragung müssen
// 50 Bitzeiten Bus-Leerlauf liegen (Datenblatt S. 38) - das sind 5,21ms. Der knappste Fall ist NICHT der
// quittierte (dort belegt die Quittung den Bus bis 26 Bitzeiten, das nächste Telegramm kann also erst
// nach 76 Bitzeiten = 7,92ms beginnen), sondern der UNQUITTIERTE: dort ist der Bus seit dem Telegrammende
// leer, die 50 Bitzeiten laufen sofort mit, und die Wiederholung startet nach 5,21ms.
//
// Am Host wird daraus etwas mehr, weil dessen erstes Zeichen erst vollständig vom Bus empfangen sein
// muss, bevor die BCU es weiterreicht: rund 6,36ms beobachtbare Stille. Das ist die harte Obergrenze.
//
// Im Busmonitor ist das anders: dort wird die Quittung durchgereicht und trifft schon nach ~3,3ms ein.
// Dafür gibt es deshalb eine eigene, längere Frist - siehe TPUART_FRAME_ACK_US. Beides in eine Konstante
// zu zwingen hieße, die kürzere der beiden zu opfern.
//
// Genau genommen ist der real wirksame Abstand immer größer als dieser Wert: gemessen wird ab der ersten
// Beobachtung "nichts mehr da", also nach dem letzten GELESENEN Byte, nicht ab dem letzten Bit auf dem
// Bus.
//
// Unkritisch ist das Ganze aus einem anderen Grund: die Pause hat überhaupt nur dann eine Wirkung, wenn
// vorher etwas schiefgelaufen ist. Läuft der Empfang sauber, steht die Maschine zwischen zwei Telegrammen
// auf Idle, und dort tut eine Pause nichts - die Telegrammlänge kommt aus dem Längenbyte, nicht aus dem
// Timing. Eine verpasste Pause (etwa weil der Tick eine Weile nicht drankam) kostet also nichts. Gebraucht
// wird sie nur, um aus dem Resync zurückzufinden, ein abgeschnittenes Telegramm abzuschließen - und im
// Busmonitor, um eine ausbleibende Quittung als solche zu erkennen.
// DIE ZAHL KOMMT AUS DEN DATENBLÄTTERN, und zwar als Unterscheidungskriterium, nicht als Schätzung:
//
//   NCN5130 S. 40 und Figures 44-47 / 50-53 ("Frame End with Silence", Senden UND Empfangen):
//   ">= 2.6 ms silence". Ab dieser Stille ist es garantiert ein Frame-Ende.
//
//   Siemens TP-UART 2 S. 32 / 2+ S. 33: "The host either has to detect an end of packet timeout by
//   supervising the EOP gap of 2 - 2,5 ms or check the CRC16-CCITT."
//
// 2600 ist damit die kleinste Frist, bei der die Aussage für BEIDE Chips trägt: darunter deutet man eine
// Lücke, die der Hersteller nicht mehr garantiert, darüber wartet man ohne Gegenwert. Hier standen einmal
// 2800 - bemessen gegen die 5,2ms bis zum nächsten Telegramm, also gegen die Notbremse statt gegen das
// dokumentierte Frame-Ende. Das war 200µs zu spät und hat die Marke beider Hersteller verfehlt.
//
// WAS DAVON ANKOMMT, HÄNGT AM TAKT. Gemessen wird ab dem ersten Tick, der "nichts da" sieht, ausgelöst
// beim ersten Tick nach Ablauf - die wirksame Pause liegt also zwischen der Frist und der Frist plus zwei
// Tickintervallen. Bei 500µs Takt sind das 2,6-3,6ms, eine Lücke von genau 2,6ms wird damit nicht
// erkannt; bei 250µs oder auf loop1 schon. Wer die Marke wirklich braucht, taktet schneller - die Frist
// tiefer zu setzen hilft nicht, sie wäre dann unterhalb dessen, was die Datenblätter zusagen.
#ifndef TPUART_FRAME_WAIT_US
#define TPUART_FRAME_WAIT_US 2600
#endif

// Wie lange nach einem vollständigen Telegramm auf dessen Antwort gewartet wird (RxState::FrameAck).
// Zwei Fälle, technisch derselbe Vorgang und deshalb dieselbe Frist:
//   Busmonitor      - die Quittung vom Bus, erreicht den Host als L_Ackn.ind
//   eigener Versand - die Quittung des Empfängers, erreicht den Host als L_Data.con
// Beide entstehen aus derselben Quittung auf dem Bus und treffen entsprechend gleich spät ein.
//
// Eigene Frist, weil die Antwort deutlich später eintrifft als die Telegrammgrenze erkennbar wäre.
// Zeitachse ab dem letzten Bit des Telegramms auf dem Bus:
//   ~0,57ms  das letzte Frame-Byte erreicht den Host (11 Bit bei 19200) - hier startet unser Fenster
//    1,56ms  die Quittung beginnt auf dem Bus (15 Bitzeiten nach dem Frame)
//   ~2,71ms  sie ist auf dem Bus fertig (11 Bit bei 9600)
//   ~3,28ms  sie erreicht den Host
// Gemessen ab Fensterstart bleiben also gut 2,7ms bis zur Quittung - und die Pausenschwelle liegt mit
// 2600 DARUNTER. Ohne eigene Frist würde die Pause also auslösen, bevor die Quittung überhaupt eintrifft,
// und der Wartezustand käme nie zu seinem Ergebnis. 4000 gibt gut 1,3ms Reserve und liegt zugleich unter
// den 6,36ms, nach denen frühestens das nächste Telegramm am Host eintreffen kann.
#ifndef TPUART_FRAME_ACK_US
#define TPUART_FRAME_ACK_US 4000
#endif

// RX-Warteschlange zwischen tick() und loop(). Größe großzügig gewählt: ein Standard-Telegramm belegt mit
// Kopf rund 12 Byte, 1024 Byte fassen also gut 80 davon.
#ifndef TPUART_RX_QUEUE_EXP
#define TPUART_RX_QUEUE_EXP 10
#endif
constexpr uint32_t TPUART_RX_QUEUE_SIZE = (1u << TPUART_RX_QUEUE_EXP);

// Kopf eines Eintrags im Ringpuffer: Länge (2 Byte, weil 263 nicht in eines passt) + Flags (1 Byte).
// Danach folgen die Rohdaten. Die Flags stehen VOR den Daten, damit sich beim Auslesen entscheiden lässt,
// ob der Eintrag überhaupt interessiert, bevor die Daten angefasst werden.
constexpr uint32_t TPUART_RX_QUEUE_HEADER_SIZE = 3;

// Die Empfangshälfte, wie Receiver in der alten Library - mit demselben Aufbau: die Klasse hält eine
// Referenz auf den DataLinkLayer, der weiterhin Interface, Statistik und Chip-Zustand besitzt.
//
// ZWEI KONTEXTE, sauber getrennt, und das ist der Grund für den Ringpuffer in der Mitte:
//   process()      - aus tick(), also später aus einem Timer-Interrupt. Verarbeitet höchstens EIN Byte,
//                    blockiert nie, alloziert nie, gibt nichts nach außen. Fertige Sequenzen wandern in den
//                    Ringpuffer und der Zustand geht sofort weiter.
//   processQueue() - aus loop(). Leert den Ringpuffer, prüft den Wiederholungsfilter und ruft die Callbacks.
//
// DREI PUFFER, DREI AUFGABEN - die Frage "warum nicht einer?" beantwortet sich an den Besitzern:
//
//   _buffer  (263, TICK)         die gerade einlaufende Sequenz. Zusammenhängend, weil das Parsen darin
//                                indiziert (Längenbyte, Prüfsumme) und der Echo-Vergleich memcmp benutzt -
//                                direkt im Ring zu parsen hieße Modulo-Indizierung im zeitkritischsten Pfad.
//   _queue   (1024, TICK->LOOP)  die ISR-Grenze UND der Rückstau. Er fasst rund 80 Telegramme, und das ist
//                                seine eigentliche Aufgabe: steht der Hauptloop eine Sekunde, liefert der
//                                Bus in der Zeit ~740 Byte nach.
//   Frame    (263, LOOP-STACK)   die Übergabe an den Callback. Kein Member: das Frame besitzt seine Daten
//                                selbst (siehe Frame.h) und entsteht je Eintrag auf dem Stack von
//                                processQueue(). Der Ringplatz ist frei, bevor fremder Code läuft.
//
// Zusammenlegen geht bei keinem Paar. _buffer gehört dem Tick, das Frame dem Loop - wären es dieselben
// Bytes, überschriebe der Tick genau das, was der Callback gerade liest. Und der Ring kann das Frame nicht
// ersetzen (er bricht um) noch umgekehrt (es fasst ein einziges Telegramm).
//
// Das Acknowledge gehört mit zur Empfangsseite, obwohl es ein Schreibvorgang ist: das WANN fällt mitten im
// laufenden Frame (bei Byte 6, sobald Ziel, Adresstyp und Restlänge feststehen), und nur der Empfang weiß,
// wie viel von diesem Frame noch aussteht. Das Byte selbst schreibt dann der Transmitter - dort liegt aller
// schreibende Zugriff auf das Interface zusammen. Genauso hielt es die alte Library: sendAcknowledge() war
// dort eine Methode des Transmitters, gerufen aus dem Empfangspfad.
//
// DIE CALLBACKS LIEGEN NICHT HIER, sondern im DataLinkLayer - auch das wie in der alten Library
// (_callbackCheckAcknowledge, _callbacksReceivedFrame). Er ist die Schnittstelle nach außen, bei ihm werden
// sie registriert, also ruft er sie auch. Diese Hälfte baut das Frame und fragt: _dll.checkAcknowledge()
// für das OB und WAS einer Quittung, _dll.deliverFrame() für die Übergabe.
class Receiver
{
  private:
    DataLinkLayer &_dll;

    // Interface, Statistik und die andere Hälfte werden über _dll gerufen, nicht als eigene Referenz
    // gehalten. Zwischenspeichern spart eine Indirektion im Byte-Pfad, kostet aber eine Zusage über die
    // Deklarationsreihenfolge im DataLinkLayer - und die wäre für den Transmitter ohnehin nicht zu halten,
    // weil die beiden Hälften einander brauchen. Gegen 2-3µs Tickdauer ist die Indirektion nicht messbar.

    // Hält die gerade einlaufende Sequenz - entweder ein Frame oder eine Steuerbyte-Sequenz, nie beides
    // zugleich, deshalb reicht dafür ein Puffer.
    //
    // IST und SOLL, und der Unterschied ist der Punkt: _bufferPos sagt, wohin das nächste Byte geht - also
    // wie viel tatsächlich eingelaufen ist, auch bei Steuerbyte-Sequenzen, die gar keine Frame-Größe haben.
    // _frameSize sagt, wie lang das Telegramm laut seinem Kopf sein müsste. Auseinander laufen sie, wenn
    // eine Pause abschneidet - das Telegramm wird dann als INVALID gemeldet.
    uint8_t _buffer[TPUART_BUFFER_SIZE];
    size_t _bufferPos = 0;

    volatile RxState _state = RxState::Idle;

    // 0 heißt "Kopf noch nicht ausgewertet" - und das ist mehr als ein Startwert: daran hängt, dass der
    // Acknowledge-Zweig in processFrameByte() pro Frame exakt EINMAL läuft. Ohne diesen Marker bräuchte es
    // ein zusätzliches "schon geackt"-Flag. Ein echtes Frame ist immer >= 8 Byte. Ein Poll-Telegramm
    // benutzt dasselbe Feld für dieselbe Aussage (7 + Slot-Count, siehe processPollByte).
    size_t _frameSize = 0;
    uint8_t _crc = 0;

    // Was der laufenden Sequenz bis zu ihrem Abschluss zuwächst - GENAU IN DER FORM, in der der Eintrag im
    // Ringpuffer sie trägt und das Frame sie später liest. Hier standen einmal zwei getypte Felder
    // (_addressed, _acknowledge), die completeSequence() erst umrechnen musste; gebraucht wird von beiden
    // aber nur das Ergebnis, und ein Zwischenwert entsteht nirgends.
    //
    // Zwei der Bits sind zwei VERSCHIEDENE Aussagen, das bleibt wichtig:
    //   ADDRESSED - das Telegramm ist für dieses Gerät bestimmt. Das entscheidet allein der
    //               Acknowledge-Callback, unabhängig davon, ob die Quittung es rechtzeitig auf den Bus
    //               geschafft hat. Ein wegen Rückstands unterdrücktes U_Ackn.req ändert nichts daran, an
    //               wen das Telegramm gerichtet war - der Aufrufer muss es trotzdem verarbeiten. Vermischt
    //               man beides, verschwindet ein an uns gerichtetes Telegramm bei Last unbemerkt aus der
    //               Zuständigkeit des Geräts.
    //   ACK       - es liegt eine Quittung vor. Drei Quellen, die sich gegenseitig ausschließen: unsere
    //               eigene (dann steht ADDRESSED daneben), eine im Busmonitor auf dem Bus beobachtete
    //               (dann nicht), oder die Bestätigung zu einem selbst gesendeten Telegramm - die reicht
    //               completeSequence() als Parameter herein (siehe RxState::FrameAck).
    uint8_t _flags = 0;

    // Seit wann das Interface nichts mehr hergibt. Eine Pause gilt erst dann als echt, wenn dieser Zustand
    // TPUART_FRAME_WAIT_US lang ununterbrochen anhält.
    //
    // Maßgeblich ist der Zeitpunkt der ERSTEN Beobachtung "nichts da" - nicht der Abstand zum zuletzt
    // gelesenen Byte. Andernfalls würde eine unterbrochene Abarbeitung (pausierter Interrupt, blockierter
    // Loop) als Pause fehlgedeutet, obwohl in dieser Zeit sehr wohl Bytes eingetroffen sind - available()
    // spiegelt den DMA-Puffer und bleibt auch dann wahrheitsgemäß. Genau dafür braucht es das zweite Feld:
    // _emptySince gilt nur, wenn die Messung überhaupt läuft.
    //
    // Ein drittes Feld "schon ausgelöst" stand hier zwischenzeitlich und ist wieder weg: gemessen wird
    // ohnehin nur, solange eine Sequenz offen ist (checkPause() kehrt bei Idle sofort um), und nach dem
    // Auslösen ist der Zustand immer Idle. Der Merker hätte also nichts verhindert, was nicht schon die
    // Zustandsprüfung verhindert - und die ist zugleich der billigere Leerlaufpfad, weil sie auch die
    // ruhigen Zeiten VOR einem Telegramm abdeckt.
    uint32_t _emptySince = 0;
    bool _emptyStarted = false;

    // Warteschlange zwischen tick() und loop(). Kopf und Schwanz laufen monoton weiter (Index per Modulo),
    // damit "voll" und "leer" nicht verwechselbar sind. Single-Producer/Single-Consumer: _queueHead
    // schreibt nur tick(), _queueTail nur loop() - und _queueHead wird erst NACH dem vollständigen
    // Schreiben eines Eintrags weitergesetzt, sodass loop() nie einen halben Eintrag sieht.
    uint8_t _queue[TPUART_RX_QUEUE_SIZE];
    volatile uint32_t _queueHead = 0;
    volatile uint32_t _queueTail = 0;


    void processByte(uint8_t value);
    void processFrameByte(uint8_t value);
    void processPollByte(uint8_t value);
    void processControlByte(uint8_t value);

    // Entscheidet über die Quittung zum laufenden Frame und lässt sie vom Transmitter absetzen.
    void sendAcknowledge();

    void checkPause();
    void handleVerifiedPause();

    // Setzt die Empfangsmaschine auf den Anfang einer neuen Sequenz und geht in nextState über. Einzige
    // Stelle, an der dieser Satz Felder angefasst wird - siehe dort.
    void resetSequence(RxState nextState);

    // Stellt die aktuell im _buffer liegende Sequenz in den Ringpuffer und geht in nextState über.
    void completeSequence(uint8_t flags, RxState nextState);
    bool pushEntry(const uint8_t *data, size_t length, uint8_t flags);

    // ANGETRIEBEN WIRD DIESE HÄLFTE AUSSCHLIESSLICH VOM DataLinkLayer, deshalb liegt alles Folgende privat
    // hinter einer Freundschaft und nicht öffentlich hinter getReceiver(). Jede dieser Methoden gehört in
    // genau EINEN Kontext, und keine davon prüft das nachträglich: ein process() aus dem Hauptloop, während
    // der Timer tickt, hätte zwei Leser auf demselben Interface, ein processQueue() von außen zwei
    // Konsumenten auf dem Ringpuffer. Nach außen bleibt darum nur, was man gefahrlos ANSEHEN kann.
    friend class DataLinkLayer;

    // Aus tick(): verarbeitet höchstens EIN Byte. Damit ist der Zeitaufwand beschränkt und unabhängig
    // davon, wie viel im Interface-Puffer liegt - wichtig, damit die ACK-Entscheidung ihr enges Zeitfenster
    // einhält.
    void process();

    // Aus loop(): leert den Ringpuffer. Telegramme gehen durch den Wiederholungsfilter und dann per
    // Callback nach außen, Steuerbyte-Sequenzen verarbeitet der DataLinkLayer selbst.
    void processQueue();

    // Bricht eine laufende Sequenz ab und geht in den Resync. Wird gerufen, wenn sich die Bedeutung des
    // Bytestroms ändert (Busmonitor an, Reset) - dort ist eine angebrochene Sequenz nicht mehr deutbar.
    void forceResync();

  public:
    explicit Receiver(DataLinkLayer &dll);

    RxState state() const;

    // --- KOMPAT: Diagnosewerte der alten Library ------------------------------------------------------
    //
    // Beide Namen stammen aus der Suche im Puffer, die es hier nicht mehr gibt - sie liefern deshalb nicht
    // das, wonach sie heißen, sondern die nächstliegende Aussage über den heutigen Empfangspfad. Zusammen
    // zeigen sie, wo im Telegramm die Verarbeitung gerade steht. Können weg, sobald bestätigt ist, dass
    // sie niemand mehr druckt.
    unsigned short getSearchBufferPosition() const; // Bytes der laufenden Sequenz im Puffer
    unsigned short getAwaitBytes() const;           // noch ausstehende Bytes des laufenden Telegramms
};

} // namespace TPUart
