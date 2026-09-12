#pragma once
#include "TPUart/TransmitQueue.h"

#include <stddef.h>
#include <stdint.h>

#include "TPUart/Types.h"

namespace TPUart
{

class DataLinkLayer;

// Die Sendewarteschlange liegt in TransmitQueue - ein statischer Bytepuffer, dem Hauptkontext allein
// gehörend, prioritätsgeordnet. Ihre Größe und die Begründung dafür stehen dort.

// Der SCHLIMMSTE Fall eines Telegrammbytes: Offset-Byte, Positionsbyte, Datenbyte. Die drei müssen
// unmittelbar aufeinander folgen, sonst schöbe sich ein zwischendurch fälliges Acknowledge dazwischen.
//
// Die Zahl dient hier nur noch als Obergrenze für TPUART_TX_INTERFACE_BUFFER. Zur Laufzeit verlangt
// process() genau so viel Platz, wie das anstehende Byte wirklich braucht - meistens 2, denn das
// Offset-Byte fällt nur alle 64 Positionen an. Pauschal 3 zu fordern hat messbar gebremst, siehe dort.
constexpr size_t TPUART_TX_ATOMIC_BYTES = 3;

// Steuercode-Warteschlange zwischen Hauptkontext und tick(). Steuercodes müssen sich auch dann absetzen
// lassen, wenn gerade ein Telegramm läuft - sie sind, genau wie das Acknowledge, kein Telegrammversand,
// sondern gehen zwischendurch raus. Deshalb eine eigene Queue statt einer Besitzübergabe über TxState:
// vorher lehnte queueControl() bei belegtem Sendeweg schlicht ab, was auch der alten Library widerspricht
// (deren bool-Rückgabe bedeutet "Vorbedingung erfüllt", nicht "gerade frei").
#ifndef TPUART_CTRL_QUEUE_EXP
#define TPUART_CTRL_QUEUE_EXP 5 // 32 Byte - reicht für mehrere Sequenzen
#endif
constexpr uint32_t TPUART_CTRL_QUEUE_SIZE = (1u << TPUART_CTRL_QUEUE_EXP);

// Längste Steuersequenz laut Datenblatt Table 12: U_SetAddress.req, U_SetRepetition.req und
// U_PollingState.req sind je 4 Byte lang. Eine Sequenz muss ununterbrochen rausgehen, sonst schiebt sich
// ein Telegramm- oder Acknowledge-Byte hinein und die BCU deutet die Reste als eigenen Befehl.
constexpr size_t TPUART_CTRL_MAX_GROUP = 4;

// WAS DER TRANSMITTER VON EINEM INTERFACE VERLANGT - beide Richtungen, und die zweite ist die unbequeme:
//
//   MINDESTENS so viele Bytes am Stück annehmen. Das ist das Maximum der beiden Zahlen oben. Wer weniger
//   nimmt, bekommt die längste Sequenz NIE durch: zerteilen darf sie sich nicht, sie bleibt also stehen und
//   blockiert alles Weitere. Genau so ist es passiert (ArduinoSerial über SerialUART, das nur 0/1 meldete).
//
//   HÖCHSTENS so viele Bytes ausstehen lassen. Ein Interface darf sich also NICHT auf eine tiefe
//   Hardware-FIFO oder einen Treiberpuffer stützen, auch wenn es sie hat: was dort liegt, verzögert alles
//   Folgende - und das nächste Byte ist womöglich ein U_Ackn.req, das binnen 2,8ms beim Chip sein muss
//   (Siemens TP-UART 2+, S. 25). 32 Byte in einer TX-FIFO sind bei 19200 Baud 18ms; die Quittung käme dann
//   nicht bloß spät, sondern würde vom Chip dem NÄCHSTEN Telegramm zugeordnet. Vier Bytes sind ~2,3ms und
//   damit die Obergrenze, die noch vertretbar ist.
//
// Beide Regeln zusammen heißen: das Interface hält einen kleinen eigenen Sendepuffer dieser Größe und gibt
// nach unten nur weiter, was auch gleich rausgeht. Groß puffern hilft hier nichts - der Bus nimmt ohnehin
// nur ein Byte je 1,354ms ab, und alles, was länger wartet, kommt zu spät. Ein größerer Puffer würde nur
// den Rückstau verstecken, den availableForWrite() dem Aufrufer melden soll.
//
// VIER PASST AUCH FÜR DEN DICHTESTEN TICK, und das ist ein zweiter, unabhängiger Grund für die Zahl:
// DataLinkLayer::tick() ruft erst _receiver.process(), dann _transmitter.process(). Ein fälliges
// Acknowledge entsteht also im EMPFANGSPFAD (Receiver::sendAcknowledge(), mitten im laufenden Frame) und
// geht dem Telegrammbyte voraus - im schlimmsten Fall 1 Byte Quittung plus die 3 Bytes einer Offset-Gruppe,
// zusammen genau vier. Dass es nur EIN Offset-Byte je Oktett ist (offset = _bufferPos >> 6, geschrieben nur
// bei Abweichung vom gemerkten Chip-Offset), ist dafür wesentlich: bei zwei ginge die Rechnung nicht auf.
//
// Die Unteilbarkeit hängt dabei NICHT an dieser Zahl, sondern an der Reihenfolge und an der Vorabprüfung in
// process(): das Acknowledge kann der Gruppe nur vorangehen, und die Gruppe wird erst geschrieben, wenn sie
// vollständig passt. Reicht der Platz nach einer Quittung nicht - etwa für eine 4-Byte-Steuersequenz -,
// wartet sie einen Tick. Verzögerung, kein Zerteilen.
//
// Die Zahl steht HIER und nicht bei den Interfaces: sie ist eine Anforderung des Protokolls, keine
// Eigenschaft einer Hardware, und sie ergibt sich aus den beiden Konstanten darüber. Die Interfaces leiten
// ihre Puffergröße daraus ab - damit kann die Anforderung nicht mehr an einer Stelle wachsen und an der
// anderen stehenbleiben.
constexpr size_t TPUART_TX_INTERFACE_BUFFER = TPUART_CTRL_MAX_GROUP > TPUART_TX_ATOMIC_BYTES ? TPUART_CTRL_MAX_GROUP : TPUART_TX_ATOMIC_BYTES;

// Wachhund für den Fall, dass das L_Data.con GAR NICHT kommt. Das ist keine Protokollfrist, sondern eine
// Notbremse: kommt die Bestätigung nicht, ist unklar, was im Sendepuffer der BCU noch liegt, und der
// einzige Weg zu einem definierten Zustand ist der Reset. Der wird nach dieser Zeit gesendet - und die
// Wiederaufnahme hängt dann nicht an ihm, sondern an der U_Reset.ind: JEDER Reset räumt den Sendepuffer
// der BCU, ganz gleich wer ihn ausgelöst hat, also wird danach schlicht neu begonnen, was noch offen ist.
//
// SIE MISST FEHLENDEN FORTSCHRITT, NICHT FEHLENDE FERTIGSTELLUNG - und deshalb setzt JEDES ECHO sie neu
// auf (echoReceived()). Das Echo ist der Beweis, dass der Chip das Telegramm gerade auf den Bus legt: er
// spiegelt jedes gesendete Oktett zurück, auch bei jeder Wiederholung. Solange es kommt, KANN die
// Bestätigung noch gar nicht da sein, und abzubrechen hieße, einem gesunden Vorgang in die Speichen zu
// greifen.
//
// Der Unterschied wird an einem ausgelasteten Bus greifbar: vor jeder Wiederholung muss der Chip warten,
// bis die Leitung frei ist. Mit einer festen Frist ab dem U_L_DataEnd.req wäre die Notbremse mitten in
// eine laufende Übertragung gefallen und hätte ein Telegramm verworfen, das unterwegs war. Zu messen ist
// der Abstand zwischen zwei Lebenszeichen, nicht die Gesamtdauer.
//
// 10s sind dafür reichlich: ein Extended-Telegramm in Maximalgröße belegt den Bus 356ms (263 Zeichen x
// 1,354ms), dazu die Wartezeit auf einen freien Bus vor der Wiederholung. Bleibt die Antwort trotzdem aus,
// ist unklar, was im Sendepuffer der BCU noch liegt.
//
// Neu aufgesetzt wird die Frist außerdem, nachdem der Wachhund selbst zugeschlagen hat: gegen eine stumme
// BCU wird damit alle 10s ein Reset versucht, statt einmal und nie wieder.
#ifndef TPUART_TX_CONFIRM_TIMEOUT_MS
#define TPUART_TX_CONFIRM_TIMEOUT_MS 10000
#endif

// Die Sendehälfte, wie Transmitter in der alten Library. HIER LIEGT JEDER SCHREIBENDE ZUGRIFF AUF DAS
// INTERFACE zusammen, und das sind drei verschiedene Dinge mit drei verschiedenen Dringlichkeiten:
//
//   Steuercodes  - kurz, zeitnah erwartet, dürfen sich laut Protokoll zwischen zwei Telegrammbytes
//                  schieben. Haben Vorrang und ihre eigene Warteschlange.
//   Acknowledge  - ein einzelnes Byte, das mitten in einem EINLAUFENDEN Frame rausmuss. Entschieden wird
//                  es im Receiver (nur der weiß, wie viel von dem Frame noch aussteht), geschrieben hier.
//   Telegramme   - der eigentliche Versand: ein Oktett je Tick, jedes mit seinem Positionsbyte, bis der
//                  Chip mit dem U_L_DataEnd.req die Übertragung auf den Bus beginnt.
//
// Nur das Letzte ist ein Zustand (TxState). Die beiden anderen gehen zwischendurch raus und lassen einen
// laufenden Versand unberührt - deshalb kann ein Acknowledge auch dann noch rechtzeitig kommen, wenn gerade
// gesendet wird.
class Transmitter
{
  private:
    DataLinkLayer &_dll;

    // Interface, Statistik und die andere Hälfte werden über _dll gerufen, nicht als eigene Referenz
    // gehalten - siehe Receiver.h, dieselbe Begründung.

    // DIE SENDEWARTESCHLANGE GEHÖRT DEM HAUPTKONTEXT ALLEIN - siehe TransmitQueue.h. Der Tick fasst sie
    // nicht an; er bekommt genau ein Telegramm vorgelegt. Nur unter dieser Bedingung darf darin überhaupt
    // nach Priorität umsortiert werden.
    TransmitQueue _queue;

    // DIE VORLAGE: ein Telegramm, das für den Tick bereitliegt. Zeigt IN den Puffer der Warteschlange,
    // es wird also nichts kopiert - kopiert wird erst der Tick in seinen eigenen _buffer.
    //
    // DREI REGELN, und sie tragen die ganze Nebenläufigkeit dieses Pfades:
    //   1. Der Hauptkontext schreibt die Vorlage NUR, wenn _stagedSeq == _takenSeq.
    //   2. Er gibt den Platz erst frei (pop), wenn _takenSeq nachgezogen hat.
    //   3. Veröffentlicht wird in beide Richtungen ZULETZT - Nutzlast vor Zähler.
    // Je ein Schreiber, gelesen wird über Kreuz. Ungleich heißt "es liegt etwas bereit".
    const uint8_t *_stagedBuffer = nullptr;
    size_t _stagedFrameSize = 0;
    volatile uint32_t _stagedSeq = 0; // schreibt nur der Hauptkontext
    volatile uint32_t _takenSeq = 0;  // schreibt nur tick()

    // Das Telegramm, das GERADE übertragen wird - bewusst getrennt von der Warteschlange, denn das sind
    // zwei verschiedene Vorgänge: die Warteschlange nimmt entgegen und reicht weiter, der Sendepuffer
    // gehört einer laufenden Übertragung. Sobald ein Telegramm hierher kopiert ist, ist sein Platz im Ring
    // wieder frei, und der Puffer bleibt unangetastet, bis die Übertragung samt Echo-Vergleich durch ist.
    // Er gehört ausschließlich dem Tick; der Hauptkontext fasst ihn nie an.
    // DIESELBEN ZWEI BEGRIFFE WIE IM RECEIVER, nur spiegelverkehrt: _bufferPos sagt, wo wir im Puffer
    // stehen (dort läuft das nächste Byte dorthin ein, hier geht das nächste Oktett von dort raus), und
    // _frameSize, wie lang das Telegramm ist. Sie hießen hier einmal _index und _length - zwei Namen für
    // Dinge, die auf der anderen Seite längst welche hatten.
    //
    // Ein Unterschied bleibt, und der liegt in der Sache: beim Empfang wächst _bufferPos auf _frameSize zu,
    // beim Senden ist der Puffer von Anfang an voll (startNextTransmission kopiert ihn am Stück) und
    // _bufferPos wandert hindurch.
    uint8_t _buffer[TPUART_BUFFER_SIZE];
    size_t _frameSize = 0; // inklusive Prüfsumme, genau wie sie im Puffer steht
    size_t _bufferPos = 0; // nächstes zu sendendes Oktett des laufenden Telegramms

    // WAS DER CHIP WEISS, nicht was wir wissen: unseren Offset rechnet process() bei jedem Oktett frisch
    // aus _bufferPos (die oberen Bits davon, weil ins Positionsbyte nur 6 passen). Gemerkt wird hier der
    // zuletzt ABGESETZTE - der Chip behält ihn, bis ein neuer kommt, also geht er nur bei Änderung raus.
    // Deshalb "chip" im Namen: ein bloßes _offset läse sich wie eine zweite Kopie von _bufferPos >> 6.
    uint8_t _chipOffset = 0;
    bool _chipOffsetValid = false; // nach einem Neubeginn ist unbekannt, was im Chip steht
    uint32_t _awaitSince = 0; // seit wann auf L_Data.con gewartet wird

    volatile TxState _state = TxState::Idle;

    // Steuercode-Warteschlange, Format je Eintrag: [len][bytes...]. Spiegelbild der RX-Warteschlange, nur
    // in die andere Richtung: ein Produzent (Hauptkontext), ein Konsument (tick()), und der Kopf wird erst
    // NACH dem vollständigen Schreiben weitergesetzt, damit tick() nie eine halbe Sequenz sieht.
    uint8_t _ctrlQueue[TPUART_CTRL_QUEUE_SIZE];
    volatile uint32_t _ctrlQueueHead = 0; // schreibt nur der Hauptkontext
    volatile uint32_t _ctrlQueueTail = 0; // schreibt nur tick()

    // Siehe confirmTimeout(). Der Tick setzt, der Hauptkontext liest und löscht.
    volatile bool _confirmTimeout = false;


    // Holt das vorgelegte Telegramm in den Sendepuffer. Liefert true, wenn danach eines zum Senden
    // bereitsteht. Gibt NICHTS frei - das gehört in den Hauptkontext (stageNextTelegram()).
    bool startNextTransmission();

    // Setzt den Fortschritt im Sendepuffer auf Anfang. Einzige Stelle, an der das passiert - gerufen vom
    // frisch geholten Telegramm und vom Neubeginn nach einem Reset.
    void beginTransmission();

    // Setzt höchstens EINE vollständige Steuersequenz ab. Liefert true, wenn der Sendeweg in diesem Tick
    // den Steuercodes gehört - dann ist der Telegrammpfad für diese Runde außen vor.
    bool processCtrlQueue();

    // DER EINZIGE SCHREIBZUGRIFF AUF DIE SCHNITTSTELLE in dieser Klasse, damit die Zählung vollständig ist:
    // Statistics::_txBytes ist der Maßstab für die Auslastung der Hostleitung, und dafür darf kein Byte
    // daran vorbeigehen. Wer hier ein write() direkt am Interface ergänzt, verfälscht die Messung.
    bool writeByte(uint8_t value);

    // ANGETRIEBEN WIRD DIESE HÄLFTE VOM DataLinkLayer, und der Receiver braucht davon drei Methoden für
    // Quittung und Echo-Vergleich. Alles Folgende liegt deshalb privat hinter diesen beiden Freundschaften
    // und nicht öffentlich hinter getTransmitter(): jede dieser Methoden gehört in genau EINEN Kontext und
    // prüft das nicht nachträglich. queueControl() wäre von außen sogar ein Weg, dem Chip beliebige
    // Steuerbytes zu schicken und damit die abgeleiteten Zustände (Busmonitor, Auto-Quittung) hinter dem
    // Rücken der Library zu verstellen - dafür gibt es die semantischen Befehle am DataLinkLayer.
    friend class DataLinkLayer;
    friend class Receiver;

    // Aus tick(): setzt höchstens EINE Steuersequenz ODER ein Telegramm-Oktett ab.
    void process();

    // Aus dem Hauptkontext: stellt ein vollständiges Telegramm EINSCHLIESSLICH Prüfsumme in die
    // Sendewarteschlange. Geprüft wird vor jeder Veränderung am Puffer, die Bytes werden kopiert.
    bool pushTransmitQueue(const Frame &frame);

    // Interne Steuerbyte-Schnittstelle für den DataLinkLayer - nach außen gibt es dafür die semantischen
    // Methoden (reset(), startMonitoring(), powerControl(), ...). Die Gruppe wird garantiert ununterbrochen
    // abgesetzt und nie von einem Telegramm- oder Acknowledge-Byte zerteilt.
    bool queueControl(uint8_t code);
    bool queueControl(const uint8_t *codes, size_t length);

    // Aus dem Empfangspfad (Receiver::sendAcknowledge), also aus dem Tick: schreibt das U_Ackn.req. Die
    // Entscheidung, OB und WAS quittiert wird, fällt dort - hier geht es nur noch raus. Liefert true, wenn
    // das Byte tatsächlich geschrieben wurde.
    bool sendAcknowledge(AckType acknowledge);

    // Ein vollständiges Echo unseres eigenen Telegramms ist eingelaufen (aus dem Tick, Receiver): der Chip
    // hat es gerade auf den Bus gelegt, die Bestätigung steht also noch aus. Setzt die Frist des Wachhunds
    // neu auf - siehe TPUART_TX_CONFIRM_TIMEOUT_MS.
    void echoReceived();

    // Die BCU hat den Versand bestätigt (L_Data.con) - der Sendeweg ist frei, egal wie sie ausfiel.
    void confirmed();

    // Ein Reset hat den Sendepuffer der BCU geräumt: ein laufendes Telegramm beginnt von vorn.
    void restart();

    // Räumt den Sendeweg: laufende Übertragung abbrechen und die Warteschlange verwerfen (Wechsel in den
    // Busmonitor). NUR aus dem Tick - die Begründung steht an der Definition.
    void abort();

    // Ist die übergebene Sequenz das Echo unseres eigenen Telegramms? Der Chip gibt jedes gesendete Oktett
    // an den Host zurück (NCN5130 S. 42), das eigene Telegramm läuft also wie ein fremdes ein. Verlangt
    // wird ein VOLLSTÄNDIGES Telegramm - gebraucht wird die Antwort erst, wenn eines fertig ist (TX-Flag,
    // und ob auf die Bestätigung zu warten ist).
    bool isEcho(const uint8_t *data, size_t length) const;

    // Dasselbe für ein Telegramm, das erst zum Teil eingelaufen ist - gebraucht bei Byte 6, wo über die
    // Quittung entschieden wird. Begründung an der Definition.
    bool isEchoPrefix(const uint8_t *data, size_t length) const;

    // Gibt den abgeholten Platz frei, räumt im Busmonitor und legt das nächste Telegramm für den Tick
    // bereit. NUR AUS DEM HAUPTKONTEXT - die Warteschlange gehört ihm allein.
    void stageNextTelegram();

    // Der Wachhund hat zugeschlagen: keine Bestätigung zum gesendeten Telegramm, die BCU wurde
    // zurückgesetzt. Gesetzt im Tick, gemeldet aus loop() - im Tick darf nichts nach außen dringen.
    bool confirmTimeout();

  public:
    explicit Transmitter(DataLinkLayer &dll);
    ~Transmitter();

    TxState state() const;
    bool isTransmitting() const;

    // Belegte BYTES der Sendewarteschlange, gegen queueSize() zu lesen. Mitgezählt ist auch ein bereits
    // vorgelegtes Telegramm: erst loop() gibt dessen Platz wieder frei.
    uint32_t queueUsed() const;
    uint32_t queueSize() const;
};

} // namespace TPUart
