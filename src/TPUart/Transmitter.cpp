#include "TPUart/Transmitter.h"

#include <stdlib.h>
#include <string.h>

#include <Arduino.h>

#include "TPUart/DataLinkLayer.h"
#include "TPUart/Interface/Abstract.h"
#include "TPUart/Statistics.h"

namespace TPUart
{

// Mindestens ein maximales Telegramm plus die Reserve muss hineinpassen - sonst könnte der Puffer nie ein
// vollständiges Telegramm aufnehmen und jeder Versand scheiterte stumm an der Kapazität.
static_assert(TPUART_TX_BUFFER_SIZE >= TPUART_BUFFER_SIZE + TPUART_TX_PRIORITY_RESERVE,
              "TPUART_TX_BUFFER_SIZE muss ein maximales Telegramm zusätzlich zur Reserve fassen");

Transmitter::Transmitter(DataLinkLayer &dll) : _dll(dll) {}

// Es gibt nichts mehr freizugeben: die Warteschlange ist ein statischer Bytepuffer. Der Destruktor stand
// hier, solange jedes wartende Telegramm ein eigener malloc-Block war.
Transmitter::~Transmitter() {}

// ---------------------------------------------------------------------------------------------------
// Zeitkritische Seite - läuft später aus einem Timer-Interrupt
// ---------------------------------------------------------------------------------------------------

// Der einzige Schreibzugriff auf die Schnittstelle in dieser Klasse - siehe Header. Zählt mit, damit
// Statistics::getTxBytes() die Auslastung der Hostleitung wirklich vollständig abbildet.
bool Transmitter::writeByte(uint8_t value)
{
    if (!_dll._interface->write((char)value)) return false;

    _dll._statistics.incrementTxBytes();
    return true;
}

// Sendet eine Steuersequenz oder ein Telegramm. Steuercodes haben Vorrang: sie sind kurz, zeitnah
// erwartet und dürfen sich laut Protokoll zwischen zwei Telegrammbytes schieben (jedes Telegrammbyte
// trägt sein eigenes Positionsbyte). Pro Tick geht höchstens eines von beidem raus.
void Transmitter::process()
{
    // Liegt ein Steuercode an, gehört ihm dieser Tick - auch wenn er gerade noch nicht abgesetzt werden
    // konnte. Der Telegrammpfad muss dann warten, sonst belegt er den Platz, auf den der Steuercode wartet.
    if (processCtrlQueue()) return;

    // IM BUSMONITOR RUHT DER GANZE SENDEPFAD - Telegrammbytes wie Wachhund. Der Wächter steht deshalb VOR
    // dem Await-Zweig und vor der Zustandsprüfung, nicht nur in startNextTransmission():
    //
    //   - Ein Telegramm, das beim Umschalten bereits LÄUFT (_state == Transmit), schöbe sonst seine
    //     restlichen Oktette in einen Chip, der sie laut Tabelle 11 (S. 32) ignoriert - U_L_DataStart/
    //     Cont/End.req sind dort "I".
    //   - Steht der Versand in Await, käme das L_Data.con im Busmonitor nicht mehr. Der Wachhund liefe ab
    //     und schickte U_Reset.req - und DER BEENDET DEN BUSMONITOR, ohne dass irgendwo steht warum. Der
    //     Anwender schaltet ihn ein und er ist zehn Sekunden später weg.
    //
    // Beides bleibt einfach stehen: in Transmit greift der Wachhund ohnehin nicht (seine Frist wird erst
    // beim Übergang nach Await aufgezogen), und in Await friert er hier ein. Verlassen wird der Busmonitor
    // nur per Reset, und dessen U_Reset.ind startet ein anstehendes Telegramm von vorn - es geht nichts
    // verloren. Die Steuercode-Warteschlange oben ist bewusst NICHT betroffen: über sie geht der Reset raus,
    // der der einzige Weg hinaus ist.
    if (_dll.isBusMonitor()) return;

    // Das Telegramm ist raus, es fehlt die Bestätigung der BCU. Sie kommt als L_Data.con und wird im
    // Empfangspfad ausgewertet (Receiver::processControlByte) - hier bleibt nur die Notbremse, damit ein
    // ausbleibendes L_Data.con den Sendeweg nicht für immer belegt.
    if (_state == TxState::Await)
    {
        if ((uint32_t)(millis() - _awaitSince) < TPUART_TX_CONFIRM_TIMEOUT_MS) return;

        // Keine Bestätigung. Was im Sendepuffer der BCU noch liegt, ist damit unbekannt - der einzige Weg
        // zu einem definierten Zustand ist der Reset. Der Zustand bleibt vorerst Await; wieder aufgenommen
        // wird der Versand von der U_Reset.ind aus, und die kommt hierauf genauso wie auf jeden anderen
        // Reset. Bleibt auch sie aus, läuft diese Frist erneut ab und der Reset geht noch einmal raus -
        // gegen eine stumme BCU gibt es nichts Besseres als weiter zu versuchen.
        if (_dll._interface->availableForWrite() < 1) return;
        if (!writeByte(U_RESET_REQ)) return;

        _dll._statistics.incrementTxControlBytes();
        _dll._statistics.incrementTxConfirmTimeouts();
        _awaitSince = millis();
        _confirmTimeout = true; // gemeldet wird das aus loop(), hier darf nichts nach außen

        // Wie bei jedem abgesetzten Steuercode: der DataLinkLayer leitet daraus den Chip-Zustand ab (der
        // Reset beendet den Busmonitor und macht eine laufende Empfangssequenz unlesbar).
        _dll.controlByteSent(U_RESET_REQ);
        return;
    }

    // Sendeweg frei? Dann das nächste Telegramm aus der Warteschlange holen. Ist keines da, gibt es nichts
    // zu tun - das ist der Normalfall und der häufigste Pfad hier.
    if (_state == TxState::Idle && !startNextTransmission()) return;

    bool last = (_bufferPos == _frameSize - 1); // das letzte Byte ist die Prüfsumme
    uint8_t offset = (uint8_t)(_bufferPos >> 6);

    // Ob das Offset-Byte mit muss, entscheidet sich VOR der Platzprüfung - und das ist der Unterschied
    // zwischen 3 und 2 verlangten Bytes. Gebraucht wird es bei einem 263-Oktett-Telegramm fünfmal, bei
    // allen übrigen 258 Positionen nicht.
    //
    // WARUM DAS MESSBAR IST: der Ring unter uns fasst vier Bytes und leert sich mit 286µs je Byte. Wer
    // pauschal drei freie Bytes verlangt, darf nur schreiben, solange höchstens EINES belegt ist - und
    // muss einen Tick aussetzen, sobald zwei drinstehen. Bei 500µs Takt kostete das gemessene 19ms je
    // Telegramm (143ms statt 124ms); auf Kern 1 mit 84000 Ticks/s fiel es nicht auf, weil dort der
    // nächste Versuch sofort kam. Verlangt wird jetzt, was wirklich gebraucht wird.
    //
    // An der Unteilbarkeit ändert das nichts: die zwei bzw. drei Bytes gehören weiterhin zusammen und
    // werden erst geschrieben, wenn sie vollständig passen.
    bool needsOffset = _dll.bcuType() == BcuType::Ncn5120 && (!_chipOffsetValid || offset != _chipOffset);
    size_t needed = needsOffset ? 3 : 2;

    if (_dll._interface->availableForWrite() < needed) return;

    // NUR beim NCN512x, siehe needsOffset oben. Der TPUART2 kennt den Dienst nicht - seine Servicetabelle
    // (docs/Siemens_TPUART.pdf, S. 21) geht von U_L_DataContinue (Index 1...62) direkt zu U_L_DataEnd (Länge 7...63),
    // der Opcode 0x08 ist dort nicht vergeben und wäre ein unbekanntes Steuerbyte. Gebraucht wird er dort
    // auch nie: mehr als 64 Oktette kann der Chip ohnehin nicht senden, der Index bleibt also unter 64.
    if (needsOffset)
    {
        writeByte((uint8_t)(U_L_DATA_OFFSET_REQ | offset));
        _chipOffset = offset;
        _chipOffsetValid = true;
    }

    writeByte((uint8_t)((last ? U_L_DATA_END_REQ : U_L_DATA_START_REQ) | (_bufferPos & U_L_DATA_POSITION_MASK)));
    writeByte(_buffer[_bufferPos]);

    _bufferPos++;

    if (!last) return;

    // Mit dem U_L_DataEnd.req beginnt der Chip die Übertragung auf den Bus.
    _dll._statistics.incrementTxFrames();
    _awaitSince = millis();
    _state = TxState::Await;
}

// Aus dem Tick. Kopiert das vorgelegte Telegramm in den Sendepuffer. Der Platz in der Warteschlange wird
// hier NICHT freigegeben - das erledigt stageNextTelegram() aus loop(), der die Queue allein gehört.
bool Transmitter::startNextTransmission()
{
    // Was noch vorliegt, ist im Busmonitor überholt - der Chip sendet dort nicht, und heraus kommt man nur
    // per Reset. Verworfen wird es hier, damit der Hauptkontext den Platz freigeben und räumen kann.
    if (_dll.isBusMonitor())
    {
        if (_stagedSeq != _takenSeq) _takenSeq = _takenSeq + 1;
        return false;
    }

    // Nichts vorgelegt - der Hauptkontext legt beim nächsten loop() nach. Er hat dafür die gesamte Dauer
    // der laufenden Übertragung Zeit (20ms und mehr), die Vorlage ist also praktisch immer da.
    if (_stagedSeq == _takenSeq) return false;

    // Kann im geordneten Betrieb nicht vorkommen (die Aufnahmeprüfung lässt nur intakte Telegramme
    // herein). Bleibt stehen, weil eine überlange Länge hier sonst über den Sendepuffer hinausschriebe.
    if (_stagedBuffer == nullptr || _stagedFrameSize == 0 || _stagedFrameSize > TPUART_BUFFER_SIZE)
    {
        _takenSeq = _takenSeq + 1;
        return false;
    }

    // memcpy und keine Byteschleife: das hier läuft im Tick, auf dem RP2040 also im Interrupt, und kopiert
    // bis zu 263 Bytes. Die Quelle liegt an einem beliebigen Byte-Offset im Warteschlangenpuffer, es wird
    // also nicht durchweg wortweise gehen - schneller als Byte für Byte ist es trotzdem, und die Laufzeit
    // eines Ticks ist genau die Zahl, an der die IRQ-Priorität dieser Schicht hängt.
    _frameSize = _stagedFrameSize;
    memcpy(_buffer, _stagedBuffer, _frameSize);

    // ZULETZT: ab hier darf der Hauptkontext den Platz freigeben und neu vorlegen. Der Tick sendet aus
    // seinem eigenen _buffer, die Bytes der Warteschlange werden also nicht mehr gebraucht - auch nicht
    // für restart() nach einem Reset.
    _takenSeq = _takenSeq + 1;

    beginTransmission();
    return true;
}

// Der Anfang einer Übertragung, für beide Fälle: ein frisch geholtes Telegramm und eines, das nach einem
// Reset von vorn beginnt. Identisch, weil der Sendepuffer in beiden Fällen derselbe ist - nur der Fortschritt
// darin geht auf null, und der Offset im Chip gilt als unbekannt.
void Transmitter::beginTransmission()
{
    _bufferPos = 0;
    _chipOffsetValid = false;
    _state = TxState::Transmit;
}

// Setzt höchstens eine vollständige Steuersequenz ab. Die Gruppe geht immer ungeteilt raus - deshalb wird
// vorab geprüft, ob das Interface sie komplett annehmen kann. Ein Zerteilen würde die BCU die Reste als
// eigenständigen Befehl deuten lassen.
//
// Der Rückgabewert heißt "der Sendeweg gehört in diesem Tick den Steuercodes" - true also auch dann, wenn
// die Gruppe noch nicht passt. Sonst wäre der Vorrang nur die halbe Miete: der Telegrammpfad greift sich
// drei Bytes, sobald drei frei sind, und im 4-Byte-Ring der RP2040 würden dann nie vier Bytes am Stück
// frei. Eine viergliedrige Gruppe (U_SetAddress.req, U_SetRepetition.req) käme während eines laufenden
// Telegramms überhaupt nicht mehr durch. Lässt der Telegrammpfad stattdessen aus, läuft der Ring in den
// nächsten Ticks leer und die Gruppe geht raus - Vorrang bedeutet also auch, Platz zu machen.
bool Transmitter::processCtrlQueue()
{
    if (_ctrlQueueTail == _ctrlQueueHead) return false;

    uint32_t tail = _ctrlQueueTail;
    size_t length = _ctrlQueue[tail % TPUART_CTRL_QUEUE_SIZE];

    // Kann im geordneten Betrieb nicht vorkommen (queueControl() prüft beim Einstellen). Bleibt stehen,
    // damit ein einzelner korrupter Eintrag nicht in eine Endlosschleife über Datenmüll führt.
    if (length == 0 || length > TPUART_CTRL_MAX_GROUP)
    {
        // Gemeldet wird das als Verlust von Steuercodes - was es auch ist, wenn auch aus anderem Grund als
        // ein voller Ring. Genauer geht es von hier aus nicht: der Tick darf nichts ausgeben.
        _ctrlQueueTail = _ctrlQueueHead;
        _dll.reportControlOverflow();
        return false;
    }

    // Noch kein Platz für die ganze Gruppe: nichts halb absetzen, aber den Sendeweg für diesen Tick auch
    // nicht freigeben (siehe oben).
    if (_dll._interface->availableForWrite() < length) return true;

    tail++;
    uint8_t code = _ctrlQueue[tail % TPUART_CTRL_QUEUE_SIZE];

    for (size_t i = 0; i < length; i++)
        writeByte(_ctrlQueue[tail++ % TPUART_CTRL_QUEUE_SIZE]);

    _ctrlQueueTail = tail;
    _dll._statistics.incrementTxControlBytes((uint32_t)length);

    // Erst jetzt gilt der Chip als umgeschaltet - vorher hätten wir einen Zustand angenommen, in dem er
    // noch gar nicht war. Abgeleitet aus dem tatsächlich gesendeten Code statt aus einem separaten
    // Wunsch-Feld: ein solches Feld wäre Nutzlast und müsste vor der Veröffentlichung geschrieben werden -
    // genau diese Reihenfolge war einmal falsch, mit dem Ergebnis, dass Chip-Zustand und Busmonitor-Flag
    // dauerhaft auseinanderliefen.
    _dll.controlByteSent(code);

    return true;
}

// Aus dem Empfangspfad und damit aus dem Tick. Entschieden hat der Receiver, hier geht das Byte nur noch
// raus - der TxState bleibt unberührt, ein Acknowledge ist kein Telegrammversand.
bool Transmitter::sendAcknowledge(AckType acknowledge)
{
    if (_dll._interface->availableForWrite() < 1) return false; // kein Platz - lieber nicht acken als blockieren

    if (!writeByte((uint8_t)(U_ACKN_REQ | (uint8_t)acknowledge))) return false;

    _dll._statistics.incrementTxAcknowledges();
    return true;
}

// Nur im Wartezustand von Belang: während der Übergabe läuft die Frist ohnehin noch nicht, und wenn sie
// nicht läuft, gibt es nichts aufzuschieben.
void Transmitter::echoReceived()
{
    if (_state != TxState::Await) return;

    _awaitSince = millis();
}

void Transmitter::confirmed()
{
    if (_state != TxState::Await) return;

    _state = TxState::Idle;
}

void Transmitter::restart()
{
    if (_state == TxState::Idle) return; // nichts unterwegs - dann gibt es auch nichts zu wiederholen

    beginTransmission();
}

// Räumt den gesamten Sendeweg: die laufende Übertragung UND alles, was noch in der Warteschlange steht.
// Gebraucht wird das beim Wechsel in den Busmonitor. Der Chip nimmt dort keine Telegrammdienste mehr an,
// und was er von einem laufenden Telegramm schon geschluckt hat, ist mit dem Moduswechsel hinfällig - ein
// halbes Telegramm später fortzusetzen oder zu wiederholen ergibt keinen Sinn, auf dem Bus hat nie jemand
// einen Anfang gesehen. Deshalb ABBRECHEN und nicht wie in restart() von vorn beginnen.
//
// LÄUFT IM TICK, und daran hängt die Korrektheit: _state behält damit genau einen Schreiber. Aus dem
// Hauptkontext ginge es nicht, dort kollidierte es mit confirmed() und echoReceived(), die der Empfangspfad
// aus demselben Tick heraus ruft.
//
// GERÄUMT WIRD HIER TROTZDEM NUR DIE VORLAGE. Die Warteschlange gehört dem Hauptkontext allein und wird von
// stageNextTelegram() geleert - diese Aufteilung ist es, die den früheren Wettlauf auflöst: solange der Tick
// _queueTail selbst vorzog, konnte die Bedingung dafür ("im Busmonitor fasst der Tick die Queue nicht an")
// zwischen Prüfung und Schreiben wegfallen, weil eine eintreffende U_Reset.ind _busMonitor löscht. Auf dem
// ESP32 läuft der Tick echt parallel, das war also kein reines Verdrängungsproblem.
//
// _chipOffsetValid wird mit zurückgesetzt, weil der Offset im Register des Chips liegt und über den
// Moduswechsel hinweg nicht mehr als bekannt gelten darf - dieselbe Überlegung wie in beginTransmission().
void Transmitter::abort()
{
    if (_stagedSeq != _takenSeq) _takenSeq = _takenSeq + 1;

    if (_state == TxState::Idle) return;

    _bufferPos = 0;
    _frameSize = 0;
    _chipOffsetValid = false;
    _state = TxState::Idle;
}

// Verglichen wird das VOLLSTÄNDIGE Telegramm. Für den halb eingelaufenen Fall gibt es isEchoPrefix()
// direkt darunter, und das ist KEIN toter Code: Receiver::sendAcknowledge() braucht es bei Byte 6, damit
// das eigene Echo nicht quittiert wird. Wer es entfernt, holt sich vier Quittungen je eigenem Telegramm
// zurück - am IP-Router gemessen.
//
// Zwei Bytes bleiben beim Vergleich außen vor, und beide aus demselben Grund: die BCU löscht beim
// Wiederholen das Wiederholungs-Bit im Kontrollbyte. Das wird deshalb maskiert verglichen, und die
// Prüfsumme am Ende gar nicht - sie hängt an eben diesem Byte.
bool Transmitter::isEcho(const uint8_t *data, size_t length) const
{
    if (_state == TxState::Idle) return false;
    if (length != _frameSize) return false;
    if (((data[0] ^ _buffer[0]) & (uint8_t)~0x20) != 0) return false;

    return memcmp(data + 1, _buffer + 1, length - 2) == 0;
}

// DASSELBE FÜR EIN NOCH LAUFENDES TELEGRAMM, gebraucht bei Byte 6 für die Quittungsentscheidung: dort ist
// das Echo erst zur Hälfte da, isEcho() kann also nicht greifen (es verlangt die vollständige Länge und
// vergleicht die Prüfsumme mit).
//
// Verglichen wird der ANFANG, und das genügt zur Unterscheidung: die ersten Oktetts tragen die Quelladresse,
// und ein fremdes Telegramm mit UNSERER Quelladresse wäre eine doppelt vergebene physikalische Adresse -
// ein Anlagenfehler, nicht ein Fall, gegen den diese Prüfung sich wappnen müsste.
//
// Das Wiederholungsbit bleibt ausgenommen (0x20): der Chip löscht es, wenn er das Telegramm wiederholt, und
// jede Wiederholung wird genauso zurückgespiegelt wie der erste Versuch.
bool Transmitter::isEchoPrefix(const uint8_t *data, size_t length) const
{
    if (_state == TxState::Idle) return false;
    // STRIKT KÜRZER: ein vollständiges Telegramm ist kein Prefix, dafür ist isEcho() da. Bei Gleichheit
    // würde hier auch das Prüfsummen-Oktett verglichen, und das unterscheidet sich bei einer Wiederholung
    // (die BCU löscht das Wiederholungsbit im Steuerbyte, die Prüfsumme hängt daran) - der Vergleich
    // schlüge also fehl, wo isEcho() zutrifft.
    if (length == 0 || length >= _frameSize) return false;
    if (((data[0] ^ _buffer[0]) & (uint8_t)~0x20) != 0) return false;

    return memcmp(data + 1, _buffer + 1, length - 1) == 0;
}

// ---------------------------------------------------------------------------------------------------
// Gemütliche Seite - läuft aus dem Hauptloop
// ---------------------------------------------------------------------------------------------------

// Aus dem Hauptkontext, dem die Warteschlange allein gehört - der Tick sieht nur die Vorlage, die
// stageNextTelegram() am Ende setzt.
bool Transmitter::pushTransmitQueue(const Frame &frame)
{
    size_t length = frame.length();

    // Jede Ablehnung nennt ihren Grund selbst. Der Aufrufer bekommt nur ein bool und kann ihn deshalb
    // nicht kennen - eine geratene Begründung in der Anwendung ist schlimmer als gar keine.
    if (!_dll.isConnected())
    {
        _dll.printError("Send rejected: no connection");
        return false;
    }

    if (_dll.isBusMonitor()) // dort ist der Chip transparent und sendet nichts
    {
        _dll.printError("Send rejected: bus monitor active");
        return false;
    }

    // Ein vollständiges Standard-Telegramm hat mindestens 8 Oktetts einschließlich Prüfsumme.
    if (length < 8 || length > TPUART_BUFFER_SIZE)
    {
        _dll.printError("Send rejected: length %u out of range (8..%u)", (unsigned)length, (unsigned)TPUART_BUFFER_SIZE);
        return false;
    }

    // Der TPUART2 kann nur 63 Nutzoktette plus Prüfsumme - dieselbe Grenze wie in der alten Library, und
    // sie steckt in seiner Servicetabelle (docs/Siemens_TPUART.pdf, S. 21): U_L_DataContinue geht bis 0xBE, also
    // Index 62, und U_L_DataEnd bis 0x7F, also Länge 63. Damit sind die Datenoktette 0...62 (das sind 63)
    // und die Prüfsumme auf Index 63 - zusammen 64 Oktette. Beim NCN512x verschiebt U_L_DataOffset.req
    // dieses Fenster, dort sind es bis zu 263.
    if (_dll.bcuType() == BcuType::Tpuart2 && length > 64)
    {
        _dll.printError("Send rejected: TPUART2 takes at most 63 byte plus checksum");
        return false;
    }

    // DIE PRÜFUNG LÄUFT VOR JEDER VERÄNDERUNG AM PUFFER, und das ist keine Stilfrage: die Warteschlange
    // führt kein Längenpräfix, sie leitet die Grenze zwischen zwei Einträgen aus dem Telegramm selbst ab.
    // Käme etwas hinein, dessen Kopf nicht zur Länge passt, begänne der nächste Eintrag an der falschen
    // Stelle und alles dahinter wäre Unsinn. Weil hier abgelehnt wird, BEVOR reserviert wird, hat ein
    // zurückgewiesenes Telegramm den Puffer nie angefasst - das braucht kein Rückabwickeln.
    //
    // Geprüft werden Steuerbyte, Länge und Prüfsumme (siehe Frame::isValid). Die Prüfsumme wird NICHT mehr
    // selbst gerechnet und angehängt: sie gehört zum Telegramm, und sie stillschweigend zu überschreiben
    // verdeckte einen Fehler im Aufrufer - das Telegramm ginge dann mit korrekter CRC über falschem Inhalt
    // hinaus.
    if (!frame.isValid())
    {
        _dll.printError("Send rejected: not a well-formed telegram");
        return false;
    }

    if (!_queue.push(frame))
    {
        _dll._statistics.incrementTxQueueOverflows();
        _dll.printError("Send rejected: queue full (%u byte)", (unsigned)TPUART_TX_BUFFER_SIZE);
        return false;
    }

    _dll._statistics.updateTxQueuePeakBytes(_queue.used());

    // Liegt nichts vor, kann das hier gleich geschehen - sonst wartete das erste Telegramm eine ganze
    // Loop-Periode, obwohl der Sendeweg frei ist.
    stageNextTelegram();
    return true;
}

// AUS DEM HAUPTKONTEXT, und hier läuft alles zusammen, was die Warteschlange betrifft: freigeben, was der
// Tick abgeholt hat, gegebenenfalls räumen, und das nächste Telegramm vorlegen.
//
// Die Reihenfolge ist nicht beliebig. Geräumt wird VOR dem Vorlegen - sonst legte man im Busmonitor gerade
// das vor, was man eben verwerfen wollte. Und der gepinnte Eintrag fällt erst, wenn _takenSeq nachgezogen
// hat; bis dahin liest der Tick womöglich noch daraus.
void Transmitter::stageNextTelegram()
{
    if (_stagedSeq != _takenSeq) return; // die Vorlage liegt noch, der Tick hat sie nicht abgeholt

    // Der Tick hat übernommen (oder verworfen) - jetzt darf der Platz weg. pop() kompaktiert dabei, es
    // bleibt also kein totes Gebiet am linken Rand zurück.
    if (_queue.pinned()) _queue.pop();

    _stagedBuffer = nullptr;
    _stagedFrameSize = 0;

    // Im Busmonitor ist alles Wartende überholt: heraus kommt man nur per Reset, und was danach hinausginge,
    // wäre längst veraltet. Räumen darf hier nur der Hauptkontext - dass ihm die Warteschlange allein
    // gehört, ist genau das, was den früheren Wettlauf an dieser Stelle auflöst.
    if (_dll.isBusMonitor()) _queue.clear();

    size_t length = 0;
    const uint8_t *data = _queue.front(length);

    // ERST NACH front(), denn dort wird das Flag gesetzt. Davor abgefragt käme die Meldung einen
    // loop()-Durchlauf zu spät - und im ungünstigen Fall nie, wenn das Gerät genau dann stehenbleibt.
    if (_queue.corrupted())
        _dll.printError("Transmit queue corrupt - dropped");

    if (data == nullptr) return;

    _stagedBuffer = data;
    _stagedFrameSize = length;
    _queue.pin(); // ab jetzt bewegt den Eintrag nichts mehr, auch keine Aufnahme

    _stagedSeq = _stagedSeq + 1; // ZULETZT - erst damit wird die Vorlage für den Tick sichtbar
}

bool Transmitter::queueControl(uint8_t code)
{
    return queueControl(&code, 1);
}

// Stellt eine Steuersequenz in die Warteschlange. Anders als früher wird NICHT abgelehnt, solange gerade
// gesendet wird: Steuercodes sind kein Telegrammversand und müssen auch dann durchkommen. Genau deshalb
// haben sie eine eigene Warteschlange und belegen keinen TxState mehr.
bool Transmitter::queueControl(const uint8_t *codes, size_t length)
{
    if (!_dll.isConnected()) return false; // vorher regelt die Verbindungsaufnahme den Chip
    if (length == 0 || length > TPUART_CTRL_MAX_GROUP) return false;

    uint32_t needed = (uint32_t)(1 + length);
    uint32_t used = _ctrlQueueHead - _ctrlQueueTail;

    if (TPUART_CTRL_QUEUE_SIZE - used < needed)
    {
        _dll.reportControlOverflow();
        return false;
    }

    _dll._statistics.updateTxControlQueuePeakBytes(used + needed);

    // Ein Produzent (dieser Kontext), ein Konsument (tick()). Der Kopf wird als ALLERLETZTES weitergesetzt,
    // damit der Tick nie eine halb geschriebene Sequenz sieht. Keine Sperre nötig, keine Besitzübergabe.
    uint32_t head = _ctrlQueueHead;
    _ctrlQueue[head++ % TPUART_CTRL_QUEUE_SIZE] = (uint8_t)length;

    for (size_t i = 0; i < length; i++)
        _ctrlQueue[head++ % TPUART_CTRL_QUEUE_SIZE] = codes[i];

    _ctrlQueueHead = head;
    return true;
}

TxState Transmitter::state() const
{
    return _state;
}

bool Transmitter::isTransmitting() const
{
    return _state != TxState::Idle;
}

// BYTES, nicht Telegramme - die Warteschlange misst sich seit dem Umbau in Bytes, weil ein gewöhnliches
// Telegramm nur rund 5% eines maximalen ausmacht und feste Plätze den Rest verschenkt hätten.
uint32_t Transmitter::queueUsed() const
{
    return (uint32_t)_queue.used();
}

uint32_t Transmitter::queueSize() const
{
    return (uint32_t)TPUART_TX_BUFFER_SIZE;
}

bool Transmitter::confirmTimeout()
{
    if (!_confirmTimeout) return false;

    _confirmTimeout = false;
    return true;
}

} // namespace TPUart
