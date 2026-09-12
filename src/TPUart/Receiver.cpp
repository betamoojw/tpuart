#include "TPUart/Receiver.h"

#include <string.h>

#include <Arduino.h>

#include "TPUart/DataLinkLayer.h"
#include "TPUart/Interface/Abstract.h"
#include "TPUart/Statistics.h"
#include "TPUart/Transmitter.h"

namespace TPUart
{

// Jedes GÜLTIGE Frame passt in den Puffer: Standard = 8 + (Längen-Nibble, max 15) = 23,
// Extended = 9 + (Längenbyte, max 254) = 263. Hier festgenagelt, damit ein Verkleinern des Puffers den
// Build bricht statt still zu überlaufen. Ein Längenbyte von 255 ist reserviert und ergäbe 264 - dieser
// Fall ist damit nicht abgedeckt, sondern wird zur Laufzeit als Fehler behandelt.
static_assert(TPUART_BUFFER_SIZE >= 9 + 254, "TPUART_BUFFER_SIZE muss das größtmögliche gültige Extended-Frame fassen");

// Ein einzelner Eintrag muss überhaupt in die Warteschlange passen, sonst käme nie etwas an.
static_assert(TPUART_RX_QUEUE_SIZE > TPUART_BUFFER_SIZE + TPUART_RX_QUEUE_HEADER_SIZE, "TPUART_RX_QUEUE_SIZE muss mindestens ein größtmögliches Telegramm fassen");

Receiver::Receiver(DataLinkLayer &dll) : _dll(dll) {}

// ---------------------------------------------------------------------------------------------------
// Zeitkritische Seite - läuft später aus einem Timer-Interrupt
// ---------------------------------------------------------------------------------------------------

void Receiver::process()
{
    size_t pending = _dll._interface->available();
    if (!pending)
    {
        checkPause();
        return;
    }

    // Der Rückstand in Bytes - gratis, weil available() ohnehin gefragt wurde. Gesund sind 0-1: der Bus
    // liefert höchstens alle 1,354ms ein Byte, der Tick holt alle 500µs eines ab. Alles darüber ist ein
    // Aussetzer des Antriebs, und ab 2 wird bei Byte 6 die Quittung unterdrückt.
    _dll._statistics.updateRxInterfacePeakBytes((uint32_t)pending);

    int value = _dll._interface->read();
    if (value < 0)
    {
        checkPause();
        return;
    }

    // Überlauf des Interfaces nur hier abfragen, nicht bei jedem Tick: entstehen kann er ausschließlich,
    // während Daten fließen, und der Leerlaufpfad soll frei von Hardwarezugriffen bleiben. Eingesammelt
    // wird er ausschließlich hier, damit der Hauptkontext das Interface nicht selbst anfassen muss - im
    // Timer-Betrieb wäre das ein zweiter Zugriffskontext, und overflow() löscht beim RP2040 nebenbei das
    // Hardware-Flag.
    if (_dll._interface->overflow()) _dll.reportInterfaceOverflow();

    _dll._statistics.incrementRxBytes();

    // Lebenszeichen der BCU. Jedes Byte zählt, nicht nur die Antwort auf eine Statusabfrage: reicht sie
    // Bus-Verkehr durch, lebt sie offensichtlich.
    _dll._lastReceivedAt = millis();

    // Es fließen Daten - eine laufende Messung ist damit hinfällig.
    _emptyStarted = false;

    processByte((uint8_t)value);
}

// Wird nur aufgerufen, wenn das Interface aktuell nichts bereithält. Der Zeitpunkt der ersten solchen
// Beobachtung wird festgehalten; erst wenn dieser Zustand TPUART_FRAME_WAIT_US lang ununterbrochen anhält,
// ist es eine echte Pause auf dem Bus (und nicht bloß eine Lücke in unserer Abarbeitung).
void Receiver::checkPause()
{
    // NICHTS IM GANGE, NICHTS ZU MESSEN. Eine Pause hat überhaupt nur dann eine Wirkung, wenn eine Sequenz
    // offen ist: sie schließt ein angefangenes Telegramm ab, beendet einen Resync oder lässt eine
    // ausbleibende Antwort auffliegen. Zwischen zwei Telegrammen steht die Maschine auf Idle, und dort gibt
    // es nichts zu beenden - die Telegrammlänge kommt aus dem Längenbyte, nicht aus dem Timing.
    //
    // Das ist zugleich der billigste Leerlaufpfad, den es hier geben kann: auf einem ruhigen Bus ist Idle
    // der Normalzustand, und ein Tick kostet dann einen Vergleich statt eines micros()-Zugriffs. Und es
    // macht einen Merker "schon ausgelöst" überflüssig - handleVerifiedPause() endet in jedem Zweig bei
    // Idle, kann also von hier aus kein zweites Mal erreicht werden.
    if (_state == RxState::Idle) return;

    uint32_t now = micros();

    if (!_emptyStarted)
    {
        _emptyStarted = true;
        _emptySince = now;
        return;
    }

    // Ein fertiges Telegramm wartet noch auf seine Antwort - im Busmonitor die Quittung vom Bus, beim
    // eigenen Versand das L_Data.con. Die trifft später ein, als die Telegrammgrenze erkennbar wäre,
    // dieser Zustand bekommt deshalb die längere Frist.
    uint32_t threshold = (_state == RxState::FrameAck) ? TPUART_FRAME_ACK_US : TPUART_FRAME_WAIT_US;

    if ((uint32_t)(now - _emptySince) < threshold) return;

    handleVerifiedPause();
}

// Eine verifizierte Pause ist eine echte Frame-Grenze - danach ist der Bytestrom in jedem Fall wieder
// synchron. Sie beendet damit sowohl einen laufenden Resync als auch eine angebrochene Sequenz.
//
// JEDER ZWEIG ENDET IN IDLE, und genau daran hängt, dass checkPause() oben abkürzen darf: ein zweites Mal
// ist von dort aus nicht zu erreichen. Der default-Zweig ist damit unerreichbar und bleibt nur als
// Vollständigkeit stehen. Wer hier einen Zweig ergänzt, der NICHT in Idle endet, hebt diese Zusage auf.
void Receiver::handleVerifiedPause()
{
    switch (_state)
    {
        // Das Frame wartete noch auf seine Antwort (Quittung im Busmonitor bzw. L_Data.con zum eigenen
        // Versand). Beide kommen rund 15 Bitzeiten nach dem Frame und damit deutlich vor dieser Frist -
        // bleiben sie bis hierher aus, gab es keine. Das Telegramm wird ohne Quittungs-Flags gemeldet.
        // Der Sendeweg wird hier NICHT freigegeben: bei Wiederholungen kann die Bestätigung noch kommen,
        // darum kümmert sich der Wachhund im Transmitter.
        case RxState::FrameAck:
            completeSequence(0, RxState::Idle);
            return;

        // Die Pause hat die Übertragung vorzeitig beendet: das Frame ist unvollständig und damit kaputt.
        // Gemeldet wird trotzdem, was angekommen ist. Ein Resync ist danach NICHT nötig - die Pause ist
        // genau die Grenze, die ein Resync erst suchen würde.
        case RxState::Frame:
        case RxState::Control:
            completeSequence(TP_FRAME_FLAG_INVALID, RxState::Idle);
            return;

        // Ein Poll-Telegramm, das hier ankommt, ist in aller Regel NICHT abgeschnitten: der Chip reicht
        // regulär nur das Steuerbyte durch, der Rest des Zyklus bleibt auf dem Bus (siehe
        // L_POLL_DATA_IND). Genau dieser Fall darf nicht als Fehler gemeldet werden, sonst zählte jeder
        // Poll-Umlauf als kaputtes Telegramm. Steht dagegen schon mehr im Puffer, lief der Zyklus zum Host
        // und wurde unterwegs abgeschnitten - etwa weil ein Slot unbeantwortet blieb.
        case RxState::Poll:
            completeSequence(_bufferPos > 1 ? TP_FRAME_FLAG_INVALID : 0, RxState::Idle);
            return;

        // Das Ende des Resyncs. Die verworfenen Bytes werden nicht gemeldet.
        case RxState::Resync:
            resetSequence(RxState::Idle);
            return;

        default:
            return;
    }
}

void Receiver::processByte(uint8_t value)
{
    switch (_state)
    {
        // Position im Bytestrom unbekannt - alles verwerfen bis zur nächsten verifizierten Pause. Die
        // Bytes werden nicht einmal gepuffert, sie interessieren niemanden mehr. Gezählt werden sie
        // trotzdem: sie sind der Unterschied zwischen "kaputt, aber gemeldet" und "nie gesehen".
        case RxState::Resync:
            _dll._statistics.incrementRxDroppedBytes();
            return;

        case RxState::Control:
            _buffer[1] = value;
            _bufferPos = 2;
            _dll._statistics.incrementRxControlBytes(); // das zweite Byte eines U_SystemStat.ind
            completeSequence(0, RxState::Idle);
            return;

        // Erwartet wird die Antwort zum fertigen Telegramm. Sie landet NICHT im Telegramm - das Frame
        // bleibt unangetastet, die Antwort wird über die Flags mitgegeben.
        case RxState::FrameAck:
            // Busmonitor: die Quittung vom Bus (Figure 35).
            if (_dll.isBusMonitor() && (value & L_ACKN_MASK) == L_ACKN_IND)
            {
                // Beide Bit-Paare sind invertiert zu lesen: gesetzte Maskenbits heißen "nicht busy"/"nicht nack".
                bool nack = !(value & L_ACKN_NACK_MASK);
                bool busy = !(value & L_ACKN_BUSY_MASK);
                _flags |= acknowledgeFlags(nack ? AckType::Nack : (busy ? AckType::Busy : AckType::Addressed));

                completeSequence(0, RxState::Idle);
                return;
            }

            // Eigener Versand: die Bestätigung der BCU. Zwei Aussagen, zwei Flags - das wurde hier erst
            // zusammengeworfen, was "keine Bestätigung" und "negative Bestätigung" unterscheidbar machte:
            //   DATA_CON - es KAM eine Bestätigung (fehlt sie, lief die Frist ab, s. handleVerifiedPause)
            //   ACK      - und sie war positiv, das Telegramm wurde auf dem Bus also quittiert
            // Damit ist auch der dritte Fall der ACK-Bedeutung belegt: "quittiert - von wem auch immer",
            // hier als Antwort auf ein selbst gesendetes Telegramm.
            //
            // BUSY bleibt dabei ungesetzt, und das ist keine Auslassung: der L_Data.con trägt genau EIN
            // Bit (NCN5130 Table 13: "z = positive ('1') or negative ('0') confirmation"; Siemens S. 31
            // gleichlautend). Die BCU hat ihre Wiederholungen selbst abgearbeitet - bis zu 3x nach NACK,
            // bis zu 3x nach BUSY - und meldet nur das Endergebnis. N steht hier deshalb für "nicht
            // positiv quittiert", was ein NACK ebenso einschließt wie ein erschöpftes BUSY oder gar keine
            // Antwort. Die echte Unterscheidung gibt es nur im Busmonitor, wo das rohe Quittungsbyte
            // durchkommt (siehe oben).
            if ((value & L_DATA_CON_MASK) == L_DATA_CON)
            {
                // ACK heißt hier "es liegt eine Quittung vor" - die gibt es, sobald überhaupt eine
                // Bestätigung kam. N qualifiziert sie als negativ. Dieselbe Kombination liefert
                // acknowledgeFlags() für AckType::Nack, die Flags bedeuten also dasselbe wie beim
                // Busmonitor und bei der eigenen Quittung.
                uint8_t flags = TP_FRAME_FLAG_DATA_CON | TP_FRAME_FLAG_ACK;
                if (!(value & 0x80)) flags |= TP_FRAME_FLAG_ACK_NACK;

                // ERST melden, dann freigeben: completeSequence() fragt den Transmitter nach dem Echo, und
                // das setzt einen belegten Sendeweg voraus - andernfalls fehlte dem eigenen Telegramm das
                // TX-Flag.
                completeSequence(flags, RxState::Idle);

                _dll._transmitter.confirmed(); // Sendeweg frei, egal wie die Bestätigung ausfiel
                return;
            }

            // Im 8-Bit-UART-Modus geht dem L_Data.con ein U_FrameState.ind voraus (NCN5130 S. 42). Es wird
            // hier verworfen und weiter auf die Bestätigung gewartet - es gehört nicht zum Telegramm.
            if ((value & U_FRAME_STATE_MASK) == U_FRAME_STATE_IND) return;

            // Etwas Anderes - typischerweise der Anfang einer WIEDERHOLUNG, deren Echo genauso zurückkommt
            // wie das erste. Das fertige Telegramm wird gemeldet, und das Byte anschließend ganz normal
            // weiterverarbeitet: es ist der Anfang von etwas Neuem, nicht Müll. Ein Resync wie früher
            // würde hier das komplette Wiederholungs-Echo verwerfen.
            completeSequence(0, RxState::Idle);
            processByte(value);
            return;

        case RxState::Frame:
            processFrameByte(value);
            return;

        case RxState::Poll:
            processPollByte(value);
            return;

        case RxState::Idle:
        {
            bool isFrameStart = (value & L_DATA_MASK) == L_DATA_STANDARD_IND || (value & L_DATA_MASK) == L_DATA_EXTENDED_IND;

            if (!isFrameStart)
            {
                processControlByte(value);
                return;
            }

            resetSequence(RxState::Frame);
            processFrameByte(value);
            return;
        }

        default:
            return;
    }
}

void Receiver::processFrameByte(uint8_t value)
{
    // Das Byte, das jetzt geschrieben wird, ist die Prüfsumme, sobald die Frame-Größe bekannt ist und dies
    // die letzte erwartete Position ist - es fließt NICHT in die laufende CRC ein, sondern wird nur mit
    // ihr verglichen (inkrementelle CRC8, kein erneuter Durchlauf am Ende nötig).
    bool isChecksumByte = _frameSize > 0 && (_bufferPos == _frameSize - 1);

    // Die Grenze ist Absicherung, kein Fall: _frameSize ist auf TPUART_BUFFER_SIZE geprüft (darüber gilt
    // das Frame als kaputt), Steuersequenzen sind höchstens 2 Byte, im Resync wird gar nicht gepuffert.
    // Sie bleibt trotzdem stehen - ein Schreibzugriff hinter den Puffer träfe die States und die Callbacks.
    if (_bufferPos < TPUART_BUFFER_SIZE) _buffer[_bufferPos] = value;
    _bufferPos++;

    // JE BYTE GEZÄHLT, nicht am Sequenzende in einem Rutsch - siehe Statistics::getBusLoadPercent().
    // Sprunghaft zu zählen hieße, dass ein laufendes Telegramm 0 beiträgt und beim Abschluss auf einen
    // Schlag seine ganze Busbelegung mitbringt, auch den Teil, der vor dem Messfenster lag. Bei einem
    // maximalen Telegramm sind das 356ms, die im falschen Fenster landen.
    _dll._statistics.incrementRxFrameBytes();

    if (!isChecksumByte) _crc ^= value;

    if (_frameSize == 0)
    {
        // Die Ableitung gehört zu Frame - dort steht sie einmal und wird von hier, von Frame::size() und
        // von der Sendewarteschlange benutzt. Statisch, damit im Tick kein 263-Byte-Objekt entsteht.
        // 0 heißt hier "der Kopf ist noch nicht vollständig", nicht "Fehler".
        size_t size = Frame::sizeOf(_buffer, _bufferPos);

        if (size > 0)
        {
            _frameSize = size;

            // Passt nur dann nicht in den Puffer, wenn das Längenbyte den reservierten Wert 255 trägt -
            // die Länge ist damit korrupt und das Frame-Ende unbekannt. Weiterzusammeln wäre Raten, also
            // wird das bisher Empfangene als kaputt gemeldet und neu aufgesetzt. Standard-Frames können
            // diesen Zweig nie erreichen (max. 23 Byte).
            if (_frameSize > TPUART_BUFFER_SIZE)
            {
                completeSequence(TP_FRAME_FLAG_INVALID, RxState::Resync);
                return;
            }

            // Genau hier wird die Frame-Länge erstmals bekannt - der Zweig läuft daher pro Frame exakt
            // einmal, ein "schon geackt"-Flag ist nicht nötig. Zugleich der früheste Zeitpunkt, zu dem
            // die Entscheidung überhaupt fallen kann: Ziel, Adresstyp UND Restlänge stehen erst jetzt fest.
            sendAcknowledge();
        }
    }

    if (_frameSize > 0 && _bufferPos == _frameSize)
    {
        bool valid = (uint8_t)(~_crc) == _buffer[_frameSize - 1];

        // Bei fehlerhafter Prüfsumme ist unklar, wo das Frame wirklich endete - vielleicht war schon das
        // Längenbyte verfälscht. Erst nach einer verifizierten Pause kann wieder sicher aufgesetzt werden.
        if (!valid)
        {
            completeSequence(TP_FRAME_FLAG_INVALID, RxState::Resync);
            return;
        }

        // Auf eine Antwort wird in zwei Fällen gewartet, und es ist derselbe Vorgang: im Busmonitor folgt
        // dem Frame die Quittung vom Bus (L_Ackn.ind), beim eigenen Telegramm die Bestätigung der BCU
        // (L_Data.con). Erst damit ist das Telegramm vollständig beschrieben.
        bool echo = _dll._transmitter.isEcho(_buffer, _bufferPos);

        // Das Echo ist zugleich ein Lebenszeichen für den laufenden Versand: der Chip legt das Telegramm
        // gerade auf den Bus, die Bestätigung kann also noch nicht gekommen sein. Bei einer Wiederholung
        // kommt es erneut - und schiebt damit den Wachhund weiter, der sonst mitten in eine laufende
        // Übertragung greifen würde.
        if (echo) _dll._transmitter.echoReceived();

        if (_dll.isBusMonitor() || echo)
        {
            _state = RxState::FrameAck;
            return;
        }

        completeSequence(0, RxState::Idle);
    }
}

// Ein Poll-Telegramm - bewusst nach demselben Muster gebaut wie processFrameByte(), damit beide gleich zu
// lesen sind: Byte ablegen, Länge aus dem Kopf bestimmen, am Ende abschließen. Die Unterschiede sind der
// Aufbau des Kopfes (Steuerbyte, Quelle 2, Poll-Adresse 2, Slot Count, Prüfsumme) und dass die Prüfsumme
// mitten in der Sequenz steht statt am Ende - die Slots dahinter deckt sie nicht ab.
//
// HIER WIRD NIE QUITTIERT. Ein Poll trägt keinen Adresstyp und erwartet kein Acknowledge; geantwortet wird
// mit dem eigenen Slot-Byte, und dafür ist U_PollingState.req zuständig, das diese Library nicht benutzt.
//
// Der Normalfall endet nicht hier, sondern in handleVerifiedPause(): meist bleibt es bei dem einen
// Steuerbyte, weil der Chip den Rest gar nicht durchreicht (siehe L_POLL_DATA_IND).
void Receiver::processPollByte(uint8_t value)
{
    bool isChecksumByte = _bufferPos == L_POLL_DATA_HEADER_SIZE - 1;

    // Wie im Frame-Pfad reine Absicherung: die Sequenz ist auf 7 + 15 Byte begrenzt, ein Poll passt immer.
    if (_bufferPos < TPUART_BUFFER_SIZE) _buffer[_bufferPos] = value;
    _bufferPos++;

    // Poll zählt zu den Telegrammbytes, siehe processFrameByte() zur Begründung des Zählens je Byte.
    _dll._statistics.incrementRxFrameBytes();

    if (!isChecksumByte) _crc ^= value;

    if (isChecksumByte)
    {
        // Der Slot-Count steht direkt vor der Prüfsumme und legt fest, wie viele Bytes noch folgen.
        uint8_t slots = _buffer[L_POLL_DATA_HEADER_SIZE - 2];

        // Zwei Gründe, dem Kopf nicht zu glauben: mehr Slots, als es Slotnummern gibt, oder eine falsche
        // Prüfsumme. In beiden Fällen ist das Ende der Sequenz unbekannt - weiterzuzählen wäre Raten, also
        // Resync wie bei einem korrupten Längenbyte.
        if (slots > L_POLL_DATA_MAX_SLOTS || (uint8_t)(~_crc) != value)
        {
            completeSequence(TP_FRAME_FLAG_INVALID, RxState::Resync);
            return;
        }

        _frameSize = L_POLL_DATA_HEADER_SIZE + slots;
    }

    // Alle Slots da - fertig, und zwar OHNE auf eine Pause zu warten. Das ist der einzige Grund, warum
    // diese Sequenz überhaupt gezählt wird.
    if (_frameSize > 0 && _bufferPos == _frameSize) completeSequence(0, RxState::Idle);
}

// Sobald Zieladresse, Adresstyp und Restlänge feststehen, entscheidet der Callback über das Acknowledge und
// das U_Ackn.req geht sofort raus - noch mitten im laufenden Frame, denn der Chip legt den
// Immediate-Acknowledge direkt hinter dem Checksum-Byte auf den Bus. Das lässt den TxState unberührt: ein
// Acknowledge ist kein Telegrammversand, sondern geht zwischendurch raus.
void Receiver::sendAcknowledge()
{
    // Im Busmonitor ist der Chip transparent und quittiert grundsätzlich nichts - ein U_Ackn.req von uns
    // hätte dort nichts verloren und würde den passiven Mitschnitt verfälschen.
    if (_dll.isBusMonitor()) return;

    // DAS EIGENE ECHO WIRD NICHT QUITTIERT - man quittiert nicht sich selbst. Der Chip gibt jedes gesendete
    // Oktett an den Host zurück (Datenblatt S. 42: "Each transmitted data octet ... will also be transmitted
    // back to the host controller"), das eigene Telegramm läuft also wie ein fremdes ein und käme sonst hier
    // durch.
    //
    // Hier stand einmal das Gegenteil, mit der Begründung "an sich selbst schickt man nichts, der Callback
    // sagt also ohnehin 'nicht für mich'". AM ECHTEN GERÄT WIDERLEGT: ein IP-Router leitet auf
    // Gruppenadressen weiter, die in seiner eigenen Filtertabelle stehen - sein Callback sagt für das eigene
    // Echo "meins". Gemessen an einem Router über 15,7 Stunden: 25196 gesendete Quittungen, exakt 4 je
    // eigenem Telegramm (Original plus die drei Wiederholungen des Chips), und keine einzige für ein fremdes
    // Telegramm. Das ist nicht der exotische Sonderfall, für den es damals gehalten wurde, sondern der
    // Normalfall des Hauptverbrauchers dieser Library.
    //
    // Wirkung hatte das Byte auf dem Bus nie (der Chip verwirft ein U_Ackn.req, das während seines eigenen
    // Sendens eintrifft) - es war aber auch nicht gratis: es belegt einen der vier Plätze in
    // TPUART_TX_INTERFACE_BUFFER, und zwar genau in dem Moment, in dem der Sendepfad drei davon für das
    // nächste Oktett braucht. Und es machte getTxAcknowledges() als Diagnose wertlos.
    //
    // Geprüft wird der ANFANG gegen das laufende Telegramm, NICHT "ist der Sendeweg belegt" - letzteres wäre
    // aktiv schädlich: TxState::Await hält bis zur Bestätigung an, und in dieser ganzen Zeit würde kein
    // einziges FREMDES Telegramm mehr quittiert.
    if (_dll._transmitter.isEchoPrefix(_buffer, _bufferPos)) return;

    // Die Entscheidung wird IMMER eingeholt, auch wenn gleich klar wird, dass nicht mehr quittiert werden
    // kann: sie beantwortet zwei verschiedene Fragen. "Ist das Telegramm für uns" (-> ADDRESSED) gilt
    // unabhängig davon, ob wir es rechtzeitig bestätigen konnten - der Aufrufer soll das Telegramm auch
    // dann als an ihn gerichtet erkennen. Nur die Quittung selbst (ACK/BUSY/NACK) hängt am Versand.
    bool extended = (_buffer[0] & L_DATA_MASK) == L_DATA_EXTENDED_IND;
    bool isGroupAddress = extended ? (_buffer[1] & 0x80) != 0 : (_buffer[5] & 0x80) != 0;
    uint16_t destination = extended ? (uint16_t)((_buffer[4] << 8) | _buffer[5]) : (uint16_t)((_buffer[3] << 8) | _buffer[4]);

    AckType acknowledge = _dll.checkAcknowledge(destination, isGroupAddress);
    if (acknowledge == AckType::None) return;

    _flags |= TP_FRAME_FLAG_ADDRESSED;

    // BUSY-MODUS: JETZT SCHWEIGEN WIR, und zwar genau deshalb, weil der Chip die Arbeit übernimmt.
    // "During this time and when autoacknowledge is active, NCN5130 rejects the frames whose destination
    // address corresponds to the stored physical address by sending the BUSY acknowledge" (S. 35) - die
    // Absage geht also raus, nur eben aus der Hardware.
    //
    // Ein U_Ackn.req von uns würde sie nicht ergänzen, sondern den Modus BEENDEN: "BUSY mode is deactivated
    // immediately if the host controller confirms a frame by sending U_Ackn.req" (S. 35), und S. 38/40
    // nennen den Dienst neben U_QuitBusy.req ausdrücklich als zweiten Ausschalter. Das gilt für JEDES
    // U_Ackn.req, auch eines mit gesetztem BUSY-Bit - der Modus überlebte sonst kein einziges an uns
    // gerichtetes Telegramm.
    //
    // BEIDE Bedingungen müssen stehen. Ohne aktive Auto-Quittung hat U_SetBusy.req laut Datenblatt "no
    // effect" - dann antwortet weder der Chip noch wir, und aus der Absage würde vollständiges Schweigen.
    //
    // Was NICHT der Chip abdeckt: Gruppenadressen und, bei einem Koppler, fremde Einzeladressen. Die
    // bleiben in diesen 700ms unquittiert, und der Absender wiederholt. Genau das ist der Zweck des
    // Modus - Gegendruck erzeugen, nicht Telegramme annehmen.
    if (_dll.isBusyMode() && _dll.isAutoAcknowledge()) return;

    // HIER WIRD NICHT AUF isAutoAcknowledge() GEPRÜFT, und das ist Absicht: die Auto-Quittung des Chips ist
    // ein Fallback für einen zu langsamen Host, kein Ersatz für dessen Antwort. Sie deckt nur die EIGENE
    // physikalische Adresse des Chips ab, während der Callback auch Gruppenadressen und - bei einem Koppler -
    // fremde Einzeladressen bejaht. Genau dieser Fehler stand hier einmal; am Bus kamen Telegramme mit
    // ADDRESSED, aber ohne ACK herein, und der Absender wiederholte sie dreimal.

    // Liegen im Interface schon so viele Bytes bereit, wie von diesem Frame überhaupt noch ausstehen, dann
    // ist das Telegramm inklusive Prüfsummen-Oktett bereits vollständig eingetroffen - auf dem Bus also
    // längst durch und sein Acknowledge-Fenster zu. Ein jetzt gesendetes U_Ackn.req käme nicht nur zu spät,
    // es würde vom Chip dem nächsten Telegramm zugeordnet und damit ein fremdes Frame fälschlich bestätigen.
    // Die Grenze ist >= und nicht >: bei Gleichheit ist das Frame eben schon komplett da. Und weil ein
    // Folgetelegramm frühestens 50 Bitzeiten später beginnen darf, gilt die Gleichheit auch nach einem
    // BELIEBIG langen Rückstand - mit > würde also genau der Stall passender Länge das ACK durchlassen,
    // das die Prüfung verhindern soll. Im Normalbetrieb bremst das nichts: dort liegen 0-1 Bytes bereit,
    // während mindestens 2 ausstehen.
    if (_dll._interface->available() >= (_frameSize - _bufferPos))
    {
        _dll._statistics.incrementTxAcknowledgesSuppressed();
        return;
    }

    // Geschrieben wird im Transmitter - dort liegt aller schreibende Zugriff auf das Interface.
    if (!_dll._transmitter.sendAcknowledge(acknowledge)) return;

    _flags |= acknowledgeFlags(acknowledge);

    // Ein abgesetztes U_Ackn.req beendet den Busy-Modus im Chip (S. 35). Hierher kommt nur, wer die Sperre
    // oben passiert hat - also gab es keine aktive Auto-Quittung und der Modus war ohnehin wirkungslos.
    // Gemeldet wird es trotzdem, damit unser Merker nicht einen Zustand behauptet, den der Chip nicht hat.
    if (_dll.isBusyMode()) _dll.reportBusyModeCancelled();
}

// Steuerbytes sind laut Datenblatt (Table 13, "Control Services") immer genau 1 Byte lang und
// selbsterklärend - mit einer Ausnahme: U_SystemStat.ind bringt ein zweites Byte mit.
void Receiver::processControlByte(uint8_t value)
{
    _buffer[0] = value;
    _bufferPos = 1;

    // Kein Steuerbyte, sondern der Anfang eines Poll-Telegramms (siehe L_POLL_DATA_IND). Es als 1-Byte-
    // Sequenz abzuschließen würde die Folgebytes als Anfang neuer Sequenzen lesen lassen - bis hin zu
    // einem ungewollten ACK auf eine aus Poll-Bytes zusammengesetzte Adresse. Ab hier zählt deshalb
    // processPollByte() mit, statt auf eine Pause zu warten.
    if (value == L_POLL_DATA_IND)
    {
        resetSequence(RxState::Poll);
        processPollByte(value);
        return;
    }

    // ERST HIER, nach dem Poll-Zweig: der zählt sein Byte selbst, und zwar als Telegrammbyte.
    _dll._statistics.incrementRxControlBytes();

    // Nur die NCN512x-Reihe kennt diesen Dienst, und sie schickt ihn ausschließlich als Antwort auf ein
    // U_SystemState.req - also nur auf requestState() hin. Beim TPUART2 wäre 0x4B etwas anderes -
    // ohne diese Abfrage würde dort das Folgebyte mitverschluckt. Die alte Library hat genauso geprüft.
    if (value == U_SYSTEM_STAT_IND && _dll.bcuType() == BcuType::Ncn5120)
    {
        _state = RxState::Control; // 2. Byte folgt noch
        return;
    }

    // Beide werden HIER im Tick ausgewertet und nicht erst aus dem Ringpuffer heraus - die Begründung steht
    // an den beiden Empfängern im DataLinkLayer.
    if ((value & U_CONFIGURE_MASK) == U_CONFIGURE_IND) _dll.configureIndication(value);
    if (value == U_RESET_IND) _dll.resetIndication();

    // Bestätigung des eigenen Versands. Sie wird hier freigegeben und nicht erst in
    // DataLinkLayer::handleControlEntry(): _txState darf nur EINEN Schreiber haben, und das ist der Tick.
    // Über den Ringpuffer käme die Freigabe zudem erst, wenn der Hauptloop wieder dran ist - der Sendeweg
    // bliebe unnötig lange belegt.
    if ((value & L_DATA_CON_MASK) == L_DATA_CON) _dll._transmitter.confirmed();

    completeSequence(0, RxState::Idle);
}

// Der Anfang einer neuen Sequenz - und die EINZIGE Stelle, an der dieser Satz Felder zurückgesetzt wird.
// Vorher stand er dreimal ausgeschrieben da (Sequenzstart, Resync, Abschluss), was ihn zu einer Falle machte:
// wer ein Feld dazunimmt, muss es sonst an drei Stellen mitnehmen, und die vergessene fällt erst als
// nachwirkender Zustand im nächsten Telegramm auf.
void Receiver::resetSequence(RxState nextState)
{
    // Die eine Stelle, an der ein Resync beginnt - alle Wege dorthin laufen hier zusammen (CRC-Fehler,
    // beschädigtes Längenoktett, unerwartetes Byte in FrameAck, Moduswechsel). Doppelt gezählt wird
    // nichts: forceResync() kehrt bei bereits laufendem Resync früh um, und completeSequence() kommt aus
    // Frame bzw. Poll, nie aus Resync selbst.
    if (nextState == RxState::Resync) _dll._statistics.incrementRxResyncs();

    _bufferPos = 0;
    _frameSize = 0;
    _crc = 0;
    _flags = 0;
    _state = nextState;
}

// Bricht eine laufende Sequenz ab und geht in den Resync. War NICHTS im Gange (Idle), gibt es auch nichts
// abzubrechen: es liegen keine Folgebytes einer angefangenen Sequenz aus, die als Anfang von etwas Neuem
// fehlgedeutet werden könnten - die als Nächstes eintreffenden Bytes werden ganz normal interpretiert. Bei
// Resync läuft die Verwerfung ohnehin schon. Einmal blind IMMER in Resync zu gehen hat genau das kaputt
// gemacht: die Reset-Antwort und das erste Frame nach einem Moduswechsel landeten als Datenmüll im Resync.
void Receiver::forceResync()
{
    if (_state == RxState::Idle || _state == RxState::Resync) return;

    // Die angebrochene Sequenz wird verworfen, nicht gemeldet - sie ist Folge unserer eigenen Umschaltung.
    // Ihre Bytes zählen als verworfen, denn gesehen hat sie niemand.
    _dll._statistics.incrementRxDroppedBytes((uint32_t)_bufferPos);

    resetSequence(RxState::Resync);
}

// Schiebt die fertige Sequenz in den Ringpuffer und macht den Empfangspuffer sofort wieder frei. Nichts
// wartet hier auf einen Abnehmer - genau deshalb kann tick() ohne Unterbrechung weiterlaufen.
void Receiver::completeSequence(uint8_t flags, RxState nextState)
{
    // Was der Sequenz unterwegs zugewachsen ist (siehe _flags), plus was der Aufrufer mitbringt.
    flags |= _flags;

    size_t length = _bufferPos < TPUART_BUFFER_SIZE ? _bufferPos : TPUART_BUFFER_SIZE;

    if (length > 0)
    {
        // Frame oder Steuerbyte-Sequenz? Dieselbe Prüfung, die beim ersten Byte entschieden hat.
        bool isFrame = (_buffer[0] & L_DATA_MASK) == L_DATA_STANDARD_IND || (_buffer[0] & L_DATA_MASK) == L_DATA_EXTENDED_IND;

        if (isFrame && _dll._transmitter.isEcho(_buffer, _bufferPos)) flags |= TP_FRAME_FLAG_TX;

        if (isFrame)
        {
            // Gemeldet wird es in beiden Fällen - der Unterschied ist nur, ob es in Ordnung war. Davon
            // klar getrennt sind die im Resync verworfenen Bytes, die nie jemand zu sehen bekommt.
            if (flags & TP_FRAME_FLAG_INVALID)
                _dll._statistics.incrementRxInvalidFrames();
            else
                _dll._statistics.incrementRxFrames();
        }

        // DIE BYTES SIND HIER SCHON GEZÄHLT - je Byte beim Eintreffen, nicht hier am Stück (siehe
        // processFrameByte()). Poll läuft dabei in die Telegrammbytes, weil es Bytes vom Bus sind wie die
        // eines Telegramms; bei den Telegramm-ZÄHLERN steht es bewusst nicht dabei - ein Poll ist kein
        // Telegramm und geht auch nicht über deliverFrame() nach oben.

        // Passt der Eintrag nicht mehr in den Ring, ist er weg - und diese Bytes hat dann WIRKLICH niemand
        // gesehen, genau wie die im Resync verworfenen. Sie gehören deshalb in denselben Zähler.
        if (!pushEntry(_buffer, length, flags))
            _dll._statistics.incrementRxDroppedBytes((uint32_t)length);
    }

    resetSequence(nextState);
}

// Ist kein Platz mehr, wird der Eintrag verworfen und NICHT etwa der älteste überschrieben: was schon
// angenommen wurde, bleibt. Der Verlust wird über queueOverflow() einmalig gemeldet.
bool Receiver::pushEntry(const uint8_t *data, size_t length, uint8_t flags)
{
    uint32_t needed = TPUART_RX_QUEUE_HEADER_SIZE + length;
    uint32_t used = _queueHead - _queueTail;

    if (TPUART_RX_QUEUE_SIZE - used < needed)
    {
        _dll.reportRxQueueOverflow();
        return false;
    }

    // Der Höchststand NACH dem Einstellen - das ist der Füllstand, der wirklich erreicht wurde. Der
    // Vergleich hängt sich an ein bereits ausgerechnetes `used` an und kostet damit fast nichts.
    _dll._statistics.updateRxQueuePeakBytes(used + needed);

    uint32_t head = _queueHead;
    _queue[head++ % TPUART_RX_QUEUE_SIZE] = (uint8_t)(length & 0xFF);
    _queue[head++ % TPUART_RX_QUEUE_SIZE] = (uint8_t)(length >> 8);
    _queue[head++ % TPUART_RX_QUEUE_SIZE] = flags;

    for (size_t i = 0; i < length; i++)
        _queue[head++ % TPUART_RX_QUEUE_SIZE] = data[i];

    // Erst ganz zum Schluss sichtbar machen - vorher könnte loop() einen halben Eintrag sehen.
    _queueHead = head;
    return true;
}

// ---------------------------------------------------------------------------------------------------
// Gemütliche Seite - läuft aus dem Hauptloop
// ---------------------------------------------------------------------------------------------------

void Receiver::processQueue()
{
    while (_queueTail != _queueHead)
    {
        uint32_t tail = _queueTail;

        size_t length = _queue[tail++ % TPUART_RX_QUEUE_SIZE];
        length |= (size_t)_queue[tail++ % TPUART_RX_QUEUE_SIZE] << 8;
        uint8_t flags = _queue[tail++ % TPUART_RX_QUEUE_SIZE];

        // Der Produzent stellt nur Längen <= TPUART_BUFFER_SIZE ein, im geordneten Betrieb kann das hier
        // also nicht greifen. Es bleibt trotzdem stehen, weil die Folge sonst maximal unangenehm wäre:
        // length ist 16 Bit breit, das Frame darunter fasst nur 263 Byte - ein überlanger Wert schriebe
        // über dessen Stackrahmen hinaus. Ein einzelner korrupter Eintrag darf nicht den ganzen Kontext
        // zerlegen, also: Ring verwerfen und weitermachen.
        if (length > TPUART_BUFFER_SIZE)
        {
            // NICHT über _queueOverflow melden - das hieße "der Ring war voll", und das ist etwas ganz
            // anderes als "der Ring enthält Unsinn". Hier läuft ohnehin der Hauptkontext, also geht die
            // Meldung direkt raus, mit dem Wert, an dem sich der Fehler festmachen lässt.
            _dll.printError("RX queue corrupt: entry length %u - queue dropped", (unsigned)length);

            _queueTail = _queueHead;
            return;
        }

        // Das Frame hält seinen Speicher selbst und liegt hier auf dem STACK - für die Dauer dieses
        // Durchlaufs. Hineinkopiert wird einzeln, weil der Eintrag im Ring umbrechen kann.
        Frame frame(length, flags);
        uint8_t *data = frame.buffer();

        for (size_t i = 0; i < length; i++)
            data[i] = _queue[tail++ % TPUART_RX_QUEUE_SIZE];

        // ERST JETZT den Platz im Ring freigeben - aber noch VOR dem Callback: was der Verbraucher braucht,
        // liegt jetzt in seinem eigenen Frame, und wie lange er damit beschäftigt ist, geht den Tick nichts
        // mehr an. Bliebe der Eintrag bis zum Ende des Callbacks stehen, liefe der Ring genau dann voll,
        // wenn der Hauptloop ohnehin hängt.
        _queueTail = tail;

        // Frame oder Steuerbyte? Frame::isFrame() prüft dasselbe erste Byte, das beim Einlesen schon
        // entschieden hat - ein eigenes Flag dafür braucht es nicht.
        if (!frame.isFrame())
        {
            _dll.handleControlEntry((const uint8_t *)frame.data(), length);
            continue;
        }

        // Wiederholungserkennung. Geprüft (und gemerkt) wird JEDES gültige Telegramm, markiert nur eine
        // Wiederholung - ohne den Vergleichswert des Originals wäre die Wiederholung nicht als solche zu
        // erkennen. Kaputte Telegramme bleiben außen vor: ihr Inhalt ist nicht verlässlich, ein
        // Fingerabdruck darüber wäre wertlos und würde den Eintrag des Absenders verderben. Die alte
        // Library kam gar nicht erst in die Verlegenheit - sie hat kaputte Telegramme verworfen statt sie
        // zu melden.
        if (frame.isValid())
        {
            bool seen = _dll._repetitionFilter.check(frame);

            if (seen && frame.isRepeated())
            {
                frame.setFiltered();
                _dll._statistics.incrementRxRepeatedFrames();
            }
        }

        _dll.deliverFrame(frame);
    }
}

RxState Receiver::state() const
{
    return _state;
}

// --- KOMPAT, siehe Header ------------------------------------------------------------------------------

unsigned short Receiver::getSearchBufferPosition() const
{
    return (unsigned short)_bufferPos;
}

// 0, solange die Größe noch nicht feststeht - sie ergibt sich erst aus dem Längenoktett. Der Vergleich
// fängt zusätzlich den Fall ab, dass _bufferPos über die Grenze hinausgelaufen ist.
unsigned short Receiver::getAwaitBytes() const
{
    if (_frameSize == 0 || _bufferPos >= _frameSize) return 0;

    return (unsigned short)(_frameSize - _bufferPos);
}

} // namespace TPUart
