#include "TPUart/DataLinkLayer.h"

// Lebenszeichen beim Bau: zeigt im Fremdprojekt, dass diese Version gelinkt wird. In der .cpp, damit die
// Zeile einmal je Bau kommt und nicht einmal je Übersetzungseinheit.
#pragma message "TPUART v2"
    
#include <stdarg.h>
#include <stdio.h>

#include <Arduino.h>

#include <string>

// Hier gibt es bewusst KEINE Sperre und keinen plattformabhängigen Code. Alles, was sich Tick-Antrieb und
// Hauptkontext teilen, ist über Besitz und Reihenfolge geregelt statt über Locks:
//   - RX-, TX- und Steuercode-Warteschlange: je ein Produzent, ein Konsument, Kopf zuletzt sichtbar
//     gemacht - der Konsument sieht damit nie einen halb geschriebenen Eintrag
//   - Callbacks: werden vor dem Start des Tick-Antriebs gesetzt, also ohne gleichzeitigen Leser
// Ein blockierendes Lock wäre ohnehin nicht möglich - auf dem RP2040 läuft tick() im Interrupt.

namespace
{

constexpr uint32_t TPUART2_BAUD_RATES[] = {19200};
constexpr uint32_t NCN5120_BAUD_RATES[] = {19200, 38400};

// Kandidaten für die Baudratenerkennung, abhängig vom Chip: der TPUART2 kennt nur 19200 fest, die
// NCN512x-Reihe zusätzlich 38400 (siehe alte Library, DataLinkLayer.cpp::tryInitialize()).
void candidateBaudRates(TPUart::BcuType type, const uint32_t *&rates, size_t &count)
{
    if (type == TPUart::BcuType::Tpuart2)
    {
        rates = TPUART2_BAUD_RATES;
        count = 1;
        return;
    }
    rates = NCN5120_BAUD_RATES;
    count = 2;
}

} // namespace

namespace TPUart
{

DataLinkLayer::DataLinkLayer(Interface::Abstract &interface) : _interface(&interface), _transmitter(*this), _receiver(*this) {}

// KOMPAT: ohne Interface - es kommt dann über begin(bcuType, interface).
DataLinkLayer::DataLinkLayer() : _transmitter(*this), _receiver(*this) {}

// Siehe Header: der Timer darf keinen Zeiger auf ein zerstörtes Objekt behalten. remove() ist billig, wenn
// die Instanz nie eingetragen war - es prüft zuerst contains().
DataLinkLayer::~DataLinkLayer()
{
    Timer::instance().remove(*this);
}

// ---------------------------------------------------------------------------------------------------
// Zeitkritische Seite - läuft später aus einem Timer-Interrupt
// ---------------------------------------------------------------------------------------------------

// Hält sich nirgends auf und blockiert nie.
void DataLinkLayer::tick()
{
    if (_interface == nullptr) return; // ohne Interface gibt es nichts zu tun (siehe KOMPAT-Konstruktor)
    if (!_initialized) return;         // begin() wurde noch nicht aufgerufen

    // TAKTMESSUNG, und sie steht bewusst VOR allen weiteren Abbrüchen: gemessen werden soll der ANTRIEB,
    // also wie oft wir gerufen werden - nicht, wie oft wir etwas zu tun hatten. Stünde sie weiter unten,
    // wären ausgerechnet die Phasen unsichtbar, in denen der Tick nichts tut und trotzdem laufen muss.
    //
    // Die Kosten sind ein micros() im Leerlaufpfad, und das ist eine bewusste Abkehr von der Ansage, diesen
    // Pfad auf möglichst wenige MMIO-Zugriffe zu halten: er hatte einen (den DMA-Zähler in available()) und
    // hat jetzt zwei. Gerechnet auf den RP2040 sind das rund 0,1% CPU zusätzlich zu den ~0,5%, die der Tick
    // ohnehin kostet. Dafür ist die häufigste Ursache von Problemen dieser Schicht überhaupt erst ablesbar,
    // statt aus ihren Folgen erraten zu werden.
    uint32_t tickNow = micros();
    uint32_t sinceLast = _tickLastUs == 0 ? 0 : tickNow - _tickLastUs;
    _tickLastUs = tickNow;

    _statistics.recordTick();

    // Die Einordnung passiert HIER und nicht in Statistics: TPUART_TICK_DEFERRED_US ist eine Protokollkonstante
    // (die Zeichenzeit des Busses), und die Zählerklasse kennt keine - dieselbe Trennung wie bei den
    // Fehlerbits des Chips. Der Zähler ist das, was den Höchstwert überhaupt erst deutbar macht: ohne ihn
    // sehen ein einzelnes Ereignis und ein Dauerzustand identisch aus.
    if (sinceLast > TPUART_TICK_DEFERRED_US) _statistics.recordTickDeferred(sinceLast);

    // Die Baudraten-SUCHE gehört dem Hauptkontext, weil sie das Interface neu konfiguriert - hier ist
    // deshalb Schluss, bis die Verbindung zum ersten Mal gestanden hat (siehe searchBaudRate).
    if (!_everConnected) return;

    if (!_connected)
    {
        reconnect();
        return;
    }

    _receiver.process();
    _transmitter.process();

    // ERST HIER gemessen, nicht an den frühen Abbrüchen: die sind ein paar Vergleiche und würden den
    // Höchstwert nur beschönigen. Was hier steht, ist der volle Durchlauf einschließlich des
    // Quittungs-Callbacks des Aufrufers - und damit die Zahl, die eine hohe IRQ-Priorität rechtfertigt
    // oder eben nicht. Zweites micros(), also nochmal ~0,1% CPU; für eine Zahl, ohne die jede
    // Prioritätswahl geraten ist, ist das billig.
    _statistics.updateTickDurationMaxUs(micros() - tickNow);
}

void DataLinkLayer::begin(BcuType bcuType)
{
    _bcuType = bcuType;
    _initialized = true;
    _connected = false;
    _detectAwaitingResponse = false;
    _detectCandidateIndex = 0;
    _detectNextAttemptAt = millis(); // sofort beim nächsten loop() fällig

    // Die Taktmessung fängt neu an: die Zeit zwischen end() und begin() ist keine Lücke im Antrieb, und
    // ohne das Zurücksetzen zählte sie beim ersten Tick danach als Verzögerung.
    _tickLastUs = 0;

    // OHNE NACHFRAGE: der Timer wird immer benutzt, einen Schalter je Instanz gibt es nicht. Der Antrieb
    // darf sofort loslaufen, obwohl noch keine Verbindung steht - bis _everConnected gesetzt ist, kehrt
    // tick() gleich wieder um und fasst das Interface nicht an; die Baudratensuche gehört dem Hauptkontext
    // (siehe searchBaudRate).
    //
    // Schlägt das Eintragen fehl - keine Timer-Plattform, Intervall 0 oder alle Plätze belegt -, bleibt es
    // beim Antrieb aus process(), und usesTimer() sagt es. Gemeldet wird das hier NICHT: zu diesem
    // Zeitpunkt muss noch kein Message-Callback gesetzt sein, die Meldung ginge ins Leere. Sichtbar wird es
    // trotzdem, denn checkTickRate() aus loop() schlägt an, sobald der Antrieb zu langsam ist - und genau
    // darauf kommt es an.
    Timer::instance().add(*this);
}

// KOMPAT: die alte Library bekam das Interface hier statt im Konstruktor.
void DataLinkLayer::begin(BcuType bcuType, Interface::Abstract *interface)
{
    _interface = interface;
    begin(bcuType);
}

// DYNAMISCH beantwortet, nicht bei begin() festgeschrieben - siehe Header. Beide Teile müssen stimmen: der
// Timer muss laufen UND diese Instanz muss eingetragen sein. Läuft er für eine andere Instanz, während für
// diese kein Platz war, ist die Antwort für uns trotzdem "nein".
bool DataLinkLayer::usesTimer() const
{
    return Timer::instance().running() && Timer::instance().contains(*this);
}

// KOMPAT: Gegenstück zu begin(). Das Interface wird geschlossen, aber NICHT freigegeben - es gehört dem
// Aufrufer.
void DataLinkLayer::end()
{
    // ZUERST austragen: Timer::remove() kehrt erst zurück, wenn garantiert kein tick() mehr für diese
    // Instanz unterwegs ist - einer, der gleich in ein geschlossenes Interface greifen würde.
    Timer::instance().remove(*this);

    _initialized = false;
    _connected = false;

    if (_interface != nullptr) _interface->end();
}

// KOMPAT: ein Einstieg für beide Hälften, wie in der alten Library.
//
// HIER WIRD NICHT GETICKT. NIE. Der Hauptloop ist kein Antrieb für tick() - das ist eine ausdrückliche
// Festlegung des Anwenders und keine Auslegungsfrage.
//
// Der Grund steht im Klassenkommentar: tick() bewegt ein Byte je Richtung und Aufruf, der Durchsatz und die
// 2,8ms-Quittungsfrist hingen also an der Loopfrequenz einer fremden Anwendung. Genau das war der Zustand,
// der im Router 457 Ticks/s statt 2000 ergab und 12% der Quittungen gekostet hat. Ein Rückfall auf den
// Hauptloop macht diesen Zustand zum stillen Normalfall - er sieht aus wie "es läuft", und niemand erfährt,
// dass der Antrieb fehlt.
//
// DER ANTRIEB IST DER TIMER, oder Timer::trigger() aus einem Kontext, den der Aufrufer selbst stellt. Gibt
// es beides nicht, tickt nichts, und das ist die richtige Antwort: eine Plattform ohne Timer braucht eine
// Entscheidung des Entwicklers, keine stillschweigende Notlösung.
void DataLinkLayer::process()
{
    loop();
}

// NICHT BLOCKIEREND: pro Durchlauf wird höchstens ein Byte betrachtet, der Ablauf ist über
// _detectAwaitingResponse/_detectRequestSentAt auf mehrere Aufrufe verteilt.
//
// LÄUFT IM HAUPTKONTEXT (aus loop()), und das ist der Grund für die Aufteilung in searchBaudRate() und
// reconnect(): pro Kandidat wird das Interface neu konfiguriert, und das gehört in keinen Interrupt. Der
// Tick kehrt deshalb um, solange !_everConnected - siehe den Block im Header, dort steht auch, warum nur
// einmal gesucht wird.
void DataLinkLayer::searchBaudRate()
{
    if (_detectAwaitingResponse)
    {
        switch (pollDetectResponse())
        {
            case DetectResult::Pending:
                return;
            case DetectResult::Connected:
                connectDetected();
                return;
            case DetectResult::Failed:
                advanceDetectCandidate();
                return;
        }
    }

    if ((int32_t)(millis() - _detectNextAttemptAt) < 0) return; // noch nicht fällig

    const uint32_t *rates;
    size_t count;
    candidateBaudRates(_bcuType, rates, count);
    uint32_t baud = rates[_detectCandidateIndex % count];

    // Baudratenwechsel erfordert einen sauberen Neustart des Interfaces - ein bereits laufendes würde bei
    // reinem begin() ohne vorheriges end() undefiniert reagieren (RP2040 würde z.B. einen zweiten
    // DMA-Channel beanspruchen wollen).
    _interface->end();
    _interface->begin(baud);

    // Geht der Request nicht raus, wird auch nicht auf Antwort gewartet: 50ms auf etwas zu warten, das
    // nie gefragt wurde, würde diesen Kandidaten grundlos verbrennen. Nach einem frischen begin() ist der
    // Sendepuffer leer, das kann also praktisch nicht vorkommen - kostet aber auch nichts.
    if (!_interface->write((char)U_RESET_REQ)) return;

    _detectAwaitingResponse = true;
    _detectRequestSentAt = millis();
}

// Aus dem Tick, nach einem Verbindungsverlust. Derselbe Ablauf wie bei der Suche, mit einem Unterschied:
// das Interface wird NICHT angefasst. Die Baudrate steht fest, das Interface ist offen und richtig
// konfiguriert - es geht nur noch darum, das U_Reset.req zu wiederholen, bis die BCU wieder antwortet.
//
// Geduldig, weil die BCU erst antwortet, sobald am Bus die Busspannung anliegt: nach jedem Fehlversuch
// TPUART_DETECT_RETRY_INTERVAL_MS Pause, statt eng weiterzuprobieren.
void DataLinkLayer::reconnect()
{
    if (_detectAwaitingResponse)
    {
        switch (pollDetectResponse())
        {
            case DetectResult::Pending:
                return;
            case DetectResult::Connected:
                connectDetected();
                return;
            case DetectResult::Failed:
                _detectAwaitingResponse = false;
                _detectNextAttemptAt = millis() + TPUART_DETECT_RETRY_INTERVAL_MS;
                return;
        }
    }

    if ((int32_t)(millis() - _detectNextAttemptAt) < 0) return; // noch nicht fällig

    if (_interface->availableForWrite() < 1) return;
    if (!_interface->write((char)U_RESET_REQ)) return;

    _detectAwaitingResponse = true;
    _detectRequestSentAt = millis();
}

// Wichtig für die Korrektheit: die Antwort muss UNMITTELBAR auf unseren eigenen Request folgen. Bei noch
// unbestätigter Baudrate ist gelesener Datenmüll nämlich beliebig - er könnte zufällig wie ein U_Reset.ind
// aussehen. Deshalb wird nur ausgewertet, während _detectAwaitingResponse gesetzt ist, und das erste Byte,
// das dabei ankommt und nicht die (laut altem Kommentar "TPUart sendet Nullen am Anfang") ignorierbare 0
// ist, entscheidet sofort über Erfolg oder Misserfolg - es wird nicht weiter auf ein verspätetes 0x03
// gewartet.
DataLinkLayer::DetectResult DataLinkLayer::pollDetectResponse()
{
    if (_interface->available())
    {
        int value = _interface->read();
        if (value >= 0)
        {
            if (value == 0) return DetectResult::Pending; // führende Null - noch keine Entscheidung
            if (value == U_RESET_IND) return DetectResult::Connected;

            return DetectResult::Failed; // irgendein anderes Byte statt der erwarteten Antwort
        }
    }

    if ((uint32_t)(millis() - _detectRequestSentAt) >= TPUART_DETECT_RESPONSE_TIMEOUT_MS) return DetectResult::Failed;

    return DetectResult::Pending;
}

// Die BCU hat geantwortet. Reihenfolge beachten: _everConnected veröffentlicht den Besitzwechsel am
// Interface, _connected ganz zuletzt gibt die beiden Hälften frei - erst wenn alles davor steht.
void DataLinkLayer::connectDetected()
{
    const uint32_t *rates;
    size_t count;
    candidateBaudRates(_bcuType, rates, count);

    // Dieselbe Behandlung wie bei jedem anderen Reset: der U_Reset.ind ist zugleich die Bestätigung, dass
    // der Sendepuffer der BCU leer ist und sie Adresse und Zähler vergessen hat.
    resetIndication();

    _connectedBaudRate = rates[_detectCandidateIndex % count];
    _detectAwaitingResponse = false;
    _lastReceivedAt = millis(); // sonst gälte die Verbindung sofort als still
    _everConnected = true;
    _connected = true;
}

// Ein Kandidat ist gescheitert (falsches Byte oder Frist abgelaufen) - weiter zum nächsten. Ist die Liste
// am Ende angekommen, wird nicht sofort neu begonnen, sondern erst nach TPUART_DETECT_RETRY_INTERVAL_MS:
// die BCU antwortet erst, sobald am Bus die Busspannung anliegt, und das kann nach dem eigenen Boot noch
// beliebig lange dauern - ein enges Dauerprobieren brächte bis dahin nichts außer unnötigem Bus-Traffic.
void DataLinkLayer::advanceDetectCandidate()
{
    _detectAwaitingResponse = false;

    const uint32_t *rates;
    size_t count;
    candidateBaudRates(_bcuType, rates, count);

    _detectCandidateIndex++;
    if (_detectCandidateIndex >= count)
    {
        _detectCandidateIndex = 0;
        _detectNextAttemptAt = millis() + TPUART_DETECT_RETRY_INTERVAL_MS;
    }
}

// ---------------------------------------------------------------------------------------------------
// Rückmeldungen der beiden Hälften - aus dem Tick
// ---------------------------------------------------------------------------------------------------

// Ein Steuercode hat den Chip erreicht (Transmitter). Nur diese beiden ändern etwas am Zustand, den die
// Library über den Chip führt.
void DataLinkLayer::controlByteSent(uint8_t code)
{
    if (code == U_BUSMON_REQ)
    {
        _busMonitor = true;

        // Mit dem Moduswechsel ist der ganze Sendeweg hinfällig: der Chip nimmt keine Telegrammdienste mehr
        // an, und was er von einem laufenden Telegramm schon hat, ergibt als halbes Telegramm keinen Sinn -
        // auf dem Bus hat nie jemand einen Anfang gesehen. Abgebrochen wird deshalb, NICHT wie beim Reset
        // von vorn begonnen. Die Warteschlange räumt der Hauptkontext beim nächsten stageNextTelegram():
        // was erst nach dem Verlassen des Modus hinausginge, wäre dann längst überholt.
        _transmitter.abort();

        // BUSMON AN HEISST BUSY AUS. Der Chip nimmt im Busmonitor keine Telegramme mehr an und quittiert
        // nichts - ein Busy-Modus, der das Quittieren nur ersetzt, kann es dort nicht geben, und heraus
        // kommt man ohnehin nur per Reset. Der Zustand ist also nicht unbekannt, sondern bekannt weg.
        //
        // Gemeldet statt selbst gelöscht: _busyModeSince gehört dem Hauptkontext (busyMode(),
        // checkBusyMode()), und ein zweiter Schreiber aus dem Tick wäre der teurere Fehler. Genau dafür
        // gibt es diesen Weg schon - checkBusyMode() zieht beim nächsten loop() nach, ohne ein Byte zu
        // senden. Ohne das liefe checkBusyMode() in eine Sackgasse: es riefe nach Ablauf der Frist
        // busyMode(false), und das lehnt im Busmonitor jetzt ab, ließe _busyModeSince also stehen und
        // versuchte es endlos erneut.
        reportBusyModeCancelled();

        // Dasselbe für die Auto-Quittung: im Busmonitor ist der Chip durchsichtig und quittiert nichts.
        // Das Flag beschreibt den Chip, nicht unseren Wunsch - es steht hier also auf falsch, bis die
        // Adresse nach dem Reset erneut abgesetzt wird (die Konfigurationsepoche holt das nach).
        _autoAcknowledge = false;
    }

    if (code == U_RESET_REQ)
    {
        // Beim VERLASSEN des Busmonitors muss die Verbindungsüberwachung neu aufgezogen werden, sonst
        // schlägt sie sofort zu. Während des Modus ruht sie (siehe processConnectionState), _lastReceivedAt
        // ist auf einem ruhigen Bus also beliebig alt - und Busmonitor auf einem ruhigen Bus ist der
        // Normalfall, man schaltet ihn ein, um zuzusehen. Zwischen diesem U_Reset.req und der U_Reset.ind,
        // die ihn beantwortet, liegt zwar nur rund eine Millisekunde, aber der Hauptloop kann genau dort
        // hineinlaufen und meldete dann einen Verbindungsabbruch, den es nie gab.
        // Erlaubt ist das hier, weil auch der Empfangspfad im Tick schreibt - ein Schreiber, ein Kontext.
        if (_busMonitor) _lastReceivedAt = millis();

        _busMonitor = false;
    }

    // Nur diese beiden Codes ändern die Bedeutung des Bytestroms (im Busmonitor kommen die Quittungen vom
    // Bus dazu, nach einem Reset fallen sie weg). Ab wann genau der Chip umschaltet, ist von hier aus nicht
    // erkennbar, eine laufende Sequenz also nicht mehr verlässlich zu deuten. Andere Steuercodes lassen das
    // Format unberührt - dort wäre ein Resync nur ein unnötig verworfenes Telegramm.
    if (code == U_BUSMON_REQ || code == U_RESET_REQ) _receiver.forceResync();
}

// Die BCU hat sich zurückgesetzt - von wem auch immer veranlasst: vom Wachhund des Transmitters, von
// reset() aus der Anwendung, oder von ihr selbst. In jedem Fall ist ihr Sendepuffer leer und ihr Zustand
// definiert. Unterschieden wird deshalb NICHT, wer den Reset ausgelöst hat: liegt noch ein Telegramm im
// Sendepuffer, beginnt es einfach von vorn; liegt keines, holt der Tick das nächste aus der Warteschlange.
void DataLinkLayer::resetIndication()
{
    _busMonitor = false;      // der Reset beendet ihn, auch wenn wir ihn nicht selbst geschickt haben
    _autoAcknowledge = false; // "After reset the address evaluation is deactivated again"

    // Adresse und Wiederholungszähler sind damit im Chip weg - der Hauptkontext setzt sie neu ab.
    _configEpoch = _configEpoch + 1;

    _transmitter.restart();
}

// Der Chip meldet seine Betriebsarten. Für die Auto-Quittung ist das eine Bestätigung, keine Nachricht:
// dass sie mit der Adresse aktiv wird, weiß applyConfiguration() schon beim Absetzen - und muss es auch
// wissen, weil dieser Dienst nur beim NCN existiert. Gilt aber trotzdem der Chip: was er hier meldet, ist
// der wahre Zustand, auch wenn er dem Flag widerspricht.
void DataLinkLayer::configureIndication(uint8_t value)
{
    _autoAcknowledge = (value & U_CONFIGURE_AUTO_ACKNOWLEDGE) != 0;
}

// ---------------------------------------------------------------------------------------------------
// Gemütliche Seite - läuft aus dem Hauptloop
// ---------------------------------------------------------------------------------------------------

void DataLinkLayer::loop()
{
    if (_interface == nullptr) return;

    _receiver.processQueue();

    // Der Tick fasst die Sendewarteschlange nicht an - er bekommt genau ein Telegramm vorgelegt. Hier
    // läuft alles zusammen, was sie betrifft: den abgeholten Platz freigeben, im Busmonitor räumen, und
    // das nächste vorlegen. Dass beides im selben Kontext liegt, ist der Grund, warum es beim Wechsel in
    // den Busmonitor keinen Wettlauf mehr gibt.
    _transmitter.stageNextTelegram();

    // Die Baudratensuche gehört hierher und nicht in den Tick: sie konfiguriert das Interface pro Kandidat
    // neu. Sie läuft genau einmal, bis die Verbindung zum ersten Mal steht - danach übernimmt der Tick
    // (siehe den Block bei searchBaudRate).
    if (!_everConnected)
        searchBaudRate();
    else
        processConnectionState();

    // Begrenzt sich selbst auf einen Messwert je Sekunde, steht deshalb hier und nicht in
    // processConnectionState(): auch ohne Verbindung soll die gemeldete Buslast auf null fallen statt
    // beim letzten Wert stehenzubleiben.
    _statistics.sampleBusLoad();

    checkBusyMode();
    checkTickRate();

    // Der eine Reset, der einen Fehler anzeigt: der Wachhund des Transmitters hat zugeschlagen, weil zum
    // gesendeten Telegramm keine Bestätigung kam. Er kann nicht selbst melden - er läuft im Tick.
    if (_transmitter.confirmTimeout())
        printError("No L_Data.con for %u ms - BCU reset, telegram will be sent again", (unsigned)TPUART_TX_CONFIRM_TIMEOUT_MS);

    // Erst nachdem der Ringpuffer leer ist: die Meldungen entstehen zum Teil genau dort oben, und in
    // derselben Runde ausgegeben zu werden ist die Erwartung des Aufrufers.
    showSystemState();
    showStateErrors();
}

// DER ANTRIEB ÜBERWACHT SICH SELBST, und das ist die Lehre aus einem Fehler, der Stunden gekostet hat: die
// Taktrate war zu niedrig, und das zeigte sich AUSSCHLIESSLICH als Folgeschaden - unterdrückte Quittungen,
// Rückstand im Interface. Die Ursache stand nirgends, sie war nur auf ausdrückliche Nachfrage über
// usesTimer() zu erfahren, und danach fragt im Betrieb niemand.
//
// GEMESSEN WIRD DIE ERREICHTE RATE, nicht die eingestellte. Damit gilt der Wächter für JEDEN Antrieb - den
// den zentralen Timer, einen selbst gebauten Antrieb und den Hauptloop - und er misst das, was tatsächlich passiert, statt
// das, was gemeint war. Ein Wächter auf usesTimer() hätte genau den Fall verfehlt, in dem jemand
// absichtlich selbst tickt und dabei zu langsam ist.
//
// DIE SCHWELLE KOMMT AUS DEM BUS, nicht aus der Konfiguration, und deshalb ist sie keine Geschmacksfrage:
// ein Tick bewegt ein Byte je Richtung, ein TP1-Zeichen dauert 1,354ms. Liegt der mittlere Tickabstand
// darüber, holt die Schicht weniger Bytes ab, als der Bus im Vollausbau liefert - dann füllen sich die
// Puffer, bis etwas überläuft, und keine Puffergröße hilft dagegen. Unterhalb der Schwelle ist alles gut,
// oberhalb ist es grundsätzlich kaputt.
//
// Gemeldet wird EINMAL je Störung, nicht je Sekunde: die Meldung ist eine Diagnose, keine Dauerbeobachtung.
// Erholt sich die Rate, wird der Melder wieder scharf, ein wiederkehrendes Problem bleibt also sichtbar.
void DataLinkLayer::checkTickRate()
{
    // VOR begin() zählt nichts, weil tick() dort umkehrt, bevor recordTick() läuft. Ohne diese Sperre
    // meldete der Wächter nach zwei Sekunden "Tick stopped", obwohl bloß noch niemand angefangen hat.
    if (!_initialized) return;

    uint32_t now = millis();

    // Der erste Durchlauf legt nur den Bezugspunkt fest - ohne Vorgänger gibt es keine Rate.
    if (_tickRateCheckedAt == 0)
    {
        _tickRateCheckedAt = now;
        _tickRateLastTicks = _statistics.getTicks();
        return;
    }

    uint32_t elapsed = now - _tickRateCheckedAt;
    if (elapsed < TPUART_TICK_RATE_WINDOW_MS) return;

    uint32_t ticks = _statistics.getTicks();
    uint32_t delta = ticks - _tickRateLastTicks;

    _tickRateCheckedAt = now;
    _tickRateLastTicks = ticks;

    // Gar kein Tick im ganzen Fenster ist der schlimmste Fall und zugleich der, der eine Division durch
    // null wäre - er wird deshalb ausdrücklich behandelt und nicht in die Rechnung gelassen.
    //
    // IN 64 BIT GERECHNET, und das ist kein Zierrat: elapsed steht in Millisekunden und ist nach oben durch
    // nichts begrenzt - es ist der Abstand zwischen zwei loop()-Durchläufen, die weit genug auseinander
    // lagen. Ab 71,6 Minuten liefe elapsed * 1000 in 32 Bit über, und das Ergebnis wäre nicht bloß
    // unbrauchbar, sondern zu KLEIN: der Wächter schwiege ausgerechnet dann, wenn es am schlimmsten steht.
    uint32_t averageUs = delta == 0 ? 0xFFFFFFFF : (uint32_t)(((uint64_t)elapsed * 1000) / delta);

    if (averageUs <= TPUART_TICK_DEFERRED_US)
    {
        _tickRateReported = false; // erholt - der Melder wird wieder scharf
        return;
    }

    if (_tickRateReported) return;
    _tickRateReported = true;

    if (delta == 0)
        printError("Tick stopped - no tick in %u ms, bus needs one below %u us", (unsigned)elapsed, (unsigned)TPUART_TICK_DEFERRED_US);
    else
        printError("Tick too slow - %u us average (%u/s), bus needs below %u us", (unsigned)averageUs, (unsigned)(((uint64_t)delta * 1000) / elapsed), (unsigned)TPUART_TICK_DEFERRED_US);
}

// Aus loop(), also aus dem Hauptkontext - hier darf queueControl() benutzt und gemeldet werden.
//
// Zwei Aufgaben in einer Frist: die regelmäßige Statusabfrage und der Verbindungsverlust. Ohne die Abfrage
// ließe sich ein Ausfall nicht von einem ruhigen Bus unterscheiden - dort kommt regulär minutenlang nichts.
// Als Lebenszeichen zählt jedes empfangene Byte (siehe Receiver::process), die Abfrage sorgt nur dafür, dass
// es auch bei völliger Stille eines gibt.
void DataLinkLayer::processConnectionState()
{
    if (!_connected) return; // die Erkennung läuft, die überwacht sich selbst

    // Herstellen kann die Verbindung beides (searchBaudRate hier, reconnect im Tick), melden nur der Hauptkontext - also hier,
    // beim ersten loop() danach.
    if (!_connectReported)
    {
        _connectReported = true;
        printMessage("BCU connected (%u baud)", (unsigned)_connectedBaudRate);
    }

    // IM BUSMONITOR RUHT DIE ÜBERWACHUNG, und das ist keine Bequemlichkeit, sondern Notwendigkeit. Sie
    // ruht auf zwei Beinen, und im Busmonitor bricht das zweite weg: Bus-Verkehr zählt als Lebenszeichen,
    // und auf einem ruhigen Bus hält die sekündliche Statusabfrage die Frist offen. Genau die ist hier aber
    // wirkungslos - Tabelle 11 (S. 32) führt U_State.req im Busmonitor als "I", also ignoriert und OHNE
    // Antwort. Auf einem ruhigen Bus liefe die 5s-Frist deshalb zwangsläufig ab, connectionLost() setzte
    // den Reconnect in Gang, und dessen U_Reset.req BEENDET DEN BUSMONITOR. Er hielte also nie länger als
    // fünf Sekunden.
    //
    // Der Preis ist bekannt und hinnehmbar: eine BCU, die im Busmonitor ausfällt, merken wir nicht - bis
    // der Modus per reset() verlassen wird und die Überwachung wieder anläuft. Passiv ist der Modus
    // ohnehin, es hängt nichts daran.
    if (_busMonitor) return;

    // REIHENFOLGE UND VORZEICHEN SIND HIER BEIDES PFLICHT, und beides hat gefehlt. _lastReceivedAt schreibt
    // der Tick, also ein Interrupt, und zwar mitten zwischen zwei beliebige Instruktionen von hier. Stand
    // `now = millis()` VOR dem Lesen von _lastReceivedAt, dann konnte dazwischen ein Byte eintreffen und
    // einen Zeitstempel setzen, der GRÖSSER ist als das schon eingefrorene now. Die Differenz wurde negativ,
    // lief als uint32_t auf ~4,29 Mrd. über und war damit sicher >= der Frist: sofortiger
    // Verbindungsabbruch, obwohl gerade eben ein Byte kam. Das Fenster ist nur ein paar Instruktionen breit
    // - bei 38400 Baud trifft aber alle ~286µs ein Byte hinein, der Fehler trat deshalb ausgerechnet unter
    // Last auf (ETS-Programmierung über den Router).
    //
    //   1. ZUERST den Zeitstempel greifen, DANACH die Uhr lesen. Dann liegt now nie vor last.
    //   2. Trotzdem VORZEICHENBEHAFTET vergleichen: der Tick kann auch nach Schritt 1 noch schreiben, und
    //      ein minimal vorauseilender Zeitstempel bedeutet "gerade eben empfangen", nicht "ewig her".
    uint32_t last = _lastReceivedAt;
    uint32_t now = millis();

    if ((int32_t)(now - last) >= (int32_t)TPUART_CONNECTION_TIMEOUT_MS)
    {
        connectionLost();
        return;
    }

    // Nach jedem Reset der BCU steht hier eine neue Epoche - dann muss die Konfiguration erneut raus.
    // Die Epoche wird erst nachgezogen, wenn alles in die Steuer-Warteschlange gepasst hat; war sie voll,
    // bleibt es beim nächsten loop() nochmal offen. Sonst wäre die Konfiguration weg, ohne dass es
    // irgendwo auffiele - und mit ihr die Quittung für alles, was an unsere Adresse geht.
    if (_configAppliedEpoch != _configEpoch)
    {
        uint32_t epoch = _configEpoch; // erst lesen, dann anwenden: der Tick kann sie dazwischen erhöhen
        if (applyConfiguration()) _configAppliedEpoch = epoch;
    }

    if ((uint32_t)(now - _lastStateRequestAt) < TPUART_STATE_INTERVAL_MS) return;

    _lastStateRequestAt = now;
    requestState();
}

// Die BCU antwortet nicht mehr. Weiter geht es mit reconnect() im Tick - mit der bereits festgestellten
// Baudrate, nicht als neue Suche. _detectCandidateIndex bleibt deshalb unangetastet: er zeigt noch auf den
// Kandidaten, der beim ersten Mal geantwortet hat, und connectDetected() liest ihn wieder.
//
// Der Ablauf ist damit derselbe wie beim Start: U_Reset.req an die BCU, und die U_Reset.ind gilt als
// Bestätigung. Sie ist zugleich der definierte Zustand, den wir nach einer Störung ohnehin haben wollen.
//
// Reihenfolge: erst die Felder der Erkennung, _connected ALS LETZTES. Der Tick liest sie nur, wenn
// _connected false ist - er sieht sie damit entweder gar nicht oder vollständig vorbereitet. Das Interface
// bleibt dabei durchgehend offen und in der Hand des Ticks; der Hauptkontext fasst es nach der ersten
// Verbindung nie wieder an (siehe searchBaudRate).
void DataLinkLayer::connectionLost()
{
    printError("BCU disconnected - no byte received for %u ms", (unsigned)TPUART_CONNECTION_TIMEOUT_MS);

    _statistics.incrementConnectionLosses();

    _connectReported = false;
    _detectAwaitingResponse = false;
    _detectNextAttemptAt = millis();
    _connected = false;
}

// Steuerbytes gehen bewusst NICHT nach außen - ihre Deutung ist Sache des Datalink Layers selbst, der
// Aufrufer bekommt davon nur die Meldungen zu sehen.
void DataLinkLayer::handleControlEntry(const uint8_t *data, size_t length)
{
    uint8_t value = data[0];

    // ZUERST DIE BEIDEN, DIE INHALT TRAGEN. Ausgegeben werden sie nicht (siehe unten), ausgewertet schon.

    // Der einzige mehrbytige Steuerdienst - das Statusbyte steht dahinter. Die Längenprüfung ist reine
    // Absicherung: eine zweibytige Sequenz entsteht ausschließlich für genau diesen Code. Trifft sie nicht
    // zu, fällt der Dienst absichtlich durch bis zur Ausgabe unten - eine halbe Sequenz will man sehen.
    if (value == U_SYSTEM_STAT_IND && length == 2)
    {
        _systemState.update(data[1]);
        checkChipRestart();
        return;
    }

    // XOR statt Maskierung: die drei Kennbits sind gesetzt und werden damit gelöscht, übrig bleiben genau
    // die Fehlerbits. Verodert, weil der Chip jedes Ereignis nur einmal meldet (siehe _stateErrors).
    if ((value & U_STATE_MASK) == U_STATE_IND)
    {
        uint8_t errors = (uint8_t)(value ^ U_STATE_MASK);

        // Die Zuordnung Bit -> Zähler steht HIER und nicht in Statistics: sie braucht die
        // Protokollkonstanten, und die kennt nur diese Schicht. Warum überhaupt einzeln gezählt wird, steht
        // an den Feldern in Statistics.h.
        if (errors & U_STATE_SLAVE_COLLISION) _statistics.incrementChipSlaveCollisions();
        if (errors & U_STATE_RECEIVE_ERROR) _statistics.incrementChipReceiveErrors();
        if (errors & U_STATE_TRANSMIT_ERROR) _statistics.incrementChipTransmitErrors();
        if (errors & U_STATE_PROTOCOL_ERROR) _statistics.incrementChipProtocolErrors();
        if (errors & U_STATE_TEMPERATURE_WARNING) _statistics.incrementChipTemperatureWarnings();

        _stateErrors |= errors;
        return;
    }

    // AUF DIE KONSOLE KOMMT NUR, WAS ÜBERRASCHT. Alles hier drunter ist die Antwort auf etwas, das wir
    // selbst veranlasst haben - es zu melden hieße, sich das eigene Echo vorlesen zu lassen:
    //
    //   U_State.ind / U_SystemStat.ind - Antwort auf die Statusabfrage, die JEDE SEKUNDE läuft. Ihr Inhalt
    //     wird gemeldet, und nur wenn er etwas sagt: Fehlerbits über showStateErrors(), der System-Status
    //     über showSystemState() bei Änderung.
    //   U_Configure.ind - Bestätigung einer Einstellung von uns (die Adresse aktiviert die Auto-Quittung).
    //     Der Zustand steht jederzeit über isAutoAcknowledge() bereit.
    //   U_Reset.ind - Antwort auf unser U_Reset.req, sei es aus reset(), aus der Verbindungsaufnahme oder
    //     vom Wachhund des Transmitters. Der EINE Reset, der einen Fehler anzeigt, meldet sich selbst und
    //     mit Begründung: "No L_Data.con ... BCU reset" (siehe loop()). Und kommt ein Reset von der BCU
    //     selbst, wird das ohnehin sichtbar - entweder über den Verbindungsverlust oder, beim NCN, an der
    //     Statuszeile, die dann POWER-UP/SYNC durchläuft (siehe checkChipRestart).
    //   U_StopMode.ind - Antwort auf stopMode(), also ebenfalls unser eigenes Werk.
    //   L_Data.con - die Bestätigung zu unserem eigenen Telegramm. Sie steht hier nur deshalb, weil sie
    //     NEBEN dem Frame ankam statt dahinter: erkennt der Empfänger das Echo, wartet er im Zustand
    //     FrameAck darauf und hängt sie als Flag an das Telegramm (Receiver::processByte). Ohne erkanntes
    //     Echo - Frame verstümmelt, im Resync verloren, Wiederholung dazwischen - landet sie hier. Das ist
    //     eine Aussage über den Empfangsweg, keine über die Bestätigung, und ablesbar ist sie am fehlenden
    //     'A' im gemeldeten Telegramm sowie an getRxInvalidFrames(). Als Konsolenzeile war sie nur Lärm.
    //
    // An der AUSWERTUNG ändert das nichts: die geschieht für Reset, Configure und L_Data.con schon im Tick
    // (resetIndication/configureIndication/Transmitter::confirmed), lange bevor dieser Eintrag hier ankommt.
    //
    // Übrig bleiben damit genau die Fälle, die etwas bedeuten: ein L_Ackn.ind außerhalb des Busmonitors,
    // ein U_FrameEnd.ind (Marker-Modus, den wir nie einschalten), ein U_FrameState.ind außerhalb eines
    // Versands - und unbekannte Bytes.
    if (value == U_RESET_IND) return;
    if (value == U_STOP_MODE_IND) return;
    if ((value & U_CONFIGURE_MASK) == U_CONFIGURE_IND) return;
    if ((value & L_DATA_CON_MASK) == L_DATA_CON) return;

    const char *name = controlServiceName(value);

    // Ein Steuerbyte außerhalb von Table 13 kann die BCU gar nicht senden - kommt trotzdem eines an, wird
    // der Bytestrom an dieser Stelle fehlgedeutet (verpasstes Folgebyte, Telegrammbyte als Sequenzanfang
    // gelesen, falsche Baudrate). Deshalb als Fehler und nicht als Meldung.
    if (!name)
    {
        printError("Unknown control byte %02X", value);
        return;
    }

    // Der Rohwert steht immer dabei: die übrigen dieser Dienste tragen ihre Aussage in den Bits (etwa
    // positiv/negativ im L_Data.con).
    if (length == 2)
        printMessage("%s %02X %02X", name, value, data[1]);
    else
        printMessage("%s %02X", name, value);
}

// Die Konfiguration hängt daran, dass wir die U_Reset.ind SEHEN - und die kann uns entgehen: verworfen im
// Resync, weg bei einem Überlauf des Ringpuffers, oder mitten in einer Sequenz aufgeschlagen. Dann stünde
// die BCU ohne Adresse da und niemand wüsste es.
//
// Der gemeldete Chip-Zustand schließt diese Lücke, ohne dass dafür etwas Zusätzliches gesendet werden muss:
// POWER-UP und SYNC durchläuft der NCN ausschließlich nach einem Reset ("Entered after Reset State",
// S. 30), und abgefragt wird der Status ohnehin jede Sekunde. Sobald er danach wieder NORMAL meldet, wird
// die Konfiguration erneut abgesetzt.
//
// GENAU DESHALB wird sie nicht stur wiederholt: Erkennen ist billiger als Nachplappern. Ein Nachsetzen im
// Sekundentakt wäre Dauerverkehr auf der Host-Strecke für einen Fall, der fast nie eintritt.
//
// STOP zählt mit, obwohl der Zustand vom Aufrufer selbst kommt (stopMode) und die Konfiguration dort nicht
// verlorengeht. Das kostet ein überflüssiges Nachsetzen nach jedem Stop-Zyklus - dieselben Werte noch
// einmal, also folgenlos. Den Fall zu unterscheiden wäre mehr Zustand als der Fehler wert ist.
void DataLinkLayer::checkChipRestart()
{
    if (!_systemState.isValid()) return;

    if (!_systemState.normalMode())
    {
        _chipOutOfNormal = true;
        return;
    }

    if (!_chipOutOfNormal) return;

    _chipOutOfNormal = false;
    markConfigurationPending();
}

// Der System-Status ist NCN-spezifisch; ohne Antwort des Chips ist er nie dirty, die Typprüfung ist also
// nur Vorsorge. Ausgegeben wird ausschließlich bei Änderung - sonst käme die Zeile bei jeder Abfrage.
void DataLinkLayer::showSystemState()
{
    if (_bcuType != BcuType::Ncn5120) return;
    if (!_systemState.dirty()) return;

    printMessage("%s", _systemState.print().c_str());
}

// Die Bits stehen für ein Ereignis SEIT DER LETZTEN ABFRAGE, nicht für einen Dauerzustand: der Chip löscht
// sie mit dem Melden. Genau deshalb werden sie hier gesammelt ausgegeben und dabei zurückgesetzt.
void DataLinkLayer::showStateErrors()
{
    if (!_stateErrors) return;

    std::string message = "TP Error:";
    if (_stateErrors & U_STATE_SLAVE_COLLISION) message += " SC";
    if (_stateErrors & U_STATE_RECEIVE_ERROR) message += " RE";
    if (_stateErrors & U_STATE_TRANSMIT_ERROR) message += " TE";
    if (_stateErrors & U_STATE_PROTOCOL_ERROR) message += " PE";
    if (_stateErrors & U_STATE_TEMPERATURE_WARNING) message += " TW";

    printError("%s", message.c_str());
    _stateErrors = 0;
}

// ---------------------------------------------------------------------------------------------------
// Konfiguration
// ---------------------------------------------------------------------------------------------------

// Aus dem Hauptkontext. Gesetzt wird sofort, wenn die Verbindung steht - sonst holt es die
// Konfigurations-Epoche nach, sobald die BCU antwortet. Zur Adresse 0 siehe den Header.
bool DataLinkLayer::setOwnAddress(uint16_t address)
{
    _ownAddress = address;

    if (!_connected) return true;         // gemerkt, wird beim Verbinden abgesetzt
    if (applyConfiguration()) return true;

    markConfigurationPending();
    return false;
}

bool DataLinkLayer::setRepetitions(uint8_t nack, uint8_t busy)
{
    if (nack > U_REPETITION_COUNTER_MASK || busy > U_REPETITION_COUNTER_MASK) return false;

    _repetitionsNack = nack;
    _repetitionsBusy = busy;

    if (!_connected) return true;
    if (applyConfiguration()) return true;

    markConfigurationPending();
    return false;
}

// Die Konfiguration ist nicht (vollständig) rausgegangen. Statt eines zweiten Merkers wird die vorhandene
// Epoche benutzt: der Hauptkontext setzt seinen Stand einen Schritt zurück, damit processConnectionState()
// beim nächsten loop() erneut anwendet. _configAppliedEpoch hat nur diesen einen Schreiber, und der Wert
// kann nie mit _configEpoch zusammenfallen - egal ob der Tick sie zwischenzeitlich erhöht.
void DataLinkLayer::markConfigurationPending()
{
    _configAppliedEpoch = _configEpoch - 1;
}

// Setzt beides ab, was die BCU nach einem Reset vergessen hat. Nur aus dem Hauptkontext - queueControl()
// hat dort seinen Produzenten.
//
// Die Zähler werden nur bei Abweichung vom Reset-Zustand geschickt (beide 3, in beiden Datenblättern), die
// Adresse nur wenn eine gesetzt ist. Damit bleibt der Regelfall ohne jeden Zusatzverkehr.
bool DataLinkLayer::applyConfiguration()
{
    if (!_connected) return false;

    // Im Busmonitor ignoriert der Chip U_SetAddress.req, U_SetRepetition.req und U_Configure.req (Tabelle 11,
    // S. 32 - "I", ohne Rückmeldung). Hier wird deshalb NICHTS abgesetzt und false geliefert: die
    // Konfigurationsepoche bleibt damit offen und der erste loop() nach dem Verlassen des Busmonitors holt
    // sie nach. Das trifft sich mit dem Ausgang selbst - verlassen wird der Modus nur per Reset, und der
    // löscht die Konfiguration im Chip ohnehin.
    if (_busMonitor) return false;

    bool complete = true;

    if (_ownAddress != 0)
    {
        uint8_t high = (uint8_t)(_ownAddress >> 8);
        uint8_t low = (uint8_t)(_ownAddress & 0xFF);
        bool queued;

        if (_bcuType == BcuType::Ncn5120)
        {
            // Das vierte Byte ist ein Dummy, das der NCN verlangt (Table 12: "X (don't care)").
            const uint8_t sequence[] = {U_NCN5120_SET_ADDRESS_REQ, high, low, 0xFF};
            queued = _transmitter.queueControl(sequence, sizeof(sequence));
        }
        else
        {
            const uint8_t sequence[] = {U_TPUART2_SET_ADDRESS_REQ, high, low};
            queued = _transmitter.queueControl(sequence, sizeof(sequence));
        }

        // Mit der Adresse übernimmt der Chip die Quittung - und zwar SOFORT, ohne dass er darauf noch
        // hinweisen müsste: "Sets the physical address of the device and activates the auto-acknowledge
        // function" (NCN5130 p.36) bzw. "If the address is set a complete address evaluation in the
        // TP-UART is activated" (Siemens p.23). Deshalb wird der Zustand HIER gesetzt und nicht erst aus
        // dem U_Configure.ind heraus: das gibt es nur beim NCN. Der TPUART2 hat den Dienst gar nicht
        // (Fig. 25 kennt host-seitig nur Reset-, ProductID-, State- und L_Data.confirm-Indication), dort
        // wäre also nie eine Bestätigung gekommen und das Flag stünde für immer falsch.
        if (!queued) complete = false;
        else _autoAcknowledge = true;
    }

    if (_repetitionsNack != 3 || _repetitionsBusy != 3)
    {
        if (_bcuType == BcuType::Ncn5120)
        {
            uint8_t counters = (uint8_t)((_repetitionsBusy << U_NCN5120_REPETITION_BUSY_SHIFT) | _repetitionsNack);
            const uint8_t sequence[] = {U_NCN5120_SET_REPETITION_REQ, counters, 0x00, 0x00};
            if (!_transmitter.queueControl(sequence, sizeof(sequence))) complete = false;
        }
        else
        {
            uint8_t counters = (uint8_t)((_repetitionsBusy << U_TPUART2_REPETITION_BUSY_SHIFT) | _repetitionsNack);
            const uint8_t sequence[] = {U_TPUART2_SET_REPETITION_REQ, counters};
            if (!_transmitter.queueControl(sequence, sizeof(sequence))) complete = false;
        }
    }

    // Die Spannungsregler - siehe queuePowerControl(). Nur wenn der Aufrufer sie je angefasst hat.
    if (!queuePowerControl()) complete = false;

    return complete;
}

// ---------------------------------------------------------------------------------------------------
// Steuerbefehle an die BCU
// ---------------------------------------------------------------------------------------------------

// Der Chip-Zustand gilt erst als umgeschaltet, wenn das Steuerbyte den Chip auch erreicht hat - bis dahin
// würden wir sonst schon Quittungen vom Bus erwarten, die noch gar nicht kommen können (bzw. umgekehrt
// welche als Steuerbytes fehldeuten). Übernommen wird der neue Zustand deshalb in controlByteSent(),
// abgeleitet aus dem vom Transmitter tatsächlich abgesetzten Steuercode.
//
// Schon im Busmonitor? Dann kein Byte senden und true melden - genau das Verhalten der alten Library.
bool DataLinkLayer::startMonitoring()
{
    if (!_connected) return false;
    if (_busMonitor) return true;

    return _transmitter.queueControl(U_BUSMON_REQ);
}

// Der Reset ist zugleich der einzige Weg aus dem Busmonitor heraus (Datenblatt S. 36, Figure 35) und
// stellt nebenbei den Default-CRC-Modus wieder her.
bool DataLinkLayer::reset()
{
    // Nach einem Reset ist der Bytestrom neu aufgesetzt; was vorher lief, ist kein Bezugspunkt mehr für
    // eine Wiederholung. Das Leeren gleich hier ist unkritisch - beide Kontexte sind derselbe.
    _repetitionFilter.clear();

    return _transmitter.queueControl(U_RESET_REQ);
}

// Der System-Status ist NCN-spezifisch. Und er ist die Vorbedingung dafür, dass überhaupt ein
// U_SystemStat.ind eintreffen kann - der einzige mehrbytige Steuerdienst, den der Receiver kennt.
// Im Busmonitor ist der Dienst wirkungslos: Tabelle 11 (S. 32) führt U_State.req und U_SystemStat.req dort
// als "I" - ignoriert, OHNE Rückmeldung an den Host. Es käme also keine U_State.ind zurück, und die beiden
// Bytes stünden nur in einer Spur, die passiv sein soll. Deshalb hier abgefangen und nicht erst an jeder
// Aufrufstelle: requestState() wird auch aus stopMode() und powerControl() gerufen.
bool DataLinkLayer::requestState()
{
    if (_busMonitor) return false;

    if (!_transmitter.queueControl(U_STATE_REQ)) return false;

    if (_bcuType == BcuType::Ncn5120) _transmitter.queueControl(U_SYSTEM_STATE_REQ);
    return true;
}

bool DataLinkLayer::stopMode(bool state)
{
    if (_bcuType != BcuType::Ncn5120) return false; // Dienst gibt es beim TPUART2 nicht

    if (!_transmitter.queueControl(state ? U_STOP_MODE_REQ : U_EXIT_STOP_MODE_REQ)) return false;

    requestState(); // die Auswirkung wird über den Status sichtbar - wie bei powerControl()
    return true;
}

// Im Busmonitor wirkungslos: U_SetBusy.req und U_QuitBusy.req stehen in Tabelle 11 (S. 32) dort als "I".
// Das Byte abzusetzen hieße, eine passive Spur zu verunreinigen für einen Modus, den der Chip gar nicht
// annimmt - und _busyModeSince behauptete danach einen Zustand, den es nicht gibt.
bool DataLinkLayer::busyMode(bool state)
{
    if (_busMonitor) return false;

    bool queued = _bcuType == BcuType::Ncn5120
                      ? _transmitter.queueControl(state ? U_NCN5120_SET_BUSY_REQ : U_NCN5120_QUIT_BUSY_REQ)
                      : _transmitter.queueControl(state ? U_TPUART2_SET_BUSY_REQ : U_TPUART2_QUIT_BUSY_REQ);

    if (!queued) return false;

    // Der Ablauf der Frist wird HIER aufgezogen bzw. gelöscht, checkBusyMode() setzt ihn nur um.
    _busyModeSince = state ? millis() : 0;
    return true;
}

// Aus dem TICK: ein U_Ackn.req ist rausgegangen und hat damit den Busy-Modus im Chip beendet (S. 35).
// Über ein Flag statt direkt, damit _busyModeSince seinen einzigen Schreiber behält.
void DataLinkLayer::reportBusyModeCancelled()
{
    _busyModeCancelled = true;
}

// Nimmt den Busy-Modus nach TPUART_BUSY_MODE_MS von selbst zurück - die Begründung für die Frist steht an
// der Konstante im Header.
//
// Läuft aus loop(), also demselben Kontext, aus dem busyMode() gerufen wird - _busyModeSince hat damit genau
// einen Schreiber und braucht weder volatile noch die Vorzeichenregel aus processConnectionState(). Aus
// demselben Grund räumt resetIndication() das Feld NICHT mit auf, obwohl ein Reset den Busy-Modus im Chip
// beendet: die Funktion läuft im Tick. Die Folge ist ein einzelnes überflüssiges U_QuitBusy.req nach der
// Frist, das einen bereits gelöschten Zustand noch einmal löscht.
void DataLinkLayer::checkBusyMode()
{
    // Der Tick hat quittiert - damit ist der Modus im Chip weg, ganz gleich wie lange die Frist noch
    // gelaufen wäre. Unser Merker zieht nach, statt einen Zustand zu behaupten, den der Chip nicht mehr
    // hat: sonst schwiege sendAcknowledge() weiter, sobald die Auto-Quittung wieder anläuft (sie kommt
    // nach jedem Reset neu), und aus der Absage würde vollständige Stille.
    if (_busyModeCancelled)
    {
        _busyModeCancelled = false;
        _busyModeSince = 0;
    }

    if (_busyModeSince == 0) return;
    if ((uint32_t)(millis() - _busyModeSince) < TPUART_BUSY_MODE_MS) return;

    busyMode(false); // löscht _busyModeSince - und versucht es beim nächsten loop() erneut, falls die
                     // Steuer-Warteschlange gerade voll war
}

// Schreibzugriff auf ACR0 - eine 2-Byte-Sequenz, die ununterbrochen rausgehen muss, deshalb als Gruppe.
// Flag-Werte wie in der alten Library: im ausgeschalteten Zustand bleiben Taktausgang und Strombegrenzung
// aktiv, nur die beiden Regler VCC2 und 20V werden abgeschaltet.
bool DataLinkLayer::powerControl(bool state)
{
    if (_bcuType != BcuType::Ncn5120) return false; // interne Register hat nur die NCN512x-Reihe

    _powerControl = state ? PowerControl::On : PowerControl::Off;

    if (!queuePowerControl()) return false;

    requestState(); // die Auswirkung wird über den Status sichtbar
    return true;
}

// Auch das ist Konfiguration, die ein Reset kassiert - deshalb steht sie hier und wird von
// applyConfiguration() mitgesetzt. ACR0 trägt laut Datenblatt (Table 16) den Reset-Wert 0111 0100, also
// beide Regler AN, und die RESET-Zustandsbeschreibung nennt ausdrücklich beide Auslöser: "Entered after
// Power On Reset (POR) or in response to a U_Reset.req service issued by the host controller. In this state
// NCN5130 gets initialized" (S. 30). Ein powerControl(false) wäre nach jedem Reset also still wieder
// aufgehoben - und Resets schickt diese Library selbst (Wachhund des Transmitters, reset() aus der
// Anwendung).
//
// Sollte ACR0 einen U_Reset.req wider Erwarten doch überleben, kostet das Wiederholen nichts: es schreibt
// denselben Wert. Die Annahme ist damit in beide Richtungen unschädlich - anders als das Weglassen.
bool DataLinkLayer::queuePowerControl()
{
    if (_powerControl == PowerControl::Unset) return true; // nie gesetzt - dann bleibt der Chip, wie er ist

    uint8_t value = ACR0_FLAG_XCLKEN | ACR0_FLAG_V20VCLIMIT;
    if (_powerControl == PowerControl::On) value |= ACR0_FLAG_DC2EN | ACR0_FLAG_V20VEN;

    const uint8_t sequence[] = {U_INT_REG_WR_REQ_ACR0, value};
    return _transmitter.queueControl(sequence, sizeof(sequence));
}

// ---------------------------------------------------------------------------------------------------
// Meldungen
// ---------------------------------------------------------------------------------------------------

// 128 Byte statt der 1024 der alten Library: der längste Text hier ist die System-Status-Zeile mit gut 60
// Zeichen. Der Puffer liegt auf dem Stack, und der ist auf dem RP2040/ESP32 knapper als auf dem PC.
void DataLinkLayer::printMessage(const char *format, ...)
{
    if (!_messageCallback) return;

    char buffer[128];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    _messageCallback(buffer, false);
}

void DataLinkLayer::printError(const char *format, ...)
{
    if (!_messageCallback) return;

    char buffer[128];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    _messageCallback(buffer, true);
}

void DataLinkLayer::registerMessage(MessageCallback callback)
{
    _messageCallback = callback;
}

// ---------------------------------------------------------------------------------------------------
// Zustand und Durchreichen an die beiden Hälften
// ---------------------------------------------------------------------------------------------------

bool DataLinkLayer::isConnected() const
{
    return _connected;
}

uint32_t DataLinkLayer::connectedBaudRate() const
{
    return _connectedBaudRate;
}

BcuType DataLinkLayer::bcuType() const
{
    return _bcuType;
}

BcuState DataLinkLayer::bcuState() const
{
    if (!_initialized) return BcuState::Uninitialized;
    if (_connected) return BcuState::Connected;

    return _everConnected ? BcuState::Disconnected : BcuState::Uninitialized;
}

const char *DataLinkLayer::bcuStateName() const
{
    switch (bcuState())
    {
        case BcuState::Connected:
            return "Connected";
        case BcuState::Disconnected:
            return "Disconnected";
        default:
            return "Uninitialized";
    }
}

bool DataLinkLayer::isAutoAcknowledge() const
{
    return _autoAcknowledge;
}

bool DataLinkLayer::isBusyMode() const
{
    return _busyModeSince != 0;
}

bool DataLinkLayer::isBusMonitor() const
{
    return _busMonitor;
}

// --- KOMPAT: alte Namen -------------------------------------------------------------------------------

bool DataLinkLayer::isMonitoring() const
{
    return isBusMonitor();
}

const char *DataLinkLayer::getBcuStateInfo() const
{
    return bcuStateName();
}

uint16_t DataLinkLayer::ownAddress() const
{
    return _ownAddress;
}

// Unkritisch: gesetzt wird aus dem Hauptkontext, gerufen wird ausschließlich aus loop() - also derselbe
// Kontext, keine Nebenläufigkeit. Genau dafür ist der Ringpuffer da.
void DataLinkLayer::registerFrameCallback(FrameCallback callback)
{
    _frameCallbacks.push_back(callback);
}

// KOMPAT: alter Name.
void DataLinkLayer::registerReceivedFrame(FrameCallback callback)
{
    registerFrameCallback(callback);
}

// Jeder registrierte Callback bekommt dasselbe Frame, in der Reihenfolge der Registrierung - und ALLE
// sehen die Änderungen der vorherigen (setFiltered() etwa). Das ist Absicht und war in der alten Library
// genauso: der Empfangspfad des Stacks und eine Diagnoseausgabe daneben sind zwei Zuhörer, kein Ersatz
// füreinander.
void DataLinkLayer::deliverFrame(Frame &frame)
{
    for (FrameCallback &callback : _frameCallbacks)
        if (callback) callback(frame);
}

void DataLinkLayer::registerCheckAcknowledge(AcknowledgeCallback callback)
{
    _acknowledgeCallback = callback;
}

// Aus dem TICK, mitten im laufenden Frame. Ohne registrierte Entscheidung wird NICHT quittiert - genauso
// wie die alte Library, deren checkAcknowledge() ohne Callback ACK_None liefert. Alles zu quittieren wäre
// am echten Bus falsch: das Gerät gäbe damit vor, unter jeder Zieladresse erreichbar zu sein.
AckType DataLinkLayer::checkAcknowledge(uint16_t destination, bool isGroupAddress)
{
    if (!_acknowledgeCallback) return AckType::None;

    // GEMESSEN, WEIL ES DER EINZIGE UNBEGRENZTE ANTEIL DES TICKS IST. Alles andere hier ist ein Byte je
    // Richtung und nach oben beschränkt; was dieser Callback kostet, bestimmt der Aufrufer, und er läuft
    // je nach Plattform in einem Interrupt. Ohne die Messung ist getTickDurationMaxUs() nicht zuzuordnen -
    // man sieht, dass ein Tick lange lief, aber nicht, ob die Library oder der Aufrufer schuld war.
    //
    // Die zwei micros() kosten hier nichts: dieser Zweig läuft genau EINMAL je empfangenem Telegramm
    // (bei Byte 6, wenn die Größe bekannt wird), nicht bei jedem Tick.
    uint32_t startedAt = micros();
    AckType result = _acknowledgeCallback(destination, isGroupAddress);
    _statistics.updateCheckAcknowledgeMaxUs(micros() - startedAt);

    return result;
}

// Aus dem Tick. Beides zusammen, weil beides dasselbe Ereignis beschreibt: der Merker beantwortet "ist
// gerade etwas passiert", der Zähler "wie oft insgesamt".
void DataLinkLayer::reportInterfaceOverflow()
{
    _interfaceOverflow = true;
    _statistics.incrementRxInterfaceOverflows();
}

void DataLinkLayer::reportRxQueueOverflow()
{
    _rxQueueOverflow = true;
    _statistics.incrementRxQueueOverflows();
}

void DataLinkLayer::reportControlOverflow()
{
    _ctrlQueueOverflow = true;
    _statistics.incrementTxControlQueueOverflows();
}

bool DataLinkLayer::queueOverflow()
{
    if (!_rxQueueOverflow) return false;

    _rxQueueOverflow = false;
    return true;
}

bool DataLinkLayer::interfaceOverflow()
{
    if (!_interfaceOverflow) return false;

    _interfaceOverflow = false;
    return true;
}

bool DataLinkLayer::controlOverflow()
{
    if (!_ctrlQueueOverflow) return false;

    _ctrlQueueOverflow = false;
    return true;
}

Receiver &DataLinkLayer::getReceiver()
{
    return _receiver;
}

Transmitter &DataLinkLayer::getTransmitter()
{
    return _transmitter;
}

Statistics &DataLinkLayer::getStatistics()
{
    return _statistics;
}

RepetitionFilter &DataLinkLayer::getRepetitionFilter()
{
    return _repetitionFilter;
}

SystemState &DataLinkLayer::getSystemState()
{
    return _systemState;
}

bool DataLinkLayer::pushTransmitQueue(const Frame &frame)
{
    return _transmitter.pushTransmitQueue(frame);
}

// Wickelt die Bytes in ein Frame und delegiert. Die Kopie ist hinnehmbar: sie läuft im Hauptkontext, und
// gespart würde sie nur durch eine nicht besitzende Sicht - dafür müsste Frame einen Zeiger statt seines
// festen Arrays halten, und das ist ein Eingriff in eine Klasse, die überall benutzt wird.
bool DataLinkLayer::pushTransmitQueue(const uint8_t *data, size_t length)
{
    if (data == nullptr) return false;
    if (length == 0 || length > TPUART_BUFFER_SIZE) return false;

    return pushTransmitQueue(Frame(data, length));
}

// KOMPAT: die Zeigerform übernimmt als einzige den BESITZ - so war es in der alten Library, und der
// knx-Stack verlässt sich darauf (er gibt das Frame nur im Fehlerfall selbst frei).
//
// Gekürzt wird hier NICHTS mehr. Früher ging die Länge um eins reduziert weiter, weil die Schicht die
// Prüfsumme selbst anhängte; jetzt gilt durchgehend "ein Frame ist ein vollständiges Telegramm
// einschließlich Prüfsumme", und die wird geprüft statt neu gerechnet.
bool DataLinkLayer::pushTransmitQueue(Frame *frame)
{
    if (frame == nullptr) return false;

    if (!pushTransmitQueue(*frame)) return false;

    delete frame;
    return true;
}

} // namespace TPUart
