#pragma once
#include <stdint.h>

#if defined(ARDUINO_ARCH_RP2040)
    #include <pico/time.h>

// PRIORITÄT DES TIMER-INTERRUPTS auf dem RP2040. Numerisch KLEINER heißt höher; das SDK setzt beim
// Hochlauf alles auf PICO_DEFAULT_IRQ_PRIORITY (0x80), und auf dem Cortex-M0+ zählen nur die oberen
// 2 Bit - es gibt also genau vier Stufen: 0x00, 0x40, 0x80, 0xC0.
//
// WARUM ÜBERHAUPT: ein Interrupt verdrängt auf Cortex-M sehr wohl einen anderen (dafür steht das N in
// NVIC), aber NUR bei echt höherer Priorität - GLEICHE Priorität verdrängt nicht. Solange wir auf 0x80
// mitschwimmen, wartet der Timer auf jeden anderen Handler, der gerade läuft, und das sind alle.
//
// WARUM 0x40 UND NICHT 0x00: im gesamten SDK-Quellbaum gibt es genau einen expliziten
// irq_set_priority()-Aufruf, und der setzt die Hintergrundarbeit des Netzwerkstacks nach UNTEN
// (async_context_threadsafe_background auf PICO_LOWEST_IRQ_PRIORITY). Über 0x80 sitzt damit niemand,
// 0x40 genügt also. 0x00 nähme nur USB und DMA die Luft, ohne etwas hinzuzugewinnen.
//
// WAS ES NICHT LÖST: PRIMASK. noInterrupts(), save_and_disable_interrupts() und
// critical_section_enter_blocking() sperren unabhängig von der Priorität. Wer eine solche Klammer lange
// hält, hält auch uns an - dagegen hilft nur ein Tick auf dem anderen Kern, denn PRIMASK ist pro Kern.
// Flash-Schreibvorgänge bleiben in jedem Fall ein Loch, weil sie zusätzlich den zweiten Kern parken.
    #ifndef TPUART_RP2040_TIMER_IRQ_PRIORITY
        #define TPUART_RP2040_TIMER_IRQ_PRIORITY 0x40
    #endif
#elif defined(ARDUINO_ARCH_ESP32)
    #include <esp_timer.h>
#endif

namespace TPUart
{

class DataLinkLayer;

// Der Takt, in dem tick() laufen soll. 500µs sind für ein normales Gerät reichlich: der Bus liefert
// höchstens alle 1,354ms ein Byte (9600 Baud, 13 Bitzeiten je TP1-Zeichen), empfangsseitig ist also
// Luft nach oben.
//
// WER VIEL SENDET, SETZT IHN HERUNTER - aber nur noch auf manchen Interfaces, und das ist gemessen, nicht
// geschätzt. Die Sendehardware fasst wenig: der RP2040-UART läuft ohne FIFO (die DMA soll jedes
// Empfangsbyte sofort bekommen) und hält deshalb nur zwei Bytes, Halte- plus Schieberegister, bei 38400
// Baud also 2 x 286µs Vorrat. Wird das ausschließlich aus dem Tick nachgelegt, steht die Leitung zwischen
// den Ticks zeitweise still: bei 500µs Takt kommen so 3 Bytes je 1000µs statt 3,5 durch, rund 14% Verlust.
// Gemessen an einem Router mit Telegrammen in Maximalgröße: 436 Hostbytes brauchten 143ms statt der 124ms,
// die die Leitung hergibt.
//
// DAS RP2040-INTERFACE IST DAVON SEIT DEM TX-INTERRUPT AUSGENOMMEN. Dort schiebt die Hardware sich selbst
// nach, sobald ein Platz frei wird (siehe RP2040.h), und der Durchsatz hängt nicht mehr am Takt.
//
// DIE REGEL GILT NUR FÜR ArduinoSerial, und der Grund ist nicht "nur der Tick füllt nach", sondern WIE
// VIEL er pro Aufruf hineinbekommt. Der RP2040-UART ohne FIFO hält zwei Bytes, und das Halteregister wird
// erst frei, wenn das Schieberegister das vorige übernommen hat - dort geht also nur EIN Byte je Tick
// hinein, egal was der Transmitter anbietet. Deshalb muss der Takt unter einer Zeichenzeit der
// HOSTLEITUNG liegen: 286µs bei 38400 Baud, 573µs bei 19200 - für einen Router dort also
// -D TPUART_TIMER_INTERVAL_US=250, zum Preis von rund 1% Rechenzeit.
//
// DER ESP32 BRAUCHT DAS NICHT. Sein Treiber hat einen 512-Byte-Ring und nimmt beide Bytes eines Oktetts im
// selben Tick (Transmitter::process() verlangt 2 bzw. 3 Bytes und schreibt sie zusammen). Ein Tick liefert
// damit ein ganzes Oktett: bei 19200 belegt das die Leitung 1146µs, bei 38400 noch 572µs - der 500µs-Takt
// ist in beiden Fällen schneller als die Leitung, die Leitung bleibt der Engpass. Genau so soll es sein.
//
// Empfangsseitig ändert der Takt ohnehin nichts, dort sind 500µs auf jedem Interface reichlich.
#ifndef TPUART_TIMER_INTERVAL_US
    #define TPUART_TIMER_INTERVAL_US 500
#endif

// AB HIER IST DER ANTRIEB GRUNDSÄTZLICH ZU LANGSAM, und die Zahl kommt aus dem Bus statt aus der
// Konfiguration: ein Tick bewegt ein Byte je Richtung, ein TP1-Zeichen dauert 1,354ms (9600 Baud,
// 13 Bitzeiten). Liegt der mittlere Tickabstand darüber, holt die Schicht weniger Bytes ab als der Bus im
// Vollausbau liefert - dann füllen sich die Puffer, bis etwas überläuft, und keine Puffergröße hilft
// dagegen. Das ist deshalb keine Empfehlung, sondern eine Untergrenze.
//
// DataLinkLayer::checkTickRate() misst dagegen die ERREICHTE Rate und meldet, wenn sie darüber liegt. Der
// Wert ist bewusst nicht der eingestellte Takt: dass jemand 500µs wollte und 2200µs bekommt, ist die eine
// Frage - ob das Ergebnis überhaupt für den Bus reicht, die andere und wichtigere.
#ifndef TPUART_TICK_DEFERRED_US
    #define TPUART_TICK_DEFERRED_US 1354
#endif

// Fenster, über das gemittelt wird. Lang genug, dass eine einzelne lange Runde im Hauptloop nichts
// auslöst - einzelne Verzögerungen zählt getTickDeferrals(), hier geht es um den Dauerzustand.
#ifndef TPUART_TICK_RATE_WINDOW_MS
    #define TPUART_TICK_RATE_WINDOW_MS 2000
#endif

// WIE VIELE DataLinkLayer der Timer treiben kann. Die Vorgabe ist 1, weil ein Gerät mit EINER BCU der
// Normalfall ist; zwei oder drei sind der Sonderfall und kosten dann bewusst ein -D.
//
// HIER STAND, ein größeres Array belaste "den am häufigsten ausgeführten Code dieser Library". Das war
// überzogen: der Callback läuft über leere Plätze mit einem Zeigervergleich hinweg, vier Plätze sind also
// grob 25 Takte von den 66.500, die bei 133MHz in einem 500µs-Tick stecken - 0,04%. Der Grund für die 1
// ist deshalb nicht die Laufzeit, sondern dass eine Grenze überhaupt sichtbar sein soll: wer eine zweite
// BCU anschließt, soll das einmal ausdrücklich hinschreiben müssen.
//
// UND DER FEHLFALL IST LAUT, das trägt die Entscheidung. Eine Instanz ohne Platz wird nicht getickt,
// DataLinkLayer::checkTickRate() sieht in ihrem loop() null Ticks im Fenster und meldet
// "Tick stopped - no tick in ... ms". Die Konsole von OGM-Common zeigt zusätzlich "(no slot)". Ein
// vergessenes -D fällt damit binnen zwei Sekunden auf und nicht erst im Betrieb.
#ifndef TPUART_TIMER_MAX_CLIENTS
    #define TPUART_TIMER_MAX_CLIENTS 1
#endif

// DER ANTRIEB FÜR tick(), EINMAL FÜR ALLE BCUs.
//
// Das Warum steht im Klassenkommentar des DataLinkLayer: tick() bearbeitet pro Aufruf genau ein Byte je
// Richtung. Aus dem Hauptloop gerufen hängt der Durchsatz damit an dessen Frequenz, und die
// Quittungsfrist von 2,8ms hängt daran mit - eine einzelne langsame Runde in einem fremden Modul kostet
// die Quittung. Mit eigenem Takt ist beides von der Anwendung entkoppelt.
//
// WARUM EIN SINGLETON UND NICHT EINER JE DataLinkLayer: die Library unterstützt mehrere BCUs an einem
// Gerät, und ein Timer je Instanz ist dafür nicht tragfähig. Auf dem RP2040 gibt es genau VIER
// Hardware-Alarme (NUM_ALARMS), von denen der Default-Pool einen belegt - noch im Runtime-Init, vor
// main(). Frei sind also drei, und die vierte Instanz fiel stillschweigend auf den Default-Pool zurück:
// Priorität 0x80 statt 0x40, serialisiert mit fremden Callbacks, also genau der Zustand, dessen
// Beseitigung den eigenen Pool überhaupt gerechtfertigt hat - und nichts meldete es. Mit einem Timer für
// alle stellt sich die Frage nicht mehr.
//
// Dass getrennte Timer auch nichts einbrächten, kommt hinzu: mehrere Alarme auf 0x40 verdrängen sich
// NICHT gegenseitig (gleiche Priorität verdrängt nicht), sie serialisieren genauso wie ein gemeinsamer
// Timer - nur unvorhersehbar statt in fester Reihenfolge.
//
// WAS DAS KOSTET: die Ticks aller Instanzen laufen hintereinander in EINEM Aufruf. Der gemessene
// Höchstwert eines einzelnen Ticks war 660µs, davon 269µs im Quittungs-Callback des Aufrufers; der Median
// liegt bei 1-2µs. Drei Instanzen im schlimmsten Fall wären also rund 2ms gegen ein Quittungsfenster von
// 2,8ms - praktisch unkritisch, aber sichtbar bleiben muss es: getTickDurationMaxUs() gibt es je
// DataLinkLayer, nicht hier.
//
// DIE BEIDEN PLATTFORMEN LÖSEN DAS UNTERSCHIEDLICH, und der Unterschied ist keine Geschmacksfrage:
//
//   RP2040 - Hardware-Timer in einem EIGENEN Alarm-Pool, tick() läuft im INTERRUPT. Erlaubt ist das nur,
//            weil beide dort verwendbaren Interfaces ISR-tauglich schreiben: RP2040 über seinen
//            Software-TX-Ring, ArduinoSerial über uart_putc_raw().
//
//   ESP32  - esp_timer, tick() läuft im TASK-Kontext des Timer-Tasks. Ein echter Interrupt scheidet aus:
//            uart_write_bytes() im ESP32-Interface nimmt einen Mutex, den ein ISR nicht nehmen darf.
//            Bewusst NICHT als eigener Task mit delayMicroseconds(): der bräuchte einen ganzen Kern für
//            sich, und auf Kern 0 sitzen im echten Gerät WLAN und Bluetooth.
//
// Auf jeder anderen Plattform meldet supported() false und add() schlägt fehl - dann tickt NICHTS, bis
// jemand trigger() aus einem eigenen Kontext ruft. Einen Rückfall auf den Hauptloop gibt es bewusst nicht:
// DataLinkLayer::process() tickt nie (Begründung dort).
//
// NUR EIN ANTRIEB GLEICHZEITIG JE DataLinkLayer. Treibt dieser Timer eine Instanz, darf tick() für sie von
// nirgendwo sonst kommen: zwei Kontexte in derselben State-Machine sind kein Nachteil, sondern ein Defekt.
// Wer selbst treiben will - eigener Task, fremder Timer, zweiter Kern -, hält den Timer mit setInterval(0)
// an und ruft trigger().
class Timer
{
  private:
    // FESTES ARRAY, kein Container: der Callback läuft im Interrupt, dort wird nicht allokiert. Ein
    // einzelner Zeiger wird atomar geschrieben, ein Platz ist also entweder leer oder vollständig belegt -
    // der Callback kann keinen halben Eintrag sehen. Deshalb braucht add() keine Sperre.
    DataLinkLayer *volatile _clients[TPUART_TIMER_MAX_CLIENTS] = {};

    // GLOBAL, nicht je Instanz - so hat der Anwender es festgelegt. Ein Takt, der für eine BCU reicht,
    // reicht auch für die zweite; zwei verschiedene Intervalle wären ein zweiter Timer.
    uint32_t _intervalUs = TPUART_TIMER_INTERVAL_US;

    // Vom Hauptkontext geschrieben, vom Timer gelesen (RP2040 zusätzlich aus dem Interrupt).
    volatile bool _running = false;

#if defined(ARDUINO_ARCH_RP2040)
    repeating_timer_t _timer;

    // EIGENER ALARM-POOL, statt des Default-Pools. Zwei Gründe, und beide zählen: die Callbacks EINES
    // Pools laufen aus einem gemeinsamen IRQ-Handler und damit serialisiert - wer sonst noch drinhängt,
    // verzögert uns um seine Laufzeit. Und nur ein eigener Pool hat einen eigenen Hardware-Alarm, dessen
    // Priorität sich anheben lässt (siehe TPUART_RP2040_TIMER_IRQ_PRIORITY).
    //
    // Einmal angelegt und behalten: der Timer kann nach einem end()/begin() erneut starten, und je
    // Durchlauf einen Pool zu erzeugen wäre ein Leck - alarm_pool_create() allokiert.
    alarm_pool_t *_pool = nullptr;

    // Ein einziger Timer je Pool, mehr wird hier nie eingehängt - und genau das ist der Sinn des
    // Singletons.
    static constexpr uint32_t POOL_MAX_TIMERS = 1;

    void claimOwnPool();
    static bool onTimer(repeating_timer_t *timer);
#elif defined(ARDUINO_ARCH_ESP32)
    esp_timer_handle_t _timer = nullptr;
    static void onTimer(void *argument);
#endif

    // Ruft tick() für jeden eingetragenen DataLinkLayer, in Reihenfolge der Plätze.
    void fire();

    bool startTimer();
    void stopTimer();

    // Nicht von außen erzeugbar: es gibt genau einen Timer, und ein zweiter wäre kein zweiter Antrieb,
    // sondern ein zweiter Kontext auf denselben Instanzen.
    Timer() = default;

    // ALLE MEMBER MÜSSEN TRIVIAL ZERSTÖRBAR BLEIBEN, und das ist eine echte Bedingung, keine Stilnote:
    // ~DataLinkLayer() greift auf instance(), um sich auszutragen. Ist der DataLinkLayer ein globales
    // Objekt, ist die Reihenfolge der statischen Zerstörung über Übersetzungseinheiten hinweg
    // unspezifiziert - er kann also NACH diesem Objekt zerstört werden. Solange hier kein nichttrivialer
    // Destruktor läuft, ist das harmlos: der Speicher bleibt für die Programmlaufzeit gültig. Wer hier
    // einen Member mit Destruktor einbaut (Container, String, Smart Pointer), macht daraus ein
    // Use-after-destruction, das nur bei globalen Instanzen und nur beim Herunterfahren auftritt.
    static Timer _instance;

  public:
    // Das Singleton. Ein Klassenmember mit statischer Speicherdauer und KEIN funktionslokales static:
    // letzteres bekäme eine Guard-Variable und damit einen __cxa_guard_acquire-Aufruf bei jedem Zugriff -
    // und instance() wird aus dem Timer-Callback erreichbar sein müssen.
    static Timer &instance();

    // Ob die Plattform überhaupt einen Timer hergibt. Compile-time-Eigenschaft, trotzdem als Funktion:
    // so muss der Aufrufer die #ifdef-Kette nicht nachbauen.
    static bool supported();

    // Trägt eine Instanz ein und startet den Timer, falls er noch nicht läuft. Liefert false, wenn die
    // Plattform keinen Timer hat, das Intervall 0 ist oder alle Plätze belegt sind - dann wird diese Instanz
    // nur noch von trigger() getickt, und DataLinkLayer::usesTimer() sagt es.
    bool add(DataLinkLayer &dll);

    // Trägt eine Instanz aus. Nach der Rückkehr läuft garantiert kein tick() mehr für sie - das ist die
    // Bedingung dafür, dass end() danach das Interface schließen darf.
    void remove(DataLinkLayer &dll);

    // VORBEDINGUNG AN DEN AUFRUFER, auf dem RP2040: add() und remove() - also begin(), end() und der
    // Destruktor des DataLinkLayer - müssen vom SELBEN Kern kommen. Der Alarm-Pool gehört dem Kern, der ihn
    // erzeugt hat, und die Zusage von remove() ("danach läuft kein tick() mehr") ruht darauf, dass der
    // Callback im Interrupt genau dieses Kerns läuft und vom Hauptkontext nicht unterbrochen wird.
    //
    // Wird das verletzt - etwa end() aus loop1(), während begin() aus setup() kam -, kann ein tick() noch
    // laufen, während die Instanz abgebaut wird. Geprüft wird es NICHT: das ist dieselbe Art von
    // Nutzungsfehler wie tick() aus zwei Kontexten zu treiben, und dagegen hilft kein Laufzeitwächter,
    // sondern nur diese Zeile. Auf dem ESP32 stellt sich die Frage nicht, dort wartet esp_timer_stop().

    bool contains(const DataLinkLayer &dll) const;

    // GLOBAL für alle eingetragenen Instanzen. 0 schaltet den Timer ab; ein laufender Timer wird mit dem
    // neuen Wert neu aufgesetzt.
    void setInterval(uint32_t intervalUs);
    uint32_t interval() const;

    bool running() const;
    uint8_t clients() const;

    // VON HAND TREIBEN, für Plattformen ohne eigene Timer-Unterstützung (supported() == false) oder für
    // einen Aufrufer, der den Takt aus einem eigenen Task, einem fremden Timer oder vom zweiten Kern setzen
    // will. Tut genau das, was der plattformeigene Callback tut: ein tick() je eingetragener Instanz.
    //
    // NEBEN EINEM LAUFENDEN TIMER IST DAS EIN DEFEKT - zwei Tick-Kontexte auf einer Schnittstelle. Wer von
    // Hand treibt, hält den Timer vorher mit setInterval(0) an.
    //
    // Es gilt dieselbe Anforderung wie an den Callback: der Aufrufkontext muss das dürfen, was tick() tut.
    // Auf dem ESP32 heißt das kein ISR, weil uart_write_bytes() einen Mutex nimmt.
    void trigger();
};

} // namespace TPUart
