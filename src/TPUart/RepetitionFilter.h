#pragma once
#include <stddef.h>
#include <stdint.h>

namespace TPUart
{

class Frame;

// Wie viele Absender gleichzeitig beobachtet werden. Wie in der alten Library 50; mehr Geräte, die sich in
// ihren Sendungen abwechseln, gibt es real nicht. Wird es doch eng, verdrängt der am längsten nicht mehr
// gesehene Absender - schlimmstenfalls kommt eine Wiederholung durch, nichts geht verloren.
#ifndef TPUART_REPETITION_FILTER_COUNT
#define TPUART_REPETITION_FILTER_COUNT 50
#endif

// ES GIBT BEWUSST KEINE VERFALLSFRIST. Ein Eintrag verschwindet ausschließlich dadurch, dass neue
// Telegramme anderer Absender ihn aus der Liste verdrängen - genau wie in der alten Library.
//
// Ein Zeitlimit wäre nicht ableitbar: wann die Wiederholung kommt, hängt daran, wann die Sendeseite den Bus
// wieder frei sieht, das können Millisekunden sein oder zehn Sekunden. Eine zu knappe Frist lässt die
// Wiederholung als vermeintliches Original durch - also genau das, was der Filter verhindern soll.
// (Kurzzeitig stand hier eine 2s-Frist. Wieder entfernt; nicht erneut einbauen.)
//
// Der eine Fall, in dem der Filter etwas kostet, wird bewusst hingenommen: Wenn wir ein Original nicht
// mitbekommen, dessen Wiederholung eintrifft und der gespeicherte Fingerabdruck noch von einem früheren,
// byteweise identischen Telegramm desselben Absenders stammt, wird das einzige Exemplar, das bei uns
// ankommt, als Wiederholung markiert. Von außen ist ohnehin nicht entscheidbar, zu welchem der beiden
// gleichen Telegramme eine Wiederholung gehört - dann lieber Pech gehabt als ein Mechanismus, der
// vorgibt, es zu wissen.

// Erkennt Wiederholungen: dasselbe Telegramm, das die Sendeseite noch einmal auf den Bus legt, weil ihr die
// Quittung fehlte. Empfangen haben wir es beim ersten Mal längst - der Aufrufer soll es also sehen, aber
// nicht zweimal verarbeiten (TP_FRAME_FLAG_FILTERED).
//
// DIE QUELLADRESSE IST DER SCHLÜSSEL, und zwar ein eindeutiger: auf TP1 sendet immer nur ein Gerät, und
// eine Wiederholung folgt dem eigenen Original unmittelbar. Ein Absender kann also nie zwei Telegramme
// gleichzeitig offen haben - nur sein letztes ist überhaupt ein möglicher Bezugspunkt. Deshalb genau ein
// Eintrag je Absender, der bei jedem neuen Telegramm überschrieben wird; eine Liste mehrerer
// Fingerabdrücke pro Gerät hätte keinen Fall, den sie abdeckt.
//
// Gemerkt wird davon ein 16-Bit-Fingerabdruck, nicht das Telegramm selbst: 50 Telegramme in Maximalgröße
// wären 13 KB, so sind es 400 Byte. Der Preis ist eine theoretische Kollision - zwei verschiedene
// Telegramme desselben Absenders mit gleichem Fingerabdruck, unmittelbar hintereinander. Bei 16 Bit
// vertretbar.
//
// Läuft ausschließlich im Hauptkontext (Receiver::processQueue()), deshalb kein volatile und keine
// Nebenläufigkeitsvorkehrungen.
class RepetitionFilter
{
  private:
    struct Entry
    {
        uint16_t _source;
        uint16_t _checksum;
        uint32_t _timestamp;
        bool _used;
    };

    Entry _entries[TPUART_REPETITION_FILTER_COUNT] = {};

    // CRC-16/SPI-FUJITSU über alles außer dem Prüfsummen-Oktett, wie in der alten Library. Das
    // Wiederholungs-Bit im Kontrollbyte wird dabei fest gesetzt - sonst hätten Original und Wiederholung
    // unterschiedliche Fingerabdrücke, und genau die sollen ja als gleich erkannt werden.
    static uint16_t fingerprint(const Frame &frame);

  public:
    // Meldet, ob genau dieses Telegramm von diesem Absender gerade schon einmal da war, und merkt es sich
    // anschließend. Für JEDES Telegramm aufzurufen, nicht nur für Wiederholungen - sonst fehlt der
    // Vergleichswert, wenn die Wiederholung kommt.
    bool check(const Frame &frame);

    void clear();

    // Belegte Plätze - für die Diagnose.
    size_t size() const;
};

} // namespace TPUart
