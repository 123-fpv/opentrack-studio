#pragma once

#include "document.h"
#include <QColor>
#include <QStringList>

namespace ot {

// Local recognition of schematic symbols. Dark outlined plans with a magenta
// racing line use obstacle shapes and the drawn route graph, including compound
// and edge-on gates. Original line samples inside symbols determine repeated
// aperture passes; occlusion continuation and 3D turn depth remain estimates.
// Visible/occluded stock crossings resolve progressive depth, aperture offsets
// and oblique flight tangents while retaining source height/curvature. Edge-on
// along-bar extent remains inferred. No gate type alone inserts a full return;
// missing occlusion evidence is retained for review rather than invented.
// Subpixel stroke fairing preserves physical crossings and essential checks;
// complete directed traces establish first-encounter order. Unresolved crossings
// or nonlocal repeats retain an explicit order-review flag instead of a guess.
// Blue-icon plans retain the simple detector below. Heights and unreadable or
// ambiguous passages require human review; no missing cells are filled by type.
// arbitrary photographs are outside this local recognizer's scope.
struct RecognitionOptions {
    double metersPerPixel = 0.05;
    int minComponentPixels = 10;
    QColor gateColor = QColor("#3688cf");
    bool autoScale = true;
};

struct RecognitionResult {
    TrackDocument document;
    QStringList warnings;
    int detectedCount = 0;
};

RecognitionResult recognizeTrack(const QImage &image, const RecognitionOptions &options = {});

} // namespace ot
