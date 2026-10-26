#pragma once

#include "document.h"
#include <QColor>
#include <QStringList>

namespace ot {

// Local, deterministic recognition of coloured gate symbols in overhead plans.
// It does not reconstruct heights, identify arbitrary photographs, or recover
// authoritative checkpoint order from an unannotated picture.
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
