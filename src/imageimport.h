#pragma once

#include <QString>
#include <QStringList>

class QMimeData;

namespace ot {
struct ImageDropResult {
    QStringList paths;
    QString error;
    // True only when every item is valid. Invalid mixed drops never return a
    // partial path list, so the caller cannot silently lose an input image.
    bool supported = false;
};

// Inspect 1..8 local PNG/JPEG/BMP file URLs. This reads image metadata only;
// it neither decodes the full image nor fetches remote URLs.
ImageDropResult inspectImageDrop(const QMimeData *mimeData);
QStringList localImagePaths(const QMimeData *mimeData);
QString imageDropError(const QMimeData *mimeData);
} // namespace ot
