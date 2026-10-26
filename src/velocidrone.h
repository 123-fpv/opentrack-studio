#pragma once

#include <QColor>
#include <QString>
#include <QVector>

namespace ot {
struct TrackDocument;
struct NativeBlockColor {
    int prefab;
    QString name;
    QColor color;
};
QVector<NativeBlockColor> nativeBlockColors();
// Closest available solid-block material by squared RGB distance. This chooses
// a native prefab; it does not add an undocumented RGB property to .trk files.
NativeBlockColor nearestNativeBlockColor(const QColor &color);

// Calibrated stock-frame solids are emitted once per physical component. Each
// ordered pass gets a separate native88 checkpoint; guides stay project-only.
// Export requires an independent takeoff grid. Legacy native models and opaque
// fields remain intact within their documented routing limits.
// No simulator account or network is used.
// The Windows build uses the system AES implementation and links against bcrypt.
bool exportVelocidrone(const QString &path, const TrackDocument &document,
                       QString *error = nullptr);
bool importVelocidrone(const QString &path, TrackDocument *document, QString *error = nullptr);
} // namespace ot
