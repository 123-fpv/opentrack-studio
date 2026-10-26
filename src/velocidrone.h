#pragma once

#include <QString>

namespace ot {
struct TrackDocument;

// Native .trk files, generated locally. No simulator account or network is used.
// The Windows build uses the system AES implementation and links against bcrypt.
bool exportVelocidrone(const QString &path, const TrackDocument &document,
                       QString *error = nullptr);
bool importVelocidrone(const QString &path, TrackDocument *document, QString *error = nullptr);
} // namespace ot
