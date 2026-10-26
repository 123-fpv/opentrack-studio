#pragma once

#include "document.h"
#include <QByteArray>
#include <QList>
#include <QStringList>
#include <atomic>
#include <memory>

namespace ot {

struct AiSettings {
    QString baseUrl = QStringLiteral("https://api.deepseek.com");
    QString model = QStringLiteral("deepseek-flash");
    QString apiKey;
    double knownGateWidth = 3.0; // Reference frame centreline spacing, never pillar thickness.
    QString notes;
};

struct AiResult {
    TrackDocument document;
    QStringList warnings;
    QString error;
    bool ok = false;
};

// Request construction and parsers are local and never perform network access.
// The API key belongs only in the HTTP Authorization header, never the JSON.
QByteArray buildVisionRequest(const QList<QImage> &images, const AiSettings &settings,
                              QString *error = nullptr);
AiResult parseVisionResponse(const QByteArray &response, const AiSettings &settings);
QString validateVisionModelResponse(const QByteArray &response, const QString &model);

// Run network functions on a worker thread. Each request has a 90-second
// deadline, a 4-MiB response cap, and no automatic redirects.
AiResult generateWithVision(const QList<QImage> &images, const AiSettings &settings,
                            const std::shared_ptr<std::atomic_bool> &cancel = {});
QString validateVisionModel(const AiSettings &settings,
                            const std::shared_ptr<std::atomic_bool> &cancel = {});

// Windows Credential Manager, generic target OpenTrackStudio/DeepSeek.
// No plaintext settings/project files or diagnostic logging are used.
bool storeApiKey(const QString &apiKey, QString *error = nullptr);
QString loadApiKey(QString *error = nullptr);
void clearApiKey();

} // namespace ot
