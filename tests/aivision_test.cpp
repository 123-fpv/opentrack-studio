#include "aivision.h"

#include <QCoreApplication>
#include <QJsonArray>
#include <QJsonDocument>
#include <cmath>
#include <iostream>

namespace {
int failures = 0;
void check(bool condition, const char *message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        ++failures;
    }
}

QJsonObject gate(double x = 0, double z = 0, double yaw = 0) {
    return {{"type", "standard"}, {"x", x},          {"z", z},
            {"elevation", 1.2},   {"width", 2.4},    {"height", 2.4},
            {"yaw", yaw},         {"label", "Gate"}, {"confidence", 0.65}};
}
QJsonObject draft() {
    return {{"track", QJsonObject{{"name", "AI course"},
                                  {"scenery", "Football Stadium"},
                                  {"gates", QJsonArray{gate(0, 0, 90), gate(10, 5, 0)}}}},
            {"warnings", QJsonArray{"Uncertain crossing order"}},
            {"scaleConfidence", 0.4}};
}
QByteArray response(const QJsonObject &object, bool fenced = false,
                    const QString &finish = "stop") {
    QString content = QString::fromUtf8(QJsonDocument(object).toJson(QJsonDocument::Compact));
    if (fenced)
        content = "```json\n" + content + "\n```";
    return QJsonDocument(QJsonObject{{"choices", QJsonArray{QJsonObject{
                                                     {"message", QJsonObject{{"content", content}}},
                                                     {"finish_reason", finish}}}}})
        .toJson(QJsonDocument::Compact);
}
} // namespace

int main(int argc, char **argv) {
    QCoreApplication application(argc, argv);
    ot::AiSettings settings;
    // A deliberate non-credential sentinel. These tests never call the network
    // or read/write/delete the user's Credential Manager entry.
    settings.apiKey = QStringLiteral("never-network-test-value");
    settings.notes =
        QStringLiteral("Use the diagram and the photograph together. ") + settings.apiKey;
    QImage diagram(3200, 2000, QImage::Format_ARGB32);
    diagram.fill(QColor(80, 140, 210, 150));
    QImage photo(800, 600, QImage::Format_RGB32);
    photo.fill(QColor("#e7ded2"));
    QString error;
    const auto payload = ot::buildVisionRequest({diagram, photo}, settings, &error);
    check(!payload.isEmpty() && error.isEmpty(), "two reference images produce a request");
    check(!payload.contains(settings.apiKey.toUtf8()),
          "API key is absent from JSON, including user notes");
    const auto request = QJsonDocument::fromJson(payload).object();
    check(request.value("model").toString() == "deepseek-flash",
          "default is the native image-capable Flash model");
    check(request.value("response_format").toObject().value("type").toString() == "json_object",
          "strict JSON output requested");
    const auto messages = request.value("messages").toArray();
    check(messages.size() == 2 && messages[0].toObject().value("role").toString() == "system",
          "instructions and image data are separate messages");
    const auto parts = messages[1].toObject().value("content").toArray();
    check(parts.size() == 3 && parts[0].toObject().value("type").toString() == "text",
          "both photos occur in the same user message");
    const auto dataUrl = parts[1].toObject().value("image_url").toObject().value("url").toString();
    check(dataUrl.startsWith("data:image/jpeg;base64,"), "local images are inline JPEG data URLs");
    QImage encoded;
    encoded.loadFromData(QByteArray::fromBase64(dataUrl.mid(dataUrl.indexOf(',') + 1).toLatin1()),
                         "JPEG");
    check(!encoded.isNull() && encoded.width() == 1600 && encoded.height() == 1000,
          "image resize preserves aspect ratio and limits the longest dimension");
    check(ot::buildVisionRequest({}, settings, &error).isEmpty() && !error.isEmpty(),
          "zero images rejected");
    QList<QImage> tooMany;
    for (int i = 0; i < 9; ++i)
        tooMany.push_back(photo);
    check(ot::buildVisionRequest(tooMany, settings, &error).isEmpty(),
          "more than eight images rejected");
    auto invalidSettings = settings;
    invalidSettings.baseUrl = "http://example.com";
    check(ot::buildVisionRequest({photo}, invalidSettings, &error).isEmpty(),
          "remote HTTP endpoint rejected");
    invalidSettings.baseUrl = "https://user:password@example.com";
    check(ot::buildVisionRequest({photo}, invalidSettings, &error).isEmpty(),
          "userinfo credentials in endpoints rejected");
    invalidSettings.baseUrl = "http://127.0.0.1:18765/v1";
    check(!ot::buildVisionRequest({photo}, invalidSettings, &error).isEmpty(),
          "localhost HTTP remains available to mock servers");
    invalidSettings = settings;
    invalidSettings.knownGateWidth = std::nan("");
    check(ot::buildVisionRequest({photo}, invalidSettings, &error).isEmpty(),
          "invalid known scale rejected");

    auto parsed = ot::parseVisionResponse(response(draft()), settings);
    check(parsed.ok && parsed.error.isEmpty() && parsed.document.gates.size() == 2,
          "structured track is converted to editable geometry");
    if (parsed.document.gates.size() == 2) {
        check(parsed.document.gates[0].yaw == 90 && parsed.document.gates[1].yaw == 0,
              "yaw0+Z and yaw90+X are preserved");
        check(parsed.document.gates[1].position == QPointF(10, 5), "X/Z are planar metres");
        check(parsed.document.gates[0].elevation == 1.2, "altitude denotes gate centre");
    }
    check(!parsed.warnings.isEmpty(), "AI geometry is always accompanied by review warnings");
    check(parsed.document.extra.value("aiVision").toObject().value("reviewRequired").toBool(),
          "draft metadata marks human review mandatory");
    check(ot::parseVisionResponse(response(draft(), true), settings).ok, "fenced JSON accepted");
    check(!ot::parseVisionResponse(response(draft(), false, "length"), settings).ok,
          "truncated output cannot be silently accepted");
    check(!ot::parseVisionResponse("not json", settings).ok, "malformed service response rejected");
    check(!ot::parseVisionResponse("{}", settings).ok, "missing choices rejected");
    auto malformed = draft();
    malformed["track"] = QJsonObject{{"gates", QJsonArray{}}};
    check(!ot::parseVisionResponse(response(malformed), settings).ok,
          "empty gate list is not fabricated into a course");
    malformed = draft();
    auto track = malformed["track"].toObject();
    auto badGate = gate();
    badGate["width"] = -2;
    track["gates"] = QJsonArray{gate(), badGate};
    malformed["track"] = track;
    const auto invalid = ot::parseVisionResponse(response(malformed), settings);
    check(!invalid.ok && invalid.document.gates.isEmpty(),
          "invalid later gate rejects the entire draft atomically");
    badGate = gate();
    badGate["type"] = "double";
    track["gates"] = QJsonArray{badGate};
    malformed["track"] = track;
    check(!ot::parseVisionResponse(response(malformed), settings).ok,
          "unsupported native prefab is rejected");
    badGate = gate();
    badGate["x"] = "not-a-number";
    track["gates"] = QJsonArray{badGate};
    malformed["track"] = track;
    check(!ot::parseVisionResponse(response(malformed), settings).ok,
          "numeric strings cannot bypass coordinate checks");
    QJsonArray huge;
    for (int i = 0; i < 257; ++i)
        huge.append(gate(i, 0));
    track["gates"] = huge;
    malformed["track"] = track;
    check(!ot::parseVisionResponse(response(malformed), settings).ok, "draft gate limit enforced");
    malformed = draft();
    malformed["error"] = "Insufficient visual evidence";
    check(!ot::parseVisionResponse(response(malformed), settings).ok,
          "model inability is an explicit error");
    malformed = draft();
    track = malformed["track"].toObject();
    track["name"] = settings.apiKey;
    badGate = gate();
    badGate["label"] = settings.apiKey;
    track["gates"] = QJsonArray{badGate};
    malformed["track"] = track;
    malformed["warnings"] = QJsonArray{settings.apiKey};
    const auto redacted = ot::parseVisionResponse(response(malformed), settings);
    check(redacted.ok && !QJsonDocument(ot::toJson(redacted.document))
                              .toJson()
                              .contains(settings.apiKey.toUtf8()),
          "provider text cannot reflect a key into a project");
    check(!redacted.warnings.join(' ').contains(settings.apiKey),
          "provider warning strings are redacted too");

    const QByteArray models =
        R"({"data":[{"id":"deepseek-flash","name":"DeepSeek-V4.1-Flash","input_modalities":["text","image"]},{"id":"deepseek-v4-pro","input_modalities":["text"]},{"id":"missing-metadata"}]})";
    check(ot::validateVisionModelResponse(models, "deepseek-flash").isEmpty(),
          "image modality verified from service metadata");
    check(!ot::validateVisionModelResponse(models, "deepseek-v4-pro").isEmpty(),
          "text-only model rejected");
    check(!ot::validateVisionModelResponse(models, "missing-metadata").isEmpty(),
          "missing modality declarations are not guessed");
    check(!ot::validateVisionModelResponse(models, "unknown-model").isEmpty(),
          "unknown model rejected");
    check(!ot::validateVisionModelResponse("invalid", "deepseek-flash").isEmpty(),
          "malformed models response rejected");
    const auto cancelled = std::make_shared<std::atomic_bool>(true);
    check(!ot::generateWithVision({photo}, settings, cancelled).ok,
          "pre-cancelled generation returns without making a request");
    check(!ot::validateVisionModel(settings, cancelled).isEmpty(),
          "pre-cancelled model validation returns without making a request");
    std::cout << (failures ? "AI vision tests FAILED\n"
                           : "AI vision tests passed (no network or credential access)\n");
    return failures ? 1 : 0;
}
