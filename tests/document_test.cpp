#include "document.h"

#include <QCoreApplication>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QTemporaryDir>

#include <cmath>
#include <iostream>
#include <limits>

namespace {
int failures = 0;
void check(bool condition, const char *description) {
    if (!condition) {
        ++failures;
        std::cerr << "FAIL: " << description << '\n';
    }
}

QJsonObject withFirstGateField(const QJsonObject &source, const QString &key,
                               const QJsonValue &value) {
    auto changed = source;
    auto gates = changed.value(QStringLiteral("gates")).toArray();
    auto first = gates.at(0).toObject();
    first.insert(key, value);
    gates[0] = first;
    changed.insert(QStringLiteral("gates"), gates);
    return changed;
}
} // namespace

int main(int argc, char **argv) {
    QCoreApplication app(argc, argv);
    QTemporaryDir temporary;
    check(temporary.isValid(), "temporary directory exists");

    auto original = ot::demoDocument();
    original.extra = QJsonObject{{QStringLiteral("source"), QStringLiteral("independent fixture")}};
    original.gates[0].extra = QJsonObject{{QStringLiteral("checkpointPass"), 2}};
    original.background = QImage(32, 24, QImage::Format_ARGB32);
    original.background.fill(qRgba(40, 120, 220, 180));
    original.metersPerPixel = 0.125;
    original.backgroundOrigin = QPointF(-12.5, 16.25);

    QString error;
    const QString project = temporary.filePath(QStringLiteral("青空.otproj"));
    check(ot::saveProject(project, original, &error), "save valid project to unicode path");
    ot::TrackDocument loaded;
    check(ot::loadProject(project, &loaded, &error), "load saved project");
    check(ot::toJson(loaded) == ot::toJson(original),
          "full model including embedded PNG and extras round trips");
    check(loaded.background.pixelColor(10, 10) == original.background.pixelColor(10, 10),
          "background pixel and alpha retained");
    check(error.isEmpty(), "successful operation clears stale error");

    const auto baseline = ot::toJson(loaded);
    auto invalid = withFirstGateField(baseline, QStringLiteral("width"), -1.0);
    check(!ot::fromJson(invalid, &loaded, &error), "reject negative width");
    check(ot::toJson(loaded) == baseline, "rejected load does not mutate destination");
    invalid = withFirstGateField(baseline, QStringLiteral("position"), QJsonArray{1e12, 0.0});
    check(!ot::fromJson(invalid, &loaded, &error), "reject coordinates outside bounds");
    invalid = withFirstGateField(baseline, QStringLiteral("yaw"), QStringLiteral("NaN"));
    check(!ot::fromJson(invalid, &loaded, &error), "reject string numeric input");
    invalid = withFirstGateField(baseline, QStringLiteral("height"), QJsonValue(QJsonValue::Null));
    check(!ot::fromJson(invalid, &loaded, &error), "reject null numeric input");
    invalid = baseline;
    auto duplicateGates = invalid.value(QStringLiteral("gates")).toArray();
    duplicateGates.append(duplicateGates.at(0));
    invalid.insert(QStringLiteral("gates"), duplicateGates);
    check(!ot::fromJson(invalid, &loaded, &error), "reject duplicate IDs");
    invalid = baseline;
    invalid.insert(QStringLiteral("backgroundPng"), QStringLiteral("%%%not-base64%%%"));
    check(!ot::fromJson(invalid, &loaded, &error), "reject malformed image encoding");
    invalid = baseline;
    invalid.insert(QStringLiteral("version"), 2);
    check(!ot::fromJson(invalid, &loaded, &error), "reject unsupported format version");
    check(!ot::fromJson(baseline, nullptr, &error), "reject null output pointer");

    auto unsupportedGate = baseline;
    unsupportedGate =
        withFirstGateField(unsupportedGate, QStringLiteral("type"), QStringLiteral("future_gate"));
    ot::TrackDocument future;
    check(ot::fromJson(unsupportedGate, &future, &error),
          "retain future gate type without data loss");
    check(future.gates[0].type == QStringLiteral("future_gate"), "unknown gate type unchanged");

    // A failed save must preserve the previously committed file.
    QFile saved(project);
    check(saved.open(QIODevice::ReadOnly), "read saved data");
    const QByteArray savedBytes = saved.readAll();
    saved.close();
    auto invalidDocument = original;
    invalidDocument.gates[0].position.setX(std::numeric_limits<double>::infinity());
    check(!ot::saveProject(project, invalidDocument, &error),
          "reject nonfinite in-memory coordinates");
    check(saved.open(QIODevice::ReadOnly), "reopen data after rejected save");
    check(saved.readAll() == savedBytes, "rejected save leaves existing file intact");
    saved.close();

    const QString malformed = temporary.filePath(QStringLiteral("broken.otproj"));
    QFile broken(malformed);
    check(broken.open(QIODevice::WriteOnly), "create malformed fixture");
    broken.write("{broken");
    broken.close();
    check(!ot::loadProject(malformed, &loaded, &error), "reject malformed JSON file");
    check(ot::toJson(loaded) == baseline, "malformed file leaves model untouched");

    ot::TrackDocument empty;
    check(ot::routePath(empty).isEmpty(), "empty route safe");
    check(ot::routeLength(empty) == 0.0, "empty route has zero length");
    ot::Gate a;
    a.id = ot::newGateId();
    a.position = QPointF(0, 0);
    ot::Gate b;
    b.id = ot::newGateId();
    b.position = QPointF(3, 4);
    empty.gates = {a, b};
    empty.closedLoop = false;
    check(std::abs(ot::routeLength(empty) - 5.0) < 1e-6,
          "open two-gate route length is Euclidean distance");
    empty.closedLoop = true;
    check(std::abs(ot::routeLength(empty) - 10.0) < 1e-6, "closed two-gate route returns to start");

    ot::Gate pillar;
    pillar.id = ot::newGateId();
    pillar.type = QStringLiteral("pillar");
    pillar.position = QPointF(10, 20);
    pillar.width = .4;
    pillar.height = 3;
    pillar.elevation = 1.5;
    check(ot::gatePassagePosition(pillar) == QPointF(11.75, 20),
          "pillar passage defaults to the local right side of the solid post");
    const double clearance =
        ot::gatePassagePosition(pillar).x() - pillar.position.x() - pillar.width / 2 - 2.4 / 2;
    check(std::abs(clearance - .35) < 1e-6, "native 2.4m checkpoint lane clears the post by 0.35m");
    pillar.extra.insert(QStringLiteral("passSide"), -1);
    check(ot::gatePassagePosition(pillar) == QPointF(8.25, 20),
          "left-side pillar passage mirrors the checkpoint lane");
    pillar.yaw = 90;
    check((ot::gatePassagePosition(pillar) - QPointF(10, 21.75)).manhattanLength() < 1e-6,
          "pillar passage rotates with the native yaw convention");
    pillar.yaw = 0;
    pillar.extra.insert(QStringLiteral("passSide"), 0);
    check(ot::gatePassagePosition(pillar) == QPointF(11.75, 20),
          "nonnegative side normalizes to the right side");
    ot::TrackDocument passageTrack;
    b.position = QPointF(11.75, 25);
    passageTrack.gates = {pillar, b};
    passageTrack.closedLoop = false;
    const auto passagePath = ot::routePath(passageTrack);
    check(passagePath.elementAt(0).x == 11.75 && passagePath.elementAt(0).y == 20,
          "route starts at the pillar passage instead of inside the solid post");
    check(std::abs(ot::routeLength(passageTrack) - 5.0) < 1e-6,
          "route length follows the offset passage");
    b.extra.insert(QStringLiteral("passSide"), -1);
    check(ot::gatePassagePosition(b) == b.position,
          "ordinary gates retain their physical passage position");

    const auto demoPath = ot::routePath(original);
    check(demoPath.elementCount() > original.gates.size(), "demo route uses cubic curves");
    check(std::isfinite(ot::routeLength(original)) && ot::routeLength(original) > 200.0,
          "demo path length finite and plausible");
    check(ot::newGateId() != ot::newGateId(), "generated IDs unique");

    if (failures == 0)
        std::cout << "All document tests passed.\n";
    return failures == 0 ? 0 : 1;
}
