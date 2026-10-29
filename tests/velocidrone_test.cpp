#include "document.h"
#include "velocidrone.h"

#include <QCoreApplication>
#include <QElapsedTimer>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QQuaternion>
#include <QSet>
#include <QTemporaryDir>
#include <cmath>
#include <iostream>
#include <limits>

namespace {
int failures = 0;
void require(bool condition, const char *message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        ++failures;
    }
}
bool near(double a, double b, double tolerance = .015) {
    return std::abs(a - b) < tolerance;
}
QByteArray readFile(const QString &path) {
    QFile file(path);
    return file.open(QIODevice::ReadOnly) ? file.readAll() : QByteArray{};
}
bool writeFile(const QString &path, const QByteArray &data) {
    QFile file(path);
    return file.open(QIODevice::WriteOnly) && file.write(data) == data.size();
}
QJsonObject nativeRoot(const ot::TrackDocument &doc) {
    return doc.extra.value(QStringLiteral("velocidroneRoot")).toObject();
}
QJsonArray nativeGates(const ot::TrackDocument &doc) {
    return nativeRoot(doc).value(QStringLiteral("gates")).toArray();
}
QJsonArray nativeBars(const ot::TrackDocument &doc) {
    return nativeRoot(doc).value(QStringLiteral("barriers")).toArray();
}
int countPrefab(const QJsonArray &objects, int prefab) {
    int count = 0;
    for (const auto &object : objects)
        count += object.toObject().value(QStringLiteral("prefab")).toInt() == prefab;
    return count;
}
ot::TrackDocument course() {
    ot::TrackDocument doc;
    doc.name = QStringLiteral("Independent synthetic course");
    doc.scenery = QStringLiteral("Empty Scene Day");
    doc.takeoff.enabled = true;
    doc.takeoff.position = {4.25, -6.75};
    doc.takeoff.elevation = -.86;
    doc.takeoff.yaw = 45;
    return doc;
}
ot::RouteStep pass(const ot::Gate &gate, const QString &face = QStringLiteral("front"),
                   int column = 0, int level = 0, bool reverse = false) {
    ot::RouteStep step;
    step.id = ot::newGateId();
    step.gateId = gate.id;
    step.face = face;
    step.column = column;
    step.level = level;
    step.reverse = reverse;
    return step;
}
QVector<ot::RouteStep> passes(const ot::TrackDocument &doc) {
    QVector<ot::RouteStep> result;
    for (const auto &step : ot::effectiveRouteSteps(doc))
        if (step.kind == QStringLiteral("pass"))
            result.append(step);
    return result;
}
QVector<ot::RouteStep> checkpointSteps(const ot::TrackDocument &doc) {
    QVector<ot::RouteStep> result;
    for (const auto &step : ot::effectiveRouteSteps(doc))
        if (step.kind == QStringLiteral("pass") ||
            (step.kind == QStringLiteral("guide") &&
             step.extra.value(QStringLiteral("raceCheckpoint")).toBool()))
            result.append(step);
    return result;
}
void requireNativeCheckpoints(const ot::TrackDocument &source, const ot::TrackDocument &native) {
    const auto steps = checkpointSteps(source);
    const auto records = nativeGates(native);
    require(steps.size() == records.size(),
            "only actual passes and explicitly marked turn controls become native checkpoints");
    int starts = 0, finishes = 0;
    for (qsizetype i = 0; i < std::min(steps.size(), records.size()); ++i) {
        const auto record = records[i].toObject();
        const auto trans = record.value(QStringLiteral("trans")).toObject();
        const auto p = trans.value(QStringLiteral("pos")).toArray();
        const auto r = trans.value(QStringLiteral("rot")).toArray();
        const auto scale = trans.value(QStringLiteral("scale")).toArray();
        require(record.value(QStringLiteral("prefab")).toInt() == 88 &&
                    record.value(QStringLiteral("gate")).toInt(-1) == i,
                "new native checks retain strict interleaved order without branch-number guesses");
        if (p.size() != 3 || r.size() != 4 || scale.size() != 3) {
            require(false, "native checkpoint transform arrays have expected arity");
            continue;
        }
        const QQuaternion q = QQuaternion(static_cast<float>(r[0].toDouble() / 1000),
                                          static_cast<float>(r[1].toDouble() / 1000),
                                          static_cast<float>(r[2].toDouble() / 1000),
                                          static_cast<float>(r[3].toDouble() / 1000))
                                  .normalized();
        const auto actualUp = q.rotatedVector({0, 0, 1});
        const auto actualNormal = q.rotatedVector({-1, 0, 0});
        const auto actualRight = q.rotatedVector({0, -1, 0});
        const auto actualCentre =
            QVector3D(static_cast<float>(p[0].toDouble() / 100),
                      static_cast<float>(p[1].toDouble() / 100),
                      static_cast<float>(p[2].toDouble() / 100)) +
            actualUp * static_cast<float>(.9900000095 * scale[2].toDouble() / 100);
        const auto expected = ot::routeStepGeometry(source, steps[i]);
        require(expected.valid && (actualCentre - expected.position).length() < .015f,
                "independently decoded native centre respects centimetre/rotation quantization");
        require(QVector3D::dotProduct(actualNormal, expected.normal) > .99999f &&
                    QVector3D::dotProduct(actualRight, expected.right) > .99999f &&
                    QVector3D::dotProduct(actualUp, expected.up) > .99999f,
                "runtime accepted minus-X direction and full window basis match route geometry");
        require(near(1.9286329746 * scale[1].toDouble() / 100, expected.openingWidth, .011) &&
                    near(1.9286329746 * scale[2].toDouble() / 100, expected.openingHeight, .011),
                "physical88 window units quantize both ordinary and turn-check apertures");
        for (const auto &value : p)
            require(value.toDouble() == std::round(value.toDouble()),
                    "native root positions are integer centimetres");
        starts += record.value(QStringLiteral("start")).toBool();
        finishes += record.value(QStringLiteral("finish")).toBool();
    }
    require(starts == 1 && finishes == 1,
            "one lap start/finish checkpoint independent of physical group count and takeoff");
}
bool roundtrip(const QString &path, const ot::TrackDocument &source, ot::TrackDocument *destination,
               const char *message) {
    QString error;
    const bool ok = ot::exportVelocidrone(path, source, &error) &&
                    ot::importVelocidrone(path, destination, &error);
    require(ok, message);
    if (!ok)
        std::cerr << error.toStdString() << '\n';
    return ok;
}
void requireRouteGeometry(const ot::TrackDocument &before, const ot::TrackDocument &after) {
    const auto a = passes(before), b = passes(after);
    require(a.size() == b.size(),
            "all ordered traversals retained; guides are not native checkpoints");
    for (qsizetype i = 0; i < std::min(a.size(), b.size()); ++i) {
        const auto x = ot::routeStepGeometry(before, a[i]), y = ot::routeStepGeometry(after, b[i]);
        require(x.valid && y.valid && (x.position - y.position).length() < .04f,
                "traversal world centre retained independently of physical centre");
        require(
            QVector3D::dotProduct(x.normal, y.normal) > .99998f &&
                QVector3D::dotProduct(x.right, y.right) > .99998f &&
                QVector3D::dotProduct(x.up, y.up) > .99998f,
            "traversal plane basis and direction retained, including horizontal gravity portal");
    }
}
// Authored synthetic data, encrypted independently using .NET AES/ECB/PKCS7.
// It has an unrelated barrier/weather field, legacy285, a separate422grid and
// nonzero trailer1/29314. No simulator asset or private track is embedded.
const QByteArray Golden =
    "e73GF0q1zUigczG3hm/zQA+O8GS0uwOtT/CYYDVuWYsC2p5td3187JLqj4F0+Rfl6nipuA6VylE0eRMeLc4K+7PXiYlR"
    "SkDThcMIlE1XggU5dtzbZpgfTL/31rOoX0t/Xyo8H7Wn+yerHil3NH9ewpXlgARWyqoGR80VLpJmEHP0XCXtOwHXneLR"
    "bDs4pRI5Fm9XgEKgjKdkkXpX+iCZLvBA0XPP+1DLd4yKzeOFR/uPyCdSzU9dYikw0XSkGE+NSciqyhzZ81lsRrvBkfzZ"
    "N9XEKqHlIVgMMUjyxHsAuUPiJY09XZP8ligb1KChYpDYjmkFzA1ZhFPjex/nyHcM46z34AKoLkK6JBQ5gzlcrzRk1MJo"
    "OptwE9J+BZ1qowPVnV4QghCsMTU8YwhZEpg9DYzTtqyhIkxsvVm4/pmbS3BWSQXQJJ1geUzAj5Juvqu/pI9CrmF2v2nC"
    "3iJxNgcfd78MbK0QjPo7I+91v+g5qOK81hCADSpnuSdOdiV1d5ZPUzYlLTIV5C4bd1odN7HXJiqv4s9gJtFICG4qlPpq"
    "7o8=";
const QByteArray Unordered90 =
    "EseV2u1rhzBeF26sBRHLgRcvYrhl6kcABsbbX99CE4UtPylGe1Gk66C01wlbBGfpjy3Iiwd9Zehae/UezxVtoidI5T7J"
    "V1/w0nFRQW44dp0moXIWCN6jR6ZDRHktEUtCg5EYEzfuHw6yZwDcWVVzdPnoPoJMcDWAe0Gs6B43JWUYcW+tNX7sEyx2"
    "fLjqHInwjRxJggE7ytQOXFcZ89Z2LpJmVnReuU0o7ZgQ2sVWrcgc/W1TklGFPlSLj39+iIyUXHOnmsqtikWdaBDfaD5z"
    "jb8MbK0QjPo7I+91v+g5qOJYL3JmXy/onnUVEwDfkr00AoO3H5IUulGOOpkKyG6EWEItlj5gf6LJmzNDpe4igft6zW3r"
    "lNlZ0pwWCNU/oFCr/gOt8GRnluGwvByhc1LUOOM8G5Alnbt5pRnUuDBSHQFGH0YhhsKNlyhNrJDCa0+kMRFFkry3/qRA"
    "49lOc/SBYmNDfYANJ1e4/QGPJnMIW12OaQXMDVmEU+N7H+fIdwzjrPfgAqguQrokFDmDOVyvNGTUwmg6m3AT0n4FnWqj"
    "A9WdXhCCEKwxNTxjCFkSmD0NjNO2rKEiTGy9Wbj+mZtLcFZJBdAknWB5TMCPkm6+q7+kj0KuYXa/acLeInE2Bx93vwxs"
    "rRCM+jsj73W/6Dmo4jqfcJy2WtFHk6JFieAlEQFU+CeL2NQtE1LabumMdeRbHlKBvaT2B1AhI/+mATd/qZ+v/XvhkNA1"
    "P1rc8WQ8FarOf7pS5oD9LebiCXn2OcFxv22qtPgU5AF0P3dkGIhGbQ==";
} // namespace

int main(int argc, char **argv) {
    QCoreApplication application(argc, argv);
#ifndef Q_OS_WIN
    std::cout << "SKIP: Windows CNG native codec.\n";
    return 0;
#else
    // Optional private integration mode, with caller-provided files only.
    // Reports counts, not user names, account information or raw track data.
    if (argc == 4 && QByteArray(argv[1]) == "--inspect") {
        ot::TrackDocument doc, again;
        QString error;
        QElapsedTimer timer;
        timer.start();
        if (!ot::importVelocidrone(QString::fromLocal8Bit(argv[2]), &doc, &error) ||
            !ot::exportVelocidrone(QString::fromLocal8Bit(argv[3]), doc, &error) ||
            !ot::importVelocidrone(QString::fromLocal8Bit(argv[3]), &again, &error)) {
            std::cerr << error.toStdString() << '\n';
            return 1;
        }
        const bool rootPreserved = nativeRoot(doc) == nativeRoot(again);
        std::cout << "physical=" << doc.gates.size() << " passes=" << passes(doc).size()
                  << " native=" << nativeGates(doc).size() << " barriers=" << nativeBars(doc).size()
                  << " takeoff=" << doc.takeoff.enabled << " rawRootPreserved=" << rootPreserved
                  << " milliseconds=" << timer.elapsed() << '\n';
        return rootPreserved ? 0 : 2;
    }
    QTemporaryDir directory;
    require(directory.isValid(), "temporary directory");
    if (!directory.isValid())
        return 1;
    const auto path = directory.filePath(QStringLiteral("course.trk"));
    QString error;
    require(writeFile(path, Golden), "write independent known-answer ciphertext");
    ot::TrackDocument legacy;
    require(ot::importVelocidrone(path, &legacy, &error), "decrypt independent.NET fixture");
    if (legacy.gates.isEmpty()) {
        std::cerr << error.toStdString();
        return 1;
    }
    require(legacy.name == QStringLiteral("Fixture {中文}") &&
                legacy.scenery == QStringLiteral("Empty Scene Day"),
            "UTF8 framing, braces and scene mapping");
    require(legacy.takeoff.enabled && near(legacy.takeoff.position.x(), 1) &&
                near(legacy.takeoff.position.y(), -6) && near(legacy.takeoff.elevation, -.86) &&
                near(legacy.takeoff.yaw, 90, .1),
            "native422 base and independent heading imported without scene offsets");
    require(legacy.gates.size() == 1 && legacy.routeStepsDefined && legacy.routeSteps.size() == 1,
            "legacy physical object and ordered traversal represented separately");
    require(legacy.gates[0].type == QStringLiteral("start") &&
                near(legacy.gates[0].position.x(), -12.34) &&
                near(legacy.gates[0].position.y(), 56.78) && near(legacy.gates[0].elevation, 1.2),
            "legacy285 centimetres/base origin retained");
    require(
        ot::exportVelocidrone(path, legacy, &error) && readFile(path) == Golden,
        "independent ciphertext byte-for-byte roundtrip includes native grid and nonzero trailer");
    legacy.gates[0].position += QPointF(2, -3);
    legacy.gates[0].height = 4.8;
    legacy.gates[0].elevation = 4.4;
    legacy.gates[0].yaw = 0;
    ot::TrackDocument edited;
    if (roundtrip(path, legacy, &edited, "legacy native pose/height edit")) {
        require(near(edited.gates[0].position.x(), -10.34) &&
                    near(edited.gates[0].position.y(), 53.78) &&
                    near(edited.gates[0].height, 4.8) && near(edited.gates[0].elevation, 4.4),
                "legacy relative edit preserves centre altitude and units");
        require(nativeBars(edited) == nativeBars(legacy),
                "unrelated barriers and native grid preserved");
        require(nativeRoot(edited).value(QStringLiteral("weather")) ==
                    nativeRoot(legacy).value(QStringLiteral("weather")),
                "unknown root/weather preserved");
    }
    auto repeatedLegacy = legacy;
    repeatedLegacy.routeSteps.append(pass(repeatedLegacy.gates[0]));
    require(!ot::exportVelocidrone(path, repeatedLegacy, &error),
            "uncalibrated legacy model is not silently duplicated for multiple traversals");

    require(writeFile(path, Unordered90), "write independent unordered native90 fixture");
    ot::TrackDocument grid90;
    require(ot::importVelocidrone(path, &grid90, &error) && grid90.gates.size() == 2 &&
                grid90.takeoff.enabled,
            "import native90 and native array order distinct from numeric traversal order");
    require(near(grid90.takeoff.yaw, -90, .1),
            "native90 race child-slot rotation included in aircraft heading");
    require(
        ot::exportVelocidrone(path, grid90, &error) && readFile(path) == Unordered90,
        "native array order, native90 quaternion, unknown metadata and flags preserved exactly");
    grid90.takeoff.position += QPointF(3, 2);
    ot::TrackDocument grid90Edited;
    if (roundtrip(path, grid90, &grid90Edited, "editing old90 grid converts to calibrated422"))
        require(countPrefab(nativeBars(grid90Edited), 90) == 0 &&
                    countPrefab(nativeBars(grid90Edited), 422) == 1 &&
                    near(grid90Edited.takeoff.yaw, -90, .1),
                "replacement422 retains actual aircraft heading, not old root+Z");

    auto simple = course();
    auto frame = ot::makeGate(QStringLiteral("standard"));
    frame.position = {1.23, 4.56};
    ot::setGateColor(frame, QColor("#e50001"));
    auto pillar = ot::makeGate(QStringLiteral("pillar"));
    pillar.position = {8, 9};
    pillar.yaw = 30;
    pillar.extra.insert(QStringLiteral("passSide"), -1);
    ot::setGateColor(pillar, QColor("#00cc00"));
    simple.gates = {frame, pillar};
    ot::TrackDocument restored;
    if (roundtrip(path, simple, &restored, "calibrated coloured frame/pillar export")) {
        require(restored.gates.size() == 2, "physical frame and pillar inferred once");
        require(nativeBars(restored).size() == 6 && countPrefab(nativeBars(restored), 422) == 1,
                "four stock bars, one pillar, exactly one independent grid");
        require(countPrefab(nativeBars(restored), 2220) == 4 &&
                    countPrefab(nativeBars(restored), 2223) == 1,
                "per-object arbitrary RGB maps to nearest verified native solid materials");
        const auto cp = nativeGates(restored)[0].toObject();
        const auto transform = cp.value(QStringLiteral("trans")).toObject();
        require(transform.value(QStringLiteral("rot")).toArray() == QJsonArray{500, -500, 500, 500},
                "actual CheckPoint reset-origin acceptance maps native minus-X to forward");
        require(transform.value(QStringLiteral("scale")).toArray() == QJsonArray{100, 150, 150},
                "measured1.9286329746m trigger window requires150percent for2.9m opening");
        const auto pos = transform.value(QStringLiteral("pos")).toArray();
        require(pos == QJsonArray{123, 12, 456},
                "trigger origin compensates native localZ+.99, centimetre rounding");
        int vertical = 0, horizontal = 0;
        for (const auto &value : nativeBars(restored)) {
            const auto bar = value.toObject();
            if (bar.value(QStringLiteral("prefab")).toInt() != 2220)
                continue;
            const auto scale = bar.value(QStringLiteral("trans"))
                                   .toObject()
                                   .value(QStringLiteral("scale"))
                                   .toArray();
            vertical += scale == QJsonArray{10, 320, 10};
            horizontal += scale == QJsonArray{320, 10, 10};
        }
        require(vertical == 2 && horizontal == 2,
                "physical collider-calibrated beams exactly3.2m with.1square section");
        requireRouteGeometry(simple, restored);
        const auto ciphertext = readFile(path);
        require(ot::exportVelocidrone(path, restored, &error) && readFile(path) == ciphertext,
                "new calibrated frame/pillar/grid byte-exact unchanged roundtrip");
        require(near(restored.takeoff.position.x(), simple.takeoff.position.x()) &&
                    near(restored.takeoff.position.y(), simple.takeoff.position.y()) &&
                    near(restored.takeoff.elevation, -.86) && near(restored.takeoff.yaw, 45, .1),
                "takeoff is independent of gate positions and route starting direction");
    }

    auto postPasses = simple;
    postPasses.routeStepsDefined = true;
    postPasses.routeSteps = {pass(frame), pass(pillar), pass(pillar, QStringLiteral("back")),
                             pass(pillar)};
    ot::RouteStep freeGuide;
    freeGuide.id = ot::newGateId();
    freeGuide.kind = QStringLiteral("guide");
    freeGuide.localGuide = {10, 5, 20};
    postPasses.routeSteps.insert(2, freeGuide);
    ot::TrackDocument postOutput;
    if (roundtrip(path, postPasses, &postOutput,
                  "repeated pillar-side traversals and standalone guide")) {
        require(postOutput.gates.size() == 2 && nativeBars(postOutput).size() == 6 &&
                    nativeGates(postOutput).size() == 4,
                "pillar solid appears once across three ordered lane traversals");
        requireRouteGeometry(postPasses, postOutput);
    }
    auto unusedPhysical = simple;
    unusedPhysical.routeStepsDefined = true;
    unusedPhysical.routeSteps = {pass(frame)};
    ot::TrackDocument unusedOutput;
    if (roundtrip(path, unusedPhysical, &unusedOutput, "physical obstacle not in ordered route"))
        require(nativeBars(unusedOutput).size() == 6 && nativeGates(unusedOutput).size() == 1,
                "unreferenced physical obstacle still exported, no implicit traversal invented");

    require(ot::nativeBlockColors().size() == 13, "thirteen verified native solid-block colours");
    for (const auto &entry : ot::nativeBlockColors())
        require(ot::nearestNativeBlockColor(entry.color).prefab == entry.prefab,
                "native palette maps exact material");

    const QList<QPair<QString, int>> kinds = {
        {"standard", 4}, {"double", 7},      {"triple", 10}, {"ladder", 7},
        {"cube", 12},    {"cube-stack", 20}, {"quad", 12},   {"ladder-blocked", 8}};
    for (const auto &kind : kinds) {
        auto doc = course();
        auto group = ot::makeGate(kind.first);
        group.position = {12, -8};
        group.yaw = 37;
        ot::setGateColor(group, QColor("#8c00e6"));
        doc.gates = {group};
        doc.routeStepsDefined = true;
        const auto metrics = ot::gateFrameMetrics(group);
        for (int column = 0; column < metrics.columns; ++column)
            for (int level = 0; level < metrics.levels; ++level) {
                if (!ot::gateCellOpen(group, column, level))
                    continue;
                doc.routeSteps.append(pass(group, QStringLiteral("front"), column, level));
                ot::RouteStep guide;
                guide.id = ot::newGateId();
                guide.gateId = group.id;
                guide.kind = QStringLiteral("guide");
                guide.localGuide = {0, 0, .8f};
                doc.routeSteps.append(guide);
                doc.routeSteps.append(pass(group, QStringLiteral("back"), column, level));
            }
        if (metrics.centerDepth > 0)
            for (const auto &face : QStringList{"left", "right", "top", "bottom"})
                for (int level = 0; level < metrics.levels; ++level) {
                    doc.routeSteps.append(pass(group, face, 0, level));
                    doc.routeSteps.append(pass(group, face, 0, level, true));
                }
        ot::TrackDocument output;
        if (!roundtrip(path, doc, &output, "compound repeated/all-face traversal export"))
            continue;
        require(output.gates.size() == 1, "multiple passes reference one physical assembly");
        if (output.gates.size() != 1) {
            std::cerr << "shape=" << kind.first.toStdString() << " physical=" << output.gates.size()
                      << '\n';
            continue;
        }
        require((output.gates[0].type == kind.first ||
                 (kind.first == QStringLiteral("standard") &&
                  output.gates[0].type == QStringLiteral("start"))) &&
                    near(output.gates[0].width, 3) && near(output.gates[0].height, 3),
                "compound physical type and per-cell dimensions retained");
        require(nativeBars(output).size() == kind.second + 1 &&
                    countPrefab(nativeBars(output), 2226) == kind.second,
                "stock count independent of repeated passes and guide count");
        requireRouteGeometry(doc, output);
        const auto cipher = readFile(path);
        require(ot::exportVelocidrone(path, output, &error) && readFile(path) == cipher,
                "all-face repeated-pass raw assembly roundtrip does not duplicate bars");
        for (const auto &value : nativeGates(output))
            require(value.toObject().value(QStringLiteral("prefab")).toInt() == 88,
                    "one separate invisible checkpoint for every physical traversal");
        auto moved = output;
        moved.gates[0].position += QPointF(3, -2);
        moved.gates[0].yaw += 22;
        ot::setGateColor(moved.gates[0], QColor("#e67300"));
        moved.takeoff.position = {-9, 12};
        moved.takeoff.yaw = -30;
        moved.takeoff.elevation = 0;
        ot::TrackDocument movedOutput;
        if (roundtrip(
                path, moved, &movedOutput,
                "move/recolour physical group with multiple traversals and independent takeoff")) {
            require(movedOutput.gates.size() == 1 &&
                        nativeBars(movedOutput).size() == kind.second + 1,
                    "editing group updates shared solids once");
            requireRouteGeometry(moved, movedOutput);
            require(near(movedOutput.takeoff.position.x(), -9) &&
                        near(movedOutput.takeoff.position.y(), 12) &&
                        near(movedOutput.takeoff.yaw, -30, .1),
                    "editing takeoff replaces rather than duplicates grid");
        }
    }
    auto invalidBlocked = course();
    invalidBlocked.gates = {ot::makeGate(QStringLiteral("ladder-blocked"))};
    invalidBlocked.routeStepsDefined = true;
    invalidBlocked.routeSteps = {pass(invalidBlocked.gates[0], QStringLiteral("front"), 0, 0)};
    const auto beforeBlockedExport = readFile(path);
    require(!ot::exportVelocidrone(path, invalidBlocked, &error) &&
                readFile(path) == beforeBlockedExport,
            "native export refuses a filled lower window and preserves the existing file");
    // Stacked/side-by-side manoeuvres change order and direction, not physical objects.
    auto patterns = course();
    auto triple = ot::makeGate(QStringLiteral("triple"));
    auto ladder = ot::makeGate(QStringLiteral("ladder"));
    ladder.position = {20, 0};
    auto cube = ot::makeGate(QStringLiteral("cube-stack"));
    cube.position = {40, 0};
    patterns.gates = {triple, ladder, cube};
    patterns.routeStepsDefined = true;
    patterns.routeSteps = ot::gateManeuverSteps(triple, QStringLiteral("left-right"));
    patterns.routeSteps += ot::gateManeuverSteps(ladder, QStringLiteral("lower-upper"));
    patterns.routeSteps += ot::gateManeuverSteps(cube, QStringLiteral("gravity-dive"));
    ot::TrackDocument patternOutput;
    if (roundtrip(path, patterns, &patternOutput, "actual maneuver patterns native checkpoints")) {
        require(patternOutput.gates.size() == 3 && nativeBars(patternOutput).size() == 38,
                "triple+ladder+cube-stack geometry unique across all manoeuvre passes");
        requireRouteGeometry(patterns, patternOutput);
    }
    // Full circles require separate, visible turn controls between the repeated
    // opening visits. Ordinary curve samples never become race constraints.
    for (const auto &direction :
         {QStringLiteral("orbit-bar-cw"), QStringLiteral("orbit-bar-ccw")}) {
        auto orbit = course();
        orbit.closedLoop = false;
        orbit.takeoff.position = {-9.73, 8.24};
        orbit.takeoff.elevation = .23;
        orbit.takeoff.yaw = -37.41;
        auto topBar = ot::makeGate(QStringLiteral("standard"));
        topBar.position = {2.137, -4.269};
        topBar.yaw = 37.17;
        auto middleBar = ot::makeGate(QStringLiteral("ladder"));
        middleBar.position = {22.314, 6.279};
        middleBar.yaw = -58.17;
        auto gravity = ot::makeGate(QStringLiteral("cube-stack"));
        gravity.position = {-16.871, 24.243};
        gravity.yaw = 127.13;
        ot::setGateColor(topBar, QColor("#e50001"));
        ot::setGateColor(middleBar, QColor("#00d9d9"));
        ot::setGateColor(gravity, QColor("#8c00e6"));
        orbit.gates = {topBar, middleBar, gravity};
        orbit.routeStepsDefined = true;
        const auto singleCircle = ot::gateManeuverSteps(topBar, direction);
        const auto ladderCircle = ot::gateManeuverSteps(middleBar, direction);
        orbit.routeSteps = singleCircle + ladderCircle;
        orbit.routeSteps += ot::gateManeuverSteps(gravity, direction.endsWith(QStringLiteral("ccw"))
                                                               ? QStringLiteral("gravity-climb")
                                                               : QStringLiteral("gravity-dive"));
        auto singleOnly = orbit;
        singleOnly.routeSteps = singleCircle;
        auto ladderOnly = orbit;
        ladderOnly.routeSteps = ladderCircle;
        require(passes(singleOnly).size() == 2 && checkpointSteps(singleOnly).size() == 5,
                "single-frame360 has two door visits plus three explicit turn checks");
        require(passes(ladderOnly).size() == 3 && checkpointSteps(ladderOnly).size() == 5,
                "ladder360 has lower-upper-lower visits plus two explicit turn checks");
        const auto visits = passes(ladderOnly);
        if (visits.size() == 3) {
            const auto rotation =
                QQuaternion::fromAxisAndAngle(0, 1, 0, static_cast<float>(middleBar.yaw));
            const float firstSign = direction.endsWith(QStringLiteral("ccw")) ? 1.f : -1.f;
            require(visits[0].level == 0 && visits[1].level == 1 && visits[2].level == 0,
                    "ladder full revolution retains lower-upper-lower cell order");
            for (int i = 0; i < 3; ++i) {
                const auto normal = ot::routeStepGeometry(ladderOnly, visits[i]).normal;
                const auto expected =
                    rotation.rotatedVector({0, 0, i == 1 ? -firstSign : firstSign});
                require(QVector3D::dotProduct(normal, expected) > .99999f,
                        "CW/CCW changes individual traversals, not physical frame yaw");
            }
        }
        ot::TrackDocument projectCopy;
        require(ot::fromJson(ot::toJson(orbit), &projectCopy, &error) &&
                    projectCopy.routeSteps.size() == orbit.routeSteps.size() &&
                    checkpointSteps(projectCopy).size() == 13,
                "project retains arc samples and explicitly marked native turn controls");
        ot::TrackDocument orbitOutput;
        if (!roundtrip(path, orbit, &orbitOutput,
                       "rotated multicolor360 plus gravity native export"))
            continue;
        require(passes(orbit).size() == 8 && nativeGates(orbitOutput).size() == 13,
                "eight door visits and five explicit turn checks interleave without hidden "
                "constraints");
        require(orbitOutput.gates.size() == 3 && nativeBars(orbitOutput).size() == 32 &&
                    countPrefab(nativeBars(orbitOutput), 2220) == 4 &&
                    countPrefab(nativeBars(orbitOutput), 2224) == 7 &&
                    countPrefab(nativeBars(orbitOutput), 2226) == 20 &&
                    countPrefab(nativeBars(orbitOutput), 422) == 1,
                "three colored physical assemblies emitted once plus one independent start grid");
        requireNativeCheckpoints(orbit, orbitOutput);
        requireRouteGeometry(orbit, orbitOutput);
        require(checkpointSteps(orbitOutput).size() == 13 && passes(orbitOutput).size() == 8,
                "native free turn checks do not masquerade as fabricated physical frames");
        for (const auto &step : orbitOutput.routeSteps)
            if (step.kind == QStringLiteral("guide"))
                require(step.gateId.isEmpty() &&
                            step.extra.value(QStringLiteral("raceCheckpoint")).toBool(),
                        "unpaired native88 restores a free explicitly marked route checkpoint");
        require(near(orbitOutput.takeoff.position.x(), -9.73) &&
                    near(orbitOutput.takeoff.position.y(), 8.24) &&
                    near(orbitOutput.takeoff.elevation, .23) &&
                    near(orbitOutput.takeoff.yaw, -37.41, .1),
                "orbit and gravity control geometry does not relocate independent takeoff");
        const auto orbitCipher = readFile(path);
        require(ot::exportVelocidrone(path, orbitOutput, &error) && readFile(path) == orbitCipher,
                "interleaved physical/free controls preserve exact native body on roundtrip");
        auto ordinaryGuideEdit = orbit;
        int changedSamples = 0;
        for (auto &step : ordinaryGuideEdit.routeSteps)
            if (step.kind == QStringLiteral("guide") &&
                !step.extra.value(QStringLiteral("raceCheckpoint")).toBool()) {
                step.localGuide += QVector3D(.031f, 0, .027f);
                ++changedSamples;
            }
        require(changedSamples > 0 && ot::exportVelocidrone(path, ordinaryGuideEdit, &error) &&
                    readFile(path) == orbitCipher,
                "editing project-only arc samples never creates or shifts native checkpoints");
        auto explicitControlEdit = orbit;
        for (auto &step : explicitControlEdit.routeSteps)
            if (step.kind == QStringLiteral("guide") &&
                step.extra.value(QStringLiteral("raceCheckpoint")).toBool()) {
                step.localGuide += QVector3D(0, .03f, 0);
                break;
            }
        require(ot::exportVelocidrone(path, explicitControlEdit, &error) &&
                    readFile(path) != orbitCipher,
                "editing an explicit turn control changes the exported race constraint");
    }
    // Free invisible native controls may have no corresponding logical solid.
    auto freeChecks = course();
    freeChecks.gates.clear();
    freeChecks.routeStepsDefined = true;
    ot::RouteStep freeControl;
    freeControl.id = ot::newGateId();
    freeControl.kind = QStringLiteral("guide");
    freeControl.localGuide = {7.123f, 9.456f, -4.789f};
    freeControl.extra = {{QStringLiteral("raceCheckpoint"), true},
                         {QStringLiteral("curveTangent"), QJsonArray{.31, .70, -.66}},
                         {QStringLiteral("checkpointUp"), QJsonArray{.1, 1, .2}},
                         {QStringLiteral("checkpointWidth"), 1.1},
                         {QStringLiteral("checkpointHeight"), .7}};
    freeChecks.routeSteps = {freeControl};
    ot::TrackDocument freeOutput;
    if (roundtrip(path, freeChecks, &freeOutput,
                  "all-free native control track without fabricated solids")) {
        require(freeOutput.gates.isEmpty() && freeOutput.routeSteps.size() == 1 &&
                    nativeBars(freeOutput).size() == 1,
                "free native88 remains a route control; only takeoff is a physical barrier");
        requireNativeCheckpoints(freeChecks, freeOutput);
        const auto sourceGeometry = ot::routeStepGeometry(freeChecks, freeControl);
        const auto importedGeometry = ot::routeStepGeometry(freeOutput, freeOutput.routeSteps[0]);
        require(QVector3D::dotProduct(sourceGeometry.up, importedGeometry.up) > .99999f &&
                    QVector3D::dotProduct(sourceGeometry.normal, importedGeometry.normal) > .99999f,
                "native free non-square window preserves roll via explicit upward axis");
        const auto freeCipher = readFile(path);
        require(
            ot::exportVelocidrone(path, freeOutput, &error) && readFile(path) == freeCipher,
            "free native control preserves exact quaternion and body without duplicate barriers");
    }
    const auto validControlFile = readFile(path);
    const auto rejectControlField = [&](const QString &key, const QJsonValue &value) {
        auto invalidControl = freeChecks;
        invalidControl.routeSteps[0].extra.insert(key, value);
        require(!ot::exportVelocidrone(path, invalidControl, &error) && !error.isEmpty() &&
                    readFile(path) == validControlFile,
                "invalid explicit control metadata fails atomically before changing native file");
    };
    rejectControlField(QStringLiteral("raceCheckpoint"), QStringLiteral("true"));
    rejectControlField(QStringLiteral("curveTangent"), QJsonArray{0, 0, 0});
    rejectControlField(QStringLiteral("curveTangent"), QJsonArray{1, 2});
    rejectControlField(QStringLiteral("curveTangent"), QJsonArray{1, "invalid", 0});
    rejectControlField(QStringLiteral("checkpointWidth"), -.8);
    rejectControlField(QStringLiteral("checkpointHeight"), QStringLiteral("0.8"));
    rejectControlField(QStringLiteral("checkpointUp"), QJsonArray{.31, .70, -.66});
    {
        auto missingTangent = freeChecks;
        missingTangent.routeSteps[0].extra.remove(QStringLiteral("curveTangent"));
        require(!ot::exportVelocidrone(path, missingTangent, &error) &&
                    readFile(path) == validControlFile,
                "no automatic tangent is invented for a marked native turn control");
    }
    // Older projects carried a +X-normal checkpoint baseline. They must not
    // reuse that raw rotation when their editor intent is forward+Z.
    auto oldProject = course();
    const auto oldFrame = ot::makeGate(QStringLiteral("start"));
    oldProject.gates = {oldFrame};
    oldProject.routeStepsDefined = true;
    oldProject.routeSteps = {pass(oldFrame)};
    oldProject.routeSteps[0].extra.insert(
        QStringLiteral("velocidroneObject"),
        QJsonObject{{QStringLiteral("prefab"), 88},
                    {QStringLiteral("gate"), 0},
                    {QStringLiteral("start"), true},
                    {QStringLiteral("finish"), true},
                    {QStringLiteral("trans"),
                     QJsonObject{{QStringLiteral("pos"), QJsonArray{0, 12, 0}},
                                 {QStringLiteral("rot"), QJsonArray{500, -500, -500, -500}},
                                 {QStringLiteral("scale"), QJsonArray{100, 150, 150}}}}});
    oldProject.routeSteps[0].extra.insert(
        QStringLiteral("velocidroneStepGeometry"),
        QJsonObject{{QStringLiteral("position"), QJsonArray{0, 1.6, 0}},
                    {QStringLiteral("normal"), QJsonArray{0, 0, 1}},
                    {QStringLiteral("right"), QJsonArray{1, 0, 0}},
                    {QStringLiteral("up"), QJsonArray{0, 1, 0}},
                    {QStringLiteral("width"), 2.9},
                    {QStringLiteral("height"), 2.9}});
    ot::TrackDocument migratedDirection;
    if (roundtrip(path, oldProject, &migratedDirection,
                  "old project native direction baseline migration"))
        requireNativeCheckpoints(oldProject, migratedDirection);
    auto nativeReverse = course();
    nativeReverse.gates = {oldFrame};
    nativeReverse.routeStepsDefined = true;
    nativeReverse.routeSteps = {pass(oldFrame, QStringLiteral("front"), 0, 0, true)};
    ot::TrackDocument preservedReverse;
    if (roundtrip(path, nativeReverse, &preservedReverse,
                  "native raw accepted direction preserved on import")) {
        requireNativeCheckpoints(nativeReverse, preservedReverse);
        const auto reverseCipher = readFile(path);
        require(ot::exportVelocidrone(path, preservedReverse, &error) &&
                    readFile(path) == reverseCipher,
                "actual native import retains original direction instead of silently repairing "
                "user intent");
    }
    // Progressive upright wraps finish in another aperture/level. Off-centre
    // flight traces affect preview geometry, not the native physical window.
    for (const auto &pattern :
         {QStringLiteral("upright-left-up"), QStringLiteral("upright-left-down"),
          QStringLiteral("upright-right-up"), QStringLiteral("upright-right-down")}) {
        auto progressive = course();
        progressive.closedLoop = false;
        auto progressiveLadder = ot::makeGate(QStringLiteral("ladder"));
        progressiveLadder.position = {7.63, -4.27};
        progressiveLadder.yaw = 37.17;
        auto threadTriple = ot::makeGate(QStringLiteral("triple"));
        threadTriple.position = {23.14, -4.27};
        threadTriple.yaw = -53.25;
        auto threadDouble = ot::makeGate(QStringLiteral("double"));
        threadDouble.position = {39.26, -4.27};
        threadDouble.yaw = 180;
        auto leftExitCube = ot::makeGate(QStringLiteral("cube"));
        leftExitCube.position = {55.37, -4.27};
        auto leftExitStack = ot::makeGate(QStringLiteral("cube-stack"));
        leftExitStack.position = {71.48, -4.27};
        progressive.gates = {progressiveLadder, threadTriple, threadDouble, leftExitCube,
                             leftExitStack};
        progressive.routeStepsDefined = true;
        progressive.routeSteps = ot::gateManeuverSteps(progressiveLadder, pattern);
        const auto ladderVisits = passes(progressive);
        require(ladderVisits.size() == 2 && checkpointSteps(progressive).size() == 3,
                "progressive upright wrap has two distinct-level visits and one outer control");
        if (ladderVisits.size() == 2) {
            const bool upward = pattern.endsWith(QStringLiteral("up"));
            require(ladderVisits[0].level == (upward ? 0 : 1) &&
                        ladderVisits[1].level == (upward ? 1 : 0),
                    "progressive upright finishes at next level without old-aperture return");
            const auto entry = ot::routeStepGeometry(progressive, ladderVisits[0]);
            const auto exit = ot::routeStepGeometry(progressive, ladderVisits[1]);
            require(
                (entry.position - exit.position).length() > 2.99f &&
                    QVector3D::dotProduct(entry.normal, exit.normal) > .99999f,
                "advancing upright helix retains same-facing physical passes at different heights");
        }
        auto addThread = [&progressive](const ot::Gate &gate, const QString &direction) {
            auto steps = ot::gateManeuverSteps(gate, direction);
            const double lateral = direction == QStringLiteral("right-left") ? -1 : 1;
            QVector<int> between;
            int visit = 0;
            for (qsizetype i = 0; i < steps.size(); ++i) {
                auto &step = steps[i];
                if (step.kind == QStringLiteral("pass")) {
                    if (!between.isEmpty()) {
                        auto &control = steps[between[between.size() / 2]];
                        control.extra.insert(QStringLiteral("raceCheckpoint"), true);
                        control.extra.insert(QStringLiteral("checkpointWidth"), .8);
                        control.extra.insert(QStringLiteral("checkpointHeight"), .8);
                    }
                    between.clear();
                    step.extra.insert(QStringLiteral("apertureU"),
                                      lateral * (visit % 2 ? -.2 : .2));
                    step.extra.insert(QStringLiteral("apertureV"), visit % 2 ? .35 : -.35);
                    step.extra.insert(QStringLiteral("flightTangent"),
                                      QJsonArray{lateral * .2, visit % 2 ? -.45 : .45,
                                                 step.face == QStringLiteral("back") ? -1 : 1});
                    ++visit;
                } else {
                    step.localGuide.setY(step.localGuide.y() + (visit % 2 ? .12f : -.12f));
                    between.append(static_cast<int>(i));
                }
            }
            progressive.routeSteps += steps;
        };
        addThread(threadTriple, QStringLiteral("left-right"));
        addThread(threadDouble, QStringLiteral("right-left"));
        auto addLeftGravity = [&progressive](const ot::Gate &gate) {
            const auto levels = ot::gateFrameMetrics(gate).levels;
            for (int level = levels - 1; level >= 0; --level) {
                auto roof = pass(gate, QStringLiteral("top"), 0, level);
                roof.extra = {{QStringLiteral("apertureU"), .2},
                              {QStringLiteral("apertureV"), -.15},
                              {QStringLiteral("flightTangent"), QJsonArray{-.18, -1, .12}}};
                progressive.routeSteps.append(roof);
            }
            ot::RouteStep interior;
            interior.id = ot::newGateId();
            interior.gateId = gate.id;
            interior.kind = QStringLiteral("guide");
            const float half = static_cast<float>(gate.width / 2), rootHalf = std::sqrt(.5f);
            const float top = static_cast<float>((1 - levels / 2.0) * gate.height);
            interior.localGuide = {(-half + half * rootHalf) / half,
                                   (top - half * rootHalf) / (levels * half), 0};
            interior.extra.insert(QStringLiteral("curveTangent"),
                                  QJsonArray{-rootHalf, -rootHalf / levels, 0});
            progressive.routeSteps.append(interior);
            auto leftExit = pass(gate, QStringLiteral("left"), 0, 0, true);
            leftExit.extra = {{QStringLiteral("apertureU"), .25},
                              {QStringLiteral("apertureV"), -.2},
                              {QStringLiteral("flightTangent"), QJsonArray{-1, -.18, .12}}};
            progressive.routeSteps.append(leftExit);
        };
        addLeftGravity(leftExitCube);
        addLeftGravity(leftExitStack);
        const auto visits = passes(progressive);
        require(visits.size() == 12 && checkpointSteps(progressive).size() == 16,
                "progressive/thread/gravity source has twelve actual visits and four explicit "
                "controls");
        int displaced = 0, oblique = 0;
        for (const auto &step : visits) {
            const auto geometry = ot::routeStepGeometry(progressive, step);
            require(geometry.valid, "synthetic progressive aperture/flight metadata remains valid");
            const auto rightBeforeReverse = step.reverse ? -geometry.right : geometry.right;
            const auto expectedPath =
                geometry.position +
                rightBeforeReverse *
                    static_cast<float>(step.extra.value(QStringLiteral("apertureU")).toDouble() *
                                       geometry.openingWidth / 2) +
                geometry.up *
                    static_cast<float>(step.extra.value(QStringLiteral("apertureV")).toDouble() *
                                       geometry.openingHeight / 2);
            require((geometry.pathPosition - expectedPath).length() < .0001f,
                    "off-centre aperture point uses face-local axes before reverse");
            require(QVector3D::dotProduct(geometry.pathTangent, geometry.normal) > .01f,
                    "oblique flight stays in the accepted physical plane hemisphere");
            displaced += (geometry.pathPosition - geometry.position).length() > .05f;
            oblique += QVector3D::dotProduct(geometry.pathTangent, geometry.normal) < .999f;
            if (step.face == QStringLiteral("left"))
                require(step.reverse && geometry.normal.x() < -.999f &&
                            near(geometry.position.x(),
                                 (step.gateId == leftExitCube.id ? leftExitCube.position.x()
                                                                 : leftExitStack.position.x()) -
                                     1.5),
                        "canonical cube/stack leaves lower LEFT face outward, not a rotated front "
                        "substitute");
        }
        require(displaced >= 8 && oblique >= 10,
                "progressive regression includes real height/depth aperture offsets and oblique "
                "flight");
        ot::TrackDocument savedProgressive;
        require(ot::fromJson(ot::toJson(progressive), &savedProgressive, &error) &&
                    ot::toJson(savedProgressive) == ot::toJson(progressive),
                "project roundtrip retains complete progressive path metadata and anchors");
        ot::TrackDocument nativeProgressive;
        if (!roundtrip(path, progressive, &nativeProgressive,
                       "progressive upright/thread/canonical-left native export"))
            continue;
        requireNativeCheckpoints(progressive, nativeProgressive);
        requireRouteGeometry(progressive, nativeProgressive);
        require(nativeProgressive.gates.size() == 5 && nativeBars(nativeProgressive).size() == 57 &&
                    nativeGates(nativeProgressive).size() == 16 &&
                    countPrefab(nativeBars(nativeProgressive), 422) == 1,
                "progressive export keeps five physical groups once and interleaves only four turn "
                "controls");
        const auto progressiveCipher = readFile(path);
        auto physicalWindowsOnly = progressive;
        for (auto &step : physicalWindowsOnly.routeSteps)
            if (step.kind == QStringLiteral("pass")) {
                step.extra.remove(QStringLiteral("apertureU"));
                step.extra.remove(QStringLiteral("apertureV"));
                step.extra.remove(QStringLiteral("flightTangent"));
            }
        require(
            ot::exportVelocidrone(path, physicalWindowsOnly, &error) &&
                readFile(path) == progressiveCipher,
            "preview aperture offsets/oblique tangents never shift or tilt the full native window");
        auto wrongInOut = progressive;
        wrongInOut.routeSteps[0].extra.insert(QStringLiteral("flightTangent"),
                                              QJsonArray{0, 0, -1});
        require(!ot::exportVelocidrone(path, wrongInOut, &error) &&
                    readFile(path) == progressiveCipher,
                "flight tangent opposing accepted entry/exit fails atomically instead of exporting "
                "wrong in/out");
    }
    auto adjacent = course();
    auto left = ot::makeGate(QStringLiteral("standard"));
    auto right = ot::makeGate(QStringLiteral("standard"));
    right.position = {3, 0};
    adjacent.gates = {left, right};
    ot::TrackDocument adjacentOutput;
    if (roundtrip(path, adjacent, &adjacentOutput, "adjacent independent frames"))
        require(adjacentOutput.gates.size() == 2 && nativeBars(adjacentOutput).size() == 9,
                "duplicated shared upright does not falsely merge two independent frames");
    auto deletion = course();
    auto a = ot::makeGate(QStringLiteral("start"));
    auto b = ot::makeGate(QStringLiteral("cube"));
    b.position = {10, 0};
    deletion.gates = {a, b};
    ot::TrackDocument deleteOutput;
    if (roundtrip(path, deletion, &deleteOutput, "assembly deletion setup") &&
        deleteOutput.gates.size() == 2) {
        const auto removedId = deleteOutput.gates[1].id;
        deleteOutput.gates.removeLast();
        for (qsizetype i = deleteOutput.routeSteps.size(); i-- > 0;)
            if (deleteOutput.routeSteps[i].gateId == removedId)
                deleteOutput.routeSteps.removeAt(i);
        ot::TrackDocument afterDelete;
        if (roundtrip(path, deleteOutput, &afterDelete, "delete physical group and its passes"))
            require(afterDelete.gates.size() == 1 && nativeBars(afterDelete).size() == 5,
                    "deleted group leaves no stale owned rods, grid retained");
    }
    // Failures preserve the existing output file and imported document atomically.
    const auto old = readFile(path);
    auto invalid = simple;
    invalid.takeoff.enabled = false;
    require(!ot::exportVelocidrone(path, invalid, &error) && !error.isEmpty() &&
                readFile(path) == old,
            "missing independent takeoff fails before file mutation");
    invalid = simple;
    invalid.routeStepsDefined = true;
    require(!ot::exportVelocidrone(path, invalid, &error),
            "explicit empty route never falls back to physical order");
    invalid = simple;
    invalid.routeStepsDefined = true;
    invalid.routeSteps = {pass(frame)};
    invalid.routeSteps[0].gateId = QStringLiteral("missing");
    require(!ot::exportVelocidrone(path, invalid, &error), "dangling traversal reference rejected");
    invalid = simple;
    invalid.takeoff.elevation = std::numeric_limits<double>::quiet_NaN();
    require(!ot::exportVelocidrone(path, invalid, &error), "nonfinite takeoff rejected");
    invalid = simple;
    invalid.gates[0].position.setX(std::numeric_limits<double>::infinity());
    require(!ot::exportVelocidrone(path, invalid, &error), "nonfinite physical pose rejected");
    invalid = simple;
    invalid.gates[0].type = QStringLiteral("flag");
    require(!ot::exportVelocidrone(path, invalid, &error), "new flag cannot masquerade as pillar");
    for (const auto &data :
         QList<QByteArray>{"", "not-base64", "AAAA", Golden.left(Golden.size() - 4)}) {
        writeFile(path, data);
        auto unchanged = simple;
        require(!ot::importVelocidrone(path, &unchanged, &error) && unchanged.name == simple.name &&
                    unchanged.gates.size() == simple.gates.size(),
                "bad file rejected atomically");
    }
    auto corrupt = QByteArray::fromBase64(Golden);
    corrupt[corrupt.size() - 1] ^= char(0x55);
    writeFile(path, corrupt.toBase64());
    require(!ot::importVelocidrone(path, &restored, &error) &&
                (error.contains(QStringLiteral("解密")) || error.contains(QStringLiteral("填充"))),
            "AES final-block malformed padding rejected before JSON parsing");
    std::cout << (failures ? "FAIL " : "PASS ") << failures << " native format checks failed.\n";
    return failures ? 1 : 0;
#endif
}
