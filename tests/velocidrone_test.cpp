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
        require(transform.value(QStringLiteral("rot")).toArray() ==
                    QJsonArray{500, -500, -500, -500},
                "measured prefab88 basis is Xnormal/Yright/Zup, not editor-helper axes");
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

    const QList<QPair<QString, int>> kinds = {{"standard", 4}, {"double", 7}, {"triple", 10},
                                              {"ladder", 7},   {"cube", 12},  {"cube-stack", 20}};
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
