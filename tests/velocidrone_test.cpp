#include "document.h"
#include "velocidrone.h"

#include <QCoreApplication>
#include <QFile>
#include <QJsonArray>
#include <QQuaternion>
#include <QSet>
#include <QTemporaryDir>
#include <QVector3D>
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
bool near(double a, double b, double tolerance = 0.011) {
    return std::abs(a - b) < tolerance;
}
bool writeFile(const QString &path, const QByteArray &data) {
    QFile file(path);
    return file.open(QIODevice::WriteOnly) && file.write(data) == data.size();
}
QByteArray readFile(const QString &path) {
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly))
        return {};
    return file.readAll();
}
// Original, synthetic one-gate course. This independent known-answer fixture was
// encrypted using .NET System.Security.Cryptography.Aes, not this implementation.
// Scene 16; UTF-8 name "Fixture {中文}"; one 90-degree gate at (-12.34,56.78),
// base altitude 0; native quaternion [707,0,707,0]; custom barrier + weather.
const QByteArray Golden = "e73GF0q1zUigczG3hm/zQA+O8GS0uwOtT/CYYDVuWYsC2p5td3187JLqj4F0+Rfl"
                          "6nipuA6VylE0eRMeLc4K+7PXiYlRSkDThcMIlE1XggU5dtzbZpgfTL/31rOoX0t/"
                          "Xyo8H7Wn+yerHil3NH9ewpXlgARWyqoGR80VLpJmEHMkycKJ5EWwB1Kn/ZB0KgakL"
                          "lxKP1JIj5jplQZ4UcM8tHSjHwx1js6pL2T0nzPl1o6615JoYICDFn+rIMFqa4Ncs6"
                          "N6mcK/hovZTxikFOkd0w+bqvy2Lcat1m7Akvkw1u75gx/7dO3+6TYA6U6TzM7oiH3"
                          "FDvJnUwtgPiF7ZNrZkdJ/TNr/OZ4hMb3vYLlk5CXcSYT99ACdutzeqY9elzP+EbWSB"
                          "MX2aTjOnX80EjTmrd+XHiBlJFCaRsGzKds7dLc=";
} // namespace

int main(int argc, char **argv) {
    QCoreApplication application(argc, argv);
#ifndef Q_OS_WIN
    std::cout << "SKIP: native .trk codec uses Windows CNG in this release.\n";
    return 0;
#else
    QTemporaryDir directory;
    require(directory.isValid(), "temporary directory");
    if (!directory.isValid())
        return 1;
    QString error;
    const auto originalPath = directory.filePath(QStringLiteral("golden.trk"));
    require(writeFile(originalPath, Golden), "write independently encoded fixture");
    ot::TrackDocument imported;
    require(ot::importVelocidrone(originalPath, &imported, &error), "decrypt .NET fixture");
    if (imported.gates.isEmpty()) {
        std::cerr << error.toStdString() << '\n';
        return 1;
    }
    require(imported.name == QStringLiteral("Fixture {中文}"), "UTF-8 name and braces preserved");
    require(imported.scenery == QStringLiteral("Empty Scene Day"), "scene mapping");
    require(imported.gates.size() == 1, "native gate count");
    const auto &first = imported.gates.first();
    require(first.type == QStringLiteral("start"), "start/finish mapping");
    require(near(first.position.x(), -12.34) && near(first.position.y(), 56.78),
            "centimetres to metres");
    require(near(first.elevation, 1.2), "native base to editor center altitude");
    require(near(first.yaw, 90, 0.1), "WXYZ quaternion and native positive yaw");
    require(near(first.width, 2.4) && near(first.height, 2.4), "percent dimensions");
    const auto root = imported.extra.value(QStringLiteral("velocidroneRoot")).toObject();
    require(root.value(QStringLiteral("barriers")).toArray().size() == 1, "barriers retained");
    require(
        root.value(QStringLiteral("weather")).toObject().value(QStringLiteral("hour")).toInt() ==
            14,
        "weather retained");
    const auto savedPath = directory.filePath(QStringLiteral("native-roundtrip.trk"));
    require(ot::exportVelocidrone(savedPath, imported, &error), "export native import");
    require(readFile(savedPath) == Golden,
            "unchanged native course matches independent encrypted fixture byte for byte");

    imported.gates[0].position += QPointF(2, -3);
    imported.gates[0].yaw = 0;
    imported.gates[0].height = 4.8;
    imported.gates[0].elevation = 4.4;
    require(ot::exportVelocidrone(savedPath, imported, &error), "export native geometry edit");
    ot::TrackDocument edited;
    require(ot::importVelocidrone(savedPath, &edited, &error), "reimport native geometry edit");
    if (!edited.gates.isEmpty()) {
        require(near(edited.gates[0].position.x(), -10.34) &&
                    near(edited.gates[0].position.y(), 53.78),
                "native move retains units");
        require(near(edited.gates[0].yaw, 0, 0.1), "native yaw edit");
        require(near(edited.gates[0].height, 4.8) && near(edited.gates[0].elevation, 4.4),
                "native resize retains center altitude");
        require(edited.extra.value(QStringLiteral("velocidroneRoot"))
                        .toObject()
                        .value(QStringLiteral("barriers")) ==
                    root.value(QStringLiteral("barriers")),
                "unrelated native barriers unchanged after geometry edit");
    }

    ot::TrackDocument created;
    created.name = QStringLiteral("测试赛道\nline");
    ot::Gate start;
    start.id = ot::newGateId();
    start.type = QStringLiteral("start");
    start.position = QPointF(1.23, -4.56);
    start.yaw = 90;
    ot::Gate gate;
    gate.id = ot::newGateId();
    gate.position = QPointF(20, 30);
    gate.elevation = 7.45;
    gate.width = 4.8;
    gate.height = 3.6;
    gate.yaw = -45;
    ot::Gate pillar;
    pillar.id = ot::newGateId();
    pillar.type = QStringLiteral("pillar");
    pillar.position = QPointF(40, 10);
    pillar.width = .4;
    pillar.height = 3;
    pillar.elevation = 1.5;
    created.gates = {start, gate, pillar};
    require(ot::exportVelocidrone(savedPath, created, &error), "export new course");
    ot::TrackDocument roundtrip;
    require(ot::importVelocidrone(savedPath, &roundtrip, &error), "read generated course");
    require(roundtrip.gates.size() == 3, "new course gate count");
    require(roundtrip.name == QStringLiteral("测试赛道 line"),
            "name newlines cannot break native framing");
    require(roundtrip.scenery == QStringLiteral("Football Stadium"), "football scene number 8");
    if (roundtrip.gates.size() == 3) {
        require(near(roundtrip.gates[1].width, 4.8) && near(roundtrip.gates[1].height, 3.6),
                "new dimensions roundtrip");
        require(near(roundtrip.gates[1].elevation, 7.45), "new center altitude roundtrip");
        require(near(roundtrip.gates[1].yaw, -45, 0.2), "negative yaw roundtrip");
        require(roundtrip.gates[2].type == QStringLiteral("pillar"),
                "native solid and adjacent checkpoint restore logical pillar");
        require(near(roundtrip.gates[2].position.x(), 40) && near(roundtrip.gates[2].width, .4) &&
                    near(roundtrip.gates[2].height, 3),
                "logical pillar retains physical dimensions");
        const auto native = roundtrip.extra.value(QStringLiteral("velocidroneRoot")).toObject();
        const auto physical = native.value(QStringLiteral("barriers")).toArray();
        require(physical.size() == 1, "one pillar creates exactly one solid barrier");
        if (physical.size() == 1) {
            const auto block = physical[0].toObject();
            require(block.value(QStringLiteral("prefab")).toInt() == 2218 &&
                        !block.contains(QStringLiteral("gate")),
                    "physical pillar is native BlockBlack barrier, not a race gate or flag");
            const auto transform = block.value(QStringLiteral("trans")).toObject();
            require(transform.value(QStringLiteral("scale")).toArray() == QJsonArray{40, 300, 40},
                    "unit cube stretches to 0.4 by 3 by 0.4 metres");
            require(transform.value(QStringLiteral("pos")).toArray() == QJsonArray{4000, 0, 1000},
                    "solid cube transform origin is at its base");
        }
        const auto checkpoint = native.value(QStringLiteral("gates")).toArray()[2].toObject();
        require(checkpoint.value(QStringLiteral("prefab")).toInt() == 88 &&
                    !checkpoint.value(QStringLiteral("start")).toBool(),
                "turn progress uses adjacent invisible checkpoint");
        const auto transform = checkpoint.value(QStringLiteral("trans")).toObject();
        require(
            transform.value(QStringLiteral("scale")).toArray() == QJsonArray{100, 150, 120},
            "invisible YZ helper lane is 2.4 metres wide and matches full 3 metre pillar height");
        const auto position = transform.value(QStringLiteral("pos")).toArray();
        const auto rotation = transform.value(QStringLiteral("rot")).toArray();
        const auto orientation = QQuaternion(static_cast<float>(rotation[0].toDouble() / 1000),
                                             static_cast<float>(rotation[1].toDouble() / 1000),
                                             static_cast<float>(rotation[2].toDouble() / 1000),
                                             static_cast<float>(rotation[3].toDouble() / 1000))
                                     .normalized();
        const auto offset = orientation.rotatedVector(QVector3D(0, 0, -1.2f));
        const double centerX = position[0].toDouble() / 100 + offset.x();
        require(near(centerX, 41.75) && near(position[1].toDouble() / 100, 1.5),
                "helper-origin compensation places checkpoint beside pillar center");
        require(near(centerX - 1.2 - 40 - .2, .35),
                "nominal checkpoint opening clears solid pillar by 0.35 metres");
        const auto normal = orientation.rotatedVector(QVector3D(1, 0, 0));
        require(near(normal.x(), 0, .002) && near(normal.z(), 1, .002),
                "native helper X-normal follows editor positive Z yaw zero");
    }
    require(ot::exportVelocidrone(savedPath, roundtrip, &error),
            "reexport imported physical pillar");
    ot::TrackDocument pairedAgain;
    require(ot::importVelocidrone(savedPath, &pairedAgain, &error), "reimport paired pillar");
    require(pairedAgain.extra.value(QStringLiteral("velocidroneRoot"))
                    .toObject()
                    .value(QStringLiteral("barriers"))
                    .toArray()
                    .size() == 1,
            "reexport does not duplicate owned solid barrier");
    if (pairedAgain.gates.size() == 3) {
        pairedAgain.gates[2].position += QPointF(5, 0);
        pairedAgain.gates[2].width = .6;
        pairedAgain.gates[2].height = 4;
        pairedAgain.gates[2].elevation = 2;
        pairedAgain.gates[2].yaw = 90;
        pairedAgain.gates[2].extra.insert(QStringLiteral("passSide"), -1);
        require(ot::exportVelocidrone(savedPath, pairedAgain, &error),
                "move resize rotate and reverse pillar passage side");
        ot::TrackDocument modifiedPillar;
        require(ot::importVelocidrone(savedPath, &modifiedPillar, &error),
                "restore modified pillar pairing");
        require(modifiedPillar.gates.size() == 3 &&
                    modifiedPillar.gates[2].type == QStringLiteral("pillar") &&
                    modifiedPillar.gates[2].extra.value(QStringLiteral("passSide")).toInt() == -1,
                "left-side passage roundtrips at positive 90 degree yaw");
        if (modifiedPillar.gates.size() == 3) {
            const auto point = ot::gatePassagePosition(modifiedPillar.gates[2]);
            require(near(point.x(), 45) && near(point.y(), 11.85),
                    "shared route helper matches exported left-side checkpoint");
            auto copy = modifiedPillar.gates[2];
            copy.id = ot::newGateId();
            copy.position += QPointF(3, 3);
            modifiedPillar.gates.append(copy);
            require(ot::exportVelocidrone(savedPath, modifiedPillar, &error),
                    "duplicate logical pillar");
            ot::TrackDocument duplicated;
            require(ot::importVelocidrone(savedPath, &duplicated, &error),
                    "restore duplicate pillars");
            require(duplicated.extra.value(QStringLiteral("velocidroneRoot"))
                            .toObject()
                            .value(QStringLiteral("barriers"))
                            .toArray()
                            .size() == 2,
                    "duplicate gets its own physical cube");
            modifiedPillar.gates.removeLast();
            modifiedPillar.gates.removeLast();
            require(ot::exportVelocidrone(savedPath, modifiedPillar, &error),
                    "delete imported pillar");
            ot::TrackDocument removed;
            require(ot::importVelocidrone(savedPath, &removed, &error),
                    "restore course after pillar deletion");
            require(removed.extra.value(QStringLiteral("velocidroneRoot"))
                        .toObject()
                        .value(QStringLiteral("barriers"))
                        .toArray()
                        .isEmpty(),
                    "deleting pillar removes owned physical cube");
        }
    }
    const auto legacyPath = directory.filePath(QStringLiteral("legacy-flag.trk"));
    require(writeFile(legacyPath,
                      "ZQUmrb8Dw13zB9S8edPtbNN1v3L0QVxOLqtHrDzXBK//"
                      "rbhPFqbFubEDk5iDSQhgZHfiPZfbc9BpLyYdQhbOd4iyrIi2J8rfQbrzF9clKfkedsnpcD5qBGo1"
                      "Fi9CzNbLKruOFEyHOQT/03PMviu8mPZOowG9WW3CpAPgpb4DN3tsBPE292GTUar4uH4PSUlSbW/"
                      "uDldFXFyve/xvZX0s1/mAXcx+THQEoVB++wyII9o="),
            "write independently encoded legacy flag fixture");
    ot::TrackDocument legacy;
    require(ot::importVelocidrone(legacyPath, &legacy, &error) && legacy.gates.size() == 1 &&
                legacy.gates[0].type == QStringLiteral("flag"),
            "legacy native 170 flags remain importable");
    require(ot::exportVelocidrone(savedPath, legacy, &error) &&
                readFile(savedPath) == readFile(legacyPath),
            "legacy imported flag retains exact native data");
    auto obsolete = created;
    obsolete.gates[2].type = QStringLiteral("flag");
    require(!ot::exportVelocidrone(savedPath, obsolete, &error),
            "new flags cannot masquerade as physical turn pillars");
    const auto colors = ot::nativeBlockColors();
    require(colors.size() == 13, "thirteen native cube materials exposed to UI");
    for (const auto &entry : colors)
        require(ot::nearestNativeBlockColor(entry.color).prefab == entry.prefab,
                "exact palette color selects its real native prefab");
    require(ot::nearestNativeBlockColor(QColor("#e50001")).prefab == 2220,
            "custom near-red RGB explicitly maps to native red material");

    ot::TrackDocument framed;
    framed.name = QStringLiteral("Exact colored beams");
    auto exact = ot::makeGate(QStringLiteral("start"));
    ot::setGateColor(exact, QColor("#e60000"));
    auto coloredPost = ot::makeGate(QStringLiteral("pillar"));
    coloredPost.position = {7, 9};
    coloredPost.yaw = 30;
    coloredPost.extra.insert(QStringLiteral("passSide"), -1);
    ot::setGateColor(coloredPost, QColor("#00cc00"));
    framed.gates = {exact, coloredPost};
    const auto compositePath = directory.filePath(QStringLiteral("composite.trk"));
    require(ot::exportVelocidrone(compositePath, framed, &error),
            "export exact colored four-cube frame and pillar");
    const QByteArray compositeCipher = readFile(compositePath);
    ot::TrackDocument recoveredFrame;
    require(ot::importVelocidrone(compositePath, &recoveredFrame, &error),
            "import composite frame assembly");
    if (recoveredFrame.gates.size() == 2) {
        require(ot::gateFrameMetrics(recoveredFrame.gates[0]).centerlineModel &&
                    near(recoveredFrame.gates[0].width, 3) &&
                    near(recoveredFrame.gates[0].height, 3),
                "four native bars and checkpoint reconstruct three metre centreline frame");
        require(ot::gateColor(recoveredFrame.gates[0]) == QColor("#e60000") &&
                    ot::gateColor(recoveredFrame.gates[1]) == QColor("#00cc00"),
                "frame and pillar retain independent native colors");
        const auto native =
            recoveredFrame.extra.value(QStringLiteral("velocidroneRoot")).toObject();
        const auto bars = native.value(QStringLiteral("barriers")).toArray();
        require(bars.size() == 5, "four frame bars plus one pillar are physical native barriers");
        int vertical = 0, horizontal = 0, green = 0;
        QSet<int> levels;
        for (const auto &value : bars) {
            const auto bar = value.toObject();
            const auto trans = bar.value(QStringLiteral("trans")).toObject();
            const auto scale = trans.value(QStringLiteral("scale")).toArray();
            if (bar.value(QStringLiteral("prefab")).toInt() == 2220) {
                require(!bar.contains(QStringLiteral("gate")),
                        "colored physical beam is not a trigger");
                if (scale == QJsonArray{10, 320, 10})
                    ++vertical;
                if (scale == QJsonArray{320, 10, 10})
                    ++horizontal;
                levels.insert(trans.value(QStringLiteral("pos")).toArray()[1].toInt());
            }
            if (bar.value(QStringLiteral("prefab")).toInt() == 2223)
                ++green;
        }
        require(vertical == 2 && horizontal == 2 && green == 1,
                "stock beams are exactly3.2m long with0.1m square section");
        require(levels == QSet<int>{0, 5, 305},
                "four beam origins reproduce centerline3m and ground contact");
        const auto checkpoint = native.value(QStringLiteral("gates")).toArray()[0].toObject();
        require(checkpoint.value(QStringLiteral("prefab")).toInt() == 88 &&
                    checkpoint.value(QStringLiteral("start")).toBool(),
                "start checkpoint is separate from solid beams");
        require(checkpoint.value(QStringLiteral("trans"))
                        .toObject()
                        .value(QStringLiteral("scale"))
                        .toArray() == QJsonArray{100, 145, 145},
                "checkpoint follows clear opening2.9m, not centerline3m or stocklength3.2m");
        require(ot::exportVelocidrone(compositePath, recoveredFrame, &error) &&
                    readFile(compositePath) == compositeCipher,
                "unchanged composite raw assemblies roundtrip byte-for-byte without barrier "
                "duplication");
        ot::setGateColor(recoveredFrame.gates[0], QColor("#0000e6"));
        ot::setGateColor(recoveredFrame.gates[1], QColor("#00d9d9"));
        recoveredFrame.gates[0].position = {4, 2};
        recoveredFrame.gates[0].yaw = 90;
        recoveredFrame.gates[0].width = 4;
        recoveredFrame.gates[0].height = 3.5;
        require(ot::exportVelocidrone(compositePath, recoveredFrame, &error),
                "recolor resize move and rotate composite geometry");
        ot::TrackDocument changed;
        require(ot::importVelocidrone(compositePath, &changed, &error),
                "restore modified colored assemblies");
        if (changed.gates.size() == 2) {
            require(ot::gateFrameMetrics(changed.gates[0]).centerlineModel &&
                        near(changed.gates[0].width, 4) && near(changed.gates[0].height, 3.5) &&
                        near(changed.gates[0].yaw, 90, .2),
                    "modified dimensions and yaw retained");
            require(ot::gateColor(changed.gates[0]) == QColor("#0000e6") &&
                        ot::gateColor(changed.gates[1]) == QColor("#00d9d9"),
                    "changed native color materials replace old physical assemblies");
            auto duplicate = changed.gates[0];
            duplicate.id = ot::newGateId();
            duplicate.type = QStringLiteral("standard");
            duplicate.position += QPointF(6, 0);
            changed.gates.append(duplicate);
            require(ot::exportVelocidrone(compositePath, changed, &error),
                    "duplicate frame assembly");
            ot::TrackDocument duplicated;
            require(ot::importVelocidrone(compositePath, &duplicated, &error),
                    "restore duplicate frame assembly");
            require(duplicated.extra.value(QStringLiteral("velocidroneRoot"))
                            .toObject()
                            .value(QStringLiteral("barriers"))
                            .toArray()
                            .size() == 9,
                    "duplicate creates four additional beams only");
            if (duplicated.gates.size() == 3) {
                duplicated.gates.removeLast();
                require(ot::exportVelocidrone(compositePath, duplicated, &error),
                        "delete imported composite assembly");
                ot::TrackDocument deleted;
                require(ot::importVelocidrone(compositePath, &deleted, &error),
                        "restore after composite deletion");
                require(deleted.extra.value(QStringLiteral("velocidroneRoot"))
                                .toObject()
                                .value(QStringLiteral("barriers"))
                                .toArray()
                                .size() == 5,
                        "deleting logical frame removes all four owned beams");
            }
        }
    } else
        require(false, "composite course logical object count");

    const QList<QPair<QString, int>> compoundKinds = {{QStringLiteral("double"), 7},
                                                      {QStringLiteral("triple"), 10},
                                                      {QStringLiteral("ladder"), 7},
                                                      {QStringLiteral("cube"), 12},
                                                      {QStringLiteral("cube-stack"), 20}};
    for (const auto &definition : compoundKinds) {
        for (int selectedCell = 0; selectedCell < 2; ++selectedCell) {
            ot::TrackDocument course;
            auto anchor = ot::makeGate(QStringLiteral("start"));
            anchor.position = {-50, -50};
            auto group = ot::makeGate(definition.first);
            group.position = {8, 12};
            group.yaw = 37;
            const auto dimensions = ot::gateFrameMetrics(group);
            if (selectedCell) {
                group.extra.insert(QStringLiteral("passColumn"), dimensions.columns - 1);
                group.extra.insert(QStringLiteral("passLevel"), dimensions.levels - 1);
            }
            ot::setGateColor(group, QColor("#f280bf"));
            course.gates = {anchor, group};
            require(ot::exportVelocidrone(compositePath, course, &error),
                    "export compound native stock assembly");
            const auto nativeCipher = readFile(compositePath);
            ot::TrackDocument compound;
            require(ot::importVelocidrone(compositePath, &compound, &error),
                    "restore complete compound assembly before matching simple face");
            if (compound.gates.size() != 2) {
                require(false, "compound logical object count");
                continue;
            }
            const auto &restored = compound.gates[1];
            if (restored.type != definition.first)
                std::cerr << "Compound type mismatch: " << definition.first.toStdString() << " -> "
                          << restored.type.toStdString() << '\n';
            require(restored.type == definition.first,
                    "compound type survives native geometry reconstruction");
            require(near(restored.position.x(), 8, .025) && near(restored.position.y(), 12, .025) &&
                        near(restored.elevation, group.elevation, .025),
                    "compound physical group center reconstructed from offset passage");
            require(restored.extra.value(QStringLiteral("passColumn")).toInt(0) ==
                            group.extra.value(QStringLiteral("passColumn")).toInt(0) &&
                        restored.extra.value(QStringLiteral("passLevel")).toInt(0) ==
                            group.extra.value(QStringLiteral("passLevel")).toInt(0),
                    "left/right and lower/upper selected passage restored");
            require(ot::gateFrameBeams(restored).size() == definition.second,
                    "compound owns exact unique stock count");
            const auto native = compound.extra.value(QStringLiteral("velocidroneRoot")).toObject();
            require(native.value(QStringLiteral("barriers")).toArray().size() ==
                        4 + definition.second,
                    "compound physical barriers contain N unique bars plus separate start gate");
            const auto cp = native.value(QStringLiteral("gates"))
                                .toArray()[1]
                                .toObject()
                                .value(QStringLiteral("trans"))
                                .toObject();
            const auto cpPos = cp.value(QStringLiteral("pos")).toArray(),
                       cpRot = cp.value(QStringLiteral("rot")).toArray();
            const auto q = QQuaternion(static_cast<float>(cpRot[0].toDouble() / 1000),
                                       static_cast<float>(cpRot[1].toDouble() / 1000),
                                       static_cast<float>(cpRot[2].toDouble() / 1000),
                                       static_cast<float>(cpRot[3].toDouble() / 1000))
                               .normalized();
            const auto o = q.rotatedVector(QVector3D(0, 0, -1.45f));
            const auto passage = ot::gatePassagePosition(group);
            require(
                near(cpPos[0].toDouble() / 100 + o.x(), passage.x(), .02) &&
                    near(cpPos[2].toDouble() / 100 + o.z(), passage.y(), .02) &&
                    near(cpPos[1].toDouble() / 100 + o.y(), ot::gatePassageElevation(group), .02),
                "compound checkpoint crosses selected cell, not central solid bars");
            require(ot::exportVelocidrone(compositePath, compound, &error) &&
                        readFile(compositePath) == nativeCipher,
                    "unchanged N-bar compound roundtrips exact native ciphertext");
            auto copy = compound.gates[1];
            copy.id = ot::newGateId();
            copy.position += QPointF(20, 0);
            compound.gates.append(copy);
            require(ot::exportVelocidrone(compositePath, compound, &error),
                    "duplicate compound assembly");
            ot::TrackDocument twice;
            require(ot::importVelocidrone(compositePath, &twice, &error),
                    "restore duplicated compound assembly");
            require(twice.extra.value(QStringLiteral("velocidroneRoot"))
                            .toObject()
                            .value(QStringLiteral("barriers"))
                            .toArray()
                            .size() == 4 + definition.second * 2,
                    "duplicate creates exactly N additional physical stocks");
            if (twice.gates.size() == 3) {
                twice.gates.removeLast();
                require(ot::exportVelocidrone(compositePath, twice, &error),
                        "delete compound assembly");
                ot::TrackDocument once;
                require(ot::importVelocidrone(compositePath, &once, &error),
                        "restore course after compound deletion");
                require(once.extra.value(QStringLiteral("velocidroneRoot"))
                                .toObject()
                                .value(QStringLiteral("barriers"))
                                .toArray()
                                .size() == 4 + definition.second,
                        "compound deletion removes all N owned bars");
            } else
                require(false, "duplicate compound logical count");
        }
    }

    ot::TrackDocument adjacent;
    auto adjacentStart = ot::makeGate(QStringLiteral("start"));
    auto adjacentSingle = ot::makeGate(QStringLiteral("standard"));
    adjacentSingle.position = {3, 0};
    adjacent.gates = {adjacentStart, adjacentSingle};
    require(ot::exportVelocidrone(compositePath, adjacent, &error),
            "export independent adjacent single gates");
    ot::TrackDocument adjacentRestored;
    require(ot::importVelocidrone(compositePath, &adjacentRestored, &error),
            "import adjacent singles without accidental group merge");
    require(adjacentRestored.gates.size() == 2 &&
                adjacentRestored.gates[0].type == QStringLiteral("start") &&
                adjacentRestored.gates[1].type == QStringLiteral("standard"),
            "multiple checkpoints prevent merging independent singles into double");
    ot::TrackDocument repeated;
    auto repeatedAnchor = ot::makeGate(QStringLiteral("start"));
    repeatedAnchor.position = {-30, -30};
    auto leftPass = ot::makeGate(QStringLiteral("double"));
    leftPass.position = {10, 20};
    auto rightPass = leftPass;
    rightPass.id = ot::newGateId();
    rightPass.extra.insert(QStringLiteral("passColumn"), 1);
    repeated.gates = {repeatedAnchor, leftPass, rightPass};
    require(ot::exportVelocidrone(compositePath, repeated, &error),
            "export overlapping repeated compound passages");
    ot::TrackDocument repeatedRestored;
    require(ot::importVelocidrone(compositePath, &repeatedRestored, &error),
            "restore repeated compound passages with complete stock copies");
    require(repeatedRestored.gates.size() == 3 &&
                repeatedRestored.gates[1].type == QStringLiteral("double") &&
                repeatedRestored.gates[2].type == QStringLiteral("double"),
            "full duplicate assemblies retain independent selected cells");

    const QByteArray sentinel("keep existing file");
    require(writeFile(savedPath, sentinel), "write sentinel");
    created.gates[1].type = QStringLiteral("cube");
    require(!ot::exportVelocidrone(savedPath, created, &error) && !error.isEmpty(),
            "unsupported newly created component rejected");
    require(readFile(savedPath) == sentinel, "failed export preserves existing file");
    created.gates[1].type = QStringLiteral("standard");
    created.gates[1].position.setX(std::numeric_limits<double>::quiet_NaN());
    require(!ot::exportVelocidrone(savedPath, created, &error),
            "NaN rejected before numeric conversion");
    created.gates[1].position.setX(20);
    created.gates[1].type = QStringLiteral("start");
    require(!ot::exportVelocidrone(savedPath, created, &error),
            "ambiguous multiple start gates rejected");

    ot::TrackDocument unchanged;
    unchanged.name = QStringLiteral("Keep me");
    const auto badPath = directory.filePath(QStringLiteral("bad.trk"));
    require(writeFile(badPath, "not valid base64 %"), "write invalid base64");
    require(!ot::importVelocidrone(badPath, &unchanged, &error), "invalid base64 rejected");
    require(unchanged.name == QStringLiteral("Keep me"), "failed import is transactional");
    auto ciphertext = QByteArray::fromBase64(Golden);
    ciphertext[ciphertext.size() - 1] = static_cast<char>(ciphertext.back() ^ 0xFF);
    require(writeFile(badPath, ciphertext.toBase64()), "write invalid padding");
    require(!ot::importVelocidrone(badPath, &unchanged, &error), "corrupt AES padding rejected");
    require(unchanged.name == QStringLiteral("Keep me"), "corrupt import preserves document");
    require(!ot::importVelocidrone(originalPath, nullptr, &error), "null output rejected");
    if (!failures)
        std::cout << "PASS: native schema, independent AES fixtures, physical pillars, adjacent "
                     "checkpoints, "
                     "pairing ownership, legacy flags and invalid input.\n";
    return failures ? 1 : 0;
#endif
}
