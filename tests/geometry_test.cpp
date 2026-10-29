#include "document.h"

#include <QCoreApplication>
#include <QJsonArray>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>

namespace {
int failures = 0;
void require(bool condition, const char *description) {
    if (!condition) {
        ++failures;
        std::cerr << "FAIL: " << description << '\n';
    }
}
bool close(double a, double b) {
    return std::abs(a - b) < 1e-5;
}
} // namespace

int main(int argc, char **argv) {
    QCoreApplication app(argc, argv);
    const ot::Gate gate = ot::makeGate();
    const auto metrics = ot::gateFrameMetrics(gate);
    require(metrics.centerlineModel && close(gate.width, 3) && close(gate.height, 3),
            "new frame uses 3m beam centerline spacing");
    require(close(metrics.beamThickness, .1) && close(metrics.horizontalBeamLength, 3.2) &&
                close(metrics.verticalBeamLength, 3.2),
            "all four new stock beams are 3.2m long with a 0.1m square section");
    require(close(metrics.openingWidth, 2.9) && close(metrics.openingHeight, 2.9),
            "clear opening subtracts the beam section from centerline spacing");
    require(close(metrics.outerWidth, 3.2) && close(metrics.outerHeight, 3.2),
            "physical beam union has 3.2m outer dimensions");
    require(close(gate.elevation, 1.6), "new stock frame rests on the ground at center1.6m");

    // These are the exact shared cuboids consumed by the 3D frame renderer.
    const auto beams = ot::gateFrameBeams(gate);
    require(beams.size() == 4, "frame consists of four solid beam cuboids");
    QVector3D minimum(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(),
                      std::numeric_limits<float>::max());
    QVector3D maximum = -minimum;
    for (qsizetype index = 0; index < beams.size(); ++index) {
        const auto &beam = beams[index];
        const QVector3D low = beam.center - beam.size / 2;
        const QVector3D high = beam.center + beam.size / 2;
        minimum.setX(std::min(minimum.x(), low.x()));
        minimum.setY(std::min(minimum.y(), low.y()));
        minimum.setZ(std::min(minimum.z(), low.z()));
        maximum.setX(std::max(maximum.x(), high.x()));
        maximum.setY(std::max(maximum.y(), high.y()));
        maximum.setZ(std::max(maximum.z(), high.z()));
        if (index < 2) {
            require(close(std::abs(beam.center.x()), 1.5) && close(beam.center.y(), 0),
                    "vertical beam centerlines are 3m apart");
            require(close(beam.size.x(), .1) && close(beam.size.y(), 3.2) &&
                        close(beam.size.z(), .1),
                    "vertical beam uses the specified stock section and length");
        } else {
            require(close(beam.size.x(), 3.2) && close(beam.size.y(), .1) &&
                        close(beam.size.z(), .1),
                    "horizontal beam uses the specified stock section and length");
        }
    }
    require(close(minimum.x(), -1.6) && close(maximum.x(), 1.6) &&
                close(minimum.y() + gate.elevation, 0) &&
                close(maximum.y() + gate.elevation, 3.2) && close(minimum.z(), -.05) &&
                close(maximum.z(), .05),
            "shared render cuboids have the requested outer bounds and ground clearance");
    require(close(beams[2].center.y() + gate.elevation, 3.1) &&
                close(beams[3].center.y() + gate.elevation, .1),
            "horizontal centerlines are 0.1m and 3.1m above ground");

    const auto pillar = ot::makeGate(QStringLiteral("pillar"));
    require(close(pillar.width, .1) && close(pillar.height, 3.2) && close(pillar.elevation, 1.6),
            "new turning pillar is a slim 0.1m square by 3.2m solid post");
    require(ot::gateFrameBeams(pillar).isEmpty() &&
                close(ot::gateFrameMetrics(pillar).outerWidth, .1),
            "a pillar is a solid post rather than an opening frame");
    const auto custom = ot::makeGate(QStringLiteral("standard"), 5, 2.5);
    const auto customMetrics = ot::gateFrameMetrics(custom);
    require(close(custom.width, 5) && close(custom.height, 2.5) &&
                close(customMetrics.horizontalBeamLength, 5.2) &&
                close(customMetrics.verticalBeamLength, 2.7) && close(custom.elevation, 1.35),
            "new custom dimensions are honored with consistent end extensions");

    struct CompoundCase {
        const char *type;
        int columns, levels, stocks;
        double outerWidth, outerHeight, outerDepth, passageX, passageZ;
    };
    const CompoundCase groups[] = {
        {"double", 2, 1, 7, 6.2, 3.2, .1, -1.5, 0},
        {"triple", 3, 1, 10, 9.2, 3.2, .1, -3, 0},
        {"ladder", 1, 2, 7, 3.2, 6.2, .1, 0, 0},
        {"quad", 2, 2, 12, 6.2, 6.2, .1, -1.5, 0},
        {"cube", 1, 1, 12, 3.2, 3.2, 3.2, 0, -1.5},
        {"cube-stack", 1, 2, 20, 3.2, 6.2, 3.2, 0, -1.5},
    };
    for (const auto &group : groups) {
        auto item = ot::makeGate(QString::fromLatin1(group.type));
        const auto shape = ot::gateFrameMetrics(item);
        const auto stocks = ot::gateFrameBeams(item);
        require(shape.columns == group.columns && shape.levels == group.levels &&
                    close(item.width, 3) && close(item.height, 3),
                "compound dimensions retain per-cell centerline spacing");
        require(stocks.size() == group.stocks && close(shape.outerWidth, group.outerWidth) &&
                    close(shape.outerHeight, group.outerHeight) &&
                    close(shape.outerDepth, group.outerDepth),
                "compound has the expected unique stock count and whole-group bounds");
        QVector3D low(1e6f, 1e6f, 1e6f), high = -low;
        for (qsizetype i = 0; i < stocks.size(); ++i) {
            const auto &stock = stocks[i];
            const auto a = stock.center - stock.size / 2;
            const auto b = stock.center + stock.size / 2;
            int longSides = 0, thinSides = 0;
            for (int axis = 0; axis < 3; ++axis) {
                low[axis] = std::min(low[axis], a[axis]);
                high[axis] = std::max(high[axis], b[axis]);
                longSides += close(stock.size[axis], 3.2);
                thinSides += close(stock.size[axis], .1);
            }
            require(longSides == 1 && thinSides == 2,
                    "every compound stock is exactly 3.2 by 0.1 by 0.1 metres");
            for (qsizetype j = 0; j < i; ++j)
                require(stock.center != stocks[j].center || stock.size != stocks[j].size,
                        "shared walls and joints contain no duplicate stock cuboids");
        }
        require(close(high.x() - low.x(), shape.outerWidth) &&
                    close(high.y() - low.y(), shape.outerHeight) &&
                    close(high.z() - low.z(), shape.outerDepth) &&
                    close(low.y() + item.elevation, 0),
                "compound metric bounds agree with all physical beams and the floor");
        auto passage = ot::gatePassagePosition(item);
        require(close(passage.x(), group.passageX) && close(passage.y(), group.passageZ) &&
                    close(ot::gatePassageElevation(item), 1.6),
                "default passage selects the leftmost lower cell and cube front wall");
        // A passage must be in a clear cell, never in a central divider or shared wall post.
        const QVector3D localPassage(
            static_cast<float>(passage.x()),
            static_cast<float>(ot::gatePassageElevation(item) - item.elevation),
            static_cast<float>(passage.y()));
        for (const auto &stock : stocks) {
            const auto delta = localPassage - stock.center;
            require(std::abs(delta.x()) > stock.size.x() / 2 ||
                        std::abs(delta.y()) > stock.size.y() / 2 ||
                        std::abs(delta.z()) > stock.size.z() / 2,
                    "checkpoint center is outside every physical stock");
        }
        item.position = {10, 20};
        item.yaw = 90;
        passage = ot::gatePassagePosition(item);
        require(close(passage.x(), 10 + group.passageZ) && close(passage.y(), 20 - group.passageX),
                "compound passage rotates and translates with the existing yaw convention");
        item.extra.insert(QStringLiteral("passColumn"), group.columns - 1);
        item.extra.insert(QStringLiteral("passLevel"), group.levels - 1);
        const auto lastColumnPassage = ot::gatePassagePosition(item);
        require(close(lastColumnPassage.x(), 10 + group.passageZ) &&
                    close(lastColumnPassage.y(), 20 + group.passageX),
                "optional passage column selects the rightmost clear cell");
        require(close(ot::gatePassageElevation(item),
                      item.elevation + (group.levels - 1) * item.height / 2),
                "optional passage level selects the upper clear cell");
        ot::TrackDocument grouped;
        grouped.gates = {item};
        ot::TrackDocument loaded;
        QString groupError;
        require(ot::fromJson(ot::toJson(grouped), &loaded, &groupError) &&
                    ot::gateFrameBeams(loaded.gates[0]).size() == group.stocks &&
                    ot::gatePassagePosition(loaded.gates[0]) == ot::gatePassagePosition(item),
                "compound geometry and selected passage survive project roundtrip");
        for (const auto &field : {QStringLiteral("passColumn"), QStringLiteral("passLevel")}) {
            const auto original = loaded.gates;
            grouped.gates[0].extra.insert(field, -1);
            require(!ot::fromJson(ot::toJson(grouped), &loaded, &groupError) &&
                        loaded.gates[0].extra == original[0].extra,
                    "negative passage indices fail without mutating the destination");
            grouped.gates[0].extra.insert(field, .5);
            require(!ot::fromJson(ot::toJson(grouped), &loaded, &groupError),
                    "fractional passage indices are rejected");
            grouped.gates[0].extra.insert(
                field, field == QStringLiteral("passColumn") ? group.columns : group.levels);
            require(!ot::fromJson(ot::toJson(grouped), &loaded, &groupError),
                    "out-of-group passage indices are rejected");
            grouped.gates[0].extra = item.extra;
        }
    }

    auto blocked = ot::makeGate(QStringLiteral("ladder-blocked"));
    const auto blockedMetrics = ot::gateFrameMetrics(blocked);
    const auto blockedSolids = ot::gateFrameBeams(blocked);
    require(blockedMetrics.columns == 1 && blockedMetrics.levels == 2 &&
                close(blockedMetrics.outerHeight, 6.2) && blockedSolids.size() == 8 &&
                close(ot::gatePassageElevation(blocked), 4.6),
            "blocked ladder retains two physical levels but defaults to its only upper aperture");
    require(!ot::gateCellOpen(blocked, 0, 0) && ot::gateCellOpen(blocked, 0, 1) &&
                !ot::gateCellOpen(blocked, 1, 1),
            "solid lower panel and out-of-group cells cannot be selected as apertures");
    const auto panel = blockedSolids.last();
    require(panel.center == QVector3D(0, -1.5f, 0) && panel.size == QVector3D(3, 3, .1f),
            "lower panel fills the whole centreline cell without changing the stock frames");
    ot::TrackDocument blockedDocument;
    blockedDocument.gates = {blocked};
    const auto defaultPasses = ot::effectiveRouteSteps(blockedDocument);
    require(defaultPasses.size() == 1 && defaultPasses[0].level == 1 &&
                ot::routeStepGeometry(blockedDocument, defaultPasses[0]).valid,
            "implicit route chooses the open upper cell of a blocked ladder");
    auto blockedPass = defaultPasses[0];
    blockedPass.level = 0;
    require(!ot::routeStepGeometry(blockedDocument, blockedPass).valid,
            "an explicit pass through a filled panel is invalid geometry");
    ot::TrackDocument blockedLoaded;
    QString blockedError;
    require(ot::fromJson(ot::toJson(blockedDocument), &blockedLoaded, &blockedError) &&
                blockedLoaded.gates[0].type == QStringLiteral("ladder-blocked") &&
                ot::gateFrameBeams(blockedLoaded.gates[0]).size() == 8,
            "blocked geometry and default open cell survive project roundtrip");
    blockedDocument.routeStepsDefined = true;
    blockedDocument.routeSteps = {blockedPass};
    require(!ot::fromJson(ot::toJson(blockedDocument), &blockedLoaded, &blockedError),
            "project validation rejects routes through the blocked lower aperture");
    blockedDocument.routeSteps.clear();
    blockedDocument.gates[0].extra.insert(QStringLiteral("passLevel"), 0);
    require(!ot::fromJson(ot::toJson(blockedDocument), &blockedLoaded, &blockedError),
            "project validation rejects a blocked default passage even with an empty route");
    require(ot::gateManeuverSteps(blocked, QStringLiteral("single")).size() == 1 &&
                ot::gateManeuverSteps(blocked, QStringLiteral("single"))[0].level == 1 &&
                ot::gateManeuverSteps(blocked, QStringLiteral("lower-upper")).isEmpty() &&
                ot::gateManeuverSteps(blocked, QStringLiteral("orbit-bar-cw")).isEmpty(),
            "manual templates never create a traversal through the blocked lower cell");

    ot::Gate legacy;
    legacy.id = QStringLiteral("legacy-dimension-fixture");
    legacy.width = 4.8;
    legacy.height = 1.9;
    legacy.elevation = 4.2;
    const auto legacyMetrics = ot::gateFrameMetrics(legacy);
    const double oldThickness = std::clamp(std::min(legacy.width, legacy.height) * .041, .055, .16);
    require(!legacyMetrics.centerlineModel && close(legacyMetrics.beamThickness, oldThickness) &&
                close(legacyMetrics.horizontalBeamLength, 4.8) &&
                close(legacyMetrics.verticalBeamLength, 1.9 + oldThickness),
            "unmarked old projects retain their earlier bar dimensions");
    legacy.type = QStringLiteral("ladder");
    legacy.extra.insert(QStringLiteral("passLevel"), 1);
    require(ot::gateFrameMetrics(legacy).levels == 1 &&
                close(ot::gatePassageElevation(legacy), legacy.elevation) &&
                ot::gatePassagePosition(legacy) == legacy.position,
            "legacy unmarked groups retain their old whole-frame dimensions and passage");
    ot::TrackDocument before;
    before.gates = {legacy, gate, pillar};
    ot::TrackDocument restored;
    QString error;
    require(ot::fromJson(ot::toJson(before), &restored, &error),
            "legacy and new geometry can coexist in the same project");
    require(close(restored.gates[0].width, 4.8) && close(restored.gates[0].height, 1.9) &&
                close(restored.gates[0].elevation, 4.2) &&
                !ot::gateFrameMetrics(restored.gates[0]).centerlineModel &&
                ot::gateFrameMetrics(restored.gates[1]).centerlineModel,
            "project loading does not silently rescale or upgrade old dimensions");
    auto broken = ot::toJson(before);
    auto array = broken.value(QStringLiteral("gates")).toArray();
    auto changed = array[1].toObject();
    auto extra = changed.value(QStringLiteral("extra")).toObject();
    extra.insert(QStringLiteral("beamThickness"), -1);
    changed.insert(QStringLiteral("extra"), extra);
    array[1] = changed;
    broken.insert(QStringLiteral("gates"), array);
    require(!ot::fromJson(broken, &restored, &error), "invalid stock section is rejected");

    const auto demo = ot::demoDocument();
    for (const auto &item : demo.gates) {
        const auto frame = ot::gateFrameMetrics(item);
        require(item.elevation - frame.outerHeight / 2 >= -1e-6,
                "new demonstration objects do not extend below the ground");
        if (item.type != QStringLiteral("pillar"))
            require(close(frame.horizontalBeamLength, 3.2) && close(frame.verticalBeamLength, 3.2),
                    "all demonstration frame beams use 3.2m stock");
    }
    if (!failures)
        std::cout
            << "PASS: stock beam geometry, centerline/opening/outer dimensions, slim pillars, "
               "compound stocks/passages, custom sizes and legacy compatibility.\n";
    return failures ? 1 : 0;
}
