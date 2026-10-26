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
               "custom sizes and legacy compatibility.\n";
    return failures ? 1 : 0;
}
