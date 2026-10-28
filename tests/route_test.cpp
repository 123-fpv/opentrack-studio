#include "document.h"

#include <QCoreApplication>
#include <QFile>
#include <QJsonArray>
#include <QQuaternion>
#include <QTemporaryDir>
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
bool close(const QVector3D &a, const QVector3D &b) {
    return (a - b).length() < 1e-4f;
}

ot::RouteStep pass(const ot::Gate &gate, int column = 0, int level = 0,
                   const QString &face = QStringLiteral("front")) {
    ot::RouteStep step;
    step.id = ot::newGateId();
    step.gateId = gate.id;
    step.column = column;
    step.level = level;
    step.face = face;
    return step;
}

bool intersectsBeam(QVector3D a, QVector3D b, const ot::GateBeam &beam, float clearance = 0) {
    const auto half = beam.size / 2 + QVector3D(clearance, clearance, clearance);
    const auto low = beam.center - half, high = beam.center + half;
    float begin = 0, end = 1;
    for (int axis = 0; axis < 3; ++axis) {
        const float delta = b[axis] - a[axis];
        if (std::abs(delta) < 1e-6f) {
            if (a[axis] < low[axis] || a[axis] > high[axis])
                return false;
        } else {
            const float t0 = (low[axis] - a[axis]) / delta;
            const float t1 = (high[axis] - a[axis]) / delta;
            begin = std::max(begin, std::min(t0, t1));
            end = std::min(end, std::max(t0, t1));
            if (begin > end)
                return false;
        }
    }
    return true;
}

void denseClearance(const ot::TrackDocument &document, const ot::RouteCurveResult &curve) {
    for (const auto &point : curve.points)
        require(std::isfinite(point.x()) && std::isfinite(point.y()) && std::isfinite(point.z()) &&
                    point.y() >= ot::routePreviewClearance() - 1e-4,
                "sampled curve remains finite and above the drone-clearance floor");
    for (const auto &gate : document.gates) {
        const auto inverse =
            QQuaternion::fromAxisAndAngle(0, 1, 0, static_cast<float>(gate.yaw)).conjugated();
        const QVector3D origin(static_cast<float>(gate.position.x()),
                               static_cast<float>(gate.elevation),
                               static_cast<float>(gate.position.y()));
        auto beams = ot::gateFrameBeams(gate);
        if (gate.type == QStringLiteral("pillar"))
            beams = {{{0, 0, 0},
                      {static_cast<float>(gate.width), static_cast<float>(gate.height),
                       static_cast<float>(gate.width)}}};
        for (qsizetype i = 1; i < curve.points.size(); ++i) {
            if (curve.breaks.contains(static_cast<int>(i)))
                continue;
            for (const auto &beam : beams)
                require(!intersectsBeam(inverse.rotatedVector(curve.points[i - 1] - origin),
                                        inverse.rotatedVector(curve.points[i] - origin), beam,
                                        static_cast<float>(ot::routePreviewClearance())),
                        "every sampled segment clears the actual rotated stock by drone radius");
        }
    }
}

double barWinding(const ot::Gate &gate, const QVector<QVector3D> &points, double centerY) {
    const auto inverse =
        QQuaternion::fromAxisAndAngle(0, 1, 0, static_cast<float>(gate.yaw)).conjugated();
    const QVector3D origin(static_cast<float>(gate.position.x()),
                           static_cast<float>(gate.elevation),
                           static_cast<float>(gate.position.y()));
    double previous = 0, winding = 0;
    bool first = true;
    for (const auto &point : points) {
        const auto local = inverse.rotatedVector(point - origin);
        const double angle = std::atan2(local.y() - centerY, local.z());
        if (!first)
            winding += std::remainder(angle - previous, 2 * 3.14159265358979323846);
        first = false;
        previous = angle;
    }
    return winding;
}

void clearRoute(const ot::TrackDocument &document) {
    const auto points = ot::routePolyline(document);
    const auto &gate = document.gates[0];
    const auto offset =
        QVector3D(static_cast<float>(gate.position.x()), static_cast<float>(gate.elevation),
                  static_cast<float>(gate.position.y()));
    for (qsizetype i = 1; i < points.size(); ++i)
        for (const auto &beam : ot::gateFrameBeams(gate))
            require(!intersectsBeam(points[i - 1] - offset, points[i] - offset, beam),
                    "configured compound traversal never cuts through a stock beam");
    qsizetype previous = -1;
    for (const auto &step : document.routeSteps) {
        const auto geometry = ot::routeStepGeometry(document, step);
        qsizetype found = -1;
        for (qsizetype i = previous + 1; i < points.size(); ++i)
            if (close(points[i], geometry.position)) {
                found = i;
                break;
            }
        require(found >= 0, "polyline retains each explicit pass and guide in order");
        if (step.kind == QStringLiteral("pass") && found > 0 && found + 1 < points.size()) {
            require(QVector3D::dotProduct(points[found] - points[found - 1], geometry.normal) > 0 &&
                        QVector3D::dotProduct(points[found + 1] - points[found], geometry.normal) >
                            0,
                    "pass approach and exit follow its independent traversal normal");
        }
        previous = found;
    }
}
} // namespace

int main(int argc, char **argv) {
    QCoreApplication app(argc, argv);
    ot::TrackDocument document;
    document.closedLoop = false;
    document.routeStepsDefined = true;
    auto ladder = ot::makeGate(QStringLiteral("ladder"));
    document.gates = {ladder};
    auto lower = pass(ladder), upper = pass(ladder, 0, 1, QStringLiteral("back"));
    auto guide = pass(ladder);
    guide.kind = QStringLiteral("guide");
    guide.localGuide = {0, 0, .8f};
    document.routeSteps = {lower, guide, upper};
    require(close(ot::routeStepPosition(document, lower), {0, 1.6f, 0}) &&
                close(ot::routeStepPosition(document, upper), {0, 4.6f, 0}),
            "repeated passes select lower and upper cells of one physical ladder");
    require(close(ot::routeStepNormal(document, lower), {0, 0, 1}) &&
                close(ot::routeStepNormal(document, upper), {0, 0, -1}),
            "lower/front and upper/back traversal preserve opposite crossing directions");
    clearRoute(document);
    const auto beamsBefore = ot::gateFrameBeams(document.gates[0]).size();
    document.routeSteps.append(lower);
    document.routeSteps.last().id = ot::newGateId();
    require(document.gates.size() == 1 &&
                ot::gateFrameBeams(document.gates[0]).size() == beamsBefore,
            "repeated route passes do not duplicate the physical obstacle or its stocks");
    document.routeSteps = {upper, guide, lower};
    guide.localGuide.setZ(-.8f);
    document.routeSteps[1] = guide;
    clearRoute(document);

    auto triple = ot::makeGate(QStringLiteral("triple"));
    document.gates = {triple};
    document.routeSteps = {pass(triple, 0), pass(triple, 1, 0, QStringLiteral("back")),
                           pass(triple, 2)};
    clearRoute(document);
    require(ot::gateFrameBeams(document.gates[0]).size() == 10,
            "serpentine three-cell traversal keeps one ten-stock assembly");

    auto cube = ot::makeGate(QStringLiteral("cube-stack"));
    document.gates = {cube};
    struct FaceCase {
        const char *name;
        QVector3D position, normal;
    };
    const FaceCase faces[] = {
        {"front", {0, 1.6f, -1.5f}, {0, 0, 1}}, {"back", {0, 1.6f, 1.5f}, {0, 0, -1}},
        {"left", {-1.5f, 1.6f, 0}, {1, 0, 0}},  {"right", {1.5f, 1.6f, 0}, {-1, 0, 0}},
        {"top", {0, 3.1f, 0}, {0, -1, 0}},      {"bottom", {0, .1f, 0}, {0, 1, 0}}};
    for (const auto &face : faces) {
        auto step = pass(cube, 0, 0, QString::fromLatin1(face.name));
        const auto geometry = ot::routeStepGeometry(document, step);
        require(geometry.valid && close(geometry.position, face.position) &&
                    close(geometry.normal, face.normal) && close(geometry.openingWidth, 2.9) &&
                    close(geometry.openingHeight, 2.9),
                "cube face exposes the correct cell plane, inward normal and clear opening");
        require(close(QVector3D::crossProduct(geometry.right, geometry.up), geometry.normal),
                "checkpoint plane basis is consistently oriented for native export");
        step.reverse = true;
        const auto reversed = ot::routeStepGeometry(document, step);
        require(close(reversed.position, geometry.position) &&
                    close(reversed.normal, -geometry.normal) &&
                    close(QVector3D::crossProduct(reversed.right, reversed.up), reversed.normal),
                "reversing traversal changes the normal without moving or rotating the frame");
    }
    auto front = pass(cube), roof = pass(cube, 0, 0, QStringLiteral("top"));
    roof.reverse = true;
    auto back = pass(cube, 0, 1, QStringLiteral("back"));
    back.reverse = true;
    document.routeSteps = {front, roof, back};
    clearRoute(document);
    require(close(ot::routeStepNormal(document, roof), {0, 1, 0}),
            "gravity traversal can exit vertically through a selected roof face");

    auto anchored = pass(cube);
    anchored.kind = QStringLiteral("guide");
    anchored.localGuide = {1, 1, -1};
    document.gates[0].position = {10, 20};
    document.gates[0].yaw = 90;
    document.gates[0].width = 4;
    document.gates[0].height = 5;
    document.gates[0].elevation = 5.1;
    require(close(ot::routeStepPosition(document, anchored), {8, 10.1f, 18}),
            "normalized guide recomputes after anchor translation, yaw and dimension edits");
    require(close(ot::routeStepNormal(document, front), {1, 0, 0}),
            "traversal normal follows the unchanged physical yaw convention");

    for (const auto &shape : {QStringLiteral("ladder"), QStringLiteral("triple"),
                              QStringLiteral("cube"), QStringLiteral("cube-stack")}) {
        ot::TrackDocument maneuver;
        maneuver.closedLoop = false;
        maneuver.routeStepsDefined = true;
        maneuver.gates = {ot::makeGate(shape)};
        const QStringList patterns =
            shape == QStringLiteral("ladder")   ? QStringList{"lower-upper", "upper-lower"}
            : shape == QStringLiteral("triple") ? QStringList{"left-right", "right-left"}
                                                : QStringList{"gravity-dive", "gravity-climb"};
        for (const auto &pattern : patterns) {
            maneuver.routeSteps = ot::gateManeuverSteps(maneuver.gates[0], pattern);
            require(maneuver.routeSteps.size() >= 2,
                    "explicit correction pattern creates a multi-face/cell maneuver");
            clearRoute(maneuver);
            for (const auto &point : ot::routePolyline(maneuver))
                require(point.y() >= 0,
                        "nominal manual maneuvers keep their approach and exit above the floor");
        }
    }
    require(ot::gateManeuverSteps(ladder, QStringLiteral("gravity-dive")).isEmpty(),
            "a planar ladder cannot silently become a cube gravity maneuver");
    for (const auto &pattern :
         {QStringLiteral("upright-left-up"), QStringLiteral("upright-left-down"),
          QStringLiteral("upright-right-up"), QStringLiteral("upright-right-down")}) {
        ot::TrackDocument progressive;
        progressive.closedLoop = false;
        progressive.routeStepsDefined = true;
        auto frame = ot::makeGate(QStringLiteral("ladder"));
        frame.position = {-4, 6};
        frame.yaw = 37;
        progressive.gates = {frame};
        progressive.routeSteps = ot::gateManeuverSteps(frame, pattern);
        const auto curve = ot::routeCurve(progressive);
        require(curve.clearanceSatisfied && curve.smooth && curve.breaks.isEmpty() &&
                    curve.points.size() > 100,
                "progressive upright helix is dense, smooth and stock-clear");
        denseClearance(progressive, curve);
        QVector<ot::RouteStep> passes;
        int turnChecks = 0;
        for (const auto &step : progressive.routeSteps) {
            if (step.kind == "pass")
                passes.append(step);
            turnChecks += step.extra.value("raceCheckpoint").toBool();
        }
        require(passes.size() == 2 && passes[0].level != passes[1].level && turnChecks == 1,
                "advancing helix crosses two different apertures with no lower-return visit");
        const auto start = ot::routeStepGeometry(progressive, passes[0]);
        const auto end = ot::routeStepGeometry(progressive, passes[1]);
        require(start.valid && end.valid &&
                    QVector3D::dotProduct(start.normal, end.normal) > .99f &&
                    std::abs(start.pathTangent.y()) > .2f && std::abs(end.pathTangent.y()) > .2f,
                "upright advance retains same-side portal normals and climbing or descending "
                "flight tangents");
        qsizetype first = -1, last = -1;
        for (qsizetype i = 0; i < curve.points.size(); ++i) {
            if (close(curve.points[i], start.pathPosition))
                first = i;
            if (close(curve.points[i], end.pathPosition))
                last = i;
        }
        const int advance = pattern.endsWith("up") ? 1 : -1;
        double winding = 0, previousAngle = 0;
        const auto inverse = QQuaternion::fromAxisAndAngle(0, 1, 0, 37).conjugated();
        const QVector3D origin(-4, static_cast<float>(frame.elevation), 6);
        const double postX =
            pattern.startsWith("upright-left") ? -frame.width / 2 : frame.width / 2;
        require(first >= 0 && last > first &&
                    close(end.pathPosition.y() - start.pathPosition.y(), advance * frame.height),
                "helix advances one full layer while winding, rather than returning before rising");
        for (qsizetype i = first; i <= last && first >= 0; ++i) {
            const auto point = inverse.rotatedVector(curve.points[i] - origin);
            const double angle = std::atan2(point.z(), point.x() - postX);
            if (i > first) {
                winding += std::remainder(angle - previousAngle, 2 * 3.14159265358979323846);
                require(
                    (curve.points[i].y() - curve.points[i - 1].y()) * advance >= -1e-5,
                    "axial height progresses throughout the wrap without a return-and-rise stage");
            }
            previousAngle = angle;
        }
        require(std::abs(winding - (pattern.startsWith("upright-left") ? 1 : -1) * 2 *
                                       3.14159265358979323846) < .03,
                "continuous advance winds the chosen physical upright in the documented direction");
    }

    ot::TrackDocument crossing;
    crossing.closedLoop = false;
    crossing.routeStepsDefined = true;
    crossing.gates = {ot::makeGate()};
    auto offsetPass = pass(crossing.gates[0]);
    offsetPass.extra.insert("apertureU", -.4);
    offsetPass.extra.insert("apertureV", .25);
    offsetPass.extra.insert("flightTangent", QJsonArray{.3, .4, 1});
    crossing.routeSteps = {offsetPass};
    const auto offsetGeometry = ot::routeStepGeometry(crossing, offsetPass);
    require(
        offsetGeometry.valid && close(offsetGeometry.position, {0, 1.6f, 0}) &&
            close(offsetGeometry.pathPosition, {-.58f, 1.9625f, 0}) &&
            offsetGeometry.pathTangent.y() > .2f,
        "off-centre oblique path crossing leaves the native checkpoint window centred and upright");
    const auto offsetCurve = ot::routeCurve(crossing);
    require(offsetCurve.clearanceSatisfied && offsetCurve.breaks.isEmpty(),
            "oblique aperture crossing remains stock-clear");
    denseClearance(crossing, offsetCurve);
    bool containsCrossing = false;
    for (const auto &point : offsetCurve.points)
        containsCrossing |= close(point, offsetGeometry.pathPosition);
    require(containsCrossing, "the rendered path retains the observed off-centre crossing exactly");
    offsetPass.reverse = true;
    offsetPass.extra.insert("flightTangent", QJsonArray{-.3, -.4, -1});
    const auto reverseOffset = ot::routeStepGeometry(crossing, offsetPass);
    require(reverseOffset.valid && close(reverseOffset.position, offsetGeometry.position) &&
                close(reverseOffset.pathPosition, offsetGeometry.pathPosition) &&
                close(reverseOffset.pathTangent, -offsetGeometry.pathTangent),
            "reversing signed flight direction does not mirror the physical aperture offset");
    auto badOffset = crossing;
    badOffset.routeSteps[0].extra.insert("apertureU", .99);
    ot::TrackDocument unchanged = crossing;
    QString crossingError;
    require(!ot::fromJson(ot::toJson(badOffset), &unchanged, &crossingError) &&
                ot::toJson(unchanged) == ot::toJson(crossing),
            "offsets that consume drone clearance are rejected without mutating the destination");
    badOffset = crossing;
    badOffset.routeSteps[0].extra.insert("flightTangent", QJsonArray{0, 0, -1});
    require(!ot::fromJson(ot::toJson(badOffset), &unchanged, &crossingError),
            "flight tangents pointing against the accepted checkpoint hemisphere are rejected");

    for (const auto &shape : {QStringLiteral("standard"), QStringLiteral("ladder")})
        for (const auto &pattern :
             {QStringLiteral("orbit-bar-cw"), QStringLiteral("orbit-bar-ccw")})
            for (const double yaw : {0.0, 37.0, 135.0}) {
                ot::TrackDocument orbit;
                orbit.closedLoop = false;
                orbit.routeStepsDefined = true;
                auto frame = ot::makeGate(shape);
                frame.position = {-4, 6};
                frame.yaw = yaw;
                orbit.gates = {frame};
                orbit.routeSteps = ot::gateManeuverSteps(frame, pattern);
                const auto curve = ot::routeCurve(orbit);
                require(curve.clearanceSatisfied && curve.smooth && curve.breaks.isEmpty() &&
                            curve.points.size() > 100,
                        "full horizontal-bar orbit is a dense smooth clearance-preserving curve");
                const double winding = barWinding(
                    frame, curve.points, shape == QStringLiteral("ladder") ? 0 : frame.height / 2);
                require(std::abs(winding - (pattern.endsWith("ccw") ? 1 : -1) * 2 *
                                               3.14159265358979323846) < .02,
                        "CW and CCW complete opposite full turns in the documented local+X view");
                require(close(curve.points.first(), curve.points.last()),
                        "full orbit closes at the exact aperture without a diagonal shortcut");
                denseClearance(orbit, curve);
                int passes = 0, checks = 0;
                qsizetype previous = -1;
                for (const auto &step : orbit.routeSteps) {
                    passes += step.kind == QStringLiteral("pass");
                    checks += step.extra.value(QStringLiteral("raceCheckpoint")).toBool(false);
                    const auto geometry = ot::routeStepGeometry(orbit, step);
                    qsizetype index = -1;
                    for (qsizetype i = previous + 1; i < curve.points.size(); ++i)
                        if (close(curve.points[i], geometry.position)) {
                            index = i;
                            break;
                        }
                    require(index >= 0,
                            "dense orbit retains all ordered portal and arc control anchors");
                    if (step.kind == QStringLiteral("pass") && index >= 0) {
                        const auto before =
                            curve.points[index > 0 ? index - 1 : curve.points.size() - 2];
                        const auto after =
                            curve.points[index + 1 < curve.points.size() ? index + 1 : 1];
                        require(QVector3D::dotProduct((geometry.position - before).normalized(),
                                                      geometry.normal) > .995 &&
                                    QVector3D::dotProduct((after - geometry.position).normalized(),
                                                          geometry.normal) > .995,
                                "both sides of each exact orbit aperture align with its traversal "
                                "normal");
                    }
                    if (step.extra.value(QStringLiteral("raceCheckpoint")).toBool(false))
                        require(geometry.valid && geometry.normal.lengthSquared() > .99f &&
                                    close(geometry.openingWidth, .8) &&
                                    close(geometry.openingHeight, .8),
                                "explicit auxiliary turn checks have valid tangent planes and "
                                "small openings");
                    previous = index;
                }
                require(passes == (shape == QStringLiteral("ladder") ? 3 : 2) &&
                            checks == (shape == QStringLiteral("ladder") ? 2 : 3),
                        "full turn uses real aperture visits plus explicit side controls, without "
                        "fake upper gates");
                auto backgroundOnly = orbit;
                backgroundOnly.background = QImage(31, 17, QImage::Format_RGB32);
                backgroundOnly.background.fill(Qt::green);
                backgroundOnly.showBackground = true;
                backgroundOnly.name = QStringLiteral("unrelated editor metadata");
                require(
                    ot::routeCurve(backgroundOnly).points == curve.points,
                    "route geometry is independent of background pixels and unrelated metadata");
            }

    for (const auto &shape : {QStringLiteral("ladder"), QStringLiteral("triple"),
                              QStringLiteral("cube"), QStringLiteral("cube-stack")}) {
        ot::TrackDocument rounded;
        rounded.closedLoop = false;
        rounded.routeStepsDefined = true;
        rounded.gates = {ot::makeGate(shape)};
        const QString pattern = shape == "ladder"   ? "lower-upper"
                                : shape == "triple" ? "left-right"
                                                    : "gravity-climb";
        rounded.routeSteps = ot::gateManeuverSteps(rounded.gates[0], pattern);
        const auto curve = ot::routeCurve(rounded);
        require(curve.clearanceSatisfied && curve.smooth && curve.breaks.isEmpty() &&
                    curve.points.size() > 50,
                "half-wrap, serpent and gravity maneuvers are rounded with no clearance fallback");
        denseClearance(rounded, curve);
    }

    ot::TrackDocument blocked;
    blocked.closedLoop = false;
    blocked.routeStepsDefined = true;
    blocked.gates = {ot::makeGate(QStringLiteral("pillar"))};
    auto freeA = guide, freeB = guide;
    freeA.id = ot::newGateId();
    freeB.id = ot::newGateId();
    freeA.gateId.clear();
    freeB.gateId.clear();
    freeA.localGuide = {-3, 1.6f, -1};
    freeB.localGuide = {3, 1.6f, 1};
    blocked.routeSteps = {freeA, freeB};
    const auto detoured = ot::routeCurve(blocked);
    require(detoured.clearanceSatisfied && detoured.breaks.isEmpty() && detoured.smooth &&
                detoured.warnings.isEmpty() && detoured.points.size() > 10,
            "blocked arbitrary join rounds the bounded clear corridor instead of exposing polygon "
            "corners");
    denseClearance(blocked, detoured);
    for (const auto &point : detoured.points)
        require(std::abs(point.y() - freeA.localGuide.y()) <= .2f,
                "rounding a flat obstacle bypass does not invent an unrelated climb or dive");
    require(close(detoured.points.first(), freeA.localGuide) &&
                close(detoured.points.last(), freeB.localGuide),
            "rounding planner corners does not move actual guide endpoints");
    double largestCorner = 0;
    for (qsizetype i = 1; i + 1 < detoured.points.size(); ++i) {
        const auto before = detoured.points[i] - detoured.points[i - 1];
        const auto after = detoured.points[i + 1] - detoured.points[i];
        if (before.lengthSquared() > 1e-10f && after.lengthSquared() > 1e-10f)
            largestCorner =
                std::max(largestCorner,
                         std::acos(std::clamp(
                             double(QVector3D::dotProduct(before.normalized(), after.normalized())),
                             -1., 1.)));
    }
    require(largestCorner < .35,
            "rounded obstacle bypass has no sharp jump between its rendered segments");
    const auto approach = (freeB.localGuide - freeA.localGuide).normalized();
    require(
        QVector3D::dotProduct((detoured.points[1] - detoured.points[0]).normalized(), approach) >
                .999f &&
            QVector3D::dotProduct(
                (detoured.points.last() - detoured.points[detoured.points.size() - 2]).normalized(),
                approach) > .999f,
        "safe detour preserves the shared endpoint approach and departure tangents");
    ot::TrackDocument componentJoin;
    componentJoin.closedLoop = false;
    componentJoin.routeStepsDefined = true;
    componentJoin.gates = {ot::makeGate(), ot::makeGate()};
    componentJoin.gates[0].position = {-6, 0};
    componentJoin.gates[1].position = {6, 0};
    const auto fittedGuide = [&](int owner, QVector3D position, QVector3D tangent) {
        ot::RouteStep step;
        step.id = ot::newGateId();
        step.kind = "guide";
        step.gateId = componentJoin.gates[owner].id;
        step.localGuide = (position - QVector3D(owner ? 6.f : -6.f, 1.6f, 0)) / 1.5f;
        step.extra.insert("curveMode", "arc-v1");
        step.extra.insert("curveTangent", QJsonArray{tangent.x(), tangent.y(), tangent.z()});
        return step;
    };
    auto sourceBefore = fittedGuide(0, {-3, 1, -1}, {.5f, -.65f, -.2f});
    auto sourceEnd = fittedGuide(0, {-2.5f, .35f, -1.2f}, {-1, 0, -1});
    auto nextComponent = fittedGuide(1, {3, 1, 3}, {5.5f, .65f, 4.2f});
    sourceEnd.extra.insert("recognition", QJsonObject{{"source", "synthetic-glyph-trace"}});
    componentJoin.routeSteps = {sourceBefore, sourceEnd, nextComponent};
    const auto outgoingAt = [&](const ot::TrackDocument &track, const ot::RouteStep &step) {
        const auto crossing = ot::routeStepGeometry(track, step).pathPosition;
        const auto curve = ot::routeCurve(track);
        for (qsizetype i = 0; i + 1 < curve.points.size(); ++i)
            if (close(curve.points[i], crossing))
                return (curve.points[i + 1] - curve.points[i]).normalized();
        return QVector3D();
    };
    require(outgoingAt(componentJoin, sourceEnd).x() > .995f,
            "recognized component-end guide uses the next component and local extrema instead "
            "of reversing along an intra-glyph derivative");
    denseClearance(componentJoin, ot::routeCurve(componentJoin));
    auto manualJoin = componentJoin;
    manualJoin.routeSteps[1].extra.remove("recognition");
    const auto manualDirection =
        ot::routeStepGeometry(manualJoin, manualJoin.routeSteps[1]).pathTangent;
    require(QVector3D::dotProduct(outgoingAt(manualJoin, manualJoin.routeSteps[1]),
                                  manualDirection) > .99f,
            "ordinary manual guide headings remain intentional and cached provenance is distinct");
    auto protectedJoin = componentJoin;
    protectedJoin.routeSteps[1].extra.insert("raceCheckpoint", true);
    const auto protectedDirection =
        ot::routeStepGeometry(protectedJoin, protectedJoin.routeSteps[1]).pathTangent;
    require(QVector3D::dotProduct(outgoingAt(protectedJoin, protectedJoin.routeSteps[1]),
                                  protectedDirection) > .99f,
            "an explicit T checkpoint keeps its accepted heading at a component boundary");
    auto protectedPass = componentJoin;
    auto physicalPass = pass(protectedPass.gates[0]);
    physicalPass.extra.insert("curveMode", "arc-v1");
    physicalPass.extra.insert("recognition", QJsonObject{{"source", "synthetic-glyph-trace"}});
    protectedPass.routeSteps[1] = physicalPass;
    require(QVector3D::dotProduct(outgoingAt(protectedPass, physicalPass),
                                  ot::routeStepGeometry(protectedPass, physicalPass).pathTangent) >
                .99f,
            "an actual aperture crossing never blends away its required entry direction");
    ot::TrackDocument roofApproach;
    roofApproach.closedLoop = false;
    roofApproach.routeStepsDefined = true;
    roofApproach.gates = {ot::makeGate("cube")};
    auto approachGuide = freeA;
    approachGuide.id = ot::newGateId();
    approachGuide.localGuide = {-10, 1.6f, 0};
    approachGuide.extra.insert("curveTangent", QJsonArray{1, 0, 0});
    roofApproach.routeSteps = {approachGuide, pass(roofApproach.gates[0], 0, 0, "top")};
    const auto roofCurve = ot::routeCurve(roofApproach);
    require(roofCurve.clearanceSatisfied && roofCurve.breaks.isEmpty(),
            "a lateral approach reaches the roof portal without crossing its edge beams");
    denseClearance(roofApproach, roofCurve);
    for (const auto &point : roofCurve.points)
        require(point.y() <= 3.66f,
                "roof entry smoothing stays near the roof instead of adding a high arch");
    blocked.gates.clear();
    blocked.routeSteps.clear();
    for (const auto &point :
         {QVector3D(-3, 1.6f, -1), QVector3D(-1, 1.6f, -1), QVector3D(0, .05f, 0),
          QVector3D(1, 1.6f, -1), QVector3D(3, 1.6f, -1)}) {
        auto step = freeA;
        step.id = ot::newGateId();
        step.localGuide = point;
        blocked.routeSteps.append(step);
    }
    const auto segmented = ot::routeCurve(blocked);
    require(!segmented.clearanceSatisfied && !segmented.breaks.isEmpty() &&
                !segmented.points.isEmpty(),
            "impossible ground anchor preserves safe strokes with explicit gaps instead of a false "
            "bridge");
    denseClearance(blocked, segmented);
    require(ot::routePolyline(blocked).size() == segmented.breaks.first(),
            "compatibility polyline stops before the first gap");

    auto invalidHint = freeA;
    invalidHint.extra.insert(QStringLiteral("curveMode"), QStringLiteral("arc-v1"));
    invalidHint.extra.insert(QStringLiteral("curveTangent"), QJsonArray{1e100, 0, 0});
    blocked.routeSteps = {invalidHint, freeB};
    const auto ignoredHint = ot::routeCurve(blocked);
    require(!ignoredHint.warnings.isEmpty() && ignoredHint.points.size() < 200000,
            "crafted huge tangent hints are ignored safely without unbounded sampling");
    denseClearance(blocked, ignoredHint);

    ot::TrackDocument vertical;
    vertical.closedLoop = false;
    vertical.routeStepsDefined = true;
    auto a = anchored, b = anchored;
    a.id = ot::newGateId();
    b.id = ot::newGateId();
    a.gateId.clear();
    b.gateId.clear();
    a.localGuide = {2, 1, 3};
    b.localGuide = {2, 6, 3};
    vertical.routeSteps = {a, b};
    require(close(ot::routeLength(vertical), 5),
            "explicit route length includes vertical travel between guide points");
    vertical.takeoff.enabled = true;
    vertical.takeoff.position = {-7, 12};
    vertical.takeoff.elevation = .2;
    vertical.takeoff.yaw = 135;
    vertical.takeoff.extra.insert(QStringLiteral("nativeGrid"), 422);
    require(close(ot::routeLength(vertical), 5) && ot::effectiveRouteSteps(vertical).size() == 2,
            "independent takeoff does not become a lap checkpoint or landing instruction");

    QString error;
    ot::TrackDocument restored;
    require(ot::fromJson(ot::toJson(vertical), &restored, &error) &&
                ot::toJson(restored) == ot::toJson(vertical),
            "optional v1 route and takeoff records preserve geometry and native extras");
    auto legacy = ot::toJson(document);
    legacy.remove(QStringLiteral("routeSteps"));
    legacy.remove(QStringLiteral("takeoff"));
    require(ot::fromJson(legacy, &restored, &error) && !restored.routeStepsDefined &&
                ot::effectiveRouteSteps(restored).size() == restored.gates.size(),
            "v1 files without route steps retain the legacy gate-order fallback");
    document.routeSteps.clear();
    require(ot::fromJson(ot::toJson(document), &restored, &error) && restored.routeStepsDefined &&
                ot::routePath(restored).isEmpty() && ot::routePolyline(restored).isEmpty(),
            "an explicitly empty route remains empty despite existing physical gates");

    document.gates = {ladder};
    document.routeSteps = {lower, guide, upper};
    require(ot::fromJson(ot::toJson(document), &restored, &error), "valid ladder fixture loads");
    const auto baseline = ot::toJson(restored);
    const auto reject = [&](QJsonObject json, const char *description) {
        require(!ot::fromJson(json, &restored, &error) && ot::toJson(restored) == baseline,
                description);
    };
    for (const auto &field : {QStringLiteral("gateId"), QStringLiteral("face"),
                              QStringLiteral("column"), QStringLiteral("level")}) {
        auto json = baseline;
        auto steps = json.value(QStringLiteral("routeSteps")).toArray();
        auto step = steps[0].toObject();
        step.insert(field, field == QStringLiteral("gateId") ? QJsonValue("missing")
                           : field == QStringLiteral("face") ? QJsonValue("top")
                                                             : QJsonValue(4));
        steps[0] = step;
        json.insert(QStringLiteral("routeSteps"), steps);
        reject(json, "invalid route references, unsupported faces and cells fail atomically");
    }
    auto duplicate = baseline;
    auto steps = duplicate.value(QStringLiteral("routeSteps")).toArray();
    steps.append(steps[0]);
    duplicate.insert(QStringLiteral("routeSteps"), steps);
    reject(duplicate, "duplicate route step IDs fail atomically");
    auto malformed = baseline;
    steps = malformed.value(QStringLiteral("routeSteps")).toArray();
    auto changed = steps[1].toObject();
    changed.insert(QStringLiteral("localGuide"), QJsonArray{0, "NaN", 0});
    steps[1] = changed;
    malformed.insert(QStringLiteral("routeSteps"), steps);
    reject(malformed, "non-numeric guide coordinates fail atomically");
    auto invalidTakeoff = baseline;
    auto takeoff = invalidTakeoff.value(QStringLiteral("takeoff")).toObject();
    takeoff.insert(QStringLiteral("elevation"), "NaN");
    invalidTakeoff.insert(QStringLiteral("takeoff"), takeoff);
    reject(invalidTakeoff, "invalid takeoff numeric fields fail atomically");

    QTemporaryDir temporary;
    const auto path = temporary.filePath(QStringLiteral("route.otproj"));
    require(ot::saveProject(path, document, &error), "valid route saves atomically");
    QFile file(path);
    file.open(QIODevice::ReadOnly);
    const auto bytes = file.readAll();
    file.close();
    document.routeSteps[1].localGuide.setX(std::numeric_limits<float>::infinity());
    require(!ot::saveProject(path, document, &error), "non-finite guide coordinates reject save");
    file.open(QIODevice::ReadOnly);
    require(file.readAll() == bytes, "invalid route save preserves the committed project");
    if (!failures)
        std::cout << "PASS: physical obstacles, repeated ordered passes, face geometry, anchored "
                     "guides, hard portal crossings, takeoff and v1 atomic persistence.\n";
    return failures ? 1 : 0;
}
