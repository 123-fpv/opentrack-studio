#include "document.h"

#include <QCoreApplication>
#include <QFile>
#include <QJsonArray>
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

bool intersectsBeam(QVector3D a, QVector3D b, const ot::GateBeam &beam) {
    const auto low = beam.center - beam.size / 2, high = beam.center + beam.size / 2;
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
