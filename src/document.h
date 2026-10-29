#pragma once

#include <QColor>
#include <QImage>
#include <QJsonObject>
#include <QPainterPath>
#include <QPointF>
#include <QString>
#include <QStringList>
#include <QVector3D>
#include <QVector>

namespace ot {

struct Gate {
    QString id;
    QString type = QStringLiteral("standard");
    QPointF position;
    // Raw defaults preserve old projects. Use makeGate() for newly created components.
    double elevation = 1.2; // Gate centre altitude, in metres.
    double width = 2.4;     // New frames: beam centreline spacing; pillars: square X/Z side length.
    double height = 2.4;
    double yaw = 0.0; // Degrees: 0 faces +Z (position.y), 90 faces +X (position.x).
    QString label;
    QJsonObject extra;
};

struct RouteStep {
    QString id;
    QString gateId; // Physical component reference; repeated passes share this ID.
    QString kind =
        QStringLiteral("pass"); // Guides become auxiliary checks only with extra.raceCheckpoint.
    QString face = QStringLiteral("front"); // front/back/left/right/top/bottom, in gate-local axes.
    int column = 0, level = 0;              // Zero-based cell, independent for every pass.
    bool reverse = false; // Reverse traversal, without rotating the physical gate.
    // Anchored guides use normalized group half-centreline extents (X/Y/Z).
    // With no gateId, this is instead an absolute world point in metres.
    QVector3D localGuide;
    QString label;
    QJsonObject extra;
};

struct TakeoffPoint {
    bool enabled = false;
    QPointF position; // Ground X/Z; the editor displays X/Y.
    double elevation = 0, yaw = 0;
    QJsonObject extra;
};

struct TrackDocument {
    QString name = QStringLiteral("未命名赛道");
    QString scenery = QStringLiteral("Football Stadium");
    QVector<Gate> gates;
    QVector<RouteStep> routeSteps;
    // An explicitly empty route must not fall back to physical gate order.
    bool routeStepsDefined = false;
    TakeoffPoint takeoff; // Independent of route steps, gate order and finish/landing behavior.
    QImage background;
    double metersPerPixel = 0.05;
    QPointF backgroundOrigin = QPointF(-30.0, -20.0);
    double backgroundOpacity = 0.35;
    bool showBackground = false;
    double gridSize = 1.0;
    bool closedLoop = true;
    QJsonObject extra;
};

// OpenTrackStudio .otproj v1: UTF-8 JSON with metres, centre altitude and yaw in degrees,
// physical gates, optional ordered routeSteps/takeoff, an embedded PNG and extra objects.
// Missing routeSteps preserves the legacy gate-order route. No landing is implied.
QJsonObject toJson(const TrackDocument &document);
bool fromJson(const QJsonObject &object, TrackDocument *document, QString *error = nullptr);
bool saveProject(const QString &path, const TrackDocument &document, QString *error = nullptr);
bool loadProject(const QString &path, TrackDocument *document, QString *error = nullptr);

TrackDocument demoDocument();
// Optional opaque per-component colour, stored as canonical #RRGGBB in extra.color.
// Missing colours retain the neutral palette; this is an editor colour and does
// not imply an arbitrary RGB material is available in the simulator.
QColor gateColor(const Gate &gate);
bool setGateColor(Gate &gate, const QColor &color, QString *error = nullptr);
void resetGateColor(Gate &gate);
QString gateTypeName(const QString &type);
QString newGateId();

struct GateFrameMetrics {
    bool centerlineModel = false;
    int columns = 1, levels = 1;
    double centerWidth = 0, centerHeight = 0;
    double centerDepth = 0;
    double beamThickness = 0;
    double horizontalBeamLength = 0, verticalBeamLength = 0;
    double openingWidth = 0, openingHeight = 0;
    double outerWidth = 0, outerHeight = 0, outerDepth = 0;
};

struct GateBeam {
    QVector3D center; // Local coordinates, before gate yaw/position/altitude.
    QVector3D size;
};

// NEW frames use extra.frameGeometry="centerline-v1", beamThickness=0.1 and
// beamOverhang=0.2 TOTAL extra length (0.1 m at each end). Unmarked gates keep
// their legacy bar dimensions. width/height values are never rescaled on load.
// Marked compound frames use PER-CELL centreline width/height: double/triple
// have 2/3 columns, ladder has 2 levels, quad has 2 columns and 2 levels,
// ladder-blocked has 2 levels with a solid lower panel, cube has four walls at
// depth=width, and cube-stack has 2 cube levels. Shared stocks are not duplicated.
GateFrameMetrics gateFrameMetrics(const Gate &gate);
// False for out-of-group cells and solid blocked apertures. The lower level of a
// ladder-blocked frame is physical obstacle geometry, never a race checkpoint.
bool gateCellOpen(const Gate &gate, int column, int level);
QVector<GateBeam> gateFrameBeams(const Gate &gate);
// Negative/zero dimensions select type defaults: frame centreline 3 x 3 m,
// pillar square section 0.1 m and height 3.2 m. The lowest beam starts at Y=0.
Gate makeGate(const QString &type = QStringLiteral("standard"), double width = -1,
              double height = -1);

// Pillar passages use a 2.4 m checkpoint lane beside the solid post, with 0.35 m
// clearance. extra.passSide selects local right (+1, default) or local left (-1).
// Marked groups select the leftmost/lower cell by default; optional zero-based
// extra.passColumn/passLevel choose another cell; ladder-blocked defaults to the
// upper open cell. Cube passages use its front
// wall at local Z=-width/2. Position and altitude describe the WHOLE group centre.
QPointF gatePassagePosition(const Gate &gate);
double gatePassageElevation(const Gate &gate);

struct RouteStepGeometry {
    bool valid = false;
    QVector3D position, normal, right, up; // World coordinates; right x up = traversal normal.
    // The physical checkpoint plane above stays centred on its aperture. Path-only
    // offsets/tangents preserve observed oblique crossings without moving native triggers.
    QVector3D pathPosition, pathTangent;
    double openingWidth = 0, openingHeight = 0;
};

// Front/back are available on planar frames. Cubes also expose left/right/top/bottom
// of the selected cell. Default face normals point into that cell; reverse flips
// normal/right. Guide positions follow their anchor's movement, yaw and dimensions.
// Optional pass extra.apertureU/V are normalized half-opening offsets in the
// face-local right/up basis BEFORE reverse. They must retain the preview clearance.
// extra.flightTangent is a signed direction in normalized gate-anchor axes; its
// world direction must lie in the accepted normal hemisphere. Missing hints use
// the physical aperture centre/normal. Reverse editing must reverse a saved tangent.
RouteStepGeometry routeStepGeometry(const TrackDocument &document, const RouteStep &step);
QVector3D routeStepPosition(const TrackDocument &document, const RouteStep &step);
QVector3D routeStepNormal(const TrackDocument &document, const RouteStep &step);
QVector<RouteStep> effectiveRouteSteps(const TrackDocument &document);
// Explicit user-selected correction patterns; this does not infer a maneuver from a symbol.
// Unsupported shape/pattern combinations return an empty vector.
// upright-{left,right}-{up,down} winds a vertical local-Y post while continuously
// advancing between two distinct levels; it never returns to the initial aperture.
QVector<RouteStep> gateManeuverSteps(const Gate &gate, const QString &pattern);
// CW/CCW bar orbits are viewed along local +X, with +Z right and +Y up.
// Arc hints use extra.curveMode="arc-v1" and finite extra.curveTangent vectors
// in the same normalized anchor axes as localGuide. Full-circle auxiliary checks
// explicitly set raceCheckpoint=true and checkpointWidth/checkpointHeight (metres).
struct RouteCurveResult {
    QVector<QVector3D> points;
    QVector<int> breaks; // Each index starts a new disconnected, collision-free stroke.
    QStringList warnings;
    bool clearanceSatisfied = true;
    bool smooth = true;
};
double routePreviewClearance();
RouteCurveResult routeCurve(const TrackDocument &document);
// Explicit routes retain every path crossing and its flight tangent. Constrained
// local curves are checked against stock beams and ground clearance. If no safe
// join exists, diagnostics retain disconnected safe strokes. This compatibility
// API returns only the first stroke; use routeCurve to retrieve all safe strokes.
// The independent takeoff point is not a lap checkpoint or an implicit route step.
QVector<QVector3D> routePolyline(const TrackDocument &document);
double routeLength(const TrackDocument &document);
QPainterPath routePath(const TrackDocument &document);

} // namespace ot
