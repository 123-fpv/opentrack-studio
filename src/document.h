#pragma once

#include <QColor>
#include <QImage>
#include <QJsonObject>
#include <QPainterPath>
#include <QPointF>
#include <QString>
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

struct TrackDocument {
    QString name = QStringLiteral("未命名赛道");
    QString scenery = QStringLiteral("Football Stadium");
    QVector<Gate> gates;
    QImage background;
    double metersPerPixel = 0.05;
    QPointF backgroundOrigin = QPointF(-30.0, -20.0);
    double backgroundOpacity = 0.35;
    bool showBackground = true;
    double gridSize = 1.0;
    bool closedLoop = true;
    QJsonObject extra;
};

// OpenTrackStudio .otproj v1: UTF-8 JSON with metres, centre altitude and yaw in degrees,
// gates in traversal order, an optional embedded PNG and extensible extra objects.
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
    double centerWidth = 0, centerHeight = 0;
    double beamThickness = 0;
    double horizontalBeamLength = 0, verticalBeamLength = 0;
    double openingWidth = 0, openingHeight = 0;
    double outerWidth = 0, outerHeight = 0;
};

struct GateBeam {
    QVector3D center; // Local coordinates, before gate yaw/position/altitude.
    QVector3D size;
};

// NEW frames use extra.frameGeometry="centerline-v1", beamThickness=0.1 and
// beamOverhang=0.2 TOTAL extra length (0.1 m at each end). Unmarked gates keep
// their legacy bar dimensions. width/height values are never rescaled on load.
GateFrameMetrics gateFrameMetrics(const Gate &gate);
QVector<GateBeam> gateFrameBeams(const Gate &gate);
// Negative/zero dimensions select type defaults: frame centreline 3 x 3 m,
// pillar square section 0.1 m and height 3.2 m. The lowest beam starts at Y=0.
Gate makeGate(const QString &type = QStringLiteral("standard"), double width = -1,
              double height = -1);

// Pillar passages use a 2.4 m checkpoint lane beside the solid post, with 0.35 m
// clearance. extra.passSide selects local right (+1, default) or local left (-1).
QPointF gatePassagePosition(const Gate &gate);
double routeLength(const TrackDocument &document);
QPainterPath routePath(const TrackDocument &document);

} // namespace ot
