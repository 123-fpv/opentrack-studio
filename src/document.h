#pragma once

#include <QImage>
#include <QJsonObject>
#include <QPainterPath>
#include <QPointF>
#include <QString>
#include <QVector>

namespace ot {

struct Gate {
    QString id;
    QString type = QStringLiteral("standard");
    QPointF position;
    double elevation = 1.2; // Gate centre altitude, in metres.
    double width = 2.4;     // Pillars use this same side length on both the X and Z axes.
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
QString gateTypeName(const QString &type);
QString newGateId();
// Pillar passages use a 2.4 m checkpoint lane beside the solid post, with 0.35 m
// clearance. extra.passSide selects local right (+1, default) or local left (-1).
QPointF gatePassagePosition(const Gate &gate);
double routeLength(const TrackDocument &document);
QPainterPath routePath(const TrackDocument &document);

} // namespace ot
