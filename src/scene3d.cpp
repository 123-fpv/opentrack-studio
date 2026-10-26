#include "scene3d.h"
#include "document.h"

#include <QElapsedTimer>
#include <QGuiApplication>
#include <QHash>
#include <QKeyEvent>
#include <QLinearGradient>
#include <QMatrix4x4>
#include <QMouseEvent>
#include <QOpenGLBuffer>
#include <QOpenGLContext>
#include <QOpenGLShaderProgram>
#include <QPainter>
#include <QPainterPath>
#include <QPolygonF>
#include <QQuaternion>
#include <QResizeEvent>
#include <QSurfaceFormat>
#include <QTimer>
#include <QVector3D>
#include <QVector4D>
#include <QWheelEvent>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

namespace ot {

QVector3D editorAxisDirection(Qt::Axis editorAxis) {
    switch (editorAxis) {
    case Qt::XAxis:
        return {1, 0, 0};
    case Qt::YAxis:
        return {0, 0, 1};
    case Qt::ZAxis:
        return {0, 1, 0};
    }
    return {};
}

namespace {
constexpr double Pi = 3.14159265358979323846;
constexpr float FieldOfView = 42.0f;
struct Vertex {
    float x, y, z, r, g, b;
};
struct Batch {
    std::vector<Vertex> vertices;
    QOpenGLBuffer buffer{QOpenGLBuffer::VertexBuffer};
    GLenum mode = GL_TRIANGLES;
    float opacity = 1;
};

QVector3D point3(const Gate &gate) {
    return {static_cast<float>(gate.position.x()), static_cast<float>(gate.elevation),
            static_cast<float>(gate.position.y())};
}

float selectionHaloRadius(const Gate &gate) {
    return static_cast<float>(std::max(1.1, gate.width * .72));
}

Vertex vertex(const QVector3D &p, const QColor &color, float shade = 1) {
    return {p.x(),
            p.y(),
            p.z(),
            static_cast<float>(color.redF()) * shade,
            static_cast<float>(color.greenF()) * shade,
            static_cast<float>(color.blueF()) * shade};
}

void triangle(Batch &batch, const QVector3D &a, const QVector3D &b, const QVector3D &c,
              const QColor &color, float shade = 1) {
    batch.vertices.push_back(vertex(a, color, shade));
    batch.vertices.push_back(vertex(b, color, shade));
    batch.vertices.push_back(vertex(c, color, shade));
}

void quad(Batch &batch, const QVector3D &a, const QVector3D &b, const QVector3D &c,
          const QVector3D &d, const QColor &color, float shade = 1) {
    triangle(batch, a, b, c, color, shade);
    triangle(batch, a, c, d, color, shade);
}

void line(Batch &batch, const QVector3D &a, const QVector3D &b, const QColor &color) {
    batch.vertices.push_back(vertex(a, color));
    batch.vertices.push_back(vertex(b, color));
}

QVector3D gatePoint(const Gate &gate, const QVector3D &local) {
    QQuaternion rotation = QQuaternion::fromAxisAndAngle(0, 1, 0, static_cast<float>(gate.yaw));
    if (gate.type == QStringLiteral("dive"))
        rotation *= QQuaternion::fromAxisAndAngle(1, 0, 0, 90);
    return point3(gate) + rotation.rotatedVector(local);
}

void box(Batch &batch, const Gate &gate, const QVector3D &center, const QVector3D &size,
         const QColor &color) {
    const QVector3D h = size * .5f;
    const std::array<QVector3D, 8> corners{
        {gatePoint(gate, center + QVector3D(-h.x(), -h.y(), -h.z())),
         gatePoint(gate, center + QVector3D(h.x(), -h.y(), -h.z())),
         gatePoint(gate, center + QVector3D(h.x(), h.y(), -h.z())),
         gatePoint(gate, center + QVector3D(-h.x(), h.y(), -h.z())),
         gatePoint(gate, center + QVector3D(-h.x(), -h.y(), h.z())),
         gatePoint(gate, center + QVector3D(h.x(), -h.y(), h.z())),
         gatePoint(gate, center + QVector3D(h.x(), h.y(), h.z())),
         gatePoint(gate, center + QVector3D(-h.x(), h.y(), h.z()))}};
    quad(batch, corners[0], corners[1], corners[2], corners[3], color, .82f);
    quad(batch, corners[5], corners[4], corners[7], corners[6], color, .94f);
    quad(batch, corners[4], corners[0], corners[3], corners[7], color, .72f);
    quad(batch, corners[1], corners[5], corners[6], corners[2], color, .86f);
    quad(batch, corners[3], corners[2], corners[6], corners[7], color, 1.10f);
    quad(batch, corners[4], corners[5], corners[1], corners[0], color, .63f);
}

void frame(Batch &batch, const Gate &gate, float depth, const QColor &color) {
    for (const auto &beam : gateFrameBeams(gate))
        box(batch, gate, beam.center + QVector3D(0, 0, depth), beam.size, color);
}

void buildGate(Batch &batch, const Gate &gate, bool selected = false) {
    const float w = static_cast<float>(gate.width), h = static_cast<float>(gate.height);
    (void)selected; // Selection uses neutral outlines/halos, retaining the component's own color.
    const QColor color = gateColor(gate);
    const auto metrics = gateFrameMetrics(gate);
    if (gate.type == QStringLiteral("pillar")) {
        // A stretched native black cube: no openings, flag cloth or decorative geometry.
        box(batch, gate, {0, 0, 0}, {w, h, w}, color);
        return;
    }
    if (gate.type == QStringLiteral("flag")) {
        box(batch, gate, {0, 0, 0}, {.065f, h, .065f}, QColor("#484848"));
        box(batch, gate, {0, -h / 2 + .045f, 0}, {.46f, .09f, .46f}, QColor("#939393"));
        const auto a = gatePoint(gate, {.04f, h / 2 - .03f, 0});
        const auto b = gatePoint(gate, {w * .42f, h / 2 - h * .13f, .07f});
        const auto c = gatePoint(gate, {w * .34f, h / 2 - h * .50f, .03f});
        const auto d = gatePoint(gate, {.04f, h / 2 - h * .42f, 0});
        quad(batch, a, b, c, d, color);
        const auto a2 = gatePoint(gate, {.08f, h / 2 - h * .17f, .012f});
        const auto b2 = gatePoint(gate, {w * .29f, h / 2 - h * .23f, .06f});
        const auto c2 = gatePoint(gate, {.08f, h / 2 - h * .24f, .012f});
        triangle(batch, a2, b2, c2, QColor("#f4f4f4"));
        return;
    }
    if (metrics.centerlineModel) {
        // The model/export share every physical stock: no legacy dividers or duplicate walls.
        frame(batch, gate, 0, color);
        return;
    }
    const bool cube = gate.type == QStringLiteral("cube");
    frame(batch, gate, cube ? -w / 2 : 0, color);
    if (cube) {
        frame(batch, gate, w / 2, color);
        for (const float x : {-w / 2, w / 2})
            for (const float y : {-h / 2, h / 2})
                box(batch, gate, {x, y, 0}, {.085f, .085f, w}, color);
    }
    if (gate.type == QStringLiteral("double"))
        box(batch, gate, {0, 0, 0}, {.09f, h, .09f}, color);
    if (gate.type == QStringLiteral("ladder"))
        box(batch, gate, {0, 0, 0}, {w, .09f, .09f}, color);
    if (gate.type == QStringLiteral("start") && !metrics.centerlineModel) {
        const int squares = std::max(4, static_cast<int>(w / .24f));
        const float tile = w / static_cast<float>(squares);
        for (int i = 0; i < squares; ++i)
            box(batch, gate, {-w / 2 + tile * (static_cast<float>(i) + .5f), h / 2, .007f},
                {tile * .97f, .19f, .12f}, i % 2 ? QColor("#f5f5f5") : QColor("#171717"));
    } else if (!metrics.centerlineModel) {
        // Small white corner sleeves make the opening legible at oblique angles.
        for (const float x : {-w / 2, w / 2})
            box(batch, gate, {x, h / 2 - .13f, .007f}, {.12f, .22f, .12f}, QColor("#e5e5e5"));
    }
}

void tube(Batch &batch, const QVector3D &a, const QVector3D &b, float radius, const QColor &color) {
    const QVector3D axis = b - a;
    if (axis.lengthSquared() < .000001f)
        return;
    const QVector3D direction = axis.normalized();
    const QVector3D reference =
        std::abs(direction.y()) > .9f ? QVector3D(1, 0, 0) : QVector3D(0, 1, 0);
    const QVector3D right = QVector3D::crossProduct(direction, reference).normalized();
    const QVector3D up = QVector3D::crossProduct(direction, right).normalized();
    for (int i = 0; i < 6; ++i) {
        const float first = static_cast<float>(i * Pi / 3),
                    last = static_cast<float>((i + 1) * Pi / 3);
        const QVector3D r1 = (right * std::cos(first) + up * std::sin(first)) * radius;
        const QVector3D r2 = (right * std::cos(last) + up * std::sin(last)) * radius;
        quad(batch, a + r1, b + r1, b + r2, a + r2, color, .80f + .12f * static_cast<float>(i % 3));
    }
}

double segmentDistance(const QPointF &point, const QPointF &a, const QPointF &b) {
    const QPointF v = b - a, u = point - a;
    const double square = QPointF::dotProduct(v, v);
    const double t = square > .001 ? std::clamp(QPointF::dotProduct(u, v) / square, 0.0, 1.0) : 0;
    const QPointF difference = point - (a + v * t);
    return std::hypot(difference.x(), difference.y());
}

class FallbackCanvas final : public QWidget {
  public:
    explicit FallbackCanvas(QWidget *parent) : QWidget(parent) {
        setAttribute(Qt::WA_TransparentForMouseEvents);
        setAttribute(Qt::WA_OpaquePaintEvent);
    }
    std::function<void(QPainter &)> draw;

  protected:
    void paintEvent(QPaintEvent *) override {
        QPainter painter(this);
        if (draw)
            draw(painter);
    }
};
} // namespace

struct Scene3D::Impl {
    explicit Impl(Scene3D *owner) : q(owner) {
        grid.mode = routeLine.mode = projection.mode = selection.mode = GL_LINES;
        shadows.opacity = .05f;
        projection.opacity = .44f;
        halo.opacity = .72f;
        fallback = new FallbackCanvas(q);
        fallback->hide();
        fallback->draw = [this](QPainter &painter) { paintSoftware(painter); };
        timer.setInterval(16);
        QObject::connect(&timer, &QTimer::timeout, q, [this] {
            const double delta = std::clamp(elapsed.restart() / 1000.0, 0.0, .10);
            travel += delta * speedKph / 3.6;
            if (!routeLengths.empty() && routeLengths.back() > .001)
                travel = std::fmod(travel, routeLengths.back());
            q->update();
            if (fallback->isVisible())
                fallback->update();
        });
    }
    Scene3D *q;
    TrackDocument *document = nullptr;
    QString selected, selectedStep, placement;
    double placeWidth = 2.4, placeHeight = 2.4;
    bool snap = true, routeVisible = true, space = false, firstFit = true;
    bool glReady = false, dirty = true, ghostDirty = true, haveGhost = false;
    bool offscreen = QGuiApplication::platformName() == QStringLiteral("offscreen") ||
                     QGuiApplication::platformName() == QStringLiteral("minimal");
    QString glError;
    enum class Gesture {
        Idle,
        Orbit,
        Pan,
        SelectOnly,
        GateDragPending,
        GateDrag
    } gesture = Gesture::Idle;
    QPointF lastMouse, pressMouse, dragOffset, ghost;
    double dragHeight = 0;
    QVector3D target{0, 1.0f, 0};
    float azimuth = -25, pitch = 43;
    double distance = 85, fittedDistance = 85, radius = 50;
    QRectF groundBounds{-45, -32, 90, 64};
    QRectF contentBounds{-30, -20, 60, 40};
    QMatrix4x4 view, projectionMatrix, viewProjection;
    QVector3D eye;
    Batch ground, shadows, grid, solids, routeTube, routeLine, projection, selection, halo, preview,
        ghostBatch;
    std::array<Batch *, 11> batches{{&ground, &shadows, &grid, &solids, &routeTube, &routeLine,
                                     &projection, &selection, &halo, &preview, &ghostBatch}};
    std::unique_ptr<QOpenGLShaderProgram> program;
    FallbackCanvas *fallback = nullptr;
    QTimer timer;
    QElapsedTimer elapsed;
    double speedKph = 70, travel = 0;
    std::vector<QVector3D> route;
    std::vector<double> routeLengths;
    struct RouteVisual {
        RouteStep step;
        RouteStepGeometry geometry;
        int number = 0, gateNumber = 0, visit = 0, overlap = 0;
    };
    std::vector<RouteVisual> routeVisuals;
    mutable QHash<QString, QRectF> routeBadgeCache;
    mutable QMatrix4x4 routeBadgeMatrix;
    mutable QSize routeBadgeSize;
    mutable bool routeBadgesReady = false;

    const Gate *gate(const QString &id) const {
        if (document)
            for (const auto &item : document->gates)
                if (item.id == id)
                    return &item;
        return nullptr;
    }

    void matrices() {
        const double yaw = azimuth * Pi / 180, elevation = pitch * Pi / 180;
        const QVector3D direction(static_cast<float>(std::sin(yaw) * std::cos(elevation)),
                                  static_cast<float>(std::sin(elevation)),
                                  static_cast<float>(std::cos(yaw) * std::cos(elevation)));
        eye = target + direction * static_cast<float>(distance);
        view.setToIdentity();
        view.lookAt(eye, target, QVector3D(0, 1, 0));
        projectionMatrix.setToIdentity();
        projectionMatrix.perspective(FieldOfView,
                                     static_cast<float>(std::max(1, q->width())) /
                                         static_cast<float>(std::max(1, q->height())),
                                     static_cast<float>(std::max(.025, distance / 20000)),
                                     static_cast<float>(std::max(1000.0, distance + radius * 5)));
        viewProjection = projectionMatrix * view;
    }

    bool project(const QVector3D &world, QPointF *screen, double *depth = nullptr) const {
        const QVector4D clip = viewProjection * QVector4D(world, 1);
        if (clip.w() <= .0001f)
            return false;
        const QVector3D ndc = clip.toVector3DAffine();
        if (!std::isfinite(ndc.x()) || !std::isfinite(ndc.y()) || ndc.z() < -1 || ndc.z() > 1)
            return false;
        if (depth)
            *depth = clip.w();
        *screen = {(ndc.x() + 1.0) * q->width() / 2.0, (1.0 - ndc.y()) * q->height() / 2.0};
        return true;
    }

    bool intersect(const QPointF &screen, double height, QPointF *point) const {
        bool invertible = false;
        const QMatrix4x4 inverse = viewProjection.inverted(&invertible);
        if (!invertible || q->width() <= 0 || q->height() <= 0)
            return false;
        const float x = static_cast<float>(2 * screen.x() / q->width() - 1);
        const float y = static_cast<float>(1 - 2 * screen.y() / q->height());
        const auto near = (inverse * QVector4D(x, y, -1, 1)).toVector3DAffine();
        const auto far = (inverse * QVector4D(x, y, 1, 1)).toVector3DAffine();
        const auto direction = far - near;
        if (std::abs(direction.y()) < .000001f)
            return false;
        const double t = (height - near.y()) / direction.y();
        if (t < 0 || !std::isfinite(t))
            return false;
        const QVector3D hit = near + direction * static_cast<float>(t);
        if (!std::isfinite(hit.x()) || !std::isfinite(hit.z()))
            return false;
        *point = {hit.x(), hit.z()};
        return true;
    }

    QPointF snapped(QPointF point) const {
        if (snap) {
            const double step = document ? std::max(.05, document->gridSize / 2) : .5;
            point = {std::round(point.x() / step) * step, std::round(point.y() / step) * step};
        }
        return point;
    }

    void layoutRouteBadges() const {
        if (routeBadgesReady && routeBadgeMatrix == viewProjection && routeBadgeSize == q->size())
            return;
        routeBadgeCache.clear();
        routeBadgeMatrix = viewProjection;
        routeBadgeSize = q->size();
        routeBadgesReady = true;
        const QRectF available(q->rect().adjusted(12, 58, -12, -55));
        QVector<QRectF> occupied;
        if (const auto *active = gate(selected)) {
            QPointF point;
            if (project(point3(*active) +
                            QVector3D(0,
                                      static_cast<float>(gateFrameMetrics(*active).outerHeight / 2),
                                      0),
                        &point))
                occupied.append(QRectF(point.x() - 22, point.y() - 25, 44, 22));
        }
        for (const auto &visual : routeVisuals) {
            QPointF point;
            if (visual.step.kind != QStringLiteral("pass") ||
                !project(visual.geometry.position, &point) || !available.contains(point))
                continue;
            const QPointF preferred = point + QPointF(15, -17);
            const qreal width = visual.number >= 100 ? 30 : 23;
            QRectF chosen;
            for (int ring = 0; ring <= 10 && chosen.isEmpty(); ++ring) {
                const int candidates = ring == 0 ? 1 : 16;
                for (int candidate = 0; candidate < candidates; ++candidate) {
                    const double angle = candidate * Pi / 8 - Pi / 2;
                    const QPointF offset =
                        ring == 0 ? QPointF()
                                  : QPointF(std::cos(angle), std::sin(angle)) * (ring * 25.0);
                    const QPointF center = preferred + offset;
                    const QRectF badge(center.x() - width / 2, center.y() - 11.5, width, 23);
                    if (!available.contains(badge))
                        continue;
                    const auto padded = badge.adjusted(-3, -3, 3, 3);
                    const bool overlaps = std::any_of(
                        occupied.begin(), occupied.end(),
                        [&padded](const QRectF &other) { return padded.intersects(other); });
                    if (!overlaps) {
                        chosen = badge;
                        break;
                    }
                }
            }
            if (!chosen.isEmpty()) {
                occupied.append(chosen);
                routeBadgeCache.insert(visual.step.id, chosen);
            }
        }
    }

    QRectF routeBadge(const RouteVisual &visual) const {
        layoutRouteBadges();
        return routeBadgeCache.value(visual.step.id);
    }

    QString pickRoute(const QPointF &position) const {
        if (routeVisible && routeVisuals.size() < 400)
            for (const auto &visual : routeVisuals)
                if (routeBadge(visual).contains(position))
                    return visual.step.id;
        return {};
    }

    QString pick(const QPointF &position) {
        matrices();
        if (!document)
            return {};
        // The visible halo is also a drag target, particularly for narrow posts.
        // It only exists after selection, preserving the two-press interaction.
        if (const auto *active = gate(selected)) {
            const float haloRadius = selectionHaloRadius(*active);
            const QVector3D base(static_cast<float>(active->position.x()), .055f,
                                 static_cast<float>(active->position.y()));
            QPolygonF outer, inner, center;
            for (int i = 0; i < 72; ++i) {
                const float angle = static_cast<float>(i * Pi / 36);
                const QVector3D direction(std::cos(angle), 0, std::sin(angle));
                QPointF a, b, c;
                if (project(base + direction * haloRadius * 1.16f, &a) &&
                    project(base + direction * haloRadius * .86f, &b) &&
                    project(base + direction * haloRadius, &c)) {
                    outer.append(a);
                    inner.append(b);
                    center.append(c);
                }
            }
            if (outer.size() == 72) {
                bool haloHit = outer.containsPoint(position, Qt::OddEvenFill) &&
                               !inner.containsPoint(position, Qt::OddEvenFill);
                for (int i = 0; i < center.size() && !haloHit; ++i)
                    haloHit =
                        segmentDistance(position, center[i], center[(i + 1) % center.size()]) < 5.0;
                if (haloHit)
                    return active->id;
            }
        }
        QString best;
        double bestScore = std::numeric_limits<double>::max();
        for (const auto &item : document->gates) {
            const auto metrics = gateFrameMetrics(item);
            const float w = static_cast<float>(metrics.centerlineModel ? metrics.outerWidth / 2
                                                                       : item.width / 2),
                        h = static_cast<float>(metrics.centerlineModel ? metrics.outerHeight / 2
                                                                       : item.height / 2);
            const float halfDepth = static_cast<float>(metrics.centerDepth / 2);
            const std::array<QVector3D, 8> corners{
                {gatePoint(item, {-w, -h, -halfDepth}), gatePoint(item, {w, -h, -halfDepth}),
                 gatePoint(item, {w, h, -halfDepth}), gatePoint(item, {-w, h, -halfDepth}),
                 gatePoint(item, {-w, -h, halfDepth}), gatePoint(item, {w, -h, halfDepth}),
                 gatePoint(item, {w, h, halfDepth}), gatePoint(item, {-w, h, halfDepth})}};
            QPointF center;
            double depth = 0;
            if (!project(point3(item), &center, &depth))
                continue;
            double hitDistance = std::numeric_limits<double>::max();
            QRectF projectedBounds;
            bool inside = false;
            for (const auto &face :
                 {std::array<int, 4>{0, 1, 2, 3}, std::array<int, 4>{4, 5, 6, 7},
                  std::array<int, 4>{0, 4, 7, 3}, std::array<int, 4>{1, 5, 6, 2},
                  std::array<int, 4>{0, 1, 5, 4}, std::array<int, 4>{3, 2, 6, 7}}) {
                QPolygonF polygon;
                for (const int corner : face) {
                    QPointF point;
                    if (project(corners[corner], &point))
                        polygon.append(point);
                }
                if (polygon.size() != 4)
                    continue;
                projectedBounds = projectedBounds.united(polygon.boundingRect());
                inside = inside || polygon.containsPoint(position, Qt::OddEvenFill);
                for (int i = 0; i < 4; ++i)
                    hitDistance = std::min(
                        hitDistance, segmentDistance(position, polygon[i], polygon[(i + 1) % 4]));
                if (halfDepth == 0)
                    break;
            }
            const double projectedSpan =
                std::max(projectedBounds.width(), projectedBounds.height());
            const double threshold = std::clamp(projectedSpan * .15, 6.0, 14.0);
            QPointF label;
            const bool hasLabel = (!routeVisible || item.id == selected) &&
                                  project(point3(item) + QVector3D(0, h, 0), &label);
            label.ry() -= 14;
            const bool labelHit =
                hasLabel && std::hypot(position.x() - label.x(), position.y() - label.y()) < 17;
            if (!inside && hitDistance > threshold && !labelHit)
                continue;
            // Prefer the nearest surface when projected openings overlap.
            const double score = depth + (inside ? 0 : hitDistance * .03);
            if (score < bestScore) {
                bestScore = score;
                best = item.id;
            }
        }
        return best;
    }

    void rebuild() {
        for (auto *batch : batches)
            batch->vertices.clear();
        route.clear();
        routeLengths.clear();
        routeVisuals.clear();
        routeBadgesReady = false;
        if (document) {
            const auto points = routePolyline(*document);
            route.assign(points.begin(), points.end());
            QHash<QString, int> gateNumbers, visits, overlaps;
            for (qsizetype i = 0; i < document->gates.size(); ++i)
                gateNumbers.insert(document->gates[i].id, static_cast<int>(i + 1));
            int number = 0;
            for (const auto &step : effectiveRouteSteps(*document)) {
                const auto geometry = routeStepGeometry(*document, step);
                if (!geometry.valid)
                    continue;
                RouteVisual visual;
                visual.step = step;
                visual.geometry = geometry;
                visual.gateNumber = gateNumbers.value(step.gateId);
                if (step.kind == QStringLiteral("pass")) {
                    visual.number = ++number;
                    visual.visit = ++visits[step.gateId];
                    const auto &p = geometry.position;
                    const QString key = QString::number(p.x(), 'f', 3) + '/' +
                                        QString::number(p.y(), 'f', 3) + '/' +
                                        QString::number(p.z(), 'f', 3);
                    visual.overlap = overlaps[key]++;
                }
                routeVisuals.push_back(visual);
            }
        }
        QRectF bounds;
        bool hasBounds = false;
        if (document) {
            for (const auto &item : document->gates) {
                const auto metrics = gateFrameMetrics(item);
                const double half =
                    std::max(2.0, std::max(metrics.outerWidth, metrics.outerDepth) / 2 + .5);
                const QRectF one(item.position.x() - half, item.position.y() - half, half * 2,
                                 half * 2);
                bounds = hasBounds ? bounds.united(one) : one;
                hasBounds = true;
                if (item.type == QStringLiteral("pillar")) {
                    const QPointF passage = gatePassagePosition(item);
                    bounds = bounds.united(QRectF(passage - QPointF(1.5, 1.5), QSizeF(3, 3)));
                }
            }
            for (const auto &point : route) {
                const QRectF one(point.x() - .75, point.z() - .75, 1.5, 1.5);
                bounds = hasBounds ? bounds.united(one) : one;
                hasBounds = true;
            }
            if (document->takeoff.enabled) {
                const QRectF one(document->takeoff.position - QPointF(1, 1), QSizeF(2, 2));
                bounds = hasBounds ? bounds.united(one) : one;
                hasBounds = true;
            }
        }
        if (!hasBounds)
            bounds = QRectF(-30, -20, 60, 40);
        contentBounds = bounds;
        const double margin = std::max(5.0, std::max(bounds.width(), bounds.height()) * .12);
        groundBounds = bounds.adjusted(-margin, -margin, margin, margin);
        const float x0 = static_cast<float>(groundBounds.left()),
                    x1 = static_cast<float>(groundBounds.right());
        const float z0 = static_cast<float>(groundBounds.top()),
                    z1 = static_cast<float>(groundBounds.bottom());
        quad(ground, {x0, -.045f, z0}, {x1, -.045f, z0}, {x1, -.045f, z1}, {x0, -.045f, z1},
             QColor("#ededed"));
        const QColor border("#bcbcbc");
        line(grid, {x0, .01f, z0}, {x1, .01f, z0}, border);
        line(grid, {x1, .01f, z0}, {x1, .01f, z1}, border);
        line(grid, {x1, .01f, z1}, {x0, .01f, z1}, border);
        line(grid, {x0, .01f, z1}, {x0, .01f, z0}, border);
        if (document) {
            for (const auto &item : document->gates) {
                buildGate(solids, item, item.id == selected);
                const float w =
                    static_cast<float>(std::max(gateFrameMetrics(item).outerWidth * .65, .7));
                const QVector3D center(
                    static_cast<float>(item.position.x() + item.elevation * .24), .016f,
                    static_cast<float>(item.position.y() + item.elevation * .16));
                for (int i = 0; i < 20; ++i) {
                    const double a = i * Pi / 10, b = (i + 1) * Pi / 10;
                    triangle(shadows, center,
                             center + QVector3D(static_cast<float>(std::cos(a)) * w, 0,
                                                static_cast<float>(std::sin(a)) * .55f),
                             center + QVector3D(static_cast<float>(std::cos(b)) * w, 0,
                                                static_cast<float>(std::sin(b)) * .55f),
                             QColor("#333333"));
                }
                const float a = static_cast<float>(item.yaw * Pi / 180);
                const QVector3D direction(std::sin(a), 0, std::cos(a));
                const QVector3D side(std::cos(a), 0, -std::sin(a));
                const QPointF passage = gatePassagePosition(item);
                const QVector3D arrowBase(static_cast<float>(passage.x()), .04f,
                                          static_cast<float>(passage.y()));
                const QColor arrowColor =
                    item.id == selected ? QColor("#222222") : QColor("#969696");
                if (!document->routeStepsDefined && document->routeSteps.isEmpty())
                    triangle(solids, arrowBase + direction * 1.15f,
                             arrowBase + direction * .65f + side * .19f,
                             arrowBase + direction * .65f - side * .19f, arrowColor);
                if (item.id == selected) {
                    const auto metrics = gateFrameMetrics(item);
                    const float halfW = static_cast<float>(metrics.outerWidth / 2 + .15);
                    const float halfH = static_cast<float>(metrics.outerHeight / 2 + .15);
                    const float halfD = static_cast<float>(metrics.outerDepth / 2 + .15);
                    const QColor selectionColor("#161616");
                    const QVector<float> depths = metrics.centerDepth > 0
                                                      ? QVector<float>{-halfD, halfD}
                                                      : QVector<float>{-.015f};
                    for (const float z : depths) {
                        const auto a0 = gatePoint(item, {-halfW, -halfH, z});
                        const auto b0 = gatePoint(item, {halfW, -halfH, z});
                        const auto c0 = gatePoint(item, {halfW, halfH, z});
                        const auto d0 = gatePoint(item, {-halfW, halfH, z});
                        line(selection, a0, b0, selectionColor);
                        line(selection, b0, c0, selectionColor);
                        line(selection, c0, d0, selectionColor);
                        line(selection, d0, a0, selectionColor);
                    }
                    if (metrics.centerDepth > 0)
                        for (const float x : {-halfW, halfW})
                            for (const float y : {-halfH, halfH})
                                line(selection, gatePoint(item, {x, y, -halfD}),
                                     gatePoint(item, {x, y, halfD}), selectionColor);
                    line(selection,
                         {static_cast<float>(item.position.x()), .03f,
                          static_cast<float>(item.position.y())},
                         point3(item), QColor("#8f8f8f"));
                    const float haloRadius = selectionHaloRadius(item);
                    const QVector3D base(static_cast<float>(item.position.x()), .055f,
                                         static_cast<float>(item.position.y()));
                    for (int i = 0; i < 72; ++i) {
                        const float angleA = static_cast<float>(i * Pi / 36),
                                    angleB = static_cast<float>((i + 1) * Pi / 36);
                        const QVector3D directionA(std::cos(angleA), 0, std::sin(angleA));
                        const QVector3D directionB(std::cos(angleB), 0, std::sin(angleB));
                        quad(halo, base + directionA * haloRadius * .86f,
                             base + directionB * haloRadius * .86f,
                             base + directionB * haloRadius * 1.16f,
                             base + directionA * haloRadius * 1.16f, QColor("#ffffff"));
                        line(selection, base + directionA * haloRadius,
                             base + directionB * haloRadius, QColor("#161616"));
                    }
                }
            }
            if (route.size() > 1) {
                double length = 0;
                routeLengths.push_back(0);
                for (size_t i = 1; i < route.size(); ++i) {
                    const auto a = route[i - 1], b = route[i];
                    length += (b - a).length();
                    routeLengths.push_back(length);
                    tube(routeTube, a, b, .035f, QColor("#626262"));
                    line(routeLine, a, b, QColor("#595959"));
                    line(projection, {a.x(), .025f, a.z()}, {b.x(), .025f, b.z()},
                         QColor("#9e9e9e"));
                }
            }
        }
        updateGhost();
        if (!gate(selected))
            selected.clear();
        if (std::none_of(
                routeVisuals.begin(), routeVisuals.end(),
                [this](const RouteVisual &visual) { return visual.step.id == selectedStep; }))
            selectedStep.clear();
        dirty = true;
    }

    void updateGhost() {
        ghostBatch.vertices.clear();
        if (!placement.isEmpty() && placement != QStringLiteral("__takeoff") && haveGhost) {
            Gate item = makeGate(placement, placeWidth, placeHeight);
            item.position = ghost;
            buildGate(ghostBatch, item);
            ghostBatch.opacity = .38f;
        }
        ghostDirty = true;
    }

    QVector3D previewPoint() const {
        if (route.empty() || routeLengths.empty())
            return {};
        const double maximum = routeLengths.back();
        const double length = maximum > .001 ? std::fmod(travel, maximum) : 0;
        const auto it = std::lower_bound(routeLengths.begin(), routeLengths.end(), length);
        const size_t index = static_cast<size_t>(std::distance(routeLengths.begin(), it));
        if (index == 0 || index >= route.size())
            return route.front();
        const double denominator = routeLengths[index] - routeLengths[index - 1];
        const float fraction =
            denominator > .0001
                ? static_cast<float>((length - routeLengths[index - 1]) / denominator)
                : 0;
        return route[index - 1] * (1 - fraction) + route[index] * fraction;
    }

    void releaseGl() {
        program.reset();
        for (auto *batch : batches)
            batch->buffer.destroy();
        glReady = false;
        dirty = true;
    }

    void takeoffOverlay(QPainter &painter, const QPointF &position, double elevation, double yaw,
                        bool ghostMarker = false) const {
        const QVector3D base(static_cast<float>(position.x()), static_cast<float>(elevation + .035),
                             static_cast<float>(position.y()));
        QPolygonF ring;
        for (int i = 0; i < 48; ++i) {
            const float angle = static_cast<float>(i * Pi / 24);
            QPointF point;
            if (project(base + QVector3D(std::cos(angle) * .8f, 0, std::sin(angle) * .8f), &point))
                ring.append(point);
        }
        if (ring.size() != 48)
            return;
        painter.save();
        painter.setOpacity(ghostMarker ? .55 : 1);
        painter.setBrush(QColor(255, 255, 255, 180));
        painter.setPen(QPen(Qt::white, 5));
        painter.drawPolygon(ring);
        painter.setPen(QPen(QColor("#202020"), 2, ghostMarker ? Qt::DashLine : Qt::SolidLine));
        painter.drawPolygon(ring);
        const float angle = static_cast<float>(yaw * Pi / 180);
        const QVector3D forward(std::sin(angle), 0, std::cos(angle));
        const QVector3D right(std::cos(angle), 0, -std::sin(angle));
        QPointF a, b, c;
        if (project(base + forward * .55f, &a) &&
            project(base - forward * .2f + right * .24f, &b) &&
            project(base - forward * .2f - right * .24f, &c)) {
            painter.setPen(Qt::NoPen);
            painter.setBrush(QColor("#252525"));
            painter.drawPolygon(QPolygonF{a, b, c});
        }
        const QRectF label(ring.boundingRect().center().x() - 25, ring.boundingRect().top() - 22,
                           50, 19);
        painter.setPen(QColor("#222222"));
        painter.setBrush(QColor(255, 255, 255, 235));
        painter.drawRoundedRect(label, 4, 4);
        painter.setFont(QFont(QStringLiteral("Segoe UI"), 8, QFont::DemiBold));
        painter.drawText(label, Qt::AlignCenter, QStringLiteral("T/O"));
        painter.restore();
    }

    void routeOverlays(QPainter &painter) const {
        painter.save();
        painter.setRenderHint(QPainter::Antialiasing);
        if (routeVisible && routeVisuals.size() < 400) {
            // Paint leaders before the badges so nearby labels cannot cross their numbers.
            painter.setPen(QPen(QColor("#ababab"), .8));
            for (const auto &visual : routeVisuals) {
                const auto badge = routeBadge(visual);
                QPointF point;
                if (!badge.isEmpty() && project(visual.geometry.position, &point))
                    painter.drawLine(point, badge.center());
            }
            for (const auto &visual : routeVisuals) {
                const auto &geometry = visual.geometry;
                const bool active = visual.step.id == selectedStep;
                QPointF center;
                if (!project(geometry.position, &center))
                    continue;
                if (visual.step.kind == QStringLiteral("guide")) {
                    const qreal glyphRadius = active ? 6 : 4;
                    painter.setPen(QPen(QColor(active ? "#111111" : "#888888"), 1.4));
                    painter.setBrush(active ? QColor("#333333") : Qt::white);
                    painter.drawPolygon(QPolygonF{
                        center + QPointF(0, -glyphRadius), center + QPointF(glyphRadius, 0),
                        center + QPointF(0, glyphRadius), center + QPointF(-glyphRadius, 0)});
                    continue;
                }
                if (active) {
                    QPolygonF window;
                    const auto r = geometry.right * static_cast<float>(geometry.openingWidth / 2);
                    const auto u = geometry.up * static_cast<float>(geometry.openingHeight / 2);
                    for (const auto &corner :
                         {geometry.position - r - u, geometry.position + r - u,
                          geometry.position + r + u, geometry.position - r + u}) {
                        QPointF point;
                        if (project(corner, &point))
                            window.append(point);
                    }
                    if (window.size() == 4) {
                        painter.setBrush(QColor(255, 255, 255, 24));
                        painter.setPen(QPen(QColor(255, 255, 255, 220), 5));
                        painter.drawPolygon(window);
                        painter.setPen(QPen(QColor("#111111"), 2));
                        painter.drawPolygon(window);
                    }
                }
                QPointF start, finish, wingA, wingB;
                if (project(geometry.position - geometry.normal * .45f, &start) &&
                    project(geometry.position + geometry.normal * .45f, &finish) &&
                    project(geometry.position + geometry.normal * .22f + geometry.right * .13f,
                            &wingA) &&
                    project(geometry.position + geometry.normal * .22f - geometry.right * .13f,
                            &wingB)) {
                    painter.setPen(QPen(Qt::white, active ? 5 : 4, Qt::SolidLine, Qt::RoundCap));
                    painter.drawLine(start, finish);
                    painter.setPen(QPen(QColor(active ? "#111111" : "#666666"), active ? 2.5 : 1.7,
                                        Qt::SolidLine, Qt::RoundCap));
                    painter.drawLine(start, finish);
                    painter.setBrush(QColor(active ? "#111111" : "#666666"));
                    painter.drawPolygon(QPolygonF{finish, wingA, wingB});
                }
            }
            for (const auto &visual : routeVisuals) {
                const auto badge = routeBadge(visual);
                if (badge.isEmpty())
                    continue;
                const bool active = visual.step.id == selectedStep;
                painter.setPen(QPen(QColor(active ? "#111111" : "#666666"), active ? 1.8 : 1));
                painter.setBrush(active ? QColor("#222222") : QColor("#ffffff"));
                painter.drawEllipse(badge);
                painter.setFont(QFont(QStringLiteral("Segoe UI"), 8, QFont::DemiBold));
                painter.setPen(active ? Qt::white : QColor("#222222"));
                painter.drawText(badge, Qt::AlignCenter, QString::number(visual.number));
            }
        }
        if (document && document->takeoff.enabled)
            takeoffOverlay(painter, document->takeoff.position, document->takeoff.elevation,
                           document->takeoff.yaw);
        if (haveGhost && placement == QStringLiteral("__takeoff"))
            takeoffOverlay(painter, ghost, 0, document ? document->takeoff.yaw : 0, true);
        painter.restore();
    }

    void labels(QPainter &painter, bool software) {
        routeOverlays(painter);
        painter.setRenderHint(QPainter::Antialiasing);
        QFont number(QStringLiteral("Segoe UI"), 8);
        number.setWeight(QFont::DemiBold);
        painter.setFont(number);
        if (document && document->gates.size() < 400) {
            for (qsizetype i = 0; i < document->gates.size(); ++i) {
                const auto &item = document->gates[i];
                if (routeVisible && item.id != selected)
                    continue;
                QPointF point;
                const auto metrics = gateFrameMetrics(item);
                const float top = static_cast<float>(
                    metrics.centerlineModel ? metrics.outerHeight / 2 : item.height / 2);
                if (!project(point3(item) + QVector3D(0, top, 0), &point))
                    continue;
                point.ry() -= 14;
                if (point.x() < 12 || point.x() > q->width() - 12 || point.y() < 12 ||
                    point.y() > q->height() - 55)
                    continue;
                const bool active = item.id == selected;
                const double badgeWidth = i >= 99 ? 40 : 32;
                const QRectF badge(point.x() - badgeWidth / 2, point.y() - 8, badgeWidth, 16);
                painter.setPen(QPen(active ? QColor("#111111") : QColor("#bfbfbf"), 1));
                painter.setBrush(active ? QColor("#222222") : QColor(255, 255, 255, 238));
                painter.drawRoundedRect(badge, 5, 5);
                painter.setPen(active ? Qt::white : QColor("#303030"));
                painter.drawText(badge, Qt::AlignCenter, QStringLiteral("G%1").arg(i + 1));
            }
        }
        painter.setFont(QFont(QStringLiteral("Microsoft YaHei UI"), 9));
        const QString title =
            software ? QStringLiteral("透视预览 · 硬件加速不可用") : QStringLiteral("3D 透视视图");
        painter.setPen(QColor("#333333"));
        painter.setBrush(QColor(255, 255, 255, 222));
        painter.drawRoundedRect(QRectF(16, 15, software ? 215 : 123, 31), 10, 10);
        painter.drawText(QRectF(27, 15, software ? 200 : 105, 31), Qt::AlignVCenter | Qt::AlignLeft,
                         title);
        const QString hint = !placement.isEmpty()
                                 ? QStringLiteral("点击地面放置 · Esc 结束放置")
                                 : QStringLiteral("先点击选中，再按住拖动 · 右键环绕 · 中键平移");
        const QRectF hintRect(16, q->height() - 44, std::min(440, q->width() - 110), 29);
        painter.setPen(Qt::NoPen);
        painter.setBrush(QColor(255, 255, 255, 226));
        painter.drawRoundedRect(hintRect, 9, 9);
        painter.setPen(QColor("#525252"));
        painter.drawText(hintRect.adjusted(11, 0, -6, 0), Qt::AlignVCenter | Qt::AlignLeft, hint);
        if (timer.isActive()) {
            const QRectF tag(16, 54, 200, 27);
            painter.setPen(Qt::NoPen);
            painter.setBrush(QColor(235, 235, 235, 244));
            painter.drawRoundedRect(tag, 8, 8);
            painter.setPen(QColor("#3e3e3e"));
            painter.drawText(tag, Qt::AlignCenter,
                             QStringLiteral("几何路线预览 · %1 km/h").arg(qRound(speedKph)));
        }
        const QPointF base(q->width() - 47.0, q->height() - 43.0);
        const auto axis = [&](const QVector3D &direction, const QColor &color,
                              const QString &name) {
            const QVector3D projected = view.mapVector(direction).normalized();
            const QPointF end = base + QPointF(projected.x(), -projected.y()) * 23;
            painter.setPen(QPen(color, 2.3, Qt::SolidLine, Qt::RoundCap));
            painter.drawLine(base, end);
            painter.setPen(QPen(QColor("#c2c2c2"), .8));
            painter.setBrush(QColor(255, 255, 255, 245));
            painter.drawEllipse(end, 8, 8);
            // Small vector glyphs remain readable even when a fallback platform
            // has no usable system font database.
            painter.save();
            painter.translate(end);
            painter.setPen(
                QPen(QColor("#222222"), 1.25, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
            painter.setBrush(Qt::NoBrush);
            QPainterPath glyph;
            if (name == QStringLiteral("X")) {
                glyph.moveTo(-3, -3.5);
                glyph.lineTo(3, 3.5);
                glyph.moveTo(-3, 3.5);
                glyph.lineTo(3, -3.5);
            } else if (name == QStringLiteral("Y")) {
                glyph.moveTo(-3, -3.5);
                glyph.lineTo(0, 0);
                glyph.lineTo(3, -3.5);
                glyph.moveTo(0, 0);
                glyph.lineTo(0, 3.5);
            } else {
                glyph.moveTo(-3, -3.5);
                glyph.lineTo(3, -3.5);
                glyph.lineTo(-3, 3.5);
                glyph.lineTo(3, 3.5);
            }
            painter.drawPath(glyph);
            painter.restore();
        };
        painter.setFont(QFont(QStringLiteral("Segoe UI"), 8, QFont::DemiBold));
        axis(editorAxisDirection(Qt::XAxis), QColor("#626262"), QStringLiteral("X"));
        axis(editorAxisDirection(Qt::YAxis), QColor("#949494"), QStringLiteral("Y"));
        axis(editorAxisDirection(Qt::ZAxis), QColor("#222222"), QStringLiteral("Z"));
    }

    void paintSoftware(QPainter &painter) {
        matrices();
        painter.setRenderHint(QPainter::Antialiasing);
        painter.fillRect(q->rect(), QColor("#fafafa"));
        QPolygonF floor;
        for (const QPointF &p : {groundBounds.topLeft(), groundBounds.topRight(),
                                 groundBounds.bottomRight(), groundBounds.bottomLeft()}) {
            QPointF screen;
            if (project({static_cast<float>(p.x()), 0, static_cast<float>(p.y())}, &screen))
                floor.append(screen);
        }
        if (floor.size() == 4) {
            painter.setPen(QPen(QColor("#bcbcbc"), 1));
            painter.setBrush(QColor("#ededed"));
            painter.drawPolygon(floor);
        }
        const auto drawLines = [&](const Batch &batch, double width) {
            for (size_t i = 1; i < batch.vertices.size(); i += 2) {
                const auto &a = batch.vertices[i - 1];
                const auto &b = batch.vertices[i];
                QPointF start, finish;
                if (!project({a.x, a.y, a.z}, &start) || !project({b.x, b.y, b.z}, &finish))
                    continue;
                painter.setPen(QPen(QColor::fromRgbF(a.r, a.g, a.b, batch.opacity), width));
                painter.drawLine(start, finish);
            }
        };
        drawLines(grid, .8);
        if (const auto *active = gate(selected)) {
            QPolygonF outer, inner;
            const float haloRadius = selectionHaloRadius(*active);
            for (int i = 0; i < 72; ++i) {
                const float angle = static_cast<float>(i * Pi / 36);
                QPointF a, b;
                const QVector3D base(static_cast<float>(active->position.x()), .055f,
                                     static_cast<float>(active->position.y()));
                const QVector3D direction(std::cos(angle), 0, std::sin(angle));
                if (project(base + direction * haloRadius * 1.16f, &a))
                    outer.append(a);
                if (project(base + direction * haloRadius * .86f, &b))
                    inner.append(b);
            }
            if (outer.size() == 72 && inner.size() == 72) {
                QPainterPath ring;
                ring.setFillRule(Qt::OddEvenFill);
                ring.addPolygon(outer);
                ring.closeSubpath();
                ring.addPolygon(inner);
                ring.closeSubpath();
                painter.setPen(Qt::NoPen);
                painter.setBrush(QColor(255, 255, 255, 184));
                painter.drawPath(ring);
            }
            drawLines(selection, 1.8);
        }
        if (routeVisible) {
            drawLines(projection, 1.4);
            drawLines(routeLine, 2);
        }
        if (document) {
            std::vector<const Gate *> ordered;
            for (const auto &item : document->gates)
                ordered.push_back(&item);
            std::sort(ordered.begin(), ordered.end(), [this](const Gate *a, const Gate *b) {
                return (eye - point3(*a)).lengthSquared() > (eye - point3(*b)).lengthSquared();
            });
            for (const auto *item : ordered) {
                if (item->type == QStringLiteral("pillar") ||
                    gateFrameMetrics(*item).centerlineModel) {
                    // Reuse actual beam/post cuboids, including their physical end overhangs.
                    Batch column;
                    buildGate(column, *item, item->id == selected);
                    struct Face {
                        QPolygonF polygon;
                        QColor color;
                        double depth = 0;
                    };
                    std::vector<Face> faces;
                    for (size_t i = 0; i + 2 < column.vertices.size(); i += 3) {
                        Face face;
                        for (size_t corner = 0; corner < 3; ++corner) {
                            const auto &v = column.vertices[i + corner];
                            QPointF point;
                            double depth = 0;
                            if (project({v.x, v.y, v.z}, &point, &depth)) {
                                face.polygon.append(point);
                                face.depth += depth;
                            }
                        }
                        const auto &color = column.vertices[i];
                        face.color = QColor::fromRgbF(std::clamp(color.r, 0.0f, 1.0f),
                                                      std::clamp(color.g, 0.0f, 1.0f),
                                                      std::clamp(color.b, 0.0f, 1.0f));
                        if (face.polygon.size() == 3)
                            faces.push_back(std::move(face));
                    }
                    std::sort(faces.begin(), faces.end(),
                              [](const Face &a, const Face &b) { return a.depth > b.depth; });
                    for (const auto &face : faces) {
                        painter.setPen(QPen(face.color, .35));
                        painter.setBrush(face.color);
                        painter.drawPolygon(face.polygon);
                    }
                    continue;
                }
                const float w = static_cast<float>(item->width / 2),
                            h = static_cast<float>(item->height / 2);
                QPolygonF polygon;
                for (const auto &corner : {QVector3D(-w, -h, 0), QVector3D(w, -h, 0),
                                           QVector3D(w, h, 0), QVector3D(-w, h, 0)}) {
                    QPointF screen;
                    if (project(gatePoint(*item, corner), &screen))
                        polygon.append(screen);
                }
                if (polygon.size() == 4) {
                    const QColor color = gateColor(*item);
                    painter.setPen(QPen(color, 3.2, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
                    painter.setBrush(QColor(color.red(), color.green(), color.blue(), 12));
                    painter.drawPolygon(polygon);
                }
            }
        }
        if (timer.isActive() && !route.empty()) {
            QPointF marker;
            if (project(previewPoint(), &marker)) {
                painter.setPen(QPen(Qt::white, 2));
                painter.setBrush(QColor("#333333"));
                painter.drawEllipse(marker, 6, 6);
            }
        }
        labels(painter, true);
    }
};

Scene3D::Scene3D(QWidget *parent) : QOpenGLWidget(parent), d_(std::make_unique<Impl>(this)) {
    QSurfaceFormat format;
    format.setRenderableType(QSurfaceFormat::OpenGL);
    format.setVersion(2, 1);
    format.setDepthBufferSize(24);
    format.setSamples(4);
    setFormat(format);
    setFocusPolicy(Qt::StrongFocus);
    setMouseTracking(true);
    setMinimumSize(320, 240);
    setAutoFillBackground(false);
    d_->rebuild();
    if (d_->offscreen)
        d_->fallback->show();
}

Scene3D::~Scene3D() {
    d_->timer.stop();
    if (context())
        QObject::disconnect(context(), nullptr, this, nullptr);
    if (context() && isValid()) {
        makeCurrent();
        d_->releaseGl();
        doneCurrent();
    }
}

void Scene3D::setDocument(TrackDocument *document) {
    d_->document = document;
    refreshGeometry();
    if (d_->firstFit && width() > 1 && height() > 1)
        fitTrack();
}

void Scene3D::refreshGeometry() {
    d_->rebuild();
    update();
    if (d_->fallback->isVisible())
        d_->fallback->update();
}

void Scene3D::fitTrack() {
    float maxY = 2.4f, minY = 0;
    std::vector<QVector3D> fitPoints;
    if (d_->document) {
        for (const auto &gate : d_->document->gates) {
            const auto metrics = gateFrameMetrics(gate);
            maxY = std::max(maxY, static_cast<float>(gate.elevation + metrics.outerHeight / 2));
            minY = std::min(minY, static_cast<float>(gate.elevation - metrics.outerHeight / 2));
            const float halfWidth = static_cast<float>(metrics.outerWidth / 2 + .15);
            const float halfHeight = static_cast<float>(metrics.outerHeight / 2 + .15);
            const float halfDepth = static_cast<float>(metrics.outerDepth / 2);
            for (const float x : {-halfWidth, halfWidth})
                for (const float y : {-halfHeight, halfHeight})
                    for (const float z : {-halfDepth, halfDepth})
                        fitPoints.push_back(gatePoint(gate, {x, y, z}));
        }
        if (d_->document->takeoff.enabled) {
            const auto &takeoff = d_->document->takeoff;
            for (const float x : {-1.f, 1.f})
                for (const float z : {-1.f, 1.f})
                    fitPoints.emplace_back(static_cast<float>(takeoff.position.x()) + x,
                                           static_cast<float>(takeoff.elevation),
                                           static_cast<float>(takeoff.position.y()) + z);
        }
    }
    const size_t routeStride =
        d_->document && (d_->document->routeStepsDefined || !d_->document->routeSteps.isEmpty())
            ? 1
            : 4;
    for (size_t i = 0; i < d_->route.size(); i += routeStride)
        fitPoints.push_back(d_->route[i]);
    if (fitPoints.empty()) {
        const QRectF bounds = d_->contentBounds.adjusted(-1.5, -1.5, 1.5, 1.5);
        for (const double x : {bounds.left(), bounds.right()})
            for (const double z : {bounds.top(), bounds.bottom()})
                fitPoints.emplace_back(static_cast<float>(x), 0, static_cast<float>(z));
    }
    QVector3D minimum = fitPoints.front(), maximum = minimum;
    for (const auto &point : fitPoints) {
        minimum.setX(std::min(minimum.x(), point.x()));
        minimum.setY(std::min(minimum.y(), point.y()));
        minimum.setZ(std::min(minimum.z(), point.z()));
        maximum.setX(std::max(maximum.x(), point.x()));
        maximum.setY(std::max(maximum.y(), point.y()));
        maximum.setZ(std::max(maximum.z(), point.z()));
    }
    const QRectF bounds =
        QRectF(QPointF(minimum.x(), minimum.z()), QPointF(maximum.x(), maximum.z()))
            .adjusted(-1.5, -1.5, 1.5, 1.5);
    minY = std::min(minY, minimum.y());
    maxY = std::max(maxY, maximum.y());
    d_->target = {static_cast<float>(bounds.center().x()), (maxY + minY) * .42f,
                  static_cast<float>(bounds.center().y())};
    const double yaw = d_->azimuth * Pi / 180, pitch = d_->pitch * Pi / 180;
    const QVector3D direction(static_cast<float>(std::sin(yaw) * std::cos(pitch)),
                              static_cast<float>(std::sin(pitch)),
                              static_cast<float>(std::cos(yaw) * std::cos(pitch)));
    const QVector3D right = QVector3D::crossProduct(QVector3D(0, 1, 0), direction).normalized();
    const QVector3D up = QVector3D::crossProduct(direction, right).normalized();
    const double tangentY =
        std::tan(FieldOfView * Pi / 360) * std::max(120, height() - 112) / std::max(1, height());
    const double tangentX =
        std::tan(FieldOfView * Pi / 360) * std::max(160, width() - 64) / std::max(1, height());
    double maxX = -std::numeric_limits<double>::max(), minX = -maxX;
    double maxUp = maxX, minUp = minX, nearest = 0;
    for (const auto &point : fitPoints) {
        const auto p = point - d_->target;
        const double depth = QVector3D::dotProduct(p, direction);
        const double x = QVector3D::dotProduct(p, right), y = QVector3D::dotProduct(p, up);
        maxX = std::max(maxX, x + tangentX * depth);
        minX = std::min(minX, x - tangentX * depth);
        maxUp = std::max(maxUp, y + tangentY * depth);
        minUp = std::min(minUp, y - tangentY * depth);
        nearest = std::max(nearest, depth);
    }
    // Fit actual frame corners and route samples in perspective, rather than
    // fitting the empty corners of a rotated world-aligned bounding rectangle.
    const double needed = std::max({6.0, nearest + 2, (maxX - minX) / (2 * tangentX),
                                    (maxUp - minUp) / (2 * tangentY)}) *
                          1.025;
    d_->target += right * static_cast<float>((maxX + minX) / 2) +
                  up * static_cast<float>((maxUp + minUp) / 2);
    d_->radius = std::sqrt(bounds.width() * bounds.width() + bounds.height() * bounds.height() +
                           (maxY - minY) * (maxY - minY)) /
                 2;
    d_->distance = d_->fittedDistance = std::clamp(needed, 5.0, 4000000.0);
    d_->firstFit = false;
    d_->matrices();
    if (onZoom)
        onZoom(100);
    update();
    d_->fallback->update();
}

void Scene3D::selectGate(const QString &id) {
    if (d_->selected == id)
        return;
    d_->selected = d_->gate(id) ? id : QString();
    refreshGeometry();
    if (onSelection)
        onSelection(d_->selected);
}

QString Scene3D::selectedGate() const {
    return d_->selected;
}

void Scene3D::selectRouteStep(const QString &id) {
    const auto found =
        std::find_if(d_->routeVisuals.begin(), d_->routeVisuals.end(),
                     [&id](const Impl::RouteVisual &visual) { return visual.step.id == id; });
    const QString selected = found == d_->routeVisuals.end() ? QString() : id;
    if (selected == d_->selectedStep)
        return;
    d_->selectedStep = selected;
    update();
    d_->fallback->update();
    if (onRouteSelection)
        onRouteSelection(selected);
}

QString Scene3D::selectedRouteStep() const {
    return d_->selectedStep;
}

void Scene3D::setPlacement(QString type, double width, double height) {
    d_->placement = std::move(type);
    d_->placeWidth = width;
    d_->placeHeight = height;
    d_->haveGhost = false;
    setCursor(d_->placement.isEmpty() ? Qt::ArrowCursor : Qt::CrossCursor);
    d_->updateGhost();
    update();
    d_->fallback->update();
}

void Scene3D::setSnap(bool enabled) {
    d_->snap = enabled;
}
void Scene3D::setRouteVisible(bool enabled) {
    d_->routeVisible = enabled;
    update();
    d_->fallback->update();
}
void Scene3D::setPreviewSpeed(double speed) {
    d_->speedKph = std::clamp(speed, 1.0, 300.0);
}

void Scene3D::setPreview(bool enabled) {
    if (enabled) {
        d_->elapsed.start();
        d_->timer.start();
    } else
        d_->timer.stop();
    update();
    d_->fallback->update();
}

void Scene3D::zoomBy(double factor) {
    if (!std::isfinite(factor) || factor <= 0)
        return;
    d_->distance = std::clamp(d_->distance / factor, 1.4, 4000000.0);
    d_->matrices();
    if (onZoom)
        onZoom(d_->fittedDistance / d_->distance * 100);
    update();
    d_->fallback->update();
}

QImage Scene3D::exportImage() {
    if (isValid() && d_->glReady && !d_->offscreen)
        return grabFramebuffer();
    QImage image(size() * 2, QImage::Format_ARGB32_Premultiplied);
    image.fill(Qt::white);
    QPainter painter(&image);
    painter.scale(2, 2);
    d_->paintSoftware(painter);
    return image;
}

void Scene3D::initializeGL() {
    initializeOpenGLFunctions();
    d_->program = std::make_unique<QOpenGLShaderProgram>();
    const char *vertexShader =
        "#version 120\nattribute vec3 position; attribute vec3 color; uniform mat4 matrix;"
        "varying vec3 vertexColor; void "
        "main(){vertexColor=color;gl_Position=matrix*vec4(position,1.0);}";
    const char *fragmentShader = "#version 120\nvarying vec3 vertexColor; uniform float opacity;"
                                 "void main(){gl_FragColor=vec4(vertexColor,opacity);}";
    if (!d_->program->addShaderFromSourceCode(QOpenGLShader::Vertex, vertexShader) ||
        !d_->program->addShaderFromSourceCode(QOpenGLShader::Fragment, fragmentShader)) {
        d_->glError = d_->program->log();
        d_->fallback->show();
        return;
    }
    d_->program->bindAttributeLocation("position", 0);
    d_->program->bindAttributeLocation("color", 1);
    if (!d_->program->link()) {
        d_->glError = d_->program->log();
        d_->fallback->show();
        return;
    }
    for (auto *batch : d_->batches)
        batch->buffer.create();
    d_->dirty = true;
    d_->glReady = true;
    d_->fallback->hide();
    QObject::connect(context(), &QOpenGLContext::aboutToBeDestroyed, this, [this] {
        makeCurrent();
        d_->releaseGl();
        doneCurrent();
    });
}

void Scene3D::resizeGL(int, int) {
    d_->matrices();
}

void Scene3D::paintGL() {
    if (!d_->glReady)
        return;
    d_->matrices();
    glViewport(0, 0, qRound(width() * devicePixelRatioF()), qRound(height() * devicePixelRatioF()));
    glClearColor(.98f, .98f, .98f, 1);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDisable(GL_CULL_FACE);
    if (d_->dirty) {
        for (auto *batch : d_->batches) {
            batch->buffer.bind();
            batch->buffer.allocate(batch->vertices.data(),
                                   static_cast<int>(batch->vertices.size() * sizeof(Vertex)));
            batch->buffer.release();
        }
        d_->dirty = false;
        d_->ghostDirty = false;
    } else if (d_->ghostDirty) {
        d_->ghostBatch.buffer.bind();
        d_->ghostBatch.buffer.allocate(
            d_->ghostBatch.vertices.data(),
            static_cast<int>(d_->ghostBatch.vertices.size() * sizeof(Vertex)));
        d_->ghostBatch.buffer.release();
        d_->ghostDirty = false;
    }
    const auto draw = [this](Batch &batch, float width = 1) {
        if (batch.vertices.empty())
            return;
        d_->program->bind();
        d_->program->setUniformValue("matrix", d_->viewProjection);
        d_->program->setUniformValue("opacity", batch.opacity);
        batch.buffer.bind();
        d_->program->enableAttributeArray(0);
        d_->program->enableAttributeArray(1);
        d_->program->setAttributeBuffer(0, GL_FLOAT, offsetof(Vertex, x), 3, sizeof(Vertex));
        d_->program->setAttributeBuffer(1, GL_FLOAT, offsetof(Vertex, r), 3, sizeof(Vertex));
        glLineWidth(width);
        glDrawArrays(batch.mode, 0, static_cast<GLsizei>(batch.vertices.size()));
        d_->program->disableAttributeArray(0);
        d_->program->disableAttributeArray(1);
        batch.buffer.release();
        d_->program->release();
    };
    draw(d_->ground);
    draw(d_->shadows);
    draw(d_->grid);
    if (!d_->halo.vertices.empty()) {
        glDepthMask(GL_FALSE);
        draw(d_->halo);
        glDepthMask(GL_TRUE);
    }
    if (d_->routeVisible)
        draw(d_->projection);
    draw(d_->solids);
    if (d_->routeVisible) {
        draw(d_->routeTube);
        draw(d_->routeLine, 1.6f);
    }
    draw(d_->selection, 2);
    if (!d_->ghostBatch.vertices.empty()) {
        glDepthMask(GL_FALSE);
        draw(d_->ghostBatch);
        glDepthMask(GL_TRUE);
    }
    if (d_->timer.isActive() && !d_->route.empty()) {
        d_->preview.vertices.clear();
        const auto center = d_->previewPoint();
        const float r = .33f;
        const std::array<QVector3D, 4> ring{
            {center + QVector3D(r, 0, 0), center + QVector3D(0, 0, r), center + QVector3D(-r, 0, 0),
             center + QVector3D(0, 0, -r)}};
        for (int i = 0; i < 4; ++i) {
            triangle(d_->preview, center + QVector3D(0, r, 0), ring[i], ring[(i + 1) % 4],
                     QColor("#444444"));
            triangle(d_->preview, center - QVector3D(0, r, 0), ring[(i + 1) % 4], ring[i],
                     QColor("#1f1f1f"));
        }
        d_->preview.buffer.bind();
        d_->preview.buffer.allocate(d_->preview.vertices.data(),
                                    static_cast<int>(d_->preview.vertices.size() * sizeof(Vertex)));
        d_->preview.buffer.release();
        draw(d_->preview);
    }
    glLineWidth(1);
    glDisable(GL_DEPTH_TEST);
    QPainter painter(this);
    d_->labels(painter, false);
}

void Scene3D::paintEvent(QPaintEvent *event) {
    if (d_->offscreen) {
        d_->fallback->show();
        d_->fallback->update();
        return;
    }
    QOpenGLWidget::paintEvent(event);
    if (!isValid() || !d_->glReady) {
        d_->fallback->show();
        d_->fallback->update();
    }
}

void Scene3D::resizeEvent(QResizeEvent *event) {
    QOpenGLWidget::resizeEvent(event);
    d_->fallback->setGeometry(rect());
    d_->matrices();
    if (d_->firstFit)
        QTimer::singleShot(0, this, [this] { fitTrack(); });
}

void Scene3D::wheelEvent(QWheelEvent *event) {
    zoomBy(std::pow(1.0014, event->angleDelta().y()));
    event->accept();
}

void Scene3D::mousePressEvent(QMouseEvent *event) {
    setFocus(Qt::MouseFocusReason);
    d_->matrices();
    d_->lastMouse = d_->pressMouse = event->position();
    if (event->button() == Qt::MiddleButton || (d_->space && event->button() == Qt::LeftButton)) {
        d_->gesture = Impl::Gesture::Pan;
        setCursor(Qt::ClosedHandCursor);
    } else if (event->button() == Qt::RightButton) {
        d_->gesture = Impl::Gesture::Orbit;
        setCursor(Qt::ClosedHandCursor);
    } else if (event->button() == Qt::LeftButton && !d_->placement.isEmpty()) {
        QPointF point;
        if (d_->intersect(event->position(), 0, &point) && onPlace)
            onPlace(d_->placement, d_->snapped(point), d_->placeWidth, d_->placeHeight);
    } else if (event->button() == Qt::LeftButton) {
        const QString routeId = d_->pickRoute(event->position());
        if (!routeId.isEmpty()) {
            selectRouteStep(routeId);
            d_->gesture = Impl::Gesture::SelectOnly;
            setCursor(Qt::PointingHandCursor);
            event->accept();
            return;
        }
        const QString id = d_->pick(event->position());
        const bool alreadySelected = !id.isEmpty() && id == d_->selected;
        selectGate(id);
        if (const auto *item = d_->gate(id)) {
            d_->gesture = Impl::Gesture::SelectOnly;
            QPointF point;
            d_->dragHeight = item->elevation;
            if (alreadySelected && d_->intersect(event->position(), d_->dragHeight, &point)) {
                d_->dragOffset = item->position - point;
                d_->gesture = Impl::Gesture::GateDragPending;
            }
            setCursor(Qt::OpenHandCursor);
        } else {
            d_->gesture = Impl::Gesture::Orbit;
            setCursor(Qt::ClosedHandCursor);
        }
    }
    event->accept();
}

void Scene3D::mouseMoveEvent(QMouseEvent *event) {
    d_->matrices();
    const QPointF delta = event->position() - d_->lastMouse;
    if (d_->gesture == Impl::Gesture::Orbit) {
        d_->azimuth -= static_cast<float>(delta.x() * .38);
        d_->pitch = std::clamp(d_->pitch + static_cast<float>(delta.y() * .30), 7.0f, 87.0f);
        d_->matrices();
        update();
        d_->fallback->update();
    } else if (d_->gesture == Impl::Gesture::Pan) {
        QPointF before, after;
        if (d_->intersect(d_->lastMouse, d_->target.y(), &before) &&
            d_->intersect(event->position(), d_->target.y(), &after)) {
            d_->target += QVector3D(static_cast<float>(before.x() - after.x()), 0,
                                    static_cast<float>(before.y() - after.y()));
            d_->matrices();
            update();
            d_->fallback->update();
        }
    } else if (d_->gesture == Impl::Gesture::GateDragPending ||
               d_->gesture == Impl::Gesture::GateDrag) {
        if (d_->gesture == Impl::Gesture::GateDragPending) {
            const auto movement = event->position() - d_->pressMouse;
            if (std::hypot(movement.x(), movement.y()) < 3) {
                event->accept();
                return;
            }
            d_->gesture = Impl::Gesture::GateDrag;
            if (onGestureBegin)
                onGestureBegin();
            setCursor(Qt::SizeAllCursor);
        }
        QPointF point;
        if (d_->intersect(event->position(), d_->dragHeight, &point)) {
            point = d_->snapped(point + d_->dragOffset);
            if (onGateMoved)
                onGateMoved(d_->selected, point);
            else if (d_->document)
                for (auto &item : d_->document->gates)
                    if (item.id == d_->selected)
                        item.position = point;
            refreshGeometry();
        }
    } else if (d_->gesture == Impl::Gesture::SelectOnly) {
        // The entire first press belongs to selection, even if the pointer
        // moves afterwards. Only a later press may begin a movement gesture.
    } else if (!d_->placement.isEmpty()) {
        QPointF point;
        d_->haveGhost = d_->intersect(event->position(), 0, &point);
        if (d_->haveGhost)
            d_->ghost = d_->snapped(point);
        d_->updateGhost();
        update();
        d_->fallback->update();
    } else {
        const auto stepId = d_->pickRoute(event->position());
        setCursor(!stepId.isEmpty()                       ? Qt::PointingHandCursor
                  : d_->pick(event->position()).isEmpty() ? Qt::ArrowCursor
                                                          : Qt::OpenHandCursor);
        QString tooltip;
        for (const auto &visual : d_->routeVisuals)
            if (visual.step.id == stepId) {
                tooltip = QStringLiteral("过门 %1 · 实体 G%2 · 第 %3 次穿越")
                              .arg(visual.number)
                              .arg(visual.gateNumber)
                              .arg(visual.visit);
                break;
            }
        setToolTip(tooltip);
    }
    d_->lastMouse = event->position();
    event->accept();
}

void Scene3D::mouseReleaseEvent(QMouseEvent *event) {
    const bool movedGate = d_->gesture == Impl::Gesture::GateDrag;
    d_->gesture = Impl::Gesture::Idle;
    setCursor(d_->placement.isEmpty() ? Qt::ArrowCursor : Qt::CrossCursor);
    if (movedGate && onGestureEnd)
        onGestureEnd();
    event->accept();
}

void Scene3D::keyPressEvent(QKeyEvent *event) {
    if (event->key() == Qt::Key_Space) {
        d_->space = true;
        setCursor(Qt::OpenHandCursor);
        event->accept();
        return;
    }
    if (event->key() == Qt::Key_Escape) {
        setPlacement({});
        event->accept();
        return;
    }
    if (event->key() == Qt::Key_F) {
        fitTrack();
        event->accept();
        return;
    }
    QOpenGLWidget::keyPressEvent(event);
}

void Scene3D::keyReleaseEvent(QKeyEvent *event) {
    if (event->key() == Qt::Key_Space) {
        d_->space = false;
        setCursor(d_->placement.isEmpty() ? Qt::ArrowCursor : Qt::CrossCursor);
        event->accept();
        return;
    }
    QOpenGLWidget::keyReleaseEvent(event);
}

void Scene3D::leaveEvent(QEvent *event) {
    if (d_->gesture == Impl::Gesture::Idle && d_->haveGhost) {
        d_->haveGhost = false;
        d_->updateGhost();
        update();
        d_->fallback->update();
    }
    QOpenGLWidget::leaveEvent(event);
}
} // namespace ot
