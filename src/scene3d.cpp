#include "scene3d.h"
#include "document.h"

#include <QElapsedTimer>
#include <QGuiApplication>
#include <QKeyEvent>
#include <QLinearGradient>
#include <QMatrix4x4>
#include <QMouseEvent>
#include <QOpenGLBuffer>
#include <QOpenGLContext>
#include <QOpenGLShaderProgram>
#include <QOpenGLTexture>
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
namespace {
constexpr double Pi = 3.14159265358979323846;
constexpr float FieldOfView = 42.0f;
struct Vertex {
    float x, y, z, r, g, b;
};
struct TextureVertex {
    float x, y, z, u, v;
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

QVector3D passage3(const Gate &gate) {
    const QPointF passage = gatePassagePosition(gate);
    return {static_cast<float>(passage.x()), static_cast<float>(gate.elevation),
            static_cast<float>(passage.y())};
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
    QString selected, placement;
    double placeWidth = 2.4, placeHeight = 2.4;
    bool snap = true, routeVisible = true, space = false, firstFit = true;
    bool glReady = false, dirty = true, textureDirty = true, ghostDirty = true, haveGhost = false;
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
    std::unique_ptr<QOpenGLShaderProgram> program, textureProgram;
    std::unique_ptr<QOpenGLTexture> texture;
    QOpenGLBuffer textureBuffer{QOpenGLBuffer::VertexBuffer};
    qint64 backgroundCacheKey = 0;
    FallbackCanvas *fallback = nullptr;
    QTimer timer;
    QElapsedTimer elapsed;
    double speedKph = 70, travel = 0;
    std::vector<QVector3D> route;
    std::vector<double> routeLengths;

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
            const float w = static_cast<float>(item.width / 2),
                        h = static_cast<float>(item.height / 2);
            const std::array<QVector3D, 4> corners{
                {gatePoint(item, {-w, -h, 0}), gatePoint(item, {w, -h, 0}),
                 gatePoint(item, {w, h, 0}), gatePoint(item, {-w, h, 0})}};
            QPolygonF polygon;
            QPointF center;
            double depth = 0;
            if (!project(point3(item), &center, &depth))
                continue;
            for (const auto &corner : corners) {
                QPointF point;
                if (project(corner, &point))
                    polygon.append(point);
            }
            if (polygon.size() != 4)
                continue;
            double hitDistance = std::numeric_limits<double>::max();
            for (int i = 0; i < 4; ++i)
                hitDistance = std::min(hitDistance,
                                       segmentDistance(position, polygon[i], polygon[(i + 1) % 4]));
            const double projectedSpan =
                std::max(polygon.boundingRect().width(), polygon.boundingRect().height());
            const double threshold = std::clamp(projectedSpan * .15, 6.0, 14.0);
            const bool inside = polygon.containsPoint(position, Qt::OddEvenFill);
            QPointF label;
            const bool hasLabel = project(point3(item) + QVector3D(0, h, 0), &label);
            label.ry() -= 14;
            const bool labelHit =
                hasLabel && std::hypot(position.x() - label.x(), position.y() - label.y()) < 11;
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
        QRectF bounds;
        bool hasBounds = false;
        if (document) {
            for (const auto &item : document->gates) {
                const double half = std::max(2.0, gateFrameMetrics(item).outerWidth / 2 + .5);
                const QRectF one(item.position.x() - half, item.position.y() - half, half * 2,
                                 half * 2);
                bounds = hasBounds ? bounds.united(one) : one;
                hasBounds = true;
                if (item.type == QStringLiteral("pillar")) {
                    const QPointF passage = gatePassagePosition(item);
                    bounds = bounds.united(QRectF(passage - QPointF(1.5, 1.5), QSizeF(3, 3)));
                }
            }
            if (!document->background.isNull() && document->showBackground) {
                const QRectF image(
                    document->backgroundOrigin,
                    QSizeF(document->background.width() * document->metersPerPixel,
                           document->background.height() * document->metersPerPixel));
                bounds = hasBounds ? bounds.united(image) : image;
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
        double step = document ? std::max(.05, document->gridSize) : 1;
        while (groundBounds.width() / step + groundBounds.height() / step > 600)
            step *= 2;
        const double beginX = std::ceil(groundBounds.left() / step) * step;
        const double beginZ = std::ceil(groundBounds.top() / step) * step;
        for (double x = beginX; x <= groundBounds.right(); x += step) {
            const bool major = std::abs(std::remainder(x / step, 5.0)) < .001;
            line(grid, {static_cast<float>(x), .005f, z0}, {static_cast<float>(x), .005f, z1},
                 major ? QColor("#c9c9c9") : QColor("#dedede"));
        }
        for (double z = beginZ; z <= groundBounds.bottom(); z += step) {
            const bool major = std::abs(std::remainder(z / step, 5.0)) < .001;
            line(grid, {x0, .005f, static_cast<float>(z)}, {x1, .005f, static_cast<float>(z)},
                 major ? QColor("#c9c9c9") : QColor("#dedede"));
        }
        const QColor border("#bcbcbc");
        line(grid, {x0, .01f, z0}, {x1, .01f, z0}, border);
        line(grid, {x1, .01f, z0}, {x1, .01f, z1}, border);
        line(grid, {x1, .01f, z1}, {x0, .01f, z1}, border);
        line(grid, {x0, .01f, z1}, {x0, .01f, z0}, border);
        if (document) {
            for (const auto &item : document->gates) {
                buildGate(solids, item, item.id == selected);
                const float w = static_cast<float>(std::max(item.width * .65, .7));
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
                triangle(solids, arrowBase + direction * 1.15f,
                         arrowBase + direction * .65f + side * .19f,
                         arrowBase + direction * .65f - side * .19f, arrowColor);
                if (item.id == selected) {
                    const auto metrics = gateFrameMetrics(item);
                    const float halfW = static_cast<float>(metrics.outerWidth / 2 + .15);
                    const float halfH = static_cast<float>(metrics.outerHeight / 2 + .15);
                    const auto a0 = gatePoint(item, {-halfW, -halfH, -.015f});
                    const auto b0 = gatePoint(item, {halfW, -halfH, -.015f});
                    const auto c0 = gatePoint(item, {halfW, halfH, -.015f});
                    const auto d0 = gatePoint(item, {-halfW, halfH, -.015f});
                    const QColor selectionColor("#161616");
                    line(selection, a0, b0, selectionColor);
                    line(selection, b0, c0, selectionColor);
                    line(selection, c0, d0, selectionColor);
                    line(selection, d0, a0, selectionColor);
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
            const qsizetype count = document->gates.size();
            if (count > 1) {
                const qsizetype segments = document->closedLoop ? count : count - 1;
                const int samples =
                    std::clamp(12000 / static_cast<int>(std::max(qsizetype(1), segments)), 3, 28);
                const auto at = [this, count](qsizetype index) {
                    if (document->closedLoop)
                        index = (index % count + count) % count;
                    else
                        index = std::clamp(index, qsizetype(0), count - 1);
                    return passage3(document->gates[index]);
                };
                for (qsizetype segment = 0; segment < segments; ++segment) {
                    const auto p0 = at(segment - 1), p1 = at(segment), p2 = at(segment + 1),
                               p3 = at(segment + 2);
                    for (int j = 0; j < samples; ++j) {
                        const float t = static_cast<float>(j) / static_cast<float>(samples);
                        const float t2 = t * t, t3 = t2 * t;
                        const QVector3D point =
                            ((p1 * 2) + (p2 - p0) * t + (p0 * 2 - p1 * 5 + p2 * 4 - p3) * t2 +
                             (-p0 + p1 * 3 - p2 * 3 + p3) * t3) *
                            .5f;
                        route.push_back(point);
                    }
                }
                route.push_back(at(segments));
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
        dirty = true;
        if (document && backgroundCacheKey != document->background.cacheKey()) {
            backgroundCacheKey = document->background.cacheKey();
            textureDirty = true;
        }
    }

    void updateGhost() {
        ghostBatch.vertices.clear();
        if (!placement.isEmpty() && haveGhost) {
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
        texture.reset();
        program.reset();
        textureProgram.reset();
        for (auto *batch : batches)
            batch->buffer.destroy();
        textureBuffer.destroy();
        glReady = false;
        dirty = textureDirty = true;
    }

    void labels(QPainter &painter, bool software) {
        painter.setRenderHint(QPainter::Antialiasing);
        QFont number(QStringLiteral("Segoe UI"), 8);
        number.setWeight(QFont::DemiBold);
        painter.setFont(number);
        if (document && document->gates.size() < 400) {
            for (qsizetype i = 0; i < document->gates.size(); ++i) {
                const auto &item = document->gates[i];
                QPointF point;
                if (!project(point3(item) + QVector3D(0, static_cast<float>(item.height / 2), 0),
                             &point))
                    continue;
                point.ry() -= 14;
                if (point.x() < 12 || point.x() > q->width() - 12 || point.y() < 12 ||
                    point.y() > q->height() - 55)
                    continue;
                const bool active = item.id == selected;
                const double badgeWidth = i >= 99 ? 25 : 19;
                const QRectF badge(point.x() - badgeWidth / 2, point.y() - 8, badgeWidth, 16);
                painter.setPen(QPen(active ? QColor("#111111") : QColor("#bfbfbf"), 1));
                painter.setBrush(active ? QColor("#222222") : QColor(255, 255, 255, 238));
                painter.drawRoundedRect(badge, 5, 5);
                painter.setPen(active ? Qt::white : QColor("#303030"));
                painter.drawText(badge, Qt::AlignCenter, QString::number(i + 1));
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
            painter.setPen(color);
            painter.drawText(QRectF(end.x() - 6, end.y() - 10, 20, 16), Qt::AlignCenter, name);
        };
        painter.setFont(QFont(QStringLiteral("Segoe UI"), 8, QFont::DemiBold));
        axis({1, 0, 0}, QColor("#282828"), QStringLiteral("X"));
        axis({0, 1, 0}, QColor("#777777"), QStringLiteral("Y"));
        axis({0, 0, 1}, QColor("#a0a0a0"), QStringLiteral("Z"));
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
        if (document && document->showBackground && !document->background.isNull()) {
            const auto &image = document->background;
            const QPointF origin = document->backgroundOrigin;
            const double x0 = origin.x(), z0 = origin.y();
            const double x1 = x0 + image.width() * document->metersPerPixel;
            const double z1 = z0 + image.height() * document->metersPerPixel;
            const QPolygonF source({QPointF(0, 0), QPointF(image.width(), 0),
                                    QPointF(image.width(), image.height()),
                                    QPointF(0, image.height())});
            QPolygonF destination;
            for (const auto &world :
                 {QVector3D(static_cast<float>(x0), 0, static_cast<float>(z0)),
                  QVector3D(static_cast<float>(x1), 0, static_cast<float>(z0)),
                  QVector3D(static_cast<float>(x1), 0, static_cast<float>(z1)),
                  QVector3D(static_cast<float>(x0), 0, static_cast<float>(z1))}) {
                QPointF screen;
                if (project(world, &screen))
                    destination.append(screen);
            }
            QTransform transform;
            if (destination.size() == 4 && QTransform::quadToQuad(source, destination, transform)) {
                painter.save();
                painter.setOpacity(document->backgroundOpacity);
                painter.setTransform(transform);
                painter.drawImage(QPointF(0, 0), image);
                painter.restore();
            }
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
    const QRectF bounds = d_->contentBounds.adjusted(-1.5, -1.5, 1.5, 1.5);
    float maxY = 2.4f, minY = 0;
    std::vector<QVector3D> fitPoints;
    if (d_->document) {
        for (const auto &gate : d_->document->gates) {
            maxY = std::max(maxY, static_cast<float>(gate.elevation + gate.height / 2));
            minY = std::min(minY, static_cast<float>(gate.elevation - gate.height / 2));
            const auto metrics = gateFrameMetrics(gate);
            const float halfWidth = static_cast<float>(metrics.outerWidth / 2 + .15);
            const float halfHeight = static_cast<float>(metrics.outerHeight / 2 + .15);
            for (const float x : {-halfWidth, halfWidth})
                for (const float y : {-halfHeight, halfHeight})
                    fitPoints.push_back(gatePoint(gate, {x, y, 0}));
        }
        const auto &doc = *d_->document;
        if (doc.showBackground && !doc.background.isNull()) {
            const double x0 = doc.backgroundOrigin.x(), z0 = doc.backgroundOrigin.y();
            const double x1 = x0 + doc.background.width() * doc.metersPerPixel;
            const double z1 = z0 + doc.background.height() * doc.metersPerPixel;
            for (const double x : {x0, x1})
                for (const double z : {z0, z1})
                    fitPoints.emplace_back(static_cast<float>(x), 0, static_cast<float>(z));
        }
    }
    for (size_t i = 0; i < d_->route.size(); i += 4)
        fitPoints.push_back(d_->route[i]);
    if (fitPoints.empty())
        for (const double x : {bounds.left(), bounds.right()})
            for (const double z : {bounds.top(), bounds.bottom()})
                fitPoints.emplace_back(static_cast<float>(x), 0, static_cast<float>(z));
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
    d_->textureProgram = std::make_unique<QOpenGLShaderProgram>();
    const char *textureVertex = "#version 120\nattribute vec3 position;attribute vec2 uv;uniform "
                                "mat4 matrix;varying vec2 texCoord;"
                                "void main(){texCoord=uv;gl_Position=matrix*vec4(position,1.0);}";
    const char *textureFragment =
        "#version 120\nvarying vec2 texCoord;uniform sampler2D image;uniform float opacity;"
        "void main(){vec4 "
        "sampleColor=texture2D(image,texCoord);gl_FragColor=vec4(sampleColor.rgb,sampleColor.a*"
        "opacity);}";
    if (!d_->textureProgram->addShaderFromSourceCode(QOpenGLShader::Vertex, textureVertex) ||
        !d_->textureProgram->addShaderFromSourceCode(QOpenGLShader::Fragment, textureFragment)) {
        d_->textureProgram.reset();
    } else {
        d_->textureProgram->bindAttributeLocation("position", 0);
        d_->textureProgram->bindAttributeLocation("uv", 1);
        if (!d_->textureProgram->link())
            d_->textureProgram.reset();
    }
    for (auto *batch : d_->batches)
        batch->buffer.create();
    d_->textureBuffer.create();
    d_->dirty = d_->textureDirty = true;
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
    if (d_->document && d_->document->showBackground && !d_->document->background.isNull() &&
        d_->textureProgram) {
        if (d_->textureDirty) {
            QImage image = d_->document->background;
            if (image.width() > 4096 || image.height() > 4096)
                image = image.scaled(4096, 4096, Qt::KeepAspectRatio, Qt::SmoothTransformation);
            image = image.convertToFormat(QImage::Format_RGBA8888).mirrored();
            d_->texture =
                std::make_unique<QOpenGLTexture>(image, QOpenGLTexture::DontGenerateMipMaps);
            d_->texture->setMinMagFilters(QOpenGLTexture::Linear, QOpenGLTexture::Linear);
            d_->texture->setWrapMode(QOpenGLTexture::ClampToEdge);
            d_->textureDirty = false;
        }
        const auto &doc = *d_->document;
        const float x0 = static_cast<float>(doc.backgroundOrigin.x()),
                    z0 = static_cast<float>(doc.backgroundOrigin.y());
        const float x1 = x0 + static_cast<float>(doc.background.width() * doc.metersPerPixel);
        const float z1 = z0 + static_cast<float>(doc.background.height() * doc.metersPerPixel);
        const std::array<TextureVertex, 6> vertices{{{x0, -.025f, z0, 0, 1},
                                                     {x1, -.025f, z0, 1, 1},
                                                     {x1, -.025f, z1, 1, 0},
                                                     {x0, -.025f, z0, 0, 1},
                                                     {x1, -.025f, z1, 1, 0},
                                                     {x0, -.025f, z1, 0, 0}}};
        d_->textureBuffer.bind();
        d_->textureBuffer.allocate(vertices.data(),
                                   static_cast<int>(vertices.size() * sizeof(TextureVertex)));
        d_->textureProgram->bind();
        d_->textureProgram->setUniformValue("matrix", d_->viewProjection);
        d_->textureProgram->setUniformValue("opacity", static_cast<float>(doc.backgroundOpacity));
        d_->textureProgram->setUniformValue("image", 0);
        d_->texture->bind(0);
        d_->textureProgram->enableAttributeArray(0);
        d_->textureProgram->enableAttributeArray(1);
        d_->textureProgram->setAttributeBuffer(0, GL_FLOAT, offsetof(TextureVertex, x), 3,
                                               sizeof(TextureVertex));
        d_->textureProgram->setAttributeBuffer(1, GL_FLOAT, offsetof(TextureVertex, u), 2,
                                               sizeof(TextureVertex));
        glDrawArrays(GL_TRIANGLES, 0, 6);
        d_->textureProgram->disableAttributeArray(0);
        d_->textureProgram->disableAttributeArray(1);
        d_->texture->release();
        d_->textureProgram->release();
        d_->textureBuffer.release();
    }
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
        setCursor(d_->pick(event->position()).isEmpty() ? Qt::ArrowCursor : Qt::OpenHandCursor);
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
