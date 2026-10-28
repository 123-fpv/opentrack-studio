#include "canvas.h"
#include "language.h"
#include <QGraphicsObject>
#include <QGraphicsScene>
#include <QGraphicsSceneMouseEvent>
#include <QMouseEvent>
#include <QPainter>
#include <QPolygonF>
#include <QRadialGradient>
#include <QScrollBar>
#include <QStyleOptionGraphicsItem>
#include <QWheelEvent>
#include <QtMath>
#include <algorithm>
#include <cmath>

namespace ot {
void paintGateSymbol(QPainter *p, const QRectF &b, const QString &type, bool selected,
                     const QColor &tint) {
    p->save();
    p->setRenderHint(QPainter::Antialiasing);
    p->translate(b.center());
    p->scale(b.width() / 64., b.height() / 56.);
    const QColor c = tint;
    if (selected) {
        p->setPen(QPen(QColor(25, 25, 25, 175), 1.2, Qt::DashLine));
        p->setBrush(QColor(255, 255, 255, 170));
        p->drawRoundedRect(QRectF(-31, -27, 62, 54), 6, 6);
    }
    p->setPen(Qt::NoPen);
    p->setBrush(QColor(35, 35, 35, 12));
    p->drawEllipse(type == "pillar" ? QRectF(-6, 19, 12, 4) : QRectF(-25, 17, 52, 9));
    if (type == "pillar") {
        // A solid square post, with the same silhouette as a stretched native cube.
        p->setPen(QPen(c, 1.1, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
        p->setBrush(c.darker(130));
        p->drawPolygon(QPolygonF(
            {QPointF(1.4, -22), QPointF(2.7, -23.8), QPointF(2.7, 19.2), QPointF(1.4, 21)}));
        p->setBrush(c.lighter(150));
        p->drawPolygon(QPolygonF(
            {QPointF(-1.4, -22), QPointF(-.1, -23.8), QPointF(2.7, -23.8), QPointF(1.4, -22)}));
        p->setBrush(c);
        p->drawRect(QRectF(-1.4, -22, 2.8, 43));
        p->setPen(QPen(QColor("#8f8f8f"), 1.4, Qt::SolidLine, Qt::RoundCap));
        p->drawLine(QPointF(-4, 23), QPointF(4, 23));
    } else if (type == "flag") {
        p->setPen(QPen(QColor("#555555"), 2.6, Qt::SolidLine, Qt::RoundCap));
        p->drawLine(QPointF(-5, 21), QPointF(-5, -22));
        QPainterPath f;
        f.moveTo(-4, -21);
        f.cubicTo(12, -27, 25, -8, 10, 3);
        f.lineTo(-4, 3);
        f.closeSubpath();
        p->setBrush(c);
        p->setPen(QPen(c.darker(112), 1));
        p->drawPath(f);
        p->setPen(QPen(QColor(255, 255, 255, 180), 1.6));
        p->drawLine(QPointF(1, -16), QPointF(12, -11));
    } else if (type == "double" || type == "triple" || type == "ladder" || type == "cube" ||
               type == "cube-stack") {
        const bool cube = type == "cube" || type == "cube-stack";
        const int columns = type == "double" ? 2 : (type == "triple" ? 3 : 1);
        const int levels = type == "ladder" || type == "cube-stack" ? 2 : 1;
        const qreal cell = cube ? (levels == 2 ? 21. : 31.) : (columns > 1 ? 50. / columns : 21.);
        const QPointF depth = cube ? QPointF(13, -9) : QPointF();
        const QRectF front(-columns * cell / 2 - depth.x() / 2, -levels * cell / 2 - depth.y() / 2,
                           columns * cell, levels * cell);
        const auto grid = [p, columns, levels](const QRectF &face, const QColor &color,
                                               qreal stroke) {
            p->setPen(QPen(color, stroke, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
            p->setBrush(QColor(color.red(), color.green(), color.blue(), 10));
            p->drawRect(face);
            for (int column = 1; column < columns; ++column) {
                const qreal x = face.left() + face.width() * column / columns;
                p->drawLine(QPointF(x, face.top()), QPointF(x, face.bottom()));
            }
            for (int level = 1; level < levels; ++level) {
                const qreal y = face.top() + face.height() * level / levels;
                p->drawLine(QPointF(face.left(), y), QPointF(face.right(), y));
            }
        };
        if (cube) {
            grid(front.translated(depth), c.lighter(155), 2.0);
            p->setPen(QPen(c.lighter(125), 2.0, Qt::SolidLine, Qt::RoundCap));
            for (int level = 0; level <= levels; ++level) {
                const qreal y = front.top() + front.height() * level / levels;
                for (const qreal x : {front.left(), front.right()})
                    p->drawLine(QPointF(x, y), QPointF(x, y) + depth);
            }
        }
        grid(front, c, 2.8);
    } else {
        const QRectF frame(-24, -20, 43, 36);
        const QPointF d(8, -5);
        p->setPen(QPen(c.lighter(145), 2.2, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
        p->setBrush(QColor(c.red(), c.green(), c.blue(), 12));
        p->drawRect(frame.translated(d));
        for (const QPointF v :
             {frame.topLeft(), frame.topRight(), frame.bottomLeft(), frame.bottomRight()})
            p->drawLine(v, v + d);
        p->setPen(QPen(c, 3, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
        p->setBrush(QColor(c.red(), c.green(), c.blue(), 14));
        p->drawRect(frame);
        p->setPen(QPen(QColor(255, 255, 255, 200), 1));
        p->drawLine(frame.topLeft() + QPointF(2, 2), frame.topRight() + QPointF(-2, 2));
        if (type == "start") {
            p->setPen(Qt::NoPen);
            for (int i = 0; i < 7; ++i) {
                p->setBrush(i % 2 ? Qt::white : c.darker(125));
                p->drawRect(QRectF(-22 + i * 6, -19, 6, 5));
            }
        }
    }
    p->restore();
}
class GateItem final : public QGraphicsObject {
  public:
    Gate gate;
    TrackCanvas *canvas;
    bool dragging = false, moveArmed = false;
    int ordinal = 0;
    QPoint pressScreen;
    QPointF pressScene, initialPosition;
    GateItem(const Gate &g, TrackCanvas *v, int index) : gate(g), canvas(v), ordinal(index) {
        setFlags(ItemIsSelectable | ItemSendsGeometryChanges);
        setAcceptHoverEvents(true);
        setPos(g.position);
        setRotation(-g.yaw);
        setZValue(10);
        setToolTip(gateTypeName(g.type) + " · " + g.label);
    }
    QRectF boundingRect() const override {
        const double w = std::max(2.4, gate.width);
        return QRectF(-w * .82, -2.5, w * 1.64, 5.1);
    }
    QPainterPath shape() const override {
        QPainterPath p;
        p.addRoundedRect(boundingRect().adjusted(.1, .1, -.1, -.1), .2, .2);
        return p;
    }
    void paint(QPainter *p, const QStyleOptionGraphicsItem *, QWidget *) override {
        if (isSelected()) {
            p->save();
            const double radius = std::max(2.4, gate.width) * .77;
            p->translate(0, -.25);
            p->scale(radius, 2.1);
            QRadialGradient glow(QPointF(0, 0), 1.0);
            glow.setColorAt(0, QColor(255, 255, 255, 0));
            glow.setColorAt(.64, QColor(255, 255, 255, 0));
            glow.setColorAt(.83, QColor(255, 255, 255, 200));
            glow.setColorAt(1, QColor(255, 255, 255, 0));
            p->setPen(Qt::NoPen);
            p->setBrush(glow);
            p->drawEllipse(QPointF(0, 0), 1, 1);
            QPen ring(QColor(22, 22, 22, 220), 1.5);
            ring.setCosmetic(true);
            p->setPen(ring);
            p->setBrush(Qt::NoBrush);
            p->drawEllipse(QPointF(0, 0), .83, .83);
            p->restore();
        }
        const double w = std::max(1.6, gate.width);
        paintGateSymbol(p, QRectF(-w * .65, -1.8, w * 1.3, 2.9), gate.type, isSelected(),
                        gateColor(gate));
        p->setPen(QPen(isSelected()
                           ? QColor("#222222")
                           : (gate.type == "start" ? QColor("#333333") : QColor("#7a7a7a")),
                       .07, Qt::SolidLine, Qt::RoundCap));
        p->drawLine(QPointF(0, 1.2), QPointF(0, 1.8));
        p->drawLine(QPointF(0, 1.8), QPointF(-.19, 1.57));
        p->drawLine(QPointF(0, 1.8), QPointF(.19, 1.57));
        p->save();
        p->rotate(gate.yaw);
        p->setBrush(isSelected() ? QColor("#222222") : QColor("#ffffff"));
        p->setPen(QPen(QColor("#bfbfbf"), .06));
        p->drawRoundedRect(QRectF(-.8, -.43, 1.6, .86), .26, .26);
        QFont f("Segoe UI");
        f.setPixelSize(12);
        p->setFont(f);
        p->scale(.07, .07);
        p->setPen(isSelected() ? Qt::white : QColor("#333333"));
        p->drawText(QRectF(-11.5, -6, 23, 12), Qt::AlignCenter, QString("G%1").arg(ordinal + 1));
        p->restore();
    }
    QVariant itemChange(GraphicsItemChange c, const QVariant &value) override {
        if (c == ItemPositionChange && dragging && canvas->snap_) {
            QPointF p = value.toPointF();
            return QPointF(qRound(p.x() * 2) / 2., qRound(p.y() * 2) / 2.);
        }
        if (c == ItemPositionHasChanged && dragging) {
            gate.position = pos();
            if (canvas->onGateMoved)
                canvas->onGateMoved(gate.id, pos());
            canvas->refreshRoute();
        }
        return QGraphicsObject::itemChange(c, value);
    }
    void mousePressEvent(QGraphicsSceneMouseEvent *e) override {
        if (e->button() == Qt::LeftButton) {
            moveArmed = isSelected();
            dragging = false;
            pressScreen = e->screenPos();
            pressScene = e->scenePos();
            initialPosition = pos();
            // Property editing, copy and delete operate on one checkpoint.
            // Keep dragging consistent with those actions: Ctrl/Shift must not
            // create a group that only moves visually without updating data.
            for (auto *item : scene()->selectedItems())
                if (item != this)
                    item->setSelected(false);
            setSelected(true);
            e->setModifiers(e->modifiers() & ~(Qt::ControlModifier | Qt::ShiftModifier));
        }
        QGraphicsObject::mousePressEvent(e);
    }
    void mouseMoveEvent(QGraphicsSceneMouseEvent *e) override {
        if (moveArmed && (e->buttons() & Qt::LeftButton)) {
            const QPoint difference = e->screenPos() - pressScreen;
            if (!dragging && std::hypot(difference.x(), difference.y()) >= 3) {
                dragging = true;
                if (canvas->onGestureBegin)
                    canvas->onGestureBegin();
                setCursor(Qt::SizeAllCursor);
            }
            if (dragging)
                setPos(initialPosition + e->scenePos() - pressScene);
        }
        e->accept();
    }
    void mouseReleaseEvent(QGraphicsSceneMouseEvent *e) override {
        QGraphicsObject::mouseReleaseEvent(e);
        moveArmed = false;
        if (dragging) {
            dragging = false;
            auto *c = canvas;
            QTimer::singleShot(0, c, [c] {
                if (c->onGestureEnd)
                    c->onGestureEnd();
            });
        }
        setCursor(Qt::OpenHandCursor);
    }
    void hoverEnterEvent(QGraphicsSceneHoverEvent *e) override {
        setCursor(Qt::OpenHandCursor);
        QGraphicsObject::hoverEnterEvent(e);
    }
};

TrackCanvas::TrackCanvas(QWidget *parent) : QGraphicsView(parent) {
    scene_ = new QGraphicsScene(this);
    setScene(scene_);
    setFrameShape(QFrame::NoFrame);
    setRenderHints(QPainter::Antialiasing | QPainter::TextAntialiasing |
                   QPainter::SmoothPixmapTransform);
    setViewportUpdateMode(BoundingRectViewportUpdate);
    setTransformationAnchor(AnchorUnderMouse);
    setResizeAnchor(AnchorViewCenter);
    setDragMode(NoDrag);
    setFocusPolicy(Qt::StrongFocus);
    setMouseTracking(true);
    scale(15, 15);
    connect(scene_, &QGraphicsScene::selectionChanged, this, [this] {
        if (onSelection)
            onSelection(selectedGate());
    });
    timer_.setInterval(16);
    connect(&timer_, &QTimer::timeout, this, [this] {
        const double dt = std::min(.1, elapsed_.restart() / 1000.);
        if (pathLength_ < .01 || !drone_) {
            if (previewHasGap_) {
                if (drone_)
                    drone_->setPos(previewPath_.currentPosition());
                timer_.stop();
            }
            return;
        }
        flightDistance_ += dt * speedKph_ / 3.6;
        if (previewHasGap_ && flightDistance_ >= pathLength_) {
            flightDistance_ = pathLength_;
            drone_->setPos(previewPath_.pointAtPercent(1));
            timer_.stop();
            return;
        }
        if (doc_ && !doc_->closedLoop && flightDistance_ >= pathLength_)
            flightDistance_ = 0;
        const auto pos = previewPath_.pointAtPercent(
            previewPath_.percentAtLength(std::fmod(flightDistance_, pathLength_)));
        drone_->setPos(pos);
    });
}
void TrackCanvas::setDocument(TrackDocument *d) {
    doc_ = d;
    rebuild();
}
void TrackCanvas::rebuild() {
    const QString selected = selectedGate();
    scene_->blockSignals(true);
    scene_->clear();
    route_ = nullptr;
    drone_ = nullptr;
    if (doc_) {
        if (!doc_->background.isNull() && doc_->showBackground) {
            auto *image = scene_->addPixmap(QPixmap::fromImage(doc_->background));
            image->setScale(doc_->metersPerPixel);
            image->setPos(doc_->backgroundOrigin);
            image->setOpacity(doc_->backgroundOpacity);
            image->setZValue(-20);
        }
        route_ = scene_->addPath(QPainterPath(), QPen(QColor("#606060"), .12, Qt::SolidLine,
                                                      Qt::RoundCap, Qt::RoundJoin));
        route_->setZValue(-1);
        for (int i = 0; i < doc_->gates.size(); ++i) {
            const auto &g = doc_->gates[i];
            auto *item = new GateItem(g, this, i);
            scene_->addItem(item);
            if (g.id == selected)
                item->setSelected(true);
        }
        drone_ = scene_->addEllipse(QRectF(-.3, -.3, .6, .6), QPen(Qt::white, .12),
                                    QBrush(QColor("#333333")));
        drone_->setZValue(30);
        drone_->setVisible(timer_.isActive());
    }
    scene_->blockSignals(false);
    refreshRoute();
    QRectF content = scene_->itemsBoundingRect();
    if (doc_ && doc_->takeoff.enabled)
        content = content.united(QRectF(doc_->takeoff.position - QPointF(2, 2), QSizeF(4, 4)));
    scene_->setSceneRect(content.united(QRectF(-38, -25, 76, 50)).adjusted(-12, -12, 12, 12));
    viewport()->update();
}
void TrackCanvas::refreshRoute() {
    if (!doc_ || !route_) {
        path_ = QPainterPath();
        previewPath_ = QPainterPath();
        pathLength_ = 0;
        previewHasGap_ = false;
        return;
    }
    const auto curve = routeCurve(*doc_);
    path_ = QPainterPath();
    for (qsizetype i = 0; i < curve.points.size(); ++i) {
        const QPointF point(curve.points[i].x(), curve.points[i].z());
        if (i == 0 || curve.breaks.contains(static_cast<int>(i)))
            path_.moveTo(point);
        else
            path_.lineTo(point);
    }
    previewHasGap_ = !curve.breaks.isEmpty();
    previewPath_ = path_;
    if (!curve.breaks.isEmpty()) {
        previewPath_ = QPainterPath();
        const int end = curve.breaks.first();
        for (int i = 0; i < end && i < curve.points.size(); ++i) {
            const QPointF point(curve.points[i].x(), curve.points[i].z());
            if (i == 0)
                previewPath_.moveTo(point);
            else
                previewPath_.lineTo(point);
        }
    }
    pathLength_ = previewPath_.length();
    route_->setPath(path_);
    route_->setVisible(routeVisible_);
}
void TrackCanvas::fitTrack() {
    if (!doc_)
        return;
    refreshRoute();
    QRectF r;
    for (const auto &g : doc_->gates)
        r = r.united(QRectF(g.position - QPointF(3, 3), QSizeF(6, 6)));
    if (doc_->takeoff.enabled)
        r = r.united(QRectF(doc_->takeoff.position - QPointF(2, 2), QSizeF(4, 4)));
    r = r.united(path_.boundingRect());
    if (doc_->gates.isEmpty() && doc_->showBackground && !doc_->background.isNull())
        r = r.united(
            QRectF(doc_->backgroundOrigin, QSizeF(doc_->background.size()) * doc_->metersPerPixel));
    if (r.isEmpty())
        r = QRectF(-30, -20, 60, 40);
    fitInView(r.adjusted(-4, -4, 4, 4), Qt::KeepAspectRatio);
    if (onZoom)
        onZoom(zoomPercent());
}
void TrackCanvas::selectGate(const QString &id) {
    if (selectedGate() == id)
        return;
    scene_->blockSignals(true);
    for (auto *item : scene_->items())
        if (auto *g = dynamic_cast<GateItem *>(item))
            g->setSelected(g->gate.id == id);
    scene_->blockSignals(false);
    if (onSelection)
        onSelection(id);
}
QString TrackCanvas::selectedGate() const {
    for (auto *item : scene_->selectedItems())
        if (auto *g = dynamic_cast<GateItem *>(item))
            return g->gate.id;
    return {};
}
void TrackCanvas::setPlacement(const QString &type, double w, double h) {
    placement_ = type;
    placeWidth_ = w;
    placeHeight_ = h;
    setCursor(type.isEmpty() ? Qt::ArrowCursor : Qt::CrossCursor);
}
void TrackCanvas::setRouteVisible(bool b) {
    routeVisible_ = b;
    if (route_)
        route_->setVisible(b);
}
void TrackCanvas::setPreview(bool b) {
    if (b) {
        flightDistance_ = 0;
        elapsed_.start();
        timer_.start();
    } else
        timer_.stop();
    if (drone_)
        drone_->setVisible(b);
}
double TrackCanvas::zoomPercent() const {
    return transform().m11() / 15 * 100;
}
void TrackCanvas::zoomBy(double f) {
    const double current = transform().m11();
    if (!std::isfinite(f) || f <= 0 || !std::isfinite(current) || current <= 0)
        return;
    // fitInView may put a large map below the normal manual zoom range.
    // Permit steps back towards that range instead of rejecting every step.
    const double target = std::clamp(current * f, std::min(1., current), std::max(180., current));
    if (qFuzzyCompare(target, current))
        return;
    scale(target / current, target / current);
    if (onZoom)
        onZoom(zoomPercent());
}
QImage TrackCanvas::exportImage() const {
    QImage img(viewport()->size() * 2, QImage::Format_ARGB32_Premultiplied);
    img.fill(Qt::white);
    QPainter p(&img);
    p.scale(2, 2);
    const_cast<TrackCanvas *>(this)->render(&p, QRectF(QPointF(), viewport()->size()),
                                            viewport()->rect());
    return img;
}
void TrackCanvas::drawBackground(QPainter *p, const QRectF &r) {
    p->fillRect(r, QColor("#f7f7f7"));
}
void TrackCanvas::drawForeground(QPainter *p, const QRectF &) {
    p->save();
    p->resetTransform();
    if (doc_) {
        if (doc_->takeoff.enabled) {
            const QPointF marker = mapFromScene(doc_->takeoff.position);
            p->setBrush(Qt::white);
            p->setPen(QPen(QColor("#181818"), 2));
            p->drawEllipse(marker, 12, 12);
            p->drawText(QRectF(marker.x() - 10, marker.y() - 10, 20, 20), Qt::AlignCenter, "T");
            p->drawText(QRectF(marker.x() + 15, marker.y() - 12, 110, 24),
                        i18n::text("独立起飞点"));
        }
        if (routeVisible_) {
            QHash<QString, int> overlapping;
            int number = 0, turn = 0;
            for (const auto &step : effectiveRouteSteps(*doc_)) {
                const bool turnCheck =
                    step.kind == "guide" && step.extra.value("raceCheckpoint").toBool();
                if (step.kind != "pass" && !turnCheck)
                    continue;
                const auto geometry = routeStepGeometry(*doc_, step);
                if (!geometry.valid)
                    continue;
                const QPointF center =
                    mapFromScene(QPointF(geometry.pathPosition.x(), geometry.pathPosition.z()));
                const QString key =
                    QString("%1:%2").arg(qRound(center.x() / 12)).arg(qRound(center.y() / 12));
                const int row = overlapping[key]++;
                const QPointF badge = center + QPointF(15, -18 - row * 20);
                p->setBrush(QColor("#202020"));
                p->setPen(Qt::NoPen);
                p->drawRoundedRect(QRectF(badge.x(), badge.y(), 28, 18), 4, 4);
                p->setPen(Qt::white);
                p->drawText(QRectF(badge.x(), badge.y(), 28, 18), Qt::AlignCenter,
                            turnCheck ? QString("T%1").arg(++turn) : QString::number(++number));
            }
        }
    }
    p->setPen(QColor("#666666"));
    QFont f("Microsoft YaHei UI", 9);
    p->setFont(f);
    p->drawText(QRectF(20, 14, 280, 22), Qt::AlignLeft | Qt::AlignVCenter,
                i18n::text("XY 俯视图 · Z 为高度"));
    p->drawText(QRectF(20, viewport()->height() - 36, 470, 22), Qt::AlignLeft | Qt::AlignVCenter,
                placement_.isEmpty()
                    ? i18n::text("先点击选中，再按住拖动 · 滚轮缩放 · 中键 / 空格平移")
                : placement_ == "__takeoff" ? i18n::text("单击放置独立起飞点 · Esc 返回选择工具")
                                            : i18n::text("点击画布放置门架 · Esc 返回选择工具"));
    const double ppm = transform().m11();
    if (ppm > 0 && std::isfinite(ppm)) {
        const double desired = 70. / ppm;
        const double base = std::pow(10., std::floor(std::log10(desired)));
        const double normalized = desired / base;
        const double length = base * (normalized >= 7.5   ? 10.
                                      : normalized >= 3.5 ? 5.
                                      : normalized >= 1.5 ? 2.
                                                          : 1.);
        const double barPixels = length * ppm;
        const double x = viewport()->width() - 24 - barPixels, y = viewport()->height() - 28;
        p->setPen(QPen(QColor("#444444"), 2));
        p->drawLine(QPointF(x, y), QPointF(x + barPixels, y));
        p->drawLine(QPointF(x, y - 4), QPointF(x, y + 4));
        p->drawLine(QPointF(x + barPixels, y - 4), QPointF(x + barPixels, y + 4));
        p->drawText(QPointF(x, y - 9), QString::number(length, 'g', 3) + " m");
    }
    p->restore();
}
void TrackCanvas::wheelEvent(QWheelEvent *e) {
    zoomBy(std::pow(1.0015, e->angleDelta().y()));
    e->accept();
}
void TrackCanvas::mousePressEvent(QMouseEvent *e) {
    if (e->button() == Qt::MiddleButton || (space_ && e->button() == Qt::LeftButton)) {
        panning_ = true;
        panStart_ = e->pos();
        setCursor(Qt::ClosedHandCursor);
        e->accept();
        return;
    }
    if (e->button() == Qt::LeftButton && !placement_.isEmpty() && onPlace) {
        QPointF pos = mapToScene(e->pos());
        if (snap_)
            pos = QPointF(qRound(pos.x() * 2) / 2., qRound(pos.y() * 2) / 2.);
        onPlace(placement_, pos, placeWidth_, placeHeight_);
        e->accept();
        return;
    }
    QGraphicsView::mousePressEvent(e);
}
void TrackCanvas::mouseMoveEvent(QMouseEvent *e) {
    if (panning_) {
        QPoint d = e->pos() - panStart_;
        panStart_ = e->pos();
        horizontalScrollBar()->setValue(horizontalScrollBar()->value() - d.x());
        verticalScrollBar()->setValue(verticalScrollBar()->value() - d.y());
        e->accept();
        return;
    }
    QGraphicsView::mouseMoveEvent(e);
}
void TrackCanvas::mouseReleaseEvent(QMouseEvent *e) {
    if (panning_) {
        panning_ = false;
        setCursor(placement_.isEmpty() ? Qt::ArrowCursor : Qt::CrossCursor);
        e->accept();
        return;
    }
    QGraphicsView::mouseReleaseEvent(e);
}
void TrackCanvas::keyPressEvent(QKeyEvent *e) {
    if (e->key() == Qt::Key_Space) {
        space_ = true;
        e->accept();
        return;
    }
    if (e->key() == Qt::Key_Escape) {
        setPlacement({});
        e->accept();
        return;
    }
    QGraphicsView::keyPressEvent(e);
}
void TrackCanvas::keyReleaseEvent(QKeyEvent *e) {
    if (e->key() == Qt::Key_Space) {
        space_ = false;
        e->accept();
        return;
    }
    QGraphicsView::keyReleaseEvent(e);
}
void TrackCanvas::resizeEvent(QResizeEvent *e) {
    QGraphicsView::resizeEvent(e);
    if (firstFit_ && doc_) {
        firstFit_ = false;
        QTimer::singleShot(0, this, [this] { fitTrack(); });
    }
}
} // namespace ot
