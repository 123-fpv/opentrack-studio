#include "canvas.h"
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
        p->drawRoundedRect(QRectF(-.48, -.43, .96, .86), .26, .26);
        QFont f("Segoe UI");
        f.setPixelSize(12);
        p->setFont(f);
        p->scale(.07, .07);
        p->setPen(isSelected() ? Qt::white : QColor("#333333"));
        p->drawText(QRectF(-6.7, -6, 13.4, 12), Qt::AlignCenter, QString::number(ordinal + 1));
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
        if (pathLength_ < .01 || !drone_)
            return;
        flightDistance_ += dt * speedKph_ / 3.6;
        if (doc_ && !doc_->closedLoop && flightDistance_ >= pathLength_)
            flightDistance_ = 0;
        const auto pos =
            path_.pointAtPercent(path_.percentAtLength(std::fmod(flightDistance_, pathLength_)));
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
    const QRectF content = scene_->itemsBoundingRect();
    scene_->setSceneRect(content.united(QRectF(-38, -25, 76, 50)).adjusted(-12, -12, 12, 12));
    viewport()->update();
}
void TrackCanvas::refreshRoute() {
    if (!doc_ || !route_) {
        path_ = QPainterPath();
        pathLength_ = 0;
        return;
    }
    path_ = routePath(*doc_);
    pathLength_ = path_.length();
    route_->setPath(path_);
    route_->setVisible(routeVisible_);
}
void TrackCanvas::fitTrack() {
    if (!doc_)
        return;
    QRectF r;
    for (const auto &g : doc_->gates)
        r = r.united(QRectF(g.position - QPointF(3, 3), QSizeF(6, 6)));
    if (!doc_->background.isNull())
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
    const double step = (transform().m11() < 5) ? 5 : 1;
    auto pen = QPen(QColor("#e5e5e5"));
    pen.setCosmetic(true);
    p->setPen(pen);
    for (double x = std::floor(r.left() / step) * step; x < r.right(); x += step)
        p->drawLine(QPointF(x, r.top()), QPointF(x, r.bottom()));
    for (double y = std::floor(r.top() / step) * step; y < r.bottom(); y += step)
        p->drawLine(QPointF(r.left(), y), QPointF(r.right(), y));
    pen.setColor(QColor("#cecece"));
    p->setPen(pen);
    for (double x = std::floor(r.left() / 10) * 10; x < r.right(); x += 10)
        p->drawLine(QPointF(x, r.top()), QPointF(x, r.bottom()));
    for (double y = std::floor(r.top() / 10) * 10; y < r.bottom(); y += 10)
        p->drawLine(QPointF(r.left(), y), QPointF(r.right(), y));
}
void TrackCanvas::drawForeground(QPainter *p, const QRectF &) {
    p->save();
    p->resetTransform();
    p->setPen(QColor("#666666"));
    QFont f("Microsoft YaHei UI", 9);
    p->setFont(f);
    p->drawText(QRectF(20, viewport()->height() - 36, 470, 22), Qt::AlignLeft | Qt::AlignVCenter,
                placement_.isEmpty()
                    ? QStringLiteral("先点击选中，再按住拖动 · 滚轮缩放 · 中键 / 空格平移")
                    : QStringLiteral("点击画布放置门架 · Esc 返回选择工具"));
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
