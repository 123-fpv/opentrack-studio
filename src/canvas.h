#pragma once
#include "document.h"
#include <QElapsedTimer>
#include <QGraphicsItem>
#include <QGraphicsView>
#include <QTimer>
#include <functional>

namespace ot {
void paintGateSymbol(QPainter *p, const QRectF &bounds, const QString &type, bool selected = false,
                     const QColor &tint = QColor("#4b4b4b"));
class GateItem;
class TrackCanvas : public QGraphicsView {
  public:
    explicit TrackCanvas(QWidget *parent = nullptr);
    void setDocument(TrackDocument *doc);
    void rebuild();
    void refreshRoute();
    void fitTrack();
    void selectGate(const QString &id);
    QString selectedGate() const;
    void setPlacement(const QString &type, double width = 2.4, double height = 2.4);
    void setSnap(bool enabled) {
        snap_ = enabled;
    }
    void setRouteVisible(bool enabled);
    void setPreview(bool enabled);
    void setPreviewSpeed(double kph) {
        speedKph_ = kph;
    }
    void zoomBy(double factor);
    double zoomPercent() const;
    QImage exportImage() const;
    std::function<void(QString)> onSelection;
    std::function<void()> onGestureBegin;
    std::function<void(QString, QPointF)> onGateMoved;
    std::function<void()> onGestureEnd;
    std::function<void(QString, QPointF, double, double)> onPlace;
    std::function<void(double)> onZoom;

  protected:
    void drawBackground(QPainter *, const QRectF &) override;
    void drawForeground(QPainter *, const QRectF &) override;
    void wheelEvent(QWheelEvent *) override;
    void mousePressEvent(QMouseEvent *) override;
    void mouseMoveEvent(QMouseEvent *) override;
    void mouseReleaseEvent(QMouseEvent *) override;
    void keyPressEvent(QKeyEvent *) override;
    void keyReleaseEvent(QKeyEvent *) override;
    void resizeEvent(QResizeEvent *) override;

  private:
    friend class GateItem;
    TrackDocument *doc_ = nullptr;
    QGraphicsScene *scene_ = nullptr;
    QGraphicsPathItem *route_ = nullptr;
    QGraphicsEllipseItem *drone_ = nullptr;
    QPainterPath path_, previewPath_;
    double pathLength_ = 0;
    QTimer timer_;
    QElapsedTimer elapsed_;
    double flightDistance_ = 0, speedKph_ = 70;
    bool previewHasGap_ = false;
    QString placement_;
    double placeWidth_ = 2.4, placeHeight_ = 2.4;
    bool snap_ = true, routeVisible_ = true, panning_ = false, space_ = false, firstFit_ = true;
    QPoint panStart_;
};
} // namespace ot
