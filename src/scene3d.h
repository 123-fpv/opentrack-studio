#pragma once

#include <QOpenGLFunctions>
#include <QOpenGLWidget>
#include <QPointF>
#include <QString>
#include <functional>
#include <memory>

class QKeyEvent;
class QMouseEvent;
class QPaintEvent;
class QResizeEvent;
class QWheelEvent;

namespace ot {
struct TrackDocument;

class Scene3D : public QOpenGLWidget, protected QOpenGLFunctions {
  public:
    explicit Scene3D(QWidget *parent = nullptr);
    ~Scene3D() override;
    void setDocument(TrackDocument *);
    void refreshGeometry();
    void fitTrack();
    void selectGate(const QString &);
    QString selectedGate() const;
    void setPlacement(QString, double width = 2.4, double height = 2.4);
    void setSnap(bool);
    void setRouteVisible(bool);
    void setPreview(bool);
    void setPreviewSpeed(double);
    void zoomBy(double);
    QImage exportImage();

    std::function<void(QString)> onSelection;
    std::function<void()> onGestureBegin, onGestureEnd;
    std::function<void(QString, QPointF)> onGateMoved;
    std::function<void(QString, QPointF, double, double)> onPlace;
    std::function<void(double)> onZoom;

  protected:
    void initializeGL() override;
    void resizeGL(int, int) override;
    void paintGL() override;
    void paintEvent(QPaintEvent *) override;
    void resizeEvent(QResizeEvent *) override;
    void wheelEvent(QWheelEvent *) override;
    void mousePressEvent(QMouseEvent *) override;
    void mouseMoveEvent(QMouseEvent *) override;
    void mouseReleaseEvent(QMouseEvent *) override;
    void keyPressEvent(QKeyEvent *) override;
    void keyReleaseEvent(QKeyEvent *) override;
    void leaveEvent(QEvent *) override;

  private:
    struct Impl;
    std::unique_ptr<Impl> d_;
};
} // namespace ot
