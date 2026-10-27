#include "canvas.h"
#include "document.h"

#include <QApplication>
#include <QEventLoop>
#include <QGraphicsPathItem>
#include <QGraphicsScene>
#include <QJsonArray>
#include <QMouseEvent>
#include <QPainter>
#include <QTimer>
#include <cmath>
#include <iostream>

namespace {
int failures = 0;
void require(bool condition, const char *description) {
    if (!condition) {
        ++failures;
        std::cerr << "FAIL: " << description << '\n';
    }
}

void mouse(ot::TrackCanvas &canvas, QEvent::Type type, QPointF at, Qt::MouseButton button,
           Qt::MouseButtons held) {
    auto *viewport = canvas.viewport();
    QMouseEvent event(type, at, QPointF(viewport->mapToGlobal(at.toPoint())), button, held,
                      Qt::NoModifier);
    QApplication::sendEvent(viewport, &event);
}

void drag(ot::TrackCanvas &canvas, QPointF from, QPointF to) {
    mouse(canvas, QEvent::MouseButtonPress, from, Qt::LeftButton, Qt::LeftButton);
    mouse(canvas, QEvent::MouseMove, to, Qt::NoButton, Qt::LeftButton);
    mouse(canvas, QEvent::MouseButtonRelease, to, Qt::LeftButton, Qt::NoButton);
    QApplication::processEvents(); // Gesture completion is queued to survive item rebuilding.
}

QImage symbol(const QString &type, bool selected = false, const QColor &color = QColor("#4b4b4b")) {
    QImage image(192, 168, QImage::Format_ARGB32_Premultiplied);
    image.fill(Qt::transparent);
    QPainter painter(&image);
    ot::paintGateSymbol(&painter, QRectF(0, 0, image.width(), image.height()), type, selected,
                        color);
    return image;
}
void advanceEvents(int milliseconds) {
    QEventLoop loop;
    QTimer::singleShot(milliseconds, &loop, &QEventLoop::quit);
    loop.exec();
}
} // namespace

int main(int argc, char **argv) {
    QApplication application(argc, argv);
    QList<QImage> glyphs;
    for (const auto &type :
         {QStringLiteral("standard"), QStringLiteral("double"), QStringLiteral("triple"),
          QStringLiteral("ladder"), QStringLiteral("cube"), QStringLiteral("cube-stack")}) {
        const auto neutral = symbol(type);
        for (const auto &previous : glyphs)
            require(neutral != previous, "compound glyphs are visually distinct from other types");
        glyphs.append(neutral);
        const auto colored = symbol(type, true, QColor("#e60000"));
        int tintedPixels = 0;
        bool monochrome = true;
        for (int y = 0; y < neutral.height(); ++y)
            for (int x = 0; x < neutral.width(); ++x) {
                const auto a = neutral.pixelColor(x, y), b = colored.pixelColor(x, y);
                if (a.alpha() > 16 && (a.red() != a.green() || a.green() != a.blue()))
                    monochrome = false;
                if (b.alpha() > 128 && b.red() > b.green() * 2 && b.red() > b.blue() * 2)
                    ++tintedPixels;
            }
        require(monochrome, "default gate glyphs retain the neutral library palette");
        require(tintedPixels > 50, "selected glyphs retain the component's chosen color");
    }
    ot::TrackDocument document;
    ot::Gate gate;
    gate.id = QStringLiteral("two-step-gate");
    document.gates.append(gate);
    ot::TrackCanvas canvas;
    canvas.resize(900, 550);
    canvas.setDocument(&document);
    canvas.show();
    QApplication::processEvents();
    canvas.fitTrack();
    const QPointF center = canvas.mapFromScene(gate.position);
    int began = 0, ended = 0, moved = 0;
    canvas.onGestureBegin = [&] { ++began; };
    canvas.onGestureEnd = [&] { ++ended; };
    canvas.onGateMoved = [&](QString id, QPointF point) {
        require(id == gate.id, "movement identifies selected item");
        ++moved;
        document.gates[0].position = point;
    };
    const QImage before = canvas.exportImage();
    drag(canvas, center, center + QPointF(65, 12));
    require(canvas.selectedGate() == gate.id, "first press selects gate");
    require(document.gates[0].position == QPointF() && moved == 0 && began == 0 && ended == 0,
            "first selecting gesture cannot move gate");
    require(canvas.exportImage() != before, "selection produces a visible 2D halo");
    drag(canvas, center, center + QPointF(1, 1));
    require(moved == 0 && began == 0 && ended == 0, "tiny selected click is not a drag");
    drag(canvas, center, center + QPointF(65, 12));
    require(moved >= 1 && began == 1 && ended == 1,
            "second press and drag creates one movement gesture");
    require(document.gates[0].position.manhattanLength() > .1,
            "second gesture updates document position");
    require(std::abs(document.gates[0].position.x() * 2 -
                     std::round(document.gates[0].position.x() * 2)) < .001,
            "movement remains snapped");
    const QPointF newCenter = canvas.mapFromScene(document.gates[0].position);
    canvas.selectGate({});
    drag(canvas, newCenter, newCenter + QPointF(-45, 0));
    require(moved >= 1 && began == 1 && ended == 1,
            "deselected gate requires selection again before movement");
    int events = 0;
    canvas.onSelection = [&](QString) { ++events; };
    canvas.selectGate(gate.id);
    canvas.selectGate(gate.id);
    require(events == 0, "selection bridge is a no-op for an already selected id");
    ot::TrackDocument legacyGap;
    legacyGap.closedLoop = false;
    for (const double x : {-12., -6., 0., 6., 12.}) {
        auto item = ot::makeGate();
        item.position = {x, 0};
        item.yaw = 90;
        if (x == 0)
            item.elevation = .1;
        legacyGap.gates.append(item);
    }
    const auto safeCurve = ot::routeCurve(legacyGap);
    require(!safeCurve.breaks.isEmpty(),
            "legacy fixture has an impossible low portal and a safe gap");
    canvas.setDocument(&legacyGap);
    canvas.fitTrack();
    int strokes = 0;
    for (auto *item : canvas.scene()->items())
        if (const auto *path = dynamic_cast<QGraphicsPathItem *>(item))
            for (int i = 0; i < path->path().elementCount(); ++i)
                strokes += path->path().elementAt(i).isMoveTo();
    require(
        strokes == safeCurve.breaks.size() + 1,
        "actual 2D scene renders separate constrained strokes even without explicit route steps");

    ot::TrackDocument offsetView;
    offsetView.closedLoop = false;
    offsetView.routeStepsDefined = true;
    offsetView.gates = {ot::makeGate()};
    ot::RouteStep crossingStep;
    crossingStep.id = ot::newGateId();
    crossingStep.gateId = offsetView.gates[0].id;
    offsetView.routeSteps = {crossingStep};
    canvas.setDocument(&offsetView);
    canvas.fitTrack();
    const auto centredCrossing = canvas.exportImage();
    offsetView.routeSteps[0].extra.insert("apertureU", .55);
    offsetView.routeSteps[0].extra.insert("flightTangent", QJsonArray{0, .4, 1});
    canvas.refreshRoute();
    const auto offsetGeometry = ot::routeStepGeometry(offsetView, offsetView.routeSteps[0]);
    bool hasOffsetCrossing = false;
    for (auto *item : canvas.scene()->items())
        if (const auto *path = dynamic_cast<QGraphicsPathItem *>(item))
            for (int i = 0; i < path->path().elementCount(); ++i) {
                const auto point = path->path().elementAt(i);
                hasOffsetCrossing |= std::abs(point.x - offsetGeometry.pathPosition.x()) < 1e-5 &&
                                     std::abs(point.y - offsetGeometry.pathPosition.z()) < 1e-5;
            }
    require(offsetGeometry.valid && hasOffsetCrossing && canvas.exportImage() != centredCrossing,
            "2D rendered curve and pass marker use the off-centre path crossing instead of the "
            "native window centre");
    ot::TrackDocument paused;
    paused.closedLoop = false;
    paused.routeStepsDefined = true;
    for (const auto &point :
         {QVector3D(-2, 1.6f, 0), QVector3D(-1.6f, 1.6f, 0), QVector3D(0, .05f, 0),
          QVector3D(2, 1.6f, 0), QVector3D(3, 1.6f, 0)}) {
        ot::RouteStep step;
        step.id = ot::newGateId();
        step.kind = "guide";
        step.localGuide = point;
        paused.routeSteps.append(step);
    }
    require(!ot::routeCurve(paused).breaks.isEmpty(), "preview pause fixture contains a gap");
    canvas.setDocument(&paused);
    canvas.fitTrack();
    canvas.setPreviewSpeed(233);
    canvas.setPreview(true);
    advanceEvents(120);
    const auto stopped = canvas.exportImage();
    for (const int milliseconds : {23, 31, 37}) {
        advanceEvents(milliseconds);
        require(canvas.exportImage() == stopped,
                "2D preview stays parked at the first gap instead of restarting across it");
    }
    canvas.setPreview(false);
    if (!failures)
        std::cout << "PASS: 2D two-step selection, halo, drag threshold, snapped movement and undo "
                     "gestures; distinct compound glyphs and color retention.\n";
    return failures ? 1 : 0;
}
