#include "canvas.h"
#include "document.h"

#include <QApplication>
#include <QMouseEvent>
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
} // namespace

int main(int argc, char **argv) {
    QApplication application(argc, argv);
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
    if (!failures)
        std::cout << "PASS: 2D two-step selection, halo, drag threshold, snapped movement and undo "
                     "gestures.\n";
    return failures ? 1 : 0;
}
