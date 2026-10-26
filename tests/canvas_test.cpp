#include "canvas.h"
#include "document.h"

#include <QApplication>
#include <QMouseEvent>
#include <QPainter>
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
    if (!failures)
        std::cout << "PASS: 2D two-step selection, halo, drag threshold, snapped movement and undo "
                     "gestures; distinct compound glyphs and color retention.\n";
    return failures ? 1 : 0;
}
