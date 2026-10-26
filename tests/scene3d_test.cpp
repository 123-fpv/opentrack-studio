#include "document.h"
#include "scene3d.h"

#include <QApplication>
#include <QImage>
#include <QKeyEvent>
#include <QMatrix4x4>
#include <QMouseEvent>
#include <cmath>
#include <iostream>
#include <limits>

namespace {
int failures = 0;
void require(bool condition, const char *description) {
    if (!condition) {
        std::cerr << "FAIL: " << description << '\n';
        ++failures;
    }
}

void mouse(ot::Scene3D &scene, QEvent::Type type, QPointF at, Qt::MouseButton button,
           Qt::MouseButtons held) {
    QMouseEvent event(type, at, QPointF(scene.mapToGlobal(at.toPoint())), button, held,
                      Qt::NoModifier);
    QApplication::sendEvent(&scene, &event);
}

void drag(ot::Scene3D &scene, QPointF from, QPointF to, Qt::MouseButton button) {
    mouse(scene, QEvent::MouseButtonPress, from, button, button);
    mouse(scene, QEvent::MouseMove, to, Qt::NoButton, button);
    mouse(scene, QEvent::MouseButtonRelease, to, button, Qt::NoButton);
}

// Compare a sparse set of colors within the scene, away from UI captions. This
// checks that a camera gesture changes projected geometry; it has no reference
// screenshot, font assumption, driver-specific checksum or exact pixel count.
bool sceneChanged(const QImage &before, const QImage &after) {
    if (before.isNull() || after.isNull() || before.size() != after.size())
        return false;
    int changed = 0;
    for (int y = before.height() / 6; y < before.height() * 5 / 6; y += 9)
        for (int x = before.width() / 8; x < before.width() * 7 / 8; x += 9) {
            const auto a = before.pixelColor(x, y), b = after.pixelColor(x, y);
            const int difference = std::abs(a.red() - b.red()) + std::abs(a.green() - b.green()) +
                                   std::abs(a.blue() - b.blue());
            if (difference > 16)
                ++changed;
        }
    return changed > 30;
}

bool snapped(double value) {
    return std::abs(value * 2 - std::round(value * 2)) < .0001;
}

QRect redBounds(const QImage &image) {
    QRect bounds;
    for (int y = 0; y < image.height(); ++y)
        for (int x = 0; x < image.width(); ++x) {
            const auto color = image.pixelColor(x, y);
            if (color.red() > 90 && color.red() > color.green() * 2 &&
                color.red() > color.blue() * 2)
                bounds = bounds.united(QRect(x, y, 1, 1));
        }
    return bounds;
}
} // namespace

int main(int argc, char **argv) {
    QApplication application(argc, argv);
    require(ot::editorAxisDirection(Qt::XAxis) == QVector3D(1, 0, 0) &&
                ot::editorAxisDirection(Qt::YAxis) == QVector3D(0, 0, 1) &&
                ot::editorAxisDirection(Qt::ZAxis) == QVector3D(0, 1, 0),
            "editor X/Y ground and Z-up labels map to the unchanged native basis");
    QMatrix4x4 axisCamera;
    axisCamera.lookAt({6, 4, 6}, {0, 0, 0}, {0, 1, 0});
    const QVector3D verticalAxis = axisCamera.mapVector(ot::editorAxisDirection(Qt::ZAxis));
    const QVector3D groundAxis = axisCamera.mapVector(ot::editorAxisDirection(Qt::YAxis));
    require(std::abs(verticalAxis.x()) < 1e-5 && verticalAxis.y() > 0 && groundAxis.y() < 0,
            "Z gizmo projects upward while Y remains on the ground plane");
    ot::TrackDocument document;
    ot::Gate gate;
    gate.id = QStringLiteral("gate-under-pointer");
    document.gates.append(gate);
    ot::Scene3D scene;
    scene.resize(980, 550);
    scene.setDocument(&document);
    scene.show();
    QApplication::processEvents();
    scene.fitTrack();
    const QPointF center(scene.width() / 2.0, scene.height() / 2.0);
    require(!scene.exportImage().isNull(),
            "scene exports with native GL or software perspective fallback");

    int began = 0, ended = 0, moved = 0, placed = 0;
    scene.onGestureBegin = [&] { ++began; };
    scene.onGestureEnd = [&] { ++ended; };
    scene.onGateMoved = [&](QString id, QPointF position) {
        require(id == gate.id, "drag identifies the picked gate");
        ++moved;
        document.gates[0].position = position;
    };
    const auto beforeSelection = scene.exportImage();
    mouse(scene, QEvent::MouseButtonPress, center, Qt::LeftButton, Qt::LeftButton);
    require(scene.selectedGate() == gate.id, "projected gate opening is pickable");
    mouse(scene, QEvent::MouseMove, center + QPointF(72, 18), Qt::NoButton, Qt::LeftButton);
    mouse(scene, QEvent::MouseButtonRelease, center + QPointF(72, 18), Qt::LeftButton,
          Qt::NoButton);
    require(moved == 0 && began == 0 && ended == 0 && document.gates[0].position == QPointF(),
            "first press only selects even if pointer moves before release");
    require(sceneChanged(beforeSelection, scene.exportImage()),
            "selection adds a visible halo and frame highlight");
    drag(scene, center, center + QPointF(1, 1), Qt::LeftButton);
    require(moved == 0 && began == 0 && ended == 0,
            "selected click jitter does not create movement or undo gesture");
    drag(scene, center, center + QPointF(72, 18), Qt::LeftButton);
    require(began == 1 && ended == 1 && moved == 1, "gate drag forms one undo gesture");
    require(document.gates[0].position.manhattanLength() > .1,
            "inverse camera ray moves gate on horizontal plane");
    require(snapped(document.gates[0].position.x()) && snapped(document.gates[0].position.y()),
            "drag honors half-grid snap");
    require(std::abs(document.gates[0].elevation - 1.2) < .001, "planar drag retains altitude");

    int selectionEvents = 0;
    scene.onSelection = [&](QString) { ++selectionEvents; };
    scene.selectGate(gate.id);
    scene.selectGate(gate.id);
    require(selectionEvents == 0, "selecting current item does not emit or rebuild again");
    drag(scene, {20, 75}, {20, 75}, Qt::LeftButton);
    require(scene.selectedGate().isEmpty(), "empty screen area does not select a distant gate");

    scene.onPlace = [&](QString type, QPointF position, double width, double height) {
        require(type == QStringLiteral("standard") && width == 2.4 && height == 2.4,
                "placement retains component specification");
        require(snapped(position.x()) && snapped(position.y()),
                "ground intersection placement snaps in world coordinates");
        ++placed;
    };
    scene.setPlacement(QStringLiteral("standard"));
    drag(scene, {scene.width() / 2.0, scene.height() * .77},
         {scene.width() / 2.0, scene.height() * .77}, Qt::LeftButton);
    require(placed == 1, "placement uses a ray intersection with the ground");
    QKeyEvent escape(QEvent::KeyPress, Qt::Key_Escape, Qt::NoModifier);
    QApplication::sendEvent(&scene, &escape);
    drag(scene, {20, 75}, {20, 75}, Qt::LeftButton);
    require(placed == 1, "Escape exits placement mode");

    scene.fitTrack();
    const auto beforeOrbit = scene.exportImage();
    const QPointF preservedPosition = document.gates[0].position;
    drag(scene, center, center + QPointF(115, -25), Qt::RightButton);
    require(sceneChanged(beforeOrbit, scene.exportImage()),
            "orbit changes the perspective projection");
    require(document.gates[0].position == preservedPosition && moved == 1,
            "orbit does not edit gate coordinates");
    const auto beforePan = scene.exportImage();
    drag(scene, center, center + QPointF(66, 21), Qt::MiddleButton);
    require(sceneChanged(beforePan, scene.exportImage()), "pan translates the camera target");
    require(document.gates[0].position == preservedPosition && moved == 1,
            "pan does not edit gate coordinates");

    double percent = 0;
    scene.onZoom = [&](double zoom) { percent = zoom; };
    scene.fitTrack();
    require(std::abs(percent - 100) < .001, "fit resets relative zoom");
    scene.zoomBy(1.5);
    require(std::abs(percent - 150) < .01, "dolly zoom reports relative camera distance");
    scene.zoomBy(0);
    scene.zoomBy(std::numeric_limits<double>::quiet_NaN());
    require(std::abs(percent - 150) < .01, "invalid zoom factors are ignored");

    ot::TrackDocument empty;
    scene.setDocument(&empty);
    scene.fitTrack();
    require(!scene.exportImage().isNull(), "empty document has a usable ground view");
    ot::TrackDocument framed;
    auto redGate = ot::makeGate();
    redGate.position = {-110, 70};
    ot::setGateColor(redGate, QColor("#e60000"));
    framed.gates = {redGate};
    scene.setDocument(&framed);
    scene.fitTrack();
    const auto withoutBackground = redBounds(scene.exportImage());
    framed.background = QImage(1200, 900, QImage::Format_RGB32);
    framed.background.fill(Qt::white);
    framed.backgroundOrigin = {-900, -700};
    framed.metersPerPixel = 2;
    scene.refreshGeometry();
    scene.fitTrack();
    const auto withBackground = redBounds(scene.exportImage());
    require(withoutBackground.width() > 20 && withoutBackground.height() > 20,
            "background-fit fixture has a clearly visible colored gate");
    require(withBackground.width() >= withoutBackground.width() * .9 &&
                withBackground.height() >= withoutBackground.height() * .9 &&
                std::abs(withBackground.center().x() - withoutBackground.center().x()) <= 3 &&
                std::abs(withBackground.center().y() - withoutBackground.center().y()) <= 3,
            "huge asymmetric background margins do not shrink or shift the fitted gate");
    require(framed.showBackground && !framed.background.isNull(),
            "fitting components retains the background and its visibility setting");
    ot::TrackDocument compound;
    for (const auto &type :
         {QStringLiteral("double"), QStringLiteral("triple"), QStringLiteral("ladder"),
          QStringLiteral("cube"), QStringLiteral("cube-stack")}) {
        compound.gates = {ot::makeGate(type)};
        scene.setDocument(&compound);
        scene.fitTrack();
        require(!scene.exportImage().isNull(),
                "compound stock frames render and fit in native or fallback perspective");
        // Perspective fitting retains the camera's earlier orbit; a wide flat
        // group can project away from the screen center. Find a selectable
        // point in the central content region without assuming pixel geometry.
        for (int row = 2; row <= 8 && scene.selectedGate() != compound.gates[0].id; ++row)
            for (int column = 2; column <= 8 && scene.selectedGate() != compound.gates[0].id;
                 ++column) {
                const QPointF pick(scene.width() * column / 10., scene.height() * row / 10.);
                mouse(scene, QEvent::MouseButtonPress, pick, Qt::LeftButton, Qt::LeftButton);
                mouse(scene, QEvent::MouseButtonRelease, pick, Qt::LeftButton, Qt::NoButton);
            }
        require(scene.selectedGate() == compound.gates[0].id,
                "whole compound frame is pickable after fitting, including stacked groups");
        require(compound.gates[0].position == QPointF(),
                "compound selection preserves the group's physical center");
    }
    ot::TrackDocument large;
    for (int i = 0; i < 500; ++i) {
        ot::Gate item;
        item.id = QString::number(i);
        item.position = QPointF(100000 + (i % 25) * 18, -200000 + (i / 25) * 14);
        item.elevation = 1.2 + (i % 4) * 2;
        large.gates.append(item);
    }
    scene.setDocument(&large);
    scene.fitTrack();
    scene.setPreview(true);
    QApplication::processEvents();
    scene.setPreview(false);
    require(!scene.exportImage().isNull(),
            "large elevated layout fits without invalid matrices or clipping failure");
    scene.setDocument(nullptr);
    scene.fitTrack();
    require(!scene.exportImage().isNull(), "detached document remains renderable");
    if (!failures)
        std::cout << "PASS: 3D picking, ray dragging, snapping, placement, orbit, pan, zoom and "
                     "fallback; nativeContext="
                  << scene.isValid() << '\n';
    return failures ? 1 : 0;
}
