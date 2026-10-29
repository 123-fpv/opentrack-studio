#include "recognizer.h"
#include <QCoreApplication>
#include <QJsonArray>
#include <QPainter>
#include <cmath>
#include <iostream>

namespace {
int failures = 0;
void check(bool value, const char *message) {
    if (!value) {
        std::cerr << "FAIL: " << message << '\n';
        ++failures;
    }
}

QImage plan(int multiplier = 1) {
    QImage image(600 * multiplier, 400 * multiplier, QImage::Format_ARGB32);
    image.fill(QColor("#fbfcfe"));
    QPainter painter(&image);
    painter.scale(multiplier, multiplier);
    painter.setPen(QPen(QColor("#e6b947"), 2));
    painter.drawEllipse(QRectF(50, 40, 500, 300));
    // This border is intentionally the same blue as gates. A naive component
    // centroid detector would incorrectly turn it into a giant central gate.
    painter.setPen(QPen(QColor("#3688cf"), 2));
    painter.setBrush(Qt::NoBrush);
    painter.drawRect(QRectF(6, 6, 588, 388));
    for (const auto &point : {QPointF(100, 100), QPointF(300, 70), QPointF(500, 100),
                              QPointF(500, 300), QPointF(300, 330), QPointF(100, 300)})
        painter.drawRect(QRectF(point.x() - 11, point.y() - 9, 22, 18));
    return image;
}

bool containsNear(const ot::RecognitionResult &result, QPointF originalPixel, double tolerance) {
    const QPointF world =
        result.document.backgroundOrigin + originalPixel * result.document.metersPerPixel;
    for (const auto &gate : result.document.gates) {
        const auto difference = gate.position - world;
        if (std::hypot(difference.x(), difference.y()) < tolerance)
            return true;
    }
    return false;
}

// An original schematic, not a copy of a user's diagram: navy symbols with
// magenta overpainting, a separate legend, compound cells and an entry platform.
QImage inkSchematic(bool maneuvers = false, bool reversed = false, bool partialTriple = false,
                    int wrapSide = 0, bool occlusionCourse = false) {
    QImage image(1000, 760, QImage::Format_ARGB32);
    image.fill(Qt::white);
    QPainter painter(&image);
    const QColor ink("#34334d"), magenta("#b42a80");
    const QVector<QPointF> course{{820, 420}, {820, 210}, {660, 120}, {430, 120}, {250, 180},
                                  {180, 360}, {250, 540}, {450, 600}, {640, 550}, {720, 440}};
    painter.setPen(QPen(ink, 3));
    const auto frame = [&](QPointF center, int columns = 1, int levels = 1) {
        const QRectF box(center.x() - 15 * columns, center.y() - 15 * levels, 30 * columns,
                         30 * levels);
        painter.drawRect(box);
        for (int c = 1; c < columns; ++c)
            painter.drawLine(QPointF(box.left() + c * 30, box.top()),
                             QPointF(box.left() + c * 30, box.bottom()));
        for (int l = 1; l < levels; ++l)
            painter.drawLine(QPointF(box.left(), box.top() + l * 30),
                             QPointF(box.right(), box.top() + l * 30));
    };
    const auto cube = [&](QPointF center, int levels = 1) {
        const QRectF front(center.x() - 20, center.y() - 15 * levels + 4, 30, 30 * levels);
        const QPointF depth(10, -8);
        painter.drawRect(front);
        painter.drawRect(front.translated(depth));
        for (const auto corner :
             {front.topLeft(), front.topRight(), front.bottomLeft(), front.bottomRight()})
            painter.drawLine(corner, corner + depth);
        for (int l = 1; l < levels; ++l) {
            const double y = front.top() + l * 30;
            painter.drawLine(QPointF(front.left(), y), QPointF(front.right(), y));
            painter.drawLine(QPointF(front.left(), y) + depth, QPointF(front.right(), y) + depth);
        }
    };
    frame(course[0]);
    frame(course[1]);
    frame(course[2], 3);
    frame(course[3]);
    cube(course[4], 2);
    QPolygonF sliver({course[5] + QPointF(-3, 23), course[5] + QPointF(3, -23),
                      course[5] + QPointF(3, 17), course[5] + QPointF(-3, 23)});
    painter.drawPolygon(sliver);
    frame(course[6]);
    frame(course[7], 2);
    if (wrapSide) {
        const auto p = course[8];
        painter.drawPolygon(QPolygonF{p + QPointF(-3, 37), p + QPointF(3, -37), p + QPointF(3, 31),
                                      p + QPointF(-3, 37)});
    } else
        frame(course[8], 1, 2);
    cube(course[9]);
    // Legend symbols are visually plausible but disconnected from the course.
    frame(QPointF(75, 180));
    frame(QPointF(75, 260), 2);
    cube(QPointF(75, 340));
    if (reversed)
        painter.fillRect(QRectF(930, 405, 45, 30), ink);
    else
        painter.fillRect(QRectF(805, 650, 30, 55), ink);
    painter.setPen(QPen(magenta, 3));
    QPainterPath line;
    QVector<QPointF> path = course;
    if (maneuvers) {
        path = {{820, 420}, {820, 210}, {690, 120}};
        if (!partialTriple)
            path += QVector<QPointF>{{675, 80}, {660, 120}, {645, 160}, {630, 120}};
        else
            path += QVector<QPointF>{{695, 80}, {430, 80}};
        path +=
            QVector<QPointF>{{430, 120}, {300, 125}, {255, 145}, {250, 175}, {240, 205}, {180, 360},
                             {250, 540}, {465, 600}, {450, 640}, {435, 600}, {640, 565}};
        if (wrapSide < 0) {
            path.removeLast();
            path += QVector<QPointF>{{690, 610}, {680, 565}, {640, 565}};
        }
        if (wrapSide) {
            for (int j = 1; j <= 24; ++j) {
                const double a = 3.14159265358979323846 * j / 24;
                // Variable radius makes the source fit observably different
                // from a canned circle, while reversing side reverses winding.
                const double radius = 19 + 6 * std::sin(a * .7);
                if (std::abs(wrapSide) > 1)
                    path.push_back({640 + (wrapSide > 0 ? 1 : -1) * radius * std::sin(2 * a),
                                    565 - 30.0 * j / 24});
                else
                    path.push_back({640 + wrapSide * radius * std::sin(a), 550 + 15 * std::cos(a)});
            }
        } else
            path += QVector<QPointF>{{600, 555}, {640, 535}};
        path += QVector<QPointF>{{720, 450}, {725, 425}, {750, 395}};
        if (occlusionCourse) {
            path = {{820, 420}, {820, 210}, {690, 120}, {690, 95},  {680, 88},  {670, 105},
                    {660, 124}, {650, 105}, {640, 88},  {630, 105}, {630, 120}, {610, 110},
                    {430, 120}, {300, 125}, {255, 145}, {250, 175}, {240, 205}, {180, 360},
                    {250, 540}, {490, 560}, {465, 580}, {465, 607}, {450, 603}, {435, 578},
                    {425, 580}, {425, 620}, {640, 580}, {638, 570}, {620, 557}, {605, 565},
                    {615, 576}, {638, 562}, {646, 530}, {648, 515}, {685, 450}, {702, 450},
                    {725, 425}, {750, 395}};
        }
    }
    line.moveTo(path[0]);
    for (int i = 1; i < path.size(); ++i)
        line.lineTo(path[i]);
    line.closeSubpath();
    painter.drawPath(line);
    painter.drawLine(reversed ? QPointF(930, 420) : QPointF(820, 650), course[0]);
    painter.drawLine(QPointF(50, 410), QPointF(95, 410));
    if (occlusionCourse) {
        // The behind-frame shaft is hidden across the opening too; leaving a
        // visible intersecting diagonal would describe a different route graph.
        painter.setPen(QPen(Qt::white, 6));
        painter.drawLine(QPointF(637, 568), QPointF(620, 557));
        painter.setPen(QPen(ink, 5));
        for (const auto p : {QPointF(690, 105), QPointF(650, 105), QPointF(465, 585),
                             QPointF(425, 585), QPointF(648, 520)})
            painter.drawLine(p - QPointF(4, 0), p + QPointF(4, 0));
        painter.drawLine(QPointF(615, 109), QPointF(615, 117));
        painter.drawLine(QPointF(625, 553), QPointF(625, 562));
    } else if (std::abs(wrapSide) > 1) {
        painter.setPen(QPen(ink, 5));
        painter.drawLine(QPointF(636, 565), QPointF(644, 565));
        painter.drawLine(QPointF(636, 535), QPointF(644, 535));
    } else if (maneuvers) {
        // Opaque stock edges interrupt the original route at two apertures.
        // Their short, aligned line fragments must be reconnected before tracing.
        painter.setPen(QPen(ink, 5));
        painter.drawLine(QPointF(685, 105), QPointF(695, 105));
        painter.drawLine(QPointF(635, 550), QPointF(645, 550));
    }
    return image;
}

QImage crossingSchematic() {
    QImage image(1000, 760, QImage::Format_ARGB32);
    image.fill(Qt::white);
    QPainter painter(&image);
    const QColor ink("#34334d"), magenta("#b42a80");
    const QVector<QPointF> order{{800, 590}, {200, 150}, {800, 150}, {200, 590}};
    painter.setPen(QPen(ink, 3));
    for (const auto &point : order)
        painter.drawRect(QRectF(point - QPointF(15, 15), QSizeF(30, 30)));
    painter.fillRect(QRectF(885, 665, 30, 50), ink);
    painter.setPen(QPen(magenta, 3));
    painter.drawLine(QPointF(900, 665), order.front());
    for (int i = 0; i < order.size(); ++i)
        painter.drawLine(order[i], order[(i + 1) % order.size()]);
    return image;
}

// An independent six-component course with a projected crossing inside a
// ladder. Thinning its two-pixel shared neck produces two adjacent Y junctions;
// consuming that neck only once strands the outgoing shaft after the loop.
QImage junctionSchematic() {
    QImage image(800, 650, QImage::Format_ARGB32);
    image.fill(Qt::white);
    QPainter painter(&image);
    const QColor ink("#34334d"), magenta("#b42a80");
    painter.setPen(QPen(ink, 3));
    for (const auto center : {QPointF(650, 480), QPointF(650, 180), QPointF(390, 130),
                              QPointF(150, 200), QPointF(160, 460)})
        painter.drawRect(QRectF(center - QPointF(15, 15), QSizeF(30, 30)));
    painter.drawRect(QRectF(385, 435, 30, 60));
    painter.drawLine(385, 465, 415, 465);
    painter.fillRect(QRectF(635, 560, 30, 50), ink);
    painter.setPen(QPen(magenta, 3));
    QPainterPath route;
    route.moveTo(650, 480);
    for (const auto point : {QPointF(650, 180), QPointF(390, 130), QPointF(150, 200),
                             QPointF(160, 460), QPointF(399, 466), QPointF(401, 466)})
        route.lineTo(point);
    route.cubicTo(430, 485, 430, 440, 399, 466);
    route.lineTo(401, 466);
    route.lineTo(400, 450);
    route.lineTo(490, 380);
    route.lineTo(650, 480);
    painter.drawPath(route);
    painter.drawLine(650, 560, 650, 480);
    return image;
}

QImage decoratedSchematic(bool extraTypes = false) {
    auto source = inkSchematic();
    const QColor ink("#34334d");
    // Replace the filled launch rectangle with a compact row of four marks.
    // The route ends before this independent pad, as on many published plans.
    for (int y = 650; y <= 705; ++y)
        for (int x = 805; x <= 835; ++x)
            if (source.pixelColor(x, y) == ink)
                source.setPixelColor(x, y, Qt::white);
    {
        QPainter painter(&source);
        for (int i = 0; i < 4; ++i)
            painter.fillRect(QRectF(795 + i * 13, 700, 6, 6), ink);
        if (extraTypes) {
            painter.fillRect(QRectF(407, 97, 46, 46), Qt::white);
            painter.setPen(QPen(ink, 3));
            painter.drawRect(QRectF(400, 90, 60, 60));
            painter.drawLine(430, 90, 430, 150);
            painter.drawLine(400, 120, 460, 120);
            painter.fillRect(QRectF(626, 552, 28, 27), ink);
        }
    }
    QImage field(source.size(), QImage::Format_ARGB32);
    field.fill(Qt::white);
    {
        QPainter painter(&field);
        QPainterPath backdrop;
        backdrop.moveTo(820, 650);
        for (const auto p : {QPointF(820, 210), QPointF(660, 120), QPointF(430, 120),
                             QPointF(250, 180), QPointF(180, 360), QPointF(250, 540),
                             QPointF(450, 600), QPointF(640, 550), QPointF(720, 440)})
            backdrop.lineTo(p);
        painter.setPen(QPen(QColor("#c8c8c8"), 50));
        painter.drawPath(backdrop);
        painter.setPen(QPen(QColor("#969696"), 2));
        painter.drawPath(backdrop);
    }
    for (int y = 0; y < source.height(); ++y)
        for (int x = 0; x < source.width(); ++x)
            if (source.pixelColor(x, y) != QColor(Qt::white))
                field.setPixel(x, y, source.pixel(x, y));
    QImage poster(2000, 1500, QImage::Format_ARGB32);
    poster.fill(QColor("#ffb718"));
    QPainter painter(&poster);
    painter.fillRect(QRectF(50, 250, 1900, 1200), Qt::white);
    painter.drawImage(QPoint(650, 350), field);
    painter.setPen(QPen(Qt::black, 3));
    painter.drawRect(QRectF(70, 270, 1860, 1160));
    // Red north arrow and blue dimension-like strokes are not route or gates.
    painter.setPen(Qt::NoPen);
    painter.setBrush(QColor("#d41a2e"));
    painter.drawPolygon(QPolygonF{{1770, 420}, {1755, 485}, {1790, 455}});
    painter.setPen(QPen(QColor("#3688cf"), 3));
    painter.drawLine(1440, 330, 1590, 330);
    painter.drawRect(QRectF(1500, 320, 10, 14));
    return poster;
}
} // namespace

int main(int argc, char **argv) {
    QCoreApplication application(argc, argv);
    ot::RecognitionOptions manual;
    manual.autoScale = false;
    manual.metersPerPixel = 0.1;
    const auto recognized = ot::recognizeTrack(plan(), manual);
    check(recognized.detectedCount == 6, "six rectangular symbols, excluding the blue border");
    check(recognized.document.gates.size() == recognized.detectedCount,
          "count matches generated gates");
    check(containsNear(recognized, QPointF(100, 100), 0.2),
          "first gate location in original pixel coordinates");
    check(containsNear(recognized, QPointF(300, 330), 0.2),
          "lower gate location in original pixel coordinates");
    check(std::abs(recognized.document.backgroundOrigin.x() + 30.0) < 0.001,
          "background origin uses physical scale");
    check(!recognized.warnings.isEmpty(), "inferred geometry and order require explicit review");
    for (const auto &gate : recognized.document.gates) {
        check(!gate.id.isEmpty(), "recognition creates persistent gate IDs");
        check(std::isfinite(gate.yaw), "suggested yaw is finite");
        check(gate.extra.contains(QStringLiteral("recognition")), "uncertainty metadata preserved");
    }
    for (int i = 0; i < recognized.document.gates.size(); ++i) {
        const auto &gates = recognized.document.gates;
        const auto direction = gates[(i + 1) % gates.size()].position -
                               gates[(i + gates.size() - 1) % gates.size()].position;
        const double radians = gates[i].yaw * 3.14159265358979323846 / 180.0;
        const double dot = std::sin(radians) * direction.x() + std::cos(radians) * direction.y();
        check(dot > 0, "yaw uses +Y=0, +X=90 and follows the inferred route tangent");
    }

    const auto enlarged = ot::recognizeTrack(plan(4), manual);
    check(enlarged.detectedCount == 6, "downsampling retains separated gate symbols");
    check(containsNear(enlarged, QPointF(400, 400), 0.7),
          "downsampling maps back to original resolution");
    check(std::abs(enlarged.document.backgroundOrigin.x() + 120.0) < 0.001,
          "downsampling does not change the user-provided scale");

    const auto scaled = ot::recognizeTrack(plan());
    check(scaled.document.metersPerPixel > 0.1 && scaled.document.metersPerPixel < 0.16,
          "automatic scale estimates the 3.0 metre frame centreline spacing");
    for (const auto &component : scaled.document.gates) {
        check(component.width == 3.0 && component.height == 3.0 && component.elevation == 1.6,
              "newly recognized frames use 3m centrelines and ground-standing 3.2m beams");
        check(component.extra.value("frameGeometry").toString() == "centerline-v1",
              "newly recognized frames opt in to accurate block-beam geometry");
    }
    QImage withWide = plan();
    {
        QPainter painter(&withWide);
        painter.setPen(QPen(QColor("#3688cf"), 2));
        painter.drawRect(QRectF(267, 191, 66, 18));
    }
    const auto wide = ot::recognizeTrack(withWide, manual);
    check(wide.detectedCount == 7, "wide symbol is retained as a candidate");
    bool foundWide = false;
    for (const auto &gate : wide.document.gates) {
        check(gate.type != QStringLiteral("double"),
              "recognition does not invent an unsupported double prefab");
        if (gate.width > 3.0) {
            foundWide = true;
            check(gate.type == QStringLiteral("standard"),
                  "uncertain wide gate remains an exportable standard frame");
            check(gate.extra.value(QStringLiteral("recognition"))
                      .toObject()
                      .value(QStringLiteral("possibleDoubleGate"))
                      .toBool(),
                  "wide gate ambiguity is recorded");
        }
    }
    check(foundWide, "wide frame width is represented in generated geometry");
    QImage withPillar = plan();
    {
        QPainter painter(&withPillar);
        painter.setPen(QPen(QColor("#3688cf"), 2));
        painter.drawRect(QRectF(299, 190, 3, 20));
    }
    const auto pillars = ot::recognizeTrack(withPillar, manual);
    check(pillars.detectedCount == 7, "thin turning marker is retained alongside gate frames");
    bool foundPillar = false;
    for (const auto &component : pillars.document.gates) {
        check(component.type != QStringLiteral("flag"),
              "new image generation never emits a flag asset");
        if (component.type != QStringLiteral("pillar"))
            continue;
        foundPillar = true;
        check(component.width == 0.1 && component.height == 3.2 && component.elevation == 1.6,
              "new turning pillar uses the confirmed 0.1m square section and 3.2m beam");
        check(component.extra.value("recognition").toObject().value("heightInferred").toBool(),
              "pillar height remains explicitly inferred");
        check(component.extra.value("passSide").toInt() == 1,
              "unresolved image marker uses the documented right-hand passing default");
    }
    check(foundPillar, "thin marker is a solid pillar rather than a flag");
    QImage blank(300, 200, QImage::Format_ARGB32);
    blank.fill(Qt::white);
    const auto empty = ot::recognizeTrack(blank);
    check(empty.detectedCount == 0 && empty.document.gates.isEmpty(),
          "blank image cannot fabricate gates");
    check(!empty.warnings.isEmpty(), "no-match result is explained");
    const auto nullResult = ot::recognizeTrack(QImage());
    check(nullResult.detectedCount == 0, "null image returns no matches");

    QImage twoGates(300, 200, QImage::Format_ARGB32);
    twoGates.fill(Qt::white);
    {
        QPainter painter(&twoGates);
        painter.setPen(QPen(QColor("#3688cf"), 2));
        painter.drawRect(QRectF(50, 90, 22, 18));
        painter.drawRect(QRectF(220, 90, 22, 18));
    }
    const auto pair = ot::recognizeTrack(twoGates, manual);
    check(pair.detectedCount == 2, "two separated gates detected");
    if (pair.detectedCount == 2) {
        check(std::abs(pair.document.gates[0].yaw - 90.0) < 0.01,
              "two-gate route faces +X with yaw90");
        check(std::abs(pair.document.gates[1].yaw + 90.0) < 0.01,
              "two-gate return direction faces -X with yaw-90");
    }

    auto badOptions = manual;
    badOptions.metersPerPixel = std::nan("");
    badOptions.minComponentPixels = -500;
    badOptions.gateColor = Qt::white;
    const auto sanitized = ot::recognizeTrack(plan(), badOptions);
    check(sanitized.detectedCount == 6, "invalid options are sanitized without failure");
    check(std::isfinite(sanitized.document.metersPerPixel),
          "invalid scale cannot leak NaN into a project");

    QImage fringes = plan(2);
    {
        QPainter painter(&fringes);
        painter.setPen(QPen(QColor("#3688cf"), 2));
        for (int i = 0; i < 14; ++i) {
            const int x = 45 + i * 18;
            // Font-independent small coloured edges and loops reproduce the
            // component sizes produced by native Windows ClearType rendering.
            if (i % 2)
                painter.drawRect(QRectF(x, 29, 8, 10));
            else {
                painter.drawLine(x, 29, x, 39);
                painter.drawLine(x, 39, x + 8, 39);
            }
        }
    }
    const auto cleanLarge = ot::recognizeTrack(plan(2));
    const auto withFringes = ot::recognizeTrack(fringes);
    check(withFringes.detectedCount == 6,
          "small font-like blue fragments cannot outvote the actual hollow frame family");
    check(std::abs(withFringes.document.metersPerPixel - cleanLarge.document.metersPerPixel) <
              .000001,
          "coloured text fragments do not contaminate automatic scale");
    check(withFringes.document.extra.value("recognition")
                  .toObject()
                  .value("rejectedFragments")
                  .toInt() >= 14,
          "rejected text fragments are available in recognition diagnostics");

    const auto schematic = ot::recognizeTrack(inkSchematic(), manual);
    check(schematic.detectedCount == 10,
          "navy schematic recognizer counts physical obstacles and excludes legend/platform");
    check(schematic.document.extra.value("recognition")
                      .toObject()
                      .value("orderHeuristic")
                      .toString() == "directed-source-trace" &&
              schematic.document.extra.value("recognition")
                  .toObject()
                  .value("sourceOrderConfirmed")
                  .toBool(),
          "complete directed shaft evidence determines gate order rather than a shortest tour");
    const QStringList expectedTypes{"standard", "standard", "triple", "standard", "cube-stack",
                                    "standard", "standard", "double", "ladder",   "cube"};
    const QVector<QPointF> expectedPixels{{820, 420}, {820, 210}, {660, 120}, {430, 120},
                                          {250, 180}, {180, 360}, {250, 540}, {450, 600},
                                          {640, 550}, {720, 440}};
    if (schematic.document.gates.size() == expectedTypes.size())
        for (int i = 0; i < expectedTypes.size(); ++i) {
            const auto &component = schematic.document.gates[i];
            check(component.type == expectedTypes[i],
                  "compound and edge-on symbols keep their physical type");
            const auto expected =
                schematic.document.backgroundOrigin + expectedPixels[i] * manual.metersPerPixel;
            check(std::hypot(component.position.x() - expected.x(),
                             component.position.y() - expected.y()) < .8,
                  "schematic sequence follows entry direction and magenta connectivity");
        }
    check(!schematic.warnings.isEmpty(),
          "compound internal passages remain explicitly subject to review");
    check(schematic.document.closedLoop,
          "directed first-encounter ordering preserves a source circuit's closing leg");

    for (const double factor : {.75, 1., 1.5}) {
        const auto poster = decoratedSchematic();
        const auto image = factor == 1. ? poster : poster.scaled(
            qRound(poster.width() * factor), qRound(poster.height() * factor),
            Qt::KeepAspectRatio, Qt::SmoothTransformation);
        const auto detected = ot::recognizeTrack(image);
        check(detected.detectedCount == 10,
              "decorated gray-backed poster retains physical symbols at multiple resolutions");
        check(detected.document.extra.value("recognition").toObject().value("method") ==
                  "local-ink-route-v2",
              "navy symbols on gray field do not fall through to blue dimension fragments");
        check(detected.document.takeoff.enabled,
              "separate compact launch-grid row automatically creates an independent takeoff");
        const QPointF expectedPad =
            detected.document.backgroundOrigin +
            QPointF((650 + 817.5) * factor, (350 + 703) * factor) *
                detected.document.metersPerPixel;
        check(std::hypot(detected.document.takeoff.position.x() - expectedPad.x(),
                         detected.document.takeoff.position.y() - expectedPad.y()) < 1.,
              "takeoff is placed on the grid marks rather than the route endpoint or first gate");
        check(std::abs(detected.document.metersPerPixel * factor - .1) < .008,
              "small poster symbols calibrate from 3m stroke centres, excluding frame thickness");
        for (const auto &gate : detected.document.gates)
            check(gate.type != "pillar", "poster labels and gray contours cannot create pillars");
    }
    const auto newTypes = ot::recognizeTrack(decoratedSchematic(true));
    bool hasQuad = false, hasBlocked = false;
    for (const auto &gate : newTypes.document.gates) {
        hasQuad |= gate.type == "quad";
        hasBlocked |= gate.type == "ladder-blocked";
        if (gate.type == "ladder-blocked")
            for (const auto &step : newTypes.document.routeSteps)
                if (step.gateId == gate.id && step.kind == "pass")
                    check(step.level == 1, "blocked lower aperture is never a flight checkpoint");
    }
    check(hasQuad && hasBlocked,
          "equal four-cell frame and lower-blocked ladder keep their physical component types");

    const auto junction = ot::recognizeTrack(junctionSchematic(), manual);
    const auto junctionEvidence = junction.document.extra.value("recognition").toObject();
    check(junction.detectedCount == 6 && junction.document.takeoff.enabled,
          "an internal projected loop retains six physical frames and the launch pad");
    check(junctionEvidence.value("routeTraceCoverage").toDouble() > .99 &&
              junctionEvidence.value("sourceOrderConfirmed").toBool(),
          "a tiny Y-to-Y junction neck cannot strand the outgoing shaft after a loop");

    const auto crossed = ot::recognizeTrack(crossingSchematic(), manual);
    const QVector<QPointF> crossingOrder{{800, 590}, {200, 150}, {800, 150}, {200, 590}};
    check(crossed.detectedCount == crossingOrder.size(),
          "route crossings are not extra gate symbols");
    check(!crossed.document.extra.value("recognition")
                  .toObject()
                  .value("sourceOrderConfirmed")
                  .toBool() &&
              crossed.warnings.join('\n').contains(QStringLiteral("通过顺序")),
          "an unmarked joined X without direction/overpass evidence is flagged for order review");

    const auto passagesFor = [](const ot::TrackDocument &document, const QString &type) {
        QVector<ot::RouteStep> result;
        for (const auto &gate : document.gates)
            if (gate.type == type)
                for (const auto &step : document.routeSteps)
                    if (step.gateId == gate.id && step.kind == QStringLiteral("pass"))
                        result.push_back(step);
        return result;
    };
    const auto maneuvers = ot::recognizeTrack(inkSchematic(true), manual);
    check(maneuvers.detectedCount == 10, "internal loops do not create extra physical components");
    check(maneuvers.document.routeStepsDefined, "traced maneuvers are an explicit route");
    check(maneuvers.document.takeoff.enabled && maneuvers.document.takeoff.elevation == 0,
          "solid launch platform is independent of gates and passages");
    check(!maneuvers.document.showBackground, "generated scene hides the source image by default");
    auto triple = passagesFor(maneuvers.document, QStringLiteral("triple"));
    check(triple.size() == 3, "three observed windows create three references to one triple");
    if (triple.size() == 3) {
        check(triple[0].column == 0 && triple[1].column == 1 && triple[2].column == 2,
              "triple column order follows the printed waveform");
        check(triple[0].face == "front" && triple[1].face == "back" && triple[2].face == "front",
              "printed serpentine receives alternating passage faces");
    }
    const auto ladder = passagesFor(maneuvers.document, QStringLiteral("ladder"));
    check(ladder.size() == 2, "two visible ladder apertures create two passes");
    if (ladder.size() == 2)
        check(ladder[0].level == 0 && ladder[1].level == 1,
              "lower-to-upper ladder order comes from line evidence");
    const auto cube = passagesFor(maneuvers.document, QStringLiteral("cube"));
    check(cube.size() == 2, "visible roof/body transition creates two cube passes");
    if (cube.size() == 2)
        check(cube[0].face != "top" && cube[1].face == "top" && cube[1].reverse,
              "body-to-roof traversal is a gravity climb");
    const auto reverse = ot::recognizeTrack(inkSchematic(true, true), manual);
    const auto reverseTriple = passagesFor(reverse.document, QStringLiteral("triple"));
    check(reverseTriple.size() == 3,
          "opposite direction retains all three observed triple windows");
    if (triple.size() == 3 && reverseTriple.size() == 3) {
        check(ot::routeStepPosition(maneuvers.document, triple.front()).x() >
                      ot::routeStepPosition(maneuvers.document, triple.back()).x() &&
                  ot::routeStepPosition(reverse.document, reverseTriple.front()).x() <
                      ot::routeStepPosition(reverse.document, reverseTriple.back()).x(),
              "reversing the printed waveform reverses world-space window order");
    }
    const auto reverseLadder = passagesFor(reverse.document, QStringLiteral("ladder"));
    check(reverseLadder.size() == 2, "opposite launch direction preserves the ladder's two passes");
    if (reverseLadder.size() == 2)
        check(reverseLadder[0].level == 1 && reverseLadder[1].level == 0,
              "reversing drawn route reverses upper/lower order without a type preset");
    const auto reverseCube = passagesFor(reverse.document, QStringLiteral("cube"));
    if (reverseCube.size() == 2)
        check(reverseCube[0].face == "top" && reverseCube[1].face != "top",
              "opposite roof/body traversal becomes a gravity dive");
    else
        check(false, "reversed cube still exposes its visible roof/body transition");
    const auto partial = ot::recognizeTrack(inkSchematic(true, false, true), manual);
    const auto partialPasses = passagesFor(partial.document, QStringLiteral("triple"));
    check(partialPasses.size() < 3,
          "a compound symbol alone cannot invent unseen internal window passages");
    const QImage maneuverImage = inkSchematic(true);
    for (const double factor : {.75, 1.5}) {
        const auto resized =
            ot::recognizeTrack(maneuverImage.scaled(int(maneuverImage.width() * factor),
                                                    int(maneuverImage.height() * factor),
                                                    Qt::KeepAspectRatio, Qt::SmoothTransformation),
                               manual);
        check(resized.detectedCount == 10,
              "resampling a schematic preserves its physical component count");
        check(passagesFor(resized.document, "triple").size() == 3 &&
                  passagesFor(resized.document, "ladder").size() == 2,
              "resampling and antialias fragments preserve observed compound passages");
    }
    QString roundtripError;
    ot::TrackDocument restored;
    check(ot::fromJson(ot::toJson(maneuvers.document), &restored, &roundtripError) &&
              restored.routeSteps.size() == maneuvers.document.routeSteps.size(),
          "physical components, repeated passages and guides survive project serialization");

    for (const int side : {-1, 1}) {
        const auto wrap = ot::recognizeTrack(inkSchematic(true, false, false, side), manual);
        const ot::Gate *barGate = nullptr;
        for (const auto &gate : wrap.document.gates)
            if (gate.extra.value("recognition").toObject().value("wrapWindingInferred").toBool())
                barGate = &gate;
        check(barGate != nullptr, "side-view source curve establishes crossbar wrap winding");
        if (!barGate)
            continue;
        const auto action =
            barGate->extra.value("recognition").toObject().value("barWrap").toObject();
        check(action.value("axis").toString() == "x" && action.value("winding").toInt() == side,
              "opposite source lobes give opposite winding in the declared local+X view");
        check(action.value("sourceCurve").toArray().size() > 10,
              "original curve evidence survives instead of only keeping a generic waypoint");
        ot::TrackDocument isolated = wrap.document;
        isolated.gates = {*barGate};
        isolated.routeSteps.clear();
        isolated.closedLoop = false;
        QVector<ot::RouteStep> passes;
        for (const auto &step : wrap.document.routeSteps)
            if (step.gateId == barGate->id) {
                isolated.routeSteps.push_back(step);
                if (step.kind == "pass")
                    passes.push_back(step);
            }
        check(passes.size() == 2, "a half-wrap never adds an unseen full-orbit pass");
        if (passes.size() != 2)
            continue;
        const auto fittedCurve = ot::routeCurve(isolated);
        check(fittedCurve.clearanceSatisfied && fittedCurve.breaks.isEmpty() && fittedCurve.smooth,
              "inferred bar wrap remains one smooth stroke with actual stock clearance");
        const auto dense = fittedCurve.points;
        const auto from = ot::routeStepPosition(isolated, passes.front()),
                   to = ot::routeStepPosition(isolated, passes.back());
        int begin = -1, end = -1;
        for (int i = 0; i < dense.size(); ++i) {
            if ((dense[i] - from).length() < .0001)
                begin = i;
            if ((dense[i] - to).length() < .0001)
                end = i;
        }
        check(begin >= 0 && end > begin, "dense fitted route retains both exact passage centres");
        const double yaw = barGate->yaw * 3.14159265358979323846 / 180;
        double sweep = 0, previous = 0, minRadius = 1000;
        for (int i = begin; i <= end && begin >= 0; ++i) {
            const auto p =
                dense[i] - QVector3D(float(barGate->position.x()), float(barGate->elevation),
                                     float(barGate->position.y()));
            const double depth = p.x() * std::sin(yaw) + p.z() * std::cos(yaw), up = p.y();
            const double angle = std::atan2(up, depth);
            if (i > begin)
                sweep += std::remainder(angle - previous, 2 * 3.14159265358979323846);
            previous = angle;
            minRadius = std::min(minRadius, std::hypot(depth, up));
        }
        check(sweep * side > 3.0 && std::abs(sweep) < 3.3,
              "dense geometry actually winds a signed half-turn around the shared stock");
        check(minRadius > .70,
              "fitted curve stays around the stock rather than cutting through it");
        check(end - begin > 30, "wrap is sampled as a smooth curve between actual passages");
    }

    const auto occludedImage = inkSchematic(true, false, false, 0, true);
    const auto occluded = ot::recognizeTrack(occludedImage, manual);
    const auto findType = [](const ot::TrackDocument &d, const QString &type) -> const ot::Gate * {
        for (const auto &g : d.gates)
            if (g.type == type)
                return &g;
        return nullptr;
    };
    const auto tripleOccluded = passagesFor(occluded.document, "triple");
    check(tripleOccluded.size() == 3, "occluded progressive triple retains three source apertures");
    if (const auto *gate = findType(occluded.document, "triple")) {
        const auto info = gate->extra.value("recognition").toObject();
        check(info.value("occlusionFit").toBool(),
              "visible and navy-covered crossings resolve frontal depth");
        bool above = false;
        for (const auto &step : occluded.document.routeSteps)
            if (step.gateId == gate->id && step.kind == "guide")
                above |= ot::routeStepGeometry(occluded.document, step).pathPosition.y() >
                         gate->elevation + gate->height / 2 + .2;
        check(above,
              "triple source excursions become height above the top stock, not ground U-turns");
        for (const auto &pass : tripleOccluded)
            check(pass.face == "front",
                  "three front-to-back source crossings keep the same accepted face");
    } else
        check(false, "progressive triple fixture remains a physical triple");
    const auto forwardLadder = passagesFor(occluded.document, "ladder");
    const auto reversedOccluded =
        ot::recognizeTrack(inkSchematic(true, true, false, 0, true), manual);
    const auto backwardLadder = passagesFor(reversedOccluded.document, "ladder");
    check(forwardLadder.size() == 2 && backwardLadder.size() == 2,
          "occluded upright ascent/reversal preserves two different aperture visits");
    if (forwardLadder.size() == 2 && backwardLadder.size() == 2) {
        check(forwardLadder[0].level == 0 && forwardLadder[1].level == 1 &&
                  backwardLadder[0].level == 1 && backwardLadder[1].level == 0,
              "reversed source arrows reverse actual helical height progress");
        check(forwardLadder[0].face == forwardLadder[1].face &&
                  backwardLadder[0].face == backwardLadder[1].face &&
                  forwardLadder[0].face != backwardLadder[0].face,
              "occlusion resolves same-face progressive passes and their reverse");
    }
    const auto mirroredOccluded = ot::recognizeTrack(occludedImage.mirrored(true, false), manual);
    const auto *up = findType(occluded.document, "ladder"),
               *mirrorUp = findType(mirroredOccluded.document, "ladder");
    if (up && mirrorUp) {
        const auto a =
            up->extra.value("recognition").toObject().value("progressiveTurns").toArray();
        const auto b =
            mirrorUp->extra.value("recognition").toObject().value("progressiveTurns").toArray();
        check(!a.isEmpty() && !b.isEmpty() &&
                  a[0].toObject().value("signedSweepDegrees").toDouble() *
                          b[0].toObject().value("signedSweepDegrees").toDouble() <
                      0,
              "mirroring source occlusion reverses upright winding without a type preset");
    } else
        check(false, "mirrored source keeps the upright structure");
    const auto cubeOccluded = passagesFor(occluded.document, "cube");
    check(cubeOccluded.size() == 2 && cubeOccluded[0].face == "left" &&
              cubeOccluded[1].face == "top",
          "isometric left-wall entry is not relabelled by rotating the cube to its front");
    for (const auto &step : occluded.document.routeSteps) {
        if (step.kind != "pass")
            continue;
        const auto g = ot::routeStepGeometry(occluded.document, step);
        check(g.valid && QVector3D::dotProduct(g.pathTangent, g.normal) > .01f,
              "observed oblique flight stays in the physical portal's accepted hemisphere");
        check(std::abs(QVector3D::dotProduct(g.pathPosition - g.position, g.normal)) < .0001,
              "source aperture offsets stay on the unchanged native checkpoint plane");
    }
    const auto edgeProgress = ot::recognizeTrack(inkSchematic(true, false, false, 2), manual);
    if (const auto *gate = findType(edgeProgress.document, "ladder")) {
        const auto r = gate->extra.value("recognition").toObject();
        check(r.value("maneuverPattern").toString() == "source-progressive-upright",
              "edge-on S plus hidden endpoints is fitted as upright progression, not a horizontal "
              "pi snap");
        const auto p = passagesFor(edgeProgress.document, "ladder");
        check(p.size() == 2 && p[0].face == p[1].face && p[0].level != p[1].level,
              "middle projected plane crossing outside the upright is not a hidden third window "
              "pass");
        int turns = 0;
        bool outside = false;
        for (const auto &step : edgeProgress.document.routeSteps)
            if (step.gateId == gate->id && step.kind == "guide") {
                turns += step.extra.value("raceCheckpoint").toBool();
                outside |= std::abs(step.localGuide.x()) > 1.3f;
            }
        check(turns == 1 && outside,
              "one essential side check preserves the outside-upright helix");
        check(r.value("alongBarExtentInferred").toBool(),
              "unobservable edge-on along-bar extent stays explicit");
    } else
        check(false, "edge-on progressive fixture retains its physical ladder");

    std::cout << (failures ? "Recognizer tests FAILED\n" : "Recognizer tests passed\n");
    return failures ? 1 : 0;
}
