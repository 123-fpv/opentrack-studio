#include "recognizer.h"
#include <QCoreApplication>
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

    std::cout << (failures ? "Recognizer tests FAILED\n" : "Recognizer tests passed\n");
    return failures ? 1 : 0;
}
