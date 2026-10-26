#include "document.h"
#include <QCoreApplication>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QTemporaryDir>
#include <iostream>

namespace {
int failures = 0;
void check(bool condition, const char *message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        ++failures;
    }
}
} // namespace

int main(int argc, char **argv) {
    QCoreApplication application(argc, argv);
    ot::Gate first;
    first.id = ot::newGateId();
    first.extra.insert("custom", QJsonObject{{"unrelated", 42}});
    ot::Gate second = first;
    second.id = ot::newGateId();
    second.type = "pillar";
    check(ot::gateColor(first) == QColor("#4b4b4b"), "legacy frame keeps its neutral default");
    check(ot::gateColor(second) == QColor("#303030"), "pillar keeps its neutral default");
    QString error;
    check(ot::setGateColor(first, QColor("#e14a25"), &error),
          "frame accepts an explicit RGB colour");
    check(first.extra.value("color").toString() == "#E14A25",
          "stored RGB text is canonical uppercase");
    check(ot::setGateColor(second, QColor("#2865d9"), &error),
          "pillar accepts an independent RGB colour");
    check(ot::gateColor(first) != ot::gateColor(second), "components retain independent hues");
    const auto unchanged = first.extra;
    check(!ot::setGateColor(first, QColor(), &error) && first.extra == unchanged,
          "invalid setter input is rejected atomically");
    check(!ot::setGateColor(first, QColor(10, 20, 30, 100), &error) && first.extra == unchanged,
          "transparency cannot make a solid component invisible");

    ot::TrackDocument source;
    source.gates = {first, second};
    const auto object = ot::toJson(source);
    ot::TrackDocument restored;
    check(ot::fromJson(object, &restored, &error), "coloured project parses");
    check(restored.gates.size() == 2 && ot::gateColor(restored.gates[0]) == ot::gateColor(first) &&
              ot::gateColor(restored.gates[1]) == ot::gateColor(second),
          "project roundtrip preserves every component colour");
    check(restored.gates[0].extra.value("custom") == first.extra.value("custom"),
          "unrelated extension metadata is retained");

    auto malformed = object;
    auto gates = malformed["gates"].toArray();
    auto item = gates[0].toObject();
    auto extra = item["extra"].toObject();
    for (const auto &bad : QList<QJsonValue>{"red", "#123", "#00112233", "#zz2211", 123,
                                             QJsonValue(QJsonValue::Null)}) {
        extra["color"] = bad;
        item["extra"] = extra;
        gates[0] = item;
        malformed["gates"] = gates;
        const auto before = ot::toJson(restored);
        check(!ot::fromJson(malformed, &restored, &error) && ot::toJson(restored) == before,
              "malformed persisted RGB is rejected without partial project replacement");
    }
    extra["color"] = "#a1b2c3";
    item["extra"] = extra;
    gates[0] = item;
    malformed["gates"] = gates;
    check(ot::fromJson(malformed, &restored, &error) &&
              restored.gates[0].extra.value("color").toString() == "#A1B2C3",
          "valid lowercase input is canonicalized on load");
    ot::resetGateColor(first);
    check(!first.extra.contains("color") && first.extra.contains("custom") &&
              ot::gateColor(first) == QColor("#4b4b4b"),
          "reset removes only the custom colour");

    QTemporaryDir temporary;
    const QString path = temporary.filePath("colours.otproj");
    check(ot::saveProject(path, source, &error), "valid coloured project saves");
    QFile saved(path);
    check(saved.open(QIODevice::ReadOnly), "saved project is readable");
    const QByteArray original = saved.readAll();
    saved.close();
    source.gates[0].extra["color"] = "rgb(1,2,3)";
    check(!ot::saveProject(path, source, &error), "invalid colour blocks saving");
    check(saved.open(QIODevice::ReadOnly) && saved.readAll() == original,
          "failed colour validation never overwrites an existing project");
    std::cout << (failures ? "Colour tests FAILED\n" : "Colour tests passed\n");
    return failures ? 1 : 0;
}
