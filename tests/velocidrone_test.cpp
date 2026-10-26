#include "document.h"
#include "velocidrone.h"

#include <QCoreApplication>
#include <QFile>
#include <QJsonArray>
#include <QTemporaryDir>
#include <cmath>
#include <iostream>
#include <limits>

namespace {
int failures = 0;
void require(bool condition, const char *message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        ++failures;
    }
}
bool near(double a, double b, double tolerance = 0.011) {
    return std::abs(a - b) < tolerance;
}
bool writeFile(const QString &path, const QByteArray &data) {
    QFile file(path);
    return file.open(QIODevice::WriteOnly) && file.write(data) == data.size();
}
QByteArray readFile(const QString &path) {
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly))
        return {};
    return file.readAll();
}
// Original, synthetic one-gate course. This independent known-answer fixture was
// encrypted using .NET System.Security.Cryptography.Aes, not this implementation.
// Scene 16; UTF-8 name "Fixture {中文}"; one 90-degree gate at (-12.34,56.78),
// base altitude 0; native quaternion [707,0,707,0]; custom barrier + weather.
const QByteArray Golden = "e73GF0q1zUigczG3hm/zQA+O8GS0uwOtT/CYYDVuWYsC2p5td3187JLqj4F0+Rfl"
                          "6nipuA6VylE0eRMeLc4K+7PXiYlRSkDThcMIlE1XggU5dtzbZpgfTL/31rOoX0t/"
                          "Xyo8H7Wn+yerHil3NH9ewpXlgARWyqoGR80VLpJmEHMkycKJ5EWwB1Kn/ZB0KgakL"
                          "lxKP1JIj5jplQZ4UcM8tHSjHwx1js6pL2T0nzPl1o6615JoYICDFn+rIMFqa4Ncs6"
                          "N6mcK/hovZTxikFOkd0w+bqvy2Lcat1m7Akvkw1u75gx/7dO3+6TYA6U6TzM7oiH3"
                          "FDvJnUwtgPiF7ZNrZkdJ/TNr/OZ4hMb3vYLlk5CXcSYT99ACdutzeqY9elzP+EbWSB"
                          "MX2aTjOnX80EjTmrd+XHiBlJFCaRsGzKds7dLc=";
} // namespace

int main(int argc, char **argv) {
    QCoreApplication application(argc, argv);
#ifndef Q_OS_WIN
    std::cout << "SKIP: native .trk codec uses Windows CNG in this release.\n";
    return 0;
#else
    QTemporaryDir directory;
    require(directory.isValid(), "temporary directory");
    if (!directory.isValid())
        return 1;
    QString error;
    const auto originalPath = directory.filePath(QStringLiteral("golden.trk"));
    require(writeFile(originalPath, Golden), "write independently encoded fixture");
    ot::TrackDocument imported;
    require(ot::importVelocidrone(originalPath, &imported, &error), "decrypt .NET fixture");
    if (imported.gates.isEmpty()) {
        std::cerr << error.toStdString() << '\n';
        return 1;
    }
    require(imported.name == QStringLiteral("Fixture {中文}"), "UTF-8 name and braces preserved");
    require(imported.scenery == QStringLiteral("Empty Scene Day"), "scene mapping");
    require(imported.gates.size() == 1, "native gate count");
    const auto &first = imported.gates.first();
    require(first.type == QStringLiteral("start"), "start/finish mapping");
    require(near(first.position.x(), -12.34) && near(first.position.y(), 56.78),
            "centimetres to metres");
    require(near(first.elevation, 1.2), "native base to editor center altitude");
    require(near(first.yaw, 90, 0.1), "WXYZ quaternion and native positive yaw");
    require(near(first.width, 2.4) && near(first.height, 2.4), "percent dimensions");
    const auto root = imported.extra.value(QStringLiteral("velocidroneRoot")).toObject();
    require(root.value(QStringLiteral("barriers")).toArray().size() == 1, "barriers retained");
    require(
        root.value(QStringLiteral("weather")).toObject().value(QStringLiteral("hour")).toInt() ==
            14,
        "weather retained");
    const auto savedPath = directory.filePath(QStringLiteral("native-roundtrip.trk"));
    require(ot::exportVelocidrone(savedPath, imported, &error), "export native import");
    require(readFile(savedPath) == Golden,
            "unchanged native course matches independent encrypted fixture byte for byte");

    imported.gates[0].position += QPointF(2, -3);
    imported.gates[0].yaw = 0;
    imported.gates[0].height = 4.8;
    imported.gates[0].elevation = 4.4;
    require(ot::exportVelocidrone(savedPath, imported, &error), "export native geometry edit");
    ot::TrackDocument edited;
    require(ot::importVelocidrone(savedPath, &edited, &error), "reimport native geometry edit");
    if (!edited.gates.isEmpty()) {
        require(near(edited.gates[0].position.x(), -10.34) &&
                    near(edited.gates[0].position.y(), 53.78),
                "native move retains units");
        require(near(edited.gates[0].yaw, 0, 0.1), "native yaw edit");
        require(near(edited.gates[0].height, 4.8) && near(edited.gates[0].elevation, 4.4),
                "native resize retains center altitude");
        require(edited.extra.value(QStringLiteral("velocidroneRoot"))
                        .toObject()
                        .value(QStringLiteral("barriers")) ==
                    root.value(QStringLiteral("barriers")),
                "unrelated native barriers unchanged after geometry edit");
    }

    ot::TrackDocument created;
    created.name = QStringLiteral("测试赛道\nline");
    ot::Gate start;
    start.id = ot::newGateId();
    start.type = QStringLiteral("start");
    start.position = QPointF(1.23, -4.56);
    start.yaw = 90;
    ot::Gate gate;
    gate.id = ot::newGateId();
    gate.position = QPointF(20, 30);
    gate.elevation = 7.45;
    gate.width = 4.8;
    gate.height = 3.6;
    gate.yaw = -45;
    ot::Gate flag;
    flag.id = ot::newGateId();
    flag.type = QStringLiteral("flag");
    flag.position = QPointF(40, 10);
    created.gates = {start, gate, flag};
    require(ot::exportVelocidrone(savedPath, created, &error), "export new course");
    ot::TrackDocument roundtrip;
    require(ot::importVelocidrone(savedPath, &roundtrip, &error), "read generated course");
    require(roundtrip.gates.size() == 3, "new course gate count");
    require(roundtrip.name == QStringLiteral("测试赛道 line"),
            "name newlines cannot break native framing");
    require(roundtrip.scenery == QStringLiteral("Football Stadium"), "football scene number 8");
    if (roundtrip.gates.size() == 3) {
        require(near(roundtrip.gates[1].width, 4.8) && near(roundtrip.gates[1].height, 3.6),
                "new dimensions roundtrip");
        require(near(roundtrip.gates[1].elevation, 7.45), "new center altitude roundtrip");
        require(near(roundtrip.gates[1].yaw, -45, 0.2), "negative yaw roundtrip");
        require(roundtrip.gates[2]
                        .extra.value(QStringLiteral("velocidroneObject"))
                        .toObject()
                        .value(QStringLiteral("prefab"))
                        .toInt() == 170,
                "flag native prefab");
    }
    const QByteArray sentinel("keep existing file");
    require(writeFile(savedPath, sentinel), "write sentinel");
    created.gates[1].type = QStringLiteral("cube");
    require(!ot::exportVelocidrone(savedPath, created, &error) && !error.isEmpty(),
            "unsupported newly created component rejected");
    require(readFile(savedPath) == sentinel, "failed export preserves existing file");
    created.gates[1].type = QStringLiteral("standard");
    created.gates[1].position.setX(std::numeric_limits<double>::quiet_NaN());
    require(!ot::exportVelocidrone(savedPath, created, &error),
            "NaN rejected before numeric conversion");
    created.gates[1].position.setX(20);
    created.gates[1].type = QStringLiteral("start");
    require(!ot::exportVelocidrone(savedPath, created, &error),
            "ambiguous multiple start gates rejected");

    ot::TrackDocument unchanged;
    unchanged.name = QStringLiteral("Keep me");
    const auto badPath = directory.filePath(QStringLiteral("bad.trk"));
    require(writeFile(badPath, "not valid base64 %"), "write invalid base64");
    require(!ot::importVelocidrone(badPath, &unchanged, &error), "invalid base64 rejected");
    require(unchanged.name == QStringLiteral("Keep me"), "failed import is transactional");
    auto ciphertext = QByteArray::fromBase64(Golden);
    ciphertext[ciphertext.size() - 1] = static_cast<char>(ciphertext.back() ^ 0xFF);
    require(writeFile(badPath, ciphertext.toBase64()), "write invalid padding");
    require(!ot::importVelocidrone(badPath, &unchanged, &error), "corrupt AES padding rejected");
    require(unchanged.name == QStringLiteral("Keep me"), "corrupt import preserves document");
    require(!ot::importVelocidrone(originalPath, nullptr, &error), "null output rejected");
    if (!failures)
        std::cout << "PASS: native schema, independent AES fixture, geometry, preservation and "
                     "invalid input.\n";
    return failures ? 1 : 0;
#endif
}
