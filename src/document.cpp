#include "document.h"

#include <QBuffer>
#include <QFile>
#include <QImageReader>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonParseError>
#include <QSaveFile>
#include <QSet>
#include <QUuid>

#include <algorithm>
#include <cmath>

namespace ot {
namespace {
constexpr qint64 kMaxProjectBytes = 128LL * 1024 * 1024;
constexpr qsizetype kMaxEncodedImageBytes = 96LL * 1024 * 1024;
constexpr int kMaxImageSide = 16384;
constexpr qint64 kMaxImagePixels = 64LL * 1024 * 1024;
constexpr qsizetype kMaxGates = 20000;
constexpr double kMaxCoordinate = 1000000.0;

bool fail(QString *error, const QString &message) {
    if (error)
        *error = message;
    return false;
}

bool validImageSize(const QSize &size) {
    return size.width() > 0 && size.height() > 0 && size.width() <= kMaxImageSide &&
           size.height() <= kMaxImageSide &&
           qint64(size.width()) * qint64(size.height()) <= kMaxImagePixels;
}

bool number(const QJsonObject &object, const QString &key, double *value, double minimum,
            double maximum, QString *error, bool required = true) {
    const auto entry = object.value(key);
    if (entry.isUndefined() && !required)
        return true;
    if (!entry.isDouble())
        return fail(error, QStringLiteral("字段 %1 必须是数字。").arg(key));
    const double candidate = entry.toDouble();
    if (!std::isfinite(candidate) || candidate < minimum || candidate > maximum)
        return fail(error, QStringLiteral("字段 %1 超出允许范围。").arg(key));
    *value = candidate;
    return true;
}

bool point(const QJsonValue &value, QPointF *output, QString *error, const QString &field) {
    if (!value.isArray())
        return fail(error, QStringLiteral("字段 %1 必须是两个坐标组成的数组。").arg(field));
    const auto coordinates = value.toArray();
    if (coordinates.size() != 2 || !coordinates.at(0).isDouble() || !coordinates.at(1).isDouble())
        return fail(error, QStringLiteral("字段 %1 必须包含两个数字。").arg(field));
    const double x = coordinates.at(0).toDouble();
    const double y = coordinates.at(1).toDouble();
    if (!std::isfinite(x) || !std::isfinite(y) || std::abs(x) > kMaxCoordinate ||
        std::abs(y) > kMaxCoordinate)
        return fail(error, QStringLiteral("字段 %1 包含无效坐标。").arg(field));
    *output = QPointF(x, y);
    return true;
}

bool textField(const QJsonObject &object, const QString &key, QString *output, qsizetype maximum,
               bool required, QString *error) {
    const auto value = object.value(key);
    if (value.isUndefined() && !required)
        return true;
    if (!value.isString())
        return fail(error, QStringLiteral("字段 %1 必须是文本。").arg(key));
    const QString candidate = value.toString();
    if (candidate.size() > maximum || (required && candidate.trimmed().isEmpty()))
        return fail(error, QStringLiteral("字段 %1 为空或过长。").arg(key));
    *output = candidate;
    return true;
}

bool boolField(const QJsonObject &object, const QString &key, bool *output, QString *error) {
    const auto value = object.value(key);
    if (value.isUndefined())
        return true;
    if (!value.isBool())
        return fail(error, QStringLiteral("字段 %1 必须是布尔值。").arg(key));
    *output = value.toBool();
    return true;
}

bool extraField(const QJsonObject &object, QJsonObject *output, QString *error) {
    const auto value = object.value(QStringLiteral("extra"));
    if (value.isUndefined())
        return true;
    if (!value.isObject())
        return fail(error, QStringLiteral("extra 字段必须是对象。"));
    *output = value.toObject();
    return true;
}

bool validateDocument(const TrackDocument &document, QString *error) {
    if (document.name.trimmed().isEmpty() || document.name.size() > 4096 ||
        document.scenery.size() > 4096)
        return fail(error, QStringLiteral("工程名称不能为空，名称长度不得超过 4096 个字符。"));
    if (document.gates.size() > kMaxGates)
        return fail(error, QStringLiteral("门架数量超过 20000 个。"));
    auto inRange = [](double value, double low, double high) {
        return std::isfinite(value) && value >= low && value <= high;
    };
    if (!inRange(document.metersPerPixel, 0.000001, 10000.0) ||
        !inRange(document.backgroundOrigin.x(), -kMaxCoordinate, kMaxCoordinate) ||
        !inRange(document.backgroundOrigin.y(), -kMaxCoordinate, kMaxCoordinate) ||
        !inRange(document.backgroundOpacity, 0.0, 1.0) ||
        !inRange(document.gridSize, 0.01, 10000.0))
        return fail(error, QStringLiteral("工程比例、网格或背景参数无效。"));
    if (!document.background.isNull() && !validImageSize(document.background.size()))
        return fail(error,
                    QStringLiteral("底图尺寸过大，最大边长为 16384，最大像素数为 6400 万。"));
    QSet<QString> ids;
    for (const Gate &gate : document.gates) {
        if (gate.id.trimmed().isEmpty() || gate.id.size() > 128 || ids.contains(gate.id))
            return fail(error, QStringLiteral("门架 ID 为空、过长或重复。"));
        ids.insert(gate.id);
        if (gate.type.trimmed().isEmpty() || gate.type.size() > 64 || gate.label.size() > 4096 ||
            !inRange(gate.position.x(), -kMaxCoordinate, kMaxCoordinate) ||
            !inRange(gate.position.y(), -kMaxCoordinate, kMaxCoordinate) ||
            !inRange(gate.elevation, -10000.0, 10000.0) || !inRange(gate.width, 0.01, 10000.0) ||
            !inRange(gate.height, 0.01, 10000.0) || !inRange(gate.yaw, -1000000000.0, 1000000000.0))
            return fail(error, QStringLiteral("门架 %1 的类型、尺寸或位置无效。").arg(gate.id));
    }
    return true;
}

QJsonArray coordinates(const QPointF &value) {
    return QJsonArray{value.x(), value.y()};
}
} // namespace

QJsonObject toJson(const TrackDocument &document) {
    QJsonArray gates;
    for (const Gate &gate : document.gates) {
        gates.append(QJsonObject{{QStringLiteral("id"), gate.id},
                                 {QStringLiteral("type"), gate.type},
                                 {QStringLiteral("position"), coordinates(gate.position)},
                                 {QStringLiteral("elevation"), gate.elevation},
                                 {QStringLiteral("width"), gate.width},
                                 {QStringLiteral("height"), gate.height},
                                 {QStringLiteral("yaw"), gate.yaw},
                                 {QStringLiteral("label"), gate.label},
                                 {QStringLiteral("extra"), gate.extra}});
    }
    QByteArray backgroundPng;
    if (!document.background.isNull()) {
        QBuffer buffer(&backgroundPng);
        buffer.open(QIODevice::WriteOnly);
        document.background.save(&buffer, "PNG");
    }
    return QJsonObject{
        {QStringLiteral("format"), QStringLiteral("OpenTrackStudio")},
        {QStringLiteral("version"), 1},
        {QStringLiteral("name"), document.name},
        {QStringLiteral("scenery"), document.scenery},
        {QStringLiteral("gates"), gates},
        {QStringLiteral("backgroundPng"), QString::fromLatin1(backgroundPng.toBase64())},
        {QStringLiteral("view"),
         QJsonObject{{QStringLiteral("metersPerPixel"), document.metersPerPixel},
                     {QStringLiteral("backgroundOrigin"), coordinates(document.backgroundOrigin)},
                     {QStringLiteral("backgroundOpacity"), document.backgroundOpacity},
                     {QStringLiteral("showBackground"), document.showBackground},
                     {QStringLiteral("gridSize"), document.gridSize},
                     {QStringLiteral("closedLoop"), document.closedLoop}}},
        {QStringLiteral("extra"), document.extra}};
}

bool fromJson(const QJsonObject &object, TrackDocument *document, QString *error) {
    if (!document)
        return fail(error, QStringLiteral("工程输出指针为空。"));
    if (object.value(QStringLiteral("format")).toString() != QStringLiteral("OpenTrackStudio") ||
        !object.value(QStringLiteral("version")).isDouble() ||
        object.value(QStringLiteral("version")).toDouble() != 1.0)
        return fail(error, QStringLiteral("不是受支持的 OpenTrackStudio v1 工程。"));

    TrackDocument candidate;
    if (!textField(object, QStringLiteral("name"), &candidate.name, 4096, true, error) ||
        !textField(object, QStringLiteral("scenery"), &candidate.scenery, 4096, false, error) ||
        !extraField(object, &candidate.extra, error))
        return false;
    const auto gatesValue = object.value(QStringLiteral("gates"));
    if (!gatesValue.isArray() || gatesValue.toArray().size() > kMaxGates)
        return fail(error, QStringLiteral("门架列表无效或超过 20000 个。"));
    const auto gates = gatesValue.toArray();
    candidate.gates.reserve(gates.size());
    for (const auto &value : gates) {
        if (!value.isObject())
            return fail(error, QStringLiteral("门架列表必须包含对象。"));
        const auto item = value.toObject();
        Gate gate;
        if (!textField(item, QStringLiteral("id"), &gate.id, 128, true, error) ||
            !textField(item, QStringLiteral("type"), &gate.type, 64, true, error) ||
            !textField(item, QStringLiteral("label"), &gate.label, 4096, false, error) ||
            !point(item.value(QStringLiteral("position")), &gate.position, error,
                   QStringLiteral("position")) ||
            !number(item, QStringLiteral("elevation"), &gate.elevation, -10000.0, 10000.0, error,
                    false) ||
            !number(item, QStringLiteral("width"), &gate.width, 0.01, 10000.0, error, false) ||
            !number(item, QStringLiteral("height"), &gate.height, 0.01, 10000.0, error, false) ||
            !number(item, QStringLiteral("yaw"), &gate.yaw, -1000000000.0, 1000000000.0, error,
                    false) ||
            !extraField(item, &gate.extra, error))
            return false;
        candidate.gates.append(gate);
    }

    const auto viewValue = object.value(QStringLiteral("view"));
    if (!viewValue.isUndefined()) {
        if (!viewValue.isObject())
            return fail(error, QStringLiteral("view 字段必须是对象。"));
        const auto view = viewValue.toObject();
        if (!number(view, QStringLiteral("metersPerPixel"), &candidate.metersPerPixel, 0.000001,
                    10000.0, error, false) ||
            !number(view, QStringLiteral("backgroundOpacity"), &candidate.backgroundOpacity, 0.0,
                    1.0, error, false) ||
            !number(view, QStringLiteral("gridSize"), &candidate.gridSize, 0.01, 10000.0, error,
                    false) ||
            !boolField(view, QStringLiteral("showBackground"), &candidate.showBackground, error) ||
            !boolField(view, QStringLiteral("closedLoop"), &candidate.closedLoop, error))
            return false;
        if (view.contains(QStringLiteral("backgroundOrigin")) &&
            !point(view.value(QStringLiteral("backgroundOrigin")), &candidate.backgroundOrigin,
                   error, QStringLiteral("backgroundOrigin")))
            return false;
    }

    const auto imageValue = object.value(QStringLiteral("backgroundPng"));
    if (!imageValue.isUndefined()) {
        if (!imageValue.isString())
            return fail(error, QStringLiteral("底图数据必须是 Base64 文本。"));
        const QString encoded = imageValue.toString();
        if (encoded.size() > kMaxEncodedImageBytes)
            return fail(error, QStringLiteral("嵌入底图数据过大。"));
        if (!encoded.isEmpty()) {
            const auto decoded = QByteArray::fromBase64Encoding(
                encoded.toLatin1(), QByteArray::AbortOnBase64DecodingErrors);
            if (!decoded)
                return fail(error, QStringLiteral("底图 Base64 数据无效。"));
            QByteArray imageBytes = decoded.decoded;
            QBuffer buffer(&imageBytes);
            buffer.open(QIODevice::ReadOnly);
            QImageReader reader(&buffer, "PNG");
            if (!validImageSize(reader.size()))
                return fail(error, QStringLiteral("底图尺寸无效或超过允许上限。"));
            candidate.background = reader.read();
            if (candidate.background.isNull())
                return fail(error, QStringLiteral("无法解码工程底图。"));
        }
    }
    if (!validateDocument(candidate, error))
        return false;
    *document = std::move(candidate);
    if (error)
        error->clear();
    return true;
}

bool saveProject(const QString &path, const TrackDocument &document, QString *error) {
    if (!validateDocument(document, error))
        return false;
    const QByteArray data = QJsonDocument(toJson(document)).toJson(QJsonDocument::Indented);
    if (data.size() > kMaxProjectBytes)
        return fail(error, QStringLiteral("工程文件超过 128 MiB 上限。"));
    QSaveFile file(path);
    if (!file.open(QIODevice::WriteOnly))
        return fail(error, QStringLiteral("无法保存工程：%1").arg(file.errorString()));
    if (file.write(data) != data.size())
        return fail(error, QStringLiteral("工程写入失败：%1").arg(file.errorString()));
    if (!file.commit())
        return fail(error, QStringLiteral("工程保存失败：%1").arg(file.errorString()));
    if (error)
        error->clear();
    return true;
}

bool loadProject(const QString &path, TrackDocument *document, QString *error) {
    if (!document)
        return fail(error, QStringLiteral("工程输出指针为空。"));
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly))
        return fail(error, QStringLiteral("无法打开工程：%1").arg(file.errorString()));
    if (file.size() > kMaxProjectBytes)
        return fail(error, QStringLiteral("工程文件超过 128 MiB 上限。"));
    const QByteArray data = file.read(kMaxProjectBytes + 1);
    if (data.size() > kMaxProjectBytes || file.error() != QFileDevice::NoError)
        return fail(error, QStringLiteral("工程过大或读取失败。"));
    QJsonParseError parseError;
    const auto json = QJsonDocument::fromJson(data, &parseError);
    if (parseError.error != QJsonParseError::NoError || !json.isObject())
        return fail(error, QStringLiteral("工程 JSON 无效：%1").arg(parseError.errorString()));
    return fromJson(json.object(), document, error);
}

QString newGateId() {
    return QUuid::createUuid().toString(QUuid::WithoutBraces);
}

QString gateTypeName(const QString &type) {
    if (type == QStringLiteral("standard"))
        return QStringLiteral("标准门");
    if (type == QStringLiteral("double"))
        return QStringLiteral("双联门");
    if (type == QStringLiteral("ladder"))
        return QStringLiteral("叠门");
    if (type == QStringLiteral("cube"))
        return QStringLiteral("立方门");
    if (type == QStringLiteral("pillar"))
        return QStringLiteral("转向柱");
    if (type == QStringLiteral("flag"))
        return QStringLiteral("绕旗");
    if (type == QStringLiteral("dive"))
        return QStringLiteral("俯冲门");
    if (type == QStringLiteral("start"))
        return QStringLiteral("起终点门");
    return QStringLiteral("自定义门");
}

TrackDocument demoDocument() {
    TrackDocument document;
    document.name = QStringLiteral("青空环线 / Aozora Circuit");
    struct DemoGate {
        double x, y;
        const char *type;
        double elevation, width, height;
    };
    // Original demonstration layout. Coordinates are metres in the editor plane.
    const DemoGate layout[] = {
        {-29, 13, "start", 1.2, 2.4, 2.4},    {-33, 3, "standard", 1.2, 4.8, 2.4},
        {-26, -9, "standard", 1.2, 3.0, 3.0}, {-19, -19, "standard", 1.2, 2.4, 2.4},
        {-9, -24, "pillar", 1.5, .4, 3.0},    {3, -25, "standard", 2.5, 2.4, 4.8},
        {16, -22, "standard", 1.2, 2.4, 2.4}, {25, -13, "standard", 1.2, 2.4, 2.4},
        {31, -3, "standard", 1.2, 3.0, 3.0},  {33, 8, "standard", 1.2, 4.8, 2.4},
        {24, 17, "standard", 1.2, 2.4, 2.4},  {12, 21, "pillar", 1.5, .4, 3.0},
        {2, 16, "standard", 3.5, 2.4, 2.4},   {6, 6, "standard", 2.5, 2.4, 4.8},
        {-1, 0, "standard", 1.2, 2.4, 2.4},   {-10, 7, "standard", 1.2, 2.4, 2.4},
        {-13, 19, "standard", 1.2, 3.0, 3.0}, {-22, 22, "pillar", 1.5, .4, 3.0}};
    constexpr qsizetype count = sizeof(layout) / sizeof(layout[0]);
    for (qsizetype i = 0; i < count; ++i) {
        Gate gate;
        gate.id = newGateId();
        gate.type = QString::fromLatin1(layout[i].type);
        gate.position = QPointF(layout[i].x, layout[i].y);
        gate.elevation = layout[i].elevation;
        gate.width = layout[i].width;
        gate.height = layout[i].height;
        const auto &previous = layout[(i + count - 1) % count];
        const auto &next = layout[(i + 1) % count];
        gate.yaw =
            std::atan2(next.x - previous.x, next.y - previous.y) * 180.0 / 3.14159265358979323846;
        gate.label = i == 0 ? QStringLiteral("START / FINISH")
                            : QStringLiteral("G%1").arg(i + 1, 2, 10, QLatin1Char('0'));
        document.gates.append(gate);
    }
    return document;
}

QPointF gatePassagePosition(const Gate &gate) {
    if (gate.type != QStringLiteral("pillar"))
        return gate.position;
    const double yaw = gate.yaw * 3.14159265358979323846 / 180.0;
    const double side = gate.extra.value(QStringLiteral("passSide")).toInt(1) < 0 ? -1.0 : 1.0;
    const QPointF right(std::cos(yaw), -std::sin(yaw));
    return gate.position + right * ((gate.width / 2.0 + 1.55) * side);
}

QPainterPath routePath(const TrackDocument &document) {
    QPainterPath path;
    const qsizetype count = document.gates.size();
    if (count == 0)
        return path;
    path.moveTo(gatePassagePosition(document.gates.first()));
    if (count == 1)
        return path;
    if (count == 2) {
        path.lineTo(gatePassagePosition(document.gates.at(1)));
        if (document.closedLoop)
            path.lineTo(gatePassagePosition(document.gates.first()));
        return path;
    }
    const auto at = [&document, count](qsizetype index) {
        if (document.closedLoop)
            return gatePassagePosition(document.gates.at((index % count + count) % count));
        return gatePassagePosition(document.gates.at(std::clamp(index, qsizetype(0), count - 1)));
    };
    const qsizetype segmentCount = document.closedLoop ? count : count - 1;
    for (qsizetype i = 0; i < segmentCount; ++i) {
        const QPointF previous = at(i - 1);
        const QPointF start = at(i);
        const QPointF end = at(i + 1);
        const QPointF next = at(i + 2);
        path.cubicTo(start + (end - previous) / 6.0, end - (next - start) / 6.0, end);
    }
    if (document.closedLoop)
        path.closeSubpath();
    return path;
}

double routeLength(const TrackDocument &document) {
    // Qt integrates cubic segments adaptively; this is geometric path length,
    // not a claimed flight distance or physics simulation.
    return routePath(document).length();
}

} // namespace ot
