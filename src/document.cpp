#include "document.h"

#include <QBuffer>
#include <QCryptographicHash>
#include <QDataStream>
#include <QFile>
#include <QHash>
#include <QImageReader>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonParseError>
#include <QQuaternion>
#include <QSaveFile>
#include <QSet>
#include <QStringList>
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
constexpr qsizetype kMaxRouteSteps = 100000;
constexpr double kMaxCoordinate = 1000000.0;
constexpr float kRouteClearance = .20f;

RouteStepGeometry geometryForStep(const Gate *gate, const RouteStep &step);
QVector3D curveTangentForStep(const Gate *gate, const RouteStep &step);

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

bool vector3(const QJsonValue &value, QVector3D *output, QString *error) {
    if (!value.isArray() || value.toArray().size() != 3)
        return fail(error, QStringLiteral("localGuide 必须是三个数字组成的数组。"));
    const auto array = value.toArray();
    QVector3D candidate;
    for (int i = 0; i < 3; ++i) {
        const auto entry = array[i];
        if (!entry.isDouble() || !std::isfinite(entry.toDouble()) ||
            std::abs(entry.toDouble()) > kMaxCoordinate)
            return fail(error, QStringLiteral("localGuide 包含无效坐标。"));
        candidate[i] = static_cast<float>(entry.toDouble());
    }
    *output = candidate;
    return true;
}

bool indexField(const QJsonObject &object, const QString &key, int *output, QString *error) {
    const auto value = object.value(key);
    if (value.isUndefined())
        return true;
    const double index = value.toDouble(-1);
    if (!value.isDouble() || !std::isfinite(index) || index < 0 || index > 10000 ||
        index != std::floor(index))
        return fail(error, QStringLiteral("路线字段 %1 必须是非负整数。").arg(key));
    *output = static_cast<int>(index);
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

bool canonicalRgbColor(const QJsonValue &raw, QString *canonical, QString *error) {
    if (raw.isUndefined())
        return true;
    if (!raw.isString())
        return fail(error, QStringLiteral("组件颜色必须是 #RRGGBB 文本。"));
    const QString value = raw.toString();
    if (value.size() != 7 || value[0] != '#')
        return fail(error, QStringLiteral("组件颜色必须是六位十六进制 #RRGGBB。"));
    for (int i = 1; i < value.size(); ++i) {
        const ushort c = value[i].unicode();
        if (!((c >= '0' && c <= '9') || (c >= 'A' && c <= 'F') || (c >= 'a' && c <= 'f')))
            return fail(error, QStringLiteral("组件颜色包含无效的十六进制数字。"));
    }
    if (canonical)
        *canonical = value.toUpper();
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
    QHash<QString, const Gate *> gatesById;
    for (const Gate &gate : document.gates) {
        if (!canonicalRgbColor(gate.extra.value(QStringLiteral("color")), nullptr, error))
            return false;
        if (gate.id.trimmed().isEmpty() || gate.id.size() > 128 || ids.contains(gate.id))
            return fail(error, QStringLiteral("门架 ID 为空、过长或重复。"));
        ids.insert(gate.id);
        gatesById.insert(gate.id, &gate);
        if (gate.type.trimmed().isEmpty() || gate.type.size() > 64 || gate.label.size() > 4096 ||
            !inRange(gate.position.x(), -kMaxCoordinate, kMaxCoordinate) ||
            !inRange(gate.position.y(), -kMaxCoordinate, kMaxCoordinate) ||
            !inRange(gate.elevation, -10000.0, 10000.0) || !inRange(gate.width, 0.01, 10000.0) ||
            !inRange(gate.height, 0.01, 10000.0) || !inRange(gate.yaw, -1000000000.0, 1000000000.0))
            return fail(error, QStringLiteral("门架 %1 的类型、尺寸或位置无效。").arg(gate.id));
        if (gate.extra.value(QStringLiteral("frameGeometry")).toString() ==
            QStringLiteral("centerline-v1")) {
            for (const auto &field :
                 {QStringLiteral("beamThickness"), QStringLiteral("beamOverhang")}) {
                const auto value = gate.extra.value(field);
                if (value.isUndefined())
                    continue;
                const double minimum = field == QStringLiteral("beamThickness") ? 0.001 : 0.0;
                if (!value.isDouble() || !inRange(value.toDouble(), minimum, 10000.0))
                    return fail(error,
                                QStringLiteral("门架 %1 的梁材参数 %2 无效。").arg(gate.id, field));
            }
            const auto metrics = gateFrameMetrics(gate);
            for (const auto &field : {QStringLiteral("passColumn"), QStringLiteral("passLevel")}) {
                const auto value = gate.extra.value(field);
                if (value.isUndefined())
                    continue;
                const int count =
                    field == QStringLiteral("passColumn") ? metrics.columns : metrics.levels;
                const double index = value.toDouble(-1);
                if (!value.isDouble() || !std::isfinite(index) || index < 0 || index >= count ||
                    index != std::floor(index))
                    return fail(error,
                                QStringLiteral("门架 %1 的通行单元 %2 无效。").arg(gate.id, field));
            }
        }
    }
    if (document.routeSteps.size() > kMaxRouteSteps)
        return fail(error, QStringLiteral("路线步骤超过 100000 个。"));
    ids.clear();
    for (const auto &step : document.routeSteps) {
        if (step.id.trimmed().isEmpty() || step.id.size() > 128 || ids.contains(step.id) ||
            step.gateId.size() > 128 || step.label.size() > 4096 ||
            (step.kind != QStringLiteral("pass") && step.kind != QStringLiteral("guide")) ||
            step.column < 0 || step.level < 0 || step.column > 10000 || step.level > 10000 ||
            !QStringList{QStringLiteral("front"), QStringLiteral("back"), QStringLiteral("left"),
                         QStringLiteral("right"), QStringLiteral("top"), QStringLiteral("bottom")}
                 .contains(step.face))
            return fail(error, QStringLiteral("路线步骤的 ID、类型或标签无效。"));
        ids.insert(step.id);
        const auto *gate = gatesById.value(step.gateId, nullptr);
        if ((step.kind == QStringLiteral("pass") && !gate) || (!step.gateId.isEmpty() && !gate))
            return fail(error, QStringLiteral("路线步骤 %1 引用了不存在的组件。").arg(step.id));
        for (int axis = 0; axis < 3; ++axis) {
            const double limit = gate ? 1000.0 : kMaxCoordinate;
            if (!inRange(step.localGuide[axis], -limit, limit))
                return fail(error, QStringLiteral("路线步骤 %1 的引导坐标无效。").arg(step.id));
        }
        if (!geometryForStep(gate, step).valid)
            return fail(error, QStringLiteral("路线步骤 %1 的面、通过格或开口无效。").arg(step.id));
        const auto hint = step.extra.value(QStringLiteral("curveTangent"));
        if (!hint.isUndefined()) {
            QVector3D tangent;
            if (!vector3(hint, &tangent, error) || tangent.lengthSquared() < 1e-12f)
                return fail(error, QStringLiteral("路线步骤 %1 的曲线切向量无效。").arg(step.id));
        }
        const auto flight = step.extra.value(QStringLiteral("flightTangent"));
        if (!flight.isUndefined()) {
            QVector3D tangent;
            if (step.kind != QStringLiteral("pass") || !vector3(flight, &tangent, error) ||
                tangent.lengthSquared() < 1e-12f)
                return fail(error, QStringLiteral("过门飞行切向量无效。"));
        }
        for (const auto &field : {QStringLiteral("apertureU"), QStringLiteral("apertureV")}) {
            const auto value = step.extra.value(field);
            if (!value.isUndefined() && (step.kind != QStringLiteral("pass") || !value.isDouble() ||
                                         !inRange(value.toDouble(), -1, 1)))
                return fail(error, QStringLiteral("窗口内穿越偏移必须是有限数字。"));
        }
        const auto checkpointUp = step.extra.value(QStringLiteral("checkpointUp"));
        if (!checkpointUp.isUndefined()) {
            QVector3D up;
            if (!vector3(checkpointUp, &up, error) || up.lengthSquared() < 1e-12f)
                return fail(error, QStringLiteral("辅助检查点上方向无效。"));
        }
        const auto race = step.extra.value(QStringLiteral("raceCheckpoint"));
        if (!race.isUndefined() && !race.isBool())
            return fail(error, QStringLiteral("辅助检查点标记必须是布尔值。"));
        if (race.toBool(false) && (step.kind != QStringLiteral("guide") || hint.isUndefined()))
            return fail(error, QStringLiteral("辅助转向检查点需要有效的引导切向量。"));
        for (const auto &field :
             {QStringLiteral("checkpointWidth"), QStringLiteral("checkpointHeight")}) {
            const auto value = step.extra.value(field);
            if (!value.isUndefined() && (!value.isDouble() || !inRange(value.toDouble(), .1, 100)))
                return fail(error, QStringLiteral("辅助检查点尺寸无效。"));
        }
    }
    if (!inRange(document.takeoff.position.x(), -kMaxCoordinate, kMaxCoordinate) ||
        !inRange(document.takeoff.position.y(), -kMaxCoordinate, kMaxCoordinate) ||
        !inRange(document.takeoff.elevation, -10000, 10000) ||
        !inRange(document.takeoff.yaw, -1000000000, 1000000000))
        return fail(error, QStringLiteral("独立起飞点的位置、高度或朝向无效。"));
    return true;
}

QJsonArray coordinates(const QPointF &value) {
    return QJsonArray{value.x(), value.y()};
}
} // namespace

QJsonObject toJson(const TrackDocument &document) {
    QJsonArray gates;
    for (const Gate &gate : document.gates) {
        auto extra = gate.extra;
        QString color;
        if (extra.contains(QStringLiteral("color")) &&
            canonicalRgbColor(extra.value(QStringLiteral("color")), &color, nullptr))
            extra.insert(QStringLiteral("color"), color);
        gates.append(QJsonObject{{QStringLiteral("id"), gate.id},
                                 {QStringLiteral("type"), gate.type},
                                 {QStringLiteral("position"), coordinates(gate.position)},
                                 {QStringLiteral("elevation"), gate.elevation},
                                 {QStringLiteral("width"), gate.width},
                                 {QStringLiteral("height"), gate.height},
                                 {QStringLiteral("yaw"), gate.yaw},
                                 {QStringLiteral("label"), gate.label},
                                 {QStringLiteral("extra"), extra}});
    }
    QByteArray backgroundPng;
    if (!document.background.isNull()) {
        QBuffer buffer(&backgroundPng);
        buffer.open(QIODevice::WriteOnly);
        document.background.save(&buffer, "PNG");
    }
    QJsonObject result{
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
    if (document.routeStepsDefined || !document.routeSteps.isEmpty()) {
        QJsonArray steps;
        for (const auto &step : document.routeSteps)
            steps.append(QJsonObject{
                {QStringLiteral("id"), step.id},
                {QStringLiteral("gateId"), step.gateId},
                {QStringLiteral("kind"), step.kind},
                {QStringLiteral("face"), step.face},
                {QStringLiteral("column"), step.column},
                {QStringLiteral("level"), step.level},
                {QStringLiteral("reverse"), step.reverse},
                {QStringLiteral("localGuide"),
                 QJsonArray{step.localGuide.x(), step.localGuide.y(), step.localGuide.z()}},
                {QStringLiteral("label"), step.label},
                {QStringLiteral("extra"), step.extra}});
        result.insert(QStringLiteral("routeSteps"), steps);
    }
    result.insert(QStringLiteral("takeoff"),
                  QJsonObject{{QStringLiteral("enabled"), document.takeoff.enabled},
                              {QStringLiteral("position"), coordinates(document.takeoff.position)},
                              {QStringLiteral("elevation"), document.takeoff.elevation},
                              {QStringLiteral("yaw"), document.takeoff.yaw},
                              {QStringLiteral("extra"), document.takeoff.extra}});
    return result;
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
        if (gate.extra.contains(QStringLiteral("color"))) {
            QString color;
            if (!canonicalRgbColor(gate.extra.value(QStringLiteral("color")), &color, error))
                return false;
            gate.extra.insert(QStringLiteral("color"), color);
        }
        candidate.gates.append(gate);
    }

    if (object.contains(QStringLiteral("routeSteps"))) {
        const auto value = object.value(QStringLiteral("routeSteps"));
        if (!value.isArray() || value.toArray().size() > kMaxRouteSteps)
            return fail(error, QStringLiteral("路线步骤必须是最多 100000 项的数组。"));
        candidate.routeStepsDefined = true;
        for (const auto &entry : value.toArray()) {
            if (!entry.isObject())
                return fail(error, QStringLiteral("路线步骤必须是对象。"));
            const auto item = entry.toObject();
            RouteStep step;
            if (!textField(item, QStringLiteral("id"), &step.id, 128, true, error) ||
                !textField(item, QStringLiteral("gateId"), &step.gateId, 128, false, error) ||
                !textField(item, QStringLiteral("kind"), &step.kind, 32, false, error) ||
                !textField(item, QStringLiteral("face"), &step.face, 32, false, error) ||
                !textField(item, QStringLiteral("label"), &step.label, 4096, false, error) ||
                !indexField(item, QStringLiteral("column"), &step.column, error) ||
                !indexField(item, QStringLiteral("level"), &step.level, error) ||
                !boolField(item, QStringLiteral("reverse"), &step.reverse, error) ||
                !extraField(item, &step.extra, error))
                return false;
            if (item.contains(QStringLiteral("localGuide")) &&
                !vector3(item.value(QStringLiteral("localGuide")), &step.localGuide, error))
                return false;
            candidate.routeSteps.append(step);
        }
    }
    if (object.contains(QStringLiteral("takeoff"))) {
        const auto value = object.value(QStringLiteral("takeoff"));
        if (!value.isObject())
            return fail(error, QStringLiteral("takeoff 必须是独立起飞点对象。"));
        const auto item = value.toObject();
        if (!boolField(item, QStringLiteral("enabled"), &candidate.takeoff.enabled, error) ||
            !number(item, QStringLiteral("elevation"), &candidate.takeoff.elevation, -10000, 10000,
                    error, false) ||
            !number(item, QStringLiteral("yaw"), &candidate.takeoff.yaw, -1000000000, 1000000000,
                    error, false) ||
            !extraField(item, &candidate.takeoff.extra, error))
            return false;
        if (item.contains(QStringLiteral("position")) &&
            !point(item.value(QStringLiteral("position")), &candidate.takeoff.position, error,
                   QStringLiteral("takeoff.position")))
            return false;
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

QColor gateColor(const Gate &gate) {
    QString canonical;
    if (gate.extra.contains(QStringLiteral("color")) &&
        canonicalRgbColor(gate.extra.value(QStringLiteral("color")), &canonical, nullptr))
        return QColor(canonical);
    if (gate.type == QStringLiteral("start"))
        return QColor("#292929");
    if (gate.type == QStringLiteral("pillar"))
        return QColor("#303030");
    if (gate.type == QStringLiteral("flag"))
        return QColor("#858585");
    return QColor("#4B4B4B");
}

bool setGateColor(Gate &gate, const QColor &color, QString *error) {
    if (error)
        error->clear();
    if (!color.isValid())
        return fail(error, QStringLiteral("请选择有效的组件颜色。"));
    if (color.alpha() != 255)
        return fail(error, QStringLiteral("组件颜色必须是不透明的 RGB 颜色。"));
    gate.extra.insert(QStringLiteral("color"), color.toRgb().name(QColor::HexRgb).toUpper());
    return true;
}

void resetGateColor(Gate &gate) {
    gate.extra.remove(QStringLiteral("color"));
}

QString gateTypeName(const QString &type) {
    if (type == QStringLiteral("standard"))
        return QStringLiteral("标准门");
    if (type == QStringLiteral("double"))
        return QStringLiteral("双联门");
    if (type == QStringLiteral("triple"))
        return QStringLiteral("三联门");
    if (type == QStringLiteral("ladder"))
        return QStringLiteral("叠门");
    if (type == QStringLiteral("cube"))
        return QStringLiteral("立方门");
    if (type == QStringLiteral("cube-stack"))
        return QStringLiteral("双层立方门");
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
        {-29, 13, "start", 1.6, 3.0, 3.0},    {-33, 3, "standard", 1.6, 3.0, 3.0},
        {-26, -9, "standard", 1.6, 3.0, 3.0}, {-19, -19, "standard", 1.6, 3.0, 3.0},
        {-9, -24, "pillar", 1.6, .1, 3.2},    {3, -25, "standard", 2.5, 3.0, 3.0},
        {16, -22, "standard", 1.6, 3.0, 3.0}, {25, -13, "standard", 1.6, 3.0, 3.0},
        {31, -3, "standard", 1.6, 3.0, 3.0},  {33, 8, "standard", 1.6, 3.0, 3.0},
        {24, 17, "standard", 1.6, 3.0, 3.0},  {12, 21, "pillar", 1.6, .1, 3.2},
        {2, 16, "standard", 3.5, 3.0, 3.0},   {6, 6, "standard", 2.5, 3.0, 3.0},
        {-1, 0, "standard", 1.6, 3.0, 3.0},   {-10, 7, "standard", 1.6, 3.0, 3.0},
        {-13, 19, "standard", 1.6, 3.0, 3.0}, {-22, 22, "pillar", 1.6, .1, 3.2}};
    constexpr qsizetype count = sizeof(layout) / sizeof(layout[0]);
    for (qsizetype i = 0; i < count; ++i) {
        Gate gate =
            makeGate(QString::fromLatin1(layout[i].type), layout[i].width, layout[i].height);
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

GateFrameMetrics gateFrameMetrics(const Gate &gate) {
    GateFrameMetrics metrics;
    metrics.centerWidth = gate.width;
    metrics.centerHeight = gate.height;
    if (gate.type == QStringLiteral("pillar")) {
        metrics.beamThickness = gate.width;
        metrics.horizontalBeamLength = metrics.outerWidth = gate.width;
        metrics.verticalBeamLength = metrics.outerHeight = gate.height;
        metrics.outerDepth = gate.width;
        return metrics;
    }
    metrics.centerlineModel = gate.extra.value(QStringLiteral("frameGeometry")).toString() ==
                              QStringLiteral("centerline-v1");
    if (metrics.centerlineModel) {
        if (gate.type == QStringLiteral("double"))
            metrics.columns = 2;
        else if (gate.type == QStringLiteral("triple"))
            metrics.columns = 3;
        if (gate.type == QStringLiteral("ladder") || gate.type == QStringLiteral("cube-stack"))
            metrics.levels = 2;
        if (gate.type == QStringLiteral("cube") || gate.type == QStringLiteral("cube-stack"))
            metrics.centerDepth = gate.width;
        const auto setting = [&gate](const QString &field, double fallback, double minimum) {
            const auto value = gate.extra.value(field);
            const double number = value.toDouble(fallback);
            return std::isfinite(number) && number >= minimum && number <= 10000.0 ? number
                                                                                   : fallback;
        };
        metrics.beamThickness = setting(QStringLiteral("beamThickness"), .1, .001);
        const double overhang = setting(QStringLiteral("beamOverhang"), .2, 0.0);
        metrics.horizontalBeamLength = gate.width + overhang;
        metrics.verticalBeamLength = gate.height + overhang;
    } else {
        // Keep existing project/import geometry exactly as the earlier renderer used it.
        metrics.beamThickness = std::clamp(std::min(gate.width, gate.height) * .041, .055, .16);
        metrics.horizontalBeamLength = gate.width;
        metrics.verticalBeamLength = gate.height + metrics.beamThickness;
    }
    metrics.openingWidth = std::max(0.0, gate.width - metrics.beamThickness);
    metrics.openingHeight = std::max(0.0, gate.height - metrics.beamThickness);
    metrics.outerWidth = (metrics.columns - 1) * gate.width +
                         std::max(metrics.horizontalBeamLength, gate.width + metrics.beamThickness);
    metrics.outerHeight = (metrics.levels - 1) * gate.height +
                          std::max(metrics.verticalBeamLength, gate.height + metrics.beamThickness);
    metrics.outerDepth =
        metrics.centerDepth > 0
            ? std::max(metrics.horizontalBeamLength, metrics.centerDepth + metrics.beamThickness)
            : metrics.beamThickness;
    return metrics;
}

QVector<GateBeam> gateFrameBeams(const Gate &gate) {
    if (gate.type == QStringLiteral("pillar"))
        return {};
    const auto metrics = gateFrameMetrics(gate);
    const float halfWidth = static_cast<float>(metrics.centerWidth / 2.0);
    const float halfHeight = static_cast<float>(metrics.centerHeight / 2.0);
    const float thickness = static_cast<float>(metrics.beamThickness);
    const float horizontal = static_cast<float>(metrics.horizontalBeamLength);
    const float vertical = static_cast<float>(metrics.verticalBeamLength);
    if (metrics.centerlineModel) {
        QVector<GateBeam> beams;
        const float width = static_cast<float>(metrics.centerWidth);
        const float height = static_cast<float>(metrics.centerHeight);
        const float left = -width * metrics.columns / 2;
        const float bottom = -height * metrics.levels / 2;
        const bool cube = metrics.centerDepth > 0;
        const QVector<float> faces =
            cube ? QVector<float>{-halfWidth, halfWidth} : QVector<float>{0};
        // Vertical stocks are separate per cell level, including shared joints.
        for (const float z : faces)
            for (int column = 0; column <= metrics.columns; ++column)
                for (int level = 0; level < metrics.levels; ++level)
                    beams.append({{left + column * width, bottom + (level + .5f) * height, z},
                                  {thickness, vertical, thickness}});
        // Descending boundaries preserve the historical four-beam order for a single frame.
        for (const float z : faces)
            for (int boundary = metrics.levels; boundary >= 0; --boundary)
                for (int column = 0; column < metrics.columns; ++column)
                    beams.append({{left + (column + .5f) * width, bottom + boundary * height, z},
                                  {horizontal, thickness, thickness}});
        if (cube)
            for (int boundary = metrics.levels; boundary >= 0; --boundary)
                for (const float x : {-halfWidth, halfWidth})
                    beams.append(
                        {{x, bottom + boundary * height, 0}, {thickness, thickness, horizontal}});
        return beams;
    }
    return {{{-halfWidth, 0, 0}, {thickness, vertical, thickness}},
            {{halfWidth, 0, 0}, {thickness, vertical, thickness}},
            {{0, halfHeight, 0}, {horizontal, thickness, thickness}},
            {{0, -halfHeight, 0}, {horizontal, thickness, thickness}}};
}

Gate makeGate(const QString &type, double width, double height) {
    Gate gate;
    gate.id = newGateId();
    gate.type = type;
    const bool pillar = type == QStringLiteral("pillar");
    gate.width = std::isfinite(width) && width > 0 ? width : (pillar ? .1 : 3.0);
    gate.height = std::isfinite(height) && height > 0 ? height : (pillar ? 3.2 : 3.0);
    if (!pillar && type != QStringLiteral("flag")) {
        gate.extra.insert(QStringLiteral("frameGeometry"), QStringLiteral("centerline-v1"));
        gate.extra.insert(QStringLiteral("beamThickness"), .1);
        gate.extra.insert(QStringLiteral("beamOverhang"), .2);
    }
    gate.elevation = gateFrameMetrics(gate).outerHeight / 2.0;
    setGateColor(gate, QColor(QStringLiteral("#0d0d0d")));
    return gate;
}

QPointF gatePassagePosition(const Gate &gate) {
    const double yaw = gate.yaw * 3.14159265358979323846 / 180.0;
    double localX = 0, localZ = 0;
    if (gate.type == QStringLiteral("pillar")) {
        const double side = gate.extra.value(QStringLiteral("passSide")).toInt(1) < 0 ? -1.0 : 1.0;
        localX = (gate.width / 2.0 + 1.55) * side;
    } else {
        const auto metrics = gateFrameMetrics(gate);
        if (!metrics.centerlineModel)
            return gate.position;
        const int column = std::clamp(gate.extra.value(QStringLiteral("passColumn")).toInt(0), 0,
                                      metrics.columns - 1);
        localX = (column - (metrics.columns - 1) / 2.0) * gate.width;
        localZ = -metrics.centerDepth / 2;
    }
    return gate.position + QPointF(std::cos(yaw) * localX + std::sin(yaw) * localZ,
                                   -std::sin(yaw) * localX + std::cos(yaw) * localZ);
}

double gatePassageElevation(const Gate &gate) {
    const auto metrics = gateFrameMetrics(gate);
    if (!metrics.centerlineModel)
        return gate.elevation;
    const int level =
        std::clamp(gate.extra.value(QStringLiteral("passLevel")).toInt(0), 0, metrics.levels - 1);
    return gate.elevation + (level - (metrics.levels - 1) / 2.0) * gate.height;
}

namespace {
QVector3D transformedCurveHint(const Gate *gate, const RouteStep &step, const QString &field) {
    const auto raw = step.extra.value(field);
    if (!raw.isArray() || raw.toArray().size() != 3)
        return {};
    QVector3D tangent;
    const auto array = raw.toArray();
    for (int axis = 0; axis < 3; ++axis) {
        const auto value = array[axis];
        if (!value.isDouble() || !std::isfinite(value.toDouble()) ||
            std::abs(value.toDouble()) > 1e6)
            return {};
        tangent[axis] = static_cast<float>(value.toDouble());
    }
    if (gate) {
        const auto metrics = gateFrameMetrics(*gate);
        tangent *= QVector3D(static_cast<float>(metrics.columns * gate->width / 2),
                             static_cast<float>(metrics.levels * gate->height / 2),
                             static_cast<float>(gate->width / 2));
        const auto rotation = QQuaternion::fromAxisAndAngle(0, 1, 0, static_cast<float>(gate->yaw));
        tangent = rotation.rotatedVector(tangent);
    }
    return tangent.lengthSquared() > 1e-12f ? tangent.normalized() : QVector3D();
}

QVector3D curveTangentForStep(const Gate *gate, const RouteStep &step) {
    return transformedCurveHint(gate, step, QStringLiteral("curveTangent"));
}

RouteStepGeometry geometryForStep(const Gate *gate, const RouteStep &step) {
    RouteStepGeometry geometry;
    if (step.kind == QStringLiteral("guide")) {
        if (!gate) {
            if (!step.gateId.isEmpty())
                return geometry;
            geometry.position = step.localGuide;
        } else {
            const auto metrics = gateFrameMetrics(*gate);
            const QVector3D scale(static_cast<float>(metrics.columns * gate->width / 2),
                                  static_cast<float>(metrics.levels * gate->height / 2),
                                  static_cast<float>(gate->width / 2));
            const auto rotation =
                QQuaternion::fromAxisAndAngle(0, 1, 0, static_cast<float>(gate->yaw));
            geometry.position = QVector3D(static_cast<float>(gate->position.x()),
                                          static_cast<float>(gate->elevation),
                                          static_cast<float>(gate->position.y())) +
                                rotation.rotatedVector(step.localGuide * scale);
        }
        geometry.valid = std::isfinite(geometry.position.x()) &&
                         std::isfinite(geometry.position.y()) &&
                         std::isfinite(geometry.position.z());
        if (step.extra.value(QStringLiteral("raceCheckpoint")).toBool(false)) {
            geometry.normal = curveTangentForStep(gate, step);
            auto reference =
                std::abs(geometry.normal.y()) > .9f ? QVector3D(0, 0, 1) : QVector3D(0, 1, 0);
            if (step.extra.contains(QStringLiteral("checkpointUp")))
                reference = transformedCurveHint(gate, step, QStringLiteral("checkpointUp"));
            geometry.right = QVector3D::crossProduct(reference, geometry.normal).normalized();
            geometry.up = QVector3D::crossProduct(geometry.normal, geometry.right).normalized();
            geometry.openingWidth =
                step.extra.value(QStringLiteral("checkpointWidth")).toDouble(.8);
            geometry.openingHeight =
                step.extra.value(QStringLiteral("checkpointHeight")).toDouble(.8);
            geometry.valid =
                geometry.valid && geometry.normal.lengthSquared() > .9f &&
                geometry.right.lengthSquared() > .9f && geometry.up.lengthSquared() > .9f &&
                std::isfinite(geometry.openingWidth) && std::isfinite(geometry.openingHeight) &&
                geometry.openingWidth >= .1 && geometry.openingWidth <= 100 &&
                geometry.openingHeight >= .1 && geometry.openingHeight <= 100;
        }
        geometry.pathPosition = geometry.position;
        geometry.pathTangent = step.extra.value(QStringLiteral("raceCheckpoint")).toBool(false)
                                   ? geometry.normal
                                   : curveTangentForStep(gate, step);
        return geometry;
    }
    if (step.kind != QStringLiteral("pass") || !gate)
        return geometry;
    const auto metrics = gateFrameMetrics(*gate);
    if (step.column < 0 || step.column >= metrics.columns || step.level < 0 ||
        step.level >= metrics.levels)
        return geometry;
    const bool cube = metrics.centerDepth > 0;
    const float x = static_cast<float>((step.column - (metrics.columns - 1) / 2.0) * gate->width);
    const float y = static_cast<float>((step.level - (metrics.levels - 1) / 2.0) * gate->height);
    const float halfDepth = static_cast<float>(metrics.centerDepth / 2);
    QVector3D local(x, y, 0), normal, right, up(0, 1, 0);
    geometry.openingWidth = gate->type == QStringLiteral("pillar") ? 2.4 : metrics.openingWidth;
    geometry.openingHeight =
        gate->type == QStringLiteral("pillar") ? gate->height : metrics.openingHeight;
    if (step.face == QStringLiteral("front")) {
        local.setZ(-halfDepth);
        normal = {0, 0, 1};
        right = {1, 0, 0};
    } else if (step.face == QStringLiteral("back")) {
        local.setZ(halfDepth);
        normal = {0, 0, -1};
        right = {-1, 0, 0};
    } else if (!cube) {
        return {};
    } else if (step.face == QStringLiteral("left")) {
        local = {-static_cast<float>(gate->width / 2), y, 0};
        normal = {1, 0, 0};
        right = {0, 0, -1};
    } else if (step.face == QStringLiteral("right")) {
        local = {static_cast<float>(gate->width / 2), y, 0};
        normal = {-1, 0, 0};
        right = {0, 0, 1};
    } else if (step.face == QStringLiteral("top") || step.face == QStringLiteral("bottom")) {
        const bool top = step.face == QStringLiteral("top");
        local = {x, y + static_cast<float>((top ? 1 : -1) * gate->height / 2), 0};
        normal = {0, top ? -1.f : 1.f, 0};
        right = {1, 0, 0};
        up = {0, 0, top ? 1.f : -1.f};
        geometry.openingHeight = metrics.openingWidth;
    } else {
        return {};
    }
    if (gate->type == QStringLiteral("pillar")) {
        const float side = gate->extra.value(QStringLiteral("passSide")).toInt(1) < 0 ? -1.f : 1.f;
        local.setX(static_cast<float>(gate->width / 2 + 1.55) * side);
    }
    if (geometry.openingWidth <= 0 || geometry.openingHeight <= 0)
        return {};
    QVector3D pathLocal = local;
    for (const auto &field : {QStringLiteral("apertureU"), QStringLiteral("apertureV")}) {
        const auto value = step.extra.value(field);
        if (value.isUndefined())
            continue;
        const bool horizontal = field == QStringLiteral("apertureU");
        const double half = (horizontal ? geometry.openingWidth : geometry.openingHeight) / 2;
        const double maximum = half > 0 ? std::max(0.0, 1 - (kRouteClearance + .02) / half) : 0;
        if (!value.isDouble() || !std::isfinite(value.toDouble()) ||
            std::abs(value.toDouble()) > maximum + 1e-7)
            return {};
        pathLocal += (horizontal ? right : up) * static_cast<float>(value.toDouble() * half);
    }
    auto rotation = QQuaternion::fromAxisAndAngle(0, 1, 0, static_cast<float>(gate->yaw));
    if (gate->type == QStringLiteral("dive"))
        rotation *= QQuaternion::fromAxisAndAngle(1, 0, 0, 90);
    if (step.reverse) {
        normal = -normal;
        right = -right;
    }
    geometry.position =
        QVector3D(static_cast<float>(gate->position.x()), static_cast<float>(gate->elevation),
                  static_cast<float>(gate->position.y())) +
        rotation.rotatedVector(local);
    geometry.normal = rotation.rotatedVector(normal).normalized();
    geometry.right = rotation.rotatedVector(right).normalized();
    geometry.up = rotation.rotatedVector(up).normalized();
    geometry.pathPosition = geometry.position + rotation.rotatedVector(pathLocal - local);
    geometry.pathTangent = geometry.normal;
    if (step.extra.contains(QStringLiteral("flightTangent"))) {
        geometry.pathTangent = transformedCurveHint(gate, step, QStringLiteral("flightTangent"));
        if (geometry.pathTangent.lengthSquared() < .9f ||
            QVector3D::dotProduct(geometry.pathTangent, geometry.normal) <= .01f)
            return {};
    }
    geometry.valid = true;
    return geometry;
}
} // namespace

RouteStepGeometry routeStepGeometry(const TrackDocument &document, const RouteStep &step) {
    const Gate *gate = nullptr;
    for (const auto &candidate : document.gates)
        if (candidate.id == step.gateId) {
            gate = &candidate;
            break;
        }
    return geometryForStep(gate, step);
}

QVector3D routeStepPosition(const TrackDocument &document, const RouteStep &step) {
    return routeStepGeometry(document, step).position;
}

QVector3D routeStepNormal(const TrackDocument &document, const RouteStep &step) {
    return routeStepGeometry(document, step).normal;
}

QVector<RouteStep> effectiveRouteSteps(const TrackDocument &document) {
    if (document.routeStepsDefined || !document.routeSteps.isEmpty())
        return document.routeSteps;
    QVector<RouteStep> steps;
    steps.reserve(document.gates.size());
    for (const auto &gate : document.gates) {
        RouteStep step;
        step.id = gate.id;
        step.gateId = gate.id;
        step.column = gate.extra.value(QStringLiteral("passColumn")).toInt(0);
        step.level = gate.extra.value(QStringLiteral("passLevel")).toInt(0);
        steps.append(step);
    }
    return steps;
}

QVector<RouteStep> gateManeuverSteps(const Gate &gate, const QString &pattern) {
    QVector<RouteStep> steps;
    const auto metrics = gateFrameMetrics(gate);
    const QVector3D scale(static_cast<float>(metrics.columns * gate.width / 2),
                          static_cast<float>(metrics.levels * gate.height / 2),
                          static_cast<float>(gate.width / 2));
    const auto addPass = [&steps, &gate](int column, int level, const QString &face,
                                         bool reverse = false) {
        RouteStep step;
        step.id = newGateId();
        step.gateId = gate.id;
        step.column = column;
        step.level = level;
        step.face = face;
        step.reverse = reverse;
        step.extra.insert(QStringLiteral("curveMode"), QStringLiteral("arc-v1"));
        steps.append(step);
    };
    const auto addGuide = [&steps, &gate, &scale](const QVector3D &local, const QVector3D &tangent,
                                                  bool raceCheckpoint = false) {
        RouteStep step;
        step.id = newGateId();
        step.gateId = gate.id;
        step.kind = QStringLiteral("guide");
        step.localGuide = local / scale;
        const auto normalizedTangent = tangent / scale;
        step.extra.insert(QStringLiteral("curveMode"), QStringLiteral("arc-v1"));
        step.extra.insert(
            QStringLiteral("curveTangent"),
            QJsonArray{normalizedTangent.x(), normalizedTangent.y(), normalizedTangent.z()});
        if (raceCheckpoint) {
            step.extra.insert(QStringLiteral("raceCheckpoint"), true);
            step.extra.insert(QStringLiteral("checkpointWidth"), .8);
            step.extra.insert(QStringLiteral("checkpointHeight"), .8);
        }
        steps.append(step);
    };
    if (pattern == QStringLiteral("single")) {
        addPass(std::clamp(gate.extra.value(QStringLiteral("passColumn")).toInt(0), 0,
                           metrics.columns - 1),
                std::clamp(gate.extra.value(QStringLiteral("passLevel")).toInt(0), 0,
                           metrics.levels - 1),
                QStringLiteral("front"));
    } else if ((pattern == QStringLiteral("upright-left-up") ||
                pattern == QStringLiteral("upright-left-down") ||
                pattern == QStringLiteral("upright-right-up") ||
                pattern == QStringLiteral("upright-right-down")) &&
               metrics.centerlineModel && metrics.levels > 1 && metrics.centerDepth == 0 &&
               metrics.openingWidth > .5 && metrics.openingHeight > .5) {
        const bool left = pattern.startsWith(QStringLiteral("upright-left"));
        const bool upward = pattern.endsWith(QStringLiteral("up"));
        const int column = std::clamp(gate.extra.value(QStringLiteral("passColumn")).toInt(0), 0,
                                      metrics.columns - 1);
        const int fromLevel = upward ? 0 : metrics.levels - 1;
        const int toLevel = upward ? metrics.levels - 1 : 0;
        const double centerX = (column - (metrics.columns - 1) / 2.0) * gate.width;
        const double postX = centerX + (left ? -1 : 1) * gate.width / 2;
        const double startY = (fromLevel - (metrics.levels - 1) / 2.0) * gate.height;
        const double advance = (toLevel - fromLevel) * gate.height;
        const double radius = gate.width / 2;
        const double phase = left ? 0 : 3.14159265358979323846;
        const double sweep = (left ? 1 : -1) * 2 * 3.14159265358979323846;
        constexpr int intervals = 16;
        for (int i = 0; i <= intervals; ++i) {
            const double t = double(i) / intervals, angle = phase + sweep * t;
            const QVector3D tangent(static_cast<float>(-radius * std::sin(angle) * sweep),
                                    static_cast<float>(advance),
                                    static_cast<float>(radius * std::cos(angle) * sweep));
            if (i == 0 || i == intervals) {
                addPass(column, i == 0 ? fromLevel : toLevel, QStringLiteral("front"));
                const auto normalized = tangent / scale;
                steps.last().extra.insert(
                    QStringLiteral("flightTangent"),
                    QJsonArray{normalized.x(), normalized.y(), normalized.z()});
            } else
                addGuide({static_cast<float>(postX + radius * std::cos(angle)),
                          static_cast<float>(startY + advance * t),
                          static_cast<float>(radius * std::sin(angle))},
                         tangent, i == intervals / 2);
            steps.last().extra.insert(
                QStringLiteral("maneuver"),
                QJsonObject{{QStringLiteral("type"), QStringLiteral("upright-progressive")},
                            {QStringLiteral("axis"), QStringLiteral("y")},
                            {QStringLiteral("side"),
                             left ? QStringLiteral("left") : QStringLiteral("right")},
                            {QStringLiteral("sweepDegrees"), (left ? 1 : -1) * 360},
                            {QStringLiteral("axialAdvance"), advance},
                            {QStringLiteral("continuousAdvance"), true},
                            {QStringLiteral("returnsToSameAperture"), false}});
        }
    } else if ((pattern == QStringLiteral("orbit-bar-cw") ||
                pattern == QStringLiteral("orbit-bar-ccw")) &&
               metrics.centerlineModel && metrics.centerDepth == 0 && metrics.openingWidth > .5 &&
               metrics.openingHeight > .5) {
        const bool ladder = metrics.levels > 1;
        const int winding = pattern == QStringLiteral("orbit-bar-ccw") ? 1 : -1;
        const int column = std::clamp(gate.extra.value(QStringLiteral("passColumn")).toInt(0), 0,
                                      metrics.columns - 1);
        const float x = static_cast<float>((column - (metrics.columns - 1) / 2.0) * gate.width);
        const float centerY = ladder ? 0.f : static_cast<float>(gate.height / 2);
        const float radius = static_cast<float>(gate.height / 2);
        for (int i = 0; i <= 8; ++i) {
            const double angle = i * 3.14159265358979323846 / 4;
            if (i == 0 || i == 8)
                addPass(column, 0, winding > 0 ? QStringLiteral("front") : QStringLiteral("back"));
            else if (ladder && i == 4)
                addPass(column, 1, winding > 0 ? QStringLiteral("back") : QStringLiteral("front"));
            else
                addGuide({x, centerY - radius * static_cast<float>(std::cos(angle)),
                          winding * radius * static_cast<float>(std::sin(angle))},
                         {0, radius * static_cast<float>(std::sin(angle)),
                          winding * radius * static_cast<float>(std::cos(angle))},
                         i == 2 || i == 6 || (!ladder && i == 4));
            steps.last().extra.insert(
                QStringLiteral("maneuver"),
                QJsonObject{
                    {QStringLiteral("type"), QStringLiteral("bar-orbit")},
                    {QStringLiteral("axis"), QStringLiteral("x")},
                    {QStringLiteral("winding"), winding},
                    {QStringLiteral("sweepDegrees"), 360},
                    {QStringLiteral("view"), QStringLiteral("along-local+X; +Z right; +Y up")}});
        }
    } else if ((pattern == QStringLiteral("lower-upper") ||
                pattern == QStringLiteral("upper-lower")) &&
               metrics.centerlineModel && metrics.levels > 1 && metrics.centerDepth == 0) {
        const bool lowerFirst = pattern == QStringLiteral("lower-upper");
        addPass(0, lowerFirst ? 0 : metrics.levels - 1, QStringLiteral("front"));
        const float radius = static_cast<float>(gate.height / 2);
        for (int i = 1; i <= 3; ++i) {
            const double angle = i * 3.14159265358979323846 / 4;
            addGuide({0, (lowerFirst ? -radius : radius) * static_cast<float>(std::cos(angle)),
                      radius * static_cast<float>(std::sin(angle))},
                     {0, (lowerFirst ? radius : -radius) * static_cast<float>(std::sin(angle)),
                      radius * static_cast<float>(std::cos(angle))});
        }
        addPass(0, lowerFirst ? metrics.levels - 1 : 0, QStringLiteral("back"));
    } else if ((pattern == QStringLiteral("left-right") ||
                pattern == QStringLiteral("right-left")) &&
               metrics.centerlineModel && metrics.columns > 1) {
        const bool leftFirst = pattern == QStringLiteral("left-right");
        for (int i = 0; i < metrics.columns; ++i) {
            const int column = leftFirst ? i : metrics.columns - i - 1;
            addPass(column, 0, i % 2 ? QStringLiteral("back") : QStringLiteral("front"));
            if (i + 1 < metrics.columns) {
                const float centerX = static_cast<float>(
                    (column + (leftFirst ? .5 : -.5) - (metrics.columns - 1) / 2.0) * gate.width);
                const float radius = static_cast<float>(gate.width / 2);
                const float direction = leftFirst ? 1.f : -1.f;
                const float side = i % 2 ? -1.f : 1.f;
                for (int j = 1; j <= 3; ++j) {
                    const double angle = j * 3.14159265358979323846 / 4;
                    addGuide({centerX - direction * radius * static_cast<float>(std::cos(angle)), 0,
                              side * radius * static_cast<float>(std::sin(angle))},
                             {direction * radius * static_cast<float>(std::sin(angle)), 0,
                              side * radius * static_cast<float>(std::cos(angle))});
                }
            }
        }
    } else if ((pattern == QStringLiteral("gravity-dive") ||
                pattern == QStringLiteral("gravity-climb")) &&
               metrics.centerDepth > 0) {
        if (pattern == QStringLiteral("gravity-dive")) {
            for (int level = metrics.levels - 1; level >= 0; --level)
                addPass(0, level, QStringLiteral("top"));
            const float halfH = static_cast<float>(gate.height / 2),
                        halfW = static_cast<float>(gate.width / 2);
            const float top = static_cast<float>((1 - metrics.levels / 2.0) * gate.height);
            const float rootHalf = std::sqrt(.5f);
            addGuide({0, top - halfH * rootHalf, -halfW + halfW * rootHalf},
                     {0, -halfH * rootHalf, -halfW * rootHalf});
            addPass(0, 0, QStringLiteral("front"), true);
        } else {
            addPass(0, 0, QStringLiteral("front"));
            const float halfH = static_cast<float>(gate.height / 2),
                        halfW = static_cast<float>(gate.width / 2);
            const float top = static_cast<float>((1 - metrics.levels / 2.0) * gate.height);
            const float rootHalf = std::sqrt(.5f);
            addGuide({0, top - halfH * rootHalf, -halfW + halfW * rootHalf},
                     {0, halfH * rootHalf, halfW * rootHalf});
            for (int level = 0; level < metrics.levels; ++level)
                addPass(0, level, QStringLiteral("top"), true);
        }
    }
    return steps;
}

namespace {
bool segmentBox(const QVector3D &a, const QVector3D &b, const QVector3D &low,
                const QVector3D &high) {
    float begin = 0, end = 1;
    for (int axis = 0; axis < 3; ++axis) {
        const float delta = b[axis] - a[axis];
        if (std::abs(delta) < 1e-8f) {
            if (a[axis] < low[axis] || a[axis] > high[axis])
                return false;
        } else {
            const float t0 = (low[axis] - a[axis]) / delta;
            const float t1 = (high[axis] - a[axis]) / delta;
            begin = std::max(begin, std::min(t0, t1));
            end = std::min(end, std::max(t0, t1));
            if (begin > end)
                return false;
        }
    }
    return true;
}

struct CurveObstacle {
    QVector3D origin, low, high;
    QQuaternion rotation, inverse;
    QVector<GateBeam> beams;
};

struct CurveClearance {
    QVector<CurveObstacle> obstacles;
    explicit CurveClearance(const TrackDocument &document) {
        for (const auto &gate : document.gates) {
            CurveObstacle obstacle;
            obstacle.origin = {static_cast<float>(gate.position.x()),
                               static_cast<float>(gate.elevation),
                               static_cast<float>(gate.position.y())};
            obstacle.rotation =
                QQuaternion::fromAxisAndAngle(0, 1, 0, static_cast<float>(gate.yaw));
            if (gate.type == QStringLiteral("dive"))
                obstacle.rotation *= QQuaternion::fromAxisAndAngle(1, 0, 0, 90);
            obstacle.inverse = obstacle.rotation.conjugated();
            obstacle.beams = gateFrameBeams(gate);
            if (gate.type == QStringLiteral("pillar"))
                obstacle.beams = {{{0, 0, 0},
                                   {static_cast<float>(gate.width), static_cast<float>(gate.height),
                                    static_cast<float>(gate.width)}}};
            obstacle.low = {1e20f, 1e20f, 1e20f};
            obstacle.high = -obstacle.low;
            for (const auto &beam : obstacle.beams) {
                const auto half =
                    beam.size / 2 + QVector3D(kRouteClearance + .01f, kRouteClearance + .01f,
                                              kRouteClearance + .01f);
                for (int mask = 0; mask < 8; ++mask) {
                    const auto point =
                        obstacle.origin +
                        obstacle.rotation.rotatedVector(beam.center +
                                                        QVector3D(mask & 1 ? half.x() : -half.x(),
                                                                  mask & 2 ? half.y() : -half.y(),
                                                                  mask & 4 ? half.z() : -half.z()));
                    for (int axis = 0; axis < 3; ++axis) {
                        obstacle.low[axis] = std::min(obstacle.low[axis], point[axis]);
                        obstacle.high[axis] = std::max(obstacle.high[axis], point[axis]);
                    }
                }
            }
            obstacles.append(obstacle);
        }
    }
    bool segment(const QVector3D &a, const QVector3D &b,
                 QVector<QPair<int, int>> *hits = nullptr) const {
        if (!std::isfinite(a.x()) || !std::isfinite(a.y()) || !std::isfinite(a.z()) ||
            !std::isfinite(b.x()) || !std::isfinite(b.y()) || !std::isfinite(b.z()) ||
            std::min(a.y(), b.y()) < kRouteClearance - 1e-5f)
            return false;
        bool clear = true;
        for (qsizetype i = 0; i < obstacles.size(); ++i) {
            const auto &obstacle = obstacles[i];
            if (!segmentBox(a, b, obstacle.low, obstacle.high))
                continue;
            const auto localA = obstacle.inverse.rotatedVector(a - obstacle.origin);
            const auto localB = obstacle.inverse.rotatedVector(b - obstacle.origin);
            for (qsizetype j = 0; j < obstacle.beams.size(); ++j) {
                const auto &beam = obstacle.beams[j];
                const auto half =
                    beam.size / 2 + QVector3D(kRouteClearance + .01f, kRouteClearance + .01f,
                                              kRouteClearance + .01f);
                if (segmentBox(localA, localB, beam.center - half, beam.center + half)) {
                    if (!hits)
                        return false;
                    clear = false;
                    hits->append({static_cast<int>(i), static_cast<int>(j)});
                }
            }
        }
        return clear;
    }
};

QVector<QVector3D> sampleCurve(const QVector3D &a, const QVector3D &b, const QVector3D &tangentA,
                               const QVector3D &tangentB) {
    const auto c = a + tangentA / 3, d = b - tangentB / 3;
    const double length = (c - a).length() + (d - c).length() + (b - d).length();
    const int samples = std::clamp(static_cast<int>(std::min(length / .06, 1024.0)) + 1, 2, 1024);
    QVector<QVector3D> points;
    points.reserve(samples + 1);
    points.append(a);
    for (int i = 1; i < samples; ++i) {
        const float t = static_cast<float>(i) / samples, u = 1 - t;
        points.append(a * (u * u * u) + c * (3 * u * u * t) + d * (3 * u * t * t) +
                      b * (t * t * t));
    }
    points.append(b);
    return points;
}

bool clearPoints(const CurveClearance &clearance, const QVector<QVector3D> &points) {
    for (qsizetype i = 1; i < points.size(); ++i)
        if (!clearance.segment(points[i - 1], points[i]))
            return false;
    return true;
}

QVector<QVector3D> detour(const CurveClearance &clearance, const QVector3D &a, const QVector3D &b,
                          const QVector3D &directionA, const QVector3D &directionB,
                          float leadLimit) {
    if (!clearance.segment(a, a) || !clearance.segment(b, b))
        return {};
    const float lead = std::min(leadLimit, (b - a).length() * .2f);
    const auto start = a + directionA * lead, end = b - directionB * lead;
    if (!clearance.segment(a, start) || !clearance.segment(end, b))
        return {};
    QVector<QVector3D> nodes{start, end};
    QVector<QPair<int, int>> hits;
    clearance.segment(start, end, &hits);
    const float maximum = std::max(8.f, (end - start).length() * 3 + 3);
    for (const auto &hit : hits) {
        const auto &obstacle = clearance.obstacles[hit.first];
        const auto &beam = obstacle.beams[hit.second];
        const auto half = beam.size / 2 + QVector3D(kRouteClearance + .14f, kRouteClearance + .14f,
                                                    kRouteClearance + .14f);
        for (int candidate = 0; candidate < 14 && nodes.size() < 74; ++candidate) {
            QVector3D local;
            if (candidate < 8)
                local = {candidate & 1 ? half.x() : -half.x(), candidate & 2 ? half.y() : -half.y(),
                         candidate & 4 ? half.z() : -half.z()};
            else {
                const int axis = (candidate - 8) / 2;
                local[axis] = candidate % 2 ? half[axis] : -half[axis];
            }
            auto point = obstacle.origin + obstacle.rotation.rotatedVector(beam.center + local);
            point.setY(std::max(point.y(), kRouteClearance + .04f));
            if ((point - start).length() + (point - end).length() <= maximum &&
                clearance.segment(point, point))
                nodes.append(point);
        }
    }
    const int count = static_cast<int>(nodes.size());
    QVector<float> distances(count, std::numeric_limits<float>::infinity());
    QVector<int> previous(count, -1);
    QVector<bool> visited(count, false);
    QVector<qint8> visible(count * count, -1);
    distances[0] = 0;
    for (int iteration = 0; iteration < count; ++iteration) {
        int current = -1;
        float best = std::numeric_limits<float>::infinity();
        for (int i = 0; i < count; ++i)
            if (!visited[i] && distances[i] + (nodes[i] - end).length() < best) {
                best = distances[i] + (nodes[i] - end).length();
                current = i;
            }
        if (current < 0 || current == 1)
            break;
        visited[current] = true;
        for (int next = 0; next < count; ++next) {
            if (visited[next] || next == current)
                continue;
            auto &edge = visible[current * count + next];
            if (edge < 0) {
                edge = clearance.segment(nodes[current], nodes[next]) ? 1 : 0;
                visible[next * count + current] = edge;
            }
            const float distance = distances[current] + (nodes[current] - nodes[next]).length();
            if (edge == 1 && distance < distances[next]) {
                distances[next] = distance;
                previous[next] = current;
            }
        }
    }
    if (!std::isfinite(distances[1]))
        return {};
    QVector<QVector3D> path;
    for (int index = 1; index >= 0; index = previous[index]) {
        path.prepend(nodes[index]);
        if (index == 0)
            break;
    }
    path.prepend(a);
    path.append(b);
    return path;
}

// Planner vertices only establish a clear corridor; they are not route anchors.
// Round their corners inside that corridor, leaving every actual endpoint and
// its approach/departure direction intact. Each accepted cubic is checked as a
// sampled swept segment against the same inflated stock used by the main path.
QVector<QVector3D> roundedDetour(const CurveClearance &clearance,
                                 const QVector<QVector3D> &vertices) {
    if (vertices.size() < 3)
        return vertices;
    QVector<QVector3D> path;
    for (const auto &point : vertices) {
        if (!path.isEmpty() && (path.last() - point).lengthSquared() < 1e-10f)
            continue;
        while (path.size() > 1) {
            const auto incoming = (path.last() - path[path.size() - 2]).normalized();
            const auto outgoing = (point - path.last()).normalized();
            if (QVector3D::dotProduct(incoming, outgoing) < .99999f ||
                !clearance.segment(path[path.size() - 2], point))
                break;
            path.removeLast();
        }
        path.append(point);
    }
    if (path.size() < 3)
        return path;
    for (const float fraction : {.45f, .32f, .22f, .14f, .08f, .04f}) {
        QVector<QVector3D> rounded{path.first()};
        bool valid = true;
        for (qsizetype i = 1; i + 1 < path.size(); ++i) {
            const auto incoming = path[i] - path[i - 1], outgoing = path[i + 1] - path[i];
            const float before = incoming.length(), after = outgoing.length();
            if (std::min(before, after) < 1e-5f) {
                valid = false;
                break;
            }
            const auto in = incoming / before, out = outgoing / after;
            const double cosine = std::clamp(double(QVector3D::dotProduct(in, out)), -1.0, 1.0);
            // A reversal needs a different corridor, not a zero-radius fillet.
            if (cosine < -.996) {
                valid = false;
                break;
            }
            const double angle = std::acos(cosine);
            const float trim = std::min(2.f, std::min(before, after) * fraction);
            const auto start = path[i] - in * trim, end = path[i] + out * trim;
            const float handle = angle < 1e-4
                                     ? trim * 2
                                     : float(4 * trim * std::tan(angle / 4) / std::tan(angle / 2));
            const auto arc = sampleCurve(start, end, in * handle, out * handle);
            if (!clearance.segment(rounded.last(), start) || !clearPoints(clearance, arc)) {
                valid = false;
                break;
            }
            if ((rounded.last() - start).lengthSquared() > 1e-10f)
                rounded.append(start);
            for (qsizetype j = 1; j < arc.size(); ++j)
                rounded.append(arc[j]);
        }
        if (valid && clearance.segment(rounded.last(), path.last())) {
            rounded.append(path.last());
            return rounded;
        }
    }
    return {};
}

struct CurveNode {
    QVector3D point, direction;
    bool pass = false, arc = false;
    QString owner;
    bool inferredGuide = false;
};

RouteCurveResult buildConstrainedCurve(const TrackDocument &document) {
    RouteCurveResult result;
    const auto steps = effectiveRouteSteps(document);
    if (steps.size() > 10000 || document.gates.size() > 10000) {
        result.clearanceSatisfied = false;
        result.smooth = false;
        result.warnings << QStringLiteral("路线过大，已暂停高精度曲线预览；请分段检查。");
        return result;
    }
    const CurveClearance clearance(document);
    QHash<QString, const Gate *> gates;
    for (const auto &gate : document.gates)
        gates.insert(gate.id, &gate);
    QVector<CurveNode> nodes;
    for (const auto &step : steps) {
        const auto *gate = gates.value(step.gateId, nullptr);
        const auto geometry = geometryForStep(gate, step);
        if (!geometry.valid || !std::isfinite(geometry.pathPosition.x()) ||
            !std::isfinite(geometry.pathPosition.y()) ||
            !std::isfinite(geometry.pathPosition.z())) {
            result.clearanceSatisfied = false;
            result.warnings << QStringLiteral("路线含无效步骤，未跳过或虚构穿越。");
            return result;
        }
        CurveNode node;
        node.point = geometry.pathPosition;
        node.pass = step.kind == QStringLiteral("pass") ||
                    step.extra.value(QStringLiteral("raceCheckpoint")).toBool(false);
        node.direction = geometry.pathTangent;
        if (!node.pass && step.extra.contains(QStringLiteral("curveTangent")) &&
            node.direction.lengthSquared() < 1e-8f)
            result.warnings << QStringLiteral("无效曲线切向提示已忽略，预览按安全连接重新计算。");
        node.arc =
            step.extra.value(QStringLiteral("curveMode")).toString() == QStringLiteral("arc-v1");
        node.owner = step.gateId;
        node.inferredGuide =
            !node.pass && node.arc && step.extra.value(QStringLiteral("recognition")).isObject();
        nodes.append(node);
    }
    if (nodes.isEmpty())
        return result;
    const bool alreadyClosed =
        nodes.size() > 2 && (nodes.first().point - nodes.last().point).lengthSquared() < 1e-8f &&
        QVector3D::dotProduct(nodes.first().direction, nodes.last().direction) > .99f;
    if (document.closedLoop && nodes.size() > 1 && !alreadyClosed)
        nodes.append(nodes.first());
    const bool closed = alreadyClosed || (document.closedLoop && nodes.size() > 1);
    if (!closed) {
        if (nodes.first().pass)
            nodes.prepend({nodes.first().point - nodes.first().direction * .6f,
                           nodes.first().direction, false, false});
        if (nodes.last().pass)
            nodes.append({nodes.last().point + nodes.last().direction * .6f, nodes.last().direction,
                          false, false});
    }
    QVector<QVector3D> tangents;
    QVector<float> factors(nodes.size(), 1);
    for (qsizetype i = 0; i < nodes.size(); ++i) {
        const auto before =
            i > 0 ? nodes[i - 1].point : (closed ? nodes[nodes.size() - 2].point : nodes[i].point);
        const auto after =
            i + 1 < nodes.size() ? nodes[i + 1].point : (closed ? nodes[1].point : nodes[i].point);
        auto direction = nodes[i].direction;
        const bool haveBefore = i > 0 || closed, haveAfter = i + 1 < nodes.size() || closed;
        const auto previousOwner =
            i > 0 ? nodes[i - 1].owner : (closed ? nodes[nodes.size() - 2].owner : nodes[i].owner);
        const auto nextOwner =
            i + 1 < nodes.size() ? nodes[i + 1].owner : (closed ? nodes[1].owner : nodes[i].owner);
        if (nodes[i].inferredGuide && haveBefore && haveAfter &&
            (previousOwner != nodes[i].owner || nextOwner != nodes[i].owner)) {
            // A fitted glyph's end derivative only sees pixels inside that glyph.
            // At a component boundary it can point away from the next obstacle,
            // forcing a millimetre-scale reversal before a long connector. Guide
            // hints are advisory; actual portal/T tangents are never changed here.
            const auto incoming = (nodes[i].point - before).normalized();
            const auto outgoing = (after - nodes[i].point).normalized();
            auto joined = incoming + outgoing;
            for (int axis = 0; axis < 3; ++axis)
                if (incoming[axis] * outgoing[axis] <= 0)
                    joined[axis] = 0;
            if (joined.lengthSquared() > 1e-8f)
                direction = joined.normalized();
        }
        if (direction.lengthSquared() < 1e-8f)
            direction = (after - before).normalized();
        float length =
            std::min((nodes[i].point - before).length(), (after - nodes[i].point).length());
        if (length < 1e-5f)
            length =
                std::max((nodes[i].point - before).length(), (after - nodes[i].point).length());
        tangents.append(direction * length * (nodes[i].arc ? 1.0395661f : .7f));
    }
    // Shared node factors retain a common tangent on both sides when handles are shortened.
    for (int iteration = 0; iteration < 5; ++iteration) {
        bool changed = false;
        for (qsizetype i = 1; i < nodes.size(); ++i) {
            const auto samples =
                sampleCurve(nodes[i - 1].point, nodes[i].point, tangents[i - 1] * factors[i - 1],
                            tangents[i] * factors[i]);
            if (!clearPoints(clearance, samples) && iteration < 4) {
                factors[i - 1] *= .5f;
                factors[i] *= .5f;
                changed = true;
            }
        }
        if (closed) {
            const float factor = std::min(factors.first(), factors.last());
            factors.first() = factors.last() = factor;
        }
        if (!changed)
            break;
    }
    const auto append = [&result](const QVector3D &point) {
        if (result.points.isEmpty() || (result.points.last() - point).lengthSquared() > 1e-10f)
            result.points.append(point);
    };
    int planned = 0;
    for (qsizetype i = 1; i < nodes.size(); ++i) {
        auto samples = sampleCurve(nodes[i - 1].point, nodes[i].point,
                                   tangents[i - 1] * factors[i - 1], tangents[i] * factors[i]);
        if (!clearPoints(clearance, samples)) {
            samples.clear();
            QVector<QVector3D> fallback;
            if (planned++ < 24) {
                const auto chord = (nodes[i].point - nodes[i - 1].point).normalized();
                const auto directionA =
                    tangents[i - 1].lengthSquared() > 1e-8f ? tangents[i - 1].normalized() : chord;
                const auto directionB =
                    tangents[i].lengthSquared() > 1e-8f ? tangents[i].normalized() : chord;
                const auto reference = detour(clearance, nodes[i - 1].point, nodes[i].point,
                                              directionA, directionB, .35f);
                float minimumY = std::min(nodes[i - 1].point.y(), nodes[i].point.y());
                float maximumY = std::max(nodes[i - 1].point.y(), nodes[i].point.y());
                for (const auto &point : reference) {
                    minimumY = std::min(minimumY, point.y());
                    maximumY = std::max(maximumY, point.y());
                }
                for (const float lead : {1.5f, .8f, .35f}) {
                    const auto corridor = detour(clearance, nodes[i - 1].point, nodes[i].point,
                                                 directionA, directionB, lead);
                    if (corridor.isEmpty())
                        continue;
                    fallback = corridor;
                    samples = roundedDetour(clearance, corridor);
                    // A gentler heading change must not invent a large climb or
                    // dive. Keep it within one preview radius of the original
                    // safe connector's vertical envelope.
                    if (std::any_of(samples.begin(), samples.end(), [&](const QVector3D &point) {
                            return point.y() < minimumY - kRouteClearance ||
                                   point.y() > maximumY + kRouteClearance;
                        }))
                        samples.clear();
                    if (!samples.isEmpty())
                        break;
                }
            }
            if (samples.isEmpty())
                samples = fallback;
            else
                fallback.clear();
            if (!fallback.isEmpty()) {
                result.smooth = false;
                result.warnings << QStringLiteral(
                    "部分连接采用保守净空绕行，局部转角未保证连续切向。");
            }
        }
        if (samples.isEmpty()) {
            result.clearanceSatisfied = false;
            result.smooth = false;
            result.warnings << QStringLiteral(
                "部分窗口或连接无法满足 0.20 m 预览净空，已断开显示，请调整高度或引导点。");
            if (!result.points.isEmpty() &&
                (result.breaks.isEmpty() || result.breaks.last() != result.points.size()))
                result.breaks.append(static_cast<int>(result.points.size()));
            continue;
        }
        if (!result.points.isEmpty() &&
            (result.points.last() - samples.first()).lengthSquared() > 1e-8f &&
            (result.breaks.isEmpty() || result.breaks.last() != result.points.size()))
            result.breaks.append(static_cast<int>(result.points.size()));
        for (const auto &point : samples)
            append(point);
        if (result.points.size() > 200000) {
            result.clearanceSatisfied = false;
            result.warnings << QStringLiteral("路线采样达到上限，预览已保留安全前段。");
            break;
        }
    }
    if (nodes.size() == 1 && clearance.segment(nodes[0].point, nodes[0].point))
        append(nodes[0].point);
    while (!result.breaks.isEmpty() && result.breaks.last() >= result.points.size())
        result.breaks.removeLast();
    result.warnings.removeDuplicates();
    return result;
}

QByteArray curveKey(const TrackDocument &document) {
    QByteArray bytes;
    QDataStream data(&bytes, QIODevice::WriteOnly);
    data << document.closedLoop << document.routeStepsDefined << document.gates.size();
    for (const auto &gate : document.gates) {
        data << gate.id << gate.type << gate.position << gate.elevation << gate.width << gate.height
             << gate.yaw;
        for (const auto &field : {QStringLiteral("frameGeometry"), QStringLiteral("beamThickness"),
                                  QStringLiteral("beamOverhang"), QStringLiteral("passColumn"),
                                  QStringLiteral("passLevel"), QStringLiteral("passSide")})
            data << QJsonDocument(QJsonObject{{field, gate.extra.value(field)}})
                        .toJson(QJsonDocument::Compact);
    }
    data << document.routeSteps.size();
    for (const auto &step : document.routeSteps) {
        data << step.id << step.gateId << step.kind << step.face << step.column << step.level
             << step.reverse << step.localGuide;
        data << (step.extra.value(QStringLiteral("curveMode")).toString() ==
                 QStringLiteral("arc-v1"))
             << step.extra.value(QStringLiteral("recognition")).isObject();
        for (const auto &field : {QStringLiteral("curveTangent"), QStringLiteral("checkpointUp"),
                                  QStringLiteral("flightTangent")}) {
            const auto raw = step.extra.value(field);
            const auto array = raw.toArray();
            if (raw.isArray() && array.size() == 3)
                data << QJsonDocument(array).toJson(QJsonDocument::Compact);
            else
                data << QByteArray(raw.isUndefined() ? "absent" : "invalid");
        }
        data << step.extra.value(QStringLiteral("raceCheckpoint")).toBool(false)
             << step.extra.value(QStringLiteral("checkpointWidth")).toDouble(.8)
             << step.extra.value(QStringLiteral("checkpointHeight")).toDouble(.8);
        for (const auto &field : {QStringLiteral("apertureU"), QStringLiteral("apertureV")}) {
            const auto value = step.extra.value(field);
            data << static_cast<int>(value.type()) << value.toDouble(0);
        }
    }
    return QCryptographicHash::hash(bytes, QCryptographicHash::Sha256);
}
} // namespace

double routePreviewClearance() {
    return kRouteClearance;
}

RouteCurveResult routeCurve(const TrackDocument &document) {
    struct Cache {
        QByteArray key;
        RouteCurveResult value;
    };
    thread_local QVector<Cache> caches;
    const auto key = curveKey(document);
    for (qsizetype i = 0; i < caches.size(); ++i)
        if (caches[i].key == key) {
            const auto value = caches[i].value;
            if (i + 1 < caches.size())
                caches.move(i, caches.size() - 1);
            return value;
        }
    Cache entry;
    entry.key = key;
    entry.value = buildConstrainedCurve(document);
    if (caches.size() >= 6)
        caches.removeFirst();
    caches.append(entry);
    return entry.value;
}

QVector<QVector3D> routePolyline(const TrackDocument &document) {
    const auto result = routeCurve(document);
    return result.breaks.isEmpty() ? result.points : result.points.mid(0, result.breaks.first());
}

QPainterPath routePath(const TrackDocument &document) {
    QPainterPath path;
    if (document.routeStepsDefined || !document.routeSteps.isEmpty()) {
        const auto curve = routeCurve(document);
        const auto &points = curve.points;
        if (!points.isEmpty()) {
            path.moveTo(points.first().x(), points.first().z());
            for (qsizetype i = 1; i < points.size(); ++i) {
                if (curve.breaks.contains(static_cast<int>(i)))
                    path.moveTo(points[i].x(), points[i].z());
                else
                    path.lineTo(points[i].x(), points[i].z());
            }
        }
        return path;
    }
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
    if (document.routeStepsDefined || !document.routeSteps.isEmpty()) {
        const auto curve = routeCurve(document);
        const auto &points = curve.points;
        double length = 0;
        for (qsizetype i = 1; i < points.size(); ++i)
            if (!curve.breaks.contains(static_cast<int>(i)))
                length += (points[i] - points[i - 1]).length();
        return length;
    }
    // Qt integrates cubic segments adaptively; this is geometric path length,
    // not a claimed flight distance or physics simulation.
    return routePath(document).length();
}

} // namespace ot
