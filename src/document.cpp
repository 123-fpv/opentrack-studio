#include "document.h"

#include <QBuffer>
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

RouteStepGeometry geometryForStep(const Gate *gate, const RouteStep &step);

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
    const auto addPass = [&steps, &gate](int column, int level, const QString &face,
                                         bool reverse = false) {
        RouteStep step;
        step.id = newGateId();
        step.gateId = gate.id;
        step.column = column;
        step.level = level;
        step.face = face;
        step.reverse = reverse;
        steps.append(step);
    };
    const auto addGuide = [&steps, &gate](const QVector3D &local) {
        RouteStep step;
        step.id = newGateId();
        step.gateId = gate.id;
        step.kind = QStringLiteral("guide");
        step.localGuide = local;
        steps.append(step);
    };
    if (pattern == QStringLiteral("single")) {
        addPass(std::clamp(gate.extra.value(QStringLiteral("passColumn")).toInt(0), 0,
                           metrics.columns - 1),
                std::clamp(gate.extra.value(QStringLiteral("passLevel")).toInt(0), 0,
                           metrics.levels - 1),
                QStringLiteral("front"));
    } else if ((pattern == QStringLiteral("lower-upper") ||
                pattern == QStringLiteral("upper-lower")) &&
               metrics.centerlineModel && metrics.levels > 1 && metrics.centerDepth == 0) {
        const bool lowerFirst = pattern == QStringLiteral("lower-upper");
        addPass(0, lowerFirst ? 0 : metrics.levels - 1, QStringLiteral("front"));
        addGuide({0, 0, .8f});
        addPass(0, lowerFirst ? metrics.levels - 1 : 0, QStringLiteral("back"));
    } else if ((pattern == QStringLiteral("left-right") ||
                pattern == QStringLiteral("right-left")) &&
               metrics.centerlineModel && metrics.columns > 1) {
        const bool leftFirst = pattern == QStringLiteral("left-right");
        for (int i = 0; i < metrics.columns; ++i) {
            const int column = leftFirst ? i : metrics.columns - i - 1;
            if (i > 0) {
                const float x = static_cast<float>(
                    (column + (leftFirst ? -.5 : .5) - (metrics.columns - 1) / 2.0) * 2 /
                    metrics.columns);
                addGuide({x, 0, i % 2 ? .8f : -.8f});
            }
            addPass(column, 0, i % 2 ? QStringLiteral("back") : QStringLiteral("front"));
        }
    } else if ((pattern == QStringLiteral("gravity-dive") ||
                pattern == QStringLiteral("gravity-climb")) &&
               metrics.centerDepth > 0) {
        if (pattern == QStringLiteral("gravity-dive")) {
            for (int level = metrics.levels - 1; level >= 0; --level)
                addPass(0, level, QStringLiteral("top"));
            addPass(0, 0, QStringLiteral("front"), true);
        } else {
            addPass(0, 0, QStringLiteral("front"));
            for (int level = 0; level < metrics.levels; ++level)
                addPass(0, level, QStringLiteral("top"), true);
        }
    }
    return steps;
}

QVector<QVector3D> routePolyline(const TrackDocument &document) {
    QVector<QVector3D> points;
    const auto steps = effectiveRouteSteps(document);
    QHash<QString, const Gate *> gates;
    for (const auto &gate : document.gates)
        gates.insert(gate.id, &gate);
    const auto append = [&points](const QVector3D &point) {
        if (points.isEmpty() || (points.last() - point).lengthSquared() > 1e-10f)
            points.append(point);
    };
    const bool explicitRoute = document.routeStepsDefined || !document.routeSteps.isEmpty();
    for (const auto &step : steps) {
        const auto geometry = geometryForStep(gates.value(step.gateId, nullptr), step);
        if (!geometry.valid)
            continue;
        if (explicitRoute && step.kind == QStringLiteral("pass")) {
            const float lead = static_cast<float>(
                std::clamp(std::min(geometry.openingWidth, geometry.openingHeight) * .2, .15, .6));
            append(geometry.position - geometry.normal * lead);
            append(geometry.position);
            append(geometry.position + geometry.normal * lead);
        } else {
            append(geometry.position);
        }
    }
    if (explicitRoute || points.size() < 3) {
        if (document.closedLoop && steps.size() > 1 && !points.isEmpty())
            append(points.first());
        return points;
    }
    const auto controls = points;
    points.clear();
    const qsizetype count = controls.size();
    const auto at = [&document, &controls, count](qsizetype index) {
        return controls[document.closedLoop ? (index % count + count) % count
                                            : std::clamp(index, qsizetype(0), count - 1)];
    };
    const qsizetype segments = document.closedLoop ? count : count - 1;
    const int samples = std::clamp(12000 / static_cast<int>(segments), 3, 28);
    for (qsizetype segment = 0; segment < segments; ++segment) {
        const auto p0 = at(segment - 1), p1 = at(segment), p2 = at(segment + 1),
                   p3 = at(segment + 2);
        for (int j = 0; j < samples; ++j) {
            const float t = static_cast<float>(j) / samples, t2 = t * t, t3 = t2 * t;
            append(((p1 * 2) + (p2 - p0) * t + (p0 * 2 - p1 * 5 + p2 * 4 - p3) * t2 +
                    (-p0 + p1 * 3 - p2 * 3 + p3) * t3) *
                   .5f);
        }
    }
    append(at(segments));
    return points;
}

QPainterPath routePath(const TrackDocument &document) {
    QPainterPath path;
    if (document.routeStepsDefined || !document.routeSteps.isEmpty()) {
        const auto points = routePolyline(document);
        if (!points.isEmpty()) {
            path.moveTo(points.first().x(), points.first().z());
            for (qsizetype i = 1; i < points.size(); ++i)
                path.lineTo(points[i].x(), points[i].z());
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
        const auto points = routePolyline(document);
        double length = 0;
        for (qsizetype i = 1; i < points.size(); ++i)
            length += (points[i] - points[i - 1]).length();
        return length;
    }
    // Qt integrates cubic segments adaptively; this is geometric path length,
    // not a claimed flight distance or physics simulation.
    return routePath(document).length();
}

} // namespace ot
