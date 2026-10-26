#include "velocidrone.h"
#include "document.h"

#include <QFile>
#include <QHash>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonParseError>
#include <QMatrix3x3>
#include <QQuaternion>
#include <QSaveFile>
#include <QSet>
#include <QStringList>
#include <QVector3D>
#include <algorithm>
#include <cmath>
#include <limits>

#ifdef Q_OS_WIN
#ifndef NOMINMAX
#define NOMINMAX
#endif
// Windows defines the types required by BCrypt. Keep this include order.
// clang-format off
#include <windows.h>
#include <bcrypt.h>
// clang-format on
#endif

namespace ot {
namespace {
constexpr qsizetype MaxTrackBytes = 32 * 1024 * 1024;
constexpr int MaxGates = 20000;
constexpr double NominalGateAperture = 2.4;
const QString NativeObject = QStringLiteral("velocidroneObject");
const QString NativeBaseline = QStringLiteral("velocidroneBaseline");
const QString NativeRoot = QStringLiteral("velocidroneRoot");
const QString NativeScene = QStringLiteral("velocidroneSceneId");
const QString PillarBarrier = QStringLiteral("velocidronePillarBarrier");
const QString OwnedPillarBarriers = QStringLiteral("velocidroneOwnedPillarBarriers");
const QString FrameBarriers = QStringLiteral("velocidroneFrameBarriers");
const QString OwnedFrameBarriers = QStringLiteral("velocidroneOwnedFrameBarriers");
constexpr int PillarPrefab = 2218;         // BlockBlack: native one-metre stretchable cube.
constexpr int PillarCheckpointPrefab = 88; // Invisible helper in its local Y/Z plane.
constexpr double InvisibleUnitWindow = 1.9286329746246338;
constexpr double InvisibleCenterOffset = .9900000095367432;
const QString NativeTrailer = QStringLiteral("velocidroneTrailer");
const QString OwnedStartGrids = QStringLiteral("velocidroneOwnedStartGrids");
const QString NativeTakeoffObject = QStringLiteral("velocidroneStartGrid");
const QString NativeTakeoffBaseline = QStringLiteral("velocidroneStartGridBaseline");
const QString NativeStepGeometry = QStringLiteral("velocidroneStepGeometry");
const QString NativeArrayIndex = QStringLiteral("velocidroneArrayIndex");

bool fail(QString *error, const QString &message) {
    if (error)
        *error = message;
    return false;
}

bool crypt(const QByteArray &input, bool encrypt, QByteArray *output, QString *error) {
#ifdef Q_OS_WIN
    struct Algorithm {
        BCRYPT_ALG_HANDLE value = nullptr;
        ~Algorithm() {
            if (value)
                BCryptCloseAlgorithmProvider(value, 0);
        }
    } algorithm;
    struct Key {
        BCRYPT_KEY_HANDLE value = nullptr;
        ~Key() {
            if (value)
                BCryptDestroyKey(value);
        }
    };
    const auto ok = [](NTSTATUS status) { return status >= 0; };
    if (!ok(BCryptOpenAlgorithmProvider(&algorithm.value, BCRYPT_AES_ALGORITHM, nullptr, 0)))
        return fail(error, QStringLiteral("无法初始化 Windows AES 加密服务。"));
    if (!ok(BCryptSetProperty(
            algorithm.value, BCRYPT_CHAINING_MODE,
            reinterpret_cast<PUCHAR>(const_cast<wchar_t *>(BCRYPT_CHAIN_MODE_ECB)),
            sizeof(BCRYPT_CHAIN_MODE_ECB), 0)))
        return fail(error, QStringLiteral("无法设置赛道文件所需的 AES 模式。"));
    ULONG objectLength = 0, returnedLength = 0;
    if (!ok(BCryptGetProperty(algorithm.value, BCRYPT_OBJECT_LENGTH,
                              reinterpret_cast<PUCHAR>(&objectLength), sizeof(objectLength),
                              &returnedLength, 0)))
        return fail(error, QStringLiteral("无法读取 Windows AES 参数。"));
    QByteArray object(static_cast<qsizetype>(objectLength), '\0');
    Key key; // Destroy the handle before releasing its backing object buffer.
    // A public interoperability key, not a user credential.
    QByteArray keyBytes("VelocidrrdicoleV", 16);
    if (!ok(BCryptGenerateSymmetricKey(
            algorithm.value, &key.value, reinterpret_cast<PUCHAR>(object.data()), objectLength,
            reinterpret_cast<PUCHAR>(keyBytes.data()), static_cast<ULONG>(keyBytes.size()), 0)))
        return fail(error, QStringLiteral("无法创建赛道文件加密密钥。"));
    auto operate = [&](PUCHAR destination, ULONG capacity, ULONG *length) {
        PUCHAR source = reinterpret_cast<PUCHAR>(const_cast<char *>(input.constData()));
        if (encrypt)
            return BCryptEncrypt(key.value, source, static_cast<ULONG>(input.size()), nullptr,
                                 nullptr, 0, destination, capacity, length, BCRYPT_BLOCK_PADDING);
        return BCryptDecrypt(key.value, source, static_cast<ULONG>(input.size()), nullptr, nullptr,
                             0, destination, capacity, length, BCRYPT_BLOCK_PADDING);
    };
    ULONG needed = 0;
    if (!ok(operate(nullptr, 0, &needed)))
        return fail(error, QStringLiteral("赛道文件解密失败：文件损坏或不是受支持的 .trk 格式。"));
    QByteArray result(static_cast<qsizetype>(needed), '\0');
    ULONG written = 0;
    if (!ok(operate(reinterpret_cast<PUCHAR>(result.data()), needed, &written)))
        return fail(error, QStringLiteral("赛道文件校验失败：加密数据或填充不完整。"));
    result.resize(static_cast<qsizetype>(written));
    *output = std::move(result);
    return true;
#else
    Q_UNUSED(input)
    Q_UNUSED(encrypt)
    Q_UNUSED(output)
    return fail(error,
                QStringLiteral("此版本的原生 .trk 导入导出仅支持 Windows。工程文件不受影响。"));
#endif
}

int sceneryId(const TrackDocument &document) {
    if (document.scenery == QStringLiteral("Football Stadium"))
        return 8;
    if (document.scenery == QStringLiteral("Empty Scene Day"))
        return 16;
    if (document.scenery == QStringLiteral("Empty Scene Night"))
        return 17;
    if (document.scenery == QStringLiteral("Sports Hall"))
        return 21;
    if (document.scenery == QStringLiteral("Warehouse"))
        return 19;
    if (document.scenery == QStringLiteral("Countryside"))
        return 12;
    if (document.scenery.startsWith(QStringLiteral("Scene "))) {
        bool valid = false;
        const int id = document.scenery.mid(6).toInt(&valid);
        if (valid && id > 0)
            return id;
    }
    return 0;
}

QString sceneryName(int id) {
    switch (id) {
    case 8:
        return QStringLiteral("Football Stadium");
    case 12:
        return QStringLiteral("Countryside");
    case 16:
        return QStringLiteral("Empty Scene Day");
    case 17:
        return QStringLiteral("Empty Scene Night");
    case 19:
        return QStringLiteral("Warehouse");
    case 21:
        return QStringLiteral("Sports Hall");
    default:
        return QStringLiteral("Scene %1").arg(id);
    }
}

bool finiteGate(const Gate &gate) {
    return std::isfinite(gate.position.x()) && std::isfinite(gate.position.y()) &&
           std::isfinite(gate.elevation) && std::isfinite(gate.width) &&
           std::isfinite(gate.height) && std::isfinite(gate.yaw) &&
           std::abs(gate.position.x()) <= 1000000 && std::abs(gate.position.y()) <= 1000000 &&
           std::abs(gate.elevation) <= 1000000 && std::abs(gate.yaw) < 1000000000 &&
           gate.width >= 0.02 && gate.width <= 1000 && gate.height >= 0.02 && gate.height <= 1000;
}

QJsonObject baseline(const Gate &gate) {
    return {{QStringLiteral("x"), gate.position.x()},
            {QStringLiteral("z"), gate.position.y()},
            {QStringLiteral("elevation"), gate.elevation},
            {QStringLiteral("width"), gate.width},
            {QStringLiteral("height"), gate.height},
            {QStringLiteral("yaw"), gate.yaw},
            {QStringLiteral("type"), gate.type},
            {QStringLiteral("color"), gateColor(gate).name(QColor::HexRgb)},
            {QStringLiteral("frameGeometry"),
             gate.extra.value(QStringLiteral("frameGeometry")).toString()},
            {QStringLiteral("beamThickness"),
             gate.extra.value(QStringLiteral("beamThickness")).toDouble(.1)},
            {QStringLiteral("beamOverhang"),
             gate.extra.value(QStringLiteral("beamOverhang")).toDouble(.2)},
            {QStringLiteral("passColumn"), gate.extra.value(QStringLiteral("passColumn")).toInt(0)},
            {QStringLiteral("passLevel"), gate.extra.value(QStringLiteral("passLevel")).toInt(0)},
            {QStringLiteral("passSide"),
             gate.extra.value(QStringLiteral("passSide")).toInt(1) < 0 ? -1 : 1}};
}

bool same(double a, double b) {
    return std::abs(a - b) < 1e-8;
}

bool numericArray(const QJsonValue &value, int count, QJsonArray *result,
                  double maximum = std::numeric_limits<int>::max()) {
    if (!value.isArray())
        return false;
    const auto array = value.toArray();
    if (array.size() != count)
        return false;
    for (const auto &item : array) {
        if (!item.isDouble() || !std::isfinite(item.toDouble()) ||
            std::abs(item.toDouble()) > maximum)
            return false;
    }
    *result = array;
    return true;
}

QQuaternion quaternion(const QJsonArray &rotation) {
    return QQuaternion(static_cast<float>(rotation[0].toDouble() / 1000.0),
                       static_cast<float>(rotation[1].toDouble() / 1000.0),
                       static_cast<float>(rotation[2].toDouble() / 1000.0),
                       static_cast<float>(rotation[3].toDouble() / 1000.0))
        .normalized();
}

QJsonArray rotationArray(const QQuaternion &rotation) {
    const auto q = rotation.normalized();
    return {qRound(q.scalar() * 1000.0), qRound(q.x() * 1000.0), qRound(q.y() * 1000.0),
            qRound(q.z() * 1000.0)};
}

bool validNativeTransform(const QJsonObject &object) {
    const auto trans = object.value(QStringLiteral("trans")).toObject();
    QJsonArray pos, rot, scale;
    if (!numericArray(trans.value(QStringLiteral("pos")), 3, &pos) ||
        !numericArray(trans.value(QStringLiteral("rot")), 4, &rot, 1000000) ||
        !numericArray(trans.value(QStringLiteral("scale")), 3, &scale, 1000000) ||
        scale[0].toDouble() <= 0 || scale[1].toDouble() <= 0 || scale[2].toDouble() <= 0)
        return false;
    double norm = 0;
    for (const auto &entry : rot)
        norm += entry.toDouble() * entry.toDouble();
    return norm >= 1;
}

bool unchangedPillar(const Gate &gate) {
    const auto previous = gate.extra.value(NativeBaseline).toObject();
    return previous.value(QStringLiteral("type")).toString() == QStringLiteral("pillar") &&
           same(previous.value(QStringLiteral("x")).toDouble(), gate.position.x()) &&
           same(previous.value(QStringLiteral("z")).toDouble(), gate.position.y()) &&
           same(previous.value(QStringLiteral("elevation")).toDouble(), gate.elevation) &&
           same(previous.value(QStringLiteral("width")).toDouble(), gate.width) &&
           same(previous.value(QStringLiteral("height")).toDouble(), gate.height) &&
           same(previous.value(QStringLiteral("yaw")).toDouble(), gate.yaw) &&
           previous.value(QStringLiteral("color")).toString() ==
               gateColor(gate).name(QColor::HexRgb) &&
           previous.value(QStringLiteral("passSide")).toInt(1) ==
               (gate.extra.value(QStringLiteral("passSide")).toInt(1) < 0 ? -1 : 1);
}

QJsonObject pillarBarrierObject(const Gate &gate) {
    auto object = gate.extra.value(PillarBarrier).toObject();
    if (unchangedPillar(gate) && !object.isEmpty())
        return object;
    auto transform = object.value(QStringLiteral("trans")).toObject();
    transform.insert(QStringLiteral("pos"),
                     QJsonArray{qRound(gate.position.x() * 100),
                                qRound((gate.elevation - gate.height / 2) * 100),
                                qRound(gate.position.y() * 100)});
    transform.insert(QStringLiteral("rot"), rotationArray(QQuaternion::fromAxisAndAngle(
                                                0, 1, 0, static_cast<float>(gate.yaw))));
    transform.insert(
        QStringLiteral("scale"),
        QJsonArray{qRound(gate.width * 100), qRound(gate.height * 100), qRound(gate.width * 100)});
    object.insert(QStringLiteral("prefab"), nearestNativeBlockColor(gateColor(gate)).prefab);
    object.insert(QStringLiteral("trans"), transform);
    return object;
}

QString pillarBucket(const QPointF &point) {
    return QString::number(static_cast<qint64>(std::floor(point.x() * 4))) + QLatin1Char(':') +
           QString::number(static_cast<qint64>(std::floor(point.y() * 4)));
}

bool compositeFrame(const Gate &gate) {
    static const QSet<QString> types{QStringLiteral("standard"),  QStringLiteral("start"),
                                     QStringLiteral("double"),    QStringLiteral("triple"),
                                     QStringLiteral("ladder"),    QStringLiteral("cube"),
                                     QStringLiteral("cube-stack")};
    return types.contains(gate.type) && gateFrameMetrics(gate).centerlineModel;
}

bool unchangedFrame(const Gate &gate) {
    return compositeFrame(gate) && gate.extra.value(NativeBaseline).toObject() == baseline(gate) &&
           gate.extra.value(FrameBarriers).toArray().size() == gateFrameBeams(gate).size();
}

QJsonArray frameBarrierObjects(const Gate &gate) {
    if (unchangedFrame(gate))
        return gate.extra.value(FrameBarriers).toArray();
    QJsonArray objects;
    const auto rotation = QQuaternion::fromAxisAndAngle(0, 1, 0, static_cast<float>(gate.yaw));
    const int prefab = nearestNativeBlockColor(gateColor(gate)).prefab;
    for (const auto &beam : gateFrameBeams(gate)) {
        const auto offset =
            rotation.rotatedVector(beam.center - QVector3D(0, beam.size.y() / 2, 0));
        QJsonObject transform{
            {QStringLiteral("pos"), QJsonArray{qRound((gate.position.x() + offset.x()) * 100),
                                               qRound((gate.elevation + offset.y()) * 100),
                                               qRound((gate.position.y() + offset.z()) * 100)}},
            {QStringLiteral("rot"), rotationArray(rotation)},
            {QStringLiteral("scale"),
             QJsonArray{qRound(beam.size.x() * 100), qRound(beam.size.y() * 100),
                        qRound(beam.size.z() * 100)}}};
        objects.append(
            QJsonObject{{QStringLiteral("prefab"), prefab}, {QStringLiteral("trans"), transform}});
    }
    return objects;
}

QJsonArray vectorArray(const QVector3D &value) {
    return {value.x(), value.y(), value.z()};
}
QJsonObject geometryBaseline(const RouteStepGeometry &geometry) {
    return {{QStringLiteral("position"), vectorArray(geometry.position)},
            {QStringLiteral("normal"), vectorArray(geometry.normal)},
            {QStringLiteral("right"), vectorArray(geometry.right)},
            {QStringLiteral("up"), vectorArray(geometry.up)},
            {QStringLiteral("width"), geometry.openingWidth},
            {QStringLiteral("height"), geometry.openingHeight}};
}
RouteStepGeometry stepGeometry(const Gate &gate, const RouteStep &step) {
    TrackDocument one;
    one.gates.append(gate);
    return routeStepGeometry(one, step);
}
QQuaternion checkpointRotation(const RouteStepGeometry &geometry) {
    // Installed race prefab88: X normal, Y right, Z up. This is not the
    // differently oriented editor-helper mesh used by third-party viewers.
    QMatrix3x3 matrix;
    for (int row = 0; row < 3; ++row) {
        matrix(row, 0) = geometry.normal[row];
        matrix(row, 1) = geometry.right[row];
        matrix(row, 2) = geometry.up[row];
    }
    auto rotation = QQuaternion::fromRotationMatrix(matrix).normalized();
    // q and -q represent the same rotation. Prefer the canonical sign used by
    // the independently captured front-facing native checkpoints.
    if (rotation.scalar() < 0)
        rotation = -rotation;
    return rotation;
}
QJsonObject checkpointTransform(const RouteStepGeometry &geometry) {
    const int widthScale = qRound(geometry.openingWidth / InvisibleUnitWindow * 100);
    const int heightScale = qRound(geometry.openingHeight / InvisibleUnitWindow * 100);
    const auto rotation = checkpointRotation(geometry);
    const auto origin = geometry.position - geometry.up * static_cast<float>(InvisibleCenterOffset *
                                                                             heightScale / 100.0);
    return {{QStringLiteral("pos"), QJsonArray{qRound(origin.x() * 100), qRound(origin.y() * 100),
                                               qRound(origin.z() * 100)}},
            {QStringLiteral("rot"), rotationArray(rotation)},
            {QStringLiteral("scale"), QJsonArray{100, widthScale, heightScale}}};
}
RouteStepGeometry invisibleGeometry(const QJsonObject &object) {
    RouteStepGeometry geometry;
    if (object.value(QStringLiteral("prefab")).toInt() != 88)
        return geometry;
    const auto transform = object.value(QStringLiteral("trans")).toObject();
    QJsonArray position, rotation, scale;
    if (!numericArray(transform.value(QStringLiteral("pos")), 3, &position) ||
        !numericArray(transform.value(QStringLiteral("rot")), 4, &rotation, 1000000) ||
        !numericArray(transform.value(QStringLiteral("scale")), 3, &scale, 1000000) ||
        scale[0].toDouble() <= 0 || scale[1].toDouble() <= 0 || scale[2].toDouble() <= 0)
        return geometry;
    const auto q = quaternion(rotation);
    geometry.normal = q.rotatedVector({1, 0, 0});
    geometry.right = q.rotatedVector({0, 1, 0});
    geometry.up = q.rotatedVector({0, 0, 1});
    geometry.position =
        QVector3D(static_cast<float>(position[0].toDouble() / 100),
                  static_cast<float>(position[1].toDouble() / 100),
                  static_cast<float>(position[2].toDouble() / 100)) +
        geometry.up * static_cast<float>(InvisibleCenterOffset * scale[2].toDouble() / 100);
    geometry.openingWidth = InvisibleUnitWindow * scale[1].toDouble() / 100;
    geometry.openingHeight = InvisibleUnitWindow * scale[2].toDouble() / 100;
    geometry.valid = true;
    return geometry;
}
QStringList gateFaces(const Gate &gate) {
    return gateFrameMetrics(gate).centerDepth > 0
               ? QStringList{QStringLiteral("front"), QStringLiteral("back"),
                             QStringLiteral("left"),  QStringLiteral("right"),
                             QStringLiteral("top"),   QStringLiteral("bottom")}
               : QStringList{QStringLiteral("front"), QStringLiteral("back")};
}
bool samePortal(const RouteStepGeometry &a, const RouteStepGeometry &b) {
    return a.valid && b.valid && (a.position - b.position).length() < .04f &&
           std::abs(a.openingWidth - b.openingWidth) < .025 &&
           std::abs(a.openingHeight - b.openingHeight) < .025 &&
           QVector3D::dotProduct(a.normal, b.normal) > .99998f &&
           QVector3D::dotProduct(a.right, b.right) > .99998f &&
           QVector3D::dotProduct(a.up, b.up) > .99998f;
}
bool matchingStep(const Gate &gate, const RouteStepGeometry &portal, RouteStep *result) {
    const auto metrics = gateFrameMetrics(gate);
    for (const auto &face : gateFaces(gate))
        for (int column = 0; column < metrics.columns; ++column)
            for (int level = 0; level < metrics.levels; ++level)
                for (bool reverse : {false, true}) {
                    RouteStep step;
                    step.gateId = gate.id;
                    step.face = face;
                    step.column = column;
                    step.level = level;
                    step.reverse = reverse;
                    if (samePortal(stepGeometry(gate, step), portal)) {
                        *result = step;
                        return true;
                    }
                }
    return false;
}
QJsonObject takeoffBaseline(const TakeoffPoint &point) {
    return {{QStringLiteral("x"), point.position.x()},
            {QStringLiteral("z"), point.position.y()},
            {QStringLiteral("elevation"), point.elevation},
            {QStringLiteral("yaw"), point.yaw}};
}
bool startGridPrefab(int prefab) {
    return prefab == 90 || prefab == 422 || prefab == 605 || prefab == 696 || prefab == 756 ||
           prefab == 2013;
}
QJsonObject startGridObject(const TakeoffPoint &point) {
    const auto original = point.extra.value(NativeTakeoffObject).toObject();
    if (!original.isEmpty() &&
        point.extra.value(NativeTakeoffBaseline).toObject() == takeoffBaseline(point))
        return original;
    // Race422 slot rotations are identity: heading0 is native world+Z. The
    // point is the grid base, not the assigned quad's raised/offset slot.
    return {{QStringLiteral("prefab"), 422},
            {QStringLiteral("trans"),
             QJsonObject{{QStringLiteral("pos"), QJsonArray{qRound(point.position.x() * 100),
                                                            qRound(point.elevation * 100),
                                                            qRound(point.position.y() * 100)}},
                         {QStringLiteral("rot"), rotationArray(QQuaternion::fromAxisAndAngle(
                                                     0, 1, 0, static_cast<float>(point.yaw)))},
                         {QStringLiteral("scale"), QJsonArray{100, 100, 100}}}}};
}
void restoreTakeoff(TrackDocument *document, const QJsonArray &barriers) {
    QJsonArray owned;
    QJsonObject original;
    for (qsizetype i = 0; i < barriers.size(); ++i) {
        const auto object = barriers[i].toObject();
        if (startGridPrefab(object.value(QStringLiteral("prefab")).toInt())) {
            owned.append(static_cast<int>(i));
            original = object;
        }
    }
    if (!owned.isEmpty())
        document->extra.insert(OwnedStartGrids, owned);
    if (owned.size() != 1)
        return; // Ambiguous/missing grid: require explicit user configuration.
    const int prefab = original.value(QStringLiteral("prefab")).toInt();
    if (prefab != 90 && prefab != 422)
        return; // Other catalogued grids are preserved but not pose-calibrated.
    const auto transform = original.value(QStringLiteral("trans")).toObject();
    QJsonArray pos, rot, scale;
    if (!numericArray(transform.value(QStringLiteral("pos")), 3, &pos) ||
        !numericArray(transform.value(QStringLiteral("rot")), 4, &rot, 1000000) ||
        !numericArray(transform.value(QStringLiteral("scale")), 3, &scale, 1000000) ||
        scale[0].toDouble() <= 0 || scale[1].toDouble() <= 0 || scale[2].toDouble() <= 0)
        return;
    double norm = 0;
    for (const auto &value : rot)
        norm += value.toDouble() * value.toDouble();
    if (norm < 1)
        return;
    auto orientation = quaternion(rot);
    if (prefab == 90) {
        // Installed race90 slots have a non-identity local rotation. Their
        // forward points along local+X; using only the root's+Z would display
        // an incorrect aircraft heading. Preserve the original root until an
        // edit explicitly replaces the grid with calibrated422.
        orientation *= QQuaternion(.5000863671f, .4999136627f, .5000864267f, .4999134839f);
    }
    const auto forward = orientation.rotatedVector({0, 0, 1});
    auto &point = document->takeoff;
    point.enabled = true;
    point.position = {pos[0].toDouble() / 100, pos[2].toDouble() / 100};
    point.elevation = pos[1].toDouble() / 100;
    point.yaw = std::atan2(forward.x(), forward.z()) * 180 / 3.14159265358979323846;
    point.extra.insert(NativeTakeoffObject, original);
    point.extra.insert(NativeTakeoffBaseline, takeoffBaseline(point));
}
struct NativeBlock {
    int index;
    QJsonObject raw;
    QJsonArray pos, scale;
    QQuaternion rotation;
};
bool matchingBar(const NativeBlock &block, const QJsonObject &object) {
    if (block.raw.value(QStringLiteral("prefab")) != object.value(QStringLiteral("prefab")))
        return false;
    const auto trans = object.value(QStringLiteral("trans")).toObject();
    const auto pos = trans.value(QStringLiteral("pos")).toArray();
    const auto scale = trans.value(QStringLiteral("scale")).toArray();
    if (std::abs(QQuaternion::dotProduct(
            block.rotation, quaternion(trans.value(QStringLiteral("rot")).toArray()))) < .99999f)
        return false;
    for (int axis = 0; axis < 3; ++axis)
        if (std::abs(block.pos[axis].toDouble() - pos[axis].toDouble()) > 3 ||
            std::abs(block.scale[axis].toDouble() - scale[axis].toDouble()) > 1)
            return false;
    return true;
}
void restorePhysicalRoute(TrackDocument *document, const QJsonArray &barriers) {
    QVector<NativeBlock> blocks;
    QHash<QString, QVector<int>> buckets;
    QVector<double> widths, heights;
    auto addDimension = [](QVector<double> &values, double value) {
        if (value < .02 || value > 1000)
            return;
        for (double previous : values)
            if (same(previous, value))
                return;
        values.append(value);
    };
    for (qsizetype i = 0; i < barriers.size(); ++i) {
        const auto object = barriers[i].toObject();
        const int prefab = object.value(QStringLiteral("prefab")).toInt();
        if (prefab < 2218 || prefab > 2230)
            continue;
        const auto trans = object.value(QStringLiteral("trans")).toObject();
        QJsonArray pos, rot, scale;
        if (!numericArray(trans.value(QStringLiteral("pos")), 3, &pos) ||
            !numericArray(trans.value(QStringLiteral("rot")), 4, &rot, 1000000) ||
            !numericArray(trans.value(QStringLiteral("scale")), 3, &scale, 1000000) ||
            scale[0].toDouble() <= 0 || scale[1].toDouble() <= 0 || scale[2].toDouble() <= 0)
            continue;
        if (!validNativeTransform(object))
            continue;
        const auto q = quaternion(rot);
        if (std::abs(q.x()) > .002f || std::abs(q.z()) > .002f)
            continue; // Only yaw-aligned physical stock groups are calibrated.
        const int index = static_cast<int>(blocks.size());
        blocks.append({static_cast<int>(i), object, pos, scale, q});
        buckets[pillarBucket({pos[0].toDouble() / 100, pos[2].toDouble() / 100})].append(index);
        if (nearbyint(scale[1].toDouble()) == 10 && nearbyint(scale[2].toDouble()) == 10)
            addDimension(widths, scale[0].toDouble() / 100 - .2);
        if (nearbyint(scale[0].toDouble()) == 10 && nearbyint(scale[1].toDouble()) == 10)
            addDimension(widths, scale[2].toDouble() / 100 - .2);
        if (nearbyint(scale[0].toDouble()) == 10 && nearbyint(scale[2].toDouble()) == 10)
            addDimension(heights, scale[1].toDouble() / 100 - .2);
    }
    auto options = [](double approximate, const QVector<double> &values) {
        QVector<double> result;
        for (double value : values)
            if (std::abs(value - approximate) < .04)
                result.append(value);
        if (result.isEmpty())
            result.append(qRound(approximate * 100) / 100.0);
        return result;
    };
    QSet<int> owned;
    auto findBars = [&](const QJsonArray &expected, QJsonArray *originals, QSet<int> *selected) {
        int minimumCopies = std::numeric_limits<int>::max(), maximumCopies = 0;
        for (const auto &value : expected) {
            const auto object = value.toObject();
            const auto pos = object.value(QStringLiteral("trans"))
                                 .toObject()
                                 .value(QStringLiteral("pos"))
                                 .toArray();
            const QPointF point(pos[0].toDouble() / 100, pos[2].toDouble() / 100);
            QVector<int> matches;
            QSet<int> checked;
            for (int x = -1; x <= 1; ++x)
                for (int z = -1; z <= 1; ++z) {
                    const auto bucket =
                        buckets.constFind(pillarBucket(point + QPointF(x * .25, z * .25)));
                    if (bucket == buckets.cend())
                        continue;
                    for (int index : bucket.value()) {
                        const auto &block = blocks[index];
                        if (checked.contains(index) || owned.contains(block.index) ||
                            selected->contains(block.index))
                            continue;
                        checked.insert(index);
                        if (matchingBar(block, object))
                            matches.append(index);
                    }
                }
            if (matches.isEmpty())
                return false;
            minimumCopies = std::min(minimumCopies, static_cast<int>(matches.size()));
            maximumCopies = std::max(maximumCopies, static_cast<int>(matches.size()));
            const auto &chosen = blocks[matches.first()];
            selected->insert(chosen.index);
            originals->append(chosen.raw);
        }
        // Adjacent independent single frames have a duplicated central upright.
        // A true shared-stock compound has uniform multiplicity throughout.
        return expected.size() <= 4 || minimumCopies == maximumCopies;
    };
    const auto rawGates = document->gates;
    document->gates.clear();
    document->routeSteps.clear();
    document->routeStepsDefined = true;
    for (const auto &rawGate : rawGates) {
        const auto raw = rawGate.extra.value(NativeObject).toObject();
        const auto portal = invisibleGeometry(raw);
        RouteStep step;
        bool matched = false;
        if (portal.valid) {
            for (const auto &physical : document->gates) {
                if (!(compositeFrame(physical) || physical.type == QStringLiteral("pillar")))
                    continue;
                if (matchingStep(physical, portal, &step)) {
                    matched = true;
                    break;
                }
            }
        }
        if (portal.valid && !matched) {
            const double theta = std::abs(portal.normal.y()) < .01
                                     ? std::atan2(portal.normal.x(), portal.normal.z())
                                     : std::atan2(-portal.right.z(), portal.right.x());
            const QStringList shapes{QStringLiteral("cube-stack"),
                                     QStringLiteral("cube"),
                                     QStringLiteral("triple"),
                                     QStringLiteral("double"),
                                     QStringLiteral("ladder"),
                                     raw.value(QStringLiteral("start")).toBool()
                                         ? QStringLiteral("start")
                                         : QStringLiteral("standard")};
            for (const auto &shape : shapes) {
                if (matched)
                    break;
                const bool horizontal = std::abs(portal.normal.y()) > .99;
                const auto widthOptions = options(portal.openingWidth + .1, widths);
                auto heightOptions =
                    horizontal ? heights : options(portal.openingHeight + .1, heights);
                if (heightOptions.isEmpty())
                    heightOptions.append(3);
                for (double width : widthOptions) {
                    if (matched)
                        break;
                    for (double height : heightOptions) {
                        if (matched)
                            break;
                        auto candidate = makeGate(shape, width, height);
                        candidate.id = rawGate.id;
                        candidate.position = {};
                        candidate.elevation = 0;
                        const auto metrics = gateFrameMetrics(candidate);
                        for (int turn = 0; turn < 4 && !matched; ++turn) {
                            candidate.yaw = std::remainder(
                                theta * 180 / 3.14159265358979323846 + turn * 90, 360.0);
                            const auto faces = gateFaces(candidate);
                            for (const auto &face : faces) {
                                if (matched)
                                    break;
                                for (int column = 0; column < metrics.columns && !matched; ++column)
                                    for (int level = 0; level < metrics.levels && !matched; ++level)
                                        for (bool reverse : {false, true}) {
                                            candidate.position = {};
                                            candidate.elevation = 0;
                                            RouteStep probe;
                                            probe.gateId = candidate.id;
                                            probe.face = face;
                                            probe.column = column;
                                            probe.level = level;
                                            probe.reverse = reverse;
                                            auto geometry = stepGeometry(candidate, probe);
                                            geometry.position = portal.position;
                                            if (!samePortal(geometry, portal))
                                                continue;
                                            const auto relative =
                                                stepGeometry(candidate, probe).position;
                                            const auto center = portal.position - relative;
                                            candidate.position = {center.x(), center.z()};
                                            candidate.elevation = center.y();
                                            for (const auto &color : nativeBlockColors()) {
                                                setGateColor(candidate, color.color);
                                                QJsonArray originals;
                                                QSet<int> selected;
                                                if (!findBars(frameBarrierObjects(candidate),
                                                              &originals, &selected))
                                                    continue;
                                                candidate.extra.insert(FrameBarriers, originals);
                                                candidate.extra.insert(NativeObject, raw);
                                                candidate.extra.insert(NativeBaseline,
                                                                       baseline(candidate));
                                                document->gates.append(candidate);
                                                owned.unite(selected);
                                                step = probe;
                                                matched = true;
                                                break;
                                            }
                                            if (matched)
                                                break;
                                        }
                            }
                        }
                    }
                }
            }
        }
        if (portal.valid && !matched) {
            // A pillar is recognized only by its exact adjacent lane convention,
            // not by interpreting an arbitrary narrow barrier as a route point.
            for (const auto &block : blocks) {
                if (owned.contains(block.index) ||
                    !same(block.scale[0].toDouble(), block.scale[2].toDouble()))
                    continue;
                Gate candidate;
                candidate.id = rawGate.id;
                candidate.type = QStringLiteral("pillar");
                candidate.width = block.scale[0].toDouble() / 100;
                candidate.height = block.scale[1].toDouble() / 100;
                candidate.position = {block.pos[0].toDouble() / 100, block.pos[2].toDouble() / 100};
                candidate.elevation = block.pos[1].toDouble() / 100 + candidate.height / 2;
                const auto forward = block.rotation.rotatedVector({0, 0, 1});
                candidate.yaw = std::atan2(forward.x(), forward.z()) * 180 / 3.14159265358979323846;
                for (int side : {-1, 1}) {
                    candidate.extra.insert(QStringLiteral("passSide"), side);
                    if (!matchingStep(candidate, portal, &step))
                        continue;
                    for (const auto &color : nativeBlockColors())
                        if (color.prefab == block.raw.value(QStringLiteral("prefab")).toInt())
                            setGateColor(candidate, color.color);
                    candidate.extra.insert(PillarBarrier, block.raw);
                    candidate.extra.insert(NativeObject, raw);
                    candidate.extra.insert(NativeBaseline, baseline(candidate));
                    document->gates.append(candidate);
                    owned.insert(block.index);
                    matched = true;
                    break;
                }
                if (matched)
                    break;
            }
        }
        if (!matched) {
            auto fallback = rawGate;
            if (portal.valid) {
                fallback.position = {portal.position.x(), portal.position.z()};
                fallback.elevation = portal.position.y();
                fallback.width = portal.openingWidth;
                fallback.height = portal.openingHeight;
                fallback.yaw =
                    std::atan2(portal.normal.x(), portal.normal.z()) * 180 / 3.14159265358979323846;
                fallback.extra.insert(NativeBaseline, baseline(fallback));
            }
            document->gates.append(fallback);
            step.gateId = fallback.id;
        }
        step.id = newGateId();
        step.extra.insert(NativeObject, raw);
        step.extra.insert(NativeArrayIndex, rawGate.extra.value(NativeArrayIndex));
        for (const auto &physical : document->gates)
            if (physical.id == step.gateId) {
                step.extra.insert(NativeStepGeometry,
                                  geometryBaseline(stepGeometry(physical, step)));
                break;
            }
        document->routeSteps.append(step);
    }
    QJsonArray indices;
    auto values = owned.values();
    std::sort(values.begin(), values.end());
    for (int value : values)
        indices.append(value);
    if (!indices.isEmpty())
        document->extra.insert(OwnedFrameBarriers, indices);
}

bool makeNativeObject(const Gate &gate, int index, bool isStart, QJsonObject *result,
                      QString *error) {
    if (!finiteGate(gate))
        return fail(error,
                    QStringLiteral("第 %1 个组件的位置、角度或尺寸超出有效范围。 ").arg(index + 1));
    QJsonObject object = gate.extra.value(NativeObject).toObject();
    const QJsonObject previous = gate.extra.value(NativeBaseline).toObject();
    const bool hasNative = !object.isEmpty() && !previous.isEmpty();
    int prefab = 0;
    if (hasNative && previous.value(QStringLiteral("type")).toString() == gate.type) {
        prefab = object.value(QStringLiteral("prefab")).toInt();
    } else if (gate.type == QStringLiteral("standard") || gate.type == QStringLiteral("start")) {
        prefab = 285;
    } else {
        return fail(error,
                    QStringLiteral("第 %1 个组件“%2”暂不支持新建后导出为原生赛道。"
                                   "请改用标准门、起终点门或实体转向柱；旗帜仅保留原生导入兼容。")
                        .arg(index + 1)
                        .arg(gate.type));
    }
    if (prefab == 88 && hasNative) {
        auto geometry = invisibleGeometry(object);
        const double oldWidth = previous.value(QStringLiteral("width")).toDouble();
        const double oldHeight = previous.value(QStringLiteral("height")).toDouble();
        if (!geometry.valid || oldWidth <= 0 || oldHeight <= 0)
            return fail(error, QStringLiteral("原生不可见检查点的尺寸基准无效。"));
        if (baseline(gate) != previous) {
            const auto delta = QQuaternion::fromAxisAndAngle(
                0, 1, 0,
                static_cast<float>(gate.yaw - previous.value(QStringLiteral("yaw")).toDouble()));
            geometry.normal = delta.rotatedVector(geometry.normal);
            geometry.right = delta.rotatedVector(geometry.right);
            geometry.up = delta.rotatedVector(geometry.up);
            geometry.position = {static_cast<float>(gate.position.x()),
                                 static_cast<float>(gate.elevation),
                                 static_cast<float>(gate.position.y())};
            geometry.openingWidth *= gate.width / oldWidth;
            geometry.openingHeight *= gate.height / oldHeight;
            object.insert(QStringLiteral("trans"), checkpointTransform(geometry));
        }
        object.insert(QStringLiteral("gate"), index);
        *result = object;
        return true;
    }
    QJsonObject transform;
    if (hasNative && previous.value(QStringLiteral("type")).toString() == gate.type) {
        transform = object.value(QStringLiteral("trans")).toObject();
        QJsonArray rawPosition, rawRotation, rawScale;
        if (!numericArray(transform.value(QStringLiteral("pos")), 3, &rawPosition) ||
            !numericArray(transform.value(QStringLiteral("rot")), 4, &rawRotation, 1000000) ||
            !numericArray(transform.value(QStringLiteral("scale")), 3, &rawScale, 1000000))
            return fail(error, QStringLiteral("原生组件附加数据损坏，无法安全导出。"));
        const double oldHeight = previous.value(QStringLiteral("height")).toDouble();
        const double oldWidth = previous.value(QStringLiteral("width")).toDouble();
        if (!(oldHeight > 0 && oldWidth > 0))
            return fail(error, QStringLiteral("原生组件的尺寸基准无效。"));
        const bool changedWidth = !same(gate.width, oldWidth);
        const bool changedHeight = !same(gate.height, oldHeight);
        const bool changedYaw = !same(gate.yaw, previous.value(QStringLiteral("yaw")).toDouble());
        const bool changedPosition =
            !same(gate.position.x(), previous.value(QStringLiteral("x")).toDouble()) ||
            !same(gate.position.y(), previous.value(QStringLiteral("z")).toDouble()) ||
            !same(gate.elevation, previous.value(QStringLiteral("elevation")).toDouble());
        QQuaternion orientation = quaternion(rawRotation);
        if (changedYaw) {
            orientation = QQuaternion::fromAxisAndAngle(
                              0, 1, 0,
                              static_cast<float>(
                                  gate.yaw - previous.value(QStringLiteral("yaw")).toDouble())) *
                          orientation;
            transform.insert(QStringLiteral("rot"), rotationArray(orientation));
        }
        if (changedWidth || changedHeight) {
            rawScale[0] = qRound(rawScale[0].toDouble() * gate.width / oldWidth);
            rawScale[1] = qRound(rawScale[1].toDouble() * gate.height / oldHeight);
            transform.insert(QStringLiteral("scale"), rawScale);
        }
        if (changedPosition || changedHeight || changedYaw) {
            const auto offset =
                orientation.rotatedVector(QVector3D(0, static_cast<float>(gate.height / 2), 0));
            transform.insert(QStringLiteral("pos"),
                             QJsonArray{qRound((gate.position.x() - offset.x()) * 100),
                                        qRound((gate.elevation - offset.y()) * 100),
                                        qRound((gate.position.y() - offset.z()) * 100)});
        }
    } else {
        const QQuaternion orientation =
            QQuaternion::fromAxisAndAngle(0, 1, 0, static_cast<float>(gate.yaw));
        const bool flag = gate.type == QStringLiteral("flag");
        transform = {
            {QStringLiteral("pos"), QJsonArray{qRound(gate.position.x() * 100),
                                               qRound((gate.elevation - gate.height / 2) * 100),
                                               qRound(gate.position.y() * 100)}},
            {QStringLiteral("rot"), rotationArray(orientation)},
            {QStringLiteral("scale"),
             flag ? QJsonArray{100, 100, 100}
                  : QJsonArray{qRound(gate.width / NominalGateAperture * 100),
                               qRound(gate.height / NominalGateAperture * 100), 100}}};
    }
    object.insert(QStringLiteral("prefab"), prefab);
    object.insert(QStringLiteral("trans"), transform);
    object.insert(QStringLiteral("gate"), index);
    // Some native courses have separate start and finish semantics. Preserve
    // these flags unless the user actually changed this component's type.
    if (!hasNative || previous.value(QStringLiteral("type")).toString() != gate.type) {
        object.insert(QStringLiteral("start"), isStart);
        object.insert(QStringLiteral("finish"), isStart);
    }
    *result = object;
    return true;
}
} // namespace

QVector<NativeBlockColor> nativeBlockColors() {
    return {{2218, QStringLiteral("黑色"), QColor("#0d0d0d")},
            {2219, QStringLiteral("白色"), QColor("#e6e6e6")},
            {2220, QStringLiteral("红色"), QColor("#e60000")},
            {2221, QStringLiteral("橙色"), QColor("#e67300")},
            {2222, QStringLiteral("黄色"), QColor("#e6e600")},
            {2223, QStringLiteral("绿色"), QColor("#00cc00")},
            {2224, QStringLiteral("青色"), QColor("#00d9d9")},
            {2225, QStringLiteral("蓝色"), QColor("#0000e6")},
            {2226, QStringLiteral("紫色"), QColor("#8c00e6")},
            {2227, QStringLiteral("粉色"), QColor("#f280bf")},
            {2228, QStringLiteral("玫红"), QColor("#e61a66")},
            {2229, QStringLiteral("浅蓝"), QColor("#0566e6")},
            {2230, QStringLiteral("深绿"), QColor("#006619")}};
}

NativeBlockColor nearestNativeBlockColor(const QColor &color) {
    const auto palette = nativeBlockColors();
    auto best = palette.first();
    int distance = std::numeric_limits<int>::max();
    const QColor source = color.isValid() ? color : best.color;
    for (const auto &entry : palette) {
        const int r = source.red() - entry.color.red(), g = source.green() - entry.color.green(),
                  b = source.blue() - entry.color.blue();
        const int candidate = r * r + g * g + b * b;
        if (candidate < distance) {
            distance = candidate;
            best = entry;
        }
    }
    return best;
}

bool exportVelocidrone(const QString &path, const TrackDocument &document, QString *error) {
    if (error)
        error->clear();
    if (document.gates.isEmpty() || document.gates.size() > MaxGates)
        return fail(error, QStringLiteral("原生赛道需要 1 至 %1 个门序组件。 ").arg(MaxGates));
    const int scene = sceneryId(document);
    if (scene <= 0)
        return fail(error, QStringLiteral("此场景尚无经过核对的 VelociDrone 编号，请选择 Football "
                                          "Stadium 或 Empty Scene Day。"));

    if (!document.takeoff.enabled)
        return fail(error,
                    QStringLiteral("请先配置独立起飞架的位置与朝向；起终点门不能代替原生起飞架。"));
    const auto &takeoff = document.takeoff;
    if (!std::isfinite(takeoff.position.x()) || !std::isfinite(takeoff.position.y()) ||
        !std::isfinite(takeoff.elevation) || !std::isfinite(takeoff.yaw) ||
        std::abs(takeoff.position.x()) > 1000000 || std::abs(takeoff.position.y()) > 1000000 ||
        std::abs(takeoff.elevation) > 1000000 || std::abs(takeoff.yaw) >= 1000000000)
        return fail(error, QStringLiteral("起飞架的基点坐标或朝向无效。"));
    QHash<QString, const Gate *> physical;
    for (const auto &gate : document.gates) {
        if (gate.id.isEmpty() || physical.contains(gate.id) || !finiteGate(gate))
            return fail(error, QStringLiteral("实体组件的ID重复、为空，或坐标/尺寸无效。"));
        physical.insert(gate.id, &gate);
    }
    const auto steps = effectiveRouteSteps(document);
    QVector<RouteStep> passes;
    QHash<QString, int> uses;
    QSet<QString> ids;
    for (const auto &step : steps) {
        if (step.id.isEmpty() || ids.contains(step.id))
            return fail(error, QStringLiteral("飞行步骤的ID为空或重复。"));
        ids.insert(step.id);
        if (step.kind == QStringLiteral("guide")) {
            if (!routeStepGeometry(document, step).valid)
                return fail(error, QStringLiteral("飞行引导点引用或坐标无效。"));
            continue;
        }
        if (step.kind != QStringLiteral("pass") || !physical.contains(step.gateId) ||
            !stepGeometry(*physical.value(step.gateId), step).valid)
            return fail(error, QStringLiteral("穿越步骤引用、面或窗口索引无效。"));
        passes.append(step);
        ++uses[step.gateId];
    }
    if (passes.isEmpty() || passes.size() > MaxGates)
        return fail(
            error,
            QStringLiteral("原生赛道需要1至%1个实际穿越步骤；引导点不计入门序。").arg(MaxGates));
    // Legacy native gate prefabs combine geometry and checkpoint in one object.
    // Until their physical and trigger geometry is calibrated, rerouting these
    // would duplicate solids or invent an unverified barrier representation.
    for (const auto &gate : document.gates)
        if (!compositeFrame(gate) && gate.type != QStringLiteral("pillar") &&
            uses.value(gate.id) != 1)
            return fail(
                error,
                QStringLiteral("旧原生模型“%1”需要保持一次原始穿越。请改用方梁组件后配置多次穿越。")
                    .arg(gate.label));
    QJsonObject layout = document.extra.value(NativeRoot).toObject();
    const auto originalGates = layout.value(QStringLiteral("gates")).toArray();
    QVector<int> originalOrder;
    for (qsizetype i = 0; i < originalGates.size(); ++i)
        originalOrder.append(static_cast<int>(i));
    std::stable_sort(originalOrder.begin(), originalOrder.end(), [&](int a, int b) {
        return originalGates[a].toObject().value(QStringLiteral("gate")).toInt(a) <
               originalGates[b].toObject().value(QStringLiteral("gate")).toInt(b);
    });
    bool preserveFlags = passes.size() == originalGates.size();
    if (preserveFlags)
        for (qsizetype i = 0; i < passes.size(); ++i) {
            const int sourceIndex = passes[i].extra.value(NativeArrayIndex).toInt(-1);
            if (sourceIndex != originalOrder[i] || passes[i].extra.value(NativeObject).toObject() !=
                                                       originalGates[sourceIndex].toObject()) {
                preserveFlags = false;
                break;
            }
        }
    QJsonArray gates;
    for (qsizetype i = 0; i < passes.size(); ++i) {
        const auto &step = passes[i];
        const auto &gate = *physical.value(step.gateId);
        const auto geometry = stepGeometry(gate, step);
        QJsonObject object = step.extra.value(NativeObject).toObject();
        if (compositeFrame(gate) || gate.type == QStringLiteral("pillar")) {
            if (geometry.openingWidth < .02 || geometry.openingHeight < .02 ||
                (compositeFrame(gate) && gateFrameMetrics(gate).beamThickness < .01))
                return fail(error, QStringLiteral("穿越窗口或梁截面过小。"));
            if (object.value(QStringLiteral("prefab")).toInt() != 88 ||
                step.extra.value(NativeStepGeometry).toObject() != geometryBaseline(geometry))
                object.insert(QStringLiteral("trans"), checkpointTransform(geometry));
            object.insert(QStringLiteral("prefab"), 88);
        } else {
            if (step.face != QStringLiteral("front") || step.reverse || step.column != 0 ||
                step.level != 0)
                return fail(error, QStringLiteral(
                                       "旧原生模型的任意面/反向穿越尚未校准，请先使用方梁结构。"));
            if (!makeNativeObject(gate, static_cast<int>(i), i == 0, &object, error))
                return false;
        }
        if (!validNativeTransform(object))
            return fail(error, QStringLiteral("检查点的原生附加变换数据无效。"));
        if (preserveFlags)
            object.insert(QStringLiteral("gate"),
                          step.extra.value(NativeObject).toObject().value(QStringLiteral("gate")));
        else
            object.insert(QStringLiteral("gate"), static_cast<int>(i));
        if (!preserveFlags) {
            object.insert(QStringLiteral("start"), i == 0);
            object.insert(QStringLiteral("finish"), i == 0);
        }
        gates.append(object);
    }
    if (preserveFlags) {
        const auto ordered = gates;
        for (qsizetype i = 0; i < passes.size(); ++i)
            gates[originalOrder[i]] = ordered[i];
    }
    layout.insert(QStringLiteral("gates"), gates);
    QSet<int> owned;
    for (const auto &value : document.extra.value(OwnedPillarBarriers).toArray())
        owned.insert(value.toInt(-1));
    for (const auto &value : document.extra.value(OwnedFrameBarriers).toArray())
        owned.insert(value.toInt(-1));
    const auto nativeBarriers = layout.value(QStringLiteral("barriers")).toArray();
    bool unchangedPhysical = true;
    int ownedCount = 0;
    for (const auto &gate : document.gates) {
        if (gate.type == QStringLiteral("pillar")) {
            unchangedPhysical = unchangedPhysical && unchangedPillar(gate) &&
                                !gate.extra.value(PillarBarrier).toObject().isEmpty();
            ++ownedCount;
        } else if (compositeFrame(gate)) {
            unchangedPhysical = unchangedPhysical && unchangedFrame(gate);
            ownedCount += static_cast<int>(gateFrameBeams(gate).size());
        }
    }
    const auto grid = startGridObject(takeoff);
    if (!validNativeTransform(grid) || (grid.value(QStringLiteral("prefab")).toInt() != 422 &&
                                        grid.value(QStringLiteral("prefab")).toInt() != 90))
        return fail(error, QStringLiteral("起飞架的原生附加数据无效，请重新配置起飞架。"));
    const bool unchangedGrid = !takeoff.extra.value(NativeTakeoffObject).toObject().isEmpty() &&
                               takeoff.extra.value(NativeTakeoffObject).toObject() == grid &&
                               document.extra.value(OwnedStartGrids).toArray().size() == 1;
    QJsonArray barriers;
    if (unchangedPhysical && ownedCount == owned.size() && unchangedGrid) {
        barriers = nativeBarriers;
    } else {
        for (qsizetype i = 0; i < nativeBarriers.size(); ++i)
            if (!owned.contains(static_cast<int>(i)) &&
                !startGridPrefab(
                    nativeBarriers[i].toObject().value(QStringLiteral("prefab")).toInt()))
                barriers.append(nativeBarriers[i]);
        for (const auto &gate : document.gates) {
            if (gate.type == QStringLiteral("pillar"))
                barriers.append(pillarBarrierObject(gate));
            else if (compositeFrame(gate))
                for (const auto &bar : frameBarrierObjects(gate))
                    barriers.append(bar);
        }
        barriers.append(grid);
    }
    layout.insert(QStringLiteral("barriers"), barriers);
    QString name = document.name;
    name.replace(QLatin1Char('\r'), QLatin1Char(' '));
    name.replace(QLatin1Char('\n'), QLatin1Char(' '));
    name.replace(QChar(0), QLatin1Char(' '));
    if (name.trimmed().isEmpty())
        name = QStringLiteral("OpenTrack Studio");
    QByteArray trailer("\n0\n0");
    if (document.extra.contains(NativeTrailer)) {
        const auto decodedTrailer = QByteArray::fromBase64Encoding(
            document.extra.value(NativeTrailer).toString().toLatin1(),
            QByteArray::AbortOnBase64DecodingErrors);
        if (!decodedTrailer || decodedTrailer.decoded.size() > 65536)
            return fail(error, QStringLiteral("原生赛道尾部附加数据无效。"));
        trailer = decodedTrailer.decoded;
    }
    const QByteArray plain = QByteArray::number(scene) + '\n' + name.toUtf8() + '\n' +
                             QJsonDocument(layout).toJson(QJsonDocument::Compact) + trailer;
    if (plain.size() > MaxTrackBytes)
        return fail(error, QStringLiteral("赛道数据超过 32 MB 限制。"));
    QByteArray encrypted;
    if (!crypt(plain, true, &encrypted, error))
        return false;
    QSaveFile destination(path);
    const QByteArray encoded = encrypted.toBase64();
    if (!destination.open(QIODevice::WriteOnly) || destination.write(encoded) != encoded.size() ||
        !destination.commit())
        return fail(error, QStringLiteral("无法保存原生赛道：%1").arg(destination.errorString()));
    return true;
}

bool importVelocidrone(const QString &path, TrackDocument *document, QString *error) {
    if (error)
        error->clear();
    if (!document)
        return fail(error, QStringLiteral("没有可接收导入内容的工程。"));
    QFile source(path);
    if (!source.open(QIODevice::ReadOnly))
        return fail(error, QStringLiteral("无法打开赛道文件：%1").arg(source.errorString()));
    if (source.size() <= 0 || source.size() > MaxTrackBytes * 2)
        return fail(error, QStringLiteral("赛道文件为空或超过大小限制。"));
    QByteArray encoded = source.readAll();
    encoded.replace("\r", "");
    encoded.replace("\n", "");
    encoded.replace(" ", "");
    encoded.replace("\t", "");
    const auto decoded =
        QByteArray::fromBase64Encoding(encoded, QByteArray::AbortOnBase64DecodingErrors);
    if (!decoded || decoded.decoded.isEmpty() || decoded.decoded.size() % 16 != 0 ||
        decoded.decoded.size() > MaxTrackBytes + 16)
        return fail(error,
                    QStringLiteral("不是有效的原生 .trk 文件：Base64 数据或 AES 区块长度错误。"));
    QByteArray plain;
    if (!crypt(decoded.decoded, false, &plain, error))
        return false;
    if (plain.startsWith("\xEF\xBB\xBF"))
        plain.remove(0, 3);
    const qsizetype firstLine = plain.indexOf('\n');
    const qsizetype secondLine = firstLine < 0 ? -1 : plain.indexOf('\n', firstLine + 1);
    const qsizetype jsonStart = secondLine < 0 ? -1 : plain.indexOf('{', secondLine + 1);
    const qsizetype jsonEnd = plain.lastIndexOf('}');
    if (firstLine <= 0 || secondLine <= firstLine || jsonStart < 0 || jsonEnd < jsonStart)
        return fail(error, QStringLiteral("原生赛道缺少场景、名称或 JSON 内容。"));
    bool validScene = false;
    const int scene = plain.left(firstLine).trimmed().toInt(&validScene);
    if (!validScene || scene <= 0)
        return fail(error, QStringLiteral("原生赛道场景编号无效。"));
    QJsonParseError parseError;
    const auto parsed =
        QJsonDocument::fromJson(plain.mid(jsonStart, jsonEnd - jsonStart + 1), &parseError);
    if (parseError.error != QJsonParseError::NoError || !parsed.isObject())
        return fail(error, QStringLiteral("原生赛道 JSON 无效：%1").arg(parseError.errorString()));
    const auto layout = parsed.object();
    const auto array = layout.value(QStringLiteral("gates")).toArray();
    if (!layout.value(QStringLiteral("gates")).isArray() || array.isEmpty() ||
        array.size() > MaxGates)
        return fail(error, QStringLiteral("原生赛道缺少有效门序。"));
    TrackDocument imported;
    imported.name =
        QString::fromUtf8(plain.mid(firstLine + 1, secondLine - firstLine - 1)).trimmed();
    imported.scenery = sceneryName(scene);
    imported.extra.insert(NativeRoot, layout);
    imported.extra.insert(NativeScene, scene);
    imported.extra.insert(NativeTrailer, QString::fromLatin1(plain.mid(jsonEnd + 1).toBase64()));
    struct OrderedGate {
        int index;
        Gate gate;
    };
    QVector<OrderedGate> ordered;
    for (qsizetype i = 0; i < array.size(); ++i) {
        const auto object = array[i].toObject();
        const auto transform = object.value(QStringLiteral("trans")).toObject();
        QJsonArray position, rotation, scale;
        if (!object.value(QStringLiteral("prefab")).isDouble() ||
            object.value(QStringLiteral("prefab")).toInt() <= 0 ||
            !numericArray(transform.value(QStringLiteral("pos")), 3, &position) ||
            !numericArray(transform.value(QStringLiteral("rot")), 4, &rotation, 1000000) ||
            !numericArray(transform.value(QStringLiteral("scale")), 3, &scale, 1000000) ||
            scale[0].toDouble() <= 0 || scale[1].toDouble() <= 0 || scale[2].toDouble() <= 0)
            return fail(error, QStringLiteral("原生赛道第 %1 个组件的变换数据无效。").arg(i + 1));
        double norm = 0;
        for (const auto &value : rotation)
            norm += value.toDouble() * value.toDouble();
        if (norm < 1)
            return fail(error, QStringLiteral("原生赛道含有无效的零旋转四元数。"));
        Gate gate;
        gate.id = newGateId();
        const int prefab = object.value(QStringLiteral("prefab")).toInt();
        if (object.value(QStringLiteral("start")).toBool())
            gate.type = QStringLiteral("start");
        else if (prefab == 170 || prefab == 2231 || prefab == 2232)
            gate.type = QStringLiteral("flag");
        else if (prefab == 286)
            gate.type = QStringLiteral("dive");
        else
            gate.type = QStringLiteral("standard");
        gate.width = NominalGateAperture * scale[0].toDouble() / 100.0;
        gate.height = NominalGateAperture * scale[1].toDouble() / 100.0;
        const auto orientation = quaternion(rotation);
        const auto offset =
            orientation.rotatedVector(QVector3D(0, static_cast<float>(gate.height / 2), 0));
        gate.position = QPointF(position[0].toDouble() / 100 + offset.x(),
                                position[2].toDouble() / 100 + offset.z());
        gate.elevation = position[1].toDouble() / 100 + offset.y();
        const auto forward = orientation.rotatedVector(QVector3D(0, 0, 1));
        gate.yaw = std::atan2(forward.x(), forward.z()) * 180.0 / 3.14159265358979323846;
        gate.label = QStringLiteral("原生组件 #%1").arg(prefab);
        gate.extra.insert(NativeObject, object);
        gate.extra.insert(NativeArrayIndex, static_cast<int>(i));
        gate.extra.insert(NativeBaseline, baseline(gate));
        const int order = object.value(QStringLiteral("gate")).toInt(static_cast<int>(i));
        ordered.append({order, std::move(gate)});
    }
    std::stable_sort(ordered.begin(), ordered.end(),
                     [](const OrderedGate &a, const OrderedGate &b) { return a.index < b.index; });
    for (const auto &entry : ordered)
        imported.gates.append(entry.gate);
    restorePhysicalRoute(&imported, layout.value(QStringLiteral("barriers")).toArray());
    restoreTakeoff(&imported, layout.value(QStringLiteral("barriers")).toArray());
    *document = std::move(imported);
    return true;
}
} // namespace ot
