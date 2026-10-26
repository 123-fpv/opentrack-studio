#include "velocidrone.h"
#include "document.h"

#include <QFile>
#include <QHash>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonParseError>
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
constexpr double PillarCheckpointSize = 2.4;

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

QJsonObject pillarCheckpointTransform(const Gate &gate) {
    const auto rotation = QQuaternion::fromAxisAndAngle(0, 1, 0, static_cast<float>(gate.yaw - 90));
    // Native helper bounds: local Y [-1,1], Z [-2,0]. Its centre is not its origin.
    const auto offset =
        rotation.rotatedVector(QVector3D(0, 0, static_cast<float>(-PillarCheckpointSize / 2)));
    const QPointF passage = gatePassagePosition(gate);
    return {{QStringLiteral("pos"), QJsonArray{qRound((passage.x() - offset.x()) * 100),
                                               qRound((gate.elevation - offset.y()) * 100),
                                               qRound((passage.y() - offset.z()) * 100)}},
            {QStringLiteral("rot"), rotationArray(rotation)},
            {QStringLiteral("scale"), QJsonArray{100, qRound(gate.height * 50), 120}}};
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

// Reconstruct only the explicitly generated pairing convention; do not put
// speculative editor fields into native JSON. Ownership is project-only data.
void restorePillars(TrackDocument *document, const QJsonArray &barriers) {
    struct Candidate {
        Gate gate;
        QJsonObject raw;
        int index;
    };
    QHash<QString, QVector<Candidate>> candidates;
    for (qsizetype i = 0; i < barriers.size(); ++i) {
        const auto object = barriers[i].toObject();
        const int prefab = object.value(QStringLiteral("prefab")).toInt();
        if (prefab < 2218 || prefab > 2230)
            continue;
        const auto transform = object.value(QStringLiteral("trans")).toObject();
        QJsonArray pos, rot, scale;
        if (!numericArray(transform.value(QStringLiteral("pos")), 3, &pos) ||
            !numericArray(transform.value(QStringLiteral("rot")), 4, &rot, 1000000) ||
            !numericArray(transform.value(QStringLiteral("scale")), 3, &scale, 1000000) ||
            scale[0].toDouble() < 2 || scale[0].toDouble() > 100000 || scale[1].toDouble() < 2 ||
            scale[1].toDouble() > 100000 || !same(scale[0].toDouble(), scale[2].toDouble()))
            continue;
        double norm = 0;
        for (const auto &value : rot)
            norm += value.toDouble() * value.toDouble();
        if (norm < 1)
            continue;
        const auto orientation = quaternion(rot);
        if (std::abs(orientation.x()) > .002f || std::abs(orientation.z()) > .002f)
            continue;
        const auto forward = orientation.rotatedVector(QVector3D(0, 0, 1));
        Gate pillar;
        pillar.type = QStringLiteral("pillar");
        for (const auto &entry : nativeBlockColors())
            if (entry.prefab == prefab) {
                setGateColor(pillar, entry.color);
                break;
            }
        pillar.position = {pos[0].toDouble() / 100, pos[2].toDouble() / 100};
        pillar.width = scale[0].toDouble() / 100;
        pillar.height = scale[1].toDouble() / 100;
        pillar.elevation = pos[1].toDouble() / 100 + pillar.height / 2;
        pillar.yaw = std::atan2(forward.x(), forward.z()) * 180.0 / 3.14159265358979323846;
        for (int side : {-1, 1}) {
            pillar.extra.insert(QStringLiteral("passSide"), side);
            candidates[pillarBucket(gatePassagePosition(pillar))].append(
                {pillar, object, static_cast<int>(i)});
        }
    }
    QSet<int> owned;
    for (auto &gate : document->gates) {
        if (gateFrameMetrics(gate).centerlineModel)
            continue;
        const auto checkpoint = gate.extra.value(NativeObject).toObject();
        if (checkpoint.value(QStringLiteral("prefab")).toInt() != PillarCheckpointPrefab)
            continue;
        if (checkpoint.value(QStringLiteral("start")).toBool() ||
            checkpoint.value(QStringLiteral("finish")).toBool())
            continue;
        const auto transform = checkpoint.value(QStringLiteral("trans")).toObject();
        const auto checkpointScale = transform.value(QStringLiteral("scale")).toArray();
        if (checkpointScale[0].toInt() != 100 || checkpointScale[2].toInt() != 120)
            continue;
        const auto pos = transform.value(QStringLiteral("pos")).toArray();
        const auto orientation = quaternion(transform.value(QStringLiteral("rot")).toArray());
        const auto centerOffset = orientation.rotatedVector(
            QVector3D(0, 0, static_cast<float>(-PillarCheckpointSize / 2)));
        const QPointF passage(pos[0].toDouble() / 100 + centerOffset.x(),
                              pos[2].toDouble() / 100 + centerOffset.z());
        const double altitude = pos[1].toDouble() / 100 + centerOffset.y();
        bool matched = false;
        for (int x = -3; x <= 3 && !matched; ++x) {
            for (int z = -3; z <= 3 && !matched; ++z) {
                const auto bucket =
                    candidates.constFind(pillarBucket(passage + QPointF(x * .25, z * .25)));
                if (bucket == candidates.cend())
                    continue;
                for (const auto &candidate : bucket.value()) {
                    if (owned.contains(candidate.index))
                        continue;
                    const auto expected = gatePassagePosition(candidate.gate);
                    const double tolerance = .025 + candidate.gate.width * .002;
                    if (std::hypot(expected.x() - passage.x(), expected.y() - passage.y()) >
                            tolerance ||
                        std::abs(candidate.gate.elevation - altitude) > .025 ||
                        std::abs(candidate.gate.height - checkpointScale[1].toDouble() / 50) > .011)
                        continue;
                    const auto desired = QQuaternion::fromAxisAndAngle(
                        0, 1, 0, static_cast<float>(candidate.gate.yaw - 90));
                    if (std::abs(QQuaternion::dotProduct(desired, orientation)) < .99999f)
                        continue;
                    Gate recovered = candidate.gate;
                    recovered.id = gate.id;
                    recovered.label = QStringLiteral("转向柱");
                    recovered.extra.insert(NativeObject, checkpoint);
                    recovered.extra.insert(PillarBarrier, candidate.raw);
                    recovered.extra.insert(NativeBaseline, baseline(recovered));
                    gate = std::move(recovered);
                    owned.insert(candidate.index);
                    matched = true;
                    break;
                }
            }
        }
    }
    QJsonArray indices;
    auto ordered = owned.values();
    std::sort(ordered.begin(), ordered.end());
    for (int index : ordered)
        indices.append(index);
    if (!indices.isEmpty())
        document->extra.insert(OwnedPillarBarriers, indices);
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

QJsonObject frameCheckpointTransform(const Gate &gate) {
    const auto frame = gateFrameMetrics(gate);
    const auto rotation = QQuaternion::fromAxisAndAngle(0, 1, 0, static_cast<float>(gate.yaw - 90));
    const auto offset =
        rotation.rotatedVector(QVector3D(0, 0, static_cast<float>(-frame.openingWidth / 2)));
    const QPointF passage = gatePassagePosition(gate);
    const double elevation = gatePassageElevation(gate);
    return {{QStringLiteral("pos"), QJsonArray{qRound((passage.x() - offset.x()) * 100),
                                               qRound((elevation - offset.y()) * 100),
                                               qRound((passage.y() - offset.z()) * 100)}},
            {QStringLiteral("rot"), rotationArray(rotation)},
            {QStringLiteral("scale"),
             QJsonArray{100, qRound(frame.openingHeight * 50), qRound(frame.openingWidth * 50)}}};
}

void restoreFrames(TrackDocument *document, const QJsonArray &barriers) {
    struct Block {
        int index;
        QJsonObject raw;
        QJsonArray pos, scale;
        QQuaternion rotation;
    };
    QHash<QString, QVector<Block>> blocks;
    for (qsizetype i = 0; i < barriers.size(); ++i) {
        const auto object = barriers[i].toObject();
        const int prefab = object.value(QStringLiteral("prefab")).toInt();
        if (prefab < 2218 || prefab > 2230)
            continue;
        const auto trans = object.value(QStringLiteral("trans")).toObject();
        QJsonArray pos, rot, scale;
        if (!numericArray(trans.value(QStringLiteral("pos")), 3, &pos) ||
            !numericArray(trans.value(QStringLiteral("rot")), 4, &rot, 1000000) ||
            !numericArray(trans.value(QStringLiteral("scale")), 3, &scale, 1000000))
            continue;
        double norm = 0;
        for (const auto &value : rot)
            norm += value.toDouble() * value.toDouble();
        if (norm < 1)
            continue;
        blocks[pillarBucket({pos[0].toDouble() / 100, pos[2].toDouble() / 100})].append(
            {static_cast<int>(i), object, pos, scale, quaternion(rot)});
    }
    QSet<int> owned;
    for (auto &gate : document->gates) {
        const auto object = gate.extra.value(NativeObject).toObject();
        if (object.value(QStringLiteral("prefab")).toInt() != 88)
            continue;
        const auto trans = object.value(QStringLiteral("trans")).toObject();
        const auto scale = trans.value(QStringLiteral("scale")).toArray();
        if (scale[0].toInt() != 100 || scale[1].toDouble() < 1 || scale[2].toDouble() < 1)
            continue;
        const double openingWidth = scale[2].toDouble() / 50,
                     openingHeight = scale[1].toDouble() / 50;
        const auto rotation = quaternion(trans.value(QStringLiteral("rot")).toArray());
        const auto normal = rotation.rotatedVector(QVector3D(1, 0, 0));
        if (std::abs(normal.y()) > .003f)
            continue;
        const auto pos = trans.value(QStringLiteral("pos")).toArray();
        const auto offset =
            rotation.rotatedVector(QVector3D(0, 0, static_cast<float>(-openingWidth / 2)));
        const QPointF passage(pos[0].toDouble() / 100 + offset.x(),
                              pos[2].toDouble() / 100 + offset.z());
        const double passageElevation = pos[1].toDouble() / 100 + offset.y();
        bool matched = false;
        // Match complete larger assemblies before accepting a four-bar face.
        const QStringList shapes{QStringLiteral("cube-stack"),
                                 QStringLiteral("cube"),
                                 QStringLiteral("triple"),
                                 QStringLiteral("double"),
                                 QStringLiteral("ladder"),
                                 object.value(QStringLiteral("start")).toBool()
                                     ? QStringLiteral("start")
                                     : QStringLiteral("standard")};
        for (const auto &shape : shapes) {
            if (matched)
                break;
            const auto model = makeGate(shape, openingWidth + .1, openingHeight + .1);
            const auto dimensions = gateFrameMetrics(model);
            for (int column = 0; column < dimensions.columns && !matched; ++column)
                for (int level = 0; level < dimensions.levels && !matched; ++level) {
                    Gate recovered = model;
                    recovered.id = gate.id;
                    recovered.position = {};
                    recovered.elevation = 0;
                    recovered.yaw =
                        std::atan2(normal.x(), normal.z()) * 180.0 / 3.14159265358979323846;
                    recovered.extra.insert(QStringLiteral("passColumn"), column);
                    recovered.extra.insert(QStringLiteral("passLevel"), level);
                    const auto relative = gatePassagePosition(recovered);
                    const auto relativeElevation = gatePassageElevation(recovered);
                    recovered.position = passage - relative;
                    recovered.elevation = passageElevation - relativeElevation;
                    for (const auto &color : nativeBlockColors()) {
                        setGateColor(recovered, color.color);
                        const auto expected = frameBarrierObjects(recovered);
                        QSet<int> selected;
                        QJsonArray originals;
                        for (const auto &value : expected) {
                            const auto wanted =
                                value.toObject().value(QStringLiteral("trans")).toObject();
                            const auto wantedPos = wanted.value(QStringLiteral("pos")).toArray();
                            const auto wantedScale =
                                wanted.value(QStringLiteral("scale")).toArray();
                            const auto wantedRotation =
                                quaternion(wanted.value(QStringLiteral("rot")).toArray());
                            const QPointF point(wantedPos[0].toDouble() / 100,
                                                wantedPos[2].toDouble() / 100);
                            bool found = false;
                            for (int x = -1; x <= 1 && !found; ++x)
                                for (int z = -1; z <= 1 && !found; ++z) {
                                    const auto bucket = blocks.constFind(
                                        pillarBucket(point + QPointF(x * .25, z * .25)));
                                    if (bucket == blocks.cend())
                                        continue;
                                    for (const auto &block : bucket.value()) {
                                        if (owned.contains(block.index) ||
                                            selected.contains(block.index) ||
                                            block.raw.value(QStringLiteral("prefab")).toInt() !=
                                                color.prefab ||
                                            std::abs(QQuaternion::dotProduct(
                                                block.rotation, wantedRotation)) < .99999f)
                                            continue;
                                        bool close = true;
                                        for (int axis = 0; axis < 3; ++axis)
                                            if (std::abs(block.pos[axis].toDouble() -
                                                         wantedPos[axis].toDouble()) > 3 ||
                                                std::abs(block.scale[axis].toDouble() -
                                                         wantedScale[axis].toDouble()) > 1)
                                                close = false;
                                        if (!close)
                                            continue;
                                        selected.insert(block.index);
                                        originals.append(block.raw);
                                        found = true;
                                        break;
                                    }
                                }
                            if (!found)
                                break;
                        }
                        if (originals.size() != expected.size())
                            continue;
                        // Two adjacent independent frames also contain a
                        // double-frame outline. Require one complete assembly
                        // per unpaired checkpoint in its cells, so a match
                        // cannot consume a neighbouring logical gate's rods.
                        if (expected.size() > 4) {
                            int checkpoints = 0;
                            for (const auto &other : document->gates) {
                                if (compositeFrame(other))
                                    continue;
                                const auto raw = other.extra.value(NativeObject).toObject();
                                if (raw.value(QStringLiteral("prefab")).toInt() != 88)
                                    continue;
                                const auto tr = raw.value(QStringLiteral("trans")).toObject();
                                const auto sc = tr.value(QStringLiteral("scale")).toArray();
                                if (sc[1] != scale[1] || sc[2] != scale[2])
                                    continue;
                                const auto qr =
                                    quaternion(tr.value(QStringLiteral("rot")).toArray());
                                if (std::abs(QQuaternion::dotProduct(qr, rotation)) < .99999f)
                                    continue;
                                const auto po = tr.value(QStringLiteral("pos")).toArray();
                                const auto off = qr.rotatedVector(
                                    QVector3D(0, 0, static_cast<float>(-openingWidth / 2)));
                                const QPointF point(po[0].toDouble() / 100 + off.x(),
                                                    po[2].toDouble() / 100 + off.z());
                                const double altitude = po[1].toDouble() / 100 + off.y();
                                bool cellMatch = false;
                                for (int c = 0; c < dimensions.columns && !cellMatch; ++c)
                                    for (int l = 0; l < dimensions.levels && !cellMatch; ++l) {
                                        auto probe = recovered;
                                        probe.extra.insert(QStringLiteral("passColumn"), c);
                                        probe.extra.insert(QStringLiteral("passLevel"), l);
                                        const auto wanted = gatePassagePosition(probe);
                                        cellMatch =
                                            std::hypot(point.x() - wanted.x(),
                                                       point.y() - wanted.y()) < .04 &&
                                            std::abs(altitude - gatePassageElevation(probe)) < .025;
                                    }
                                if (cellMatch)
                                    ++checkpoints;
                            }
                            bool enough = true;
                            if (checkpoints > 1)
                                for (const auto &bar : expected) {
                                    const auto tr =
                                        bar.toObject().value(QStringLiteral("trans")).toObject();
                                    const auto po = tr.value(QStringLiteral("pos")).toArray(),
                                               sc = tr.value(QStringLiteral("scale")).toArray();
                                    const auto qr =
                                        quaternion(tr.value(QStringLiteral("rot")).toArray());
                                    const QPointF point(po[0].toDouble() / 100,
                                                        po[2].toDouble() / 100);
                                    int copies = 0;
                                    for (int x = -1; x <= 1; ++x)
                                        for (int z = -1; z <= 1; ++z) {
                                            const auto bucket = blocks.constFind(
                                                pillarBucket(point + QPointF(x * .25, z * .25)));
                                            if (bucket == blocks.cend())
                                                continue;
                                            for (const auto &block : bucket.value()) {
                                                if (owned.contains(block.index) ||
                                                    block.raw.value(QStringLiteral("prefab"))
                                                            .toInt() != color.prefab ||
                                                    std::abs(QQuaternion::dotProduct(block.rotation,
                                                                                     qr)) < .99999f)
                                                    continue;
                                                bool close = true;
                                                for (int axis = 0; axis < 3; ++axis)
                                                    if (std::abs(block.pos[axis].toDouble() -
                                                                 po[axis].toDouble()) > 3 ||
                                                        std::abs(block.scale[axis].toDouble() -
                                                                 sc[axis].toDouble()) > 1)
                                                        close = false;
                                                if (close)
                                                    ++copies;
                                            }
                                        }
                                    if (copies < checkpoints) {
                                        enough = false;
                                        break;
                                    }
                                }
                            if (!enough)
                                continue;
                        }
                        recovered.label = gateTypeName(recovered.type);
                        recovered.extra.insert(NativeObject, object);
                        recovered.extra.insert(FrameBarriers, originals);
                        recovered.extra.insert(NativeBaseline, baseline(recovered));
                        gate = std::move(recovered);
                        owned.unite(selected);
                        matched = true;
                        break;
                    }
                }
        }
        Q_UNUSED(matched)
    }
    QJsonArray indices;
    auto ordered = owned.values();
    std::sort(ordered.begin(), ordered.end());
    for (int index : ordered)
        indices.append(index);
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
    if (compositeFrame(gate)) {
        const auto frame = gateFrameMetrics(gate);
        if (frame.openingWidth < .02 || frame.openingHeight < .02 || frame.beamThickness < .01)
            return fail(error, QStringLiteral("方梁门的净开口或梁截面无效，请增加中心线间距。"));
        object.insert(QStringLiteral("prefab"), 88);
        if (!unchangedFrame(gate))
            object.insert(QStringLiteral("trans"), frameCheckpointTransform(gate));
        object.insert(QStringLiteral("gate"), index);
        if (!hasNative || previous.value(QStringLiteral("type")).toString() != gate.type) {
            object.insert(QStringLiteral("start"), isStart);
            object.insert(QStringLiteral("finish"), isStart);
        }
        *result = object;
        return true;
    }
    if (gate.type == QStringLiteral("pillar")) {
        if (isStart)
            return fail(error, QStringLiteral("转向柱是实体障碍，不能作为起终点，请添加标准门。"));
        object.insert(QStringLiteral("prefab"), PillarCheckpointPrefab);
        if (!unchangedPillar(gate))
            object.insert(QStringLiteral("trans"), pillarCheckpointTransform(gate));
        object.insert(QStringLiteral("gate"), index);
        object.insert(QStringLiteral("start"), false);
        object.insert(QStringLiteral("finish"), false);
        *result = object;
        return true;
    }
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
    int startingIndex = -1;
    for (qsizetype i = 0; i < document.gates.size(); ++i) {
        const auto &candidate = document.gates[i];
        const bool nativeStart =
            compositeFrame(candidate) &&
            candidate.extra.value(NativeBaseline)
                    .toObject()
                    .value(QStringLiteral("type"))
                    .toString() == candidate.type &&
            candidate.extra.value(NativeObject).toObject().value(QStringLiteral("start")).toBool();
        if (candidate.type == QStringLiteral("start") || nativeStart) {
            if (startingIndex >= 0)
                return fail(error, QStringLiteral("只能有一个起终点门。"));
            startingIndex = static_cast<int>(i);
        }
    }
    if (startingIndex < 0) {
        for (qsizetype i = 0; i < document.gates.size(); ++i)
            if (document.gates[i].type != QStringLiteral("pillar")) {
                startingIndex = static_cast<int>(i);
                break;
            }
    }
    if (startingIndex < 0)
        return fail(error, QStringLiteral("赛道不能只有转向柱，请添加一个标准门作为起终点。"));
    QJsonArray gates;
    for (qsizetype i = 0; i < document.gates.size(); ++i) {
        QJsonObject object;
        if (!makeNativeObject(document.gates[i], static_cast<int>(i), i == startingIndex, &object,
                              error))
            return false;
        gates.append(object);
    }
    QJsonObject layout = document.extra.value(NativeRoot).toObject();
    layout.remove(QStringLiteral("meta"));
    layout.insert(QStringLiteral("gates"), gates);
    QSet<int> owned;
    for (const auto &value : document.extra.value(OwnedPillarBarriers).toArray())
        owned.insert(value.toInt(-1));
    for (const auto &value : document.extra.value(OwnedFrameBarriers).toArray())
        owned.insert(value.toInt(-1));
    QJsonArray barriers;
    const auto nativeBarriers = layout.value(QStringLiteral("barriers")).toArray();
    for (qsizetype i = 0; i < nativeBarriers.size(); ++i)
        if (!owned.contains(static_cast<int>(i)))
            barriers.append(nativeBarriers[i]);
    for (const auto &gate : document.gates) {
        if (gate.type == QStringLiteral("pillar"))
            barriers.append(pillarBarrierObject(gate));
        else if (compositeFrame(gate))
            for (const auto &bar : frameBarrierObjects(gate))
                barriers.append(bar);
    }
    layout.insert(QStringLiteral("barriers"), barriers);
    QString name = document.name;
    name.replace(QLatin1Char('\r'), QLatin1Char(' '));
    name.replace(QLatin1Char('\n'), QLatin1Char(' '));
    name.replace(QChar(0), QLatin1Char(' '));
    if (name.trimmed().isEmpty())
        name = QStringLiteral("OpenTrack Studio");
    const QByteArray plain = QByteArray::number(scene) + '\n' + name.toUtf8() + '\n' +
                             QJsonDocument(layout).toJson(QJsonDocument::Compact) + "\n0\n0";
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
        gate.extra.insert(NativeBaseline, baseline(gate));
        const int order = object.value(QStringLiteral("gate")).toInt(static_cast<int>(i));
        ordered.append({order, std::move(gate)});
    }
    std::stable_sort(ordered.begin(), ordered.end(),
                     [](const OrderedGate &a, const OrderedGate &b) { return a.index < b.index; });
    for (const auto &entry : ordered)
        imported.gates.append(entry.gate);
    restoreFrames(&imported, layout.value(QStringLiteral("barriers")).toArray());
    restorePillars(&imported, layout.value(QStringLiteral("barriers")).toArray());
    *document = std::move(imported);
    return true;
}
} // namespace ot
