#include "recognizer.h"

#include <QJsonArray>
#include <QJsonObject>
#include <QtMath>
#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <vector>

namespace ot {
namespace {
constexpr int MaxWorkingDimension = 1800;
constexpr int MaxRawComponents = 2048;
constexpr int MaxGates = 512;
constexpr double Pi = 3.14159265358979323846;

struct Component {
    int left = std::numeric_limits<int>::max();
    int top = std::numeric_limits<int>::max();
    int right = -1;
    int bottom = -1;
    int count = 0;
    double sumX = 0;
    double sumY = 0;

    int width() const {
        return right - left + 1;
    }
    int height() const {
        return bottom - top + 1;
    }
    QPointF center() const {
        return {sumX / count, sumY / count};
    }
    void add(int x, int y) {
        left = std::min(left, x);
        right = std::max(right, x);
        top = std::min(top, y);
        bottom = std::max(bottom, y);
        ++count;
        sumX += x;
        sumY += y;
    }
    void merge(const Component &other) {
        left = std::min(left, other.left);
        right = std::max(right, other.right);
        top = std::min(top, other.top);
        bottom = std::max(bottom, other.bottom);
        count += other.count;
        sumX += other.sumX;
        sumY += other.sumY;
    }
};

double pixelHue(int r, int g, int b, int maximum, int minimum) {
    const int difference = maximum - minimum;
    if (difference == 0)
        return 0;
    double hue = 0;
    if (maximum == r)
        hue = 60.0 * (g - b) / difference;
    else if (maximum == g)
        hue = 120.0 + 60.0 * (b - r) / difference;
    else
        hue = 240.0 + 60.0 * (r - g) / difference;
    return hue < 0 ? hue + 360.0 : hue;
}

double squaredDistance(const QPointF &a, const QPointF &b) {
    const auto d = a - b;
    return d.x() * d.x() + d.y() * d.y();
}

std::vector<int> suggestedOrder(const QVector<Gate> &gates) {
    std::vector<int> order;
    if (gates.isEmpty())
        return order;
    order.reserve(gates.size());
    std::vector<bool> used(static_cast<size_t>(gates.size()), false);
    int current = 0;
    for (int i = 1; i < gates.size(); ++i) {
        const auto a = gates[i].position;
        const auto b = gates[current].position;
        if (a.x() + a.y() < b.x() + b.y())
            current = i;
    }
    for (int step = 0; step < gates.size(); ++step) {
        order.push_back(current);
        used[static_cast<size_t>(current)] = true;
        double bestDistance = std::numeric_limits<double>::infinity();
        int next = -1;
        for (int i = 0; i < gates.size(); ++i) {
            if (used[static_cast<size_t>(i)])
                continue;
            const double d = squaredDistance(gates[current].position, gates[i].position);
            if (d < bestDistance) {
                bestDistance = d;
                next = i;
            }
        }
        if (next < 0)
            break;
        current = next;
    }

    // A bounded planar travelling-salesman heuristic, not a reading of the
    // printed racing line. Intentional crossovers must be corrected by a user.
    const int n = static_cast<int>(order.size());
    if (n >= 4 && n <= 256) {
        for (int pass = 0; pass < 6; ++pass) {
            bool changed = false;
            for (int i = 0; i < n - 2; ++i) {
                for (int j = i + 2; j < n; ++j) {
                    if (i == 0 && j == n - 1)
                        continue;
                    const auto &a = gates[order[i]].position;
                    const auto &b = gates[order[(i + 1) % n]].position;
                    const auto &c = gates[order[j]].position;
                    const auto &d = gates[order[(j + 1) % n]].position;
                    const double before =
                        std::sqrt(squaredDistance(a, b)) + std::sqrt(squaredDistance(c, d));
                    const double after =
                        std::sqrt(squaredDistance(a, c)) + std::sqrt(squaredDistance(b, d));
                    if (after + 0.0001 < before) {
                        std::reverse(order.begin() + i + 1, order.begin() + j + 1);
                        changed = true;
                    }
                }
            }
            if (!changed)
                break;
        }
    }
    return order;
}
} // namespace

RecognitionResult recognizeTrack(const QImage &input, const RecognitionOptions &options) {
    RecognitionResult result;
    auto &document = result.document;
    document.name = QStringLiteral("图片生成的赛道");
    document.gates.clear();
    const bool validScale = std::isfinite(options.metersPerPixel) &&
                            options.metersPerPixel >= 0.00001 && options.metersPerPixel <= 1000.0;
    document.metersPerPixel = validScale ? options.metersPerPixel : 0.05;
    if (!validScale)
        result.warnings << QStringLiteral("无效的比例尺已恢复为 0.05 米/像素。");
    if (input.isNull() || input.width() < 8 || input.height() < 8) {
        result.warnings << QStringLiteral("图片为空或尺寸过小，未生成门架。");
        return result;
    }
    if (input.width() > 30000 || input.height() > 30000 ||
        qint64(input.width()) * input.height() > 100000000) {
        result.warnings << QStringLiteral("图片尺寸过大，请先裁剪赛道区域或缩小至 1 亿像素以内。");
        return result;
    }

    document.background = input;
    document.showBackground = true;
    document.backgroundOpacity = 0.35;
    QImage working = input;
    if (std::max(input.width(), input.height()) > MaxWorkingDimension)
        working = input.scaled(MaxWorkingDimension, MaxWorkingDimension, Qt::KeepAspectRatio,
                               Qt::SmoothTransformation);
    working = working.convertToFormat(QImage::Format_ARGB32);
    if (working.isNull()) {
        result.warnings << QStringLiteral("图片处理内存不足，未生成门架；请先缩小图片。");
        return result;
    }
    const int width = working.width();
    const int height = working.height();
    const double scaleX = double(input.width()) / width;
    const double scaleY = double(input.height()) / height;
    const int minimumPixels = std::clamp(options.minComponentPixels, 3, 500);

    QColor reference = options.gateColor;
    if (!reference.isValid() || reference.hsvSaturationF() < 0.15) {
        reference = QColor("#3688cf");
        result.warnings << QStringLiteral("识别颜色需要有明显色相，已恢复为蓝色门架。");
    }
    const double referenceHue = reference.hsvHueF() * 360.0;
    std::vector<unsigned char> mask(static_cast<size_t>(width) * height, 0);
    for (int y = 0; y < height; ++y) {
        const auto *line = reinterpret_cast<const QRgb *>(working.constScanLine(y));
        for (int x = 0; x < width; ++x) {
            const QRgb pixel = line[x];
            if (qAlpha(pixel) < 100)
                continue;
            const int r = qRed(pixel), g = qGreen(pixel), b = qBlue(pixel);
            const int maximum = std::max({r, g, b});
            const int minimum = std::min({r, g, b});
            if (maximum < 45 || maximum - minimum < 28 ||
                double(maximum - minimum) / maximum < 0.28)
                continue;
            const double hue = pixelHue(r, g, b, maximum, minimum);
            double difference = std::abs(hue - referenceHue);
            difference = std::min(difference, 360.0 - difference);
            if (difference <= 29.0)
                mask[static_cast<size_t>(y) * width + x] = 1;
        }
    }

    const int maximumLength = std::min(240, std::max(36, int(std::min(width, height) * 0.24)));
    std::vector<Component> components;
    components.reserve(256);
    std::vector<int> frontier;
    frontier.reserve(4096);
    bool componentLimit = false;
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            const int start = y * width + x;
            if (!mask[static_cast<size_t>(start)])
                continue;
            Component component;
            frontier.clear();
            frontier.push_back(start);
            mask[static_cast<size_t>(start)] = 0;
            for (size_t cursor = 0; cursor < frontier.size(); ++cursor) {
                const int at = frontier[cursor];
                const int px = at % width, py = at / width;
                component.add(px, py);
                for (int dy = -1; dy <= 1; ++dy) {
                    const int ny = py + dy;
                    if (ny < 0 || ny >= height)
                        continue;
                    for (int dx = -1; dx <= 1; ++dx) {
                        const int nx = px + dx;
                        if (nx < 0 || nx >= width)
                            continue;
                        const int ni = ny * width + nx;
                        if (!mask[static_cast<size_t>(ni)])
                            continue;
                        mask[static_cast<size_t>(ni)] = 0;
                        frontier.push_back(ni);
                    }
                }
            }
            // Full-map frames, long grid lines, and tiny antialiasing specks
            // are excluded before the small-component merge below.
            if (component.count < std::max(3, minimumPixels / 3) ||
                component.width() > maximumLength || component.height() > maximumLength ||
                component.left <= 0 || component.top <= 0 || component.right >= width - 1 ||
                component.bottom >= height - 1)
                continue;
            if (components.size() < MaxRawComponents)
                components.push_back(component);
            else
                componentLimit = true;
        }
    }

    // Join breaks of up to two working pixels, as often caused by anti-aliasing
    // or a printed racing line drawn across an outline. Never join distant gates.
    std::vector<bool> merged(components.size(), false);
    for (size_t i = 0; i < components.size(); ++i) {
        if (merged[i])
            continue;
        for (size_t j = i + 1; j < components.size(); ++j) {
            if (merged[j])
                continue;
            const auto &a = components[i];
            const auto &b = components[j];
            const int gapX = std::max({0, a.left - b.right - 1, b.left - a.right - 1});
            const int gapY = std::max({0, a.top - b.bottom - 1, b.top - a.bottom - 1});
            const int unionWidth = std::max(a.right, b.right) - std::min(a.left, b.left) + 1;
            const int unionHeight = std::max(a.bottom, b.bottom) - std::min(a.top, b.top) + 1;
            if (gapX <= 2 && gapY <= 2 && std::max(unionWidth, unionHeight) <= maximumLength &&
                std::max(unionWidth, unionHeight) <=
                    8 * std::max(1, std::min(unionWidth, unionHeight))) {
                components[i].merge(components[j]);
                merged[j] = true;
            }
        }
    }

    std::vector<Component> candidates;
    for (size_t i = 0; i < components.size(); ++i) {
        if (merged[i])
            continue;
        const auto &c = components[i];
        if (c.count < minimumPixels || c.width() < 3 || c.height() < 3)
            continue;
        const double fill = double(c.count) / (c.width() * c.height());
        const double aspect =
            double(std::max(c.width(), c.height())) / std::min(c.width(), c.height());
        if (aspect > 8.0 || (fill > 0.88 && std::min(c.width(), c.height()) > 7))
            continue;
        candidates.push_back(c);
        if (candidates.size() == MaxGates)
            break;
    }
    if (componentLimit || candidates.size() == MaxGates)
        result.warnings << QStringLiteral(
            "图片中的同色元素过多，识别数量已限制；建议裁剪掉界面和说明文字。");

    std::vector<double> symbolWidths;
    bool hasWideCandidates = false;
    bool hasPillarCandidates = false;
    for (const auto &c : candidates) {
        const double aspect =
            double(std::max(c.width(), c.height())) / std::min(c.width(), c.height());
        if (aspect <= 2.8)
            symbolWidths.push_back(std::max(c.width() * scaleX, c.height() * scaleY));
    }
    double medianSymbolWidth = 0;
    if (!symbolWidths.empty()) {
        auto middle = symbolWidths.begin() + symbolWidths.size() / 2;
        std::nth_element(symbolWidths.begin(), middle, symbolWidths.end());
        medianSymbolWidth = *middle;
        if (options.autoScale) {
            document.metersPerPixel = std::clamp(2.4 / medianSymbolWidth, 0.00001, 1000.0);
            result.warnings << QStringLiteral(
                "比例尺按典型门架宽 2.4 米估算，请用图纸上的已知距离校准。");
        }
    }
    document.backgroundOrigin = QPointF(-input.width() * document.metersPerPixel / 2.0,
                                        -input.height() * document.metersPerPixel / 2.0);

    for (const auto &c : candidates) {
        const QPointF pixel =
            QPointF((c.center().x() + 0.5) * scaleX, (c.center().y() + 0.5) * scaleY);
        Gate gate;
        gate.id = newGateId();
        gate.position = document.backgroundOrigin + pixel * document.metersPerPixel;
        const int major = std::max(c.width(), c.height());
        const int minor = std::min(c.width(), c.height());
        const double aspect = double(major) / minor;
        if (aspect > 3.2 && minor <= 6) {
            gate.type = QStringLiteral("pillar");
            // The thin marker gives a rough footprint, not its upright height.
            // Keep the inferred column small and explicitly request review.
            gate.width = std::clamp(std::min(c.width() * scaleX, c.height() * scaleY) *
                                        document.metersPerPixel,
                                    0.2, 0.8);
            gate.height = 3.0;
            gate.elevation = 1.5;
            gate.extra.insert(QStringLiteral("passSide"), 1);
            hasPillarCandidates = true;
        } else if (aspect > 1.9 && medianSymbolWidth > 0 &&
                   std::max(c.width() * scaleX, c.height() * scaleY) > medianSymbolWidth * 1.45) {
            // A wide blue symbol cannot establish whether the real object has
            // one opening or two. Export an editable wide standard frame and
            // mark the ambiguity, rather than inventing a native double prefab.
            gate.type = QStringLiteral("standard");
            gate.width = 4.8;
            hasWideCandidates = true;
        } else {
            gate.type = QStringLiteral("standard");
        }
        gate.extra.insert(
            QStringLiteral("recognition"),
            QJsonObject{{QStringLiteral("source"), QStringLiteral("local-color-components-v1")},
                        {QStringLiteral("confidence"), aspect <= 2.8 ? 0.72 : 0.50},
                        {QStringLiteral("heightInferred"), true},
                        {QStringLiteral("directionInferred"), true},
                        {QStringLiteral("typeInferred"), true},
                        {QStringLiteral("possibleDoubleGate"), gate.width > 2.4},
                        {QStringLiteral("pixelBounds"),
                         QJsonArray{c.left * scaleX, c.top * scaleY, c.width() * scaleX,
                                    c.height() * scaleY}}});
        document.gates.push_back(gate);
    }

    QVector<Gate> ordered;
    const auto order = suggestedOrder(document.gates);
    ordered.reserve(document.gates.size());
    for (int index : order)
        ordered.push_back(document.gates[index]);
    document.gates = ordered;
    for (int i = 0; i < document.gates.size(); ++i) {
        auto &gate = document.gates[i];
        gate.label = gate.type == QStringLiteral("pillar")
                         ? QStringLiteral("转向柱 %1（待确认）").arg(i + 1)
                     : gate.width > 2.4 ? QStringLiteral("宽门 %1（待确认）").arg(i + 1)
                                        : QStringLiteral("门 %1").arg(i + 1);
        if (document.gates.size() > 1) {
            const QPointF previous =
                document.gates.size() == 2
                    ? gate.position
                    : document.gates[(i + document.gates.size() - 1) % document.gates.size()]
                          .position;
            const auto direction =
                document.gates[(i + 1) % document.gates.size()].position - previous;
            if (!qFuzzyIsNull(direction.x()) || !qFuzzyIsNull(direction.y()))
                // Project convention: 0 degrees points along planar +Y
                // (VelociDrone +Z), and +90 degrees points along planar +X.
                gate.yaw = std::atan2(direction.x(), direction.y()) * 180.0 / Pi;
        }
    }
    result.detectedCount = document.gates.size();
    document.extra.insert(
        QStringLiteral("recognition"),
        QJsonObject{
            {QStringLiteral("method"), QStringLiteral("local-color-components-v1")},
            {QStringLiteral("orderHeuristic"), QStringLiteral("nearest-neighbour-and-2opt")},
            {QStringLiteral("requiresReview"), true},
            {QStringLiteral("detectedCount"), result.detectedCount}});
    if (result.detectedCount == 0) {
        result.warnings << QStringLiteral(
            "未找到可辨认的蓝色门架。请使用清晰的俯视赛道图、裁剪赛道区域，或手动放置门架。");
    } else {
        result.warnings << QStringLiteral(
                               "已识别 %1 个候选门架；门型、方向、高度和通过顺序均需校对。")
                               .arg(result.detectedCount);
        result.warnings << QStringLiteral(
            "通过顺序采用平面距离启发式估算，交叉、回环和重复穿门无法从图片可靠确定。");
        if (hasWideCandidates)
            result.warnings << QStringLiteral(
                "较宽的符号暂按 4.8 米宽单门生成，请确认是否实际为双门或组合门。");
        if (hasPillarCandidates)
            result.warnings << QStringLiteral(
                "细长标记按实心转向柱生成；柱宽为图示估算，高度暂按 3 米、中心高度 1.5 "
                "米，绕行侧暂按右侧，请校对实际尺寸和绕行方向。");
    }
    return result;
}

} // namespace ot
