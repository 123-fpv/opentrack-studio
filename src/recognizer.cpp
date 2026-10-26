#include "recognizer.h"

#include <QJsonArray>
#include <QJsonObject>
#include <QtMath>
#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <set>
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
struct InkBlob {
    Component shape;
    std::vector<int> pixels;
};

std::vector<InkBlob> inkBlobs(std::vector<unsigned char> mask, int width, int height,
                              int minimum = 8) {
    std::vector<InkBlob> result;
    std::vector<int> queue;
    for (int at = 0; at < width * height; ++at) {
        if (!mask[size_t(at)])
            continue;
        queue.clear();
        queue.push_back(at);
        mask[size_t(at)] = 0;
        Component component;
        for (size_t cursor = 0; cursor < queue.size(); ++cursor) {
            const int p = queue[cursor], x = p % width, y = p / width;
            component.add(x, y);
            for (int yy = std::max(0, y - 1); yy <= std::min(height - 1, y + 1); ++yy)
                for (int xx = std::max(0, x - 1); xx <= std::min(width - 1, x + 1); ++xx) {
                    const int next = yy * width + xx;
                    if (mask[size_t(next)]) {
                        mask[size_t(next)] = 0;
                        queue.push_back(next);
                    }
                }
        }
        if (component.count >= minimum && result.size() < 4096)
            result.push_back({component, queue});
    }
    return result;
}

std::vector<unsigned char> morphology(const std::vector<unsigned char> &source, int width,
                                      int height, int radius, bool dilate) {
    std::vector<unsigned char> horizontal(source.size()), result(source.size());
    for (int y = 0; y < height; ++y)
        for (int x = 0; x < width; ++x) {
            bool value = !dilate;
            for (int xx = x - radius; xx <= x + radius; ++xx) {
                const bool pixel = xx >= 0 && xx < width && source[size_t(y * width + xx)];
                value = dilate ? value || pixel : value && pixel;
            }
            horizontal[size_t(y * width + x)] = value;
        }
    for (int y = 0; y < height; ++y)
        for (int x = 0; x < width; ++x) {
            bool value = !dilate;
            for (int yy = y - radius; yy <= y + radius; ++yy) {
                const bool pixel = yy >= 0 && yy < height && horizontal[size_t(yy * width + x)];
                value = dilate ? value || pixel : value && pixel;
            }
            result[size_t(y * width + x)] = value;
        }
    return result;
}

std::vector<unsigned char> closeInk(const std::vector<unsigned char> &source, int width, int height,
                                    int radius) {
    return morphology(morphology(source, width, height, radius, true), width, height, radius,
                      false);
}

QRect blobBounds(const Component &c) {
    return QRect(c.left, c.top, c.width(), c.height());
}
QPointF boxCenter(const Component &c) {
    return QPointF((c.left + c.right + 1) / 2.0, (c.top + c.bottom + 1) / 2.0);
}

std::vector<int> enclosedHoles(const std::vector<unsigned char> &mask, int width,
                               const Component &shape) {
    const int bw = shape.width(), bh = shape.height();
    std::vector<unsigned char> visited(size_t(bw) * bh, 0);
    std::vector<int> holes, queue;
    for (int start = 0; start < bw * bh; ++start) {
        const int sx = start % bw, sy = start / bw;
        if (visited[size_t(start)] || mask[size_t((shape.top + sy) * width + shape.left + sx)])
            continue;
        bool boundary = false;
        queue.clear();
        queue.push_back(start);
        visited[size_t(start)] = 1;
        for (size_t cursor = 0; cursor < queue.size(); ++cursor) {
            const int p = queue[cursor], x = p % bw, y = p / bw;
            boundary |= x == 0 || y == 0 || x == bw - 1 || y == bh - 1;
            for (const QPoint delta : {QPoint(-1, 0), QPoint(1, 0), QPoint(0, -1), QPoint(0, 1)}) {
                const int xx = x + delta.x(), yy = y + delta.y();
                if (xx < 0 || yy < 0 || xx >= bw || yy >= bh)
                    continue;
                const int next = yy * bw + xx;
                if (!visited[size_t(next)] &&
                    !mask[size_t((shape.top + yy) * width + shape.left + xx)]) {
                    visited[size_t(next)] = 1;
                    queue.push_back(next);
                }
            }
        }
        if (!boundary && queue.size() >= size_t(std::max(4, int(bw * bh * .025))))
            holes.push_back(int(queue.size()));
    }
    std::sort(holes.begin(), holes.end(), std::greater<int>());
    return holes;
}

struct DiagramNode {
    Component shape;
    QString type;
    bool edgeOn = false;
    double confidence = .75;
};

struct DiagramAttempt {
    RecognitionResult result;
    bool applicable = false;
};

// Dark schematic symbols need a different detector from blue editor icons.
// The coloured racing line establishes context and graph connectivity; it is
// never replaced with a shortest tour through arbitrary coloured fragments.
DiagramAttempt recognizeInkDiagram(const QImage &input, const QImage &working,
                                   const RecognitionOptions &options, const TrackDocument &base) {
    DiagramAttempt attempt;
    const int width = working.width(), height = working.height();
    const double sx = double(input.width()) / width, sy = double(input.height()) / height;
    const double resolution = std::clamp(width / 1000.0, .4, 2.0);
    const size_t total = size_t(width) * height;
    std::vector<unsigned char> route(total), dark(total);
    int routePixels = 0;
    for (int y = 0; y < height; ++y) {
        const auto *line = reinterpret_cast<const QRgb *>(working.constScanLine(y));
        for (int x = 0; x < width; ++x) {
            const int r = qRed(line[x]), g = qGreen(line[x]), b = qBlue(line[x]);
            route[size_t(y * width + x)] =
                qAlpha(line[x]) >= 100 && r > 90 && r > g * 1.45 && b > g * 1.18 && b > 35;
            routePixels += route[size_t(y * width + x)] != 0;
        }
    }
    if (routePixels < 120 * resolution)
        return attempt;
    const auto routeClosed =
        closeInk(route, width, height, std::max(1, int(std::round(2 * resolution))));
    auto routeParts = inkBlobs(routeClosed, width, height);
    int largest = 0;
    for (const auto &part : routeParts) {
        const auto &s = part.shape;
        if (double(s.count) / (s.width() * s.height()) < .40)
            largest = std::max(largest, s.count);
    }
    if (largest < 120 * resolution)
        return attempt;
    QRect course;
    std::vector<unsigned char> courseLine(total, 0);
    for (const auto &part : routeParts) {
        const auto &s = part.shape;
        if (s.count < std::max(64 * resolution, largest * .07) ||
            double(s.count) / (s.width() * s.height()) >= .40)
            continue;
        course = course.united(blobBounds(s));
        for (int p : part.pixels)
            courseLine[size_t(p)] = 1;
    }
    if (course.width() < width * .15 || course.height() < height * .15)
        return attempt;
    const int margin = std::max(10, int(std::round(30 * resolution)));
    const QRect field =
        course.adjusted(-margin, -margin, margin, margin).intersected(QRect(0, 0, width, height));
    for (int y = field.top(); y <= field.bottom(); ++y) {
        const auto *line = reinterpret_cast<const QRgb *>(working.constScanLine(y));
        for (int x = field.left(); x <= field.right(); ++x) {
            const int r = qRed(line[x]), g = qGreen(line[x]), b = qBlue(line[x]);
            const int maximum = std::max({r, g, b}), minimum = std::min({r, g, b});
            dark[size_t(y * width + x)] =
                qAlpha(line[x]) >= 100 && maximum < 185 && (b >= r * .95 || maximum - minimum < 24);
        }
    }
    const int radius = std::max(1, int(std::round(2 * resolution)));
    const auto connectedInk = closeInk(dark, width, height, radius);
    const auto shapeInk = closeInk(dark, width, height, radius + 1);
    const auto objects = inkBlobs(connectedInk, width, height, std::max(12, int(20 * resolution)));
    struct Shape {
        Component component;
        std::vector<int> holes;
        double fill = 0;
    };
    std::vector<Shape> shapes;
    std::vector<Component> platforms;
    std::vector<double> cellSizes;
    for (const auto &object : objects) {
        const auto &s = object.shape;
        const int major = std::max(s.width(), s.height()), minor = std::min(s.width(), s.height());
        if (major < 18 * resolution || major > std::min(width, height) * .45)
            continue;
        const double fill = double(s.count) / (s.width() * s.height());
        if (fill > .90) {
            platforms.push_back(s);
            continue;
        }
        auto holes = enclosedHoles(shapeInk, width, s);
        const double ratio = double(s.width()) / s.height();
        if (holes.size() == 1 && ratio > .7 && ratio < 1.4 &&
            holes[0] > s.width() * s.height() * .20 && minor >= 14 * resolution)
            cellSizes.push_back(std::min(s.width(), s.height()));
        shapes.push_back({s, std::move(holes), fill});
    }
    if (cellSizes.size() < 3)
        return attempt;
    std::sort(cellSizes.begin(), cellSizes.end());
    const double cellPixels = cellSizes[cellSizes.size() / 2];
    std::vector<DiagramNode> nodes;
    for (const auto &object : shapes) {
        const auto &s = object.component;
        const double bw = s.width(), bh = s.height(), major = std::max(bw, bh),
                     minor = std::min(bw, bh);
        if (major < cellPixels * .6)
            continue;
        const QRect proximity = blobBounds(s)
                                    .adjusted(-radius * 3, -radius * 3, radius * 3, radius * 3)
                                    .intersected(QRect(0, 0, width, height));
        int routeTouches = 0;
        for (int y = proximity.top(); y <= proximity.bottom(); ++y)
            for (int x = proximity.left(); x <= proximity.right(); ++x)
                routeTouches += courseLine[size_t(y * width + x)] != 0;
        if (routeTouches < 2)
            continue;
        DiagramNode node;
        node.shape = s;
        if (major / minor > 3 && minor < cellPixels * .5) {
            node.edgeOn = true;
            node.type =
                major > cellPixels * 2.05 ? QStringLiteral("ladder") : QStringLiteral("standard");
            node.confidence = .68;
        } else if (bw > cellPixels * 1.18 && bh > cellPixels * 1.18 &&
                   ((object.holes.size() >= 3 &&
                     object.holes.front() > object.holes.back() * 1.65) ||
                    object.fill > .50)) {
            node.type =
                bh > cellPixels * 1.65 ? QStringLiteral("cube-stack") : QStringLiteral("cube");
            node.confidence = .72;
        } else if (bw > cellPixels * 2.4 && bh < cellPixels * 1.6 && object.holes.size() >= 2) {
            node.type = QStringLiteral("triple");
        } else if (bw > cellPixels * 1.5 && bh < cellPixels * 1.6 && !object.holes.empty()) {
            node.type = QStringLiteral("double");
        } else if (bh > cellPixels * 1.5 && bw < cellPixels * 1.5 && !object.holes.empty()) {
            node.type = QStringLiteral("ladder");
        } else if (bw > cellPixels * .6 && bw < cellPixels * 1.5 && bh > cellPixels * .6 &&
                   bh < cellPixels * 1.6 && !object.holes.empty()) {
            node.type = QStringLiteral("standard");
        } else
            continue;
        nodes.push_back(node);
        if (nodes.size() == MaxGates)
            break;
    }
    if (nodes.size() < 3)
        return attempt;
    attempt.applicable = true;
    auto &result = attempt.result;
    auto &document = result.document;
    document = base;
    document.gates.clear();
    if (options.autoScale)
        document.metersPerPixel = std::clamp(3.0 / (cellPixels * (sx + sy) / 2), .00001, 1000.0);
    document.backgroundOrigin = QPointF(-input.width() * document.metersPerPixel / 2,
                                        -input.height() * document.metersPerPixel / 2);

    const int count = int(nodes.size()), padding = std::max(3, int(std::round(cellPixels * .56)));
    std::vector<int> zones(total, -1);
    auto remaining = courseLine;
    for (int p = 0; p < width * height; ++p) {
        if (!courseLine[size_t(p)])
            continue;
        const QPoint pixel(p % width, p / width);
        double best = std::numeric_limits<double>::infinity();
        for (int i = 0; i < count; ++i) {
            if (!blobBounds(nodes[size_t(i)].shape)
                     .adjusted(-padding, -padding, padding, padding)
                     .contains(pixel))
                continue;
            const double distance = squaredDistance(pixel, boxCenter(nodes[size_t(i)].shape));
            if (distance < best) {
                best = distance;
                zones[size_t(p)] = i;
            }
        }
        if (zones[size_t(p)] >= 0)
            remaining[size_t(p)] = 0;
    }
    std::vector<std::set<int>> adjacency(static_cast<size_t>(count));
    const auto link = [&](int a, int b) {
        if (a < 0 || b < 0 || a == b)
            return;
        adjacency[size_t(a)].insert(b);
        adjacency[size_t(b)].insert(a);
    };
    const auto neighbours = [&](int p, const auto &callback) {
        const int x = p % width, y = p / width;
        for (int yy = std::max(0, y - 1); yy <= std::min(height - 1, y + 1); ++yy)
            for (int xx = std::max(0, x - 1); xx <= std::min(width - 1, x + 1); ++xx)
                callback(yy * width + xx);
    };
    for (int p = 0; p < width * height; ++p)
        if (zones[size_t(p)] >= 0)
            neighbours(p, [&](int next) { link(zones[size_t(p)], zones[size_t(next)]); });
    struct Terminal {
        Component shape;
        int node;
    };
    std::vector<Terminal> terminals;
    bool ambiguousJunction = false;
    for (const auto &segment : inkBlobs(remaining, width, height, 2)) {
        std::set<int> touches;
        for (int p : segment.pixels)
            neighbours(p, [&](int next) {
                if (zones[size_t(next)] >= 0)
                    touches.insert(zones[size_t(next)]);
            });
        if (touches.size() == 2)
            link(*touches.begin(), *touches.rbegin());
        else if (touches.size() == 1)
            terminals.push_back({segment.shape, *touches.begin()});
        else if (touches.size() > 2)
            ambiguousJunction = true;
    }
    int start = 0;
    QPointF startReference;
    bool startKnown = false;
    double bestStart = cellPixels * cellPixels * 4;
    for (const auto &platform : platforms)
        for (const auto &terminal : terminals) {
            const QPointF a = boxCenter(platform);
            const QRectF box(blobBounds(terminal.shape));
            const QPointF nearest(std::clamp(a.x(), box.left(), box.right()),
                                  std::clamp(a.y(), box.top(), box.bottom()));
            const double distance = squaredDistance(a, nearest);
            if (distance < bestStart) {
                bestStart = distance;
                start = terminal.node;
                startReference = a;
                startKnown = true;
            }
        }
    if (!startKnown) {
        for (int i = 1; i < count; ++i) {
            const auto p = boxCenter(nodes[size_t(i)].shape),
                       best = boxCenter(nodes[size_t(start)].shape);
            if (p.y() < best.y() || (p.y() == best.y() && p.x() < best.x()))
                start = i;
        }
        startReference = boxCenter(nodes[size_t(start)].shape) - QPointF(0, 1);
    }
    std::vector<int> order;
    std::vector<bool> visited(size_t(count), false);
    int current = start, previous = -1;
    QPointF incoming = boxCenter(nodes[size_t(start)].shape) - startReference;
    while (order.size() < nodes.size()) {
        order.push_back(current);
        visited[size_t(current)] = true;
        int next = -1;
        double best = -std::numeric_limits<double>::infinity();
        for (int candidate : adjacency[size_t(current)]) {
            if (visited[size_t(candidate)])
                continue;
            const QPointF direction =
                boxCenter(nodes[size_t(candidate)].shape) - boxCenter(nodes[size_t(current)].shape);
            const double length = std::hypot(direction.x(), direction.y());
            const double score =
                length > .001
                    ? (direction.x() * incoming.x() + direction.y() * incoming.y()) / length
                    : -1;
            if (score > best) {
                best = score;
                next = candidate;
            }
        }
        if (next < 0)
            break;
        previous = current;
        current = next;
        incoming =
            boxCenter(nodes[size_t(current)].shape) - boxCenter(nodes[size_t(previous)].shape);
    }
    const bool connected = order.size() == nodes.size();
    const bool cycle = connected && adjacency[size_t(order.back())].count(start) != 0;
    for (int i = 0; i < count; ++i)
        if (!visited[size_t(i)])
            order.push_back(i);
    document.closedLoop = cycle;
    int compounds = 0;
    for (int i = 0; i < count; ++i) {
        const auto &node = nodes[size_t(order[size_t(i)])];
        Gate gate = makeGate(node.type);
        const auto pixel = boxCenter(node.shape);
        gate.position = document.backgroundOrigin +
                        QPointF(pixel.x() * sx, pixel.y() * sy) * document.metersPerPixel;
        const int before = i > 0 ? i - 1 : (cycle ? count - 1 : i);
        const int after = i + 1 < count ? i + 1 : (cycle ? 0 : i);
        const auto direction = boxCenter(nodes[size_t(order[size_t(after)])].shape) -
                               boxCenter(nodes[size_t(order[size_t(before)])].shape);
        // The diagram's front-facing/edge-on symbol axis supplies orientation;
        // the traced line chooses its sign. Compound internal passes stay editable.
        gate.yaw = node.edgeOn ? (direction.x() < 0 ? -90 : 90) : (direction.y() < 0 ? 180 : 0);
        gate.label = QStringLiteral("%1 %2").arg(gateTypeName(node.type)).arg(i + 1);
        gate.extra.insert(
            QStringLiteral("recognition"),
            QJsonObject{{QStringLiteral("source"), QStringLiteral("local-ink-route-v2")},
                        {QStringLiteral("confidence"), node.confidence},
                        {QStringLiteral("edgeOnSymbol"), node.edgeOn},
                        {QStringLiteral("heightInferred"), true},
                        {QStringLiteral("directionInferred"), true},
                        {QStringLiteral("typeInferred"), true},
                        {QStringLiteral("compoundPassesNeedReview"),
                         node.type != QStringLiteral("standard")},
                        {QStringLiteral("pixelBounds"),
                         QJsonArray{node.shape.left * sx, node.shape.top * sy,
                                    node.shape.width() * sx, node.shape.height() * sy}}});
        compounds += node.type != QStringLiteral("standard");
        document.gates.push_back(gate);
    }
    result.detectedCount = count;
    document.extra.insert(
        QStringLiteral("recognition"),
        QJsonObject{{QStringLiteral("method"), QStringLiteral("local-ink-route-v2")},
                    {QStringLiteral("orderHeuristic"), QStringLiteral("drawn-route-connectivity")},
                    {QStringLiteral("startDetectedFromPlatform"), startKnown},
                    {QStringLiteral("routeGraphConnected"), connected},
                    {QStringLiteral("requiresReview"), true},
                    {QStringLiteral("detectedCount"), count}});
    result.warnings
        << QStringLiteral(
               "已按深色障碍轮廓和洋红路线识别 %1 个实体障碍；已排除图例、文字和实心起飞台。")
               .arg(count);
    if (options.autoScale)
        result.warnings << QStringLiteral(
            "比例尺按单门图示中心距 3 米估算，图示含透视符号，请校对真实尺寸。");
    if (!startKnown)
        result.warnings << QStringLiteral("无法可靠确认起飞点，请手动确认起点和飞行方向。");
    if (!connected || ambiguousJunction)
        result.warnings << QStringLiteral(
            "部分路线连接或交叉关系不明确，未使用最短巡回代替原路线，请检查通过顺序。");
    if (compounds)
        result.warnings << QStringLiteral("复合门保留为组合组件；内部重复过门、左右单元、上下层与准"
                                          "确高度仍需校对，实体障碍数不等于检查点通过次数。");
    result.warnings << QStringLiteral(
        "细长闭合符号按侧视门识别；位置、通过方向和高度为草稿，需在模拟器中验收。");
    return attempt;
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

    auto schematic = recognizeInkDiagram(input, working, options, document);
    if (schematic.applicable) {
        schematic.result.warnings = result.warnings + schematic.result.warnings;
        return schematic.result;
    }

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

    const auto originalBlueMask = mask;
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

    // Grey Windows text can contain blue ClearType fringes. Establish a family
    // of hollow frame shapes before admitting other fragments or using scale.
    int rejectedFragments = 0;
    if (!candidates.empty()) {
        const auto closedBlue = closeInk(originalBlueMask, width, height, 1);
        std::vector<double> frameSpans, hollowFractions;
        hollowFractions.reserve(candidates.size());
        for (const auto &candidate : candidates) {
            const auto holes = enclosedHoles(closedBlue, width, candidate);
            const double enclosed = std::accumulate(holes.begin(), holes.end(), 0.0);
            const double fraction = enclosed / (candidate.width() * candidate.height());
            hollowFractions.push_back(fraction);
            if (std::min(candidate.width(), candidate.height()) >= 8 && fraction >= .12)
                frameSpans.push_back(std::max(candidate.width(), candidate.height()));
        }
        std::vector<Component> supported;
        if (!frameSpans.empty()) {
            std::sort(frameSpans.begin(), frameSpans.end());
            const double familySpan = frameSpans[frameSpans.size() * 3 / 4];
            for (size_t i = 0; i < candidates.size(); ++i) {
                const auto &candidate = candidates[i];
                const double major = std::max(candidate.width(), candidate.height());
                const double minor = std::min(candidate.width(), candidate.height());
                const bool hollow = hollowFractions[i] >= .10 && minor >= 6;
                const bool narrowMarker =
                    major / minor >= 3.2 && minor <= familySpan * .25 && major >= familySpan * .55;
                if (major >= familySpan * .30 && (hollow || narrowMarker))
                    supported.push_back(candidate);
            }
        }
        rejectedFragments = int(candidates.size() - supported.size());
        candidates = std::move(supported);
        if (rejectedFragments)
            result.warnings << QStringLiteral(
                                   "已忽略 %1 个缺少门框形状或尺寸不符的蓝色文字／细碎元素。")
                                   .arg(rejectedFragments);
    }

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
            document.metersPerPixel = std::clamp(3.0 / medianSymbolWidth, 0.00001, 1000.0);
            result.warnings << QStringLiteral(
                "比例尺按标准门框中心线间距 3.0 米估算，请用图纸上的已知距离校准。");
        }
    }
    document.backgroundOrigin = QPointF(-input.width() * document.metersPerPixel / 2.0,
                                        -input.height() * document.metersPerPixel / 2.0);

    for (const auto &c : candidates) {
        const QPointF pixel =
            QPointF((c.center().x() + 0.5) * scaleX, (c.center().y() + 0.5) * scaleY);
        Gate gate = makeGate(QStringLiteral("standard"));
        const int major = std::max(c.width(), c.height());
        const int minor = std::min(c.width(), c.height());
        const double aspect = double(major) / minor;
        if (aspect > 3.2 && minor <= 6) {
            gate = makeGate(QStringLiteral("pillar"));
            gate.extra.insert(QStringLiteral("passSide"), 1);
            hasPillarCandidates = true;
        } else if (aspect > 1.9 && medianSymbolWidth > 0 &&
                   std::max(c.width() * scaleX, c.height() * scaleY) > medianSymbolWidth * 1.45) {
            // A wide blue symbol cannot establish whether the real object has
            // one opening or two. Export an editable wide standard frame and
            // mark the ambiguity, rather than inventing a native double prefab.
            gate.type = QStringLiteral("standard");
            gate.width = 6.0;
            hasWideCandidates = true;
        } else {
            gate.type = QStringLiteral("standard");
        }
        gate.position = document.backgroundOrigin + pixel * document.metersPerPixel;
        gate.extra.insert(
            QStringLiteral("recognition"),
            QJsonObject{{QStringLiteral("source"), QStringLiteral("local-color-components-v1")},
                        {QStringLiteral("confidence"), aspect <= 2.8 ? 0.72 : 0.50},
                        {QStringLiteral("heightInferred"), true},
                        {QStringLiteral("directionInferred"), true},
                        {QStringLiteral("typeInferred"), true},
                        {QStringLiteral("possibleDoubleGate"), gate.width > 3.0},
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
                     : gate.width > 3.0 ? QStringLiteral("宽门 %1（待确认）").arg(i + 1)
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
            {QStringLiteral("rejectedFragments"), rejectedFragments},
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
                "较宽的符号暂按 6.0 米中心线间距单门生成，请确认是否实际为双门或组合门。");
        if (hasPillarCandidates)
            result.warnings << QStringLiteral(
                "细长标记按 0.1 米厚、3.2 米高的实心转向柱生成，中心高度 1.6 米，"
                "绕行侧暂按右侧；请校对组件类型、实际尺寸和绕行方向。");
    }
    return result;
}

} // namespace ot
