#include "recognizer.h"

#include <QJsonArray>
#include <QJsonObject>
#include <QQuaternion>
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

// Measure between the centres of opposite frame strokes. Bounding-box widths
// include both half-stocks and antialiasing, which substantially underestimates
// metres per pixel for the small symbols on a decorated poster.
double frameCentreSpan(const std::vector<unsigned char> &ink, int width,
                       const Component &shape) {
    std::vector<double> spans;
    for (int axis = 0; axis < 2; ++axis) {
        const int length = axis ? shape.height() : shape.width();
        const int cross = axis ? shape.width() : shape.height();
        const auto at = [&](int along, int across) {
            return ink[size_t((shape.top + (axis ? along : across)) * width +
                              shape.left + (axis ? across : along))] != 0;
        };
        for (int row = cross / 4; row <= cross * 3 / 4; ++row) {
            int first = 0, last = length - 1;
            while (first < length && !at(first, row))
                ++first;
            while (last >= 0 && !at(last, row))
                --last;
            if (first > length / 4 || last < length * 3 / 4)
                continue;
            int leftEnd = first, rightBegin = last;
            while (leftEnd + 1 < length && at(leftEnd + 1, row))
                ++leftEnd;
            while (rightBegin > 0 && at(rightBegin - 1, row))
                --rightBegin;
            if (leftEnd - first > length / 4 || last - rightBegin > length / 4 ||
                rightBegin <= leftEnd)
                continue;
            spans.push_back((rightBegin + last - first - leftEnd) / 2.0);
        }
    }
    if (spans.size() < 6)
        return 0;
    std::sort(spans.begin(), spans.end());
    return spans[spans.size() / 2];
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

struct PrintedTrace {
    QVector<QPointF> points;
    QVector<unsigned char> hidden; // 0 visible shaft, 1 navy-occluded continuation, 2 other gap.
    double coverage = 0;
    int occlusions = 0;
    bool ambiguous = false;
};

// Trace the ORIGINAL coloured line, including the parts inside symbols. Closing
// an entire symbol neighbourhood would erase precisely the loops that distinguish
// lower/upper and left/right passages. Thin first, reconnect only aligned ends,
// then consume edges in tangent order rather than taking a shortest path.
PrintedTrace tracePrintedLine(const std::vector<unsigned char> &raw, int width, int height,
                              const std::vector<unsigned char> &ink, const QRect &field,
                              const std::vector<DiagramNode> &nodes, double cell, QPointF launch,
                              QPointF firstGate) {
    PrintedTrace result;
    const size_t total = raw.size();
    auto clipped = raw;
    for (int p = 0; p < width * height; ++p)
        if (!field.contains(QPoint(p % width, p / width)))
            clipped[size_t(p)] = 0;
    std::vector<unsigned char> skeleton(total, 0);
    for (const auto &part : inkBlobs(std::move(clipped), width, height, 3)) {
        bool inside = false;
        if (part.shape.count < cell * .9)
            for (const auto &node : nodes)
                if (blobBounds(node.shape).intersects(blobBounds(part.shape))) {
                    inside = true;
                    break;
                }
        // A few antialias pixels inside an opaque stock are not a usable shaft.
        // Keeping them creates two unreliable endpoint tangents that can steal
        // the true occlusion continuation at higher source resolutions.
        if (part.shape.count >= cell * .9 ||
            (inside && part.shape.count >= std::max(3.0, cell * .12)))
            for (int p : part.pixels)
                skeleton[size_t(p)] = 1;
    }
    const QRect roi = field.intersected(QRect(1, 1, width - 2, height - 2));
    std::vector<int> remove;
    for (int iteration = 0; iteration < 64; ++iteration) {
        bool changed = false;
        for (int phase = 0; phase < 2; ++phase) {
            remove.clear();
            for (int y = roi.top(); y <= roi.bottom(); ++y)
                for (int x = roi.left(); x <= roi.right(); ++x) {
                    const int p = y * width + x;
                    if (!skeleton[size_t(p)])
                        continue;
                    const int n[8] = {skeleton[size_t(p - width)], skeleton[size_t(p - width + 1)],
                                      skeleton[size_t(p + 1)],     skeleton[size_t(p + width + 1)],
                                      skeleton[size_t(p + width)], skeleton[size_t(p + width - 1)],
                                      skeleton[size_t(p - 1)],     skeleton[size_t(p - width - 1)]};
                    int count = 0, transitions = 0;
                    for (int i = 0; i < 8; ++i) {
                        count += n[i];
                        transitions += !n[i] && n[(i + 1) % 8];
                    }
                    if (count < 2 || count > 6 || transitions != 1)
                        continue;
                    const bool keep = phase == 0 ? n[0] * n[2] * n[4] || n[2] * n[4] * n[6]
                                                 : n[0] * n[2] * n[6] || n[0] * n[4] * n[6];
                    if (!keep)
                        remove.push_back(p);
                }
            changed |= !remove.empty();
            for (int p : remove)
                skeleton[size_t(p)] = 0;
        }
        if (!changed)
            break;
    }
    std::vector<int> indices(total, -1), pixels;
    for (int p = 0; p < width * height; ++p)
        if (skeleton[size_t(p)]) {
            if (pixels.size() == 250000) {
                result.ambiguous = true;
                return result;
            }
            indices[size_t(p)] = int(pixels.size());
            pixels.push_back(p);
        }
    if (pixels.empty())
        return result;
    const auto point = [&](int v) {
        const int p = pixels[size_t(v)];
        return QPointF(p % width, p / width);
    };
    std::vector<std::set<int>> graph(pixels.size());
    for (int v = 0; v < int(pixels.size()); ++v) {
        const int p = pixels[size_t(v)], x = p % width, y = p / width;
        for (int dy = -1; dy <= 1; ++dy)
            for (int dx = -1; dx <= 1; ++dx) {
                if ((!dx && !dy) || x + dx < 0 || y + dy < 0 || x + dx >= width || y + dy >= height)
                    continue;
                const int q = p + dy * width + dx;
                if (indices[size_t(q)] < 0)
                    continue;
                // Do not create a triangular shortcut around a one-pixel bend.
                if (dx && dy && (skeleton[size_t(p + dx)] || skeleton[size_t(p + dy * width)]))
                    continue;
                graph[size_t(v)].insert(indices[size_t(q)]);
            }
    }
    const auto outward = [&](int v) {
        int previous = -1, current = v;
        for (int i = 0; i < std::max(4, int(cell * .22)); ++i) {
            int next = -1;
            for (int candidate : graph[size_t(current)])
                if (candidate != previous) {
                    if (next >= 0) {
                        next = -2;
                        break;
                    }
                    next = candidate;
                }
            if (next < 0)
                break;
            previous = current;
            current = next;
        }
        const auto d = point(v) - point(current);
        const double length = std::hypot(d.x(), d.y());
        return length > .001 ? d / length : QPointF();
    };
    std::vector<int> endpoints;
    for (int v = 0; v < int(pixels.size()); ++v)
        if (graph[size_t(v)].size() == 1)
            endpoints.push_back(v);
    if (endpoints.size() > 2048) {
        result.ambiguous = true;
        return result;
    }
    struct Gap {
        double score;
        int a, b;
    };
    std::vector<Gap> gaps;
    std::vector<QPointF> tangents;
    for (int v : endpoints)
        tangents.push_back(outward(v));
    for (size_t i = 0; i < endpoints.size(); ++i)
        for (size_t j = i + 1; j < endpoints.size(); ++j) {
            const auto a = point(endpoints[i]), b = point(endpoints[j]), delta = b - a;
            const double length = std::hypot(delta.x(), delta.y());
            if (length < 1 || length > cell * 1.45)
                continue;
            if (length > cell * .38) {
                bool inSymbol = false;
                const QPoint midpoint = ((a + b) / 2).toPoint();
                for (const auto &node : nodes)
                    inSymbol |= blobBounds(node.shape)
                                    .adjusted(int(-cell * .3), int(-cell * .3), int(cell * .3),
                                              int(cell * .3))
                                    .contains(midpoint);
                if (!inSymbol)
                    continue;
            }
            const auto direction = delta / length;
            const double aa = QPointF::dotProduct(tangents[i], direction),
                         ab = -QPointF::dotProduct(tangents[j], direction);
            if (aa > .1 && ab > .1)
                gaps.push_back({length / (.2 + aa + ab), endpoints[i], endpoints[j]});
        }
    std::sort(gaps.begin(), gaps.end(),
              [](const Gap &a, const Gap &b) { return a.score < b.score; });
    std::set<int> joined;
    std::vector<int> parent(pixels.size());
    std::iota(parent.begin(), parent.end(), 0);
    const auto root = [&](int v) {
        while (parent[size_t(v)] != v) {
            parent[size_t(v)] = parent[size_t(parent[size_t(v)])];
            v = parent[size_t(v)];
        }
        return v;
    };
    for (int v = 0; v < int(graph.size()); ++v)
        for (int next : graph[size_t(v)])
            parent[size_t(root(v))] = root(next);
    // Prefer completing broken shafts before closing a small projected loop.
    // Otherwise the nearest pair on one upright can consume both ends of the
    // return arc and strand the route that leaves the next aperture.
    for (int phase = 0; phase < 2; ++phase)
        for (const auto &gap : gaps)
            if (!joined.count(gap.a) && !joined.count(gap.b) &&
                (phase || root(gap.a) != root(gap.b))) {
                graph[size_t(gap.a)].insert(gap.b);
                graph[size_t(gap.b)].insert(gap.a);
                parent[size_t(root(gap.a))] = root(gap.b);
                joined.insert(gap.a);
                joined.insert(gap.b);
                ++result.occlusions;
            }
    // Printed arrow heads create short leaf branches. Keep the shaft and the
    // long input tail; a short cycle is retained and consumed by the edge walk.
    for (int iteration = 0; iteration < 4; ++iteration) {
        std::set<int> discarded;
        for (int v = 0; v < int(pixels.size()); ++v) {
            if (graph[size_t(v)].size() != 1)
                continue;
            std::vector<int> branch{v};
            int previous = -1, current = v;
            double length = 0;
            while (graph[size_t(current)].size() <= 2 && branch.size() < size_t(cell * .5 + 3)) {
                int next = -1;
                for (int candidate : graph[size_t(current)])
                    if (candidate != previous)
                        next = candidate;
                if (next < 0)
                    break;
                length += std::sqrt(squaredDistance(point(next), point(current)));
                previous = current;
                current = next;
                branch.push_back(current);
            }
            if (graph[size_t(current)].size() >= 3 && length <= cell * .29)
                discarded.insert(branch.begin(), branch.end() - 1);
        }
        if (discarded.empty())
            break;
        for (int v : discarded) {
            for (int next : graph[size_t(v)])
                graph[size_t(next)].erase(v);
            graph[size_t(v)].clear();
        }
    }
    int start = -1;
    double closest = std::numeric_limits<double>::infinity();
    int edges = 0;
    for (int v = 0; v < int(pixels.size()); ++v) {
        edges += int(graph[size_t(v)].size());
        if (!graph[size_t(v)].empty()) {
            const double d = squaredDistance(point(v), launch);
            if (d < closest) {
                closest = d;
                start = v;
            }
        }
    }
    if (start < 0)
        return result;
    std::set<std::pair<int, int>> consumed;
    std::vector<int> walk{start};
    int current = start;
    const auto edge = [](int a, int b) { return std::make_pair(std::min(a, b), std::max(a, b)); };
    for (size_t iteration = 0; iteration < pixels.size() * 3; ++iteration) {
        const auto incoming =
            walk.size() >= 5 ? point(current) - point(walk[walk.size() - 5]) : firstGate - launch;
        int next = -1;
        double best = -std::numeric_limits<double>::infinity();
        for (int candidate : graph[size_t(current)]) {
            if (consumed.count(edge(current, candidate)))
                continue;
            int previous = current, look = candidate;
            for (int i = 0; i < 4; ++i) {
                if (graph[size_t(look)].size() != 2)
                    break;
                const int further = *graph[size_t(look)].begin() == previous
                                        ? *graph[size_t(look)].rbegin()
                                        : *graph[size_t(look)].begin();
                previous = look;
                look = further;
            }
            auto d = point(look) - point(current);
            const double length = std::hypot(d.x(), d.y());
            const double score = length > .001 ? QPointF::dotProduct(incoming, d / length) : -1;
            if (score > best) {
                best = score;
                next = candidate;
            }
        }
        if (next < 0 && graph[size_t(current)].size() >= 3) {
            // A thinned X can become two nearby Y junctions joined by a short
            // neck. Reuse only that bounded connector to reach its unused exit;
            // completed shafts and projected loops remain consumed.
            const double radius = std::max(3.0, cell * .12);
            std::vector<int> previous(pixels.size(), -1), queue{current};
            previous[size_t(current)] = current;
            int branch = -1;
            for (size_t at = 0; at < queue.size() && branch < 0; ++at) {
                const int v = queue[at];
                if (v != current && graph[size_t(v)].size() >= 3)
                    for (int n : graph[size_t(v)])
                        if (!consumed.count(edge(v, n))) {
                            branch = v;
                            break;
                        }
                if (branch >= 0)
                    break;
                for (int n : graph[size_t(v)]) {
                    if (previous[size_t(n)] >= 0 || !consumed.count(edge(v, n)) ||
                        squaredDistance(point(n), point(current)) > radius * radius)
                        continue;
                    previous[size_t(n)] = v;
                    queue.push_back(n);
                }
            }
            if (branch >= 0) {
                bool inSymbol = false;
                for (const auto &node : nodes)
                    inSymbol |= blobBounds(node.shape).contains(point(current).toPoint());
                // Traversing an unmarked field crossing does not establish its
                // over/under or first-branch meaning. A split junction inside
                // a physical symbol is instead resolved by its window visits.
                result.ambiguous |= !inSymbol;
                std::vector<int> connector;
                for (int v = branch; v != current; v = previous[size_t(v)])
                    connector.push_back(v);
                for (auto it = connector.rbegin(); it != connector.rend(); ++it)
                    walk.push_back(*it);
                current = branch;
                continue;
            }
        }
        if (next < 0)
            break;
        consumed.insert(edge(current, next));
        current = next;
        walk.push_back(current);
    }
    result.coverage = edges ? double(consumed.size()) / (edges / 2) : 0;
    result.ambiguous |= result.coverage < .90;
    // Virtual occlusion edges need intermediate samples for narrow, edge-on
    // windows. They carry the same inferred-status metadata as the whole trace.
    result.points.push_back(point(walk.front()));
    result.hidden.push_back(0);
    for (size_t i = 1; i < walk.size(); ++i) {
        const auto a = point(walk[i - 1]), b = point(walk[i]);
        const int samples = std::max(1, int(std::ceil(std::sqrt(squaredDistance(a, b)))));
        int inkSamples = 0;
        if (samples > 2)
            for (int j = 1; j <= samples; ++j) {
                const auto p = (a + (b - a) * (double(j) / samples)).toPoint();
                inkSamples += ink[size_t(p.y() * width + p.x())] != 0;
            }
        const unsigned char occlusion =
            samples > 2 ? (inkSamples >= std::max(1.0, samples * .04) ? 1 : 2) : 0;
        for (int j = 1; j <= samples; ++j) {
            result.points.push_back(a + (b - a) * (double(j) / samples));
            result.hidden.push_back(occlusion);
        }
    }
    return result;
}

struct WindowVisit {
    int column = 0, level = 0, first = 0, last = 0;
    bool interrupted = false;
};

// Fit the centre of a raster stroke without treating its one-pixel staircase as
// a flight manoeuvre. Arc distance, rather than pixel index, supplies the fit's
// parameter. Semantic crossings are still located on the original trace below.
class StrokeCentreline {
    QVector<QPointF> points_;
    double maximumShift_ = 0;

  public:
    StrokeCentreline(const PrintedTrace &trace, double cell) : points_(trace.points) {
        if (points_.size() < 5)
            return;
        QVector<double> distances(points_.size());
        for (qsizetype i = 1; i < points_.size(); ++i)
            distances[i] =
                distances[i - 1] + std::sqrt(squaredDistance(trace.points[i], trace.points[i - 1]));
        const double radius = std::max(2., cell * .15), cap = cell * .025;
        int first = 0, last = 0;
        for (int i = 1; i + 1 < points_.size(); ++i) {
            while (first < i && distances[i] - distances[first] > radius)
                ++first;
            last = std::max(last, i);
            while (last + 1 < points_.size() && distances[last + 1] - distances[i] <= radius)
                ++last;
            if (last - first < 3)
                continue;
            double matrix[3][5]{};
            for (int j = first; j <= last; ++j) {
                const double t = (distances[j] - distances[i]) / radius;
                const double powers[5]{1, t, t * t, t * t * t, t * t * t * t};
                const double weight = std::exp(-2 * t * t);
                for (int r = 0; r < 3; ++r) {
                    for (int c = 0; c < 3; ++c)
                        matrix[r][c] += weight * powers[r + c];
                    matrix[r][3] += weight * powers[r] * trace.points[j].x();
                    matrix[r][4] += weight * powers[r] * trace.points[j].y();
                }
            }
            bool valid = true;
            for (int column = 0; column < 3; ++column) {
                int pivot = column;
                for (int row = column + 1; row < 3; ++row)
                    if (std::abs(matrix[row][column]) > std::abs(matrix[pivot][column]))
                        pivot = row;
                if (std::abs(matrix[pivot][column]) < 1e-10) {
                    valid = false;
                    break;
                }
                for (int c = column; c < 5; ++c)
                    std::swap(matrix[column][c], matrix[pivot][c]);
                const double divisor = matrix[column][column];
                for (int c = column; c < 5; ++c)
                    matrix[column][c] /= divisor;
                for (int row = 0; row < 3; ++row) {
                    if (row == column)
                        continue;
                    const double factor = matrix[row][column];
                    for (int c = column; c < 5; ++c)
                        matrix[row][c] -= factor * matrix[column][c];
                }
            }
            if (!valid)
                continue;
            auto displacement = QPointF(matrix[0][3], matrix[0][4]) - trace.points[i];
            const double distance = std::hypot(displacement.x(), displacement.y());
            if (distance > cap)
                displacement *= cap / distance;
            points_[i] += displacement;
            maximumShift_ = std::max(maximumShift_, std::min(distance, cap));
        }
    }
    QPointF at(double index) const {
        const double bounded = std::clamp(index, 0., double(points_.size() - 1));
        const int first = int(bounded), last = std::min(first + 1, int(points_.size()) - 1);
        return points_[first] * (1 - (bounded - first)) + points_[last] * (bounded - first);
    }
    QJsonObject evidence(double cell) const {
        return {{"method", "bounded-quadratic-centreline-v1"},
                {"maximumPixelShift", maximumShift_},
                {"pixelShiftLimit", cell * .025},
                {"preservesSemanticCrossings", true},
                {"preservesEssentialChecks", true}};
    }
};

// Raster noise and the ground clamp can leave millimetre-scale reversals in an
// otherwise smooth inferred turn. Fair ordinary guides only. Physical portal
// crossings and the explicitly exported side checks are immutable constraints.
void refineFittedStroke(const Gate &gate, QVector<RouteStep> *steps, QJsonObject *evidence) {
    if (steps->size() < 4)
        return;
    TrackDocument document;
    document.gates = {gate};
    document.closedLoop = false;
    document.routeStepsDefined = true;
    document.routeSteps = *steps;
    QVector<RouteStep> candidate = *steps;
    const auto position = [&](const RouteStep &step) {
        return routeStepGeometry(document, step).pathPosition;
    };
    const auto fixed = [](const RouteStep &step) {
        return step.kind != "guide" || step.extra.value("raceCheckpoint").toBool();
    };
    int removed = 0;
    for (int i = 1; i + 1 < candidate.size();) {
        if (!fixed(candidate[i]) &&
            std::min((position(candidate[i]) - position(candidate[i - 1])).length(),
                     (position(candidate[i + 1]) - position(candidate[i])).length()) < .14f) {
            candidate.removeAt(i);
            ++removed;
        } else
            ++i;
    }
    const auto m = gateFrameMetrics(gate);
    const QVector3D scale(float(m.columns * gate.width / 2), float(m.levels * gate.height / 2),
                          float(gate.width / 2));
    const auto inverse = QQuaternion::fromAxisAndAngle(0, 1, 0, float(gate.yaw)).conjugated();
    for (int i = 1; i + 1 < candidate.size(); ++i) {
        if (fixed(candidate[i]))
            continue;
        const auto incoming = (position(candidate[i]) - position(candidate[i - 1])).normalized();
        const auto outgoing = (position(candidate[i + 1]) - position(candidate[i])).normalized();
        const auto direction = (incoming + outgoing).normalized();
        if (direction.lengthSquared() < .01f)
            continue;
        const auto tangent = (inverse.rotatedVector(direction) / scale).normalized();
        candidate[i].extra.insert("curveTangent",
                                  QJsonArray{tangent.x(), tangent.y(), tangent.z()});
    }
    // Essential outside-bar checks retain the turn's homotopy. In addition,
    // ensure each documented signed turn remains unchanged by guide removal.
    const auto turns = evidence->value("progressiveTurns").toArray();
    const auto winding = [&](const QVector<RouteStep> &route, const QJsonObject &turn) {
        const int firstPass = turn.value("fromComponentPass").toInt(1),
                  lastPass = turn.value("toComponentPass").toInt(2);
        const double barX = turn.value("barX").toDouble();
        int pass = 0;
        bool begun = false;
        double previous = 0, sweep = 0;
        for (const auto &step : route) {
            if (step.kind == "pass")
                ++pass;
            if (pass < firstPass)
                continue;
            auto p = inverse.rotatedVector(position(step) - QVector3D(float(gate.position.x()),
                                                                      float(gate.elevation),
                                                                      float(gate.position.y())));
            const double angle = std::atan2(p.z(), p.x() - barX);
            if (begun)
                sweep += std::remainder(angle - previous, 2 * Pi);
            previous = angle;
            begun = true;
            if (pass == lastPass && step.kind == "pass")
                break;
        }
        return sweep;
    };
    bool topology = true;
    for (const auto &value : turns) {
        const auto turn = value.toObject();
        topology &= std::abs(winding(candidate, turn) - winding(*steps, turn)) < Pi / 36;
    }
    document.routeSteps = candidate;
    const auto curve = topology ? routeCurve(document) : RouteCurveResult();
    if (topology && curve.clearanceSatisfied && curve.smooth && curve.breaks.isEmpty()) {
        *steps = candidate;
        evidence->insert("guideFairing", QJsonObject{{"method", "semantic-anchor-preserving-v1"},
                                                     {"removedRedundantGuides", removed},
                                                     {"sourceCrossingsPreserved", true},
                                                     {"essentialChecksPreserved", true},
                                                     {"windingPreserved", true},
                                                     {"clearanceVerified", true}});
    }
}

// A side-on drawing establishes a height/depth cross-section. Front-facing
// symbols do not: their screen-space loop can be an elevation change or a
// projected turn around an upright. Preserve those samples as evidence, but do
// not assign a 3D orbit handedness from the component type alone.
bool sourceCrossbarWrap(const Gate &gate, const DiagramNode &node, const PrintedTrace &trace,
                        const QVector<WindowVisit> &visits, double cell, QVector<RouteStep> *steps,
                        QJsonObject *evidence) {
    if (!node.edgeOn || visits.size() != 2 || visits[0].level == visits[1].level ||
        visits[0].column != visits[1].column)
        return false;
    const auto metrics = gateFrameMetrics(gate);
    const QRectF box(blobBounds(node.shape));
    const auto crossing = [&](const WindowVisit &visit) {
        int best = visit.first;
        double distance = std::numeric_limits<double>::infinity();
        for (int i = visit.first; i <= visit.last; ++i) {
            const double d = std::abs(trace.points[i].x() - box.center().x());
            if (d < distance) {
                best = i;
                distance = d;
            }
        }
        return best;
    };
    const int first = crossing(visits[0]), last = crossing(visits[1]);
    if (last - first < 4)
        return false;
    const double halfWidth = metrics.columns * gate.width / 2,
                 halfHeight = metrics.levels * gate.height / 2, halfDepth = gate.width / 2;
    const double fromY = (visits[0].level - (metrics.levels - 1) / 2.0) * gate.height,
                 toY = (visits[1].level - (metrics.levels - 1) / 2.0) * gate.height,
                 centerY = (fromY + toY) / 2, radius = std::abs(toY - fromY) / 2;
    const double depthSign = std::sin(qDegreesToRadians(gate.yaw));
    const auto startPixel = trace.points[first], endPixel = trace.points[last];
    QVector<QPointF> section;
    QJsonArray sourceCurve;
    for (int i = first; i <= last; ++i) {
        const double t = double(i - first) / (last - first);
        const auto pixel = trace.points[i];
        // Correct endpoint offsets to portal centres without erasing the
        // original lobe/curvature. Absolute drawing depth remains estimated.
        const double baselineX = startPixel.x() * (1 - t) + endPixel.x() * t,
                     baselineY = startPixel.y() * (1 - t) + endPixel.y() * t;
        const double z = (pixel.x() - baselineX) * depthSign * gate.width / cell,
                     y = fromY * (1 - t) + toY * t -
                         (pixel.y() - baselineY) * gate.height * metrics.levels / box.height();
        section.push_back({z, y - centerY});
    }
    double angle = std::atan2(section[0].y(), section[0].x()), sweep = 0, variation = 0,
           minimumRadius = std::numeric_limits<double>::infinity();
    QVector<double> phases{0};
    for (int i = 0; i < section.size(); ++i) {
        const auto p = section[i];
        minimumRadius = std::min(minimumRadius, std::hypot(p.x(), p.y()));
        if (i) {
            const double next = std::atan2(p.y(), p.x());
            const double delta = std::remainder(next - angle, 2 * Pi);
            sweep += delta;
            variation += std::abs(delta);
            angle = next;
            phases.push_back(sweep);
        }
        if (i == 0 || i == section.size() - 1 ||
            i % std::max(1, int((section.size() + 47) / 48)) == 0)
            sourceCurve.push_back(QJsonArray{p.x() / halfDepth, (p.y() + centerY) / halfHeight});
    }
    const bool reliable = std::abs(sweep) > Pi * .65 && std::abs(sweep) < Pi * 1.4 &&
                          variation < std::abs(sweep) * 2.25 && minimumRadius > gate.width * .055;
    const int winding = sweep >= 0 ? 1 : -1;
    QJsonObject action{
        {QStringLiteral("type"), QStringLiteral("bar-wrap")},
        {QStringLiteral("axis"), QStringLiteral("x")},
        {QStringLiteral("winding"), winding},
        {QStringLiteral("view"), QStringLiteral("along-local+X; +Z right; +Y up")},
        {QStringLiteral("observedSweepDegrees"), qRadiansToDegrees(sweep)},
        {QStringLiteral("sourceCurve"), sourceCurve},
        {QStringLiteral("sourceCurveCoordinates"), QStringLiteral("normalized-local-ZY")},
        {QStringLiteral("depthEstimated"), true},
        {QStringLiteral("requiresReview"), true},
        {QStringLiteral("inferredFromSideView"), reliable}};
    if (!reliable) {
        action.insert(QStringLiteral("ambiguous"), true);
        evidence->insert(QStringLiteral("barWrap"), action);
        return false;
    }
    QVector<RouteStep> passes;
    for (const auto &step : *steps)
        if (step.kind == QStringLiteral("pass"))
            passes.push_back(step);
    if (passes.size() != 2)
        return false;
    const double initialAngle = fromY > toY ? Pi / 2 : -Pi / 2;
    // Radius is fitted from the traced lobe, bounded away from the actual stock.
    // Sampling in polar angle retains source curvature and cannot cut the bar
    // with the single diagonal guide used by the earlier recognizer.
    const auto radial = [&](double t) {
        double weighted = 0, weights = 0;
        for (int i = 0; i < section.size(); ++i) {
            const double phase = std::clamp(phases[i] / sweep, 0.0, 1.0), d = (phase - t) / .14;
            const double weight = std::exp(-d * d);
            weighted += weight * std::hypot(section[i].x(), section[i].y());
            weights += weight;
        }
        const double fitted = weights > .00001 ? weighted / weights : radius;
        const double blend = std::pow(std::sin(Pi * t), 2);
        return radius * (1 - blend) + std::clamp(fitted, radius * .68, radius * 1.55) * blend;
    };
    const auto localAt = [&](double t) {
        const double a = initialAngle + winding * Pi * t, r = radial(t);
        return QVector3D(
            float((visits[0].column - (metrics.columns - 1) / 2.0) * gate.width / halfWidth),
            float((centerY + r * std::sin(a)) / halfHeight), float(r * std::cos(a) / halfDepth));
    };
    const double firstNormal = -std::sin(initialAngle) * winding;
    for (auto &pass : passes)
        pass.reverse = firstNormal < 0;
    action.insert(QStringLiteral("sweepDegrees"), winding * 180.0);
    action.insert(QStringLiteral("curveFit"), QStringLiteral("source-polar-radius-profile"));
    action.insert(QStringLiteral("ambiguous"), false);
    passes[0].extra.insert(QStringLiteral("maneuver"), action);
    QVector<RouteStep> fitted{passes[0]};
    constexpr int ArcSegments = 12;
    for (int j = 1; j < ArcSegments; ++j) {
        const double t = double(j) / ArcSegments;
        RouteStep guide;
        guide.id = newGateId();
        guide.gateId = gate.id;
        guide.kind = QStringLiteral("guide");
        guide.localGuide = localAt(t);
        const auto tangent = (localAt(t + .001) - localAt(t - .001)).normalized();
        guide.extra.insert(QStringLiteral("curveMode"), QStringLiteral("arc-v1"));
        guide.extra.insert(QStringLiteral("curveTangent"),
                           QJsonArray{tangent.x(), tangent.y(), tangent.z()});
        auto guideAction = action;
        guideAction.remove(QStringLiteral("sourceCurve"));
        guide.extra.insert(QStringLiteral("maneuver"), guideAction);
        fitted.push_back(guide);
    }
    fitted.push_back(passes[1]);
    *steps = fitted;
    evidence->insert(QStringLiteral("barWrap"), action);
    evidence->insert(QStringLiteral("wrapWindingInferred"), true);
    return true;
}

// Depth is signed by stroke/stock occlusion, not by a gate-type template.
// Visible crossings of a projected stock are in front; navy-covered gaps are
// behind. Height is retained from the glyph, including excursions above bars.
bool fitOccludedFrontalStroke(const Gate &gate, const DiagramNode &node, const PrintedTrace &trace,
                              const QVector<WindowVisit> &visits, double cell,
                              QVector<RouteStep> *steps, QJsonObject *evidence) {
    const auto m = gateFrameMetrics(gate);
    if (node.edgeOn || m.centerDepth > 0 || (m.columns == 1 && m.levels == 1) ||
        trace.hidden.size() != trace.points.size())
        return false;
    const QRectF box(blobBounds(node.shape));
    const double band = std::max(1.4, cell * .065);
    const QRectF context = box.adjusted(-band * 1.7, -band * 1.7, band * 1.7, band * 1.7);
    struct Event {
        int first, last;
        bool far;
    };
    QVector<Event> events;
    int first = -1, last = -1;
    for (int i = 0; i < trace.points.size(); ++i) {
        const auto p = trace.points[i];
        if (!context.contains(p))
            continue;
        if (first < 0)
            first = i;
        last = i;
        double distance = std::numeric_limits<double>::infinity();
        for (int c = 0; c <= m.columns; ++c)
            distance =
                std::min(distance, std::abs(p.x() - box.left() - c * box.width() / m.columns));
        for (int r = 0; r <= m.levels; ++r)
            distance =
                std::min(distance, std::abs(p.y() - box.top() - r * box.height() / m.levels));
        const bool covered = trace.hidden[i] == 1;
        if (!covered && (distance > band || trace.hidden[i] == 2))
            continue;
        if (events.isEmpty() || i - events.last().last > std::max(3.0, cell * .22))
            events.push_back({i, i, covered});
        else {
            events.last().last = i;
            events.last().far |= covered;
        }
    }
    bool near = false, far = false;
    for (const auto &e : events) {
        near |= !e.far;
        far |= e.far;
    }
    if (!near || !far || events.size() < 3 || first < 0)
        return false;
    const QVector3D scale(float(m.columns * gate.width / 2), float(m.levels * gate.height / 2),
                          float(gate.width / 2));
    const double horizontalSign = std::cos(qDegreesToRadians(gate.yaw)) < 0 ? -1 : 1;
    const double depth = gate.width * .34;
    const StrokeCentreline centreline(trace, cell);
    const double tangentSpan = std::max(3., cell * .12);
    const auto eventIndex = [&](int i) { return (events[i].first + events[i].last) / 2; };
    const auto localAt = [&](double index, bool denoise = false) {
        const int at = std::clamp(int(index), 0, int(trace.points.size()) - 1);
        const auto p = denoise ? centreline.at(index) : trace.points[at];
        double z = events.front().far ? depth : -depth;
        for (int j = 1; j < events.size(); ++j) {
            if (index < eventIndex(j - 1))
                break;
            const double a = events[j - 1].far ? depth : -depth, b = events[j].far ? depth : -depth;
            const double t = std::clamp((index - eventIndex(j - 1)) /
                                            std::max(1, eventIndex(j) - eventIndex(j - 1)),
                                        0.0, 1.0);
            z = a + (b - a) * t * t * (3 - 2 * t);
            if (index <= eventIndex(j))
                break;
        }
        const double x =
            (p.x() - box.center().x()) * gate.width * m.columns / box.width() * horizontalSign;
        const double y = std::max(.35 - gate.elevation, (box.center().y() - p.y()) * gate.height *
                                                            m.levels / box.height());
        return QVector3D(float(x), float(y), float(z));
    };
    struct Crossing {
        int index;
        RouteStep step;
        QVector3D point;
    };
    QVector<Crossing> crossings;
    bool adjusted = false;
    for (int j = 1; j < events.size(); ++j) {
        if (events[j - 1].far == events[j].far)
            continue;
        const int index = (eventIndex(j - 1) + eventIndex(j)) / 2;
        auto p = localAt(index);
        const int column = int(std::floor((p.x() + scale.x()) / gate.width)),
                  level = int(std::floor((p.y() + scale.y()) / gate.height));
        if (!gateCellOpen(gate, column, level))
            continue; // Outside the frame or inside a blocked cell is not a portal.
        const double cx = (column - (m.columns - 1) / 2.0) * gate.width,
                     cy = (level - (m.levels - 1) / 2.0) * gate.height;
        double u = (p.x() - cx) / (m.openingWidth / 2), v = (p.y() - cy) / (m.openingHeight / 2);
        if (std::abs(u) > 1.02 || std::abs(v) > 1.02)
            continue;
        adjusted |= std::abs(u) > .84 || std::abs(v) > .84;
        u = std::clamp(u, -.84, .84);
        v = std::clamp(v, -.84, .84);
        RouteStep pass;
        pass.id = newGateId();
        pass.gateId = gate.id;
        pass.column = column;
        pass.level = level;
        pass.face = events[j - 1].far ? QStringLiteral("back") : QStringLiteral("front");
        pass.extra.insert("apertureU", pass.face == "back" ? -u : u);
        pass.extra.insert("apertureV", v);
        const auto tangent =
            ((localAt(index + tangentSpan, true) - localAt(index - tangentSpan, true)) / scale)
                .normalized();
        pass.extra.insert("flightTangent", QJsonArray{tangent.x(), tangent.y(), tangent.z()});
        pass.extra.insert("curveMode", "arc-v1");
        p = QVector3D(float(cx + u * m.openingWidth / 2), float(cy + v * m.openingHeight / 2), 0);
        crossings.push_back({index, pass, p});
    }
    if (crossings.size() < 2)
        return false;
    for (const auto &visit : visits) {
        const bool represented =
            std::any_of(crossings.begin(), crossings.end(), [&](const Crossing &c) {
                return c.step.column == visit.column && c.step.level == visit.level;
            });
        if (!represented)
            return false;
    }
    QVector<RouteStep> fitted;
    QVector<int> sourceIndices;
    const auto appendGuide = [&](int index) {
        RouteStep guide;
        guide.id = newGateId();
        guide.gateId = gate.id;
        guide.kind = "guide";
        guide.localGuide = localAt(index, true) / scale;
        const auto tangent =
            ((localAt(index + tangentSpan, true) - localAt(index - tangentSpan, true)) / scale)
                .normalized();
        if (tangent.lengthSquared() < .01f)
            return;
        guide.extra.insert("curveMode", "arc-v1");
        guide.extra.insert("curveTangent", QJsonArray{tangent.x(), tangent.y(), tangent.z()});
        fitted.push_back(guide);
        sourceIndices.push_back(index);
    };
    int next = 0;
    const int spacing = std::max(3, int(cell * .15));
    for (int i = first; i <= last; ++i) {
        if (next < crossings.size() && i == crossings[next].index) {
            fitted.push_back(crossings[next].step);
            sourceIndices.push_back(i);
            ++next;
        } else if ((i == first || i == last || (i - first) % spacing == 0) &&
                   std::none_of(crossings.begin(), crossings.end(),
                                [&](const Crossing &c) { return std::abs(c.index - i) < 3; }))
            appendGuide(i);
    }
    QJsonArray transitions, sourceEvents, sourceSamples;
    for (const auto &e : events) {
        const auto p = trace.points[(e.first + e.last) / 2];
        sourceEvents.push_back(
            QJsonObject{{"pixel", QJsonArray{p.x(), p.y()}},
                        {"depthSide", e.far ? "behind-stock" : "in-front-of-stock"}});
    }
    for (int i = first; i <= last; i += std::max(1, (last - first + 47) / 48)) {
        const auto p = localAt(i) / scale;
        sourceSamples.push_back(QJsonArray{p.x(), p.y(), p.z()});
    }
    for (int j = 1; j < crossings.size(); ++j) {
        const auto &a = crossings[j - 1], &b = crossings[j];
        const bool upright = a.step.column != b.step.column || m.levels > 1;
        double barX = (a.point.x() + b.point.x()) / 2;
        if (m.levels > 1 && a.step.column == b.step.column) {
            double left = 0, right = 0;
            for (int i = a.index; i <= b.index; ++i) {
                const auto p = localAt(i);
                left = std::min(left, double(p.x()));
                right = std::max(right, double(p.x()));
            }
            barX = std::abs(left) > std::abs(right) ? -scale.x() : scale.x();
        }
        double sweep = 0, previous = 0, bestScore = -1e30;
        int essential = -1;
        for (int k = 0; k < fitted.size(); ++k) {
            const int index = sourceIndices[k];
            if (index < a.index || index > b.index)
                continue;
            const auto p = localAt(index);
            const double angle = std::atan2(p.z(), p.x() - barX);
            if (index > a.index)
                sweep += std::remainder(angle - previous, 2 * Pi);
            previous = angle;
            if (fitted[k].kind != "guide")
                continue;
            const double score = m.levels > 1 ? (barX > 0 ? p.x() : -p.x()) : p.y();
            if (score > bestScore) {
                bestScore = score;
                essential = k;
            }
        }
        QJsonObject action{{"type", "progressive-bar-wrap"},
                           {"axis", upright ? "y" : "x"},
                           {"barX", barX},
                           {"fromComponentPass", j},
                           {"toComponentPass", j + 1},
                           {"signedSweepDegrees", qRadiansToDegrees(sweep)},
                           {"axialProgress", b.point.y() - a.point.y()},
                           {"view", "along-local+Y; +X right; +Z up"},
                           {"source", "visible/occluded-stock-crossings"},
                           {"depthEstimated", true}};
        transitions.push_back(action);
        if (essential >= 0) {
            // A native side check remains the same semantic constraint after
            // stroke fairing; only ordinary visual guides are allowed to move.
            const int index = sourceIndices[essential];
            fitted[essential].localGuide = localAt(index) / scale;
            const auto tangent = ((localAt(index + 2) - localAt(index - 2)) / scale).normalized();
            fitted[essential].extra.insert("curveTangent",
                                           QJsonArray{tangent.x(), tangent.y(), tangent.z()});
            fitted[essential].extra.insert("raceCheckpoint", true);
            fitted[essential].extra.insert("checkpointWidth", .8);
            fitted[essential].extra.insert("checkpointHeight", .8);
            fitted[essential].extra.insert("maneuver", action);
        }
    }
    for (auto &step : fitted)
        step.extra.insert("recognition", QJsonObject{{"source", "occlusion-progressive-stroke"},
                                                     {"requiresReview", true},
                                                     {"depthEstimated", true}});
    evidence->insert("occlusionFit", true);
    evidence->insert("sourceCurveFit", centreline.evidence(cell));
    evidence->insert("occlusionEvents", sourceEvents);
    evidence->insert("sourceCurve3D", sourceSamples);
    evidence->insert("progressiveTurns", transitions);
    evidence->insert("manualOrbitChoiceRequired", false);
    evidence->insert("passageAmbiguous", false);
    evidence->insert("clearanceAdjusted", adjusted);
    evidence->insert("depthEstimated", true);
    evidence->insert("maneuverPattern", "source-progressive-wrap");
    refineFittedStroke(gate, &fitted, evidence);
    *steps = fitted;
    return true;
}

bool fitSideUprightStroke(const Gate &gate, const DiagramNode &node, const PrintedTrace &trace,
                          const QVector<WindowVisit> &visits, double cell,
                          QVector<RouteStep> *steps, QJsonObject *evidence) {
    if (!node.edgeOn || visits.size() != 2 || visits[0].level == visits[1].level)
        return false;
    const auto m = gateFrameMetrics(gate);
    const QRectF box(blobBounds(node.shape));
    const auto crossing = [&](const WindowVisit &v) {
        int at = v.first;
        for (int i = v.first; i <= v.last; ++i)
            if (std::abs(trace.points[i].x() - box.center().x()) <
                std::abs(trace.points[at].x() - box.center().x()))
                at = i;
        return at;
    };
    const int first = crossing(visits[0]), last = crossing(visits[1]);
    if (last - first < cell)
        return false;
    const auto a = trace.points[first], b = trace.points[last];
    const double sign = std::sin(qDegreesToRadians(gate.yaw));
    QVector<double> depths;
    double positive = 0, negative = 0;
    int maximum = 0, minimum = 0;
    bool hiddenFirst = false, hiddenLast = false, visibleMiddle = false;
    for (int i = first; i <= last; ++i) {
        const double t = double(i - first) / (last - first);
        const double z =
            (trace.points[i].x() - a.x() * (1 - t) - b.x() * t) * sign * gate.width / cell;
        depths.push_back(z);
        if (z > positive) {
            positive = z;
            maximum = i;
        }
        if (z < negative) {
            negative = z;
            minimum = i;
        }
        if (i < first + cell * .35)
            hiddenFirst |= trace.hidden[i] == 1;
        if (i > last - cell * .35)
            hiddenLast |= trace.hidden[i] == 1;
        if (i > first + (last - first) * .3 && i < first + (last - first) * .7)
            visibleMiddle |= trace.hidden[i] == 0;
    }
    const int occlusionPadding = std::max(3, int(cell * .40));
    for (int i = std::max(0, first - occlusionPadding);
         i <= std::min(int(trace.hidden.size()) - 1, first + occlusionPadding); ++i)
        hiddenFirst |= trace.hidden[i] == 1;
    for (int i = std::max(0, last - occlusionPadding);
         i <= std::min(int(trace.hidden.size()) - 1, last + occlusionPadding); ++i)
        hiddenLast |= trace.hidden[i] == 1;
    if (positive < gate.width * .12 || negative > -gate.width * .12 ||
        (!hiddenFirst && !hiddenLast) || !visibleMiddle)
        return false;
    const int peak1 = std::min(maximum, minimum), peak2 = std::max(maximum, minimum);
    const int winding = maximum < minimum ? 1 : -1;
    const QVector3D scale(float(m.columns * gate.width / 2), float(m.levels * gate.height / 2),
                          float(gate.width / 2));
    const double barX = -gate.width / 2, radius = gate.width / 2;
    const StrokeCentreline centreline(trace, cell);
    const double tangentSpan = std::max(3., cell * .12);
    const auto localAt = [&](double index, bool denoise = false) {
        index = std::clamp(index, double(first), double(last));
        const int at = int(index), next = std::min(at + 1, last);
        const double fraction = index - at;
        const double z = depths[at - first] * (1 - fraction) + depths[next - first] * fraction;
        const double normalized = z / (z >= 0 ? positive : -negative);
        const double branch = index < peak1 || index > peak2 ? 1 : -1;
        const double x =
            barX + branch * radius * std::sqrt(std::max(0.0, 1 - normalized * normalized));
        const auto pixel = denoise
                               ? centreline.at(index)
                               : trace.points[at] * (1 - fraction) + trace.points[next] * fraction;
        const double y = (box.center().y() - pixel.y()) * gate.height * m.levels / box.height();
        return QVector3D(float(x), float(y), float(z));
    };
    QVector<RouteStep> fitted;
    QJsonArray samples;
    int essential = -1;
    double outer = 0, sweep = 0, previous = 0;
    const int spacing = std::max(3, int(cell * .12));
    for (int i = first; i <= last; ++i) {
        if (i != first && i != last && (i - first) % spacing != 0)
            continue;
        const auto p = localAt(i, i != first && i != last);
        RouteStep step;
        step.id = newGateId();
        step.gateId = gate.id;
        auto tangent = ((localAt(i + tangentSpan, true) - localAt(i - tangentSpan, true)) / scale)
                           .normalized();
        if (i == first || i == last) {
            step.level = i == first ? visits[0].level : visits[1].level;
            step.face = winding > 0 ? "front" : "back";
            const double cy = (step.level - (m.levels - 1) / 2.0) * gate.height;
            step.extra.insert("apertureU", 0);
            step.extra.insert("apertureV",
                              std::clamp((p.y() - cy) / (m.openingHeight / 2), -.84, .84));
            step.extra.insert("flightTangent", QJsonArray{tangent.x(), tangent.y(), tangent.z()});
        } else {
            step.kind = "guide";
            step.localGuide = p / scale;
            step.extra.insert("curveTangent", QJsonArray{tangent.x(), tangent.y(), tangent.z()});
            if (p.x() < outer) {
                outer = p.x();
                essential = fitted.size();
            }
        }
        step.extra.insert("curveMode", "arc-v1");
        step.extra.insert("recognition", QJsonObject{{"source", "side-view-occluded-upright"},
                                                     {"alongBarExtentInferred", true},
                                                     {"requiresReview", true}});
        const double angle = std::atan2(p.z(), p.x() - barX);
        if (i > first)
            sweep += std::remainder(angle - previous, 2 * Pi);
        previous = angle;
        const auto normalized = p / scale;
        samples.push_back(QJsonArray{normalized.x(), normalized.y(), normalized.z()});
        fitted.push_back(step);
    }
    QJsonObject action{{"type", "progressive-upright-wrap"},
                       {"axis", "y"},
                       {"barX", barX},
                       {"fromComponentPass", 1},
                       {"toComponentPass", 2},
                       {"signedSweepDegrees", qRadiansToDegrees(sweep)},
                       {"axialProgress", localAt(last).y() - localAt(first).y()},
                       {"view", "along-local+Y; +X right; +Z up"},
                       {"alongBarExtentInferred", true},
                       {"sameApertureReturn", false}};
    if (essential >= 0) {
        // Keep the outside-post check's position and accepted direction stable.
        // Its source index is recoverable from this fixed-spacing sample list.
        const int index = first + essential * spacing;
        fitted[essential].localGuide = localAt(index) / scale;
        const auto tangent = ((localAt(index + 2) - localAt(index - 2)) / scale).normalized();
        fitted[essential].extra.insert("curveTangent",
                                       QJsonArray{tangent.x(), tangent.y(), tangent.z()});
        fitted[essential].extra.insert("raceCheckpoint", true);
        fitted[essential].extra.insert("checkpointWidth", .8);
        fitted[essential].extra.insert("checkpointHeight", .8);
        fitted[essential].extra.insert("maneuver", action);
    }
    evidence->insert("sourceCurve3D", samples);
    evidence->insert("sourceCurveFit", centreline.evidence(cell));
    evidence->insert("progressiveTurns", QJsonArray{action});
    evidence->insert("occlusionFit", true);
    evidence->insert("alongBarExtentInferred", true);
    evidence->insert("endpointOcclusionComplete", hiddenFirst && hiddenLast);
    evidence->insert("passageAmbiguous", true);
    evidence->insert("manualOrbitChoiceRequired", false);
    evidence->insert("maneuverPattern", "source-progressive-upright");
    refineFittedStroke(gate, &fitted, evidence);
    *steps = fitted;
    return true;
}

QVector<RouteStep> traceComponentPasses(Gate &gate, const DiagramNode &node,
                                        const PrintedTrace &trace, double cell, int *uncertain) {
    const auto metrics = gateFrameMetrics(gate);
    const QRectF box(blobBounds(node.shape));
    QVector<WindowVisit> visits;
    int firstInside = -1, lastInside = -1;
    const bool cube =
        gate.type == QStringLiteral("cube") || gate.type == QStringLiteral("cube-stack");
    const double yaw = qDegreesToRadians(gate.yaw);
    for (int i = 0; i < trace.points.size(); ++i) {
        const auto p = trace.points[i];
        if (!box.contains(p))
            continue;
        if (firstInside < 0)
            firstInside = i;
        lastInside = i;
        if (cube)
            continue;
        const double cellWidth = box.width() / metrics.columns,
                     cellHeight = box.height() / metrics.levels;
        const int imageColumn =
                      std::clamp(int((p.x() - box.left()) / cellWidth), 0, metrics.columns - 1),
                  imageRow =
                      std::clamp(int((p.y() - box.top()) / cellHeight), 0, metrics.levels - 1);
        // The central aperture excludes a line merely skimming a stock beam.
        // On a side-view symbol, interpolated occlusion samples still cross it.
        const QRectF core(box.left() + (imageColumn + .18) * cellWidth,
                          box.top() + (imageRow + .18) * cellHeight, cellWidth * .64,
                          cellHeight * .64);
        if (!core.contains(p))
            continue;
        const int column = std::cos(yaw) < -.5 ? metrics.columns - 1 - imageColumn : imageColumn,
                  level = metrics.levels - 1 - imageRow;
        if (!gateCellOpen(gate, column, level))
            continue;
        if (!visits.isEmpty() && visits.last().column == column && visits.last().level == level) {
            visits.last().interrupted |= i - visits.last().last > cell * .6;
            visits.last().last = i;
        } else
            visits.push_back({column, level, i, i});
    }
    auto evidence = gate.extra.value(QStringLiteral("recognition")).toObject();
    QJsonArray observed;
    for (const auto &visit : visits)
        observed.push_back(QJsonObject{{QStringLiteral("column"), visit.column},
                                       {QStringLiteral("level"), visit.level},
                                       {QStringLiteral("traceStart"), visit.first},
                                       {QStringLiteral("traceEnd"), visit.last}});
    evidence.insert(QStringLiteral("observedWindows"), observed);
    evidence.insert(QStringLiteral("routeEvidence"), QStringLiteral("original-magenta-skeleton"));
    evidence.insert(QStringLiteral("traceOcclusions"), trace.occlusions);
    QVector<RouteStep> sourceSteps;
    if (fitOccludedFrontalStroke(gate, node, trace, visits, cell, &sourceSteps, &evidence)) {
        gate.extra.insert("recognition", evidence);
        return sourceSteps;
    }
    if (fitSideUprightStroke(gate, node, trace, visits, cell, &sourceSteps, &evidence)) {
        gate.extra.insert("recognition", evidence);
        ++*uncertain;
        return sourceSteps;
    }
    QString pattern;
    QString cubeBodyFace = QStringLiteral("front");
    int cubeBodyLevel = 0;
    bool ambiguous = trace.ambiguous || firstInside < 0;
    for (const auto &visit : visits)
        ambiguous |= visit.interrupted;
    if (cube && firstInside >= 0 && lastInside > firstInside) {
        const auto entry = trace.points[firstInside], exit = trace.points[lastInside];
        // A roof/body transition must be visible. The physical cube type alone
        // is never sufficient evidence for a gravity maneuver.
        const bool entryRoof =
            entry
                .y()<box.top() + box.width() * .40,
                     exitRoof = exit.y() < box.top() + box.width() * .40, entryBody = entry.y()>
                    box.top() +
            box.width() * .45,
                   exitBody = exit.y() > box.top() + box.width() * .45;
        if (entryRoof && exitBody)
            pattern = QStringLiteral("gravity-dive");
        else if (entryBody && exitRoof)
            pattern = QStringLiteral("gravity-climb");
        else
            ambiguous = true;
        evidence.insert(QStringLiteral("roofBodyTransition"), !pattern.isEmpty());
        evidence.insert(QStringLiteral("entryPixel"), QJsonArray{entry.x(), entry.y()});
        evidence.insert(QStringLiteral("exitPixel"), QJsonArray{exit.x(), exit.y()});
        if (!pattern.isEmpty() && metrics.levels > 1) {
            const double bodyY = pattern == QStringLiteral("gravity-dive") ? exit.y() : entry.y();
            const double frontTop = box.top() + box.width() * .22;
            const double rowHeight = (box.bottom() - frontTop) / metrics.levels;
            cubeBodyLevel = metrics.levels - 1 -
                            std::clamp(int((bodyY - frontTop) / rowHeight), 0, metrics.levels - 1);
            evidence.insert(QStringLiteral("observedBodyLevel"), cubeBodyLevel);
        }
        // Keep the glyph's physical orientation. An exit through its left wall
        // is a LEFT face, not a rotated structure relabelled as a front exit.
        if (!pattern.isEmpty()) {
            gate.yaw = 0;
            const auto body = pattern == QStringLiteral("gravity-dive") ? exit : entry;
            if (body.x() < box.left() + box.width() * .30)
                cubeBodyFace = "left";
            else if (body.x() > box.right() - box.width() * .30)
                cubeBodyFace = "right";
            evidence.insert("entryFace", pattern == "gravity-dive" ? "top" : cubeBodyFace);
            evidence.insert("exitFace", pattern == "gravity-dive" ? cubeBodyFace : "top");
        }
    } else if (metrics.levels == 2 && visits.size() == 2 && visits[0].level != visits[1].level) {
        pattern =
            visits[0].level == 0 ? QStringLiteral("lower-upper") : QStringLiteral("upper-lower");
    } else if (metrics.columns > 1 && visits.size() == metrics.columns) {
        bool ascending = true, descending = true;
        for (int i = 0; i < visits.size(); ++i) {
            ascending &= visits[i].column == i;
            descending &= visits[i].column == metrics.columns - 1 - i;
        }
        if (ascending)
            pattern = QStringLiteral("left-right");
        else if (descending)
            pattern = QStringLiteral("right-left");
    } else if (metrics.columns == 1 && metrics.levels == 1 && !cube) {
        pattern = QStringLiteral("single");
    }
    QVector<RouteStep> steps;
    if (!pattern.isEmpty())
        steps = gateManeuverSteps(gate, pattern);
    if (cube && !pattern.isEmpty()) {
        const auto bodyPixel = trace.points[pattern == "gravity-dive" ? lastInside : firstInside];
        const double projectedDepth = std::max(cell * .20, box.width() - cell);
        const double zBody =
            (bodyPixel.x() - box.left() - projectedDepth / 2) * gate.width / projectedDepth;
        const double yBody =
            (box.bottom() - projectedDepth / 2 - bodyPixel.y()) * gate.height / cell -
            metrics.levels * gate.height / 2;
        const QVector3D anchorScale(float(gate.width / 2), float(metrics.levels * gate.height / 2),
                                    float(gate.width / 2));
        for (auto &step : steps) {
            if (step.kind == "guide" && cubeBodyFace != "front") {
                const auto p = step.localGuide;
                step.localGuide = cubeBodyFace == "left" ? QVector3D(p.z(), p.y(), -p.x())
                                                         : QVector3D(-p.z(), p.y(), p.x());
                auto t = step.extra.value("curveTangent").toArray();
                if (t.size() == 3) {
                    const double x = t[0].toDouble(), z = t[2].toDouble();
                    t[0] = cubeBodyFace == "left" ? z : -z;
                    t[2] = cubeBodyFace == "left" ? -x : x;
                    step.extra.insert("curveTangent", t);
                }
            } else if (step.kind == "pass" && step.face == "front") {
                step.face = cubeBodyFace;
                if (cubeBodyFace != "front") {
                    const double u =
                        (cubeBodyFace == "left" ? -zBody : zBody) / (metrics.openingWidth / 2);
                    const double cy = (step.level - (metrics.levels - 1) / 2.0) * gate.height;
                    step.extra.insert("apertureU", std::clamp(u, -.84, .84));
                    step.extra.insert(
                        "apertureV",
                        std::clamp((yBody - cy) / (metrics.openingHeight / 2), -.84, .84));
                }
            }
            if (step.kind == "pass") {
                const double sign = pattern == "gravity-dive" ? -1 : 1;
                if (step.face == "top") {
                    const double roofY =
                        box.top() + projectedDepth / 2 + (metrics.levels - 1 - step.level) * cell;
                    int closest = firstInside;
                    double distance = std::numeric_limits<double>::infinity();
                    for (int i = firstInside; i <= lastInside; ++i) {
                        const double d = std::abs(trace.points[i].y() - roofY);
                        if (d < distance) {
                            distance = d;
                            closest = i;
                        }
                    }
                    const auto p = trace.points[closest];
                    const double z = -(p.y() - roofY) * gate.width / projectedDepth;
                    const double x = (p.x() - box.center().x() - z * projectedDepth / gate.width) *
                                     gate.width / cell;
                    step.extra.insert("apertureU",
                                      std::clamp(x / (metrics.openingWidth / 2), -.84, .84));
                    step.extra.insert("apertureV",
                                      std::clamp(z / (metrics.openingHeight / 2), -.84, .84));
                }
                QVector3D t =
                    step.face == "top" ? QVector3D(cubeBodyFace == "left"    ? float(sign * .2)
                                                   : cubeBodyFace == "right" ? float(-sign * .2)
                                                                             : 0,
                                                   float(sign), 0)
                    : cubeBodyFace == "left"  ? QVector3D(float(sign), float(sign * .20), 0)
                    : cubeBodyFace == "right" ? QVector3D(float(-sign), float(sign * .20), 0)
                                              : QVector3D(0, float(sign * .20), float(sign));
                t = (t / anchorScale).normalized();
                step.extra.insert("flightTangent", QJsonArray{t.x(), t.y(), t.z()});
            }
        }
    }
    if (cube && cubeBodyLevel > 0 && !steps.isEmpty()) {
        QVector<RouteStep> visible;
        for (auto step : steps) {
            if (step.kind == QStringLiteral("guide"))
                step.localGuide.setY(step.localGuide.y() +
                                     float(2.0 * cubeBodyLevel / metrics.levels));
            else if (step.face == cubeBodyFace)
                step.level = cubeBodyLevel;
            else if (step.level < cubeBodyLevel)
                continue;
            visible.push_back(step);
        }
        steps = visible;
        ambiguous = true;
    }
    if (steps.isEmpty()) {
        // Partial evidence produces only the actually observed windows. Missing
        // cells are flagged for correction, rather than filled by a type preset.
        ambiguous = true;
        if (visits.isEmpty())
            visits.push_back({0, gate.type == "ladder-blocked" ? 1 : 0,
                              std::max(0, firstInside), std::max(0, lastInside)});
        for (int i = 0; i < visits.size(); ++i) {
            RouteStep step;
            step.id = newGateId();
            step.gateId = gate.id;
            step.column = visits[i].column;
            step.level = visits[i].level;
            step.face = i % 2 ? QStringLiteral("back") : QStringLiteral("front");
            steps.push_back(step);
            if (i + 1 < visits.size()) {
                RouteStep guide;
                guide.id = newGateId();
                guide.gateId = gate.id;
                guide.kind = QStringLiteral("guide");
                guide.localGuide = QVector3D(
                    float((visits[i].column + visits[i + 1].column + 1.0 - metrics.columns) /
                          metrics.columns),
                    float((visits[i].level + visits[i + 1].level + 1.0 - metrics.levels) /
                          metrics.levels),
                    i % 2 ? -.8f : .8f);
                steps.push_back(guide);
            }
        }
    }
    // Printed symbol elevation is not a metric depth measurement. Preserve its
    // observed ordering, while making the nominal 3D turn depth explicitly editable.
    const bool estimatedDepth = metrics.columns > 1 || metrics.levels > 1 || cube;
    bool reverseInitial = false;
    if (!cube && !visits.isEmpty() && !trace.points.isEmpty()) {
        const auto &first = visits.front();
        const int at = first.first, before = std::max(0, at - int(cell * .30)),
                  after = std::min(int(trace.points.size()) - 1, at + int(cell * .30));
        const auto d = trace.points[after] - trace.points[before];
        const double signedDirection = node.edgeOn ? d.x() * std::sin(yaw) : d.y() * std::cos(yaw);
        reverseInitial = signedDirection < 0;
        if (std::abs(signedDirection) < 1)
            ambiguous = true;
    }
    for (auto &step : steps) {
        if (reverseInitial) {
            if (step.kind == QStringLiteral("pass"))
                step.reverse = !step.reverse;
            else {
                step.localGuide.setZ(-step.localGuide.z());
                auto tangent = step.extra.value(QStringLiteral("curveTangent")).toArray();
                if (tangent.size() == 3) {
                    tangent[2] = -tangent[2].toDouble();
                    step.extra.insert(QStringLiteral("curveTangent"), tangent);
                }
            }
        }
    }
    const bool fittedWrap = sourceCrossbarWrap(gate, node, trace, visits, cell, &steps, &evidence);
    if (!fittedWrap && visits.size() > 1 && (metrics.columns > 1 || metrics.levels > 1)) {
        // A frontal projection does not distinguish height from near/far depth.
        // Retain the observed curve and expose a manual orbit choice instead of
        // silently declaring its screen-space winding to be a 3D direction.
        QJsonArray transitions;
        for (int j = 1; j < visits.size(); ++j) {
            const int first = visits[j - 1].first, last = visits[j].first;
            if (last <= first)
                continue;
            QJsonArray samples;
            for (int i = first; i <= last; i += std::max(1, (last - first) / 40)) {
                const auto p = trace.points[i];
                samples.push_back(QJsonArray{(p.x() - box.center().x()) / (box.width() / 2),
                                             (box.center().y() - p.y()) / (box.height() / 2)});
            }
            transitions.push_back(
                QJsonObject{{QStringLiteral("type"), QStringLiteral("projected-bar-turn")},
                            {QStringLiteral("axisCandidate"),
                             metrics.levels > 1 ? QStringLiteral("x-or-y") : QStringLiteral("y")},
                            {QStringLiteral("view"), QStringLiteral("source-image; X right; Y up")},
                            {QStringLiteral("sourceCurve"), samples},
                            {QStringLiteral("windingInferred"), false},
                            {QStringLiteral("manualOrbitChoiceRequired"), true}});
        }
        evidence.insert(QStringLiteral("projectedTurns"), transitions);
        evidence.insert(QStringLiteral("wrapWindingInferred"), false);
        evidence.insert(QStringLiteral("manualOrbitChoiceRequired"), true);
        ambiguous = true;
    }
    for (auto &step : steps) {
        step.extra.insert(
            QStringLiteral("recognition"),
            QJsonObject{{QStringLiteral("source"), QStringLiteral("printed-line-trace")},
                        {QStringLiteral("confidence"), ambiguous ? .45 : .78},
                        {QStringLiteral("requiresReview"), true},
                        {QStringLiteral("ambiguous"), ambiguous},
                        {QStringLiteral("guideDepthEstimated"), estimatedDepth}});
    }
    evidence.insert(QStringLiteral("maneuverPattern"), pattern);
    evidence.insert(QStringLiteral("passageAmbiguous"), ambiguous);
    evidence.insert(QStringLiteral("guideDepthEstimated"), estimatedDepth);
    evidence.insert(QStringLiteral("compoundPassesNeedReview"), estimatedDepth);
    gate.extra.insert(QStringLiteral("recognition"), evidence);
    *uncertain += ambiguous;
    return steps;
}

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
                qAlpha(line[x]) >= 100 && r > 90 && r > g * 1.45 && b > g * 1.18 &&
                b > std::max(35., r * .38);
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
    // Label whole ink objects before applying the route context. Clipping the
    // mask to the course ROI would truncate a launch pad extending beyond it.
    for (int y = 0; y < height; ++y) {
        const auto *line = reinterpret_cast<const QRgb *>(working.constScanLine(y));
        for (int x = 0; x < width; ++x) {
            const int r = qRed(line[x]), g = qGreen(line[x]), b = qBlue(line[x]);
            const int maximum = std::max({r, g, b});
            dark[size_t(y * width + x)] =
                qAlpha(line[x]) >= 100 && maximum < 185 &&
                (maximum < 100 || (b - r >= 12 && b - g >= 8));
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
        if (!field.contains(boxCenter(s).toPoint()))
            continue;
        const int major = std::max(s.width(), s.height()), minor = std::min(s.width(), s.height());
        if (major < 12 || major > std::min(width, height) * .45)
            continue;
        const double fill = double(s.count) / (s.width() * s.height());
        if (fill > .90 || (fill > .75 && s.width() > s.height() * 4)) {
            platforms.push_back(s);
            continue;
        }
        auto holes = enclosedHoles(shapeInk, width, s);
        const double ratio = double(s.width()) / s.height();
        if (holes.size() == 1 && ratio > .7 && ratio < 1.4 &&
            holes[0] > s.width() * s.height() * .20 && minor >= 10)
            cellSizes.push_back(std::min(s.width(), s.height()));
        shapes.push_back({s, std::move(holes), fill});
    }
    if (cellSizes.size() < 3) {
        int framed = 0;
        for (const auto &shape : shapes)
            framed += !shape.holes.empty();
        // A coloured route with several dark frames is a schematic even when
        // its calibration fails. Blue dimension labels must not become pillars
        // through the unrelated editor-icon fallback.
        if (framed >= 3) {
            attempt.applicable = true;
            attempt.result.document = base;
            attempt.result.document.extra.insert(
                QStringLiteral("recognition"),
                QJsonObject{{"method", "local-ink-unresolved"}, {"requiresReview", true}});
            attempt.result.warnings << QStringLiteral(
                "检测到深色门框和洋红路线，但无法可靠确定图示尺寸；请裁剪赛道区域后重新导入。"
                "未将文字或尺寸标注当作门架生成。");
        }
        return attempt;
    }
    std::sort(cellSizes.begin(), cellSizes.end());
    const double cellPixels = cellSizes[cellSizes.size() / 2];
    std::vector<double> centreSpans;
    for (const auto &shape : shapes) {
        const auto &s = shape.component;
        const double ratio = double(s.width()) / s.height();
        if (shape.holes.size() != 1 || ratio < .90 || ratio > 1.10 ||
            s.width() < cellPixels * .8 || s.width() > cellPixels * 1.2)
            continue;
        const double span = frameCentreSpan(dark, width, s);
        if (span >= cellPixels * .65 && span <= cellPixels)
            centreSpans.push_back(span);
    }
    std::sort(centreSpans.begin(), centreSpans.end());
    const double metricCellPixels = centreSpans.size() >= 3
                                        ? centreSpans[centreSpans.size() / 2]
                                        : cellPixels;
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
        if (major / minor > 2.3 && minor < cellPixels * .8) {
            node.edgeOn = true;
            node.type =
                major > cellPixels * 2.05 ? QStringLiteral("ladder") : QStringLiteral("standard");
            node.confidence = .68;
        } else if (bw > cellPixels * 1.5 && bh > cellPixels * 1.5 &&
                   bw < cellPixels * 2.5 && bh < cellPixels * 2.5 &&
                   object.holes.size() == 4 && object.holes.front() < object.holes.back() * 1.65) {
            node.type = QStringLiteral("quad");
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
            node.type = object.holes.size() == 1 && object.fill > .55
                            ? QStringLiteral("ladder-blocked")
                            : QStringLiteral("ladder");
        } else if (bw > cellPixels * .6 && bw < cellPixels * 1.5 && bh > cellPixels * .6 &&
                   bh < cellPixels * 1.6 && !object.holes.empty()) {
            node.type = QStringLiteral("standard");
        } else if (object.holes.empty() && bw > cellPixels * 1.2 && bw < cellPixels * 1.8 &&
                   bh > cellPixels * .9 && bh < cellPixels * 1.6 && object.fill < .65) {
            node.type = QStringLiteral("start");
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
        document.metersPerPixel =
            std::clamp(3.0 / (metricCellPixels * (sx + sy) / 2), .00001, 1000.0);
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
    double bestStart = cellPixels * cellPixels * 16;
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
    if (startKnown) {
        document.takeoff.enabled = true;
        document.takeoff.position =
            document.backgroundOrigin +
            QPointF(startReference.x() * sx, startReference.y() * sy) * document.metersPerPixel;
        document.takeoff.elevation = 0;
        const auto direction = boxCenter(nodes[size_t(start)].shape) - startReference;
        document.takeoff.yaw = qRadiansToDegrees(std::atan2(direction.x(), direction.y()));
        document.takeoff.extra.insert(
            QStringLiteral("recognition"),
            QJsonObject{{QStringLiteral("source"), QStringLiteral("filled-or-grid-launch-platform")},
                        {QStringLiteral("pixelPosition"),
                         QJsonArray{startReference.x() * sx, startReference.y() * sy}},
                        {QStringLiteral("requiresReview"), true}});
    }
    const auto trace = tracePrintedLine(route, width, height, dark, field, nodes, cellPixels,
                                        startReference, boxCenter(nodes[size_t(start)].shape));
    // Connectivity alone cannot decide which branch of a crossing is visited
    // first. When the directed shaft walk covers every symbol, use its actual
    // first encounters instead of a greedy tour through the adjacency graph.
    QVector<int> traceFirst(count, -1), encounters;
    for (int i = 0; i < trace.points.size(); ++i) {
        int nearest = -1;
        double nearestDistance = std::numeric_limits<double>::infinity();
        for (int n = 0; n < count; ++n) {
            const QRectF box(blobBounds(nodes[size_t(n)].shape));
            const auto core = box.adjusted(box.width() * .16, box.height() * .16,
                                           -box.width() * .16, -box.height() * .16);
            if (core.contains(trace.points[i])) {
                if (traceFirst[n] < 0)
                    traceFirst[n] = i;
                const double distance = squaredDistance(trace.points[i], box.center());
                if (distance < nearestDistance) {
                    nearestDistance = distance;
                    nearest = n;
                }
            }
        }
        if (nearest >= 0 && (encounters.isEmpty() || encounters.last() != nearest))
            encounters.append(nearest);
    }
    std::set<int> uniqueEncounters;
    bool interleavedRevisit = false;
    for (int i = 0; i < encounters.size(); ++i) {
        if (i + 1 == encounters.size() && i > 0 && encounters[i] == encounters.first())
            continue;
        interleavedRevisit |= !uniqueEncounters.insert(encounters[i]).second;
    }
    const bool tracedOrder =
        startKnown && !interleavedRevisit && !trace.ambiguous && trace.coverage >= .90 &&
        std::all_of(traceFirst.begin(), traceFirst.end(), [](int index) { return index >= 0; });
    if (tracedOrder)
        std::stable_sort(order.begin(), order.end(),
                         [&](int a, int b) { return traceFirst[a] < traceFirst[b]; });
    const bool tracedCycle =
        tracedOrder && encounters.size() > 1 && encounters.first() == encounters.last();
    // A traced lap may stop on the start frame's boundary before reaching its
    // inner core again. Do not discard an existing, fully connected graph cycle.
    document.closedLoop = cycle || tracedCycle;
    document.routeSteps.clear();
    document.routeStepsDefined = true;
    int ambiguousPassages = 0;
    int compounds = 0, fittedWraps = 0, manualOrbitGroups = 0, progressiveGroups = 0;
    for (int i = 0; i < count; ++i) {
        const auto &node = nodes[size_t(order[size_t(i)])];
        Gate gate = makeGate(node.type);
        const auto pixel = boxCenter(node.shape);
        gate.position = document.backgroundOrigin +
                        QPointF(pixel.x() * sx, pixel.y() * sy) * document.metersPerPixel;
        const int before = i > 0 ? i - 1 : (document.closedLoop ? count - 1 : i);
        const int after = i + 1 < count ? i + 1 : (document.closedLoop ? 0 : i);
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
                        {QStringLiteral("sourceOrdinal"), i + 1},
                        {QStringLiteral("sourceTraceFirstIndex"), traceFirst[order[size_t(i)]]},
                        {QStringLiteral("sourceOrderConfirmed"), tracedOrder},
                        {QStringLiteral("compoundPassesNeedReview"),
                         node.type != QStringLiteral("standard")},
                        {QStringLiteral("pixelBounds"),
                         QJsonArray{node.shape.left * sx, node.shape.top * sy,
                                    node.shape.width() * sx, node.shape.height() * sy}}});
        compounds += node.type != QStringLiteral("standard");
        auto componentSteps =
            traceComponentPasses(gate, node, trace, cellPixels, &ambiguousPassages);
        QJsonObject passage;
        int componentPasses = 0;
        for (auto &step : componentSteps) {
            if (step.kind != "pass")
                continue;
            const bool volume = gateFrameMetrics(gate).centerDepth > 0;
            const QString entry =
                volume ? (step.reverse ? QString("inside") : step.face)
                       : (((step.face == "back") != step.reverse) ? "back" : "front");
            const QString exit = volume ? (step.reverse ? step.face : QString("inside"))
                                        : (entry == "front" ? "back" : "front");
            auto provenance = step.extra.value("recognition").toObject();
            provenance.insert("sourceOrdinal", i + 1);
            provenance.insert("componentPass", ++componentPasses);
            provenance.insert("entrySide", entry);
            provenance.insert("exitSide", exit);
            provenance.insert("sourceOrderConfirmed", tracedOrder);
            step.extra.insert("recognition", provenance);
            if (componentPasses == 1)
                passage.insert("entrySide", entry);
            passage.insert("exitSide", exit);
        }
        passage.insert("passCount", componentPasses);
        auto componentEvidence = gate.extra.value("recognition").toObject();
        passage.insert("turns", componentEvidence.value("progressiveTurns").toArray());
        componentEvidence.insert("passage", passage);
        gate.extra.insert("recognition", componentEvidence);
        document.routeSteps += componentSteps;
        const auto routeEvidence = gate.extra.value(QStringLiteral("recognition")).toObject();
        fittedWraps += routeEvidence.value(QStringLiteral("wrapWindingInferred")).toBool();
        manualOrbitGroups +=
            routeEvidence.value(QStringLiteral("manualOrbitChoiceRequired")).toBool();
        progressiveGroups += routeEvidence.value(QStringLiteral("occlusionFit")).toBool();
        document.gates.push_back(gate);
    }
    result.detectedCount = count;
    document.extra.insert(
        QStringLiteral("recognition"),
        QJsonObject{{QStringLiteral("method"), QStringLiteral("local-ink-route-v2")},
                    {QStringLiteral("orderHeuristic"),
                     tracedOrder ? QStringLiteral("directed-source-trace")
                                 : QStringLiteral("drawn-route-connectivity")},
                    {QStringLiteral("sourceOrderConfirmed"), tracedOrder},
                    {QStringLiteral("interleavedRevisitRequiresReview"), interleavedRevisit},
                    {QStringLiteral("startDetectedFromPlatform"), startKnown},
                    {QStringLiteral("scaleMethod"),
                     centreSpans.size() >= 3 ? QStringLiteral("frame-stroke-centres-3m")
                                             : QStringLiteral("frame-bounds-3m-estimate")},
                    {QStringLiteral("scaleSymbolPixels"), metricCellPixels * (sx + sy) / 2},
                    {QStringLiteral("routeGraphConnected"), connected || tracedOrder},
                    {QStringLiteral("routeTraceCoverage"), trace.coverage},
                    {QStringLiteral("routeTracePoints"), trace.points.size()},
                    {QStringLiteral("routeOcclusionBridges"), trace.occlusions},
                    {QStringLiteral("ambiguousPassageGroups"), ambiguousPassages},
                    {QStringLiteral("sourceFittedWraps"), fittedWraps},
                    {QStringLiteral("manualOrbitChoiceGroups"), manualOrbitGroups},
                    {QStringLiteral("occlusionFittedGroups"), progressiveGroups},
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
    if ((!connected || ambiguousJunction || interleavedRevisit) && !tracedOrder)
        result.warnings << QStringLiteral(
            "部分路线连接或交叉关系不明确，未使用最短巡回代替原路线，请检查通过顺序。");
    if (compounds)
        result.warnings << QStringLiteral(
            "已沿原始洋红线保留复合门的单元/层级通过顺序和重复过门；遮挡续接"
            "及三维转弯深度为估算，实体障碍数不等于通过次数。");
    if (progressiveGroups)
        result.warnings
            << QStringLiteral(
                   "已按可见/遮挡线拟合 %1 组连续绕杆及高度推进，保留实际窗口和斜向通过；"
                   "侧视沿杆范围及未标注的深度半径仍为估算。")
                   .arg(progressiveGroups);
    if (fittedWraps)
        result.warnings
            << QStringLiteral("已从侧视曲线拟合 %1 组横杆绕行弧线；顺逆方向以沿组件本地 +X 轴观察、"
                              "+Z 向右、+Y 向上为准，深度半径仍为估算。")
                   .arg(fittedWraps);
    if (manualOrbitGroups)
        result.warnings << QStringLiteral(
                               "%1 组正视投影无法唯一确定绕竖杆/横杆及顺逆方向；保留原线证据并使用"
                               "待校对弧线，请手动选绕杆动作，未自动增加完整 360° 通过。")
                               .arg(manualOrbitGroups);
    if (ambiguousPassages || trace.ambiguous)
        result.warnings << QStringLiteral("有 %1 组内部路径证据不足或存在投影歧义，已标记为待校对；"
                                          "未按组件类型补齐看不见的单元，请在路线面板修正。")
                               .arg(ambiguousPassages);
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
    document.showBackground = false;
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
