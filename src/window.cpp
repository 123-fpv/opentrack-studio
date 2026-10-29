#include "window.h"
#include "aidialog.h"
#include "imageimport.h"
#include "language.h"
#include "recognizer.h"
#include "scene3d.h"
#include "velocidrone.h"
#include <QDragEnterEvent>
#include <QDropEvent>
#include <QElapsedTimer>
#include <QFutureWatcher>
#include <QImageReader>
#include <QJsonArray>
#include <QJsonDocument>
#include <QMimeData>
#include <QSaveFile>
#include <QScopedValueRollback>
#include <QStandardItemModel>
#include <QtConcurrentRun>
#include <QtWidgets>
#include <cmath>

namespace ot {
namespace {
struct ImageWork {
    RecognitionResult recognition;
    QString error;
    qint64 elapsedMs = 0;
};
void makeRouteExplicit(TrackDocument &document) {
    if (!document.routeStepsDefined && document.routeSteps.isEmpty())
        document.routeSteps = effectiveRouteSteps(document);
    document.routeStepsDefined = true;
}
const Gate *findGate(const TrackDocument &document, const QString &id) {
    for (const auto &gate : document.gates)
        if (gate.id == id)
            return &gate;
    return nullptr;
}
QString passDescription(const Gate &gate, const RouteStep &step) {
    QStringList parts;
    const auto metrics = gateFrameMetrics(gate);
    if (metrics.columns > 1)
        parts << i18n::text("第%1窗").arg(step.column + 1);
    if (metrics.levels > 1)
        parts << (step.level == 0 ? i18n::text("下层") : i18n::text("上层"));
    const QMap<QString, QString> faces{
        {"front", i18n::text("前面")}, {"back", i18n::text("背面")},
        {"left", i18n::text("左面")},  {"right", i18n::text("右面")},
        {"top", i18n::text("顶面")},   {"bottom", i18n::text("底面")}};
    if (metrics.centerDepth > 0)
        parts << faces.value(step.face, step.face);
    if (step.face == "top")
        parts << (step.reverse ? i18n::text("向上穿出") : i18n::text("向下穿入"));
    else if (step.face == "bottom")
        parts << (step.reverse ? i18n::text("向下穿出") : i18n::text("向上穿入"));
    else if (metrics.centerDepth > 0)
        parts << (step.reverse ? i18n::text("向外穿出") : i18n::text("向内穿入"));
    else
        parts << ((step.face == "back") != step.reverse ? i18n::text("背面 → 前面")
                                                        : i18n::text("前面 → 背面"));
    return parts.join(" · ");
}
bool sameDocument(const TrackDocument &a, const TrackDocument &b) {
    if (a.background.cacheKey() != b.background.cacheKey())
        return false;
    auto x = a, y = b;
    x.background = QImage();
    y.background = QImage();
    return toJson(x) == toJson(y);
}
class DocumentCommand final : public QUndoCommand {
    StudioWindow *window_;
    TrackDocument before_, after_;

  public:
    DocumentCommand(StudioWindow *w, QString title, TrackDocument b, TrackDocument a)
        : QUndoCommand(title), window_(w), before_(std::move(b)), after_(std::move(a)) {}
    void undo() override {
        window_->applyDocument(before_);
    }
    void redo() override {
        window_->applyDocument(after_);
    }
};
QLabel *label(const QString &text, const QString &name = {}, QWidget *parent = nullptr) {
    auto *l = new QLabel(text, parent);
    if (!name.isEmpty())
        l->setObjectName(name);
    return l;
}
QPushButton *button(const QString &text, const QString &name = {}) {
    auto *b = new QPushButton(text);
    b->setCursor(Qt::PointingHandCursor);
    if (!name.isEmpty())
        b->setObjectName(name);
    return b;
}
QFrame *line() {
    auto *f = new QFrame;
    f->setFrameShape(QFrame::HLine);
    f->setObjectName("divider");
    return f;
}
QDoubleSpinBox *number(double min, double max, double step = 0.1) {
    auto *s = new QDoubleSpinBox;
    s->setRange(min, max);
    s->setDecimals(2);
    s->setSingleStep(step);
    s->setButtonSymbols(QAbstractSpinBox::NoButtons);
    s->setKeyboardTracking(false);
    return s;
}
class ManeuverPreview final : public QWidget {
    QVector<QVector3D> points_;
    QVector<int> breaks_;
    QVector<GateBeam> bars_;
    QVector<QVector3D> crossings_;

  public:
    explicit ManeuverPreview(QWidget *parent = nullptr) : QWidget(parent) {
        setMinimumHeight(170);
        setMaximumHeight(170);
        setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
        setToolTip(i18n::text("三维动作预览：按编号和箭头连续绕杆，灰色杆件会遮住后方路线。"));
    }
    void setManeuver(const Gate *source, const QString &pattern,
                     const QVector<RouteStep> &sourceSteps) {
        points_.clear();
        breaks_.clear();
        bars_.clear();
        crossings_.clear();
        if (source) {
            Gate gate = *source;
            gate.position = {};
            gate.yaw = 0;
            TrackDocument preview;
            preview.closedLoop = false;
            preview.routeStepsDefined = true;
            preview.gates = {gate};
            preview.routeSteps =
                pattern == "source-route" ? sourceSteps : gateManeuverSteps(gate, pattern);
            const auto curve = routeCurve(preview);
            points_ = curve.points;
            breaks_ = curve.breaks;
            for (auto beam : gateFrameBeams(gate)) {
                beam.center += QVector3D(0, float(gate.elevation), 0);
                bars_.append(beam);
            }
            for (const auto &step : preview.routeSteps)
                if (step.kind == "pass")
                    crossings_.append(routeStepGeometry(preview, step).pathPosition);
        }
        setVisible(source && pattern != "single");
        update();
    }
    void paintEvent(QPaintEvent *) override {
        QPainter painter(this);
        painter.setRenderHint(QPainter::Antialiasing);
        painter.setPen(QColor("#d3d3d3"));
        painter.setBrush(QColor("#fafafa"));
        painter.drawRoundedRect(rect().adjusted(1, 1, -1, -1), 4, 4);
        const auto project = [](const QVector3D &point) {
            return QPointF(.82 * point.x() - .57 * point.z(),
                           -.93 * point.y() + .24 * point.x() + .34 * point.z());
        };
        const auto depth = [](const QVector3D &point) {
            return .53 * point.x() + .36 * point.y() + .67 * point.z();
        };
        QRectF bounds;
        bool initialized = false;
        const auto include = [&](const QVector3D &point) {
            const QRectF one(project(point) - QPointF(.02, .02), QSizeF(.04, .04));
            bounds = initialized ? bounds.united(one) : one;
            initialized = true;
        };
        for (const auto &point : points_)
            include(point);
        for (const auto &bar : bars_)
            for (int corner = 0; corner < 8; ++corner)
                include(bar.center + QVector3D(corner & 1 ? bar.size.x() / 2 : -bar.size.x() / 2,
                                               corner & 2 ? bar.size.y() / 2 : -bar.size.y() / 2,
                                               corner & 4 ? bar.size.z() / 2 : -bar.size.z() / 2));
        if (!initialized)
            return;
        bounds = bounds.adjusted(-.45, -.4, .45, .4);
        const QRectF area = QRectF(rect()).adjusted(14, 24, -14, -12);
        const double scale = std::min(area.width() / std::max(.1, bounds.width()),
                                      area.height() / std::max(.1, bounds.height()));
        const auto pixel = [&](const QPointF &point) {
            return area.center() + (point - bounds.center()) * scale;
        };
        struct Primitive {
            QPolygonF polygon;
            double depth = 0;
            bool stroke = false;
            QColor fill;
        };
        QVector<Primitive> primitives;
        for (const auto &bar : bars_) {
            QVector<QVector3D> corners;
            for (int corner = 0; corner < 8; ++corner)
                corners.append(bar.center +
                               QVector3D(corner & 1 ? bar.size.x() / 2 : -bar.size.x() / 2,
                                         corner & 2 ? bar.size.y() / 2 : -bar.size.y() / 2,
                                         corner & 4 ? bar.size.z() / 2 : -bar.size.z() / 2));
            const int faces[6][4] = {{0, 1, 3, 2}, {4, 6, 7, 5}, {0, 4, 5, 1},
                                     {2, 3, 7, 6}, {0, 2, 6, 4}, {1, 5, 7, 3}};
            for (const auto &face : faces) {
                Primitive item;
                item.fill = QColor("#aaaaaa");
                for (int index : face) {
                    item.polygon.append(pixel(project(corners[index])));
                    item.depth += depth(corners[index]) / 4;
                }
                primitives.append(item);
            }
        }
        for (qsizetype i = 1; i < points_.size(); ++i)
            if (!breaks_.contains(int(i)))
                primitives.append(
                    {QPolygonF{pixel(project(points_[i - 1])), pixel(project(points_[i]))},
                     (depth(points_[i - 1]) + depth(points_[i])) / 2,
                     true,
                     {}});
        for (int fraction : {1, 3, 5, 7}) {
            const qsizetype index = points_.size() * fraction / 8;
            if (index <= 0 || index >= points_.size() || breaks_.contains(int(index)))
                continue;
            const QPointF tip = pixel(project(points_[index]));
            QPointF direction = tip - pixel(project(points_[index - 1]));
            const double length = std::hypot(direction.x(), direction.y());
            if (length < .01)
                continue;
            direction /= length;
            const QPointF side(-direction.y(), direction.x());
            primitives.append(
                {QPolygonF{tip, tip - direction * 5 + side * 2.5, tip - direction * 5 - side * 2.5},
                 depth(points_[index]) + .001, false, QColor("#202020")});
        }
        std::stable_sort(primitives.begin(), primitives.end(),
                         [](const Primitive &a, const Primitive &b) { return a.depth < b.depth; });
        for (const auto &item : primitives) {
            if (item.stroke) {
                painter.setPen(QPen(QColor("#202020"), 1.7, Qt::SolidLine, Qt::RoundCap));
                painter.drawLine(item.polygon[0], item.polygon[1]);
            } else {
                painter.setPen(QPen(item.fill.darker(120), .4));
                painter.setBrush(item.fill);
                painter.drawPolygon(item.polygon);
            }
        }
        for (qsizetype i = 0; i < crossings_.size(); ++i) {
            const QPointF point = pixel(project(crossings_[i]));
            const QRectF badge(point - QPointF(6, 6), QSizeF(12, 12));
            painter.setPen(QPen(QColor("#222222"), .8));
            painter.setBrush(Qt::white);
            painter.drawEllipse(badge);
            painter.drawText(badge, Qt::AlignCenter, QString::number(i + 1));
        }
        painter.setPen(QColor("#555555"));
        painter.drawText(QRect(9, 4, width() - 18, 18),
                         i18n::text("3D 动作 · 按编号和箭头连续通过"));
    }
};
class GateCard final : public QPushButton {
    QString type_, title_, subtitle_;

  public:
    GateCard(QString type, QString title, QString subtitle)
        : type_(type), title_(title), subtitle_(subtitle) {
        setStyleSheet(i18n::english() ? "min-height:112px;max-height:112px;padding:0px;"
                                      : "min-height:104px;max-height:104px;padding:0px;");
        setMinimumWidth(86);
        setCursor(Qt::PointingHandCursor);
        setCheckable(true);
        setAutoExclusive(true);
        setToolTip(title + i18n::text(" · 选中后点击画布放置"));
    }
    void paintEvent(QPaintEvent *) override {
        QPainter p(this);
        p.setRenderHint(QPainter::Antialiasing);
        const bool active = isChecked() || underMouse();
        p.setBrush(active ? QColor("#ededed") : QColor("#ffffff"));
        p.setPen(QPen(active ? QColor("#242424") : QColor("#c9c9c9"), isChecked() ? 2 : 1));
        p.drawRoundedRect(rect().adjusted(1, 1, -1, -1), 3, 3);
        paintGateSymbol(&p, QRectF(width() / 2. - 28, 7, 56, 48), type_);
        QFont f = font();
        f.setPointSize(i18n::english() ? 8 : 9);
        f.setWeight(QFont::DemiBold);
        p.setFont(f);
        p.setPen(QColor("#222222"));
        p.drawText(i18n::english() ? QRect(2, 56, width() - 4, 28) : QRect(0, 64, width(), 18),
                   Qt::AlignCenter | (i18n::english() ? Qt::TextWordWrap : 0), title_);
        f.setWeight(QFont::Normal);
        f.setPointSize(i18n::english() ? 7 : 8);
        p.setFont(f);
        p.setPen(QColor("#707070"));
        p.drawText(i18n::english() ? QRect(2, 87, width() - 4, 22) : QRect(0, 83, width(), 15),
                   Qt::AlignCenter | (i18n::english() ? Qt::TextWordWrap : 0), subtitle_);
    }
};
} // namespace

StudioWindow::StudioWindow() {
    doc_ = demoDocument();
    doc_.name = i18n::text(doc_.name); // Only this newly created, built-in demo name.
    setupUi();
    setupShortcuts();
    canvas_->setDocument(&doc_);
    applyDocument(doc_);
    undo_.setClean();
    setWindowTitle(i18n::text("OpenTrack Studio %1 — VelociDrone 赛道工作台")
                       .arg(QCoreApplication::applicationVersion()));
    setMinimumSize(1140, 740);
    const QRect available = QGuiApplication::primaryScreen()->availableGeometry();
    resize(std::min(1480, available.width() - 40), std::min(900, available.height() - 65));
    move(available.center() - rect().center());
    QTimer::singleShot(100, this, [this] { fitViews(); });
    connect(&undo_, &QUndoStack::cleanChanged, this,
            [this](bool clean) { setWindowModified(!clean); });
}
StudioWindow::~StudioWindow() {
    // Members are destroyed before QObject destroys its child widgets. Remove
    // their callbacks while the document and undo stack are still alive.
    for (auto *child : findChildren<QObject *>())
        QObject::disconnect(child, nullptr, this, nullptr);
    canvas_->onSelection = {};
    canvas_->onGateMoved = {};
    canvas_->onGestureBegin = {};
    canvas_->onGestureEnd = {};
    canvas_->onPlace = {};
    canvas_->onZoom = {};
    canvas_->setPreview(false);
    canvas_->setDocument(nullptr);
    if (scene3d_) {
        scene3d_->onSelection = {};
        scene3d_->onRouteSelection = {};
        scene3d_->onGateMoved = {};
        scene3d_->onGestureBegin = {};
        scene3d_->onGestureEnd = {};
        scene3d_->onPlace = {};
        scene3d_->onZoom = {};
        scene3d_->setPreview(false);
        scene3d_->setDocument(nullptr);
    }
}
void StudioWindow::setupUi() {
    auto *root = new QWidget;
    auto *all = new QVBoxLayout(root);
    all->setContentsMargins(0, 0, 0, 0);
    all->setSpacing(0);
    setCentralWidget(root);
    auto *header = new QFrame;
    header->setObjectName("header");
    header->setFixedHeight(76);
    auto *hl = new QHBoxLayout(header);
    hl->setContentsMargins(24, 10, 24, 10);
    hl->setSpacing(14);
    auto *logo = label("", "logo");
    logo->setFixedSize(42, 42);
    logo->setPixmap(
        QPixmap(":/icons/app.png").scaled(42, 42, Qt::KeepAspectRatio, Qt::SmoothTransformation));
    hl->addWidget(logo);
    auto *titles = new QVBoxLayout;
    titles->setSpacing(1);
    titles->addWidget(label("OpenTrack Studio", "brand"));
    titles->addWidget(label(i18n::text("为 VelociDrone 打造的开源赛道工作台"), "brandSub"));
    hl->addLayout(titles);
    hl->addSpacing(28);
    hl->addWidget(label(i18n::text("赛道编辑"), "headerTab"));
    hl->addStretch();
    hl->addWidget(label(i18n::text("●  本地工程 · 无需登录"), "localBadge"));
    language_ = new QComboBox;
    language_->setObjectName("languageSelector");
    language_->addItem("简体中文", "zh");
    language_->addItem("English", "en");
    language_->setCurrentIndex(i18n::english() ? 1 : 0);
    language_->setToolTip(i18n::text("界面语言"));
    language_->setMinimumContentsLength(7);
    hl->addWidget(language_);
    connect(language_, &QComboBox::currentIndexChanged, this, [this](int index) {
        const QString code = language_->itemData(index).toString();
        QTimer::singleShot(0, this, [this, code] { changeLanguage(code); });
    });
    auto *imp = button(i18n::text("＋  导入图片"), "headerImport");
    auto *importMenu = new QMenu(imp);
    auto *planAction = importMenu->addAction(i18n::text("平面赛道图 · 本地识别"));
    auto *photoAction = importMenu->addAction(i18n::text("实拍 / 航拍照片 · AI 生成"));
    imp->setMenu(importMenu);
    auto *exp = button(i18n::text("导出赛道 .trk  ↗"), "primary");
    hl->addWidget(imp);
    hl->addWidget(exp);
    connect(planAction, &QAction::triggered, this, [this] { importImage(); });
    connect(photoAction, &QAction::triggered, this, [this] { importPhotosWithAi(); });
    connect(exp, &QPushButton::clicked, this, [this] { exportTrack(); });
    all->addWidget(header);
    auto *tools = new QFrame;
    tools->setObjectName("toolbar");
    tools->setFixedHeight(54);
    auto *tl = new QHBoxLayout(tools);
    tl->setContentsMargins(18, 7, 18, 7);
    tl->setSpacing(8);
    auto tool = [&](const QString &text, const QString &tip, const std::function<void()> &fn) {
        auto *b = button(text, "tool");
        b->setToolTip(tip);
        connect(b, &QPushButton::clicked, this, fn);
        tl->addWidget(b);
        return b;
    };
    tool(i18n::text("新建"), "Ctrl+N", [this] { newProject(); });
    tool(i18n::text("打开"), "Ctrl+O", [this] { openProject(); });
    tool(i18n::text("保存工程"), "Ctrl+S", [this] { save(); });
    tl->addSpacing(12);
    auto *undo = tool(i18n::text("↶  撤销"), "Ctrl+Z", [this] { undo_.undo(); });
    auto *redo = tool(i18n::text("↷  重做"), "Ctrl+Shift+Z", [this] { undo_.redo(); });
    connect(&undo_, &QUndoStack::canUndoChanged, undo, &QPushButton::setEnabled);
    connect(&undo_, &QUndoStack::canRedoChanged, redo, &QPushButton::setEnabled);
    undo->setEnabled(undo_.canUndo());
    redo->setEnabled(undo_.canRedo());
    tl->addSpacing(12);
    tool(i18n::text("选择 / 移动"), "Esc", [this] { setPlacement({}); });
    tool(i18n::text("设置起飞点"), i18n::text("独立起飞架"), [this] {
        setPlacement("__takeoff", .8, .1);
        notice_->setText(i18n::text("在地面单击放置独立起飞点；它不占用过门编号。Esc 取消。"));
    });
    tool(i18n::text("复制选择"), "Ctrl+D", [this] {
        if (!selectedRouteStep_.isEmpty())
            duplicateRoutePass();
        else
            duplicateGate();
    });
    tool(i18n::text("删除选择"), "Delete", [this] {
        if (!selectedRouteStep_.isEmpty())
            deleteRoutePass();
        else
            deleteGate();
    });
    tl->addStretch();
    auto *snap = new QCheckBox(i18n::text("0.5 m 吸附"));
    snap->setObjectName("snapToGrid");
    snap->setChecked(true);
    auto *route = new QCheckBox(i18n::text("显示航线"));
    route->setObjectName("showRoute");
    route->setChecked(true);
    tl->addWidget(snap);
    tl->addWidget(route);
    connect(snap, &QCheckBox::toggled, this, [this](bool b) {
        canvas_->setSnap(b);
        if (scene3d_)
            scene3d_->setSnap(b);
    });
    connect(route, &QCheckBox::toggled, this, [this](bool b) {
        canvas_->setRouteVisible(b);
        if (scene3d_)
            scene3d_->setRouteVisible(b);
    });
    tl->addSpacing(10);
    tool(i18n::text("适应视角"), "F", [this] { fitViews(); });
    tool(i18n::text("操作说明"), i18n::text("快速上手"), [this] {
        QMessageBox help(this);
        help.setWindowTitle(i18n::text("OpenTrack Studio · 操作说明"));
        help.setTextFormat(Qt::PlainText);
        help.setText(i18n::text(
            "① 查看赛道\n右键旋转视角，中键平移，滚轮缩放；F 显示整条赛道。\n\n"
            "② 移动组件\n先单击并松开，看到光圈后，再按住组件或脚下光圈拖动。\n"
            "左侧修改中心距、高度、朝向和颜色；转向柱可选择左右绕行。\n"
            "标准门中心距 3 m，杆长 3.2 m，杆件截面 0.1 × 0.1 m。\n\n"
            "③ 从图片生成\n平面赛道图在本地识别；实拍、航拍照片使用 AI 生成草稿，预览后应用。\n\n"
            "④ 保存和使用\n保存工程得到 .otproj，方便继续编辑。\n"
            "导出赛道得到 .trk，在 VelociDrone 的 Track Editor → Import Track 中导入。\n\n"
            "Ctrl+Z 撤销 · Ctrl+D 复制 · Delete 删除 · Q/E 旋转组件"));
        help.setText(
            help.text() + "\n\n" +
            i18n::text(
                "Copyright © 2026 OpenTrack Studio contributors\n"
                "本软件按 GNU GPL-3.0-or-later 授权，可按许可证条款修改和再分发，不提供任何担保。\n"
                "源码与许可证：https://github.com/123-fpv/opentrack-studio"));
        help.setStandardButtons(QMessageBox::Ok);
        help.exec();
    });
    all->addWidget(tools);
    auto *main = new QSplitter(Qt::Horizontal);
    main->setHandleWidth(1);
    all->addWidget(main, 1);
    auto *leftScroll = new QScrollArea;
    leftScroll->setWidgetResizable(true);
    leftScroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    leftScroll->setMinimumWidth(236);
    leftScroll->setMaximumWidth(290);
    leftScroll->setObjectName("sidebar");
    auto *left = new QWidget;
    auto *ll = new QVBoxLayout(left);
    ll->setContentsMargins(18, 20, 18, 18);
    ll->setSpacing(12);
    leftScroll->setWidget(left);
    main->addWidget(leftScroll);
    ll->addWidget(label(i18n::text("工程设置"), "sectionTitle"));
    ll->addWidget(label(i18n::text("赛道名称"), "fieldLabel"));
    name_ = new QLineEdit;
    name_->setMaxLength(120);
    ll->addWidget(name_);
    connect(name_, &QLineEdit::editingFinished, this, [this] {
        if (updating_)
            return;
        const QString value = name_->text().trimmed();
        if (!value.isEmpty() && value != doc_.name)
            edit(i18n::text("修改赛道名称"), [value](TrackDocument &d) { d.name = value; });
    });
    ll->addWidget(label(i18n::text("模拟器场景"), "fieldLabel"));
    scenery_ = new QComboBox;
    scenery_->addItems({"Football Stadium", "Empty Scene Day"});
    ll->addWidget(scenery_);
    connect(scenery_, &QComboBox::currentTextChanged, this, [this](const QString &s) {
        if (!updating_)
            edit(i18n::text("修改场景"), [s](TrackDocument &d) { d.scenery = s; });
    });
    auto *stats = new QFrame;
    stats->setObjectName("statsCard");
    auto *sl = new QHBoxLayout(stats);
    auto *a = new QVBoxLayout;
    gateCount_ = label("18", "statValue");
    a->addWidget(gateCount_);
    a->addWidget(label(i18n::text("实体 / 过门次数"), "subtle"));
    auto *b = new QVBoxLayout;
    length_ = label("0 m", "statValue");
    b->addWidget(length_);
    b->addWidget(label(i18n::text("参考航线长度"), "subtle"));
    sl->addLayout(a);
    sl->addLayout(b);
    ll->addWidget(stats);
    auto *inventoryBox = new QGroupBox(i18n::text("障碍物统计"));
    auto *inventoryLayout = new QVBoxLayout(inventoryBox);
    obstacleInventory_ = label({}, "subtle");
    obstacleInventory_->setObjectName("obstacleInventory");
    obstacleInventory_->setTextFormat(Qt::RichText);
    obstacleInventory_->setWordWrap(true);
    obstacleInventory_->setToolTip(
        i18n::text("按实体分类计数；同一座门多次通过仍只算一座。起飞台单独统计。"));
    inventoryLayout->addWidget(obstacleInventory_);
    ll->addWidget(inventoryBox);
    auto *takeoffBox = new QGroupBox(i18n::text("独立起飞点"));
    auto *takeoffLayout = new QVBoxLayout(takeoffBox);
    takeoffLayout->setSpacing(7);
    takeoffStatus_ = label(i18n::text("尚未设置 · 导出前必须设置"), "subtle");
    takeoffStatus_->setWordWrap(true);
    takeoffLayout->addWidget(takeoffStatus_);
    auto *placeTakeoff = button(i18n::text("在场景放置起飞点"), "secondary");
    placeTakeoff->setObjectName("placeTakeoff");
    takeoffLayout->addWidget(placeTakeoff);
    connect(placeTakeoff, &QPushButton::clicked, this, [this] {
        setPlacement("__takeoff", .8, .1);
        notice_->setText(
            i18n::text("单击地面设置起飞点，再输入朝向；起飞点独立于赛道门和过门步骤。"));
    });
    auto *takeoffCoordinates = new QWidget;
    auto *takeoffForm = new QGridLayout(takeoffCoordinates);
    takeoffForm->setContentsMargins(0, 0, 0, 0);
    takeoffX_ = number(-10000, 10000);
    takeoffY_ = number(-10000, 10000);
    takeoffZ_ = number(-1000, 1000);
    takeoffYaw_ = number(-3600, 3600, 15);
    const QList<QPair<QString, QDoubleSpinBox *>> takeoffFields = {
        {"X / m", takeoffX_},
        {"Y / m", takeoffY_},
        {i18n::text("基点 Z / m"), takeoffZ_},
        {i18n::text("朝向 / °"), takeoffYaw_}};
    for (int i = 0; i < takeoffFields.size(); ++i) {
        takeoffForm->addWidget(label(takeoffFields[i].first, "fieldLabel"), (i / 2) * 2, i % 2);
        takeoffForm->addWidget(takeoffFields[i].second, (i / 2) * 2 + 1, i % 2);
        connect(takeoffFields[i].second, &QDoubleSpinBox::editingFinished, this,
                [this] { editTakeoff(); });
    }
    takeoffBox->setToolTip(
        i18n::text("这里设置原生起飞架的基点和朝向；模拟器会在起飞架上分配起飞位。"));
    auto *takeoffDetails = button(i18n::text("坐标与朝向 ▸"), "tool");
    takeoffDetails->setCheckable(true);
    takeoffLayout->addWidget(takeoffDetails);
    takeoffCoordinates->hide();
    takeoffLayout->addWidget(takeoffCoordinates);
    connect(takeoffDetails, &QPushButton::toggled, takeoffCoordinates, &QWidget::setVisible);
    ll->addWidget(takeoffBox);
    ll->addWidget(line());
    ll->addWidget(label(i18n::text("图片生成赛道"), "sectionTitle"));
    auto *import = button(i18n::text("拖入赛道图片\n或点击选择，自动生成"), "imageDropZone");
    import->setMinimumHeight(60);
    import->setToolTip(i18n::text(
        "支持 PNG / JPG / BMP；也可直接拖到 3D 场景或俯视图。多张照片进入 AI 草稿预览。"));
    dropZone_ = import;
    ll->addWidget(import);
    connect(import, &QPushButton::clicked, this, [this] { importImage(); });
    imageStatus_ = label(i18n::text("支持 PNG / JPG / BMP\n可拖到这里、3D 场景或俯视图"), "subtle");
    imageStatus_->setWordWrap(true);
    ll->addWidget(imageStatus_);
    auto *aiPhotos = button(i18n::text("AI 解析当前图 / 多张照片"), "secondary");
    ll->addWidget(aiPhotos);
    connect(aiPhotos, &QPushButton::clicked, this, [this] { importPhotosWithAi(); });
    auto *advancedToggle = button(i18n::text("比例与底图设置  ▸"), "tool");
    advancedToggle->setCheckable(true);
    ll->addWidget(advancedToggle);
    auto *advanced = new QWidget;
    auto *al = new QVBoxLayout(advanced);
    al->setContentsMargins(0, 0, 0, 0);
    al->setSpacing(9);
    advanced->hide();
    ll->addWidget(advanced);
    connect(advancedToggle, &QPushButton::toggled, advanced, &QWidget::setVisible);
    autoScale_ = new QCheckBox(i18n::text("按中心距 3 m 估算比例"));
    autoScale_->setChecked(true);
    al->addWidget(autoScale_);
    al->addWidget(label(i18n::text("底图比例 · 米 / 像素"), "fieldLabel"));
    scale_ = number(.0001, 10, .001);
    scale_->setDecimals(4);
    al->addWidget(scale_);
    connect(scale_, &QDoubleSpinBox::editingFinished, this, [this] {
        if (updating_ || qFuzzyCompare(scale_->value(), doc_.metersPerPixel))
            return;
        const double n = scale_->value();
        edit(i18n::text("调整底图比例"), [n](TrackDocument &d) {
            const double factor = n / d.metersPerPixel;
            d.backgroundOrigin *= factor;
            for (auto &g : d.gates)
                g.position *= factor;
            d.takeoff.position *= factor;
            for (auto &step : d.routeSteps)
                if (step.kind == "guide" && step.gateId.isEmpty()) {
                    step.localGuide.setX(step.localGuide.x() * float(factor));
                    step.localGuide.setZ(step.localGuide.z() * float(factor));
                }
            d.metersPerPixel = n;
        });
    });
    recognizeButton_ = button(i18n::text("识别图片并生成赛道"), "primary");
    ll->addWidget(recognizeButton_);
    connect(recognizeButton_, &QPushButton::clicked, this, [this] { startRecognition(); });
    imageVisible_ = new QCheckBox(i18n::text("仅在俯视图显示参考图"));
    imageVisible_->setChecked(false);
    al->addWidget(imageVisible_);
    connect(imageVisible_, &QCheckBox::toggled, this, [this](bool b) {
        if (!updating_)
            edit(i18n::text("切换底图"), [b](TrackDocument &d) { d.showBackground = b; });
    });
    al->addWidget(label(i18n::text("底图透明度"), "fieldLabel"));
    opacity_ = new QSlider(Qt::Horizontal);
    opacity_->setRange(0, 100);
    al->addWidget(opacity_);
    connect(opacity_, &QSlider::sliderReleased, this, [this] {
        const double n = opacity_->value() / 100.;
        edit(i18n::text("调整底图透明度"), [n](TrackDocument &d) { d.backgroundOpacity = n; });
    });
    ll->addWidget(line());
    properties_ = new QWidget;
    auto *pl = new QVBoxLayout(properties_);
    pl->setContentsMargins(0, 0, 0, 0);
    pl->setSpacing(10);
    selectionTitle_ = label(i18n::text("门架属性"), "sectionTitle");
    selectionTitle_->setWordWrap(true);
    pl->addWidget(selectionTitle_);
    auto *grid = new QGridLayout;
    grid->setHorizontalSpacing(9);
    grid->setVerticalSpacing(6);
    x_ = number(-10000, 10000);
    z_ = number(-10000, 10000);
    alt_ = number(0, 1000);
    yaw_ = number(-3600, 3600, 15);
    width_ = number(.01, 100, .01);
    height_ = number(.1, 100);
    const QList<QPair<QString, QDoubleSpinBox *>> props = {
        {i18n::text("X 位置 / m"), x_},       {i18n::text("Y 位置 / m"), z_},
        {i18n::text("Z 中心高度 / m"), alt_}, {i18n::text("朝向 / °"), yaw_},
        {i18n::text("宽度 / m"), width_},     {i18n::text("高度 / m"), height_}};
    for (int i = 0; i < props.size(); ++i) {
        auto *field = label(props[i].first, "fieldLabel");
        if (i == 4)
            widthLabel_ = field;
        if (i == 5)
            heightLabel_ = field;
        grid->addWidget(field, (i / 2) * 2, i % 2);
        grid->addWidget(props[i].second, (i / 2) * 2 + 1, i % 2);
        connect(props[i].second, &QDoubleSpinBox::editingFinished, this,
                [this] { editSelection(); });
    }
    pl->addLayout(grid);
    passCell_ = new QWidget;
    auto *passCellLayout = new QHBoxLayout(passCell_);
    passCellLayout->setContentsMargins(0, 0, 0, 0);
    passCellLayout->addWidget(label(i18n::text("通过格"), "fieldLabel"));
    passColumn_ = new QComboBox;
    passLevel_ = new QComboBox;
    passCellLayout->addWidget(passColumn_);
    passCellLayout->addWidget(passLevel_);
    pl->addWidget(passCell_);
    const auto editCell = [this] {
        if (updating_ || selected_.isEmpty())
            return;
        if (!selectedRouteStep_.isEmpty()) {
            editRouteSelection();
            return;
        }
        const QString id = selected_;
        const int column = passColumn_->currentIndex(), level = passLevel_->currentIndex();
        edit(i18n::text("调整组合门通过格"), [id, column, level](TrackDocument &d) {
            for (auto &gate : d.gates)
                if (gate.id == id) {
                    gate.extra.insert("passColumn", std::max(0, column));
                    gate.extra.insert("passLevel", std::max(0, level));
                }
        });
    };
    connect(passColumn_, &QComboBox::currentIndexChanged, this, editCell);
    connect(passLevel_, &QComboBox::currentIndexChanged, this, editCell);
    stepPanel_ = new QWidget;
    auto *stepLayout = new QVBoxLayout(stepPanel_);
    stepLayout->setContentsMargins(0, 0, 0, 0);
    stepSummary_ = label(i18n::text("选择一次穿越，编辑窗口和方向"), "subtle");
    stepSummary_->setWordWrap(true);
    stepLayout->addWidget(stepSummary_);
    stepFace_ = new QComboBox;
    stepReverse_ = new QComboBox;
    stepReverse_->addItems({i18n::text("正向穿越"), i18n::text("反转穿越方向")});
    stepLayout->addWidget(label(i18n::text("穿越面与方向"), "fieldLabel"));
    stepLayout->addWidget(stepFace_);
    stepLayout->addWidget(stepReverse_);
    pl->addWidget(stepPanel_);
    connect(stepFace_, &QComboBox::currentIndexChanged, this, [this] { editRouteSelection(); });
    connect(stepReverse_, &QComboBox::currentIndexChanged, this, [this] { editRouteSelection(); });
    maneuver_ = new QComboBox;
    maneuver_->setObjectName("maneuverPattern");
    auto *applyManeuver = button(i18n::text("应用动作"), "tool");
    applyManeuver->setObjectName("applyManeuver");
    pl->addWidget(label(i18n::text("手动校正过门动作"), "fieldLabel"));
    pl->addWidget(maneuver_);
    maneuverPreview_ = new ManeuverPreview;
    pl->addWidget(maneuverPreview_);
    maneuverHint_ = label({}, "subtle");
    maneuverHint_->setWordWrap(true);
    pl->addWidget(maneuverHint_);
    pl->addWidget(applyManeuver);
    connect(maneuver_, &QComboBox::currentIndexChanged, this, [this] {
        if (!updating_)
            updateManeuverPreview();
    });
    connect(applyManeuver, &QPushButton::clicked, this, [this] { this->applyManeuver(); });
    dimensions_ = label({}, "subtle");
    dimensions_->setObjectName("componentDimensions");
    dimensions_->setWordWrap(true);
    pl->addWidget(dimensions_);
    pl->addWidget(label(i18n::text("组件颜色"), "fieldLabel"));
    auto *colorRow = new QHBoxLayout;
    color_ = new QComboBox;
    color_->setObjectName("componentColor");
    color_->setMinimumWidth(80);
    for (const auto &entry : nativeBlockColors()) {
        QPixmap swatch(18, 18);
        swatch.fill(entry.color);
        color_->addItem(QIcon(swatch), i18n::text(entry.name), entry.color.name());
    }
    color_->addItem(i18n::text("自定义颜色"), QString());
    colorRow->addWidget(color_, 1);
    auto *customColor = button(i18n::text("调色"), "tool");
    customColor->setObjectName("customComponentColor");
    colorRow->addWidget(customColor);
    pl->addLayout(colorRow);
    colorMapping_ = label({}, "subtle");
    colorMapping_->setWordWrap(true);
    pl->addWidget(colorMapping_);
    connect(color_, &QComboBox::currentIndexChanged, this, [this, customColor](int index) {
        if (updating_ || index < 0)
            return;
        const QString value = color_->itemData(index).toString();
        if (value.isEmpty())
            customColor->click();
        else
            editColor(QColor(value));
    });
    connect(customColor, &QPushButton::clicked, this, [this] {
        QColor initial;
        for (const auto &g : doc_.gates)
            if (g.id == selected_)
                initial = gateColor(g);
        if (!initial.isValid())
            return;
        const QColor chosen = QColorDialog::getColor(initial, this, i18n::text("选择组件颜色"));
        if (chosen.isValid())
            editColor(chosen);
        updateSelection(selected_);
    });
    passSideLabel_ = label(i18n::text("绕柱通过侧"), "fieldLabel");
    passSide_ = new QComboBox;
    passSide_->addItems({i18n::text("右侧绕过"), i18n::text("左侧绕过")});
    pl->addWidget(passSideLabel_);
    pl->addWidget(passSide_);
    connect(passSide_, &QComboBox::currentIndexChanged, this, [this](int index) {
        if (updating_ || selected_.isEmpty())
            return;
        const QString id = selected_;
        edit(i18n::text("调整绕柱侧"), [id, index](TrackDocument &d) {
            for (auto &g : d.gates)
                if (g.id == id && g.type == "pillar")
                    g.extra.insert("passSide", index == 0 ? 1 : -1);
        });
    });
    // Keep the selected component's controls visible above project/import settings.
    pl->addWidget(line());
    ll->insertWidget(0, properties_);
    properties_->hide();
    ll->addStretch();
    auto *about =
        label(i18n::text("OPEN SOURCE / GPL-3.0-or-later\n无需注册 · 支持自带 API"), "localNote");
    about->setTextFormat(Qt::RichText);
    about->setOpenExternalLinks(true);
    about->setText(about->text().toHtmlEscaped().replace("\n", "<br>") +
                   "<br><a href=\"https://github.com/123-fpv/opentrack-studio\">" +
                   i18n::text("源码与许可证") + "</a>");
    about->setWordWrap(true);
    ll->addWidget(about);
    auto *center = new QWidget;
    center->setObjectName("center");
    auto *cl = new QVBoxLayout(center);
    cl->setContentsMargins(14, 14, 14, 12);
    cl->setSpacing(10);
    main->addWidget(center);
    auto *ch = new QHBoxLayout;
    ch->addWidget(label(i18n::text("赛道场景"), "sectionTitle"));
    auto *view3 = button(i18n::text("3D 视角"), "viewToggle");
    auto *view2 = button(i18n::text("俯视图"), "viewToggle");
    view3->setCheckable(true);
    view2->setCheckable(true);
    auto *viewGroup = new QButtonGroup(this);
    viewGroup->setExclusive(true);
    viewGroup->addButton(view3);
    viewGroup->addButton(view2);
    view3->setChecked(true);
    ch->addSpacing(10);
    ch->addWidget(view3);
    ch->addWidget(view2);
    ch->addStretch();
    auto *preview = button(i18n::text("▶  航线预览"), "secondary");
    preview->setCheckable(true);
    ch->addWidget(preview);
    connect(preview, &QPushButton::toggled, this, [this, preview](bool b) {
        canvas_->setPreview(b);
        if (scene3d_)
            scene3d_->setPreview(b);
        preview->setText(b ? i18n::text("Ⅱ  暂停预览") : i18n::text("▶  航线预览"));
    });
    auto *speed = new QComboBox;
    speed->setObjectName("previewSpeed");
    speed->addItems({"40 km/h", "70 km/h", "100 km/h"});
    speed->setCurrentIndex(1);
    speed->setMaximumWidth(98);
    ch->addWidget(speed);
    connect(speed, &QComboBox::currentIndexChanged, this, [this](int i) {
        canvas_->setPreviewSpeed(i == 0 ? 40 : i == 1 ? 70 : 100);
        if (scene3d_)
            scene3d_->setPreviewSpeed(i == 0 ? 40 : i == 1 ? 70 : 100);
    });
    cl->addLayout(ch);
    notice_ = label(i18n::text("示例赛道 · 导入俯视图，即可识别生成并继续编辑。"), "notice");
    notice_->setWordWrap(true);
    cl->addWidget(notice_);
    routeStatus_ = label({}, "subtle");
    routeStatus_->setObjectName("routeStatus");
    routeStatus_->setWordWrap(true);
    routeStatus_->hide();
    cl->addWidget(routeStatus_);
    auto *frame = new QFrame;
    frame->setObjectName("canvasFrame");
    auto *fl = new QVBoxLayout(frame);
    fl->setContentsMargins(1, 1, 1, 1);
    views_ = new QStackedWidget;
    canvas_ = new TrackCanvas;
    views_->addWidget(canvas_);
    if (QGuiApplication::platformName() != "offscreen") {
        scene3d_ = new Scene3D;
        views_->addWidget(scene3d_);
        views_->setCurrentWidget(scene3d_);
    } else {
        view2->setChecked(true);
        view3->setEnabled(false);
    }
    fl->addWidget(views_);
    cl->addWidget(frame, 1);
    connect(view3, &QPushButton::clicked, this, [this] {
        if (scene3d_) {
            views_->setCurrentWidget(scene3d_);
            scene3d_->fitTrack();
        }
    });
    connect(view2, &QPushButton::clicked, this, [this] {
        views_->setCurrentWidget(canvas_);
        canvas_->fitTrack();
    });
    auto *orderHead = new QHBoxLayout;
    orderHead->addWidget(label(i18n::text("飞行顺序"), "sectionTitle"));
    orderHead->addWidget(label(i18n::text("实体门可被多次穿越；编号按过门次数排列"), "subtle"));
    orderHead->addStretch();
    auto *prev = button(i18n::text("← 前移"), "tool");
    auto *next = button(i18n::text("后移 →"), "tool");
    orderHead->addWidget(prev);
    orderHead->addWidget(next);
    connect(prev, &QPushButton::clicked, this, [this] {
        if (sequenceTabs_->currentIndex() == 0)
            moveRoutePass(-1);
        else
            moveOrder(-1);
    });
    connect(next, &QPushButton::clicked, this, [this] {
        if (sequenceTabs_->currentIndex() == 0)
            moveRoutePass(1);
        else
            moveOrder(1);
    });
    auto *addPass = button(i18n::text("＋ 穿越"), "tool");
    auto *copyPass = button(i18n::text("重复本次"), "tool");
    auto *removePass = button(i18n::text("删本次"), "tool");
    orderHead->addWidget(addPass);
    orderHead->addWidget(copyPass);
    orderHead->addWidget(removePass);
    connect(addPass, &QPushButton::clicked, this, [this] { addRoutePass(); });
    connect(copyPass, &QPushButton::clicked, this, [this] { duplicateRoutePass(); });
    connect(removePass, &QPushButton::clicked, this, [this] { deleteRoutePass(); });
    cl->addLayout(orderHead);
    order_ = new QListWidget;
    order_->setObjectName("orderList");
    order_->setFlow(QListView::LeftToRight);
    order_->setWrapping(false);
    order_->setHorizontalScrollMode(QAbstractItemView::ScrollPerPixel);
    order_->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    order_->setFixedHeight(81);
    order_->setSpacing(5);
    routeOrder_ = new QListWidget;
    routeOrder_->setObjectName("routeOrderList");
    routeOrder_->setFlow(QListView::LeftToRight);
    routeOrder_->setWrapping(false);
    routeOrder_->setHorizontalScrollMode(QAbstractItemView::ScrollPerPixel);
    routeOrder_->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    routeOrder_->setSpacing(5);
    sequenceTabs_ = new QTabWidget;
    sequenceTabs_->addTab(routeOrder_, i18n::text("过门步骤"));
    sequenceTabs_->addTab(order_, i18n::text("实体组件"));
    sequenceTabs_->setFixedHeight(112);
    cl->addWidget(sequenceTabs_);
    connect(sequenceTabs_, &QTabWidget::currentChanged, this, [this](int tab) {
        if (!updating_ && tab == 1) {
            selectedRouteStep_.clear();
            routeOrder_->setCurrentRow(-1);
            updateSelection(selected_);
        }
    });
    connect(routeOrder_, &QListWidget::currentRowChanged, this, [this](int row) {
        if (!updating_ && row >= 0)
            updateRouteSelection(routeOrder_->item(row)->data(Qt::UserRole).toString());
    });
    connect(order_, &QListWidget::currentRowChanged, this, [this](int i) {
        if (!updating_ && i >= 0 && i < doc_.gates.size()) {
            selectedRouteStep_.clear();
            canvas_->selectGate(doc_.gates[i].id);
            updateSelection(doc_.gates[i].id);
        }
    });
    connect(order_, &QListWidget::itemClicked, this, [this](QListWidgetItem *) {
        if (!updating_) {
            selectedRouteStep_.clear();
            routeOrder_->setCurrentRow(-1);
            updateSelection(selected_);
        }
    });
    auto *rightScroll = new QScrollArea;
    rightScroll->setWidgetResizable(true);
    rightScroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    rightScroll->setMinimumWidth(218);
    rightScroll->setMaximumWidth(250);
    auto *right = new QFrame;
    right->setObjectName("rightSidebar");
    rightScroll->setWidget(right);
    auto *rl = new QVBoxLayout(right);
    rl->setContentsMargins(16, 20, 16, 16);
    rl->setSpacing(12);
    main->addWidget(rightScroll);
    rl->addWidget(label(i18n::text("赛道组件"), "sectionTitle"));
    rl->addWidget(label(i18n::text("选择组件，再点击画布放置"), "subtle"));
    auto *cards = new QGridLayout;
    auto *placementGroup = new QButtonGroup(right);
    placementGroup->setObjectName("placementGroup");
    placementGroup->setExclusive(true);
    cards->setSpacing(9);
    struct Definition {
        QString type, title, sub;
        double w, h;
    };
    const QList<Definition> defs = {
        {"standard", i18n::text("标准门"), i18n::text("中心距 3 × 3 m"), 3.0, 3.0},
        {"start", i18n::text("起终点门"), i18n::text("中心距 3 × 3 m"), 3.0, 3.0},
        {"pillar", i18n::text("转向柱"), "0.1 × 3.2 m", .1, 3.2}};
    for (int i = 0; i < defs.size(); ++i) {
        const auto d = defs[i];
        auto *card = new GateCard(d.type, d.title, d.sub);
        placementGroup->addButton(card);
        cards->addWidget(card, i / 2, i % 2);
        connect(card, &QPushButton::clicked, this, [this, d] {
            setPlacement(d.type, d.w, d.h);
            notice_->setText(i18n::text("已选择") + d.title +
                             i18n::text(" · 点击画布放置，Esc 返回选择工具。"));
        });
    }
    rl->addLayout(cards);
    auto *compoundToggle = button(i18n::text("组合门 · 展开"), "tool");
    compoundToggle->setCheckable(true);
    rl->addWidget(compoundToggle);
    auto *compoundPanel = new QWidget;
    auto *compoundCards = new QGridLayout(compoundPanel);
    compoundCards->setContentsMargins(0, 0, 0, 0);
    compoundCards->setSpacing(9);
    const QList<Definition> compounds = {
        {"double", i18n::text("双连门"), i18n::text("两列单元"), 3, 3},
        {"triple", i18n::text("三连门"), i18n::text("三列单元"), 3, 3},
        {"ladder", i18n::text("日字门"), i18n::text("上下两层"), 3, 3},
        {"quad", i18n::text("田字门"), i18n::text("两列两层"), 3, 3},
        {"ladder-blocked", i18n::text("双层单门"), i18n::text("下层封闭，仅上层通行"), 3, 3},
        {"cube", i18n::text("单层重力门"), i18n::text("立方框架"), 3, 3},
        {"cube-stack", i18n::text("双层重力门"), i18n::text("两层立方框架"), 3, 3}};
    for (int i = 0; i < compounds.size(); ++i) {
        const auto d = compounds[i];
        auto *card = new GateCard(d.type, d.title, d.sub);
        placementGroup->addButton(card);
        compoundCards->addWidget(card, i / 2, i % 2);
        connect(card, &QPushButton::clicked, this, [this, d] {
            setPlacement(d.type, d.w, d.h);
            notice_->setText(i18n::text("已选择") + d.title +
                             i18n::text(" · 点击场景放置，选中后可调整通过格。"));
        });
    }
    compoundPanel->hide();
    rl->addWidget(compoundPanel);
    connect(compoundToggle, &QPushButton::toggled, this,
            [compoundToggle, compoundPanel](bool open) {
                compoundPanel->setVisible(open);
                compoundToggle->setText(open ? i18n::text("组合门 · 收起")
                                             : i18n::text("组合门 · 展开"));
            });
    auto *stock =
        label(i18n::text("标准杆件长 3.2 m\n截面 0.1 × 0.1 m\n选中组件可改颜色和尺寸"), "subtle");
    stock->setWordWrap(true);
    rl->addWidget(stock);
    rl->addStretch();
    auto *sample = button(i18n::text("加载示例赛道"), "secondary");
    rl->addWidget(sample);
    connect(sample, &QPushButton::clicked, this, [this] { newProject(true); });
    auto *png = button(i18n::text("导出赛道预览 PNG"), "tool");
    rl->addWidget(png);
    connect(png, &QPushButton::clicked, this, [this] { exportPng(); });
    rl->addWidget(label(i18n::text("拖动空白处旋转视角\n滚轮缩放 · 中键平移"), "workflow"));
    main->setStretchFactor(0, 0);
    main->setStretchFactor(1, 1);
    main->setStretchFactor(2, 0);
    main->setSizes(i18n::english() ? QList<int>{284, 998, 238} : QList<int>{252, 1030, 238});
    for (auto *combo : left->findChildren<QComboBox *>()) {
        // Long translated menu entries must not expand the scroll area's content
        // beyond its visible width. The popup still shows the complete entry.
        combo->setSizeAdjustPolicy(QComboBox::AdjustToMinimumContentsLengthWithIcon);
        combo->setMinimumContentsLength(0);
        combo->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    }
    statusBar()->setSizeGripEnabled(false);
    statusBar()->showMessage(i18n::text("就绪 · 全部编辑在本机完成"));
    recognitionTimer_ = new QTimer(this);
    recognitionTimer_->setInterval(200);
    connect(recognitionTimer_, &QTimer::timeout, this, [this] {
        if (recognizing_)
            statusBar()->showMessage(
                i18n::text("正在本地解析图片… %1 秒 · 可继续查看场景")
                    .arg((QDateTime::currentMSecsSinceEpoch() - recognitionStartedMs_) / 1000.0, 0,
                         'f', 1));
    });
    const QList<QWidget *> dropTargets = {this, dropZone_, canvas_, canvas_->viewport(), scene3d_};
    for (auto *target : dropTargets)
        if (target) {
            target->setAcceptDrops(true);
            target->installEventFilter(this);
        }
    zoom_ = label("100%");
    auto *minus = button("−", "zoomButton");
    auto *plus = button("＋", "zoomButton");
    statusBar()->addPermanentWidget(minus);
    statusBar()->addPermanentWidget(zoom_);
    statusBar()->addPermanentWidget(plus);
    connect(minus, &QPushButton::clicked, this, [this] { zoomActive(.8); });
    connect(plus, &QPushButton::clicked, this, [this] { zoomActive(1.25); });
    canvas_->onZoom = [this](double n) { zoom_->setText(QString::number(qRound(n)) + "%"); };
    canvas_->onSelection = [this](QString id) { updateSelection(id); };
    canvas_->onPlace = [this](QString type, QPointF p, double w, double h) {
        addGate(type, p, w, h);
    };
    canvas_->onGestureBegin = [this] { gestureBefore_ = doc_; };
    canvas_->onGateMoved = [this](QString id, QPointF p) {
        for (auto &g : doc_.gates)
            if (g.id == id)
                g.position = p;
        updateSelection(id);
        if (scene3d_)
            scene3d_->refreshGeometry();
    };
    canvas_->onGestureEnd = [this] {
        if (!sameDocument(gestureBefore_, doc_))
            pushEdit(i18n::text("移动门架"), gestureBefore_, doc_);
    };
    if (scene3d_) {
        scene3d_->onSelection = [this](QString id) { canvas_->selectGate(id); };
        scene3d_->onRouteSelection = [this](QString id) { updateRouteSelection(id); };
        scene3d_->onPlace = canvas_->onPlace;
        scene3d_->onGestureBegin = canvas_->onGestureBegin;
        scene3d_->onGestureEnd = canvas_->onGestureEnd;
        scene3d_->onZoom = canvas_->onZoom;
        scene3d_->onGateMoved = [this](QString id, QPointF p) {
            for (auto &g : doc_.gates)
                if (g.id == id)
                    g.position = p;
            updateSelection(id);
            canvas_->refreshRoute();
        };
    }
}

void StudioWindow::changeLanguage(const QString &code) {
    if (code == i18n::language())
        return;
    if (recognizing_) {
        const QSignalBlocker blocked(language_);
        language_->setCurrentIndex(i18n::english() ? 1 : 0);
        statusBar()->showMessage(i18n::text("请等待图片识别完成后再切换语言。"), 5000);
        return;
    }
    const auto document = doc_;
    const int view = views_->currentIndex(), sequence = sequenceTabs_->currentIndex();
    const bool snap = findChild<QCheckBox *>("snapToGrid")->isChecked();
    const bool route = findChild<QCheckBox *>("showRoute")->isChecked();
    const int speed = findChild<QComboBox *>("previewSpeed")->currentIndex();
    const QString selected = selected_, step = selectedRouteStep_;
    canvas_->onSelection = {};
    canvas_->onGateMoved = {};
    canvas_->onGestureBegin = {};
    canvas_->onGestureEnd = {};
    canvas_->onPlace = {};
    canvas_->onZoom = {};
    canvas_->setPreview(false);
    canvas_->setDocument(nullptr);
    if (scene3d_) {
        scene3d_->onSelection = {};
        scene3d_->onRouteSelection = {};
        scene3d_->onGateMoved = {};
        scene3d_->onGestureBegin = {};
        scene3d_->onGestureEnd = {};
        scene3d_->onPlace = {};
        scene3d_->onZoom = {};
        scene3d_->setPreview(false);
        scene3d_->setDocument(nullptr);
    }
    delete recognitionTimer_;
    recognitionTimer_ = nullptr;
    delete takeCentralWidget();
    delete statusBar();
    canvas_ = nullptr;
    scene3d_ = nullptr;
    i18n::setLanguage(code, true);
    setupUi();
    selected_ = selected;
    selectedRouteStep_ = step;
    applyDocument(document);
    views_->setCurrentIndex(std::min(view, views_->count() - 1));
    sequenceTabs_->setCurrentIndex(sequence);
    findChild<QCheckBox *>("snapToGrid")->setChecked(snap);
    findChild<QCheckBox *>("showRoute")->setChecked(route);
    findChild<QComboBox *>("previewSpeed")->setCurrentIndex(speed);
    setWindowModified(!undo_.isClean());
    fitViews();
}

void StudioWindow::fitViews() {
    canvas_->fitTrack();
    if (scene3d_)
        scene3d_->fitTrack();
}
void StudioWindow::setPlacement(const QString &type, double w, double h) {
    if (type.isEmpty())
        if (auto *group = findChild<QButtonGroup *>("placementGroup")) {
            group->setExclusive(false);
            for (auto *card : group->buttons())
                card->setChecked(false);
            group->setExclusive(true);
        }
    canvas_->setPlacement(type, w, h);
    if (scene3d_)
        scene3d_->setPlacement(type, w, h);
}
void StudioWindow::zoomActive(double factor) {
    if (scene3d_ && views_->currentWidget() == scene3d_)
        scene3d_->zoomBy(factor);
    else
        canvas_->zoomBy(factor);
}

void StudioWindow::setupShortcuts() {
    auto shortcut = [this](const QKeySequence &key, const std::function<void()> &fn) {
        auto *s = new QShortcut(key, this);
        connect(s, &QShortcut::activated, this, fn);
    };
    shortcut(QKeySequence::New, [this] { newProject(); });
    shortcut(QKeySequence::Open, [this] { openProject(); });
    shortcut(QKeySequence::Save, [this] { save(); });
    shortcut(QKeySequence("Ctrl+Shift+S"), [this] { save(true); });
    shortcut(QKeySequence("Ctrl+I"), [this] { importImage(); });
    shortcut(QKeySequence::Undo, [this] { undo_.undo(); });
    shortcut(QKeySequence("Ctrl+Shift+Z"), [this] { undo_.redo(); });
    auto canvasKey = [this, shortcut](const QKeySequence &key, std::function<void()> fn) {
        shortcut(key, [this, fn] {
            auto *focus = QApplication::focusWidget();
            if (qobject_cast<QLineEdit *>(focus) || qobject_cast<QAbstractSpinBox *>(focus) ||
                qobject_cast<QComboBox *>(focus))
                return;
            fn();
        });
    };
    canvasKey(QKeySequence::Delete, [this] {
        if (!selectedRouteStep_.isEmpty())
            deleteRoutePass();
        else
            deleteGate();
    });
    canvasKey(QKeySequence("Ctrl+D"), [this] {
        if (!selectedRouteStep_.isEmpty())
            duplicateRoutePass();
        else
            duplicateGate();
    });
    canvasKey(QKeySequence("F"), [this] { fitViews(); });
    canvasKey(QKeySequence("Q"), [this] { rotateGate(-15); });
    canvasKey(QKeySequence("E"), [this] { rotateGate(15); });
}
void StudioWindow::applyDocument(const TrackDocument &d) {
    updating_ = true;
    doc_ = d;
    ++revision_;
    name_->setText(doc_.name);
    scenery_->setCurrentText(doc_.scenery);
    scale_->setValue(doc_.metersPerPixel);
    opacity_->setValue(qRound(doc_.backgroundOpacity * 100));
    imageVisible_->setChecked(doc_.showBackground);
    takeoffX_->setValue(doc_.takeoff.position.x());
    takeoffY_->setValue(doc_.takeoff.position.y());
    takeoffZ_->setValue(doc_.takeoff.elevation);
    takeoffYaw_->setValue(doc_.takeoff.yaw);
    takeoffStatus_->setText(doc_.takeoff.enabled ? i18n::text("已设置独立起飞点，不占过门编号")
                                                 : i18n::text("尚未设置 · 导出前必须设置"));
    recognizeButton_->setEnabled(!doc_.background.isNull() && !recognizing_);
    imageStatus_->setText(doc_.background.isNull()
                              ? i18n::text("支持 PNG / JPG / BMP\n可拖到小框、3D 场景或俯视图")
                              : i18n::text("已导入 %1 × %2 像素\n可先调整比例，再识别生成")
                                    .arg(doc_.background.width())
                                    .arg(doc_.background.height()));
    canvas_->setDocument(&doc_);
    if (scene3d_)
        scene3d_->setDocument(&doc_);
    order_->clear();
    int row = -1;
    for (int i = 0; i < doc_.gates.size(); ++i) {
        const auto &g = doc_.gates[i];
        auto *item = new QListWidgetItem(
            QString("G%1  %2\n%3")
                .arg(i + 1, 2, 10, QChar('0'))
                .arg(i18n::text(gateTypeName(g.type)), g.label.isEmpty() ? "Checkpoint" : g.label));
        item->setSizeHint(QSize(138, 52));
        order_->addItem(item);
        if (g.id == selected_)
            row = i;
    }
    order_->setCurrentRow(row);
    rebuildRouteList();
    updating_ = false;
    updateStats();
    canvas_->selectGate(selected_);
    updateSelection(selected_);
    if (scene3d_)
        scene3d_->selectRouteStep(selectedRouteStep_);
    setWindowTitle("OpenTrack Studio " + QCoreApplication::applicationVersion() + " — " +
                   doc_.name + "[*]");
}
void StudioWindow::edit(const QString &title, const std::function<void(TrackDocument &)> &change) {
    auto after = doc_;
    change(after);
    if (sameDocument(after, doc_))
        return;
    pushEdit(title, doc_, after);
}
void StudioWindow::pushEdit(const QString &title, const TrackDocument &before,
                            const TrackDocument &after) {
    undo_.push(new DocumentCommand(this, title, before, after));
}
void StudioWindow::updateStats() {
    int passes = 0, turnChecks = 0;
    for (const auto &step : effectiveRouteSteps(doc_)) {
        passes += step.kind == "pass";
        turnChecks += step.kind == "guide" && step.extra.value("raceCheckpoint").toBool();
    }
    gateCount_->setText(QString("%1 / %2").arg(doc_.gates.size()).arg(passes));
    QMap<QString, int> counts;
    for (const auto &gate : doc_.gates)
        ++counts[gate.type];
    const QList<QPair<QString, QString>> inventoryNames{{"start", i18n::text("计时 / 起终点门")},
                                                        {"standard", i18n::text("单门")},
                                                        {"cube", i18n::text("单层重力门")},
                                                        {"cube-stack", i18n::text("双层重力门")},
                                                        {"ladder-blocked", i18n::text("双层单门")},
                                                        {"ladder", i18n::text("日字门")},
                                                        {"double", i18n::text("双连门")},
                                                        {"triple", i18n::text("三连门")},
                                                        {"quad", i18n::text("田字门")},
                                                        {"pillar", i18n::text("转向柱")}};
    QString inventory = "<table width=\"100%\" cellspacing=\"2\">";
    auto inventoryRow = [&inventory](const QString &name, int count) {
        inventory += QString("<tr><td>%1</td><td align=\"right\">× %2</td></tr>")
                         .arg(name.toHtmlEscaped())
                         .arg(count);
    };
    for (const auto &entry : inventoryNames)
        if (const int count = counts.take(entry.first))
            inventoryRow(entry.second, count);
    for (auto it = counts.cbegin(); it != counts.cend(); ++it)
        inventoryRow(i18n::text(gateTypeName(it.key())), it.value());
    inventoryRow(i18n::text("独立起飞台"), doc_.takeoff.enabled ? 1 : 0);
    obstacleInventory_->setText(inventory + "</table>");
    gateCount_->setToolTip(
        turnChecks ? i18n::text("另有 %1 个翻绕检查点，在模拟器中约束绕行顺序。").arg(turnChecks)
                   : QString());
    length_->setText(QString::number(routeLength(doc_), 'f', 1) + " m");
    const auto curve = routeCurve(doc_);
    QStringList hints;
    if (turnChecks)
        hints << (turnChecks == 1
                      ? i18n::text("T1：1 个辅助翻绕检查点，随赛道导出。")
                      : i18n::text("T1–T%1：%1 个辅助翻绕检查点，随赛道导出。").arg(turnChecks));
    if (!curve.warnings.isEmpty()) {
        QString message = i18n::text(curve.warnings.first());
        if (message.contains("连续切向"))
            message = i18n::text("部分连接转弯较急，可调整门位或绕行点。");
        hints << i18n::text("路线待校正：") + message;
    }
    routeStatus_->setText(hints.join("  "));
    routeStatus_->setToolTip(i18n::messages(curve.warnings).join("\n"));
    routeStatus_->setVisible(!hints.isEmpty());
}
void StudioWindow::rebuildRouteList() {
    routeOrder_->clear();
    int number = 0, selectedRow = -1;
    const auto steps = effectiveRouteSteps(doc_);
    for (const auto &step : steps) {
        if (step.kind != "pass")
            continue;
        const Gate *gate = findGate(doc_, step.gateId);
        if (!gate)
            continue;
        int physical = 0;
        for (int i = 0; i < doc_.gates.size(); ++i)
            if (doc_.gates[i].id == gate->id) {
                physical = i + 1;
                break;
            }
        const bool ambiguous =
            step.extra.value("recognition").toObject().value("ambiguous").toBool();
        auto *item =
            new QListWidgetItem(QString("%1 · G%2 %3\n%4")
                                    .arg(++number)
                                    .arg(physical)
                                    .arg(i18n::text(gateTypeName(gate->type)),
                                         (ambiguous ? i18n::text("？待校对 · ") : QString()) +
                                             passDescription(*gate, step)));
        item->setData(Qt::UserRole, step.id);
        item->setData(Qt::UserRole + 1, step.gateId);
        item->setSizeHint(QSize(185, 54));
        item->setToolTip(
            (ambiguous ? i18n::text("图中路线存在遮挡，窗口及方向需要人工校对。\n") : QString()) +
            step.label);
        routeOrder_->addItem(item);
        if (step.id == selectedRouteStep_)
            selectedRow = routeOrder_->count() - 1;
    }
    routeOrder_->setCurrentRow(selectedRow);
    sequenceTabs_->setTabText(0, i18n::text("过门步骤 (%1)").arg(number));
    sequenceTabs_->setTabText(1, i18n::text("实体组件 (%1)").arg(doc_.gates.size()));
    if (selectedRow < 0)
        selectedRouteStep_.clear();
}
void StudioWindow::updateRouteSelection(const QString &id) {
    if (updating_ || selectingRoute_)
        return;
    QScopedValueRollback<bool> guard(selectingRoute_, true);
    selectedRouteStep_ = id;
    if (!id.isEmpty())
        sequenceTabs_->setCurrentIndex(0);
    QString gateId = selected_;
    for (const auto &step : effectiveRouteSteps(doc_))
        if (step.id == id) {
            gateId = step.gateId;
            break;
        }
    if (!gateId.isEmpty()) {
        canvas_->selectGate(gateId);
        updateSelection(gateId);
    }
    if (scene3d_)
        scene3d_->selectRouteStep(selectedRouteStep_);
    const QSignalBlocker blocker(routeOrder_);
    for (int i = 0; i < routeOrder_->count(); ++i)
        if (routeOrder_->item(i)->data(Qt::UserRole).toString() == selectedRouteStep_) {
            routeOrder_->setCurrentRow(i);
            routeOrder_->scrollToItem(routeOrder_->item(i));
            break;
        }
}
void StudioWindow::editRouteSelection() {
    if (updating_ || selectedRouteStep_.isEmpty())
        return;
    const QString id = selectedRouteStep_, face = stepFace_->currentData().toString();
    const int column = std::max(0, passColumn_->currentIndex());
    const int level = std::max(0, passLevel_->currentIndex());
    const bool reverse = stepReverse_->currentIndex() == 1;
    if (const auto *gate = findGate(doc_, selected_))
        if (!gateCellOpen(*gate, column, level)) {
            updateSelection(selected_);
            return;
        }
    edit(i18n::text("调整本次穿越"), [=](TrackDocument &d) {
        makeRouteExplicit(d);
        for (auto &step : d.routeSteps)
            if (step.id == id && step.kind == "pass") {
                if (step.face != face) {
                    step.extra.remove("flightTangent");
                    step.extra.remove("apertureU");
                    step.extra.remove("apertureV");
                } else if (step.reverse != reverse && step.extra.contains("flightTangent")) {
                    auto tangent = step.extra.value("flightTangent").toArray();
                    for (qsizetype axis = 0; axis < tangent.size(); ++axis)
                        tangent[axis] = -tangent[axis].toDouble();
                    step.extra.insert("flightTangent", tangent);
                }
                step.face = face;
                step.column = column;
                step.level = level;
                step.reverse = reverse;
                if (step.extra.contains("recognition")) {
                    auto evidence = step.extra.value("recognition").toObject();
                    evidence.insert("ambiguous", false);
                    evidence.insert("reviewedByUser", true);
                    step.extra.insert("recognition", evidence);
                }
            }
    });
}
void StudioWindow::addRoutePass() {
    const Gate *gate = findGate(doc_, selected_);
    if (!gate) {
        statusBar()->showMessage(i18n::text("先选择一个实体门，再添加穿越步骤"), 4000);
        return;
    }
    RouteStep step;
    step.id = newGateId();
    step.gateId = gate->id;
    step.column = gate->extra.value("passColumn").toInt(0);
    step.level = gate->extra.value("passLevel").toInt(gate->type == "ladder-blocked" ? 1 : 0);
    const QString afterId = selectedRouteStep_;
    selectedRouteStep_ = step.id;
    edit(i18n::text("新增一次穿越"), [step, afterId](TrackDocument &d) {
        makeRouteExplicit(d);
        qsizetype at = d.routeSteps.size();
        for (qsizetype i = 0; i < d.routeSteps.size(); ++i)
            if (d.routeSteps[i].id == afterId) {
                at = i + 1;
                while (at < d.routeSteps.size() && d.routeSteps[at].kind == "guide")
                    ++at;
                break;
            }
        d.routeSteps.insert(at, step);
    });
}
void StudioWindow::duplicateRoutePass() {
    for (const auto &step : effectiveRouteSteps(doc_))
        if (step.id == selectedRouteStep_ && step.kind == "pass") {
            RouteStep copy = step;
            copy.id = newGateId();
            const QString afterId = step.id;
            selectedRouteStep_ = copy.id;
            edit(i18n::text("重复本次穿越"), [copy, afterId](TrackDocument &d) {
                makeRouteExplicit(d);
                for (qsizetype i = 0; i < d.routeSteps.size(); ++i)
                    if (d.routeSteps[i].id == afterId) {
                        d.routeSteps.insert(i + 1, copy);
                        break;
                    }
            });
            return;
        }
}
void StudioWindow::deleteRoutePass() {
    if (selectedRouteStep_.isEmpty())
        return;
    const QString id = selectedRouteStep_;
    selectedRouteStep_.clear();
    edit(i18n::text("删除本次穿越"), [id](TrackDocument &d) {
        makeRouteExplicit(d);
        for (qsizetype i = 0; i < d.routeSteps.size(); ++i)
            if (d.routeSteps[i].id == id) {
                const QString gateId = d.routeSteps[i].gateId;
                qsizetype end = i + 1;
                while (end < d.routeSteps.size() && d.routeSteps[end].kind == "guide" &&
                       d.routeSteps[end].gateId == gateId)
                    ++end;
                d.routeSteps.remove(i, end - i);
                break;
            }
    });
}
void StudioWindow::moveRoutePass(int delta) {
    if (selectedRouteStep_.isEmpty())
        return;
    const QString id = selectedRouteStep_;
    edit(i18n::text("调整过门顺序"), [id, delta](TrackDocument &d) {
        makeRouteExplicit(d);
        QVector<QVector<RouteStep>> blocks;
        QVector<RouteStep> prefix;
        int active = -1;
        for (const auto &step : d.routeSteps) {
            if (step.kind == "pass") {
                blocks.append(QVector<RouteStep>{step});
                if (step.id == id)
                    active = int(blocks.size()) - 1;
            } else if (blocks.isEmpty())
                prefix.append(step);
            else
                blocks.last().append(step);
        }
        const int other = active + delta;
        if (active < 0 || other < 0 || other >= blocks.size())
            return;
        blocks.swapItemsAt(active, other);
        d.routeSteps = prefix;
        for (const auto &block : blocks)
            d.routeSteps.append(block);
    });
}
void StudioWindow::applyManeuver() {
    const Gate *gate = findGate(doc_, selected_);
    if (!gate)
        return;
    const QString pattern = maneuver_->currentData().toString();
    if (pattern == "source-route")
        return;
    const auto replacement = gateManeuverSteps(*gate, pattern);
    if (replacement.isEmpty())
        return;
    const QString gateId = gate->id;
    for (const auto &step : replacement)
        if (step.kind == "pass") {
            selectedRouteStep_ = step.id;
            break;
        }
    edit(i18n::text("替换此门的过门动作"), [gateId, replacement, pattern](TrackDocument &d) {
        makeRouteExplicit(d);
        qsizetype first = d.routeSteps.size();
        for (qsizetype i = 0; i < d.routeSteps.size(); ++i)
            if (d.routeSteps[i].gateId == gateId) {
                first = i;
                break;
            }
        d.routeSteps.erase(
            std::remove_if(d.routeSteps.begin(), d.routeSteps.end(),
                           [gateId](const RouteStep &step) { return step.gateId == gateId; }),
            d.routeSteps.end());
        first = std::min(first, d.routeSteps.size());
        for (qsizetype i = 0; i < replacement.size(); ++i)
            d.routeSteps.insert(first + i, replacement[i]);
        for (auto &item : d.gates)
            if (item.id == gateId)
                item.extra.insert("routePattern", pattern);
    });
    sequenceTabs_->setCurrentIndex(0);
    notice_->setText(i18n::text("连续绕杆动作已应用；按编号检查入口、出口与升降。Ctrl+Z 可恢复。"));
}
void StudioWindow::updateManeuverPreview() {
    const Gate *gate = findGate(doc_, selected_);
    const QString pattern = maneuver_->currentData().toString();
    QVector<RouteStep> sourceSteps;
    if (gate)
        for (const auto &step : effectiveRouteSteps(doc_))
            if (step.gateId == gate->id)
                sourceSteps.append(step);
    static_cast<ManeuverPreview *>(maneuverPreview_)->setManeuver(gate, pattern, sourceSteps);
    const bool supported =
        gate && (pattern == "source-route" || !gateManeuverSteps(*gate, pattern).isEmpty());
    if (auto *apply = findChild<QPushButton *>("applyManeuver"))
        apply->setEnabled(supported && pattern != "source-route");
    const bool orbit = pattern == "orbit-bar-cw" || pattern == "orbit-bar-ccw";
    maneuverHint_->setText(
        pattern == "source-route"
            ? i18n::text("当前识别路线：按编号连续绕杆、换层和穿出；选择本项不会改动动作。")
        : pattern.startsWith("upright-")
            ? i18n::text("边绕立柱边换层，从一个窗口直接接到另一个；不会回到原窗口后再升降。")
        : orbit
            ? i18n::text(
                  "独立整圈动作，不用于替代图中的连续换层。方向按组件左侧沿横杆看向右侧的视角。")
        : pattern == "single" ? i18n::text("选择动作预览，再点“应用动作”替换此门已有的穿越。")
                              : i18n::text("按图示绕过杆件；检查预览后点“应用动作”。"));
    if (pattern == "source-route") {
        QStringList passages;
        int ordinal = 0;
        for (const auto &step : sourceSteps)
            if (step.kind == "pass")
                passages << QString("%1. %2").arg(++ordinal).arg(passDescription(*gate, step));
        maneuverHint_->setText(i18n::text("过门顺序：%1").arg(passages.join(" → ")) + "\n" +
                               i18n::text("请按编号核对进入面、出口和转向。"));
    }
    if (gate && !supported)
        maneuverHint_->setText(i18n::text("当前门型不支持此动作，请保留原路线或选择单次穿越。"));
}
void StudioWindow::editTakeoff() {
    if (updating_)
        return;
    const QPointF position(takeoffX_->value(), takeoffY_->value());
    const double elevation = takeoffZ_->value(), yaw = takeoffYaw_->value();
    if (position == doc_.takeoff.position && elevation == doc_.takeoff.elevation &&
        yaw == doc_.takeoff.yaw)
        return;
    edit(i18n::text("设置独立起飞点"), [=](TrackDocument &d) {
        d.takeoff.enabled = true;
        d.takeoff.position = position;
        d.takeoff.elevation = elevation;
        d.takeoff.yaw = yaw;
    });
}
void StudioWindow::updateSelection(const QString &id) {
    if (updating_)
        return;
    const bool revealProperties = selected_ != id || properties_->isHidden();
    selected_ = id;
    RouteStep activeStep;
    bool hasActiveStep = false;
    for (const auto &step : effectiveRouteSteps(doc_))
        if (step.id == selectedRouteStep_ && step.gateId == id && step.kind == "pass") {
            activeStep = step;
            hasActiveStep = true;
            break;
        }
    if (!hasActiveStep)
        selectedRouteStep_.clear();
    const Gate *gate = nullptr;
    int row = -1;
    for (int i = 0; i < doc_.gates.size(); ++i)
        if (doc_.gates[i].id == id) {
            gate = &doc_.gates[i];
            row = i;
            break;
        }
    if (scene3d_)
        scene3d_->selectGate(id);
    properties_->setVisible(gate != nullptr);
    updating_ = true;
    const bool pillar = gate && gate->type == "pillar";
    const auto frame = gate ? gateFrameMetrics(*gate) : GateFrameMetrics{};
    stepPanel_->setVisible(gate && hasActiveStep);
    passCell_->setVisible(
        gate && frame.centerlineModel && (frame.columns > 1 || frame.levels > 1) &&
        (hasActiveStep || (!doc_.routeStepsDefined && doc_.routeSteps.isEmpty())));
    if (gate) {
        passColumn_->clear();
        passLevel_->clear();
        for (int i = 0; i < frame.columns; ++i)
            passColumn_->addItem(i18n::text("第 %1 列").arg(i + 1));
        for (int i = 0; i < frame.levels; ++i)
            passLevel_->addItem(i18n::text("第 %1 层").arg(i + 1));
        if (gate->type == "ladder-blocked")
            if (auto *model = qobject_cast<QStandardItemModel *>(passLevel_->model()))
                if (auto *item = model->item(0))
                    item->setEnabled(false);
        passColumn_->setCurrentIndex(hasActiveStep ? activeStep.column
                                                   : gate->extra.value("passColumn").toInt(0));
        passLevel_->setCurrentIndex(hasActiveStep ? activeStep.level
                                                  : gate->extra.value("passLevel").toInt(0));
        passColumn_->setVisible(frame.columns > 1);
        passLevel_->setVisible(frame.levels > 1);
        stepFace_->clear();
        stepFace_->addItem(i18n::text("前面"), "front");
        stepFace_->addItem(i18n::text("背面"), "back");
        if (frame.centerDepth > 0) {
            stepFace_->addItem(i18n::text("左面"), "left");
            stepFace_->addItem(i18n::text("右面"), "right");
            stepFace_->addItem(i18n::text("顶面"), "top");
            stepFace_->addItem(i18n::text("底面"), "bottom");
        }
        stepFace_->setCurrentIndex(std::max(0, stepFace_->findData(activeStep.face)));
        stepReverse_->setCurrentIndex(activeStep.reverse ? 1 : 0);
        stepSummary_->setText(hasActiveStep ? passDescription(*gate, activeStep)
                                            : i18n::text("选择一次穿越"));
        maneuver_->clear();
        const bool hasSourceRoute = !gate->extra.value("recognition")
                                         .toObject()
                                         .value("routeEvidence")
                                         .toString()
                                         .isEmpty();
        if (hasSourceRoute)
            maneuver_->addItem(i18n::text("当前识别路线 · 连续绕杆"), "source-route");
        maneuver_->addItem(i18n::text("单次穿越"), "single");
        if (frame.levels > 1 && frame.centerDepth == 0) {
            maneuver_->addItem(i18n::text("绕左立柱上升 · 下层进、上层出"), "upright-left-up");
            maneuver_->addItem(i18n::text("绕左立柱下降 · 上层进、下层出"), "upright-left-down");
            maneuver_->addItem(i18n::text("绕右立柱上升 · 下层进、上层出"), "upright-right-up");
            maneuver_->addItem(i18n::text("绕右立柱下降 · 上层进、下层出"), "upright-right-down");
            maneuver_->addItem(i18n::text("绕中横杆 · 下层进、上层出"), "lower-upper");
            maneuver_->addItem(i18n::text("绕中横杆 · 上层进、下层出"), "upper-lower");
        }
        if (frame.columns > 1) {
            maneuver_->addItem(i18n::text("左窗 → 右窗，交替穿越"), "left-right");
            maneuver_->addItem(i18n::text("右窗 → 左窗，交替穿越"), "right-left");
        }
        if (frame.centerDepth > 0) {
            maneuver_->addItem(i18n::text("重力门 · 圆弧俯冲穿出"), "gravity-dive");
            maneuver_->addItem(i18n::text("重力门 · 圆弧爬升穿顶"), "gravity-climb");
        }
        if (frame.centerlineModel && frame.centerDepth == 0 && !pillar) {
            maneuver_->insertSeparator(maneuver_->count());
            const QString bar = frame.levels > 1 ? i18n::text("中横杆") : i18n::text("顶横杆");
            maneuver_->addItem(i18n::text("高级：绕") + bar + i18n::text("独立整圈 ↻"),
                               "orbit-bar-cw");
            maneuver_->addItem(i18n::text("高级：绕") + bar + i18n::text("独立整圈 ↺"),
                               "orbit-bar-ccw");
        }
        const QString currentPattern = gate->extra.value("routePattern").toString();
        maneuver_->setCurrentIndex(
            std::max(0, maneuver_->findData(currentPattern.isEmpty() && hasSourceRoute
                                                ? QString("source-route")
                                                : currentPattern)));
    }
    widthLabel_->setText(pillar                  ? i18n::text("柱体边长 / m")
                         : frame.centerlineModel ? i18n::text("横向中心距 / m")
                                                 : i18n::text("宽度 / m"));
    heightLabel_->setText(pillar                  ? i18n::text("柱长 / m")
                          : frame.centerlineModel ? i18n::text("竖向中心距 / m")
                                                  : i18n::text("高度 / m"));
    passSideLabel_->setVisible(pillar);
    passSide_->setVisible(pillar);
    if (pillar)
        passSide_->setCurrentIndex(gate->extra.value("passSide").toInt(1) < 0 ? 1 : 0);
    for (auto *spin : {x_, z_, alt_, yaw_, width_, height_})
        spin->setEnabled(gate != nullptr);
    selectionTitle_->setText(
        gate ? i18n::text("%1属性 · G%2").arg(i18n::text(gateTypeName(gate->type))).arg(row + 1)
             : i18n::text("门架属性 · 未选择"));
    if (gate) {
        width_->setMinimum(pillar ? .01 : .2);
        height_->setMinimum(pillar ? .1 : .2);
        x_->setValue(gate->position.x());
        z_->setValue(gate->position.y());
        alt_->setValue(gate->elevation);
        yaw_->setValue(gate->yaw);
        width_->setValue(gate->width);
        height_->setValue(gate->height);
        const QColor tint = gateColor(*gate);
        const int colorIndex = color_->findData(tint.name());
        color_->setCurrentIndex(colorIndex >= 0 ? colorIndex : color_->count() - 1);
        const auto nativeColor = nearestNativeBlockColor(tint);
        colorMapping_->setText(tint == nativeColor.color
                                   ? i18n::text("方块颜色：%1").arg(i18n::text(nativeColor.name))
                                   : i18n::text("自定义 %1 · 导出匹配：%2")
                                         .arg(tint.name().toUpper(), i18n::text(nativeColor.name)));
        if (!pillar && !frame.centerlineModel)
            colorMapping_->setText(i18n::text("导入原模型：此颜色用于编辑预览，导出保留原材质。"));
        if (pillar) {
            dimensions_->setText(i18n::text("截面 %1 × %1 m · 柱长 %2 m")
                                     .arg(gate->width, 0, 'f', 2)
                                     .arg(gate->height, 0, 'f', 2));
        } else if (frame.centerlineModel) {
            dimensions_->setText(
                i18n::text("杆件截面 %1 × %1 m\n横杆长 %2 m · 竖杆长 %3 m\n净开口 %4 × %5 m")
                    .arg(frame.beamThickness, 0, 'f', 2)
                    .arg(frame.horizontalBeamLength, 0, 'f', 2)
                    .arg(frame.verticalBeamLength, 0, 'f', 2)
                    .arg(frame.openingWidth, 0, 'f', 2)
                    .arg(frame.openingHeight, 0, 'f', 2));
            if (frame.columns > 1 || frame.levels > 1 || frame.centerDepth > 0)
                dimensions_->setText(
                    dimensions_->text() +
                    i18n::text("\n组合外形 %1 × %2 × %3 m\n各格尺寸相同，复杂穿越顺序需校对")
                        .arg(frame.outerWidth, 0, 'f', 2)
                        .arg(frame.outerHeight, 0, 'f', 2)
                        .arg(frame.outerDepth, 0, 'f', 2));
        } else {
            dimensions_->setText(i18n::text("导入组件：原模型尺寸约定"));
        }
    }
    order_->setCurrentRow(row);
    if (scene3d_)
        scene3d_->selectRouteStep(selectedRouteStep_);
    if (gate && revealProperties)
        if (auto *sidebar = findChild<QScrollArea *>("sidebar"))
            sidebar->verticalScrollBar()->setValue(0);
    updating_ = false;
    updateManeuverPreview();
    statusBar()->showMessage(
        gate ? i18n::text("已选中 #%1 %2 · 再次按住组件或脚下光圈可移动 · Q/E 旋转")
                   .arg(row + 1)
                   .arg(i18n::text(gateTypeName(gate->type)))
             : i18n::text(
                   "先单击组件选中，再按住组件或光圈拖动 · 右键旋转视角 · 中键平移 · 滚轮缩放"));
}
void StudioWindow::editColor(const QColor &color) {
    if (updating_ || selected_.isEmpty() || !color.isValid())
        return;
    const QString id = selected_;
    edit(i18n::text("调整组件颜色"), [id, color](TrackDocument &d) {
        for (auto &g : d.gates)
            if (g.id == id)
                setGateColor(g, color);
    });
}
void StudioWindow::editSelection() {
    if (updating_ || selected_.isEmpty())
        return;
    const QString id = selected_;
    const auto pos = QPointF(x_->value(), z_->value());
    const double alt = alt_->value(), yaw = yaw_->value(), w = width_->value(),
                 h = height_->value();
    edit(i18n::text("调整门架属性"), [=](TrackDocument &d) {
        for (auto &g : d.gates)
            if (g.id == id) {
                g.position = pos;
                g.elevation = alt;
                g.yaw = yaw;
                g.width = w;
                g.height = h;
            }
    });
}
void StudioWindow::addGate(const QString &type, QPointF p, double w, double h) {
    if (type == "__takeoff") {
        edit(i18n::text("放置独立起飞点"), [p](TrackDocument &d) {
            d.takeoff.enabled = true;
            d.takeoff.position = p;
        });
        setPlacement({});
        notice_->setText(
            i18n::text("已放置独立起飞点；可在左侧展开“坐标与朝向”校准基点高度和机头方向。"));
        return;
    }
    Gate g = makeGate(type, w, h);
    g.position = p;
    selected_ = g.id;
    selectedRouteStep_.clear();
    edit(i18n::text("放置门架"), [g](TrackDocument &d) {
        d.gates.append(g);
        if (d.routeStepsDefined || !d.routeSteps.isEmpty()) {
            RouteStep step;
            step.id = newGateId();
            step.gateId = g.id;
            d.routeSteps.append(step);
        }
    });
}
void StudioWindow::duplicateGate() {
    for (const auto &gate : doc_.gates)
        if (gate.id == selected_) {
            auto g = gate;
            g.id = newGateId();
            g.position += QPointF(2, 2);
            selected_ = g.id;
            selectedRouteStep_.clear();
            edit(i18n::text("复制门架"), [g](TrackDocument &d) {
                d.gates.append(g);
                if (d.routeStepsDefined || !d.routeSteps.isEmpty()) {
                    RouteStep step;
                    step.id = newGateId();
                    step.gateId = g.id;
                    step.column = g.extra.value("passColumn").toInt(0);
                    step.level = g.extra.value("passLevel").toInt(0);
                    d.routeSteps.append(step);
                }
            });
            return;
        }
}
void StudioWindow::deleteGate() {
    const QString id = selected_;
    if (id.isEmpty())
        return;
    selected_.clear();
    selectedRouteStep_.clear();
    edit(i18n::text("删除门架"), [id](TrackDocument &d) {
        d.routeSteps.erase(
            std::remove_if(d.routeSteps.begin(), d.routeSteps.end(),
                           [id](const RouteStep &step) { return step.gateId == id; }),
            d.routeSteps.end());
        for (int i = 0; i < d.gates.size(); ++i)
            if (d.gates[i].id == id) {
                d.gates.removeAt(i);
                return;
            }
    });
}
void StudioWindow::rotateGate(double delta) {
    const QString id = selected_;
    edit(i18n::text("旋转门架"), [id, delta](TrackDocument &d) {
        for (auto &g : d.gates)
            if (g.id == id)
                g.yaw = std::remainder(g.yaw + delta, 360.);
    });
}
void StudioWindow::moveOrder(int delta) {
    const QString id = selected_;
    edit(i18n::text("调整通过顺序"), [id, delta](TrackDocument &d) {
        for (int i = 0; i < d.gates.size(); ++i)
            if (d.gates[i].id == id) {
                int j = i + delta;
                if (j >= 0 && j < d.gates.size())
                    d.gates.move(i, j);
                return;
            }
    });
}
void StudioWindow::fail(const QString &message) {
    QMessageBox::warning(this, "OpenTrack Studio", i18n::text(message));
}
bool StudioWindow::maybeSave() {
    if (undo_.isClean())
        return true;
    const auto answer = QMessageBox::question(
        this, i18n::text("保存工程"), i18n::text("当前工程有未保存的修改，是否保存？"),
        QMessageBox::Save | QMessageBox::Discard | QMessageBox::Cancel, QMessageBox::Save);
    if (answer == QMessageBox::Cancel)
        return false;
    if (answer == QMessageBox::Save)
        return save();
    return true;
}
void StudioWindow::newProject(bool demo) {
    if (!maybeSave())
        return;
    path_.clear();
    selected_.clear();
    undo_.clear();
    auto fresh = demo ? demoDocument() : TrackDocument();
    fresh.name = i18n::text(fresh.name); // User/opened project names are never translated.
    applyDocument(fresh);
    undo_.setClean();
    fitViews();
    notice_->setText(demo ? i18n::text("示例赛道已加载 · 先点击选中门架，再按住拖动。")
                          : i18n::text("新工程 · 导入赛道图开始识别，或从右侧组件库放置门架。"));
}
void StudioWindow::openProject() {
    if (!maybeSave())
        return;
    const QString path =
        QFileDialog::getOpenFileName(this, i18n::text("打开工程或 VelociDrone 赛道"), {},
                                     i18n::text("工程 / 赛道 (*.otproj *.trk);;所有文件 (*)"));
    if (!path.isEmpty())
        openPath(path);
}
bool StudioWindow::openPath(const QString &p) {
    TrackDocument d;
    QString error;
    const bool trk = p.endsWith(".trk", Qt::CaseInsensitive);
    if (!(trk ? importVelocidrone(p, &d, &error) : loadProject(p, &d, &error))) {
        fail(error);
        return false;
    }
    path_ = trk ? QString() : p;
    selected_.clear();
    undo_.clear();
    applyDocument(d);
    undo_.setClean();
    fitViews();
    notice_->setText(trk ? i18n::text("VelociDrone 赛道已导入 · 可编辑后导出。")
                         : i18n::text("工程已打开 · 底图、门架和通过顺序已恢复。"));
    return true;
}
bool StudioWindow::save(bool as) {
    QString p = path_;
    if (as || p.isEmpty())
        p = QFileDialog::getSaveFileName(this, i18n::text("保存工程"), doc_.name + ".otproj",
                                         i18n::text("OpenTrack 工程 (*.otproj)"));
    if (p.isEmpty())
        return false;
    if (!p.endsWith(".otproj", Qt::CaseInsensitive))
        p += ".otproj";
    QString error;
    if (!saveProject(p, doc_, &error)) {
        fail(error);
        return false;
    }
    path_ = p;
    undo_.setClean();
    statusBar()->showMessage(i18n::text("工程已保存：") + QFileInfo(p).fileName(), 5000);
    return true;
}
void StudioWindow::exportTrack() {
    if (!doc_.takeoff.enabled) {
        setPlacement("__takeoff", .8, .1);
        notice_->setText(i18n::text("尚未设置独立起飞点。请在地面单击放置起飞架，再导出赛道。"));
        statusBar()->showMessage(i18n::text("起飞点必须单独设置，不能用第一个门代替"), 8000);
        return;
    }
    const auto steps = effectiveRouteSteps(doc_);
    if (std::none_of(steps.begin(), steps.end(), [](const RouteStep &step) {
            return step.kind == "pass" ||
                   (step.kind == "guide" && step.extra.value("raceCheckpoint").toBool());
        })) {
        fail(i18n::text("请先设置穿越步骤或翻绕检查点。"));
        return;
    }
    QString p = QFileDialog::getSaveFileName(
        this, i18n::text("导出 VelociDrone 赛道"),
        QDir(QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation))
            .filePath(doc_.name + ".trk"),
        i18n::text("VelociDrone 赛道 (*.trk)"));
    if (p.isEmpty())
        return;
    if (!p.endsWith(".trk", Qt::CaseInsensitive))
        p += ".trk";
    QString error;
    if (!exportVelocidrone(p, doc_, &error)) {
        fail(error);
        return;
    }
    notice_->setText(i18n::text("已导出 .trk · 在 VelociDrone 的 Track Editor → Import Track "
                                "中导入。请在模拟器内检查高度、朝向和通过顺序。"));
    statusBar()->showMessage(i18n::text("赛道已导出：") + p, 8000);
}
void StudioWindow::exportPng() {
    QString p = QFileDialog::getSaveFileName(this, i18n::text("导出赛道预览"), doc_.name + ".png",
                                             i18n::text("PNG 图片 (*.png)"));
    if (p.isEmpty())
        return;
    if (!p.endsWith(".png", Qt::CaseInsensitive))
        p += ".png";
    if (!(scene3d_ && views_->currentWidget() == scene3d_ ? scene3d_->exportImage()
                                                          : canvas_->exportImage())
             .save(p))
        fail(i18n::text("图片保存失败。"));
    else
        statusBar()->showMessage(i18n::text("赛道预览已保存"), 5000);
}
void StudioWindow::importPhotosWithAi(const QStringList &paths) {
    AiVisionDialog dialog(this);
    QTemporaryDir previewFiles;
    if (!paths.isEmpty()) {
        dialog.setImagePaths(paths);
    } else if (!doc_.background.isNull() && previewFiles.isValid()) {
        const QString current = previewFiles.filePath(i18n::text("当前赛道图.png"));
        if (doc_.background.save(current))
            dialog.setImagePaths({current});
    }
    if (dialog.exec() != QDialog::Accepted || !dialog.result().ok)
        return;
    const auto &result = dialog.result();
    selected_.clear();
    pushEdit(i18n::text("AI 照片生成赛道"), doc_, result.document);
    fitViews();
    notice_->setText(
        i18n::text("AI 草稿 · %1 个门架 · 请校对真实距离、高度、朝向和通过顺序，修改可撤销。")
            .arg(doc_.gates.size()));
    notice_->setToolTip("<pre>" + i18n::messages(result.warnings).join("\n").toHtmlEscaped() +
                        "</pre>");
}
void StudioWindow::importImage() {
    const QString p = QFileDialog::getOpenFileName(
        this, i18n::text("导入赛道俯视图"), {}, i18n::text("赛道图片 (*.png *.jpg *.jpeg *.bmp)"));
    if (p.isEmpty())
        return;
    importImagePath(p);
}
void StudioWindow::importImagePath(const QString &path) {
    QMimeData mime;
    mime.setUrls({QUrl::fromLocalFile(path)});
    const auto checked = inspectImageDrop(&mime);
    if (!checked.supported) {
        fail(checked.error);
        return;
    }
    setPlacement({});
    runImageRecognition(checked.paths.front(), {});
}
void StudioWindow::startRecognition(bool confirmReplace) {
    if (recognizing_ || doc_.background.isNull())
        return;
    if (confirmReplace && !doc_.gates.isEmpty()) {
        if (QMessageBox::question(this, i18n::text("生成赛道"),
                                  i18n::text("识别结果将替换当前门架，可通过撤销恢复。继续生成？"),
                                  QMessageBox::Yes | QMessageBox::No) != QMessageBox::Yes)
            return;
    }
    runImageRecognition({}, doc_.background);
}
void StudioWindow::runImageRecognition(const QString &imagePath, const QImage &image) {
    recognizing_ = true;
    recognizeButton_->setEnabled(false);
    recognizeButton_->setText(i18n::text("正在本地生成…"));
    recognitionStartedMs_ = QDateTime::currentMSecsSinceEpoch();
    recognitionTimer_->start();
    const quint64 serial = ++recognitionSerial_;
    const quint64 revision = revision_;
    RecognitionOptions options;
    options.autoScale = autoScale_->isChecked();
    options.metersPerPixel = scale_->value();
    auto *watcher = new QFutureWatcher<ImageWork>(this);
    statusBar()->showMessage(i18n::text("正在读取并解析赛道图…"));
    connect(
        watcher, &QFutureWatcher<ImageWork>::finished, this,
        [this, watcher, revision, serial, imagePath] {
            const auto work = watcher->result();
            watcher->deleteLater();
            if (serial != recognitionSerial_)
                return;
            recognizing_ = false;
            recognitionTimer_->stop();
            recognizeButton_->setEnabled(!doc_.background.isNull());
            recognizeButton_->setText(i18n::text("识别图片并生成赛道"));
            if (revision != revision_) {
                notice_->setText(
                    i18n::text("识别期间工程已改变，本次结果已丢弃。请在当前工程重新识别。"));
                return;
            }
            if (!work.error.isEmpty()) {
                fail(work.error);
                return;
            }
            const auto &result = work.recognition;
            if (result.detectedCount == 0 && imagePath.isEmpty()) {
                notice_->setText(i18n::text(
                    "没有可靠的识别结果，当前组件已保留。可用“AI 解析当前图”生成预览草稿。"));
                return;
            }
            auto after = result.document;
            after.name = imagePath.isEmpty() ? doc_.name : QFileInfo(imagePath).completeBaseName();
            after.scenery = doc_.scenery;
            selected_.clear();
            pushEdit(i18n::text("图片生成赛道"), doc_, after);
            recognizeButton_->setEnabled(!doc_.background.isNull());
            fitViews();
            int passes = 0;
            for (const auto &step : effectiveRouteSteps(after))
                passes += step.kind == "pass";
            const auto evidence = after.extra.value("recognition").toObject();
            const int uncertain = evidence.value("ambiguousPassageGroups").toInt();
            const bool orderConfirmed = evidence.value("sourceOrderConfirmed").toBool();
            const QString review =
                !orderConfirmed ? i18n::text("门序尚未确认，请校对起点、箭头与过门顺序。")
                : uncertain ? i18n::text("%1 组遮挡待校对；请核对三维高度与绕行。").arg(uncertain)
                            : i18n::text("请核对三维高度与通过顺序。");
            notice_->setText(
                result.detectedCount
                    ? i18n::text("已生成 %1 个实体 / %2 次过门 · 本地 %3 秒 · %4")
                          .arg(result.detectedCount)
                          .arg(passes)
                          .arg(work.elapsedMs / 1000.0, 0, 'f', 2)
                          .arg(review)
                    : i18n::text("图片已导入，未生成不可靠的组件。可点击“AI 解析当前图”继续。"));
            notice_->setToolTip(
                "<pre>" + i18n::messages(result.warnings).join("\n").toHtmlEscaped() + "</pre>");
            statusBar()->showMessage(i18n::text("导入完成 · Ctrl+Z 可恢复导入前的工程"), 6000);
        });
    watcher->setFuture(QtConcurrent::run([imagePath, image, options] {
        QElapsedTimer timer;
        timer.start();
        ImageWork work;
        QImage input = image;
        if (!imagePath.isEmpty()) {
            QImageReader reader(imagePath);
            reader.setAutoTransform(true);
            const QSize size = reader.size();
            if (!size.isValid() || qint64(size.width()) * size.height() > 80000000) {
                work.error = i18n::text("图片尺寸无效或超过限制。");
                return work;
            }
            if (size.width() > 3200 || size.height() > 3200)
                reader.setScaledSize(size.scaled(3200, 3200, Qt::KeepAspectRatio));
            input = reader.read();
        }
        if (input.isNull()) {
            work.error = i18n::text("无法读取图片，请检查文件是否完整。");
            return work;
        }
        work.recognition = recognizeTrack(input, options);
        work.elapsedMs = timer.elapsed();
        return work;
    }));
}
void StudioWindow::closeEvent(QCloseEvent *e) {
    if (maybeSave())
        e->accept();
    else
        e->ignore();
}
bool StudioWindow::eventFilter(QObject *target, QEvent *event) {
    const auto highlight = [this](bool active) {
        if (dropZone_->property("dragOver").toBool() == active)
            return;
        dropZone_->setProperty("dragOver", active);
        dropZone_->style()->unpolish(dropZone_);
        dropZone_->style()->polish(dropZone_);
        dropZone_->update();
    };
    if (event->type() == QEvent::DragEnter || event->type() == QEvent::DragMove) {
        auto *drag = static_cast<QDragMoveEvent *>(event);
        const auto checked = inspectImageDrop(drag->mimeData());
        const bool accepted = checked.supported && drag->possibleActions().testFlag(Qt::CopyAction);
        highlight(accepted);
        if (accepted) {
            drag->setDropAction(Qt::CopyAction);
            drag->accept();
        } else {
            drag->ignore();
            statusBar()->showMessage(i18n::text(checked.error), 5000);
        }
        return true;
    }
    if (event->type() == QEvent::DragLeave) {
        highlight(false);
        event->accept();
        return true;
    }
    if (event->type() == QEvent::Drop) {
        auto *drop = static_cast<QDropEvent *>(event);
        highlight(false);
        const auto checked = inspectImageDrop(drop->mimeData());
        if (!checked.supported || !drop->possibleActions().testFlag(Qt::CopyAction)) {
            statusBar()->showMessage(i18n::text(checked.error), 5000);
            drop->ignore();
            return true;
        }
        drop->setDropAction(Qt::CopyAction);
        drop->accept();
        if (checked.paths.size() == 1)
            importImagePath(checked.paths.front());
        else
            QTimer::singleShot(0, this,
                               [this, paths = checked.paths] { importPhotosWithAi(paths); });
        return true;
    }
    return QMainWindow::eventFilter(target, event);
}
QImage makeExampleImage() {
    QImage img(1400, 900, QImage::Format_RGB32);
    img.fill(QColor("#fcfdfd"));
    QPainter p(&img);
    p.setRenderHint(QPainter::Antialiasing);
    p.setPen(QPen(QColor("#eef1f4"), 1));
    for (int x = 0; x < 1400; x += 40)
        p.drawLine(x, 0, x, 900);
    for (int y = 0; y < 900; y += 40)
        p.drawLine(0, y, 1400, y);
    const QVector<QPointF> points = {{190, 620},  {200, 400},  {290, 220},  {500, 160},  {750, 170},
                                     {1010, 240}, {1170, 410}, {1190, 650}, {1000, 730}, {780, 680},
                                     {690, 480},  {490, 560},  {420, 720}};
    QPainterPath route;
    route.moveTo(points[0]);
    for (int i = 1; i < points.size(); ++i)
        route.lineTo(points[i]);
    route.closeSubpath();
    p.setPen(QPen(QColor("#e8b74b"), 3));
    p.drawPath(route);
    for (int i = 0; i < points.size(); ++i) {
        p.save();
        p.translate(points[i]);
        p.setPen(QPen(QColor("#3688cf"), 4));
        p.setBrush(Qt::NoBrush);
        p.drawRect(QRectF(-24, -20, 48, 40));
        p.setPen(QPen(QColor("#7cadd5"), 2));
        p.drawLine(QPointF(-24, -20), QPointF(-17, -26));
        p.drawLine(QPointF(-17, -26), QPointF(31, -26));
        p.drawLine(QPointF(24, -20), QPointF(31, -26));
        p.drawLine(QPointF(31, -26), QPointF(31, 14));
        p.drawLine(QPointF(24, 20), QPointF(31, 14));
        p.restore();
    }
    p.setPen(QColor("#7c8999"));
    p.setFont(QFont("Segoe UI", 12));
    p.drawText(QRect(35, 30, 700, 32), "OpenTrack Studio  /  Example overhead track plan");
    return img;
}
bool StudioWindow::runLanguageSmokeTest(const QString &dir) {
    QSettings preferences;
    const bool hadPreference = preferences.contains("interface/language");
    const QVariant savedPreference = preferences.value("interface/language");
    const QString initialLanguage = i18n::language();
    struct RestorePreference {
        bool had;
        QVariant value;
        QString language;
        ~RestorePreference() {
            i18n::setLanguage(language);
            QSettings settings;
            if (had)
                settings.setValue("interface/language", value);
            else
                settings.remove("interface/language");
        }
    } restore{hadPreference, savedPreference, initialLanguage};
    const auto original = doc_;
    undo_.clear();
    edit("Language smoke edit", [](TrackDocument &document) {
        document.name = QString::fromUtf8("User-defined 用户 project");
        document.gates[0].position += QPointF(1.5, .5);
    });
    routeOrder_->setCurrentRow(0);
    findChild<QCheckBox *>("snapToGrid")->setChecked(false);
    findChild<QCheckBox *>("showRoute")->setChecked(false);
    findChild<QComboBox *>("previewSpeed")->setCurrentIndex(2);
    const auto edited = toJson(doc_);
    const QString selected = selected_, step = selectedRouteStep_;
    const int undoCount = undo_.count(), undoIndex = undo_.index();
    changeLanguage("en");
    if (i18n::catalog().size() < 500 || toJson(doc_) != edited || selected_ != selected ||
        selectedRouteStep_ != step || undo_.count() != undoCount || undo_.index() != undoIndex ||
        !isWindowModified() || name_->text() != QString::fromUtf8("User-defined 用户 project") ||
        findChild<QCheckBox *>("snapToGrid")->isChecked() ||
        findChild<QCheckBox *>("showRoute")->isChecked() ||
        findChild<QComboBox *>("previewSpeed")->currentIndex() != 2)
        return false;
    const QRegularExpression han(QStringLiteral("[\\x{3400}-\\x{9fff}]"));
    const auto englishWidgets = [&](QWidget *root) {
        for (auto *widget : root->findChildren<QWidget *>()) {
            if (widget->objectName() == "languageSelector")
                continue; // Language endonyms are intentionally stable.
            QStringList visibleText;
            if (auto *value = qobject_cast<QLabel *>(widget))
                visibleText << value->text();
            if (auto *value = qobject_cast<QAbstractButton *>(widget))
                visibleText << value->text();
            if (auto *value = qobject_cast<QGroupBox *>(widget))
                visibleText << value->title();
            if (auto *value = qobject_cast<QComboBox *>(widget))
                for (int i = 0; i < value->count(); ++i)
                    visibleText << value->itemText(i);
            if (auto *value = qobject_cast<QTabWidget *>(widget))
                for (int i = 0; i < value->count(); ++i)
                    visibleText << value->tabText(i);
            if (auto *value = qobject_cast<QLineEdit *>(widget))
                visibleText << value->placeholderText(); // Never inspect user text or credentials.
            if (auto *value = qobject_cast<QPlainTextEdit *>(widget))
                visibleText << value->placeholderText();
            visibleText << widget->toolTip();
            for (const auto &text : visibleText)
                if (text.contains(han)) {
                    qWarning() << "Untranslated generated UI text in"
                               << widget->metaObject()->className() << widget->objectName();
                    return false;
                }
        }
        return true;
    };
    if (!englishWidgets(this))
        return false;
    {
        AiVisionDialog dialog(this, false); // No credential read or network request in this test.
        if (!englishWidgets(&dialog))
            return false;
    }
    if (i18n::text(QString::fromUtf8("字段 position 必须是数字。")) !=
            "Field position must be a number." ||
        i18n::text(QString::fromUtf8("组件颜色必须是不透明的 RGB 颜色。")) !=
            "Obstacle color must be an opaque RGB color.")
        return false;
    QDir().mkpath(dir);
    QApplication::processEvents();
    QEventLoop renderEvents;
    QTimer::singleShot(120, &renderEvents, &QEventLoop::quit);
    renderEvents.exec();
    const auto capture =
        QGuiApplication::platformName() == "windows" ? screen()->grabWindow(winId()) : grab();
    if (!capture.save(dir + "/english-ui.png"))
        return false;
    changeLanguage("zh");
    if (toJson(doc_) != edited || selected_ != selected || selectedRouteStep_ != step ||
        undo_.count() != undoCount || undo_.index() != undoIndex ||
        findChild<QCheckBox *>("snapToGrid")->isChecked() ||
        findChild<QCheckBox *>("showRoute")->isChecked() ||
        findChild<QComboBox *>("previewSpeed")->currentIndex() != 2)
        return false;
    undo_.undo();
    if (!sameDocument(doc_, original))
        return false;
    undo_.redo();
    if (toJson(doc_) != edited)
        return false;
    const auto buttons = findChildren<QPushButton *>();
    const bool chineseControls =
        std::any_of(buttons.cbegin(), buttons.cend(), [](QPushButton *button) {
            return button->text() == QString::fromUtf8("保存工程");
        });
    if (!chineseControls)
        return false;
    QFile report(dir + "/language-report.json");
    if (!report.open(QIODevice::WriteOnly))
        return false;
    report.write(QJsonDocument(QJsonObject{{"englishMainAndAiControls", true},
                                           {"liveSwitchPreservesProjectSelectionAndUndo", true},
                                           {"liveSwitchPreservesEditingControls", true},
                                           {"userTextUnchanged", true},
                                           {"credentialRead", false},
                                           {"networkRequests", false},
                                           {"preferenceRestored", true}})
                     .toJson());
    undo_.clear();
    return true;
}

bool StudioWindow::runSmokeTest(const QString &dir) {
    QDir().mkpath(dir);
    QFile::remove(dir + "/failure.json");
    const auto smokeFailure = [this, dir](int line) {
        QFile failure(dir + "/failure.json");
        if (failure.open(QIODevice::WriteOnly)) {
            const QJsonObject details{{"line", line},
                                      {"gates", doc_.gates.size()},
                                      {"recognizing", recognizing_},
                                      {"revision", qint64(revision_)},
                                      {"view", views_->currentWidget() == scene3d_ ? "3D" : "2D"}};
            failure.write(QJsonDocument(details).toJson());
        }
        return false;
    };

    QString error;
    const QImage sample = makeExampleImage();
    if (!sample.save(dir + "/example-track.png"))
        return smokeFailure(__LINE__);
    const TrackDocument original = doc_;
    QWidget *initialView = views_->currentWidget();
    // File drop follows the real asynchronous import path, including a single undo transaction.
    for (auto *target : QList<QWidget *>{dropZone_, canvas_->viewport(), scene3d_}) {
        if (!target)
            continue;
        if (target == canvas_->viewport())
            views_->setCurrentWidget(canvas_);
        else if (target == scene3d_)
            views_->setCurrentWidget(scene3d_);
        QApplication::processEvents();
        const TrackDocument beforeDrop = doc_;
        QMimeData mime;
        mime.setUrls({QUrl::fromLocalFile(QDir(dir).absoluteFilePath("example-track.png"))});
        QDragEnterEvent enter(QPoint(12, 12), Qt::CopyAction | Qt::MoveAction, &mime,
                              Qt::LeftButton, Qt::NoModifier);
        QApplication::sendEvent(target, &enter);
        QDropEvent drop(QPointF(12, 12), Qt::CopyAction | Qt::MoveAction, &mime, Qt::LeftButton,
                        Qt::NoModifier);
        QApplication::sendEvent(target, &drop);
        if (!enter.isAccepted() || !drop.isAccepted() || drop.dropAction() != Qt::CopyAction) {
            qWarning() << "Drop test rejected" << target->metaObject()->className()
                       << enter.isAccepted() << drop.isAccepted() << drop.dropAction();
            return smokeFailure(__LINE__);
        }
        QElapsedTimer wait;
        wait.start();
        while (recognizing_ && wait.elapsed() < 5000)
            QApplication::processEvents(QEventLoop::AllEvents, 20);
        if (recognizing_ || doc_.gates.size() != 13 || doc_.background.isNull()) {
            qWarning() << "Drop import failed" << recognizing_ << doc_.gates.size();
            return smokeFailure(__LINE__);
        }
        undo_.undo();
        if (!sameDocument(doc_, beforeDrop)) {
            qWarning() << "Drop undo failed" << target->metaObject()->className();
            return smokeFailure(__LINE__);
        }
        undo_.redo();
        if (doc_.gates.size() != 13)
            return smokeFailure(__LINE__);
    }
    views_->setCurrentWidget(initialView);
    applyDocument(original);
    undo_.clear();
    const auto result = recognizeTrack(sample);
    if (result.detectedCount != 13) {
        qWarning() << "Recognition count" << result.detectedCount << result.warnings;
        return smokeFailure(__LINE__);
    }
    if (result.document.gates.isEmpty() ||
        !gateFrameMetrics(result.document.gates[0]).centerlineModel)
        return smokeFailure(__LINE__);
    TrackDocument before = doc_;
    // A filled lower cell must remain unavailable in the actual editing controls.
    TrackDocument blockedUi;
    blockedUi.gates = {makeGate("quad"), makeGate("ladder-blocked")};
    blockedUi.gates[1].position = QPointF(15, 0);
    applyDocument(blockedUi);
    undo_.clear();
    routeOrder_->setCurrentRow(1);
    auto *levelModel = qobject_cast<QStandardItemModel *>(passLevel_->model());
    if (!levelModel || !levelModel->item(0) || levelModel->item(0)->isEnabled() ||
        passLevel_->currentIndex() != 1)
        return smokeFailure(__LINE__);
    const auto blockedBefore = toJson(doc_);
    passLevel_->setCurrentIndex(0); // A stale/programmatic index must be rejected too.
    if (toJson(doc_) != blockedBefore || passLevel_->currentIndex() != 1)
        return smokeFailure(__LINE__);
    maneuver_->setCurrentIndex(maneuver_->findData(QString("lower-upper")));
    if (findChild<QPushButton *>("applyManeuver")->isEnabled())
        return smokeFailure(__LINE__);
    maneuver_->setCurrentIndex(maneuver_->findData(QString("single")));
    if (!findChild<QPushButton *>("applyManeuver")->isEnabled())
        return smokeFailure(__LINE__);
    addRoutePass();
    for (const auto &step : effectiveRouteSteps(doc_))
        if (step.gateId == blockedUi.gates[1].id &&
            (step.level != 1 || !routeStepGeometry(doc_, step).valid))
            return smokeFailure(__LINE__);
    // Exercise the actual route controls: one physical assembly, repeated passes,
    // independent cells/directions, and contextual toolbar actions.
    TrackDocument maneuvers;
    maneuvers.gates = {makeGate("ladder"), makeGate("triple"), makeGate("cube")};
    maneuvers.gates[1].position = QPointF(12, 0);
    maneuvers.gates[2].position = QPointF(24, 0);
    applyDocument(maneuvers);
    undo_.clear();
    editTakeoff(); // A focus-out with unchanged numeric values must not configure a spawn.
    if (doc_.takeoff.enabled)
        return smokeFailure(__LINE__);
    canvas_->onPlace("__takeoff", QPointF(0, -6), .8, .1);
    if (!doc_.takeoff.enabled || doc_.gates.size() != 3 || routeOrder_->count() != 3)
        return smokeFailure(__LINE__);
    undo_.undo();
    if (doc_.takeoff.enabled)
        return smokeFailure(__LINE__);
    undo_.redo();
    takeoffZ_->setValue(-.86);
    if (!QMetaObject::invokeMethod(takeoffZ_, "editingFinished", Qt::DirectConnection) ||
        std::abs(doc_.takeoff.elevation + .86) > 1e-6)
        return smokeFailure(__LINE__);
    routeOrder_->setCurrentRow(0);
    if (selectedRouteStep_.isEmpty() || selected_ != doc_.gates[0].id)
        return smokeFailure(__LINE__);
    const int ladderPattern = maneuver_->findData(QString("lower-upper"));
    if (ladderPattern < 0)
        return smokeFailure(__LINE__);
    maneuver_->setCurrentIndex(ladderPattern);
    auto clickButton = [this](const QString &text) {
        for (auto *control : findChildren<QPushButton *>())
            if (control->text() == text) {
                control->click();
                return true;
            }
        return false;
    };
    if (!clickButton(i18n::text("应用动作")) || doc_.gates.size() != 3 || routeOrder_->count() != 4)
        return smokeFailure(__LINE__);
    QVector<RouteStep> ladderPasses;
    for (const auto &step : effectiveRouteSteps(doc_))
        if (step.kind == "pass" && step.gateId == doc_.gates[0].id)
            ladderPasses.append(step);
    if (ladderPasses.size() != 2 || ladderPasses[0].level != 0 || ladderPasses[1].level != 1 ||
        ladderPasses[1].face != "back")
        return smokeFailure(__LINE__);
    const auto beforeThreading = doc_;
    const int uprightPattern = maneuver_->findData(QString("upright-left-up"));
    if (uprightPattern < 0)
        return smokeFailure(__LINE__);
    maneuver_->setCurrentIndex(uprightPattern);
    if (!clickButton(i18n::text("应用动作")) || routeOrder_->count() != 4)
        return smokeFailure(__LINE__);
    QVector<RouteStep> threadedPasses;
    for (const auto &step : effectiveRouteSteps(doc_))
        if (step.kind == "pass" && step.gateId == doc_.gates[0].id)
            threadedPasses.append(step);
    if (threadedPasses.size() != 2 || threadedPasses[0].level != 0 ||
        threadedPasses[1].level != 1 ||
        (routeStepGeometry(doc_, threadedPasses[1]).pathPosition -
         routeStepGeometry(doc_, threadedPasses[0]).pathPosition)
                .y() < 2.9f)
        return smokeFailure(__LINE__);
    if (!saveProject(dir + "/threading-demo.otproj", doc_, &error) ||
        !exportVelocidrone(dir + "/threading-demo.trk", doc_, &error))
        return smokeFailure(__LINE__);
    edit(i18n::text("准备识别动作预览检查"), [](TrackDocument &document) {
        auto &gate = document.gates[0];
        gate.extra.insert("recognition", QJsonObject{{"routeEvidence", "synthetic-source-stroke"}});
        gate.extra.remove("routePattern");
    });
    if (maneuver_->currentData().toString() != "source-route" ||
        findChild<QPushButton *>("applyManeuver")->isEnabled())
        return smokeFailure(__LINE__);
    routeOrder_->setCurrentRow(0);
    const auto crossingBeforeReverse = routeStepGeometry(doc_, doc_.routeSteps[0]);
    stepReverse_->setCurrentIndex(doc_.routeSteps[0].reverse ? 0 : 1);
    const auto crossingAfterReverse = routeStepGeometry(doc_, doc_.routeSteps[0]);
    if (!crossingAfterReverse.valid ||
        (crossingAfterReverse.pathPosition - crossingBeforeReverse.pathPosition).length() > .001f ||
        QVector3D::dotProduct(crossingAfterReverse.pathTangent, crossingBeforeReverse.pathTangent) >
            -.99f)
        return smokeFailure(__LINE__);
    undo_.undo();
    undo_.undo();
    undo_.undo();
    if (!sameDocument(doc_, beforeThreading))
        return smokeFailure(__LINE__);
    const auto beforeOrbit = doc_;
    const int orbitPattern = maneuver_->findData(QString("orbit-bar-cw"));
    if (orbitPattern < 0)
        return smokeFailure(__LINE__);
    maneuver_->setCurrentIndex(orbitPattern);
    if (maneuverPreview_->isHidden() || !clickButton(i18n::text("应用动作")) ||
        doc_.gates.size() != 3 || routeOrder_->count() != 5)
        return smokeFailure(__LINE__);
    int turnChecks = 0;
    for (const auto &step : effectiveRouteSteps(doc_))
        turnChecks += step.kind == "guide" && step.extra.value("raceCheckpoint").toBool();
    if (turnChecks != 2 || !routeStatus_->text().contains(i18n::text("2 个辅助翻绕检查点")) ||
        doc_.gates[0].extra.value("routePattern") != QJsonValue("orbit-bar-cw"))
        return smokeFailure(__LINE__);
    if (!saveProject(dir + "/circle-demo.otproj", doc_, &error) ||
        !exportVelocidrone(dir + "/circle-demo.trk", doc_, &error))
        return smokeFailure(__LINE__);
    undo_.undo();
    if (!sameDocument(doc_, beforeOrbit) || routeOrder_->count() != 4)
        return smokeFailure(__LINE__);
    routeOrder_->setCurrentRow(1);
    stepReverse_->setCurrentIndex(1);
    const QString editedPass = selectedRouteStep_;
    bool editedDirection = false;
    for (const auto &step : effectiveRouteSteps(doc_))
        if (step.id == editedPass)
            editedDirection = step.reverse && step.level == 1 && step.face == "back";
    if (!editedDirection || doc_.gates.size() != 3)
        return smokeFailure(__LINE__);
    if (!clickButton(i18n::text("复制选择")) || routeOrder_->count() != 5 || doc_.gates.size() != 3)
        return smokeFailure(__LINE__);
    const QString repeatedPass = selectedRouteStep_;
    moveRoutePass(1);
    if (selectedRouteStep_ != repeatedPass || routeOrder_->currentRow() != 3)
        return smokeFailure(__LINE__);
    if (!clickButton(i18n::text("删除选择")) || routeOrder_->count() != 4 || doc_.gates.size() != 3)
        return smokeFailure(__LINE__);
    undo_.undo();
    if (routeOrder_->count() != 5 || doc_.gates.size() != 3)
        return smokeFailure(__LINE__);
    routeOrder_->setCurrentRow(0);
    sequenceTabs_->setCurrentIndex(1);
    if (!selectedRouteStep_.isEmpty() || !stepPanel_->isHidden())
        return smokeFailure(__LINE__);
    if (!clickButton(i18n::text("复制选择")) || doc_.gates.size() != 4 || routeOrder_->count() != 6)
        return smokeFailure(__LINE__);
    undo_.undo();
    const QString removedGate = doc_.gates[0].id;
    canvas_->selectGate(removedGate);
    updateSelection(removedGate);
    if (!clickButton(i18n::text("删除选择")) || doc_.gates.size() != 2)
        return smokeFailure(__LINE__);
    for (const auto &step : effectiveRouteSteps(doc_))
        if (step.gateId == removedGate)
            return smokeFailure(__LINE__);
    undo_.undo();
    if (doc_.gates.size() != 3 || !doc_.takeoff.enabled)
        return smokeFailure(__LINE__);
    auto scaling = doc_;
    RouteStep freeGuide;
    freeGuide.id = newGateId();
    freeGuide.kind = "guide";
    freeGuide.localGuide = QVector3D(9, 4, -7);
    scaling.routeSteps.append(freeGuide);
    applyDocument(scaling);
    scale_->setValue(scaling.metersPerPixel * 2);
    if (!QMetaObject::invokeMethod(scale_, "editingFinished", Qt::DirectConnection) ||
        doc_.takeoff.position != scaling.takeoff.position * 2 ||
        doc_.gates[1].position != scaling.gates[1].position * 2 ||
        doc_.routeSteps.last().localGuide != QVector3D(18, 4, -14))
        return smokeFailure(__LINE__);
    undo_.undo();
    if (!sameDocument(doc_, scaling))
        return smokeFailure(__LINE__);

    applyDocument(result.document);
    undo_.clear();
    selectedRouteStep_.clear();
    canvas_->onPlace("__takeoff", QPointF(-4, -6), .8, .1);
    auto beforeEdit = doc_;
    selected_ = doc_.gates[0].id;
    canvas_->selectGate(selected_);
    duplicateGate();
    if (doc_.gates.size() != 14)
        return smokeFailure(__LINE__);
    undo_.undo();
    if (doc_.gates.size() != 13)
        return smokeFailure(__LINE__);
    undo_.redo();
    if (doc_.gates.size() != 14)
        return smokeFailure(__LINE__);
    undo_.undo();
    const QString project = dir + "/example.otproj", trk = dir + "/example.trk";
    // Exercise the actual property control, document undo, and persisted color together.
    selected_ = doc_.gates[0].id;
    canvas_->selectGate(selected_);
    const QColor originalColor = gateColor(doc_.gates[0]);
    const int redIndex = color_->findData(QString("#e60000"));
    if (redIndex < 0)
        return smokeFailure(__LINE__);
    color_->setCurrentIndex(redIndex);
    if (gateColor(doc_.gates[0]) != QColor("#e60000"))
        return smokeFailure(__LINE__);
    undo_.undo();
    if (gateColor(doc_.gates[0]) != originalColor)
        return smokeFailure(__LINE__);
    undo_.redo();
    if (gateColor(doc_.gates[0]) != QColor("#e60000"))
        return smokeFailure(__LINE__);
    if (!saveProject(project, doc_, &error)) {
        qWarning() << error;
        return smokeFailure(__LINE__);
    }
    TrackDocument restored;
    if (!loadProject(project, &restored, &error) || toJson(restored) != toJson(doc_)) {
        qWarning() << error;
        return smokeFailure(__LINE__);
    }
    if (!exportVelocidrone(trk, doc_, &error)) {
        qWarning() << error;
        return smokeFailure(__LINE__);
    }
    TrackDocument imported;
    if (!importVelocidrone(trk, &imported, &error) || imported.gates.size() != doc_.gates.size() ||
        !imported.takeoff.enabled || imported.takeoff.position != doc_.takeoff.position) {
        qWarning() << error;
        return smokeFailure(__LINE__);
    }
    QElapsedTimer time;
    time.start();
    for (int i = 0; i < 100; ++i) {
        zoomActive(1.01);
        zoomActive(1 / 1.01);
    }
    const qint64 zoomMs = time.elapsed();
    applyDocument(before);
    fitViews();
    QApplication::processEvents();
    if (!grab().save(dir + "/studio-preview.png"))
        return smokeFailure(__LINE__);
    QJsonObject report{{"recognizedGates", result.detectedCount},
                       {"projectRoundtrip", true},
                       {"trackCodecRoundtrip", true},
                       {"undoRedo", true},
                       {"colorPropertyUndoRedo", true},
                       {"imageDropAndAtomicUndo", true},
                       {"routeControlsAndContextualActions", true},
                       {"progressiveUprightThreadingAndSourcePreview", true},
                       {"clockwiseOrbitPreviewAndNativeTurnChecks", true},
                       {"independentTakeoffUndoRedo", true},
                       {"scaleKeepsTakeoffAndGuidesAligned", true},
                       {"zoom200OperationsMs", zoomMs},
                       {"simulatorImportTested", false}};
    QFile file(dir + "/smoke-report.json");
    if (!file.open(QIODevice::WriteOnly))
        return smokeFailure(__LINE__);
    file.write(QJsonDocument(report).toJson());
    undo_.clear();
    undo_.setClean();
    return true;
}
} // namespace ot
