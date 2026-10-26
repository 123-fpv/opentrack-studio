#include "window.h"
#include "aidialog.h"
#include "recognizer.h"
#include "scene3d.h"
#include "velocidrone.h"
#include <QElapsedTimer>
#include <QFutureWatcher>
#include <QImageReader>
#include <QJsonDocument>
#include <QSaveFile>
#include <QtConcurrentRun>
#include <QtWidgets>
#include <cmath>

namespace ot {
namespace {
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
class GateCard final : public QPushButton {
    QString type_, title_, subtitle_;

  public:
    GateCard(QString type, QString title, QString subtitle)
        : type_(type), title_(title), subtitle_(subtitle) {
        setStyleSheet("min-height:104px;max-height:104px;padding:0px;");
        setMinimumWidth(86);
        setCursor(Qt::PointingHandCursor);
        setCheckable(true);
        setAutoExclusive(true);
        setToolTip(title + " · 选中后点击画布放置");
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
        f.setPointSize(9);
        f.setWeight(QFont::DemiBold);
        p.setFont(f);
        p.setPen(QColor("#222222"));
        p.drawText(QRect(0, 64, width(), 18), Qt::AlignCenter, title_);
        f.setWeight(QFont::Normal);
        f.setPointSize(8);
        p.setFont(f);
        p.setPen(QColor("#707070"));
        p.drawText(QRect(0, 83, width(), 15), Qt::AlignCenter, subtitle_);
    }
};
} // namespace

StudioWindow::StudioWindow() {
    doc_ = demoDocument();
    setupUi();
    setupShortcuts();
    canvas_->setDocument(&doc_);
    applyDocument(doc_);
    undo_.setClean();
    setWindowTitle(QString("OpenTrack Studio %1 — VelociDrone 赛道工作台")
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
    titles->addWidget(label("为 VelociDrone 打造的开源赛道工作台", "brandSub"));
    hl->addLayout(titles);
    hl->addSpacing(28);
    hl->addWidget(label("赛道编辑", "headerTab"));
    hl->addStretch();
    hl->addWidget(label("●  本地工程 · 无需登录", "localBadge"));
    auto *imp = button("＋  导入图片", "headerImport");
    auto *importMenu = new QMenu(imp);
    auto *planAction = importMenu->addAction("平面赛道图 · 本地识别");
    auto *photoAction = importMenu->addAction("实拍 / 航拍照片 · AI 生成");
    imp->setMenu(importMenu);
    auto *exp = button("导出赛道 .trk  ↗", "primary");
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
    tool("新建", "Ctrl+N", [this] { newProject(); });
    tool("打开", "Ctrl+O", [this] { openProject(); });
    tool("保存工程", "Ctrl+S", [this] { save(); });
    tl->addSpacing(12);
    auto *undo = tool("↶  撤销", "Ctrl+Z", [this] { undo_.undo(); });
    auto *redo = tool("↷  重做", "Ctrl+Shift+Z", [this] { undo_.redo(); });
    connect(&undo_, &QUndoStack::canUndoChanged, undo, &QPushButton::setEnabled);
    connect(&undo_, &QUndoStack::canRedoChanged, redo, &QPushButton::setEnabled);
    undo->setEnabled(false);
    redo->setEnabled(false);
    tl->addSpacing(12);
    tool("选择 / 移动", "Esc", [this] { setPlacement({}); });
    tool("复制门架", "Ctrl+D", [this] { duplicateGate(); });
    tool("删除", "Delete", [this] { deleteGate(); });
    tl->addStretch();
    auto *snap = new QCheckBox("网格吸附");
    snap->setChecked(true);
    auto *route = new QCheckBox("显示航线");
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
    tool("适应视角", "F", [this] { fitViews(); });
    tool("操作说明", "快速上手", [this] {
        QMessageBox help(this);
        help.setWindowTitle("OpenTrack Studio · 操作说明");
        help.setTextFormat(Qt::PlainText);
        help.setText(
            "① 查看赛道\n右键旋转视角，中键平移，滚轮缩放；F 显示整条赛道。\n\n"
            "② 移动组件\n先单击并松开，看到光圈后，再按住组件或脚下光圈拖动。\n"
            "左侧修改中心距、高度、朝向和颜色；转向柱可选择左右绕行。\n"
            "标准门中心距 3 m，杆长 3.2 m，杆件截面 0.1 × 0.1 m。\n\n"
            "③ 从图片生成\n平面赛道图在本地识别；实拍、航拍照片使用 AI 生成草稿，预览后应用。\n\n"
            "④ 保存和使用\n保存工程得到 .otproj，方便继续编辑。\n"
            "导出赛道得到 .trk，在 VelociDrone 的 Track Editor → Import Track 中导入。\n\n"
            "Ctrl+Z 撤销 · Ctrl+D 复制 · Delete 删除 · Q/E 旋转组件");
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
    ll->addWidget(label("工程设置", "sectionTitle"));
    ll->addWidget(label("赛道名称", "fieldLabel"));
    name_ = new QLineEdit;
    name_->setMaxLength(120);
    ll->addWidget(name_);
    connect(name_, &QLineEdit::editingFinished, this, [this] {
        if (updating_)
            return;
        const QString value = name_->text().trimmed();
        if (!value.isEmpty() && value != doc_.name)
            edit("修改赛道名称", [value](TrackDocument &d) { d.name = value; });
    });
    ll->addWidget(label("模拟器场景", "fieldLabel"));
    scenery_ = new QComboBox;
    scenery_->addItems({"Football Stadium", "Empty Scene Day"});
    ll->addWidget(scenery_);
    connect(scenery_, &QComboBox::currentTextChanged, this, [this](const QString &s) {
        if (!updating_)
            edit("修改场景", [s](TrackDocument &d) { d.scenery = s; });
    });
    auto *stats = new QFrame;
    stats->setObjectName("statsCard");
    auto *sl = new QHBoxLayout(stats);
    auto *a = new QVBoxLayout;
    gateCount_ = label("18", "statValue");
    a->addWidget(gateCount_);
    a->addWidget(label("门架 / 检查点", "subtle"));
    auto *b = new QVBoxLayout;
    length_ = label("0 m", "statValue");
    b->addWidget(length_);
    b->addWidget(label("参考航线长度", "subtle"));
    sl->addLayout(a);
    sl->addLayout(b);
    ll->addWidget(stats);
    ll->addWidget(line());
    ll->addWidget(label("图片生成赛道", "sectionTitle"));
    auto *import = button("导入平面图并识别", "secondary");
    ll->addWidget(import);
    connect(import, &QPushButton::clicked, this, [this] { importImage(); });
    imageStatus_ = label("支持 PNG / JPG / BMP\n请使用蓝色门架符号的俯视图", "subtle");
    imageStatus_->setWordWrap(true);
    ll->addWidget(imageStatus_);
    auto *aiPhotos = button("照片生成赛道（AI）", "secondary");
    ll->addWidget(aiPhotos);
    connect(aiPhotos, &QPushButton::clicked, this, [this] { importPhotosWithAi(); });
    auto *advancedToggle = button("比例与底图设置  ▸", "tool");
    advancedToggle->setCheckable(true);
    ll->addWidget(advancedToggle);
    auto *advanced = new QWidget;
    auto *al = new QVBoxLayout(advanced);
    al->setContentsMargins(0, 0, 0, 0);
    al->setSpacing(9);
    advanced->hide();
    ll->addWidget(advanced);
    connect(advancedToggle, &QPushButton::toggled, advanced, &QWidget::setVisible);
    autoScale_ = new QCheckBox("按中心距 3 m 估算比例");
    autoScale_->setChecked(true);
    al->addWidget(autoScale_);
    al->addWidget(label("底图比例 · 米 / 像素", "fieldLabel"));
    scale_ = number(.0001, 10, .001);
    scale_->setDecimals(4);
    al->addWidget(scale_);
    connect(scale_, &QDoubleSpinBox::editingFinished, this, [this] {
        if (updating_ || qFuzzyCompare(scale_->value(), doc_.metersPerPixel))
            return;
        const double n = scale_->value();
        edit("调整底图比例", [n](TrackDocument &d) {
            const double factor = n / d.metersPerPixel;
            d.backgroundOrigin *= factor;
            for (auto &g : d.gates)
                g.position *= factor;
            d.metersPerPixel = n;
        });
    });
    recognizeButton_ = button("识别图片并生成赛道", "primary");
    ll->addWidget(recognizeButton_);
    connect(recognizeButton_, &QPushButton::clicked, this, [this] { startRecognition(); });
    imageVisible_ = new QCheckBox("显示参考底图");
    imageVisible_->setChecked(true);
    al->addWidget(imageVisible_);
    connect(imageVisible_, &QCheckBox::toggled, this, [this](bool b) {
        if (!updating_)
            edit("切换底图", [b](TrackDocument &d) { d.showBackground = b; });
    });
    al->addWidget(label("底图透明度", "fieldLabel"));
    opacity_ = new QSlider(Qt::Horizontal);
    opacity_->setRange(0, 100);
    al->addWidget(opacity_);
    connect(opacity_, &QSlider::sliderReleased, this, [this] {
        const double n = opacity_->value() / 100.;
        edit("调整底图透明度", [n](TrackDocument &d) { d.backgroundOpacity = n; });
    });
    ll->addWidget(line());
    properties_ = new QWidget;
    auto *pl = new QVBoxLayout(properties_);
    pl->setContentsMargins(0, 0, 0, 0);
    pl->setSpacing(10);
    selectionTitle_ = label("门架属性", "sectionTitle");
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
        {"X 位置 / m", x_}, {"Z 位置 / m", z_},   {"中心高度 / m", alt_},
        {"朝向 / °", yaw_}, {"宽度 / m", width_}, {"高度 / m", height_}};
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
    dimensions_ = label({}, "subtle");
    dimensions_->setObjectName("componentDimensions");
    dimensions_->setWordWrap(true);
    pl->addWidget(dimensions_);
    pl->addWidget(label("组件颜色", "fieldLabel"));
    auto *colorRow = new QHBoxLayout;
    color_ = new QComboBox;
    color_->setObjectName("componentColor");
    color_->setMinimumWidth(80);
    for (const auto &entry : nativeBlockColors()) {
        QPixmap swatch(18, 18);
        swatch.fill(entry.color);
        color_->addItem(QIcon(swatch), entry.name, entry.color.name());
    }
    color_->addItem("自定义颜色", QString());
    colorRow->addWidget(color_, 1);
    auto *customColor = button("调色", "tool");
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
        const QColor chosen = QColorDialog::getColor(initial, this, "选择组件颜色");
        if (chosen.isValid())
            editColor(chosen);
        updateSelection(selected_);
    });
    passSideLabel_ = label("绕柱通过侧", "fieldLabel");
    passSide_ = new QComboBox;
    passSide_->addItems({"右侧绕过", "左侧绕过"});
    pl->addWidget(passSideLabel_);
    pl->addWidget(passSide_);
    connect(passSide_, &QComboBox::currentIndexChanged, this, [this](int index) {
        if (updating_ || selected_.isEmpty())
            return;
        const QString id = selected_;
        edit("调整绕柱侧", [id, index](TrackDocument &d) {
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
    auto *about = label("OPEN SOURCE / MIT\n无需注册 · 支持自带 API", "localNote");
    about->setWordWrap(true);
    ll->addWidget(about);
    auto *center = new QWidget;
    center->setObjectName("center");
    auto *cl = new QVBoxLayout(center);
    cl->setContentsMargins(14, 14, 14, 12);
    cl->setSpacing(10);
    main->addWidget(center);
    auto *ch = new QHBoxLayout;
    ch->addWidget(label("赛道场景", "sectionTitle"));
    auto *view3 = button("3D 视角", "viewToggle");
    auto *view2 = button("俯视图", "viewToggle");
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
    auto *preview = button("▶  航线预览", "secondary");
    preview->setCheckable(true);
    ch->addWidget(preview);
    connect(preview, &QPushButton::toggled, this, [this, preview](bool b) {
        canvas_->setPreview(b);
        if (scene3d_)
            scene3d_->setPreview(b);
        preview->setText(b ? "Ⅱ  暂停预览" : "▶  航线预览");
    });
    auto *speed = new QComboBox;
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
    notice_ = label("示例赛道 · 导入俯视图，即可识别生成并继续编辑。", "notice");
    notice_->setWordWrap(true);
    cl->addWidget(notice_);
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
    orderHead->addWidget(label("通过顺序", "sectionTitle"));
    orderHead->addWidget(label("选择门架后可调整先后", "subtle"));
    orderHead->addStretch();
    auto *prev = button("← 前移", "tool");
    auto *next = button("后移 →", "tool");
    orderHead->addWidget(prev);
    orderHead->addWidget(next);
    connect(prev, &QPushButton::clicked, this, [this] { moveOrder(-1); });
    connect(next, &QPushButton::clicked, this, [this] { moveOrder(1); });
    cl->addLayout(orderHead);
    order_ = new QListWidget;
    order_->setObjectName("orderList");
    order_->setFlow(QListView::LeftToRight);
    order_->setWrapping(false);
    order_->setHorizontalScrollMode(QAbstractItemView::ScrollPerPixel);
    order_->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    order_->setFixedHeight(81);
    order_->setSpacing(5);
    cl->addWidget(order_);
    connect(order_, &QListWidget::currentRowChanged, this, [this](int i) {
        if (!updating_ && i >= 0 && i < doc_.gates.size())
            canvas_->selectGate(doc_.gates[i].id);
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
    rl->addWidget(label("赛道组件", "sectionTitle"));
    rl->addWidget(label("选择组件，再点击画布放置", "subtle"));
    auto *cards = new QGridLayout;
    cards->setSpacing(9);
    struct Definition {
        QString type, title, sub;
        double w, h;
    };
    const QList<Definition> defs = {{"standard", "标准门", "中心距 3 × 3 m", 3.0, 3.0},
                                    {"start", "起终点门", "中心距 3 × 3 m", 3.0, 3.0},
                                    {"pillar", "转向柱", "0.1 × 3.2 m", .1, 3.2}};
    for (int i = 0; i < defs.size(); ++i) {
        const auto d = defs[i];
        auto *card = new GateCard(d.type, d.title, d.sub);
        cards->addWidget(card, i / 2, i % 2);
        connect(card, &QPushButton::clicked, this, [this, d] {
            setPlacement(d.type, d.w, d.h);
            notice_->setText("已选择" + d.title + " · 点击画布放置，Esc 返回选择工具。");
        });
    }
    rl->addLayout(cards);
    auto *stock = label("标准杆件长 3.2 m\n截面 0.1 × 0.1 m\n选中组件可改颜色和尺寸", "subtle");
    stock->setWordWrap(true);
    rl->addWidget(stock);
    rl->addStretch();
    auto *sample = button("加载示例赛道", "secondary");
    rl->addWidget(sample);
    connect(sample, &QPushButton::clicked, this, [this] { newProject(true); });
    auto *png = button("导出赛道预览 PNG", "tool");
    rl->addWidget(png);
    connect(png, &QPushButton::clicked, this, [this] { exportPng(); });
    rl->addWidget(label("拖动空白处旋转视角\n滚轮缩放 · 中键平移", "workflow"));
    main->setStretchFactor(0, 0);
    main->setStretchFactor(1, 1);
    main->setStretchFactor(2, 0);
    main->setSizes({252, 1030, 238});
    statusBar()->setSizeGripEnabled(false);
    statusBar()->showMessage("就绪 · 全部编辑在本机完成");
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
            pushEdit("移动门架", gestureBefore_, doc_);
    };
    if (scene3d_) {
        scene3d_->onSelection = [this](QString id) { canvas_->selectGate(id); };
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

void StudioWindow::fitViews() {
    canvas_->fitTrack();
    if (scene3d_)
        scene3d_->fitTrack();
}
void StudioWindow::setPlacement(const QString &type, double w, double h) {
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
    canvasKey(QKeySequence::Delete, [this] { deleteGate(); });
    canvasKey(QKeySequence("Ctrl+D"), [this] { duplicateGate(); });
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
    recognizeButton_->setEnabled(!doc_.background.isNull() && !recognizing_);
    imageStatus_->setText(doc_.background.isNull()
                              ? "支持 PNG / JPG / BMP\n请使用蓝色门架符号的俯视图"
                              : QString("已导入 %1 × %2 像素\n可先调整比例，再识别生成")
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
            QString("%1  %2\n%3")
                .arg(i + 1, 2, 10, QChar('0'))
                .arg(gateTypeName(g.type), g.label.isEmpty() ? "Checkpoint" : g.label));
        item->setSizeHint(QSize(138, 52));
        order_->addItem(item);
        if (g.id == selected_)
            row = i;
    }
    order_->setCurrentRow(row);
    updating_ = false;
    updateStats();
    canvas_->selectGate(selected_);
    updateSelection(selected_);
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
    gateCount_->setText(QString::number(doc_.gates.size()));
    length_->setText(QString::number(routeLength(doc_), 'f', 1) + " m");
}
void StudioWindow::updateSelection(const QString &id) {
    if (updating_)
        return;
    const bool revealProperties = selected_ != id || properties_->isHidden();
    selected_ = id;
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
    widthLabel_->setText(pillar                  ? "柱体边长 / m"
                         : frame.centerlineModel ? "横向中心距 / m"
                                                 : "宽度 / m");
    heightLabel_->setText(pillar                  ? "柱长 / m"
                          : frame.centerlineModel ? "竖向中心距 / m"
                                                  : "高度 / m");
    passSideLabel_->setVisible(pillar);
    passSide_->setVisible(pillar);
    if (pillar)
        passSide_->setCurrentIndex(gate->extra.value("passSide").toInt(1) < 0 ? 1 : 0);
    for (auto *spin : {x_, z_, alt_, yaw_, width_, height_})
        spin->setEnabled(gate != nullptr);
    selectionTitle_->setText(
        gate ? QString("%1属性 · #%2").arg(gateTypeName(gate->type)).arg(row + 1)
             : "门架属性 · 未选择");
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
        colorMapping_->setText(
            tint == nativeColor.color
                ? QString("方块颜色：%1").arg(nativeColor.name)
                : QString("自定义 %1 · 导出匹配：%2").arg(tint.name().toUpper(), nativeColor.name));
        if (!pillar && !frame.centerlineModel)
            colorMapping_->setText("导入原模型：此颜色用于编辑预览，导出保留原材质。");
        if (pillar) {
            dimensions_->setText(QString("截面 %1 × %1 m · 柱长 %2 m")
                                     .arg(gate->width, 0, 'f', 2)
                                     .arg(gate->height, 0, 'f', 2));
        } else if (frame.centerlineModel) {
            dimensions_->setText(
                QString("杆件截面 %1 × %1 m\n横杆长 %2 m · 竖杆长 %3 m\n净开口 %4 × %5 m")
                    .arg(frame.beamThickness, 0, 'f', 2)
                    .arg(frame.horizontalBeamLength, 0, 'f', 2)
                    .arg(frame.verticalBeamLength, 0, 'f', 2)
                    .arg(frame.openingWidth, 0, 'f', 2)
                    .arg(frame.openingHeight, 0, 'f', 2));
        } else {
            dimensions_->setText("导入组件：原模型尺寸约定");
        }
    }
    order_->setCurrentRow(row);
    if (gate && revealProperties)
        if (auto *sidebar = findChild<QScrollArea *>("sidebar"))
            sidebar->verticalScrollBar()->setValue(0);
    updating_ = false;
    statusBar()->showMessage(
        gate
            ? QString("已选中 #%1 %2 · 再次按住组件或脚下光圈可移动 · Q/E 旋转")
                  .arg(row + 1)
                  .arg(gateTypeName(gate->type))
            : QString("先单击组件选中，再按住组件或光圈拖动 · 右键旋转视角 · 中键平移 · 滚轮缩放"));
}
void StudioWindow::editColor(const QColor &color) {
    if (updating_ || selected_.isEmpty() || !color.isValid())
        return;
    const QString id = selected_;
    edit("调整组件颜色", [id, color](TrackDocument &d) {
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
    edit("调整门架属性", [=](TrackDocument &d) {
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
    Gate g = makeGate(type, w, h);
    g.position = p;
    selected_ = g.id;
    edit("放置门架", [g](TrackDocument &d) { d.gates.append(g); });
}
void StudioWindow::duplicateGate() {
    for (const auto &gate : doc_.gates)
        if (gate.id == selected_) {
            auto g = gate;
            g.id = newGateId();
            g.position += QPointF(2, 2);
            selected_ = g.id;
            edit("复制门架", [g](TrackDocument &d) { d.gates.append(g); });
            return;
        }
}
void StudioWindow::deleteGate() {
    const QString id = selected_;
    if (id.isEmpty())
        return;
    selected_.clear();
    edit("删除门架", [id](TrackDocument &d) {
        for (int i = 0; i < d.gates.size(); ++i)
            if (d.gates[i].id == id) {
                d.gates.removeAt(i);
                return;
            }
    });
}
void StudioWindow::rotateGate(double delta) {
    const QString id = selected_;
    edit("旋转门架", [id, delta](TrackDocument &d) {
        for (auto &g : d.gates)
            if (g.id == id)
                g.yaw = std::remainder(g.yaw + delta, 360.);
    });
}
void StudioWindow::moveOrder(int delta) {
    const QString id = selected_;
    edit("调整通过顺序", [id, delta](TrackDocument &d) {
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
    QMessageBox::warning(this, "OpenTrack Studio", message);
}
bool StudioWindow::maybeSave() {
    if (undo_.isClean())
        return true;
    const auto answer = QMessageBox::question(
        this, "保存工程", "当前工程有未保存的修改，是否保存？",
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
    applyDocument(demo ? demoDocument() : TrackDocument());
    undo_.setClean();
    fitViews();
    notice_->setText(demo ? "示例赛道已加载 · 先点击选中门架，再按住拖动。"
                          : "新工程 · 导入赛道图开始识别，或从右侧组件库放置门架。");
}
void StudioWindow::openProject() {
    if (!maybeSave())
        return;
    const QString path = QFileDialog::getOpenFileName(this, "打开工程或 VelociDrone 赛道", {},
                                                      "工程 / 赛道 (*.otproj *.trk);;所有文件 (*)");
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
    notice_->setText(trk ? "VelociDrone 赛道已导入 · 可编辑后导出。"
                         : "工程已打开 · 底图、门架和通过顺序已恢复。");
    return true;
}
bool StudioWindow::save(bool as) {
    QString p = path_;
    if (as || p.isEmpty())
        p = QFileDialog::getSaveFileName(this, "保存工程", doc_.name + ".otproj",
                                         "OpenTrack 工程 (*.otproj)");
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
    statusBar()->showMessage("工程已保存：" + QFileInfo(p).fileName(), 5000);
    return true;
}
void StudioWindow::exportTrack() {
    if (doc_.gates.isEmpty()) {
        fail("请先生成或放置门架。");
        return;
    }
    QString p = QFileDialog::getSaveFileName(
        this, "导出 VelociDrone 赛道",
        QDir(QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation))
            .filePath(doc_.name + ".trk"),
        "VelociDrone 赛道 (*.trk)");
    if (p.isEmpty())
        return;
    if (!p.endsWith(".trk", Qt::CaseInsensitive))
        p += ".trk";
    QString error;
    if (!exportVelocidrone(p, doc_, &error)) {
        fail(error);
        return;
    }
    notice_->setText("已导出 .trk · 在 VelociDrone 的 Track Editor → Import Track "
                     "中导入。请在模拟器内检查高度、朝向和通过顺序。");
    statusBar()->showMessage("赛道已导出：" + p, 8000);
}
void StudioWindow::exportPng() {
    QString p =
        QFileDialog::getSaveFileName(this, "导出赛道预览", doc_.name + ".png", "PNG 图片 (*.png)");
    if (p.isEmpty())
        return;
    if (!p.endsWith(".png", Qt::CaseInsensitive))
        p += ".png";
    if (!(scene3d_ && views_->currentWidget() == scene3d_ ? scene3d_->exportImage()
                                                          : canvas_->exportImage())
             .save(p))
        fail("图片保存失败。");
    else
        statusBar()->showMessage("赛道预览已保存", 5000);
}
void StudioWindow::importPhotosWithAi() {
    AiVisionDialog dialog(this);
    if (dialog.exec() != QDialog::Accepted || !dialog.result().ok)
        return;
    const auto &result = dialog.result();
    selected_.clear();
    pushEdit("AI 照片生成赛道", doc_, result.document);
    fitViews();
    notice_->setText(
        QString("AI 草稿 · %1 个门架 · 请校对真实距离、高度、朝向和通过顺序，修改可撤销。")
            .arg(doc_.gates.size()));
    notice_->setToolTip("<pre>" + result.warnings.join("\n").toHtmlEscaped() + "</pre>");
}
void StudioWindow::importImage() {
    const QString p = QFileDialog::getOpenFileName(this, "导入赛道俯视图", {},
                                                   "赛道图片 (*.png *.jpg *.jpeg *.bmp)");
    if (p.isEmpty())
        return;
    QFileInfo info(p);
    if (info.size() > 80LL * 1024 * 1024) {
        fail("图片超过 80 MB，请压缩或裁剪后导入。");
        return;
    }
    const quint64 imageRevision = revision_;
    auto *watcher = new QFutureWatcher<QImage>(this);
    statusBar()->showMessage("正在读取图片…");
    connect(watcher, &QFutureWatcher<QImage>::finished, this, [this, watcher, p, imageRevision] {
        const auto image = watcher->result();
        watcher->deleteLater();
        if (imageRevision != revision_) {
            statusBar()->showMessage("工程已改变，请重新导入图片", 4000);
            return;
        }
        if (image.isNull()) {
            fail("无法读取图片，或图片尺寸超过限制。");
            return;
        }
        edit("导入赛道图片", [image](TrackDocument &d) {
            d.background = image;
            d.backgroundOrigin = QPointF(-image.width() * d.metersPerPixel / 2,
                                         -image.height() * d.metersPerPixel / 2);
            d.showBackground = true;
        });
        fitViews();
        notice_->setText("已导入 " + QFileInfo(p).fileName() +
                         " · 点击左侧“识别图片并生成赛道”，识别结果可继续校对。");
        statusBar()->showMessage("图片已载入", 4000);
        startRecognition(false);
    });
    watcher->setFuture(QtConcurrent::run([p] {
        QImageReader reader(p);
        reader.setAutoTransform(true);
        const QSize size = reader.size();
        if (!size.isValid() || qint64(size.width()) * size.height() > 80LL * 1000 * 1000)
            return QImage();
        if (size.width() > 3200 || size.height() > 3200)
            reader.setScaledSize(size.scaled(3200, 3200, Qt::KeepAspectRatio));
        return reader.read();
    }));
}
void StudioWindow::startRecognition(bool confirmReplace) {
    if (recognizing_ || doc_.background.isNull())
        return;
    if (confirmReplace && !doc_.gates.isEmpty()) {
        if (QMessageBox::question(this, "生成赛道",
                                  "识别结果将替换当前门架，可通过撤销恢复。继续生成？",
                                  QMessageBox::Yes | QMessageBox::No) != QMessageBox::Yes)
            return;
    }
    recognizing_ = true;
    recognizeButton_->setEnabled(false);
    recognizeButton_->setText("正在识别…");
    const quint64 revision = revision_;
    const QImage input = doc_.background;
    RecognitionOptions options;
    options.autoScale = autoScale_->isChecked();
    options.metersPerPixel = scale_->value();
    auto *watcher = new QFutureWatcher<RecognitionResult>(this);
    statusBar()->showMessage("正在本地识别门架和布局…");
    connect(watcher, &QFutureWatcher<RecognitionResult>::finished, this, [this, watcher, revision] {
        const auto result = watcher->result();
        watcher->deleteLater();
        recognizing_ = false;
        recognizeButton_->setEnabled(!doc_.background.isNull());
        recognizeButton_->setText("识别图片并生成赛道");
        if (revision != revision_) {
            notice_->setText("识别期间工程已改变，本次结果已丢弃。请在当前工程重新识别。");
            return;
        }
        if (result.detectedCount == 0) {
            notice_->setText(
                "未找到可识别的蓝色门架。请使用清晰俯视图、裁掉界面边框，或手动放置门架。");
            return;
        }
        auto after = result.document;
        after.name = doc_.name;
        after.scenery = doc_.scenery;
        pushEdit("图片生成赛道", doc_, after);
        fitViews();
        notice_->setText(QString("已识别 %1 个门架 · 请在 3D 场景校对高度、朝向与通过顺序。")
                             .arg(result.detectedCount));
        notice_->setToolTip("<pre>" + result.warnings.join("\n").toHtmlEscaped() + "</pre>");
        statusBar()->showMessage("识别完成 · 可撤销或继续调整", 6000);
    });
    watcher->setFuture(
        QtConcurrent::run([input, options] { return recognizeTrack(input, options); }));
}
void StudioWindow::closeEvent(QCloseEvent *e) {
    if (maybeSave())
        e->accept();
    else
        e->ignore();
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
bool StudioWindow::runSmokeTest(const QString &dir) {
    QDir().mkpath(dir);
    QString error;
    const QImage sample = makeExampleImage();
    if (!sample.save(dir + "/example-track.png"))
        return false;
    const auto result = recognizeTrack(sample);
    if (result.detectedCount != 13) {
        qWarning() << "Recognition count" << result.detectedCount << result.warnings;
        return false;
    }
    if (result.document.gates.isEmpty() ||
        !gateFrameMetrics(result.document.gates[0]).centerlineModel)
        return false;
    TrackDocument before = doc_;
    applyDocument(result.document);
    auto beforeEdit = doc_;
    selected_ = doc_.gates[0].id;
    canvas_->selectGate(selected_);
    duplicateGate();
    if (doc_.gates.size() != 14)
        return false;
    undo_.undo();
    if (doc_.gates.size() != 13)
        return false;
    undo_.redo();
    if (doc_.gates.size() != 14)
        return false;
    undo_.undo();
    const QString project = dir + "/example.otproj", trk = dir + "/example.trk";
    // Exercise the actual property control, document undo, and persisted color together.
    selected_ = doc_.gates[0].id;
    canvas_->selectGate(selected_);
    const QColor originalColor = gateColor(doc_.gates[0]);
    const int redIndex = color_->findData(QString("#e60000"));
    if (redIndex < 0)
        return false;
    color_->setCurrentIndex(redIndex);
    if (gateColor(doc_.gates[0]) != QColor("#e60000"))
        return false;
    undo_.undo();
    if (gateColor(doc_.gates[0]) != originalColor)
        return false;
    undo_.redo();
    if (gateColor(doc_.gates[0]) != QColor("#e60000"))
        return false;
    if (!saveProject(project, doc_, &error)) {
        qWarning() << error;
        return false;
    }
    TrackDocument restored;
    if (!loadProject(project, &restored, &error) || toJson(restored) != toJson(doc_)) {
        qWarning() << error;
        return false;
    }
    if (!exportVelocidrone(trk, doc_, &error)) {
        qWarning() << error;
        return false;
    }
    TrackDocument imported;
    if (!importVelocidrone(trk, &imported, &error) || imported.gates.size() != doc_.gates.size()) {
        qWarning() << error;
        return false;
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
        return false;
    QJsonObject report{{"recognizedGates", result.detectedCount},
                       {"projectRoundtrip", true},
                       {"trackCodecRoundtrip", true},
                       {"undoRedo", true},
                       {"colorPropertyUndoRedo", true},
                       {"zoom200OperationsMs", zoomMs},
                       {"simulatorImportTested", false}};
    QFile file(dir + "/smoke-report.json");
    if (!file.open(QIODevice::WriteOnly))
        return false;
    file.write(QJsonDocument(report).toJson());
    undo_.clear();
    undo_.setClean();
    return true;
}
} // namespace ot
